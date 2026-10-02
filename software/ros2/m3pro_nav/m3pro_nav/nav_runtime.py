#!/usr/bin/env python3
"""NavRuntime —— 真实闭环协调器 (纯 Python, 无 ROS; 节点只做消息接线).

数据流 (GPT 定稿: 事件驱动, 绝不 while 串行):
    /scan 到达     → on_scan: 投影→关联→RealObservationAdapter→nav.observe
    /odom 到达     → on_odom_pose: GridEventDetector→on_crossed/on_entered
                     (事件只 commit/verify; 一致性校验记 mismatch)
    planner tick   → planner_tick: 编链/延长 horizon/返航 (cursor 驱动)
    control tick   → control_tick: MotionRuntimeCore.update → Twist2D

坐标约定: 规划/地图/模板全部在 maze 帧 (与仿真同构); odom 帧只存在于
MotionRuntimeCore 的 follower 锚定 (planner(maze)→odom 刚性变换)。
核心算法 (StreamNav/ActionHorizon/MotionPlanner/MotionRuntimeCore) 一行不改。
"""

import math
import time as _time
from collections import deque
from copy import deepcopy

from .action_horizon import ActionHorizon
from .event_detector import GridEventDetector
from .frame_projector import FrameProjector, LaserExtrinsic, ManualMazeAnchor
from .frame_transform import RigidFrameTransform
from .grid_association import GridAssociation
from .motion_planner import MotionPlanner, _primitive_end
from .motion_primitive import MotionPrimitive
from .motion_runtime import (FeedbackSample, MotionRuntimeCore, RuntimeState,
                             SafetyState)
from .observation_adapter import FrameObservationStats, RealObservationAdapter
from .pose import C, Pose2D, TH
from .pose_correction import (KnownWallSegment, ProjectedEndpoint,
                              propose_pose_correction)
from .position_controller import PositionController
from .scan_adapter import parse_scan
from .scan_time_alignment import OdomPoseHistory, ScanTimeAligner
from .stream_nav import StreamNav
from .trust_policy import TrustPolicy, TrustThresholds


class _GyroYawFusion:
    """Short-window yaw delta fusion for scan pose prediction.

    Translation and the motion follower continue to use wheel odometry. Gyro
    samples only influence the yaw stored in scan-alignment history. If gyro
    coverage is stale or has a gap, that interval falls back to wheel yaw.
    """

    def __init__(self, *, enabled=False, imu_weight=0.5, max_age_s=0.12,
                 max_gap_s=0.12, bias_radps=0.0, sign=1.0):
        for name, value in (('imu_weight', imu_weight),
                            ('max_age_s', max_age_s),
                            ('max_gap_s', max_gap_s),
                            ('bias_radps', bias_radps), ('sign', sign)):
            if not math.isfinite(float(value)):
                raise ValueError(f'{name} must be finite')
        if not 0.0 <= imu_weight <= 1.0 or max_age_s <= 0 or max_gap_s <= 0:
            raise ValueError('invalid gyro fusion bounds')
        self.enabled = bool(enabled)
        self.weight = float(imu_weight)
        self.max_age_s = float(max_age_s)
        self.max_gap_s = float(max_gap_s)
        self.bias = float(bias_radps)
        self.sign = float(sign)
        self.samples = deque(maxlen=512)
        self.last_odom = None
        self.fused_yaw = None

    def add_gyro(self, stamp, wz):
        stamp, wz = float(stamp), float(wz)
        if not math.isfinite(stamp) or not math.isfinite(wz):
            return False
        if self.samples and stamp <= self.samples[-1][0]:
            return False
        self.samples.append((stamp, self.sign * (wz - self.bias)))
        return True

    def _integral(self, start, end):
        samples = tuple((t, w) for t, w in self.samples
                        if start - self.max_age_s <= t <= end + self.max_age_s)
        if not samples:
            return None
        # ROS odom and IMU callbacks arrive in source-time order only
        # approximately. Requiring a *future* IMU sample to bracket the odom
        # timestamp made every live interval fall back to wheel yaw. Hold the
        # latest rate for a bounded fraction of a sample period instead.
        def rate_at(target):
            before = next((sample for sample in reversed(samples)
                           if sample[0] <= target), None)
            if before is None or target - before[0] > self.max_age_s:
                return None
            after = next((sample for sample in samples
                          if sample[0] > target), None)
            if after is None:
                return before[1]
            gap = after[0] - before[0]
            if gap > self.max_gap_s:
                return None
            alpha = (target - before[0]) / gap
            return before[1] + alpha * (after[1] - before[1])

        left_rate, right_rate = rate_at(start), rate_at(end)
        if left_rate is None or right_rate is None:
            return None
        inside = [(t, w) for t, w in samples if start < t < end]
        points = [(start, left_rate), *inside, (end, right_rate)]
        if any(b[0] - a[0] > self.max_gap_s
               for a, b in zip(points, points[1:])):
            return None
        return sum((b[0] - a[0]) * (a[1] + b[1]) * 0.5
                   for a, b in zip(points, points[1:]))

    def fuse(self, stamp, wheel_pose):
        stamp = float(stamp)
        if self.last_odom is None:
            self.last_odom = (stamp, wheel_pose.copy())
            self.fused_yaw = wheel_pose.yaw
            return wheel_pose.copy(), 'WHEEL_INIT'
        prev_stamp, prev_pose = self.last_odom
        if stamp <= prev_stamp:
            return wheel_pose.copy(), 'WHEEL_REJECTED'
        wheel_delta = math.atan2(math.sin(wheel_pose.yaw - prev_pose.yaw),
                                 math.cos(wheel_pose.yaw - prev_pose.yaw))
        gyro_delta = self._integral(prev_stamp, stamp) if self.enabled else None
        if gyro_delta is None or stamp - self.samples[-1][0] > self.max_age_s:
            delta, mode = wheel_delta, 'WHEEL_FALLBACK'
        else:
            delta = ((1.0 - self.weight) * wheel_delta +
                     self.weight * gyro_delta)
            mode = 'WHEEL_GYRO'
        self.fused_yaw = math.atan2(math.sin(self.fused_yaw + delta),
                                    math.cos(self.fused_yaw + delta))
        self.last_odom = (stamp, wheel_pose.copy())
        return Pose2D(wheel_pose.x, wheel_pose.y, self.fused_yaw), mode


def _calibrated(value, cast=float):
    if value is None or (isinstance(value, str) and
                         any(token in value for token in
                             ('__CALIBRATE__', '__MEASURE__'))):
        return None
    return cast(value)


def _ensure_terminal_stop(prims, cursor_cell):
    """core follower 契约: 链必须以 STOP 收尾 (max_steps 耗尽的 compile 尾
    没有 STOP —— 仿真 executor 容忍, core 不容忍). 就地补: 末段刹零 +
    WAIT STOP (到达即 HOLDING, 等下一轮 compile 续)."""
    if not prims or prims[-1].kind == 'STOP':
        return prims
    last = prims[-1]
    if last.kind in ('STRAIGHT', 'ARC'):
        last.v_end = 0.0
    end = _primitive_end(last)
    prims.append(MotionPrimitive(
        'STOP', Pose2D(end[0], end[1], 0.0), p0=end, yaw0=0.0,
        duration=0.2, meta={'wait': cursor_cell}))
    return prims


class NavRuntimePhase:
    WAITING_SENSORS = 'WAITING_SENSORS'    # 等首个 odom 建 anchor
    EXPLORING = 'EXPLORING'                # 正常探索 (含边界 WAIT)
    GOING_HOME = 'GOING_HOME'              # 已装返航计划
    DONE = 'DONE'                          # 返航完成 (出场)


class NavRuntime:
    def __init__(self, *, entry, order='LFR', n=7, cell, heading,
                 dry_run=True, perception=None, controller=None,
                 a_acc=1.0, a_dec=1.0, extrinsic=None,
                 max_feedback_age_s=0.5, offset=(0.0, 0.0, 0.0),
                 required_blocks=None, task_mode=False):
        """cell/heading: 人工摆车位置 (maze 锚定); perception: 感知参数 dict
        (dphi_deg/gate/rng/confirm_near —— 现场待审, 传 sim 默认即用默认)."""
        perc = {'dphi_deg': 0.30, 'gate': 0.06, 'rng': 0.02,
                'confirm_near': 0.6}
        perc.update(perception or {})
        self.nav = StreamNav(entry, order=order, n=n, task_mode=task_mode,
                             dphi_deg=perc['dphi_deg'], gate=perc['gate'],
                             rng=perc['rng'],
                             confirm_near=perc['confirm_near'])
        self.planner = MotionPlanner()
        self.horizon = ActionHorizon(self.planner)
        self.core = MotionRuntimeCore(
            a_acc=a_acc, a_dec=a_dec,
            controller=controller if controller is not None
            else PositionController(),
            max_feedback_age_s=max_feedback_age_s)
        self.detector = GridEventDetector(n)
        assoc = perc.get('association', {})
        self.association = GridAssociation(
            max_residual=float(assoc.get('max_residual_m', 0.05)),
            ambiguity_margin=float(assoc.get('ambiguity_margin_m', 0.02)),
            corner_guard=float(assoc.get('corner_guard_m', 0.05)))
        trust = perc.get('trust', {})
        thresholds = TrustThresholds(
            max_residual_m=_calibrated(trust.get('max_residual_m')),
            max_incidence_rad=_calibrated(trust.get('max_incidence_rad')),
            max_range_m=_calibrated(trust.get('max_range_m')),
            min_uniqueness_margin_m=_calibrated(
                trust.get('min_uniqueness_margin_m')),
            min_corner_distance_m=_calibrated(
                trust.get('min_corner_distance_m')),
            min_votes=_calibrated(trust.get('min_votes'), int))
        self.trust_policy = TrustPolicy(
            diagnostic_only=bool(trust.get('diagnostic_only', True)),
            thresholds=thresholds)
        self.beam_origin_preserving = bool(
            perc.get('beam_origin_preserving', False))
        self.adapter = RealObservationAdapter(
            self.association, trust_policy=self.trust_policy,
            allow_open_evidence=self.beam_origin_preserving)
        self.projector = FrameProjector(
            extrinsic if extrinsic is not None else LaserExtrinsic.missing())
        sync = perc.get('sync', {})
        self.odom_history = OdomPoseHistory(
            history_s=float(sync.get('history_s', 1.0)))
        self.scan_aligner = ScanTimeAligner(
            self.odom_history,
            max_bracket_gap_s=float(sync.get('max_bracket_gap_s', 0.15)),
            max_extrapolation_s=float(sync.get('max_extrapolation_s', 0.03)),
            max_trust_motion_mps=float(sync.get('max_trust_motion_mps', 0.03)),
            max_trust_yaw_rate_radps=float(
                sync.get('max_trust_yaw_rate_radps', 0.05)),
            static_window_fallback_s=float(
                sync.get('static_window_fallback_s', 0.25)))
        self._pending_scans = deque()
        self._pending_shadow_scans = deque()
        yaw_cfg = perc.get('gyro_yaw', {})
        self.gyro_yaw = _GyroYawFusion(
            enabled=bool(yaw_cfg.get('enabled', False)),
            imu_weight=float(yaw_cfg.get('imu_weight', 0.5)),
            max_age_s=float(yaw_cfg.get('max_age_s', 0.12)),
            max_gap_s=float(yaw_cfg.get('max_gap_s', 0.12)),
            bias_radps=float(yaw_cfg.get('bias_radps', 0.0)),
            sign=float(yaw_cfg.get('sign', 1.0)))
        self._frame_grid_shadow = None
        shadow_cfg = perc.get('frame_grid_shadow', {})
        self._shadow_max_lag_s = float(shadow_cfg.get('max_scan_lag_s', 0.8))
        if not math.isfinite(self._shadow_max_lag_s) or self._shadow_max_lag_s <= 0:
            raise ValueError('frame_grid_shadow.max_scan_lag_s must be positive')
        if bool(shadow_cfg.get('enabled', False)) and dry_run:
            from .frame_grid_shadow import FrameGridShadow
            self._frame_grid_shadow = FrameGridShadow()
        self.correction_enabled = bool(
            perc.get('correction', {}).get('enabled', False))
        self._last_correction_stamp = None
        self.entry = entry
        self.dry_run = dry_run
        self.required_blocks = required_blocks
        self.task_mode = task_mode

        # 状态 (全部显式, 无隐藏耦合)
        self.anchor = None                  # 首个 odom 建立后冻结
        self.phase = NavRuntimePhase.WAITING_SENSORS
        self.cursor = (None, entry)         # 拓扑游标 (规划真相)
        self.plan_state = {}                # ActionHorizon overlay
        self.planned_cells = [entry]        # 执行一致性校验序列
        self.mismatches = []                # [(t, entered, planned_head)]
        self.prev_maze_pose = None
        self.latest_odom = None             # (stamp, Pose2D odom 帧)
        self.events = []                    # 结构化事件日志 (evidence)
        self.frames = []                    # 每帧观测统计 (evidence)
        self.exited = False
        self._prune_notice = None
        self._queued_plan_base = None
        self._last_yaw_log_stamp = None
        self._last_yaw_mode = None

    # ---- 传感器输入 (事件源) ----

    def on_odom(self, odom_state, receive_stamp=None):
        """OdometryState (odom 帧 实测位姿+速度) → 建 anchor / 拓扑事件 /
        反馈缓存. receive_stamp: 节点收到消息的时刻 (反馈钟, None=odom stamp)."""
        odom_pose = odom_state.pose
        stamp = receive_stamp if receive_stamp is not None else odom_state.stamp
        scan_pose, yaw_mode = self.gyro_yaw.fuse(odom_state.stamp, odom_pose)
        try:
            self.odom_history.add(odom_state.stamp, scan_pose, stamp)
        except ValueError as exc:
            # A repeated/regressed source stamp cannot calibrate a scan. The
            # feedback path still sees this callback and has its own watchdog.
            self._log_event('odom_history_rejected', stamp, error=str(exc))
        if self.anchor is None:
            self.anchor = ManualMazeAnchor(
                self._anchor_cell, self._anchor_heading, odom_pose,
                offset=self._anchor_offset)
            maze0 = self.anchor.maze_pose(odom_pose)
            self.nav.on_entered(self.entry, None, ts=stamp)
            self._log_event('anchor', stamp, maze=[round(maze0.x, 4),
                                                   round(maze0.y, 4),
                                                   round(maze0.yaw, 4)])
            self.prev_maze_pose = maze0
            self.phase = NavRuntimePhase.EXPLORING
        if (yaw_mode != self._last_yaw_mode or
                self._last_yaw_log_stamp is None or
                odom_state.stamp - self._last_yaw_log_stamp >= 1.0):
            self._log_event('yaw_prediction', odom_state.stamp,
                            mode=yaw_mode,
                            fused_yaw=round(scan_pose.yaw, 6),
                            wheel_yaw=round(odom_pose.yaw, 6))
            self._last_yaw_log_stamp = odom_state.stamp
            self._last_yaw_mode = yaw_mode
        self.latest_odom = (stamp, odom_pose,
                            odom_state.vx_world, odom_state.vy_world,
                            odom_state.wz)
        maze_pose = self.anchor.maze_pose(odom_pose)
        # 几何事件桥: 连续位姿 → CrossedEdge/EnteredCell (只 commit/verify)
        for ev in self.detector.detect(self.prev_maze_pose, maze_pose,
                                       timestamp=stamp):
            if 0 <= ev.to_cell[0] < self.nav.n and \
                    0 <= ev.to_cell[1] < self.nav.n:
                self.nav.on_crossed(ev.from_cell, ev.direction, ts=stamp)
                self.nav.on_entered(ev.to_cell, ev.from_cell, ts=stamp)
                self._verify_plan(ev.to_cell, stamp)
                self._log_event('crossed', stamp,
                                **{'from': list(ev.from_cell),
                                   'to': list(ev.to_cell)})
        self.prev_maze_pose = maze_pose
        # The real /scan_multi publisher leaves both timing fields at zero.
        # Its scans can only be trusted after future odom confirms a stationary
        # window around the source stamp, so process queued scans with delay.
        processed = []
        guard = self.scan_aligner.static_window_fallback_s
        while (self._pending_scans and
               self._pending_scans[0].stamp + guard <= odom_state.stamp):
            processed.append(self._process_scan_frame(
                self._pending_scans.popleft()))
        return [stats for stats in processed if stats is not None]

    def on_imu(self, stamp, angular_velocity_z):
        """Accept source-stamped planar gyro rate; invalid samples are dropped."""
        accepted = self.gyro_yaw.add_gyro(stamp, angular_velocity_z)
        if not accepted:
            self._log_event('imu_abstain', stamp,
                            reason='INVALID_OR_NONMONOTONIC_IMU')
        return accepted

    def _verify_plan(self, entered_cell, stamp):
        """实际进入格必须符合计划序列 (mismatch = 定位/执行/事件链故障)."""
        if len(self.planned_cells) >= 2 and entered_cell == self.planned_cells[1]:
            self.planned_cells.pop(0)
        elif entered_cell != self.planned_cells[0]:
            self.mismatches.append((stamp, entered_cell,
                                    list(self.planned_cells[:3])))
            self._log_event('mismatch', stamp,
                            entered=list(entered_cell),
                            planned=[list(c) for c in self.planned_cells[:3]])

    def on_scan(self, scan_msg, *, cell_hint=None):
        """Time-aligned diagnostic rays → trust gate → bounded edge evidence."""
        if self.anchor is None or self.latest_odom is None:
            return None                     # 传感器未就绪: 丢弃
        frame = parse_scan(scan_msg)
        if self._frame_grid_shadow is not None:
            if len(self._pending_shadow_scans) >= 32:
                dropped = self._pending_shadow_scans.popleft()
                self._log_event('frame_grid_shadow_drop', dropped.stamp,
                                reason='PENDING_SHADOW_OVERFLOW')
            self._pending_shadow_scans.append(frame)
        if len(frame.rays) > 1 and frame.time_increment == 0.0:
            if len(self._pending_scans) >= 32:
                dropped = self._pending_scans.popleft()
                self._log_event('scan_abstain', dropped.stamp,
                                reason='PENDING_SCAN_OVERFLOW')
            self._pending_scans.append(frame)
            return None
        return self._process_scan_frame(frame)

    def process_shadow_pending(self, max_frames=1):
        """Run a small shadow-only budget outside the 50 Hz control callback."""
        if self._frame_grid_shadow is None or not self._pending_shadow_scans:
            return 0
        processed = 0
        history = self.odom_history.samples
        while self._pending_shadow_scans and processed < max_frames:
            frame = self._pending_shadow_scans[0]
            midpoint = frame.stamp + max(0, len(frame.rays) - 1) * \
                frame.time_increment * 0.5
            if not history or midpoint > history[-1].stamp:
                break
            lag_s = history[-1].stamp - midpoint
            if lag_s > self._shadow_max_lag_s:
                self._pending_shadow_scans.popleft()
                self._log_event('frame_grid_shadow_drop', frame.stamp,
                                reason='SHADOW_BACKLOG', lag_s=lag_s)
                continue
            sample = self.odom_history.interpolate(
                midpoint,
                max_bracket_gap_s=self.scan_aligner.max_bracket_gap_s,
                max_extrapolation_s=self.scan_aligner.max_extrapolation_s)
            if sample is None:
                if midpoint < history[0].stamp:
                    self._pending_shadow_scans.popleft()
                    self._log_event('frame_grid_shadow_drop', frame.stamp,
                                    reason='ODOM_HISTORY_EXPIRED')
                    continue
                break
            self._pending_shadow_scans.popleft()
            pose, _skew = sample
            try:
                result = self._frame_grid_shadow.process(
                    frame, pose, self.projector.extrinsic, self.anchor)
                self._log_event('frame_grid_shadow', frame.stamp,
                                **self._shadow_json(result))
            except Exception as exc:  # diagnostic failure cannot alter runtime
                self._log_event('frame_grid_shadow_error', frame.stamp,
                                error=f'{type(exc).__name__}: {exc}')
            processed += 1
        return processed

    def _process_scan_frame(self, frame):
        if not self.projector.extrinsic.available:
            self._log_event('no_transform', frame.stamp)
            return None
        alignment = self.scan_aligner.align(frame)
        if not alignment.accepted:
            self._log_event('scan_abstain', frame.stamp,
                            reason=alignment.reason,
                            scan_end=alignment.scan_end,
                            odom_skew_s=alignment.max_nearest_odom_skew)
            stats = FrameObservationStats(
                n_rays=len(frame.rays), n_rejected=len(frame.rays),
                reject_reasons={alignment.reason: len(frame.rays)})
            stats.alignment_mode = alignment.mode
            stats.odom_skew_s = alignment.max_nearest_odom_skew
            stats.stamp = frame.stamp
            self.frames.append(stats)
            return stats
        odom_pose = alignment.pose
        maze_pose = self.anchor.maze_pose(odom_pose)
        world_rays = self.projector.project(frame, odom_pose, self.anchor)
        if world_rays is None:
            self._log_event('no_transform', frame.stamp)
            return None
        if self.correction_enabled:
            if self._maybe_correct_pose(frame.stamp, odom_pose, maze_pose,
                                        world_rays):
                maze_pose = self.anchor.maze_pose(odom_pose)
                world_rays = self.projector.project(
                    frame, odom_pose, self.anchor)
        hits, opens, stats = self.adapter.to_nav_observation(world_rays,
                                                             maze_pose,
                                                             stamp=frame.stamp)
        stats.alignment_mode = alignment.mode
        stats.odom_skew_s = alignment.max_nearest_odom_skew
        self.nav.observe(hits, opens, stamp=frame.stamp)
        for cell in list(self.nav.visits.cells):
            self.nav.refresh_branch(cell)
        stats.stamp = frame.stamp
        self.frames.append(stats)
        return stats

    @staticmethod
    def _shadow_json(result):
        if not isinstance(result, dict):
            return {'result': str(result)}
        out = {}
        for key, value in result.items():
            if isinstance(value, Pose2D):
                out[key] = [value.x, value.y, value.yaw]
            elif value is None or isinstance(value, (str, int, float, bool)):
                out[key] = value
            elif isinstance(value, (list, tuple)):
                out[key] = [NavRuntime._shadow_json_value(item)
                            for item in value]
            elif isinstance(value, dict):
                out[key] = {str(k): NavRuntime._shadow_json_value(v)
                            for k, v in value.items()}
            else:
                out[key] = str(value)
        return out

    @staticmethod
    def _shadow_json_value(value):
        if isinstance(value, Pose2D):
            return [value.x, value.y, value.yaw]
        if value is None or isinstance(value, (str, int, float, bool)):
            return value
        if isinstance(value, (list, tuple)):
            return [NavRuntime._shadow_json_value(item) for item in value]
        if isinstance(value, dict):
            return {str(k): NavRuntime._shadow_json_value(v)
                    for k, v in value.items()}
        return str(value)

    def _known_wall_segments(self):
        """Use pre-existing sensor-confirmed walls, never current-scan votes."""
        segments = []
        for key, belief in self.nav.edges.soft.items():
            if belief.get('state') != 'WALL':
                continue
            if key[0] == 'B':
                _, (i, j), direction = key
            else:
                (i, j), direction = key
            if direction in ('E', 'W'):
                x = (i + (direction == 'E')) * C
                start, end = (x, j * C), (x, (j + 1) * C)
            else:
                y = (j + (direction == 'N')) * C
                start, end = (i * C, y), ((i + 1) * C, y)
            segments.append(KnownWallSegment(
                start, end, confirmed=True, wall_id=str(key)))
        return segments

    def _maybe_correct_pose(self, stamp, odom_pose, maze_pose, world_rays):
        """At a measured stop, re-anchor maze<-odom and replan from zero."""
        if self.core.state not in (RuntimeState.IDLE, RuntimeState.HOLDING,
                                   RuntimeState.FINISHED):
            return False
        if (self._last_correction_stamp is not None and
                stamp - self._last_correction_stamp < 0.5):
            return False
        _, _, vx, vy, wz = self.latest_odom
        if math.hypot(vx, vy) > 0.025 or abs(wz) > 0.05:
            return False
        walls = self._known_wall_segments()
        points = [ProjectedEndpoint(ray.hx, ray.hy)
                  for ray in world_rays if ray.valid]
        result = propose_pose_correction(points, walls, maze_pose)
        self._log_event('pose_correction_proposal', stamp,
                        accepted=result.accepted, reason=result.reason,
                        dx=result.dx, dy=result.dy, dyaw=result.dyaw,
                        walls=result.n_walls, inliers=result.n_inliers)
        if not result.accepted:
            return False
        if math.hypot(result.dx, result.dy) < 0.01 and abs(result.dyaw) < 0.01:
            return False
        corrected_cell = (int(math.floor(result.corrected_pose.x / C)),
                          int(math.floor(result.corrected_pose.y / C)))
        if corrected_cell != self.cursor[1]:
            self._log_event('pose_correction_rejected', stamp,
                            reason='WOULD_CHANGE_TOPOLOGICAL_CELL',
                            corrected_cell=list(corrected_cell),
                            cursor_cell=list(self.cursor[1]))
            return False
        # Never change the wheel odometry. The overlay is changed only while
        # stopped; an old follower retains its frame, so discard that plan
        # before allowing the next planner tick to compile under the new one.
        self.core.reset()
        self.anchor.transform = RigidFrameTransform(
            odom_pose, result.corrected_pose)
        self.anchor.maze_anchor = self.anchor.maze_pose(self.anchor.odom_anchor)
        self.prev_maze_pose = self.anchor.maze_pose(self.latest_odom[1])
        self.planned_cells = [self.cursor[1]]
        self._last_correction_stamp = stamp
        self._log_event('pose_correction_applied', stamp,
                        maze=[self.prev_maze_pose.x,
                              self.prev_maze_pose.y,
                              self.prev_maze_pose.yaw])
        return True

    # ---- 规划 tick (10Hz 级) ----

    def planner_tick(self, tick_time):
        """编链 / 延长 horizon / 返航. 只在传感器就绪后生效."""
        if self.phase == NavRuntimePhase.WAITING_SENSORS:
            return
        if self.phase == NavRuntimePhase.GOING_HOME:
            if self.core.state == RuntimeState.FINISHED:
                self.phase = NavRuntimePhase.DONE
                self._log_event('done', tick_time)
            return
        if self.phase == NavRuntimePhase.DONE:
            return
        if self.core.safety != SafetyState.ARMED:
            self.core.arm()
        if (self._queued_plan_base is not None and
                not self.core.has_queued_suffix):
            self._queued_plan_base = None
        prune = self._planned_prune_candidate()
        if prune is not None and self.core.cancel_queued_suffix():
            base = self._queued_plan_base
            if base is not None:
                self.cursor = base[0]
                self.plan_state = base[1]
                self.planned_cells = base[2]
            self._queued_plan_base = None
            self._log_event('task_prune_queued_suffix_cancelled', tick_time,
                            parent=list(prune[0]), child=list(prune[1]),
                            active_primitive_preserved=True,
                            replan_anchor=list(self.cursor[1]))
            prune = None
        if prune is not None and self._prune_notice != prune:
            self._prune_notice = prune
            self._log_event('task_prune_suffix_pending', tick_time,
                            parent=list(prune[0]), child=list(prune[1]),
                            active_primitive_preserved=True,
                            replan_anchor=list(self.cursor[1]),
                            action='hold_extension_until_existing_wait')
        elif prune is None:
            self._prune_notice = None
        # 任务完成判定 (与仿真同语义)
        done = (self.nav.got >= self.required_blocks
                or not self.nav.active_frontier()) \
            if self.task_mode else self.nav.all_resolved()
        if done:
            route = self.nav.home_route(self.cursor[1])
            if route is not None:
                # 链到位 (边界等待/空闲) 才装返航; TRACKING 中等下一 tick
                if self.core.state in (RuntimeState.HOLDING,
                                       RuntimeState.IDLE,
                                       RuntimeState.FINISHED):
                    self._load_home(route, tick_time)
                return
            # 出口未连通: 继续探索 (与仿真同语义)
        # TRACKING + 尾部 WAIT STOP → 滚动延长 (safe-extension 在 core 内)
        tail = self.core.tail_wait_stop()
        if (not done and self.core.state == RuntimeState.TRACKING
                and tail is not None):
            if prune is not None:
                # MotionRuntimeCore deliberately has no API to retime a live
                # primitive. Keep the installed chain intact; do not add more
                # stale horizon. The existing WAIT STOP is the next verified
                # legal anchor at which normal compilation re-evaluates pruning.
                return
            (tx, ty), _ = tail
            base = (self.cursor, deepcopy(self.plan_state),
                    list(self.planned_cells))
            suffix, terminal, seq, next_state = self.horizon.compile_with_state(
                self.nav, Pose2D(tx, ty, self.anchor.maze_anchor.yaw),
                self.cursor, self.plan_state)
            _ensure_terminal_stop(suffix, terminal[1])
            if suffix and suffix[0].kind != 'STOP':
                self.core.append_suffix(suffix)
                if self.core.has_queued_suffix and self._queued_plan_base is None:
                    self._queued_plan_base = base
                elif not self.core.has_queued_suffix:
                    self._queued_plan_base = None
                self.cursor = terminal
                self.plan_state = next_state
                self.planned_cells.extend(seq[1:])
                self._log_event('extend', tick_time, cells=list(seq[1:]))
            return
        # HOLDING 在 WAIT 边界 → 新信息成熟则续跑
        if self.core.state == RuntimeState.HOLDING and tail is not None:
            (tx, ty), _ = tail
            prims, terminal, seq, next_state = self.horizon.compile_with_state(
                self.nav, Pose2D(tx, ty, self.anchor.maze_anchor.yaw),
                self.cursor, self.plan_state)
            _ensure_terminal_stop(prims, terminal[1])
            if prims and prims[0].kind != 'STOP':
                self.core.append_suffix(prims)
                self.cursor = terminal
                self.plan_state = next_state
                self.planned_cells = list(seq)
                self._log_event('resume', tick_time, cells=list(seq[1:]))
            if prune is None:
                self._prune_notice = None
            return
        # IDLE/FINISHED → 新链 (仅首次: 探索链都以 WAIT STOP 收尾进 HOLDING)
        if self.core.state in (RuntimeState.IDLE, RuntimeState.FINISHED):
            if not self.core.has_feedback:
                self.control_tick(tick_time)    # 先让 core 记录锚点反馈
                if not self.core.has_feedback:
                    return
            pose = (self.prev_maze_pose if self.prev_maze_pose is not None
                    else self.anchor.maze_anchor)
            prims, terminal, seq, next_state = self.horizon.compile_with_state(
                self.nav, pose, self.cursor, self.plan_state)
            _ensure_terminal_stop(prims, terminal[1])
            try:
                self.core.load_plan(
                    prims, planner_yaw=self.anchor.maze_anchor.yaw)
            except ValueError as exc:
                self._log_event('plan_rejected', tick_time, error=str(exc))
                return
            self.cursor = terminal
            self.plan_state = next_state
            self.planned_cells = list(seq)
            self._log_event('plan', tick_time, first=prims[0].kind,
                            cells=list(seq))

    def _planned_prune_candidate(self):
        """Return the first newly provable pruned directed edge in the suffix.

        Pruning stays dynamically derived from BlockMap and never mutates the
        physical map or the active motion primitive.
        """
        if not self.task_mode or len(self.planned_cells) < 2:
            return None
        cells = self.planned_cells
        # _verify_plan consumes completed cells from the head, while cursor is
        # the far end of the compiled horizon. Scan the whole remaining route.
        for parent, child in zip(cells, cells[1:]):
            if self.nav._is_pruned(parent, child):
                return parent, child
        return None

    def _load_home(self, route, tick_time):
        """route = (path_cells, exit_dir); 从当前边界锚编译返航链 + 终端 STOP."""
        path, exit_dir = route
        pose = (self.prev_maze_pose if self.prev_maze_pose is not None
                else self.anchor.maze_anchor)
        try:
            prims = self.planner.compile_home(pose, self.cursor, path,
                                              exit_dir)
            # follower 契约: 链必须以 STOP 收尾, 其前一段 v_end=0
            if prims[-1].kind != 'STOP':
                last = prims[-1]
                if last.kind == 'STRAIGHT':
                    last.v_end = 0.0
                end = last.p1 if last.p1 else (last.start_pose.x,
                                               last.start_pose.y)
                from .motion_primitive import MotionPrimitive
                prims.append(MotionPrimitive(
                    'STOP', Pose2D(end[0], end[1], 0.0), p0=end,
                    yaw0=0.0, duration=0.2, meta={'home_end': True}))
            if self.core.state in (RuntimeState.IDLE, RuntimeState.FINISHED):
                self.core.load_plan(
                    prims, planner_yaw=self.anchor.maze_anchor.yaw)
            else:
                self.core.append_suffix(prims)
        except ValueError as exc:
            self._log_event('home_rejected', tick_time, error=str(exc))
            return
        self.phase = NavRuntimePhase.GOING_HOME
        self.planned_cells = list(path)    # 返航校验序列 = 归途格路径
        self._log_event('home', tick_time, path=[list(c) for c in path],
                        exit=exit_dir)

    # ---- 控制 tick (20-50Hz) ----

    def control_tick(self, tick_time):
        """core.update → RuntimeOutput. dry_run 下同样计算, 节点不发."""
        feedback = None
        if self.latest_odom is not None:
            stamp, odom_pose, vx, vy, wz = self.latest_odom
            feedback = FeedbackSample(stamp, odom_pose, vx, vy, wz)
        return self.core.update(tick_time, feedback)

    # ---- evidence ----

    def _log_event(self, kind, ts, **kv):
        self.events.append({'t': ts, 'kind': kind, **kv})

    # ---- anchor 参数 (on_odom 首帧前由节点注入) ----

    _anchor_cell = (0, 0)
    _anchor_heading = 'N'
    _anchor_offset = (0.0, 0.0, 0.0)

    def set_anchor_spec(self, cell, heading, offset=(0.0, 0.0, 0.0)):
        self._anchor_cell = tuple(int(v) for v in cell)
        self._anchor_heading = heading
        self._anchor_offset = tuple(float(v) for v in offset)

    def evidence_snapshot(self):
        """一次 run 的全部证据 (喂 LLM 分析用)."""
        return {
            'phase': self.phase,
            'mismatches': len(self.mismatches),
            'events': self.events,
            'frames': [f.to_json() for f in self.frames[-50:]],
            'blocks_got': self.nav.got,
            'pruned_cells': sorted(self.nav.pruned_cells),
            'wrong_edges_note': 'compare edges vs truth offline',
        }


# ---- 配置装载 (纯 Python; 节点复用) ----

_PLACEHOLDERS = ('__CALIBRATE__', '__MEASURE__')


def load_runtime_config(path):
    """读 config yaml; 返回 (config, uncalibrated_keys).

    占位符 (__CALIBRATE__/__MEASURE__) 不解析为数字 —— 未标定项必须
    fail closed (dry_run 允许, 实跑拒绝)。

    条件性占位 (GPT 审查收尾): laser_extrinsic.source == 'tf' 时, YAML
    分支的 x/y/yaw 占位**不会**被使用 (实际外参来自 TF) —— 不计入
    uncalibrated, 否则现场会碰到"TF 正常却拒绝启动"的假 fail。"""
    import yaml

    def walk(node, prefix=()):
        out = {}
        for k, v in (node or {}).items():
            if isinstance(v, dict):
                out.update(walk(v, prefix + (k,)))
            else:
                out['.'.join(prefix + (str(k),))] = v
        return out

    with open(path) as f:
        raw = yaml.safe_load(f)
    flat = walk(raw)
    # 条件性占位: source=tf 时 YAML 外参分支整体豁免
    ext = raw.get('laser_extrinsic') or {}
    exempt = ('laser_extrinsic.',) if str(ext.get('source', 'yaml')) == 'tf' \
        else ()
    uncalibrated = [k for k, v in flat.items()
                    if isinstance(v, str)
                    and any(p in v for p in _PLACEHOLDERS)
                    and not k.startswith(exempt)]
    return raw, uncalibrated
