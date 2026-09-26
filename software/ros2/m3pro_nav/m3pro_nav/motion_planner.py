#!/usr/bin/env python3
"""MotionPlanner —— 三格固定模板编译 (图接口, 2026-09-25 定稿).

公开接口只有格子三元组:

    template(prev_cell, cell, next_cell, pose)
      din  = cell - prev
      dout = next - cell
      dout ==  din  → STRAIGHT   (入口边中点 → 出口边中点)
      dout == -din  → REVERSE 宏 (编译为两段 STRAIGHT: 入中心→原路退出, 不是 TURN180)
      cross(din,dout) > 0 → LEFT_ARC   R = C/2 = 0.2m 四分之一圆
      cross(din,dout) < 0 → RIGHT_ARC

entry_side/exit_side 只是函数内部为算边中点临时派生的量, 绝不向上暴露.
MotionPlanner 不知道方块 / 剪枝 / DFS —— 它只把图递推给出的 (prev,cell,next)
逐个编译成固定模板.

Graph traversal, branch bookkeeping and STOP/wait behavior belong to
``ActionHorizon``. This module compiles fixed geometry only."""

import math
from .pose import Pose2D, C, DIRV
from .motion_primitive import MotionPrimitive, ARC_RADIUS

R_ARC = ARC_RADIUS      # Fixed and enforced at MotionPrimitive construction.
V_ARC = 0.45           # 弧内速度 (全局模板参数, 保守)
V_CRUISE = 0.70
GRAB_V = 0.15          # 死路倒车速度
GEOM_EPS = 1e-9


class PlanGeometryMismatch(ValueError):
    """Raised when graph cursor and the physical pose disagree geometrically."""

    def __init__(self, message, *, prev=None, cell=None, nxt=None, expected=None,
                 actual=None, cursor=None):
        self.context = dict(prev=prev, cell=cell, nxt=nxt, expected=expected,
                            actual=actual, cursor=cursor)
        super().__init__(f"{message}; context={self.context}")


def _primitive_end(prim):
    if prim.kind == 'STRAIGHT':
        return prim.p1
    if prim.kind == 'ARC':
        r = prim.meta['r']
        a = prim.yaw0 + prim.yaw1
        return (prim.p0[0] + r * math.cos(a), prim.p0[1] + r * math.sin(a))
    return (prim.start_pose.x, prim.start_pose.y)


def _is_unit_cardinal(dv):
    return dv in ((0, 1), (1, 0), (0, -1), (-1, 0))


def validate_geometry(prims, *, cursor=None):
    """Hard structural gate for motion types and primitive-to-primitive joins."""
    previous_end = None
    for index, prim in enumerate(prims):
        if prim.kind == 'STRAIGHT':
            dx, dy = abs(prim.p1[0] - prim.p0[0]), abs(prim.p1[1] - prim.p0[1])
            if dx > GEOM_EPS and dy > GEOM_EPS:
                raise PlanGeometryMismatch("STRAIGHT is not axis aligned", cursor=cursor,
                    expected="x0==x1 or y0==y1", actual=(prim.p0, prim.p1))
            template = prim.meta.get('template')
            if template is not None:
                _, cell, _ = template
                x0, y0 = cell[0] * C, cell[1] * C
                x1, y1 = x0 + C, y0 + C
                for point in (prim.p0, prim.p1):
                    if not (x0 - GEOM_EPS <= point[0] <= x1 + GEOM_EPS and
                            y0 - GEOM_EPS <= point[1] <= y1 + GEOM_EPS):
                        raise PlanGeometryMismatch("STRAIGHT leaves its template cell corridor",
                            cursor=cursor, expected=(x0, y0, x1, y1), actual=point)
            start = prim.p0
        elif prim.kind == 'ARC':
            r = prim.meta.get('r')
            if r is None or abs(r - R_ARC) > GEOM_EPS or abs(abs(prim.yaw1) - math.pi / 2) > GEOM_EPS:
                raise PlanGeometryMismatch("ARC violates fixed quarter-circle contract", cursor=cursor,
                    expected=(R_ARC, math.pi / 2), actual=(r, prim.yaw1))
            start = (prim.p0[0] + r * math.cos(prim.yaw0),
                     prim.p0[1] + r * math.sin(prim.yaw0))
            if 'entry_anchor' not in prim.meta or 'exit_anchor' not in prim.meta:
                raise PlanGeometryMismatch("ARC lacks compiled edge-anchor contract", cursor=cursor,
                    expected="entry_anchor and exit_anchor", actual=prim.meta)
            end = _primitive_end(prim)
            if (math.hypot(start[0] - prim.meta['entry_anchor'][0],
                           start[1] - prim.meta['entry_anchor'][1]) > GEOM_EPS or
                    math.hypot(end[0] - prim.meta['exit_anchor'][0],
                               end[1] - prim.meta['exit_anchor'][1]) > GEOM_EPS):
                raise PlanGeometryMismatch("ARC endpoints are not template edge midpoints", cursor=cursor,
                    expected=(prim.meta['entry_anchor'], prim.meta['exit_anchor']),
                    actual=(start, end))
            triple = prim.meta.get('template')
            if triple is None:
                raise PlanGeometryMismatch("ARC lacks graph template triple", cursor=cursor,
                    expected="(prev, cell, next)", actual=triple)
            prev, cell, nxt = triple
            din = (cell[0] - prev[0], cell[1] - prev[1])
            dout = (nxt[0] - cell[0], nxt[1] - cell[1])
            if not _is_unit_cardinal(din) or not _is_unit_cardinal(dout) or din == dout or din == (-dout[0], -dout[1]):
                raise PlanGeometryMismatch("ARC has invalid graph triple", cursor=cursor,
                    expected="unit cardinal turn", actual=triple)
            cell_center = ((cell[0] + 0.5) * C, (cell[1] + 0.5) * C)
            expected_entry = (cell_center[0] - din[0] * R_ARC,
                              cell_center[1] - din[1] * R_ARC)
            expected_exit = (cell_center[0] + dout[0] * R_ARC,
                             cell_center[1] + dout[1] * R_ARC)
            expected_corner = (expected_entry[0] + dout[0] * R_ARC,
                               expected_entry[1] + dout[1] * R_ARC)
            if (math.hypot(prim.p0[0] - expected_corner[0], prim.p0[1] - expected_corner[1]) > GEOM_EPS or
                    math.hypot(prim.meta['entry_anchor'][0] - expected_entry[0],
                               prim.meta['entry_anchor'][1] - expected_entry[1]) > GEOM_EPS or
                    math.hypot(prim.meta['exit_anchor'][0] - expected_exit[0],
                               prim.meta['exit_anchor'][1] - expected_exit[1]) > GEOM_EPS):
                raise PlanGeometryMismatch("ARC geometry is outside its graph template", cursor=cursor,
                    expected=(expected_corner, expected_entry, expected_exit),
                    actual=(prim.p0, prim.meta['entry_anchor'], prim.meta['exit_anchor']))
            start_tangent = ((-math.sin(prim.yaw0) if prim.yaw1 > 0 else math.sin(prim.yaw0)),
                             ( math.cos(prim.yaw0) if prim.yaw1 > 0 else -math.cos(prim.yaw0)))
            end_angle = prim.yaw0 + prim.yaw1
            end_tangent = ((-math.sin(end_angle) if prim.yaw1 > 0 else math.sin(end_angle)),
                           ( math.cos(end_angle) if prim.yaw1 > 0 else -math.cos(end_angle)))
            if (math.hypot(start_tangent[0] - din[0], start_tangent[1] - din[1]) > GEOM_EPS or
                    math.hypot(end_tangent[0] - dout[0], end_tangent[1] - dout[1]) > GEOM_EPS):
                raise PlanGeometryMismatch("ARC tangents disagree with graph triple", cursor=cursor,
                    expected=(din, dout), actual=(start_tangent, end_tangent))
        elif prim.kind == 'STOP':
            start = (prim.start_pose.x, prim.start_pose.y)
        else:
            raise PlanGeometryMismatch("unknown motion primitive kind", cursor=cursor,
                expected=("STRAIGHT", "ARC", "STOP"), actual=prim.kind)
        if previous_end is not None and math.hypot(start[0] - previous_end[0], start[1] - previous_end[1]) > GEOM_EPS:
            raise PlanGeometryMismatch("primitive chain is discontinuous", cursor=cursor,
                expected=previous_end, actual=start)
        previous_end = _primitive_end(prim)


class MotionPlanner:
    def __init__(self, v_cruise=V_CRUISE, v_arc=V_ARC, exit_len=0.6):
        self.v_cruise = v_cruise
        self.v_arc = v_arc
        self.exit_len = exit_len

    # ---- 单步: (prev, cell, nxt) → 模板 ----

    def template(self, prev, cell, nxt, pose):
        """Compile exactly one graph triple into its fixed geometric template.

        Return ``(primitives, end_point, end_cursor)``. The pose must be the
        legal incoming anchor, except for the explicit root bootstrap.
        """
        cur = pose
        din = (cell[0] - prev[0], cell[1] - prev[1]) if prev is not None else (0, 1)
        dout = (nxt[0] - cell[0], nxt[1] - cell[1])
        if not _is_unit_cardinal(dout) or (prev is not None and not _is_unit_cardinal(din)):
            raise PlanGeometryMismatch("template graph edges must be unit cardinal moves",
                prev=prev, cell=cell, nxt=nxt, expected="unit cardinal edges",
                actual=(din, dout), cursor=(prev, cell))
        center = ((cell[0] + 0.5) * C, (cell[1] + 0.5) * C)
        m_in = (center[0] - din[0] * R_ARC, center[1] - din[1] * R_ARC)
        m_out = (center[0] + dout[0] * R_ARC, center[1] + dout[1] * R_ARC)
        prims = []

        def seg(p0, p1, v_max, v_end):
            return MotionPrimitive(
                kind='STRAIGHT', start_pose=Pose2D(p0[0], p0[1], 0.0),
                p0=p0, p1=p1, yaw0=0.0, yaw1=0.0,
                length=math.hypot(p1[0] - p0[0], p1[1] - p0[1]),
                v_max=v_max, v_end=v_end)

        def template_seg(p0, p1, v_max, v_end):
            prim = seg(p0, p1, v_max, v_end)
            prim.meta['template'] = (prev, cell, nxt)
            return prim

        if prev is None:
            # The root has one explicit bootstrap template: cell center to its
            # selected outgoing edge midpoint. It is never a generic connector.
            if math.hypot(cur.x - center[0], cur.y - center[1]) > GEOM_EPS:
                raise PlanGeometryMismatch("root bootstrap must start at cell center",
                    prev=prev, cell=cell, nxt=nxt, expected=center,
                    actual=(cur.x, cur.y), cursor=(prev, cell))
            if dout[0] and dout[1]:
                raise PlanGeometryMismatch("root bootstrap direction is not axis aligned",
                    prev=prev, cell=cell, nxt=nxt, expected="axis direction",
                    actual=dout, cursor=(prev, cell))
            prims.append(template_seg(center, m_out, self.v_cruise, self.v_cruise))
            return prims, m_out, (cell, nxt)

        if math.hypot(cur.x - m_in[0], cur.y - m_in[1]) > GEOM_EPS:
            raise PlanGeometryMismatch("pose is not at template entry anchor",
                prev=prev, cell=cell, nxt=nxt, expected=m_in,
                actual=(cur.x, cur.y), cursor=(prev, cell))

        if dout == din:
            # STRAIGHT: 后处理统一衔接 (ARC 前降为 v_arc)
            prims.append(template_seg(m_in, m_out, self.v_cruise, self.v_cruise))
            return prims, m_out, (cell, nxt)

        if dout == (-din[0], -din[1]):
            # Dead-end retreat is the named center-to-entry-and-back template.
            # The next parent-cell template begins at this shared edge anchor.
            prims.append(template_seg(m_in, center, GRAB_V, 0.0))
            prims.append(template_seg(center, m_in, GRAB_V, GRAB_V))
            return prims, m_in, (cell, prev)

        # Quarter-turn circle center is the inner grid corner.
        corner = (m_in[0] + dout[0] * R_ARC, m_in[1] + dout[1] * R_ARC)
        cross = din[0] * dout[1] - din[1] * dout[0]
        delta = math.pi / 2 if cross > 0 else -math.pi / 2
        a0 = math.atan2(-dout[1], -dout[0])
        arc_end = (corner[0] + R_ARC * math.cos(a0 + delta),
                   corner[1] + R_ARC * math.sin(a0 + delta))
        if math.hypot(arc_end[0] - m_out[0], arc_end[1] - m_out[1]) > GEOM_EPS:
            raise PlanGeometryMismatch("turn template does not end at outgoing edge anchor",
                prev=prev, cell=cell, nxt=nxt, expected=m_out, actual=arc_end,
                cursor=(prev, cell))
        prims.append(MotionPrimitive(
            kind='ARC', start_pose=Pose2D(m_in[0], m_in[1], 0.0),
            p0=corner, yaw0=a0, yaw1=delta,
            length=abs(delta) * R_ARC, v_max=self.v_arc, v_end=self.v_arc,
            meta={'r': R_ARC, 'entry_anchor': m_in, 'exit_anchor': m_out,
                  'template': (prev, cell, nxt)}))
        return prims, m_out, (cell, nxt)

    # ---- Return path geometry: supplied cell path + fixed exit segment ----

    def compile_home(self, pose, cursor, path_cells, exit_dir):
        """Compile home from preserved (predecessor,current) cursor context."""
        prims = []
        cur = pose.copy()
        path = list(path_cells)
        prev, current = cursor
        if not path or path[0] != current:
            raise PlanGeometryMismatch("home path must begin at cursor cell", prev=prev,
                cell=current, expected=current, actual=path[0] if path else None,
                cursor=cursor)
        if len(path) == 1 and prev is None:
            raise PlanGeometryMismatch("home exit cell requires predecessor context",
                prev=prev, cell=current, expected="(predecessor, exit_cell)",
                actual=cursor, cursor=cursor)

        center = ((path[-1][0] + 0.5) * C, (path[-1][1] + 0.5) * C)
        m_out = (center[0] + DIRV[exit_dir][0] * R_ARC,
                 center[1] + DIRV[exit_dir][1] * R_ARC)
        for i, cell in enumerate(path):
            if i + 1 < len(path):
                nxt = path[i + 1]
            else:
                nxt = (cell[0] + DIRV[exit_dir][0], cell[1] + DIRV[exit_dir][1])
            step, end_pt, _ = self.template(prev, cell, nxt, cur)
            prims += step
            cur = Pose2D(end_pt[0], end_pt[1], cur.yaw)
            prev = cell
        # 出场段: 沿出口边方向冲出
        dv = DIRV[exit_dir]
        prims.append(MotionPrimitive(
            kind='STRAIGHT', start_pose=Pose2D(m_out[0], m_out[1], cur.yaw),
            p0=m_out, p1=(m_out[0] + dv[0] * self.exit_len,
                          m_out[1] + dv[1] * self.exit_len),
            yaw0=cur.yaw, yaw1=cur.yaw,
            length=self.exit_len, v_max=0.6, v_end=0.6,
            meta={'route_exit': True}))
        # 衔接后处理: ARC 前的直线段 v_end = v_arc (速度模长连续)
        for i in range(len(prims) - 1):
            if prims[i + 1].kind == 'ARC' and prims[i].kind == 'STRAIGHT':
                prims[i].v_end = self.v_arc
        validate_geometry(prims, cursor=cursor)
        return prims
