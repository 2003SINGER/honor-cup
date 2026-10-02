"""Shared finite-wall fit and causal Manhattan frame correction.

The implementation is extracted verbatim from the measured offline replay.
No map truth or future observations enter this module. ``solve_frame_correction``
changes no navigation state; callers decide whether to consume its proposal.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
import random
from .pose import Pose2D, C
CELL_SIZE_M = C


@dataclass(frozen=True)
class Segment:
    """Finite line segment with unit tangent/normal, span, and fit residual."""
    a: tuple[float, float]
    b: tuple[float, float]
    inliers: int
    rms: float
    support: tuple[tuple[float, float], ...] = ()

    @property
    def tangent(self):
        dx, dy = self.b[0]-self.a[0], self.b[1]-self.a[1]
        n = math.hypot(dx, dy)
        return dx/n, dy/n

    @property
    def length(self):
        return math.dist(self.a, self.b)

def _fit_tls(points):
    mx = sum(p[0] for p in points)/len(points)
    my = sum(p[1] for p in points)/len(points)
    xx = sum((p[0]-mx)**2 for p in points)
    yy = sum((p[1]-my)**2 for p in points)
    xy = sum((p[0]-mx)*(p[1]-my) for p in points)
    theta = .5*math.atan2(2*xy, xx-yy)
    tx, ty = math.cos(theta), math.sin(theta)
    nx, ny = -ty, tx
    residuals = [(p[0]-mx)*nx+(p[1]-my)*ny for p in points]
    lo = min((p[0]-mx)*tx+(p[1]-my)*ty for p in points)
    hi = max((p[0]-mx)*tx+(p[1]-my)*ty for p in points)
    return ((mx+lo*tx, my+lo*ty), (mx+hi*tx, my+hi*ty),
            math.sqrt(sum(r*r for r in residuals)/len(residuals)))

def extract_segments(points, *, threshold=.018, min_inliers=6,
                     min_span=.12, max_gap=.075, seed=17, max_lines=16):
    """Deterministic pair-RANSAC, TLS refit, then split at point-cloud gaps."""
    remaining = list(dict.fromkeys((float(x), float(y)) for x, y in points))
    rng = random.Random(seed)
    result = []
    while len(remaining) >= min_inliers and len(result) < max_lines:
        best = []
        pairs = [(i, j) for i in range(len(remaining))
                 for j in range(i+1, len(remaining))]
        if len(pairs) > 1200:
            pairs = rng.sample(pairs, 1200)
        for i, j in pairs:
            ax, ay = remaining[i]; bx, by = remaining[j]
            dx, dy = bx-ax, by-ay
            norm = math.hypot(dx, dy)
            if norm < min_span: continue
            nx, ny = -dy/norm, dx/norm
            inliers = [p for p in remaining
                       if abs((p[0]-ax)*nx+(p[1]-ay)*ny) <= threshold]
            if len(inliers) > len(best): best = inliers
        if len(best) < min_inliers: break
        a, b, _ = _fit_tls(best)
        tx, ty = (b[0]-a[0])/math.dist(a,b), (b[1]-a[1])/math.dist(a,b)
        mx = sum(p[0] for p in best)/len(best); my = sum(p[1] for p in best)/len(best)
        ordered = sorted(best, key=lambda p: (p[0]-mx)*tx+(p[1]-my)*ty)
        groups, group = [], [ordered[0]]
        for p in ordered[1:]:
            if math.dist(p, group[-1]) > max_gap:
                groups.append(group); group = []
            group.append(p)
        groups.append(group)
        consumed = set(best)
        remaining = [p for p in remaining if p not in consumed]
        for group in groups:
            if len(group) < min_inliers: continue
            a, b, rms = _fit_tls(group)
            seg = Segment(a, b, len(group), rms, tuple(group))
            if seg.length >= min_span and rms <= threshold:
                result.append(seg)
    return sorted(result, key=lambda s: (-s.inliers, s.a))

@dataclass(frozen=True)
class GridSnapConfig:
    max_dx: float = .12
    max_dy: float = .12
    max_dyaw: float = math.radians(6)
    max_wall_angle: float = math.radians(15)
    association_gate: float = .12
    association_ambiguity: float = .025
    consensus_gate: float = .035
    max_consensus_mad: float = .025
    min_residual_improvement: float = .002
    min_score_margin: float = .001
    min_translation_walls: int = 2
    translation_step: float = .005
    yaw_step: float = math.radians(.25)

@dataclass(frozen=True)
class FrameCorrection:
    accepted: bool
    mode: str
    reason: str
    dx: float
    dy: float
    dyaw: float
    corrected_pose: Pose2D
    fitted_wall_count: int
    associated_wall_count: int
    inlier_wall_count: int
    pre_residual_m: float | None
    post_residual_m: float | None
    score_margin: float | None
    outlier_indices: tuple[int, ...]
    x_residual_median: float | None = None
    x_residual_mad: float | None = None
    y_residual_median: float | None = None
    y_residual_mad: float | None = None

def _wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))

def _axis_angle(seg: Segment):
    a = math.atan2(seg.tangent[1], seg.tangent[0])
    return ((a + math.pi / 2) % math.pi) - math.pi / 2

def _transform_segment(seg: Segment, pose: Pose2D, dx: float, dy: float,
                       dyaw: float) -> Segment:
    # Correction yaw is about the robot origin; translation is world-frame.
    c, s = math.cos(dyaw), math.sin(dyaw)
    def tr(p):
        x, y = p[0] - pose.x, p[1] - pose.y
        return (pose.x + dx + c*x - s*y, pose.y + dy + s*x + c*y)
    return Segment(tr(seg.a), tr(seg.b), seg.inliers, seg.rms,
                   tuple(tr(p) for p in seg.support))

def _median(xs):
    if not xs: return None
    a = sorted(xs); n = len(a)
    return a[n//2] if n % 2 else (a[n//2-1] + a[n//2]) / 2

def _mad(xs):
    m = _median(xs)
    return _median([abs(x-m) for x in xs]) if m is not None else None

def _associate_axis(segments, pose, yaw_delta, orient, cell, field_n, cfg):
    records = []
    for idx, original in enumerate(segments):
        seg = _transform_segment(original, pose, 0., 0., yaw_delta)
        angle = abs(_axis_angle(seg))
        is_vertical = angle > math.pi/4
        if (orient == 'V') != is_vertical:
            continue
        if (math.pi/2-angle if is_vertical else angle) > cfg.max_wall_angle:
            continue
        coord = (seg.a[0]+seg.b[0])/2 if is_vertical else (seg.a[1]+seg.b[1])/2
        support=seg.support or (seg.a,seg.b)
        tangential=[p[1] if is_vertical else p[0] for p in support]
        if not any(0. <= t < field_n*cell for t in tangential):
            continue
        k0 = round(coord / cell)
        # Only one or two identities in the local correction window.
        ks = [k for k in range(max(0, k0-1), min(field_n, k0+1)+1)
              if abs(k*cell-coord) <= cfg.association_gate]
        ds = sorted((abs(k*cell-coord), k) for k in ks)
        if not ds: continue
        if len(ds) > 1 and ds[1][0]-ds[0][0] < cfg.association_ambiguity:
            continue
        residual = ds[0][1]*cell - coord
        records.append((idx, residual, coord, ds[0][1], _axis_angle(seg)))
    return records

def _axis_consensus(records, cfg):
    if not records: return [], None, None, None
    # Collapse every RANSAC fragment on one absolute infinite grid line to a
    # single vote so a fragmented/high-point-count wall cannot dominate.
    by_line={}
    for rec in records: by_line.setdefault(rec[3],[]).append(rec)
    collapsed=[]
    for k,group in sorted(by_line.items()):
        med_res=_median([r[1] for r in group])
        rep=min(group,key=lambda r:abs(r[1]-med_res))
        collapsed.append((rep[0],med_res,_median([r[2] for r in group]),k,rep[4]))
    records=collapsed
    values = [r[1] for r in records]
    center = _median(values)
    inliers = [r for r in records if abs(r[1]-center) <= cfg.consensus_gate]
    residuals = [r[1] for r in inliers]
    return inliers, _median(residuals), _mad(residuals), center

def solve_frame_correction(segments, predicted_pose, cell_size=CELL_SIZE_M,
                           field_n=7, config=GridSnapConfig()):
    """Find a shared bounded correction for segments already in predicted world.

    `segments` must be projected from this frame's local hits using
    `predicted_pose`. Associations are limited to nearby lattice lines and are
    recomputed under each yaw hypothesis. A single axis can constrain its
    normal translation and yaw, while the tangential coordinate is inherited.
    """
    pred = predicted_pose
    empty = lambda why: FrameCorrection(False, 'REJECTED', why, 0., 0., 0., pred,
        len(segments), 0, 0, None, None, None, tuple(range(len(segments))))
    if not segments: return empty('NO_WALL_SEGMENTS')
    # Direction estimate: robustly seek a yaw which aligns as many independent
    # segment directions to the nearest Manhattan axis as possible.
    candidates=[]
    nstep=max(1, round(2*config.max_dyaw/config.yaw_step))
    for q in range(nstep+1):
        yaw=-config.max_dyaw + q*(2*config.max_dyaw/nstep)
        errs=[]
        for seg in segments:
            a=_axis_angle(_transform_segment(seg,pred,0,0,yaw))
            e=min(abs(a), abs(abs(a)-math.pi/2))
            if e <= config.max_wall_angle: errs.append(e)
        if errs:
            candidates.append((len(errs), -(_median(errs) or 0.), yaw))
    if not candidates: return empty('NO_MANHATTAN_DIRECTIONS')
    # First form an independent robust yaw estimate from measured directions.
    # The joint search may refine it, but translation residuals are not allowed
    # to invent a yaw that the wall directions themselves do not support.
    direction_groups={}
    for seg in segments:
        a=_axis_angle(seg)
        if abs(a)<=math.pi/4:
            off=a; orient='H'; normal=(seg.a[1]+seg.b[1])/2
        else:
            off=a-math.copysign(math.pi/2,a); orient='V'; normal=(seg.a[0]+seg.b[0])/2
        if abs(off)<=config.max_wall_angle:
            # Collapse fragments on the same approximate infinite support line.
            k=round(normal/cell_size)
            direction_groups.setdefault((orient,k),[]).append(off)
    axis_offsets=[_median(v) for v in direction_groups.values()]
    if not axis_offsets: return empty('NO_YAW_CONSENSUS')
    yaw_center=-_median(axis_offsets)
    yaw_spread=_mad(axis_offsets) or 0.
    yaw_allowance=max(math.radians(1.),3*yaw_spread)
    candidates=[c for c in candidates if abs(c[2]-yaw_center)<=yaw_allowance]
    if not candidates: return empty('NO_YAW_CONSENSUS')
    # Evaluate whole shared pose hypotheses, not only a yaw optimum followed
    # by a separate translation. Adjacent yaw lattice samples are one basin;
    # competing basins must be separated by >1 degree or >2 cm.
    hypotheses=[]
    for _,_,yaw in candidates:
        vx=_associate_axis(segments,pred,yaw,'V',cell_size,field_n,config)
        hy=_associate_axis(segments,pred,yaw,'H',cell_size,field_n,config)
        xi,xmed,xmad,_=_axis_consensus(vx,config)
        yi,ymed,ymad,_=_axis_consensus(hy,config)
        x_ok=(len({r[3] for r in xi})>=config.min_translation_walls and
              xmad is not None and xmad<=config.max_consensus_mad)
        y_ok=(len({r[3] for r in yi})>=config.min_translation_walls and
              ymad is not None and ymad<=config.max_consensus_mad)
        if not (x_ok or y_ok): continue
        dx=xmed if x_ok else 0.; dy=ymed if y_ok else 0.
        if abs(dx)>config.max_dx+1e-9 or abs(dy)>config.max_dy+1e-9: continue
        normal_residuals=[]; angle_errors=[]
        for recs,enabled,axis in ((xi,x_ok,'V'),(yi,y_ok,'H')):
            if not enabled: continue
            for idx,_,coord,k,_ in recs:
                seg=_transform_segment(segments[idx],pred,dx,dy,yaw)
                linecoord=(seg.a[0]+seg.b[0])/2 if axis=='V' else (seg.a[1]+seg.b[1])/2
                normal_residuals.append(abs(k*cell_size-linecoord))
        # One orientation vote per associated infinite support line. This
        # avoids a fragmented wall dominating yaw through repeated segments.
        for axis,recs in (('V',xi),('H',yi)):
            for idx,_,_,_,_ in recs:
                moved=_transform_segment(segments[idx],pred,dx,dy,yaw)
                a=_axis_angle(moved); e=min(abs(a),abs(abs(a)-math.pi/2))
                angle_errors.append(e)
        # Keep the two evidence types separately robust. Pooling angle and
        # distance values into one median can discard all direction evidence
        # when line count is balanced, allowing yaw to counterfeit translation.
        fit=(_median(normal_residuals) or 0.) + .40*(_median(angle_errors) or 0.)
        if fit is None: continue
        # Small explicit odometry prior tie-breaks nearly equal geometric fits.
        score=fit + .05*math.hypot(dx,dy) + .02*abs(yaw)
        hypotheses.append({'score':score,'dx':dx,'dy':dy,'yaw':yaw,
            'vx':vx,'hy':hy,'xi':xi,'yi':yi,'xmed':xmed,'xmad':xmad,
            'ymed':ymed,'ymad':ymad,'xok':x_ok,'yok':y_ok,'fit':fit})
    if not hypotheses: return empty('INSUFFICIENT_INDEPENDENT_WALLS')
    hypotheses.sort(key=lambda h:h['score'])
    best=hypotheses[0]
    competitor=next((h for h in hypotheses[1:]
        if abs(h['yaw']-best['yaw'])>math.radians(1) or
           math.hypot(h['dx']-best['dx'],h['dy']-best['dy'])>.02),None)
    score_margin=(competitor['score']-best['score']) if competitor else None
    vx,hy,xi,yi=(best['vx'],best['hy'],best['xi'],best['yi'])
    x_ok,y_ok=best['xok'],best['yok']
    dx,dy,dyaw=best['dx'],best['dy'],best['yaw']
    xmed,xmad,ymed,ymad=best['xmed'],best['xmad'],best['ymed'],best['ymad']
    # Pre/post residual use the selected associations and held constant line IDs.
    pre_vals=[]; post_vals=[]
    for axis,recs,enabled in (('V',xi,x_ok),('H',yi,y_ok)):
        if not enabled: continue
        for idx,_,_,k,_ in recs:
            before=_transform_segment(segments[idx],pred,0,0,0)
            after=_transform_segment(segments[idx],pred,dx,dy,dyaw)
            bcoord=(before.a[0]+before.b[0])/2 if axis=='V' else (before.a[1]+before.b[1])/2
            acoord=(after.a[0]+after.b[0])/2 if axis=='V' else (after.a[1]+after.b[1])/2
            pre_vals.append(abs(k*cell_size-bcoord)); post_vals.append(abs(k*cell_size-acoord))
    pre=_median(pre_vals); post=_median(post_vals)
    improvement=(pre-post) if pre is not None and post is not None else 0.
    accepted_lines={('V',r[3]) for r in xi} if x_ok else set()
    accepted_lines|={('H',r[3]) for r in yi} if y_ok else set()
    inlier_ids=set()
    for axis,recs,enabled,center in (('V',vx,x_ok,xmed),('H',hy,y_ok,ymed)):
        if not enabled: continue
        for idx,residual,_,k,_ in recs:
            if (axis,k) in accepted_lines and abs(residual-center)<=config.consensus_gate:
                inlier_ids.add(idx)
    if score_margin is not None and score_margin<config.min_score_margin:
        reason='AMBIGUOUS_JOINT_HYPOTHESIS'; accepted=False
    elif improvement < config.min_residual_improvement:
        reason='INSUFFICIENT_RESIDUAL_IMPROVEMENT'; accepted=False
    else:
        accepted=True; reason='ACCEPTED'
    full=x_ok and y_ok
    mode='XY_YAW' if full else ('X_ONLY' if x_ok else 'Y_ONLY')
    if not accepted: mode='REJECTED'; dx=dy=dyaw=0.
    corrected=Pose2D(pred.x+dx,pred.y+dy,_wrap_pi(pred.yaw+dyaw))
    return FrameCorrection(accepted,mode,reason,dx,dy,dyaw,corrected,len(segments),
        len(vx)+len(hy),len(accepted_lines),pre,post,score_margin,
        tuple(i for i in range(len(segments)) if i not in inlier_ids),xmed,xmad,ymed,ymad)
