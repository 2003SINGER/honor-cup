# Robot-local WALL trust prior: offline diagnostic

## Implementation boundary

`m3pro_nav.wall_trust_prior` computes a soft, diagnostic-only prior for a new
WALL promotion from one post-snap edge hit. It is not wired into `EdgeMap`,
`nav_runtime`, or frame-grid-snap. Pose correction always has weight 1.0; yaw
rate changes only the returned WALL-promotion weight.

The side-wall envelope uses corrected-pose local geometry: walls whose tangent
is parallel to robot-forward are side walls. The prior uses observed endpoint
positions along that direction, with weight 1.0 through 1.0 m, a smooth medium
band through 1.2 m, a declining transition through 1.6 m, then 0.12. Behind
the robot receives 0.18. Front walls do not use this side visibility mask.
Range, contiguous narrow-ROI support, normal MAD, and actual beam incidence
when an explicit incidence field exists contribute continuous factors. The
heading-to-wall-normal proxy is ignored as beam incidence.

## Replay study

The evaluator applies the established narrow-ROI spatial floor (±5 cm, one
contiguous cluster with at least 6 points and 18 cm span), then freezes a
maximum prior credit per 20 cm/20° independent-view cluster. The displayed
credit thresholds are an offline comparison scale, not runtime EdgeMap score
units. Far observations (side visibility weight ≤0.55 or range factor <0.55)
need two independent viewpoint clusters; other observations need one. Repeated
stationary frames cannot create more than one credit within a cluster.

| Capture | Frames | Eligible edges | Threshold | True walls confirmed / 62 | False OPEN edges |
|---|---:|---:|---:|---:|---:|
| 17:47 full | 1,763 | 63 | 0.00 | 62/62 | 1: `N(4,5)` |
| 17:47 full | 1,763 | 63 | 1.00 | 62/62 | 0 |
| 17:47 full | 1,763 | 63 | 2.50 | 60/62 | 0 |
| 17:41 short | 848 | 17 | 0.00 | 16/62 | 0 |
| 17:41 short | 848 | 17 | 1.00 | 9/62 | 0 |
| 17:41 short | 848 | 17 | 2.50 | 3/62 | 0 |

The short capture only observed a small route subset; on its 17 eligible edge
IDs, the no-weight baseline confirms 16 true walls and no false wall. Weight
1.00 cuts this to 9, and 2.50 is too strict at 3. The full capture shows the
opposite limitation: it can retain 62/62 while removing its sole low-threshold
false edge, `N(4,5)`, but that does not establish a portable threshold. `E(4,2)`
does not pass the spatial floor in these logs, consistent with its fragmented
short-span ROI support; the prior does not need to “rescue” it.

These are same-site, same-session-derived captures and posthoc topology
scoring. They do not establish production parameters or generalization. The
current merged `/scan_multi` records have no beam source IDs, so actual beam
incidence is unavailable; the measured field only exposes heading proxy and
that proxy is not used. Yaw rate is present for 81 of 601 full-capture selected
view credits and 4 of 32 short-capture credits, so the turn factor is sparsely
exercised by this data.

## Verification and reproduction

Six focused unit tests cover the measured side bands, behind-car penalty,
front-wall separation, incidence semantics, smooth range/ROI/MAD weights, and
turn-only promotion penalty. Run them with:

```bash
python3 -m unittest software.tests.test_wall_trust_prior -v
```

Replay both compressed logs with:

```bash
python3 software/tools/evaluate_wall_trust_prior.py \
  --log experiments/2026-10-02-wall-replay/frame-grid-snap-quality.jsonl.zst \
  --truth field/maze_truth_7x7.json
python3 software/tools/evaluate_wall_trust_prior.py \
  --log experiments/2026-10-02-wall-replay/frame-grid-snap-174144-quality.jsonl.zst \
  --truth field/maze_truth_7x7.json
```

No pose-correction, runtime, map, or existing sweep file was changed.
