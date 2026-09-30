# 2026-10-01 LiDAR/odometry offline audit

## Scope and result

This is a Mac-local, read-only replay of four copied rosbag2 SQLite bags. It decodes `/scan_multi`, `/odom_raw`, and `/tf_static` with the repository's standard-library CDR reader, projects returns using each `session.yaml` cell/heading and timestamp-interpolated odometry, then runs the current `GridAssociation` and the offline `propose_pose_correction`. It does not change odometry, write the navigation map, or command the vehicle.

The current runtime has no wall snapping or LiDAR-to-odometry correction feedback. `NavRuntime.on_scan` projects with the latest odometry and frozen manual maze anchor, then passes associated WALL/OPEN evidence to `StreamNav`; it does not update odometry, the anchor, or the follower pose. `pose_correction.py` only proposes a bounded correction from caller-supplied endpoints and confirmed segments. Its only callers are tests. The scan debug node remains diagnostic-only.

## Raw bag replay

The table compares each current-code association vote with the supplied 7×7 topology, under the recorded cell/heading anchor. “Hit precision” is the fraction of UNIQUE wall candidates landing on topology walls; “OPEN precision” is the fraction of free-path edge votes landing on topology openings. Votes are individual correlated rays, not independent samples. Every percentage is conditional on the physical maze axes matching the recorded anchor.

| Session | Scans | UNIQUE: wall / open | Hit precision | OPEN: open / wall | OPEN precision | UNIQUE residual p50 / p90 / p95 |
|---|---:|---:|---:|---:|---:|---:|
| `210036_entry_sidewall_N_2cell` | 455 | 32,442 / 445 | 98.65% | 106,730 / 1,841 | 98.30% | 8.8 / 31.8 / 35.5 mm |
| `211258_west_shift_front_sidewall` | 566 | 76,108 / 0 | 100.00% | 53,747 / 0 | 100.00% | 11.3 / 36.1 / 37.3 mm |
| `212853_junction_NE_open_SW_wall` | 1,477 | 332,172 / 732 | 99.78% | 483,647 / 5,568 | 98.86% | 18.7 / 39.6 / 44.1 mm |
| `213746_tf_cleanup_smoke` | 105 | 23,694 / 45 | 99.81% | 34,457 / 382 | 98.90% | 18.1 / 39.5 / 44.1 mm |

The archived `frames.jsonl`/summary votes differ substantially for some sessions. The previous [static calibration report](2026-09-30-lidar-static-calibration.md) records that those summaries predate the current outer-boundary OPEN guard and same-frame WALL-veto logic. These raw-bag numbers are therefore a replay of the current conflict-suppression code as a candidate fix; they do not describe the original session's online output or prove the fix on an independent live run.

## Static wall geometry and correction proposals

At the junction capture `(1,2), N`, I used only three local wall segments supported by both the supplied topology and the walked-path notes: `(1,2) S`, `(1,2) W`, and the dead-end `(1,4) N`. The reported `(0,4) N` visible segment was excluded from fitting because its finite physical segment mapping is not surveyed well enough for this fit.

For each confirmed segment, raw points fell within a 40 mm band on all 1,477 scans. Measured line summaries from `analyze_scan_edge.py`:

| Segment | Returns per scan | Expected-line absolute residual p50 / p90 / p95 | Per-scan fitted-line p95 residual: median / p95 |
|---|---:|---:|---:|
| `(1,2) S`, y=0.8 m | 77–88 | 13.7 / 19.2 / 20.7 mm | 10.7 / 19.3 mm |
| `(1,2) W`, x=0.4 m | 74–86 | 26.9 / 37.2 / 38.0 mm | 8.8 / 11.8 mm |
| `(1,4) N`, y=2.0 m | 23 | 12.9 / 15.3 / 15.6 mm | 3.8 / 5.3 mm |

The `(1,2)` junction bag was split by scan index: even scans estimated per-frame proposals; odd scans evaluated one shared median delta. All 738 fit scans proposed a correction. Median proposal was **(+23.25 mm x, +11.77 mm y, +1.20° yaw)**. On the odd-scan held-out endpoints matched to the same three segments, residuals changed from **14.90 / 34.56 / 36.56 mm** to **7.70 / 18.71 / 22.39 mm** (p50/p90/p95); the fraction within 35 mm changed from **91.35% to 98.28%**.

The short `213746_tf_cleanup_smoke` bag has the same recorded `(1,2), N` label. Under the unverified assumption it was the same physical pose and frame, applying the junction median delta to this separate session changed held-out local-wall residuals from **14.66 / 33.63 / 35.46 mm** to **7.26 / 18.95 / 22.68 mm**, with the within-35-mm fraction increasing from **93.84% to 98.32%**. This is repeatability at a nominal static pose, not independent spatial or moving-robot validation.

Both results support a consistent small static alignment discrepancy under the assumed wall coordinates. They do not identify its cause: manual cell-center placement, heading alignment, physical wall offset, scan assembly calibration, or another frame bias can produce similar results. No proposed delta was applied.

## Odometry timing and observability

All four bags contain a bitwise-stationary decoded `/odom_raw` pose sequence: odometry path length and maximum displacement are **0.0 m**. Thus these sessions cannot validate wheel scale, odometry drift, scan deskew, or moving-pose mutual calibration.

As an executor-timing proxy, I compared each scan header stamp with the latest odometry header stamp whose bag record arrived before that scan. Median lag was **47–48 ms**, p95 **85 ms** in all sessions. Seven scans in the entry session exceeded 200 ms; its maximum was 1.451 s. The odometry pose was constant, so the estimated position/yaw error in these bags is zero; that is a property of the stopped captures, not evidence that this lag is safe while moving. `NavRuntime.on_scan` currently consumes `latest_odom` instead of interpolating by scan timestamp, so a future moving capture should quantify or remove this skew before wall-based pose estimates are trusted. The proxy is based on bag record order, not a direct ROS callback trace.

The captured static TF path `base_footprint <- base_link` resolves to identity `(0,0,0)`. Session metadata calls `/scan_multi` frame `base_link` but does not identify how the fused scan's own sensor origins/extrinsics were calibrated. The bag has `/tf` too, but dynamic TF was not needed for the recorded pose chain: raw odometry supplies base pose and static TF supplies the scan-frame transform.

## Reproduction

From the repository root, this command writes the machine-readable summary next to this report and evaluates the junction proposal on the nominally same-pose smoke session:

```bash
python3 software/tools/audit_lidar_odom_calibration.py \
  field_data/20260930_210036_entry_sidewall_N_2cell \
  field_data/20260930_211258_west_shift_front_sidewall \
  field_data/20260930_212853_junction_NE_open_SW_wall \
  field_data/20260930_213746_tf_cleanup_smoke \
  --cross-evaluate-from field_data/20260930_212853_junction_NE_open_SW_wall \
  --output docs/reports/2026-10-01-lidar-odom-offline-audit-data.json
```

The script opens SQLite in read-only mode and never edits bag contents. Existing CDR tests plus three truth/projection/holdout geometry tests passed:

```bash
/Users/2003singer/.venv-html-to-docx/bin/python -m pytest -q \
  software/tests/test_analyze_scan_edge.py \
  software/tests/test_audit_lidar_odom_calibration.py \
  software/tests/test_field_maze_truth.py
```

Result: **8 passed**. Session `git_sha` is `unknown`; the field JSON explicitly leaves the physical image transform unresolved. Treat the topology comparison as conditional on the local manual frame interpretation, not as a full surveyed calibration.
