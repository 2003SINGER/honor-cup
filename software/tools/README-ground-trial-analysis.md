# Ground trial CSV analysis

Run against one or several `control_probe --ground-odom-straight` CSVs after copying them to the Mac:

```sh
python3 software/tools/analyze_ground_trial.py field_data/trial-a/control_probe.csv field_data/trial-b/control_probe.csv
```

The comparison row reports requested travel, peak forward overshoot, final position error, peak cross-track and yaw error, settled state, stop reason, integrated command distance, and odometry cadence. It also prints the run's recorded `axis`, `distance`, speed, deceleration, `kp_pos`, and `kd_vel` values. Use `--json` for machine-readable output.

The odometry cadence check uses advancing source-stamp gaps, repeated-stamp hold time, and CSV sample receipt gaps against a default 0.5 s limit. The CSV does not record each odometry callback's receipt time, so this is a staleness indicator, not a direct age measurement. The analyzer only reads input CSVs and does not suggest or apply parameter changes.
