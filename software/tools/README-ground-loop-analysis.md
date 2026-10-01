# Ground loop trial CSV comparison

Run the read-only analyzer on one or more `ground_loop_trial` logs:

```sh
python3 software/tools/analyze_ground_loop_trial.py \
  /path/to/first-ground-loop.csv \
  /path/to/second-ground-loop.csv
```

The report transforms every pose into the body frame recorded by `trial_start`.
It reports peak midpoint left/forward overrun relative to the fixed 0.4 m by
0.4 m template, return right/back overrun relative to the start, terminal
position and wrapped yaw error, outbound/return hold durations, stop reason,
and parameters encoded in the trial start record. `--json` emits structured
output.

Odometry frequency is calculated only from individual `odom_sample` callback
rows, separately for source stamps and monotonic receipt times. Older logs
without those rows show the cadence as unavailable; their `control_sample`
rate is never presented as the odometry publisher rate. The input files are
read only.
