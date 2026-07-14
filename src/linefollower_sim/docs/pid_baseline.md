# PID baseline — the numbers to beat

Recorded 2026-07-12 (Phase 2, sim only). This is the classical-controller
benchmark the RL policy (Phase 3) has to outperform.

## Setup

- Track: `worlds/track_oval.sdf` — the fixed oval benchmark (lap ≈ 6.1 m by odometry)
- Controller: `scripts/pid_controller.py` via `bash scripts/pid_lap.sh 2.5`
  — gains **kp=2.5, ki=0, kd=0**, cruise speed **0.15 m/s** (hand-set constant)
- Perception: `scripts/line_detector.py` — black/white thresholding of the
  64×64 camera image, line position from the bottom band (rows 42–63)
- Metrics: `scripts/lap_metrics.py`, **sim time** from odometry stamps
  (the sim runs at 40–80 % of real time on this laptop, so wall-clock
  times would be inflated and unfair for later comparisons)

## Result — three consecutive laps, zero losses

| Lap | Time (sim s) | Distance (m) | Line losses |
|-----|--------------|--------------|-------------|
| 1   | 40.4         | 6.1          | 0           |
| 2   | 40.4         | 6.1          | 0           |
| 3   | 40.3         | 6.1          | 0           |

**Baseline: 40.4 s per lap, 0 line losses.**
Evidence: `docs/evidence/2026-07-12_pid_lap3_complete.png` (terminal with lap
timer + the thresholded PID view + Gazebo in one frame); full uncut 3-lap
recording kept locally (`D:\Videos\2026-07-12 18-10-34.mkv`).

## Tuning history (honest)

- **kp=1.0 (default), attempt 1:** lost the line on the first curve at
  t≈18 s / 2.7 m and wandered off blind. Pure-P understeer: the tightest
  curve needs ~0.28 rad/s of sustained turn at 0.15 m/s, which kp=1.0 only
  produces at a ~0.28 position error — too close to the edge of the
  detection band.
- **kp=1.0, attempt 2:** same gains failed at t≈6 s / 0.9 m — the async
  control loop makes an unstable controller fail at a different point
  every run.
- **kp=2.5:** stable on the first try; three clean laps, no kd/ki needed
  at this speed.

## What "beating the baseline" means for the RL policy

1. Faster than 40.4 s/lap on this same oval. Note the PID's speed is a
   hand-picked constant (0.15 m/s); the policy chooses wheel speeds itself.
2. Zero line losses, matching the PID.
3. The part the PID cannot do at all: keep working on random tracks it has
   never seen, and under randomized lighting/camera angles — the PID's
   perception (fixed threshold, fixed band) only works in this clean world.

## Caveats

- This is a *reasonably tuned* PID, not the best possible one: speed was
  left at 0.15 m/s and gains were tuned only until laps were clean. A
  faster PID is possible; if the RL comparison ever looks unfair, re-tune
  the PID at higher speed first.
- The line-loss counter counts every visible→lost transition undebounced.
  Irrelevant here (0 losses), but define a debounced loss metric before
  quoting loss counts in the Phase 3 comparison — edge flicker once counted
  15 "losses" for one physical event.
- The lap detector triggers when the robot is back within 0.35 m of the
  start point, then re-anchors the start THERE — so a "lap" covers ~6.0 m
  of the 6.35 m loop (~5 % flattering) and the finish line creeps ~0.35 m
  backward every lap (spotted 2026-07-14 in the RL footage). The 6.1 m in
  the table above is the same effect. It cancels out in PID-vs-RL
  comparisons because both use this script; ground-truth lap times
  (`test_rl_policy.py`, centerline arc length) are the citable numbers.
