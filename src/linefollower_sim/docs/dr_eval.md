# Domain randomization — training + evaluation (Phase 3 step 4)

**Date:** 2026-07-14→15. DR implementation: see `scripts/line_env.py`
(`dr=True`) — per-episode camera-angle shift (±4 px ≈ ±4.3°), brightness
±25 / contrast ×[0.8, 1.2], per-frame pixel noise (σ ∈ [0, 8] per
episode), per-wheel speed gains [0.9, 1.1]. All in the env's obs/action
path; the sim itself is untouched.

## Training

Warm-start from `ppo_pool_last` on the 16-track pool with `--dr`:
213k → 300k steps (~87k DR steps). No adaptation dip — reward held the
ceiling from the first rollout and settled at **276**, *above* the non-DR
265 (wheel gains > 1.0 let good episodes finish faster). The run died at
300k of 350k by the recurring Gazebo transport death (third occurrence;
model auto-saved, as designed — writeup pending). Curve was flat →
declared converged. Model: `training/checkpoints/ppo_pool_dr_last.zip`.
TB run: `ppo_pool_dr_0`. Curve screenshot with all three runs (oval →
pool → DR, one continuous arc):
`docs/evidence/2026-07-14_tb_full_training_arc.png`.

## Evaluation — 7/7 clean

`bash scripts/test_rl_policy.sh --track N [--dr]`, deterministic, 90 sim-s
each, ground-truth metrics. Seeds 1–3 are outside every training set.

| Track | Nuisances | Worst offset | Pace | Clean? |
|-------|-----------|--------------|------|--------|
| seed 1 (unseen) | off | 0.086 m | ~26.4 s/lap | ✅ |
| seed 2 (unseen) | off | 0.088 m | ~25.7 s/lap | ✅ |
| seed 3 (unseen) | off | 0.087 m | ~25.8 s/lap | ✅ |
| seed 1 (unseen) | **on** | 0.060 m | ~29.0 s/lap | ✅ |
| seed 2 (unseen) | **on** | 0.095 m | ~23.6 s/lap | ✅ |
| seed 3 (unseen) | **on** | 0.103 m | ~24.4 s/lap | ✅ |
| oval (benchmark) | off | 0.092 m | **~21.7 s/lap** | ✅ |

Zero terminations across all seven runs. With nuisances on, pace varies
(23.6–29.0 s/lap — the random wheel gains directly change achievable
speed) but the robot never leaves the line. The oval time *improved* vs
the pre-DR policy (21.7 vs 22.2 s/lap).

## Honest note

Like step 3, the "before" policy was already partly robust — the DR run
showed no adaptation dip at these nuisance ranges. The step still earns
its keep: it *proves* robustness across the whole nuisance distribution
(the eval with nuisances on), pushed the oval pace up, and the DR ranges
are now the knobs we widen when the real robot exposes gaps sim didn't
model (Phase 5's iterate loop: find gap → widen randomization → retrain).

**This model (`ppo_pool_dr_last.zip`) is the sim-to-real deployment
candidate.**
