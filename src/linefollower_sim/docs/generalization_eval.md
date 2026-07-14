# Unseen-track evaluation (Phase 3 step 3)

**Date:** 2026-07-14. The headline claim — *"follows a track it has never
seen"* — tested in sim for the first time. Eval seeds 1–3 can never appear
in a training pool (pool seeds start at 100000 by construction).

Setup: `bash scripts/test_rl_policy.sh --track N [model.zip]` — headless,
deterministic, one 900-step episode (90 sim-s), ground-truth metrics.

## Results

| Policy | Track | Laps in 90 s | Worst offset | Pace | Clean? |
|--------|-------|-------------|--------------|------|--------|
| pool-trained | seed 1 (unseen) | 3.38 of 7.74 m | 0.071 m | ~26.6 s/lap | ✅ |
| pool-trained | seed 2 (unseen) | 3.48 of 7.52 m | 0.075 m | ~25.8 s/lap | ✅ |
| pool-trained | seed 3 (unseen) | 3.47 of 7.56 m | 0.071 m | ~25.9 s/lap | ✅ |
| oval-only (control) | seed 1 (unseen) | 3.32 of 7.74 m | 0.055 m | ~27.1 s/lap | ✅ |

Zero terminations anywhere; every run ended by the step cap, still on the
line. Pace vs the PID, normalized per meter (tracks differ in length):
**RL ≈ 3.4 s/m on tracks it has never seen; the PID does 6.4 s/m on its
home oval.** Nearly 2× faster on unfamiliar ground than the classical
controller on familiar ground.

## Filmed demo (2026-07-14)

Uncut OBS recording of the pool policy on unseen seed 1, GUI mode
(`bash scripts/rl_lap.sh --track 1` — launch banner naming the unseen seed
in frame): 3 complete laps + most of a 4th in 3:56, lap timer showing
25.4 s / 7.1 m / 0 losses at lap 3 (lap 1 reads 34.1 s because it includes
~6 s of controller startup; the usual 0.35 m-early lap-timer bias applies,
see `pid_baseline.md`). Kept locally: `D:\Videos\2026-07-14 15-48-52.mkv`.
Stills: `docs/evidence/2026-07-14_rl_unseen_seed1_launch_banner.png`,
`docs/evidence/2026-07-14_rl_unseen_seed1_lap3_complete.png`.

## The honest surprise

We built random-track-per-episode training (a 16-track pool world) to
*force* generalization, expecting an oval-only policy to memorize its
track. The control row shows the assumption was wrong: **the oval-only
policy already generalizes** — it laps unseen tracks nearly as well as the
pool-trained one (even slightly tighter on offset, slightly slower on pace).

In hindsight this is reasonable: the policy sees only a 64×64 crop of the
~20–40 cm ahead of it. At that horizon, every track is just "a line
curving some amount" — the oval's curvature range apparently covered
enough of the distribution. Line following is a *local* task; memorizing
a global track shape would require information the camera never provides.

Pool training still wasn't wasted: it confirmed the policy holds its
reward ceiling (~265) across 16 varied tracks for 55k steps, and it is the
right infrastructure for domain randomization (step 4), where lighting,
camera angle, and sensor noise vary per episode — the variation that
matters for the real robot.

Pool training detail: warm-started from the oval model, ran 158k→213k
steps before Gazebo's transport layer died (same failure mode as the
2026-07-13 crash — second occurrence, watchdog + save-on-exit worked
again). Curve had been flat at ~265 the whole run, so it was not resumed:
converged is converged. Model: `training/checkpoints/ppo_pool_last.zip`,
TB run `ppo_pool_0`.
