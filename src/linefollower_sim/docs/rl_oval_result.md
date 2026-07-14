# RL policy on the oval — first result (Phase 3, steps 1–2)

Trained overnight 2026-07-12→13, PPO (Stable-Baselines3 CnnPolicy) on the
fixed oval, camera-only: 64×64 grayscale in, two wheel speeds out. Model:
`training/checkpoints/ppo_oval_last.zip` (run died at ~158k of 300k steps
when Gazebo's transport dropped at 1:15am — the reward curve had already
been flat for ~40k steps, so nothing was lost).

## Headline numbers (ground truth)

Measured by `scripts/test_rl_policy.py` — lap times from **centerline
arc length using the ground-truth pose**, not the lap-timer script:

| Metric | PID baseline | RL policy |
|--------|--------------|-----------|
| Lap time (sim s) | 40.4 | **~22.2 (≈1.8× faster)** |
| Line losses | 0 | **0** |
| Worst centerline offset | — | **0.068 m** |
| Continuous laps evaluated | 3 | 4.05 (90 sim-s) |

The policy picks its own speed (up to 0.3 m/s per wheel); the PID cruises
at a hand-set 0.15 m/s — see the fairness caveats in `pid_baseline.md`.

## Learning curve

`rollout/ep_rew_mean` went from −5.6 to ~265, plateauing around 120k steps
(~4.5 h wall). Raw TensorBoard event files: `training/tb/ppo_oval_1/`.
Curve screenshot: `docs/evidence/2026-07-14_tb_ep_rew_mean_ppo_oval.png`.

## Filmed laps (2026-07-14)

Uncut OBS recording, same window layout as the PID footage (Gazebo god
view + the policy's 64×64 camera + lap-timer terminal): **4 complete laps,
20.6–20.7 s each on the timer, 0 line losses**, kept locally
(`D:\Videos\2026-07-14 10-51-36.mkv`, 3:01). Still:
`docs/evidence/2026-07-14_rl_lap4_complete.png`.

The on-screen 20.7 s is *not* the citable lap time: the lap detector
triggers 0.35 m early and re-anchors there, so its "laps" cover ~5.7 m of
the 6.35 m loop (details in `pid_baseline.md` caveats — the PID's 40.4 s
carries the same bias, so the comparison is fair). Scaled to the full
loop, 20.7 × 6.35/5.7 ≈ 23 s — consistent with the ground-truth 22.2 s.

## What this does NOT show yet

Only the training track. The claim that matters — generalizing to tracks
the policy has never seen — is Phase 3 steps 3–5 (random track per
episode, domain randomization, unseen-track eval).
