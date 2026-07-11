# Line-Follower Robot (RL Policy)

A line-following robot that navigates with a **camera only** — no hardcoded rules.

An RL policy (small neural network) is trained in a Gazebo simulation on thousands of randomly
generated tracks, then transferred to a real robot (Jetson Orin Nano). The real poster track is
never in the training set — *"follows a track it has never seen"* is the headline demo.
A classical PID controller (IR sensors + Arduino) serves as the baseline to beat.

## Stack
- **Sim:** Gazebo Fortress + ROS 2 Humble (Ubuntu 22.04 / WSL2)
- **RL:** Gym environment + PPO (Stable-Baselines3), 64×64 camera obs → wheel speeds
- **Robot:** Jetson Orin Nano, differential drive, front camera tilted ~55°

## Roadmap
- [x] Phase 0 — Setup: WSL2, ROS 2 Humble, Gazebo, this repo
- [ ] Phase 1 — Sim world: robot model, random track generator
- [ ] Phase 2 — PID baseline in sim (numbers to beat)
- [ ] Phase 3 — RL training: PPO, random tracks, domain randomization
- [ ] Phase 4 — Hardware build
- [ ] Phase 5 — Sim-to-real transfer
- [ ] Phase 6 — Documentation & failure writeups (ongoing)

Everything gets documented here, including honest failure writeups.
