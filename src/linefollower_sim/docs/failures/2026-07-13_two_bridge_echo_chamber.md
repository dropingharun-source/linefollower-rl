# Failure writeup: the two-bridge echo chamber

**Date:** 2026-07-13 · **Phase:** 3 (RL training) · **Cost:** one morning

**What happened:** the overnight PPO run crashed at 1:15 am (Gazebo's
internal messaging dropped), but the model was saved and its reward curve
looked great. In the morning we ran it — and the robot drove in useless
spirals, like it had learned nothing.

**Wrong guess first:** maybe evaluating without exploration noise broke it.
Two-minute test with noise on: failed identically. Guess dead.

**The clue:** the step-by-step trace showed impossible physics — full-forward
wheel commands with almost no motion, and the distance-to-line jumping
around like the robot was teleporting. Robots don't teleport. So the policy
wasn't broken; something *below* it was lying.

**Root cause:** the crash had killed the training stack but left its
`parameter_bridge` (the program that passes messages between ROS 2 and
Gazebo) alive as an orphan — a normal Ctrl-C kills it, a crash doesn't.
The morning eval launched a second bridge, and the two forwarded each
other's messages forever: stale wheel commands echoing around, every camera
frame arriving twice. A good policy fed scrambled inputs looks exactly like
a bad policy.

**Fix:** killed the orphan, re-ran the same eval — 4.05 clean laps, zero
line losses. Then made every launcher script kill leftover bridges
(`pkill -f '[p]arameter_bridge'`) before starting, so this can't recur.

**Lessons:**
- Physically impossible readings = stop debugging the top layer; the
  plumbing below is lying.
- Crash cleanup ≠ Ctrl-C cleanup: assume the previous run died dirty.
- If a known-good component suddenly acts insane, suspect its environment
  before the component — the model file never changed.
- Test the cheapest hypothesis first: two minutes killed the wrong guess.
