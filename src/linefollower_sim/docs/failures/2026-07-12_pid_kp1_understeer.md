# Failure writeup: kp=1.0 understeers off the first curve

**Date:** 2026-07-12 · **Phase:** 2 (PID baseline) · **Cost:** two recorded
runs — by design. The evidence rules said film the first attempt *before*
knowing if it works, because a first failure can't be re-filmed honestly.

**What happened (attempt 1, on camera):** first-ever PID run with default
gains (kp=1.0, no kd/ki). Tracked the straight cleanly for ~18 seconds,
then at the first curve the line slid sideways out of the camera's
detection band. The counter logged **15 "line lost" events in about one
second** (the line flickering at the band's edge), then the robot went
permanently blind, crawled around in its recovery spin, and wandered off
the oval. Lap never completed.

**Attempt 2 (accidental, also on camera):** meant to rerun with better
gains, actually reran the defaults. Same controller failed *differently* —
line out the LEFT edge after just 6 seconds. Same gains, half the lifetime.
The control loop runs asynchronously, so an unstable controller doesn't
even fail reproducibly.

**Diagnosis — pure-P understeer:** a P-only controller's turn command is
proportional to how far the line is from center. To hold the oval's
tightest curve at 0.15 m/s the robot needs a *sustained* turn of about
0.28 rad/s — which kp=1.0 only produces once the line is already ~0.28
away from center. Add sensor lag, and the line walks out of the detection
band before the correction catches up. The controller wasn't buggy; it was
mathematically too weak for the curve.

**Fix:** kp=2.5 — enough turn authority that the line never drifts far.
Three consecutive clean laps (40.4 s each, zero losses) on the first try,
which became the baseline the RL policy had to beat.

**Bonus find:** those "15 losses" were one physical event, double-counted
by edge flicker. The loss counter needs debouncing before any PID-vs-RL
loss comparison — logged as a to-do so the baseline stats stay honest.

**Lessons:**
- A proportional controller needs a steady error to produce a steady
  output — on curves, that error is a losing race against your sensor's
  field of view.
- Unstable systems fail non-deterministically; "it failed differently"
  is not evidence that anything changed.
- Film the first attempt. The kp=1.0 footage is the "before" that makes
  the working baseline mean something.
- Look inside your metrics: one event can masquerade as fifteen.
