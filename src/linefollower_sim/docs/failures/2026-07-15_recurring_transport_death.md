# Failure writeup: the recurring transport death (and why we stopped fighting it)

**Dates:** 2026-07-13, 2026-07-14 (×2) · **Phase:** 3 · **Cost:** zero work
lost, three times in a row — that's the point of this writeup.

**What happens:** every long headless training run on this laptop
eventually dies the same way. After 3.5–6 hours, Gazebo's transport layer
(its internal messaging system) just stops — `Host unreachable` in the
Gazebo log, and the next service call from our code (usually the episode
reset's teleport) times out and raises. Three occurrences:

1. **2026-07-13, 1:15 am** — first overnight oval run, died at 158k of
   300k steps.
2. **2026-07-14, afternoon** — pool training, died at 213k of 300k.
3. **2026-07-14, night** — DR training, died at 300k of 350k.

**What we did about it — and what we deliberately didn't:** we never found
the root cause, and we stopped looking. It's somewhere in the
WSL2 + software-rendering + Gazebo Fortress transport stack — below our
code, intermittent, hours to reproduce. Chasing it would burn days for a
sim that's nearly done serving its purpose. Instead the failure was made
*cheap*:

- the env raises after 15 s without a camera frame (watchdog) instead of
  hanging forever;
- the training script saves the model on ANY exit — Ctrl-C, crash,
  whatever (`finally:` block);
- checkpoints every 5k steps cap the worst case at ~10 minutes of loss;
- every launcher kills leftover processes from dead runs, because a crash
  here reliably orphans a `parameter_bridge` (see the two-bridge writeup
  for what happens if you don't).

**The other half of the mitigation is knowing your learning curves:** all
three deaths cost nothing partly because the reward curve had already been
flat for tens of thousands of steps each time. "The run died at 86%" only
matters if the last 14% would have changed anything; a flat curve says it
wouldn't.

**Lessons:**
- Some failures should be *survived*, not solved. Root-causing an
  intermittent bug in someone else's stack, below your abstraction, on a
  6-hour reproduction cycle, is usually worse than making the crash cost
  ten minutes.
- Design for dirty death from day one: watchdogs, save-on-exit,
  checkpoints, self-cleaning launchers. Each was added after the first
  crash; runs two and three then cost literally nothing.
- Check the curve before mourning a dead run — flat means finished,
  whatever the step counter says.
