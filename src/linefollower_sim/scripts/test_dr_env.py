#!/usr/bin/env python3
"""Done-check for domain randomization (Phase 3 step 4). Assumes the oval
stack is up (test_dr_env.sh launches it). Five checks:
  1. dr=False is bit-identical to the raw camera frame (no silent change
     to the non-DR pipeline).
  2. dr=True actually perturbs the frame, and nuisance draws vary across
     episodes.
  3. Same env seed -> same nuisance draw (reproducible runs).
  4. Noise is resampled per frame (two steps at the same pose differ).
  5. Full speed ahead still makes progress with wheel gains applied.
"""
import sys

import numpy as np

from line_env import LineFollowEnv


def draws(env):
    return (env._dr_shift, round(env._dr_bright, 3),
            round(env._dr_contrast, 3), round(env._dr_noise, 3),
            (round(env._dr_gain[0], 3), round(env._dr_gain[1], 3)))


def main():
    print('check 1: dr=False leaves the frame untouched ...')
    env = LineFollowEnv(track='oval', dr=False)
    obs, _ = env.reset(seed=0)
    with env._lock:
        raw = env._frame.copy()
    assert np.array_equal(obs, raw), 'non-DR obs differs from raw frame'
    env.close()
    print('  PASS')

    env = LineFollowEnv(track='oval', dr=True)

    print('check 2: dr=True perturbs the frame, draws vary per episode ...')
    seen = set()
    for i in range(6):
        obs, _ = env.reset(seed=100 + i)
        seen.add(draws(env))
    with env._lock:
        raw = env._frame.copy()
    diff = np.abs(obs.astype(int) - raw.astype(int)).mean()
    print(f'  mean |obs - raw| = {diff:.2f}, {len(seen)} distinct draws in 6')
    assert diff > 0.5, 'augmentation did nothing'
    assert len(seen) >= 5, f'draws barely vary: {seen}'
    print('  PASS')

    print('check 3: same seed -> same nuisance draw ...')
    env.reset(seed=42)
    a = draws(env)
    env.reset(seed=42)
    b = draws(env)
    assert a == b, (a, b)
    print(f'  PASS {a}')

    print('check 4: noise differs frame to frame at the same pose ...')
    # stand still (wheels at -1 = speed 0) and take two steps: same view,
    # noise must differ. Use a seed whose noise draw is comfortably > 0.
    for s in range(42, 62):
        env.reset(seed=s)
        if env._dr_noise > 2.0:
            break
    o1, *_ = env.step(np.array([-1.0, -1.0], np.float32))
    o2, *_ = env.step(np.array([-1.0, -1.0], np.float32))
    d = np.abs(o1.astype(int) - o2.astype(int)).mean()
    print(f'  noise sigma {env._dr_noise:.2f}, frame-to-frame diff {d:.2f}')
    assert d > 0.1, 'noise looks frozen across frames'
    print('  PASS')

    print('check 5: full speed 10 steps still progresses under wheel gains ...')
    obs, info = env.reset(seed=7)
    print(f'  gains {env._dr_gain[0]:.3f} / {env._dr_gain[1]:.3f}')
    progress = 0.0
    for _ in range(10):
        obs, r, terminated, truncated, info = env.step(
            np.array([1.0, 1.0], np.float32))
        progress += info['progress_m']
        if terminated:
            break
    print(f'  progress={progress:.3f} m, terminated={terminated}')
    assert progress > 0.12, f'no forward progress: {progress:.3f} m'
    print('  PASS')

    env.close()
    print('test_dr_env: ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
