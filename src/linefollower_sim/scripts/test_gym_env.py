#!/usr/bin/env python3
"""Done-check for LineFollowEnv (Phase 3 step 1). Assumes the sim stack is
already up (test_gym_env.sh launches it). Five checks:
  1. SB3's check_env accepts the env (spaces, dtypes, API contract).
  2. reset() teleports to spawn: on the centerline, line visible in the frame.
  3. Driving forward makes positive progress without leaving the line.
  4. Driving straight forever leaves the oval corridor -> terminated, d > 0.2.
  5. reset() after termination puts us back on the line.
"""
import sys

import numpy as np
from stable_baselines3.common.env_checker import check_env

from line_env import LineFollowEnv, OFF_LINE_M


def main():
    env = LineFollowEnv(track='oval', world='track_oval')

    print('check 1: sb3 check_env ...')
    check_env(env, warn=True)
    print('  PASS')

    print('check 2: reset lands on the line, line in frame ...')
    obs, info = env.reset()
    assert obs.shape == (64, 64, 1) and obs.dtype == np.uint8, obs.shape
    d0 = info['centerline_dist']
    dark = int((obs < 80).sum())
    print(f'  centerline_dist={d0:.3f} m, dark pixels in frame={dark}')
    assert d0 < 0.05, f'spawn is {d0:.3f} m off the line'
    assert dark > 20, 'line not visible in the camera frame after reset'
    print('  PASS')

    print('check 3: full speed ahead for 15 steps stays on, moves forward ...')
    progress = 0.0
    for _ in range(15):
        obs, r, terminated, truncated, info = env.step(
            np.array([1.0, 1.0], np.float32))
        progress += info['progress_m']
        assert not terminated, (
            f'went off line already at progress={progress:.2f} m '
            f'(d={info["centerline_dist"]:.3f})')
    print(f'  progress={progress:.3f} m, d={info["centerline_dist"]:.3f} m')
    assert progress > 0.2, f'no forward progress: {progress:.3f} m'
    print('  PASS')

    print('check 4: keep driving straight -> must leave the corridor ...')
    terminated = False
    last_r = 0.0
    for i in range(100):
        obs, last_r, terminated, truncated, info = env.step(
            np.array([1.0, 1.0], np.float32))
        if terminated:
            break
    d = info['centerline_dist']
    print(f'  terminated={terminated} after {i + 1} more steps, '
          f'd={d:.3f} m, final step reward={last_r:.2f}')
    assert terminated and d > OFF_LINE_M, 'never left the corridor?'
    assert last_r < -5, f'off-line penalty missing from reward: {last_r:.2f}'
    print('  PASS')

    print('check 5: reset after termination lands back on the line ...')
    obs, info = env.reset()
    print(f'  centerline_dist={info["centerline_dist"]:.3f} m')
    assert info['centerline_dist'] < 0.05
    print('  PASS')

    env.close()
    print('test_gym_env: ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
