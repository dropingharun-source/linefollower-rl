#!/usr/bin/env python3
"""Done-check for pool mode (Phase 3 step 3). Assumes the pool world stack is
already up (test_pool_env.sh launches it). Five checks:
  1. Resets visit several DIFFERENT tracks, each landing on that track's
     centerline with the line visible in the camera frame.
  2. Same env seed -> same track sequence (reproducible training runs).
  3. Full speed ahead on a pool track makes progress without going off-line
     (the reward math and the world geometry agree).
  4. Driving straight leaves the corridor -> terminated with the penalty.
  5. Camera keeps a sane frame rate with 16 tracks in the scene (the pool's
     2048 static boxes must not choke software rendering).
"""
import sys
import time

import numpy as np

from line_env import LineFollowEnv, OFF_LINE_M

POOL_SIZE = 16  # must match what test_pool_env.sh generated


def main():
    env = LineFollowEnv(track='pool', pool_seed=0, pool_size=POOL_SIZE)

    print('check 1: resets land on the line across different tracks ...')
    visited = []
    for i in range(8):
        obs, info = env.reset(seed=i)
        d, idx = info['centerline_dist'], info['track_idx']
        dark = int((obs < 80).sum())
        print(f'  reset {i}: track {idx:2d}, centerline_dist={d:.3f} m, '
              f'dark pixels={dark}')
        assert d < 0.05, f'reset {i} is {d:.3f} m off track {idx}'
        assert dark > 20, f'no line visible on track {idx}'
        visited.append(idx)
    assert len(set(visited)) >= 4, f'only visited tracks {set(visited)}'
    print(f'  PASS ({len(set(visited))} distinct tracks in 8 resets)')

    print('check 2: same seed -> same track ...')
    _, a = env.reset(seed=123)
    _, b = env.reset(seed=123)
    assert a['track_idx'] == b['track_idx'], (a, b)
    print(f'  PASS (both landed on track {a["track_idx"]})')

    print('check 3: full speed ahead for 10 steps stays on, moves forward ...')
    obs, info = env.reset(seed=3)
    print(f'  on track {info["track_idx"]}')
    progress = 0.0
    for _ in range(10):
        obs, r, terminated, truncated, info = env.step(
            np.array([1.0, 1.0], np.float32))
        progress += info['progress_m']
        assert not terminated, (
            f'went off line already at progress={progress:.2f} m '
            f'(d={info["centerline_dist"]:.3f})')
    print(f'  progress={progress:.3f} m, d={info["centerline_dist"]:.3f} m')
    assert progress > 0.15, f'no forward progress: {progress:.3f} m'
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

    print('check 5: camera rate with the full pool in the scene ...')
    with env._lock:
        start = env._frame_count
    t0 = time.monotonic()
    time.sleep(5.0)
    with env._lock:
        got = env._frame_count - start
    wall_hz = got / (time.monotonic() - t0)
    print(f'  {got} frames in 5 s wall = {wall_hz:.1f} Hz '
          f'(sim runs 40-80% real time; oval stack gave ~12-25 Hz wall)')
    assert wall_hz > 8, f'camera crawling at {wall_hz:.1f} Hz wall'
    print('  PASS')

    env.close()
    print('test_pool_env: ALL PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
