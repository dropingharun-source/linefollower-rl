#!/usr/bin/env python3
"""Headless eval of the trained policy through LineFollowEnv (the sim stack
must be up — test_gym_env.sh's launch set). One deterministic episode;
reports distance covered, worst centerline offset, and lap count. The
FILMED laps use rl_lap.sh + lap_metrics like the PID did; this is the
plumbing/behavior check.
"""
import glob
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from line_env import LineFollowEnv, OFF_LINE_M

from stable_baselines3 import PPO

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
CKPT_DIR = os.path.join(REPO, 'training', 'checkpoints')


def main():
    stochastic = '--stochastic' in sys.argv
    args = [a for a in sys.argv[1:] if not a.startswith('--')]
    zips = sorted(glob.glob(os.path.join(CKPT_DIR, '*_last.zip')),
                  key=os.path.getmtime)
    model_path = args[0] if args else (zips or [None])[-1]
    if not model_path:
        sys.exit('no *_last.zip in ' + CKPT_DIR)
    model = PPO.load(model_path, device='cpu')
    print('evaluating', model_path,
          '(stochastic — training-style sampling)' if stochastic
          else '(deterministic)')

    env = LineFollowEnv(track='oval', world='track_oval', max_steps=900)
    obs, info = env.reset()
    total_progress, worst_d, steps = 0.0, 0.0, 0
    terminated = truncated = False
    trace = '--trace' in sys.argv
    while not (terminated or truncated):
        action, _ = model.predict(obs, deterministic=not stochastic)
        obs, r, terminated, truncated, info = env.step(action)
        total_progress += info['progress_m']
        worst_d = max(worst_d, info['centerline_dist'])
        steps += 1
        if trace and (steps % 10 == 0 or terminated):
            print(f'  step {steps:>4}  d={info["centerline_dist"]:.3f}  '
                  f's={info["arc_s"]:.2f}  action=[{action[0]:+.2f} {action[1]:+.2f}]')
    env.close()

    lap_len = env.centerline.length
    laps = total_progress / lap_len
    sim_s = steps / 10.0  # 10 Hz control
    print(f'steps={steps} ({sim_s:.1f} sim-s)  progress={total_progress:.2f} m '
          f'({laps:.2f} laps of {lap_len:.2f} m)')
    print(f'worst centerline offset={worst_d:.3f} m (off-line limit {OFF_LINE_M})')
    print(f'terminated={terminated} (went off line)  truncated={truncated}')
    if laps >= 1.0 and not terminated:
        est = sim_s / laps
        print(f'estimated lap time ~{est:.1f} sim-s  (PID baseline: 40.4 s)')
        print('test_rl_policy: PASS')
        return 0
    print('test_rl_policy: FAIL — did not complete a clean lap')
    return 1


if __name__ == '__main__':
    sys.exit(main())
