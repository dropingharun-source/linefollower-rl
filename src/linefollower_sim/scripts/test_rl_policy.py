#!/usr/bin/env python3
"""Headless eval of the trained policy through LineFollowEnv (the sim stack
must be up — test_rl_policy.sh launches the matching world). One
deterministic episode; reports distance covered, worst centerline offset,
and lap count. The FILMED laps use rl_lap.sh + lap_metrics like the PID
did; this is the plumbing/behavior check.

--track N evaluates on random track seed N (launch via test_rl_policy.sh
--track N so the same track is loaded in Gazebo). Seeds below 100000 are
guaranteed never to be in a training pool — that's the unseen-track eval.
"""
import argparse
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
    ap = argparse.ArgumentParser()
    ap.add_argument('model', nargs='?', default=None,
                    help='model .zip; default: newest *_last.zip')
    ap.add_argument('--track', default='oval',
                    help="'oval' or a random-track seed; must match the "
                         'world the launcher loaded')
    ap.add_argument('--stochastic', action='store_true')
    ap.add_argument('--trace', action='store_true')
    ap.add_argument('--dr', action='store_true',
                    help='evaluate WITH domain randomization active '
                         '(robustness check)')
    args = ap.parse_args()
    stochastic = args.stochastic

    zips = sorted(glob.glob(os.path.join(CKPT_DIR, '*_last.zip')),
                  key=os.path.getmtime)
    model_path = args.model or (zips or [None])[-1]
    if not model_path:
        sys.exit('no *_last.zip in ' + CKPT_DIR)
    model = PPO.load(model_path, device='cpu')
    print('evaluating', model_path, 'on track', args.track,
          '(stochastic — training-style sampling)' if stochastic
          else '(deterministic)')

    # generate_track.py --seed N names its world "track_random"
    world = 'track_oval' if args.track == 'oval' else 'track_random'
    env = LineFollowEnv(track=args.track, world=world, max_steps=900,
                        dr=args.dr)
    obs, info = env.reset()
    total_progress, worst_d, steps = 0.0, 0.0, 0
    terminated = truncated = False
    trace = args.trace
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
