#!/usr/bin/env python3
"""Phase 3 step 2: PPO training on the fixed oval.

PPO (Stable-Baselines3) collects rollouts from LineFollowEnv and updates a
small CNN policy. Assumes the sim stack is already up — launch everything
with rl_train.sh, not this file directly.

Artifacts land under <repo>/training/:
    training/checkpoints/   model snapshots every 5k steps + *_last.zip
                            on exit (Ctrl-C is safe — the model is saved)
    training/tb/            TensorBoard event files (evidence material!)
                            watch live: python3 -m tensorboard.main --logdir
                            ~/linefollower_ws/training/tb   -> localhost:6006

Resume after a stop:
    python3 train_ppo.py --resume auto      # newest checkpoint
    python3 train_ppo.py --resume PATH.zip

The sim keeps running in real time while PPO does its gradient updates
(one to two minutes of number-crunching every n_steps); the wheels are
stopped for the duration so the robot doesn't wander off mid-update on a
stale command (StopWheelsOnUpdate below).
"""
import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from line_env import LineFollowEnv

from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback, CheckpointCallback
from stable_baselines3.common.monitor import Monitor

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..'))
CKPT_DIR = os.path.join(REPO, 'training', 'checkpoints')
TB_DIR = os.path.join(REPO, 'training', 'tb')


class StopWheelsOnUpdate(BaseCallback):
    """Zero the wheels while PPO crunches gradients (sim time keeps flowing)."""

    def __init__(self, raw_env):
        super().__init__()
        self.raw_env = raw_env

    def _on_rollout_end(self):
        self.raw_env._publish_wheels(-1.0, -1.0)

    def _on_step(self):
        return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--steps', type=int, default=300000)
    ap.add_argument('--resume', default=None,
                    help="checkpoint .zip to continue from, or 'auto'")
    ap.add_argument('--smoke', action='store_true',
                    help='tiny n_steps, nothing saved — plumbing test only')
    args = ap.parse_args()

    os.makedirs(CKPT_DIR, exist_ok=True)
    os.makedirs(TB_DIR, exist_ok=True)

    raw = LineFollowEnv(track='oval', world='track_oval')
    env = Monitor(raw)

    resume_path = None
    if args.resume == 'auto':
        zips = sorted(glob.glob(os.path.join(CKPT_DIR, '*.zip')),
                      key=os.path.getmtime)
        if not zips:
            sys.exit('--resume auto: no checkpoints in ' + CKPT_DIR)
        resume_path = zips[-1]
    elif args.resume:
        resume_path = args.resume

    if resume_path:
        print('resuming from', resume_path)
        model = PPO.load(resume_path, env=env, tensorboard_log=TB_DIR)
    else:
        # ent_coef 0.01: a little exploration pressure so "stand still for a
        # safe 0" doesn't become a comfortable local optimum.
        model = PPO('CnnPolicy', env, verbose=1, ent_coef=0.01,
                    n_steps=64 if args.smoke else 2048,
                    batch_size=64, tensorboard_log=TB_DIR)

    callbacks = [StopWheelsOnUpdate(raw)]
    if not args.smoke:
        callbacks.append(CheckpointCallback(
            save_freq=5000, save_path=CKPT_DIR, name_prefix='ppo_oval'))

    try:
        model.learn(total_timesteps=args.steps,
                    callback=callbacks,
                    tb_log_name='smoke' if args.smoke else 'ppo_oval',
                    reset_num_timesteps=resume_path is None)
    except KeyboardInterrupt:
        print('\ninterrupted — saving before exit')
    finally:
        if not args.smoke:
            last = os.path.join(CKPT_DIR, 'ppo_oval_last')
            model.save(last)
            print('model saved:', last + '.zip')
            print("resume with: python3 scripts/train_ppo.py --resume auto")
        env.close()


if __name__ == '__main__':
    main()
