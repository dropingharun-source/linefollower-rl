#!/usr/bin/env python3
"""LineFollowEnv — Gym environment (the standard RL interface: reset() starts
an episode, step(action) advances one control tick) wrapping the Gazebo sim.

The env does NOT launch Gazebo. A launcher script (see test_gym_env.sh /
future train script) starts the sim headless plus a ros_gz_bridge for:
    /cmd_vel                              geometry_msgs/Twist
    /camera                               sensor_msgs/Image
    /model/linefollower_bot/pose_gt       nav_msgs/Odometry  (ground truth)
The env just connects to those topics, like the PID nodes do.

Observation: 64x64x1 grayscale uint8 — exactly what the camera publishes,
             luminance-converted. SB3's CnnPolicy takes this directly.
Action:      Box(-1, 1, (2,)) = [left, right] wheel commands, mapped to
             wheel speeds in [0, WHEEL_MAX] m/s (forward-only; turning in
             place = one wheel at 0). Published as an equivalent Twist,
             which is what the DiffDrive plugin listens to.
Control rate: one step per CONTROL_EVERY camera frames (30 Hz camera ->
             10 Hz control), so pacing follows SIM time like the PID did —
             wall-clock pacing would drift, the sim runs at 40-80% real time
             under software rendering.

Reward: progress along the track centerline (arc-length delta, meters,
        wrap-aware) x PROGRESS_SCALE, minus a small per-step cost so
        standing still bleeds. Off the line (ground-truth distance from
        centerline > OFF_LINE_M) ends the episode with OFF_LINE_PENALTY.
        Progress is measured from ground-truth pose, not pixels — the
        policy can't inflate it by fooling its own camera.

Reset: teleports the robot back to the spawn pose via the world's set_pose
       service (the reason pose_gt exists: wheel odometry never notices
       teleports), waits for physics + a fresh camera frame.

The track centerline comes from generate_track.py (same folder): 'oval' for
the benchmark oval, or an int seed for a random track — the launcher must
have generated and loaded that same track; the env only knows the math.

track='pool' (Phase 3 step 3): the launcher loads worlds/track_pool.sdf —
pool_size random tracks on a grid of panels in ONE world — and every reset
teleports the robot to a RANDOM track's spawn. Same proven set_pose path as
the single-track reset, just 16 destinations instead of 1; no world reloads,
no runtime scene edits. pool_seed/pool_size must match what the launcher
passed to generate_track.py --pool.

dr=True (Phase 3 step 4, domain randomization): per-episode nuisances,
applied entirely in this file's observation/action path — the sim itself
never changes, so long-run stability is untouched:
  camera angle   image shifted ±4 px vertical / ±2 px horizontal
                 (FOV is 1.204 rad / 64 px ≈ 1.08°/px, so ±4 px ≈ ±4.3° —
                 the plan's ±3–5° mount-angle jitter, first-order approx)
  lighting       brightness offset ±25 and contrast ×[0.8, 1.2]
  sensor noise   gaussian pixel noise, sigma drawn per episode from [0, 8],
                 resampled every frame
  motor gap      each wheel's commanded speed × its own gain in [0.9, 1.1]
                 (covers motor asymmetry AND friction/slip variation — the
                 form floor-friction differences actually take at the axle)
"""
import math
import os
import subprocess
import sys
import threading

import numpy as np
import gymnasium as gym
from gymnasium import spaces

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import generate_track

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

WHEEL_MAX = 0.3        # m/s per wheel (PID baseline drove 0.15 constant)
WHEEL_SEP = 0.165      # from model.sdf
CONTROL_EVERY = 3      # camera frames per env step: 30 Hz / 3 = 10 Hz
OFF_LINE_M = 0.20      # centerline distance that ends the episode
OFF_LINE_PENALTY = -10.0
PROGRESS_SCALE = 10.0  # reward per meter of centerline progress
STEP_COST = 0.01
FRAME_TIMEOUT_S = 15.0  # wall seconds without a camera frame -> sim is dead

# domain randomization ranges (see module docstring)
DR_SHIFT_V_PX = 4     # camera pitch jitter, ~1.08 deg per px
DR_SHIFT_H_PX = 2     # camera yaw jitter
DR_BRIGHT = 25.0      # brightness offset (uint8 scale)
DR_CONTRAST = 0.2     # contrast in [1-x, 1+x]
DR_NOISE_MAX = 8.0    # per-episode pixel-noise sigma drawn from [0, this]
DR_GAIN = 0.1         # per-wheel speed gain in [1-x, 1+x]


class _Polyline:
    """Closed centerline with point -> (arc length s, distance d) projection."""

    def __init__(self, pts):
        self.pts = pts
        n = len(pts)
        self.seg_start_s = []
        self.length = 0.0
        for i in range(n):
            self.seg_start_s.append(self.length)
            self.length += math.dist(pts[i], pts[(i + 1) % n])

    def project(self, x, y):
        n = len(self.pts)
        best = (math.inf, 0.0)  # (d, s)
        for i in range(n):
            ax, ay = self.pts[i]
            bx, by = self.pts[(i + 1) % n]
            vx, vy = bx - ax, by - ay
            seg_len2 = vx * vx + vy * vy
            t = ((x - ax) * vx + (y - ay) * vy) / seg_len2 if seg_len2 else 0.0
            t = min(1.0, max(0.0, t))
            px, py = ax + t * vx, ay + t * vy
            d = math.hypot(x - px, y - py)
            if d < best[0]:
                best = (d, self.seg_start_s[i] + t * math.sqrt(seg_len2))
        return best  # (d, s)


class LineFollowEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, track='oval', world='track_oval', max_steps=1000,
                 pool_seed=0, pool_size=16, dr=False):
        super().__init__()
        self.observation_space = spaces.Box(0, 255, (64, 64, 1), np.uint8)
        self.action_space = spaces.Box(-1.0, 1.0, (2,), np.float32)
        self.max_steps = max_steps
        self.world = world
        self.dr = dr
        self._dr_neutral()

        if track == 'pool':
            all_pts = generate_track.pool_tracks(pool_seed, pool_size)
        else:
            all_pts = [generate_track.oval_points() if track == 'oval'
                       else generate_track.random_points(int(track))]
        # one (centerline, spawn) per track; reset() picks one
        self._pool = [(_Polyline(p), generate_track.spawn_pose(p))
                      for p in all_pts]
        self.centerline, self.spawn = self._pool[0]
        self._track_idx = 0

        if not rclpy.ok():
            rclpy.init()
        self.node = Node('line_follow_env')
        self.cmd_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.node.create_subscription(Image, '/camera', self._on_frame, 10)
        self.node.create_subscription(
            Odometry, '/model/linefollower_bot/pose_gt', self._on_pose, 10)

        self._lock = threading.Condition()
        self._frame = None
        self._frame_count = 0
        self._pose = None
        self._steps = 0
        self._last_s = None

        self._running = True
        self._spin = threading.Thread(target=self._spin_loop, daemon=True)
        self._spin.start()

    # ---- ROS plumbing -----------------------------------------------------
    def _spin_loop(self):
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def _on_frame(self, msg):
        rgb = np.frombuffer(msg.data, np.uint8).reshape(64, 64, 3)
        gray = (0.299 * rgb[:, :, 0] + 0.587 * rgb[:, :, 1]
                + 0.114 * rgb[:, :, 2]).astype(np.uint8)
        with self._lock:
            self._frame = gray.reshape(64, 64, 1)
            self._frame_count += 1
            self._lock.notify_all()

    def _on_pose(self, msg):
        p = msg.pose.pose.position
        with self._lock:
            self._pose = (p.x, p.y)

    def _wait_frames(self, n):
        with self._lock:
            target = self._frame_count + n
            while self._frame_count < target:
                if not self._lock.wait(timeout=FRAME_TIMEOUT_S):
                    raise RuntimeError(
                        'no camera frame for %.0f s — is the sim stack '
                        'running? (see test_gym_env.sh for the launch set)'
                        % FRAME_TIMEOUT_S)
            return self._frame.copy()

    def _publish_wheels(self, left, right):
        vl = (float(left) + 1.0) / 2.0 * WHEEL_MAX * self._dr_gain[0]
        vr = (float(right) + 1.0) / 2.0 * WHEEL_MAX * self._dr_gain[1]
        t = Twist()
        t.linear.x = (vl + vr) / 2.0
        t.angular.z = (vr - vl) / WHEEL_SEP
        self.cmd_pub.publish(t)

    # ---- domain randomization -------------------------------------------
    def _dr_neutral(self):
        self._dr_shift = (0, 0)
        self._dr_bright = 0.0
        self._dr_contrast = 1.0
        self._dr_noise = 0.0
        self._dr_gain = (1.0, 1.0)

    def _dr_sample(self):
        rng = self.np_random
        self._dr_shift = (
            int(rng.integers(-DR_SHIFT_V_PX, DR_SHIFT_V_PX + 1)),
            int(rng.integers(-DR_SHIFT_H_PX, DR_SHIFT_H_PX + 1)))
        self._dr_bright = float(rng.uniform(-DR_BRIGHT, DR_BRIGHT))
        self._dr_contrast = float(rng.uniform(1 - DR_CONTRAST, 1 + DR_CONTRAST))
        self._dr_noise = float(rng.uniform(0.0, DR_NOISE_MAX))
        self._dr_gain = (float(rng.uniform(1 - DR_GAIN, 1 + DR_GAIN)),
                         float(rng.uniform(1 - DR_GAIN, 1 + DR_GAIN)))

    def _augment(self, img):
        """Apply this episode's nuisances to one camera frame."""
        if not self.dr:
            return img
        a = img.astype(np.float32)
        dv, dh = self._dr_shift
        if dv or dh:  # edge-replicated shift = small mount-angle change
            p = DR_SHIFT_V_PX
            pad = np.pad(a, ((p, p), (p, p), (0, 0)), mode='edge')
            a = pad[p + dv:p + dv + 64, p + dh:p + dh + 64]
        a = (a - 128.0) * self._dr_contrast + 128.0 + self._dr_bright
        if self._dr_noise > 0:  # fresh noise every frame
            a = a + self.np_random.normal(0.0, self._dr_noise, a.shape)
        return np.clip(a, 0, 255).astype(np.uint8)

    def _teleport_to_spawn(self):
        x, y, yaw = self.spawn
        req = ('name: "linefollower_bot", '
               'position: {x: %.4f, y: %.4f, z: 0.05}, '
               'orientation: {w: %.6f, z: %.6f}'
               % (x, y, math.cos(yaw / 2), math.sin(yaw / 2)))
        out = subprocess.run(
            ['ign', 'service', '-s', '/world/%s/set_pose' % self.world,
             '--reqtype', 'ignition.msgs.Pose',
             '--reptype', 'ignition.msgs.Boolean',
             '--timeout', '3000', '--req', req],
            capture_output=True, text=True)
        if 'true' not in out.stdout:
            raise RuntimeError('set_pose failed: %s %s'
                               % (out.stdout, out.stderr))

    # ---- Gym API ----------------------------------------------------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if len(self._pool) > 1:  # pool mode: a random track each episode
            self._track_idx = int(self.np_random.integers(len(self._pool)))
            self.centerline, self.spawn = self._pool[self._track_idx]
        if self.dr:  # new nuisance draw every episode
            self._dr_sample()
        self._publish_wheels(-1.0, -1.0)  # wheel speed 0
        self._wait_frames(2)              # let the stop reach the sim
        self._teleport_to_spawn()
        obs = self._augment(self._wait_frames(3))  # settle + flush stale
        with self._lock:
            pose = self._pose
        d, s = self.centerline.project(*pose)
        self._last_s = s
        self._steps = 0
        return obs, {'centerline_dist': d, 'track_idx': self._track_idx}

    def step(self, action):
        self._publish_wheels(action[0], action[1])
        obs = self._augment(self._wait_frames(CONTROL_EVERY))
        with self._lock:
            pose = self._pose
        d, s = self.centerline.project(*pose)

        half = self.centerline.length / 2.0
        ds = ((s - self._last_s + half) % self.centerline.length) - half
        self._last_s = s
        self._steps += 1

        reward = PROGRESS_SCALE * ds - STEP_COST
        terminated = d > OFF_LINE_M
        if terminated:
            reward += OFF_LINE_PENALTY
        truncated = self._steps >= self.max_steps

        info = {'centerline_dist': d, 'progress_m': ds, 'arc_s': s}
        return obs, reward, terminated, truncated, info

    def close(self):
        self._publish_wheels(-1.0, -1.0)
        # Stop the spin thread BEFORE tearing down rclpy — destroying the
        # node under an active spin_once aborts the whole process at exit.
        self._running = False
        self._spin.join(timeout=2.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
