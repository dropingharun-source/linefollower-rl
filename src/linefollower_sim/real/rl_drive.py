#!/usr/bin/env python3
"""Laptop driver node: webcam -> policy -> serial. The real-world rl_controller.

Runs on the WINDOWS side (webcam + COM port don't reach into WSL), 10 Hz
wall time — in the real world, wall time IS sim time.

Deps (install in the sim-to-real session, torch is a big download):
    pip install pyserial opencv-python stable-baselines3

Model: copy the deployment candidate out of WSL first, e.g.
    copy \\\\wsl.localhost\\Ubuntu-22.04\\home\\harun\\linefollower_ws\\training\\checkpoints\\ppo_pool_dr_last.zip .

Usage:
    python rl_drive.py --model ppo_pool_dr_last.zip [--port COMx] [--cam 0]
    ESC or q in the preview window = emergency stop + exit.

Pipeline, mirroring line_env.py exactly:
    frame -> grayscale -> resize 64x64 -> (64,64,1) uint8 -> policy
    action a in [-1,1] per wheel  ->  PWM = (a+1)/2 * 255   (forward-only,
    same mapping as training: (a+1)/2 * WHEEL_MAX)
The Arduino's deadband remap handles the cheap-motor threshold; run the
teleop calibration wizard once before first policy attempt.
"""
import argparse
import sys
import time

import numpy as np

try:
    import cv2
    import serial
    import serial.tools.list_ports
    from stable_baselines3 import PPO
except ImportError as e:
    sys.exit(f"missing dep ({e.name}): pip install pyserial opencv-python stable-baselines3")

CONTROL_HZ = 10.0


def find_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "ch340" in desc or "arduino" in desc or "usb-serial" in desc:
            return p.device
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", required=True)
    ap.add_argument("--port", default=None)
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--scale", type=float, default=1.0,
                    help="global speed scale 0..1 for careful first runs")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("no Arduino found - pass --port COMx")

    model = PPO.load(args.model)
    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        sys.exit(f"webcam {args.cam} did not open")
    ser = serial.Serial(port, 115200, timeout=0.05)
    time.sleep(2.0)  # Uno resets on port open

    print(f"driving via {port}, cam {args.cam}, scale {args.scale} - ESC to stop")
    period = 1.0 / CONTROL_HZ
    try:
        while True:
            t0 = time.time()
            ok, frame = cap.read()
            if not ok:
                print("camera frame failed - stopping")
                break
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            obs = cv2.resize(gray, (64, 64), interpolation=cv2.INTER_AREA)
            obs = obs.reshape(64, 64, 1)

            action, _ = model.predict(obs, deterministic=True)
            # same action->speed mapping as line_env._publish_wheels
            pwm_l = int(np.clip((action[0] + 1) / 2 * 255 * args.scale, 0, 255))
            pwm_r = int(np.clip((action[1] + 1) / 2 * 255 * args.scale, 0, 255))
            ser.write(f"L:{pwm_l} R:{pwm_r}\n".encode())

            view = cv2.resize(obs, (256, 256), interpolation=cv2.INTER_NEAREST)
            cv2.putText(view, f"L{pwm_l} R{pwm_r}", (6, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, 255, 1)
            cv2.imshow("policy view (what the robot sees)", view)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord("q")):
                break
            time.sleep(max(0.0, period - (time.time() - t0)))
    finally:
        ser.write(b"S\n")
        ser.close()
        cap.release()
        cv2.destroyAllWindows()
        print("stopped.")


if __name__ == "__main__":
    main()
