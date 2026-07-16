#!/usr/bin/env python3
"""Keyboard teleop for the tethered robot (Windows console) + deadband wizard.

First-motion test for Phase 4.5: drive the real robot over the USB tether
before the policy ever touches it. Film it.

    pip install pyserial
    python teleop_serial.py            # auto-detects the CH340/Uno port
    python teleop_serial.py --port COM5

Keys:
    w / x    forward / reverse (both wheels at current speed)
    a / d    pivot left / right
    space    STOP
    [ / ]    speed -20 / +20 PWM
    c        deadband calibration wizard (per motor, saves to Arduino EEPROM)
    g        show deadbands stored on the Arduino
    q        quit (stops motors first)

The firmware stops the motors after 500 ms of serial silence, so this
script resends the current command every 200 ms — hold nothing, the last
key sticks until space or a new key.
"""
import argparse
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("pyserial missing:  pip install pyserial")

try:
    import msvcrt  # Windows-only console keys; fine — the laptop is Windows
except ImportError:
    sys.exit("this teleop uses msvcrt (Windows console). Run it on Windows.")

RESEND_S = 0.2


def find_port():
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if "ch340" in desc or "arduino" in desc or "usb-serial" in desc:
            return p.device
    return None


def send(ser, line):
    ser.write((line + "\n").encode())


def read_replies(ser):
    while ser.in_waiting:
        reply = ser.readline().decode(errors="replace").strip()
        if reply:
            print(f"  <- {reply}")


def calibrate(ser):
    """Ramp one motor at a time; you hit SPACE the moment the wheel turns."""
    print("\n=== DEADBAND WIZARD ===")
    print("Wheels OFF the ground (prop the chassis up).")
    print("For each motor: PWM ramps up slowly - press SPACE the moment")
    print("the wheel starts to turn. Esc aborts.\n")
    dead = {}
    for name, fmt in (("LEFT", "L:{} R:0"), ("RIGHT", "L:0 R:{}")):
        input(f"{name} motor - press Enter to start the ramp...")
        found = None
        for pwm in range(20, 200, 2):
            send(ser, fmt.format(pwm))
            print(f"\r  {name} PWM = {pwm}   ", end="", flush=True)
            t0 = time.time()
            while time.time() - t0 < 0.15:
                if msvcrt.kbhit():
                    k = msvcrt.getch()
                    if k == b" ":
                        found = pwm
                    elif k == b"\x1b":
                        send(ser, "S")
                        print("\naborted")
                        return
                    break
            if found:
                break
        send(ser, "S")
        if not found:
            print(f"\n{name}: no motion by PWM 200 - check wiring/battery!")
            return
        dead[name] = max(0, found - 2)
        print(f"\n{name} deadband = {dead[name]}")
    send(ser, "D:{},{}".format(dead["LEFT"], dead["RIGHT"]))
    time.sleep(0.2)
    read_replies(ser)
    print("Saved to Arduino EEPROM - survives power cycles.\n")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--speed", type=int, default=140, help="initial PWM")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("no Arduino found - plug in the tether or pass --port COMx")
    ser = serial.Serial(port, args.baud, timeout=0.1)
    time.sleep(2.0)  # Uno resets on serial open; wait for TETHER READY
    read_replies(ser)
    send(ser, "P")
    time.sleep(0.3)
    read_replies(ser)

    speed = args.speed
    cmd = "S"
    print(__doc__.split("Keys:")[1].split("The firmware")[0])
    print(f"connected on {port}, speed={speed}. Drive!")

    last = 0.0
    while True:
        if msvcrt.kbhit():
            k = msvcrt.getch().lower()
            if k == b"q":
                send(ser, "S")
                break
            elif k == b"w":
                cmd = f"L:{speed} R:{speed}"
            elif k == b"x":
                cmd = f"L:{-speed} R:{-speed}"
            elif k == b"a":
                cmd = f"L:0 R:{speed}"
            elif k == b"d":
                cmd = f"L:{speed} R:0"
            elif k == b" ":
                cmd = "S"
            elif k == b"]":
                speed = min(255, speed + 20)
                print(f"speed={speed}")
            elif k == b"[":
                speed = max(40, speed - 20)
                print(f"speed={speed}")
            elif k == b"g":
                send(ser, "G")
            elif k == b"c":
                calibrate(ser)
                cmd = "S"
            send(ser, cmd)
            last = time.time()
            print(f"\r-> {cmd}      ", end="", flush=True)
        if time.time() - last > RESEND_S:
            send(ser, cmd)
            last = time.time()
        read_replies(ser)
        time.sleep(0.01)


if __name__ == "__main__":
    main()
