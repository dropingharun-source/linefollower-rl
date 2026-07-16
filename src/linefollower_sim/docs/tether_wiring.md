# Phase 4.5 tether wiring (build night cheat sheet)

Architecture (locked 2026-07-15): **cable, not WiFi.**

```
webcam ──USB──> laptop (policy) ──USB serial──> Arduino Uno ──PWM──> L298N ──> 2x TT motor
                                                                       ^
                                                          6xAA pack (9 V) ──┘
```

## Wiring, step by step

**Battery → L298N (the muscle circuit):**
| From | To |
|---|---|
| 6×AA pack **+** (red) | L298N `+12V` / `VMS` screw terminal |
| 6×AA pack **−** (black) | L298N `GND` screw terminal |

Leave the L298N's **5V-EN jumper ON** — with 9 V in, the board makes its own
logic 5 V. Do **not** feed the L298N's 5V terminal from the Arduino while the
jumper is on.

**Arduino → L298N (the nerve circuit):**
| Arduino pin | L298N pin | Role |
|---|---|---|
| 5 | ENA | left motor speed (PWM) — **remove the ENA jumper cap first** |
| 4 | IN1 | left direction |
| 2 | IN2 | left direction |
| 6 | ENB | right motor speed (PWM) — **remove the ENB jumper cap first** |
| 7 | IN3 | right direction |
| 8 | IN4 | right direction |
| **GND** | **GND** | ⚠ **COMMON GROUND — the #1 forgotten wire.** Without it the control signals are noise. |

**Motors:** left motor → OUT1/OUT2, right motor → OUT3/OUT4. Solder the leads
to the motor tabs (twisted wires fall off a driving robot). Polarity is a
coin flip — fix it in the teleop test below, don't agonize now.

**Arduino → laptop:** plain USB (this powers the Arduino too — no Vin, ever).

## First-run order (film from step 3)

1. **Flash** `real/firmware/tether/tether.ino` (Arduino IDE or arduino-cli,
   board = Uno). Open serial monitor at 115200 → should say `TETHER READY`;
   type `P` → `PONG`.
2. **Wheels-off-ground test:** prop the chassis up.
   `python real/teleop_serial.py` → press `w`. Both wheels forward?
   - A wheel spins **backward** → swap that motor's two wires at OUT terminals
     (or its solder tabs). This is normal, not a bug.
   - A wheel doesn't spin → check its EN jumper cap was removed, then GND.
3. **Deadband wizard:** press `c` in teleop, follow prompts. Saves to the
   Arduino's EEPROM. Write the two numbers in the session log.
4. **First motion on the floor** — OBS + phone rolling BEFORE you press `w`.
   This is roadmap item "teleop the real robot", the un-refilmable one.
5. Policy driving (`rl_drive.py`) is the NEXT session — it needs torch
   installed on Windows (big download) and the deadband numbers from step 3.
   Start it with `--scale 0.5` for the first attempt.

## Safety facts

- Firmware stops motors after **500 ms of serial silence** (yanked tether,
  crashed script). Teleop resends every 200 ms, so this never fires in
  normal driving.
- `space` in teleop = instant stop; `q` stops then quits.
- Motors + laptop share no power — battery drives muscles, USB drives brain.
- If a motor stalls (blocked wheel) for long, the L298N heatsink gets HOT —
  it's on standoffs, not touching the fomiks, right?
