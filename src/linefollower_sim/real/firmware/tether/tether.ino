// tether.ino — Phase 4.5 tether firmware (written 2026-07-16, pre-hardware).
//
// The robot's "hands": the laptop runs the policy and sends wheel commands
// over USB serial; this sketch turns them into L298N PWM. The Jetson later
// replaces the laptop and this file keeps working unchanged.
//
//   laptop --USB serial--> Arduino Uno --PWM--> L298N --> 2x TT motor
//
// Protocol: newline-terminated ASCII at 115200 baud.
//   "L:<pwm> R:<pwm>"  set both wheels, pwm in [-255..255], negative = reverse
//   "S"                stop both motors
//   "D:<l>,<r>"        set per-motor deadbands (saved to EEPROM)
//   "G"                print current deadbands
//   "P"                replies "PONG" (connection test)
//
// Deadband: cheap TT motors hum but don't turn below some PWM threshold —
// the predicted #1 sim-to-real gap. Requested speeds are remapped so that
// any nonzero command starts at the motor's real threshold:
//   out = dead + v * (255 - dead) / 255      (v = requested 1..255)
// Measure thresholds with teleop_serial.py's calibration wizard ('c'),
// which stores them here via "D:" so calibration survives power cycles.
//
// Safety: if no valid command arrives for 500 ms the motors stop. The
// laptop side resends continuously, so silence means an unplugged tether,
// a crashed driver, or a wedged serial port — all cases where a robot
// running its last command off the track is the wrong outcome.

#include <EEPROM.h>

// L298N pin map (ENA/ENB must be PWM-capable pins)
const int ENA = 5;   // left motor PWM
const int IN1 = 4;   // left motor direction
const int IN2 = 2;
const int ENB = 6;   // right motor PWM
const int IN3 = 7;   // right motor direction
const int IN4 = 8;

const unsigned long TIMEOUT_MS = 500;
const byte EEPROM_MAGIC = 0x42;  // addr 0; deadbands live at addr 1 and 2

int deadL = 0;
int deadR = 0;
unsigned long lastCmdMs = 0;
bool moving = false;

char buf[32];
byte bufLen = 0;

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();
  if (EEPROM.read(0) == EEPROM_MAGIC) {
    deadL = EEPROM.read(1);
    deadR = EEPROM.read(2);
  }
  Serial.begin(115200);
  Serial.println("TETHER READY");
}

void stopMotors() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  moving = false;
}

int remap(int v, int dead) {
  // 0 stays 0; 1..255 maps linearly onto dead..255
  if (v == 0) return 0;
  return dead + (int)((long)v * (255 - dead) / 255);
}

void driveMotor(int pwm, int dead, int en, int inA, int inB) {
  int mag = remap(min(abs(pwm), 255), dead);
  digitalWrite(inA, pwm > 0 ? HIGH : LOW);
  digitalWrite(inB, pwm > 0 ? LOW : (pwm < 0 ? HIGH : LOW));
  analogWrite(en, mag);
}

void drive(int l, int r) {
  driveMotor(l, deadL, ENA, IN1, IN2);
  driveMotor(r, deadR, ENB, IN3, IN4);
  moving = (l != 0 || r != 0);
}

void handleLine(char* s) {
  if (s[0] == 'P') { Serial.println("PONG"); lastCmdMs = millis(); return; }
  if (s[0] == 'S') { stopMotors(); lastCmdMs = millis(); return; }
  if (s[0] == 'G') {
    Serial.print("DEAD "); Serial.print(deadL);
    Serial.print(" "); Serial.println(deadR);
    lastCmdMs = millis(); return;
  }
  if (s[0] == 'D' && s[1] == ':') {
    char* comma = strchr(s + 2, ',');
    if (!comma) { Serial.println("ERR"); return; }
    deadL = constrain(atoi(s + 2), 0, 254);
    deadR = constrain(atoi(comma + 1), 0, 254);
    EEPROM.update(0, EEPROM_MAGIC);
    EEPROM.update(1, (byte)deadL);
    EEPROM.update(2, (byte)deadR);
    Serial.print("DEAD "); Serial.print(deadL);
    Serial.print(" "); Serial.println(deadR);
    lastCmdMs = millis(); return;
  }
  if (s[0] == 'L' && s[1] == ':') {
    char* rpart = strstr(s, "R:");
    if (!rpart) { Serial.println("ERR"); return; }
    int l = constrain(atoi(s + 2), -255, 255);
    int r = constrain(atoi(rpart + 2), -255, 255);
    drive(l, r);
    lastCmdMs = millis(); return;
  }
  Serial.println("ERR");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (bufLen > 0) { buf[bufLen] = '\0'; handleLine(buf); bufLen = 0; }
    } else if (bufLen < sizeof(buf) - 1) {
      buf[bufLen++] = c;
    } else {
      bufLen = 0;  // oversized garbage — drop the line
    }
  }
  if (moving && millis() - lastCmdMs > TIMEOUT_MS) {
    stopMotors();
    Serial.println("TIMEOUT");
  }
}
