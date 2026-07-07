# ESP32 Maze-Solving Robot

A distance-driven, wall-following maze solver built on the ESP32, designed for 30x30 cm competition maze tiles. Built with a group of juniors from my old school for their IT Fest.

## Overview

The robot navigates a maze using three ultrasonic sensors (front, left, right) and a wall-following strategy: creep forward toward the front wall, then prefer turning right, then left, and U-turn if it's a dead end. A color sensor on the floor detects black and blue markers, triggering a buzzer or LED respectively.

This is a **starting point**, not a finished competition-ready build — timing and distance thresholds need to be calibrated on the actual robot and surface before use.

## Hardware

| Component | Purpose |
|---|---|
| ESP32 DevKit (38-pin) | Main controller |
| L298N motor driver + 2 DC motors | Drive system |
| 3x HC-SR04 ultrasonic sensors | Front / Left / Right wall detection |
| TCS3200 color sensor | Detects BLACK and BLUE floor markers |
| Blue LED | Lights up on BLUE detection |
| Buzzer | Beeps on BLACK detection |

### Pin Mapping

**Motor driver (L298N)**
- `ENA` 32, `IN1` 33, `IN2` 25 — Left motor
- `ENB` 14, `IN3` 26, `IN4` 27 — Right motor

**Ultrasonic sensors**
- Front: `TRIG` 5, `ECHO` 34
- Left: `TRIG` 4, `ECHO` 17
- Right: `TRIG` 13, `ECHO` 35

**Color sensor (TCS3200)**
- `S0` 18, `S1` 19, `S2` 21, `S3` 22, `OUT` 23

**Indicators**
- Blue LED: 2
- Buzzer: 15

## Navigation Logic

1. **Approach front wall** — creep forward, re-checking the front sensor every loop, until the distance drops to ≤ `FRONT_APPROACH_CM` (15 cm). A safety timeout stops the loop if a sensor glitches.
2. **Check right** — if the right side is open (≥ `RIGHT_OPEN_CM`, 20 cm), turn right 90°.
3. **Check left** — if right was blocked, check left. If open (> `LEFT_OPEN_CM`, 20 cm), turn left 90°.
4. **Dead end** — if front, right, and left are all blocked, perform a U-turn.
5. **Emergency escape** — if any sensor reads dangerously close (< 6 cm), the robot backs up and pivots away from the tighter side before resuming normal logic.

Floor color detection runs independently every loop: BLUE lights the LED, BLACK triggers a short buzzer beep.

## ⚠️ Calibration Required

All distance thresholds, speeds, and turn timings in this sketch are **starting estimates**. Motor torque, battery voltage, floor friction, and wheel diameter all vary by robot — you must calibrate on your actual robot and competition surface before relying on it.

### 1. Color Sensor
1. Flash the sketch and open Serial Monitor at `115200` baud.
2. Uncomment the `Serial.printf` line inside `handleColor()`.
3. Hold the sensor ~1 cm above a white floor and note the R/G/B/sum values.
4. Hold over a black marker, note the sum, and set `BLACK_SUM_THRESHOLD` to roughly halfway between the white and black sums (divided by 3).
5. Hold over a blue marker and confirm blue reads clearly lower than red/green; adjust `BLUE_DOMINANCE_GAP` if needed.

### 2. Turns
1. Place the robot in open space.
2. Call `turnRight90()` / `turnLeft90()` in isolation and adjust `TURN_90_TIME_MS` until the turn is as close to 90° as possible.
3. Repeat for `UTURN_TIME_MS` (~180°).
4. If the robot stalls or barely moves at `BASE_SPEED` / `TURN_SPEED`, raise both in small steps (5–10) — there's a minimum PWM below which the motors can't overcome static friction, and it varies by robot.

### 3. Navigation Distances (30x30 cm tiles)
- `FRONT_APPROACH_CM` (15 cm): distance from the front wall where the robot should stop creeping forward. Watch the Serial log during a test run to confirm it stops roughly there.
- `RIGHT_OPEN_CM` / `LEFT_OPEN_CM` (20 cm): threshold separating "wall" from "opening" on a 30 cm tile. Adjust based on sensor mounting angle/offset.
- `FORWARD_APPROACH_TIMEOUT_MS`: safety-only backstop in case a sensor never reports a close-enough distance. Not a precision value — just make sure it's long enough to cross the longest straight corridor at `BASE_SPEED`.

## Tunable Parameters

```cpp
// Navigation thresholds
FRONT_APPROACH_CM = 15;
RIGHT_OPEN_CM     = 20;
LEFT_OPEN_CM      = 20;

// Speeds (0-255)
BASE_SPEED = 90;
TURN_SPEED = 70;

// Timing (ms) — calibrate on your surface
TURN_90_TIME_MS   = 360;
UTURN_TIME_MS     = 1550;

// Color detection
BLACK_SUM_THRESHOLD = 1000;
BLUE_DOMINANCE_GAP  = 10;
```

## Known Limitations

- Timed turns (`delay`-based) assume consistent battery voltage and floor friction — they will drift over time and need periodic recalibration.
- `moveForwardTimed()` is a leftover fallback used only when the front sensor returns no echo; it isn't distance-accurate.
- No encoder or IMU feedback — all movement is open-loop aside from the ultrasonic approach logic.
- This is a **wall-following** solver (right-hand-preference), not a mapping/shortest-path algorithm — it won't necessarily find the optimal route.

## Credits

Built with juniors from my old school for their IT Fest maze-solving challenge. A learning project for all of us — not perfect, but a solid base to iterate on.
