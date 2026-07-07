/*
  =========================================================
   ESP32 MAZE-SOLVING ROBOT  (SLOW MODE / 30x30 cm TILES)
   DISTANCE-DRIVEN NAVIGATION VARIANT
  =========================================================
  Hardware:
    - ESP32 DevKit (38-pin)
    - L298N motor driver + 2 DC motors
    - 3x HC-SR04 ultrasonic sensors (Front, Left, Right)
    - TCS3200 color sensor (detects BLACK and BLUE)
    - Blue LED   -> lights up when BLUE is detected (robot keeps moving)
    - Buzzer     -> beeps when BLACK is detected

  NAVIGATION LOGIC IN THIS VERSION (per request - speeds UNCHANGED):
    1. Poll FRONT sensor. If distance > FRONT_APPROACH_CM (15 cm),
       creep forward, re-checking the front sensor every loop,
       until the front distance drops to <= FRONT_APPROACH_CM.
       (This replaces the old fixed-time "moveForwardTimed" nudge
       with an actual distance-feedback approach, since a timed
       move can't guarantee stopping at a specific distance.)
    2. Once within 15 cm of the front wall, stop and check RIGHT.
       - If dRight < RIGHT_OPEN_CM (20 cm) -> right is blocked,
         do NOT turn right. Fall through to step 3.
       - If dRight >= RIGHT_OPEN_CM (open) -> turn right 90 deg.
         (Not explicitly stated in the request, but implied as the
         natural complement to "don't go right if blocked" -
         change this block if you intended something different.)
    3. Check LEFT.
       - If dLeft > LEFT_OPEN_CM (20 cm) -> turn left 90 deg.
       - Else (both right and left are blocked, and front is also
         blocked) -> dead end -> U-turn. (Added as a fallback so
         the robot doesn't stall forever; remove uTurn() call in
         solveMaze() if you don't want this behavior.)

  !!! ALL TIMING / DISTANCE VALUES BELOW ARE STARTING ESTIMATES !!!
  Motor stall torque, battery voltage, floor friction and wheel
  diameter all affect the real numbers -- you MUST re-run the
  TURN calibration step (see bottom of file) on your actual
  competition surface. Speeds (BASE_SPEED / TURN_SPEED) and turn
  timing (TURN_90_TIME_MS / UTURN_TIME_MS) are UNCHANGED from the
  previous version, as requested.

  !!! YOU MUST CALIBRATE BEFORE RUNNING !!!
    1. Open Serial Monitor at 115200 baud.
    2. Uncomment CALIBRATE_COLOR / CALIBRATE_TURNS blocks below
       (see comments) to tune thresholds and turn timing for
       YOUR robot, surface, and lighting.
  =========================================================
*/

// ---------------- MOTOR DRIVER (L298N) ----------------
#define ENA 32   // PWM - Left motor speed
#define IN1 33   // Left motor direction
#define IN2 25
#define IN3 26   // Right motor direction
#define IN4 27
#define ENB 14   // PWM - Right motor speed

const int PWM_FREQ = 1000;
const int PWM_RES  = 8;     // 8-bit -> 0-255

// ---------------- ULTRASONIC SENSORS ----------------
#define TRIG_FRONT 5
#define ECHO_FRONT 34
#define TRIG_LEFT  4
#define ECHO_LEFT  17
#define TRIG_RIGHT 13
#define ECHO_RIGHT 35

// ---------------- TCS3200 COLOR SENSOR ----------------
#define S0 18
#define S1 19
#define S2 21
#define S3 22
#define OUT_PIN 23

// ---------------- INDICATORS ----------------
#define BLUE_LED 2
#define BUZZER   15

// =========================================================
//                    TUNABLE PARAMETERS
// =========================================================
// --- Tile / corridor geometry: competition tiles are 30 x 30 cm ---
const float TILE_SIZE_CM = 30.0;

// --- Navigation distance thresholds (per your requested workflow) ---
int FRONT_APPROACH_CM = 15;   // creep forward until front reads <= this
int RIGHT_OPEN_CM     = 20;   // right considered "open" if >= this
int LEFT_OPEN_CM      = 20;   // left considered "open" if > this

// Old generic wall/front thresholds kept for reference / calibration guide below
int WALL_DIST_CM   = 22;
int FRONT_STOP_CM  = 12;

// --- SPEED: UNCHANGED, as requested ---
int BASE_SPEED     = 90;   // forward speed      (0-255)
int TURN_SPEED     = 70;   // pivot turn speed   (0-255)

// Timing - UNCHANGED, as requested. Still needs on-surface calibration.
unsigned long TURN_90_TIME_MS   = 360;   // time to pivot ~90 deg - CALIBRATE
unsigned long NUDGE_FWD_TIME_MS = 550;   // (unused by new forward-approach logic, kept for turnLeft90/turnRight90)
unsigned long UTURN_TIME_MS     = 1550;  // time to turn ~180 deg  - CALIBRATE

// Safety cap so the "creep forward" loop can't run away forever if a
// sensor glitches (e.g. always returns -1 / out of range). Distance
// is in cm of travel time estimated at BASE_SPEED - purely a safety
// backstop, not a precision measurement.
unsigned long FORWARD_APPROACH_TIMEOUT_MS = 4000;

// Color thresholds (pulseIn microseconds - LOWER = MORE light of that color)
// CALIBRATE with the sketch mentioned at the bottom of this file.
int BLACK_SUM_THRESHOLD = 1000;   // R+G+B pulse widths summed; higher => darker
int BLUE_DOMINANCE_GAP  = 10;    // blue must be this much "brighter"(lower) than red & green

// =========================================================

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcAttach(ENA, PWM_FREQ, PWM_RES);   // ESP32 core 3.x API
  ledcAttach(ENB, PWM_FREQ, PWM_RES);

  // Ultrasonic pins
  pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
  pinMode(TRIG_LEFT,  OUTPUT); pinMode(ECHO_LEFT,  INPUT);
  pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

  // Color sensor pins
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(OUT_PIN, INPUT);
  digitalWrite(S0, LOW); digitalWrite(S1, HIGH); // 2% frequency scaling (slower, more stable)

  // Indicators
  pinMode(BLUE_LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(BUZZER, LOW);

  stopMotors();
  delay(1500); // settle time after power-up
}

void loop() {
  handleColor();   // check floor color -> LED / buzzer
  solveMaze();      // navigate using ultrasonics
}

// =========================================================
//                  ULTRASONIC DISTANCE
// =========================================================
long getDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000UL); // 25ms timeout ~= 4m
  if (duration == 0) return -1;  // no echo => treat as "open / out of range"

  long distance = duration * 0.0343 / 2.0;  // cm
  return distance;
}

// =========================================================
//                     MOTOR CONTROL
// =========================================================
void setLeftMotor(int speed, bool forward) {
  digitalWrite(IN1, forward ? HIGH : LOW);
  digitalWrite(IN2, forward ? LOW  : HIGH);
  ledcWrite(ENA, constrain(speed, 0, 255));
}

void setRightMotor(int speed, bool forward) {
  digitalWrite(IN3, forward ? HIGH : LOW);
  digitalWrite(IN4, forward ? LOW  : HIGH);
  ledcWrite(ENB, constrain(speed, 0, 255));
}

void stopMotors() {
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

void driveForward(int speed) {
  setLeftMotor(speed, true);
  setRightMotor(speed, true);
}

void pivotRight(int speed) {
  setLeftMotor(speed, true);
  setRightMotor(speed, false);
}

void pivotLeft(int speed) {
  setLeftMotor(speed, false);
  setRightMotor(speed, true);
}

void moveForwardTimed(unsigned long ms) {
  driveForward(BASE_SPEED);
  delay(ms);
  stopMotors();
}

// NEW: distance-driven forward creep. Drives forward, re-polling the
// front sensor, until the front distance is <= stopCM (or a safety
// timeout elapses in case of a sensor glitch).
void moveForwardUntilDistance(int stopCM) {
  unsigned long startTime = millis();
  driveForward(BASE_SPEED);

  while (true) {
    long dFront = getDistanceCM(TRIG_FRONT, ECHO_FRONT);
    Serial.printf("[approach] F:%ld\n", dFront);

    // dFront == -1 means "no echo / out of range", i.e. still open -
    // keep creeping forward.
    if (dFront != -1 && dFront <= stopCM) {
      break;
    }
    if (millis() - startTime > FORWARD_APPROACH_TIMEOUT_MS) {
      Serial.println("[approach] timeout - stopping as safety fallback");
      break;
    }
    delay(40); // small settle delay between ultrasonic pings
  }
  stopMotors();
}

void turnRight90() {
  pivotRight(TURN_SPEED);
  delay(TURN_90_TIME_MS);
  stopMotors();
}

void turnLeft90() {
  pivotLeft(TURN_SPEED);
  delay(TURN_90_TIME_MS);
  stopMotors();
}

void uTurn() {
  pivotRight(TURN_SPEED);
  delay(UTURN_TIME_MS);
  stopMotors();
}

// =========================================================
//                  MAZE-SOLVING LOGIC
//   (Front-approach, then right-check, then left-check)
// =========================================================

void executeEscape() {
  Serial.println("[ESCAPE] Too close or jammed! Backing up...");

  stopMotors();
  delay(100);
  setLeftMotor(BASE_SPEED, false);
  setRightMotor(BASE_SPEED, false);
  delay(500);
  stopMotors();
  delay(100);

  long dLeft = getDistanceCM(TRIG_LEFT, ECHO_LEFT);
  long dRight = getDistanceCM(TRIG_RIGHT, ECHO_RIGHT);

  if (dLeft > 0 && dLeft < dRight) {
    pivotRight(TURN_SPEED);
    delay(300);
  } else {
    pivotLeft(TURN_SPEED);
    delay(300);
  }
}
void solveMaze() {
  long checkFront = getDistanceCM(TRIG_FRONT, ECHO_FRONT);
  long checkLeft = getDistanceCM(TRIG_LEFT, ECHO_LEFT);
  long checkRight = getDistanceCM(TRIG_RIGHT, ECHO_RIGHT);

  if ((checkFront > 0 && checkFront < 6) || (checkLeft > 0 && checkLeft < 6) || (checkRight > 0 && checkRight < 6)) {
    executeEscape();
    return;
  }

  long dFront = getDistanceCM(TRIG_FRONT, ECHO_FRONT);
  Serial.printf("[solveMaze] initial F:%ld\n",dFront);

  if (dFront > FRONT_APPROACH_CM) {
    moveForwardUntilDistance(FRONT_APPROACH_CM);
  }
  else if (dFront == -1) {
    moveForwardTimed(150);
  }

  delay(60);
  long dRight = getDistanceCM(TRIG_RIGHT, ECHO_RIGHT);

  bool rightBlocked = (dRight > 0) && (dRight < RIGHT_OPEN_CM);
  Serial.printf("[solveMaze] R:%ld blocked:%d\n", dRight, rightBlocked);

  if (!rightBlocked) {
    turnRight90();
    delay(200);
    return;
  }

  delay(60);
  long dLeft = getDistanceCM(TRIG_LEFT, ECHO_LEFT);
  bool leftOpen = (dLeft > LEFT_OPEN_CM) || (dLeft == -1);
  Serial.printf("[solveMaze] L:%ld open:%d\n", dLeft, leftOpen);

  if (leftOpen) {
    turnLeft90();
    delay(200);
    return;
  }

  Serial.println("[solveMaze] dead end - U turn");
  uTurn();
  delay(200);
}

// =========================================================
//                   COLOR DETECTION (TCS3200)
// =========================================================
// Returns pulse width (microseconds). Lower value = MORE light
// reflected of that color (higher frequency output).
long readColorPulse(bool s2, bool s3) {
  digitalWrite(S2, s2 ? HIGH : LOW);
  digitalWrite(S3, s3 ? HIGH : LOW);
  delayMicroseconds(50);
  long pulse = pulseIn(OUT_PIN, LOW, 25000UL); // measure LOW pulse width
  if (pulse == 0) pulse = 25000; // timeout -> treat as "no light" (very dark)
  return pulse;
}

void handleColor() {
  long red   = readColorPulse(LOW,  LOW);   // S2=L,S3=L -> Red filter
  long blue  = readColorPulse(LOW,  HIGH);  // S2=L,S3=H -> Blue filter
  long green = readColorPulse(HIGH, HIGH);  // S2=H,S3=H -> Green filter

  long sum = red + green + blue;
  bool isBlack = (sum > BLACK_SUM_THRESHOLD * 3); // all channels dark
  bool isBlue  = (!isBlack) &&
                 (blue < red - BLUE_DOMINANCE_GAP) &&
                 (blue < green - BLUE_DOMINANCE_GAP);

  // Uncomment for calibration:
  // Serial.printf("R:%ld G:%ld B:%ld  sum:%ld\n", red, green, blue, sum);

  if (isBlue) {
    digitalWrite(BLUE_LED, HIGH);
    noTone(BUZZER);
    digitalWrite(BUZZER, LOW);
  } else if (isBlack) {
    digitalWrite(BLUE_LED, LOW);
    tone(BUZZER, 2000, 150); // short beep
  } else {
    digitalWrite(BLUE_LED, LOW);
    digitalWrite(BUZZER, LOW);
  }
}

/*
  =========================================================
   CALIBRATION GUIDE
  =========================================================
  1. COLOR SENSOR:
     - Flash the sketch, open Serial Monitor (115200 baud).
     - Uncomment the Serial.printf line inside handleColor().
     - Hold the sensor ~1cm above WHITE floor, note R/G/B/sum.
     - Hold over BLACK marker, note sum -> set BLACK_SUM_THRESHOLD
       to roughly halfway between white-sum and black-sum (divided by 3).
     - Hold over BLUE marker, confirm blue value is clearly lower
       than red/green -> adjust BLUE_DOMINANCE_GAP accordingly.

  2. TURNS:
     - Place robot in open space.
     - Call turnRight90() / turnLeft90() alone (via Serial command
       or temporarily in setup()) and adjust TURN_90_TIME_MS until
       robot turns as close to 90 degrees as possible.
     - Do the same for UTURN_TIME_MS (~180 degrees).
     - If the robot barely creeps or stalls at BASE_SPEED/TURN_SPEED,
       raise both by small steps (5-10) - there is a minimum PWM
       below which the motors won't overcome static friction, and
       that minimum varies robot to robot.

  3. NAVIGATION DISTANCES (30 cm x 30 cm tiles):
     - FRONT_APPROACH_CM (15 cm): distance from the front wall at
       which the robot should stop creeping forward. Watch the
       Serial log during a test run and confirm it stops roughly
       15 cm short, not right up against the wall or way short of it.
     - RIGHT_OPEN_CM / LEFT_OPEN_CM (20 cm): threshold that separates
       "wall present" from "opening" on a 30 cm tile. If the robot
       misjudges a real wall as open (or vice versa), nudge this up
       or down - sensor mounting angle/offset from center matters here.
     - FORWARD_APPROACH_TIMEOUT_MS: safety-only backstop in case a
       sensor glitches and never reports <= FRONT_APPROACH_CM. Not a
       precision distance value - just make sure it's long enough to
       cross the longest straight corridor in your maze at BASE_SPEED.
  =========================================================
*/
