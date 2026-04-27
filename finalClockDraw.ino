#include <AccelStepper.h>
#include <ESP32Servo.h>
#include <math.h>

// ============================================================================
// XY CLOCK PLOTTER
// ----------------------------------------------------------------------------
// - Built for the same XY plotter hardware as drawing_machine_websocket.ino
// - No Wi-Fi, no websockets, no internet
// - Homes ONCE at startup
// - Waits for time over Serial at startup
// - Redraws the full clock face every minute
//
// Serial time format:
//   HH:MM
//   HH:MM:SS
// Example:
//   14:37
//   09:15:30
//
// Optional runtime serial commands:
//   T HH:MM[:SS]   -> set time immediately
//   D              -> redraw now
//   H              -> re-home now
//
// IMPORTANT:
// 1) Set SERVO_UP and SERVO_DOWN below.
// 2) Set CLOCK_CENTER_X_MM and CLOCK_CENTER_Y_MM so the clock fits your bed.
// 3) If the clock is vertically flipped, change Y_MM_INCREASES_UPWARD.
// ============================================================================

// ------------------------------- Pinout --------------------------------------
const int xLimit = D7;
const int yLimit = D8;

const int stepPinX = D1;
const int dirPinX  = D6;
const int stepPinY = D3;
const int dirPinY  = D2;

const int servoPin = D5;

// -------------------------- Mechanical calibration ---------------------------
float pulleyDiamX = 0.8;
float pulleyDiamY = 0.8;

const int xMicrosteps = 1;
const int yMicrosteps = 1;
const int stepsPerRev = 200;

const float xMmToSteps = (xMicrosteps * stepsPerRev) / (pulleyDiamX * PI);
const float yMmToSteps = (yMicrosteps * stepsPerRev) / (pulleyDiamY * PI);

// ----------------------------- Motion tuning ---------------------------------
const float BASE_MAX_SPEED   = 3000.0f;
const float BASE_ACCEL       = 3000.0f;
const float HOMING_SPEED     = -1000.0f;
const int   SERVO_MOVE_DELAY = 5;      // ms per degree during pen transition

// ------------------------------ Servo tuning ---------------------------------
// Set these to whatever works for your machine.
const int SERVO_DOWN = 0;
const int SERVO_UP   = 180;

// ------------------------------ Clock layout ---------------------------------
const float CLOCK_RADIUS_MM      = 70.0f;  // 10 cm radius per your request
const float CLOCK_CENTER_X_MM    = 120.0f;  // CHANGE to where you want the clock center
const float CLOCK_CENTER_Y_MM    = 80.0f;  // CHANGE to where you want the clock center
const float TICK_INNER_RADIUS_MM = 20.0f;
const float HOUR_HAND_LENGTH_MM  = 50.0f;
const float MIN_HAND_LENGTH_MM   = 60.0f;
const int   CIRCLE_SEGMENTS      = 12;     // smoother circle, still reasonable

// Axis direction flags.
// Set X_MM_INCREASES_RIGHTWARD to false if positive X commands move toward the
// X home switch instead of away from it.
// Set Y_MM_INCREASES_UPWARD to false if the clock appears vertically flipped.
const bool X_MM_INCREASES_RIGHTWARD = true;
const bool Y_MM_INCREASES_UPWARD = true;

// ---------------------------- Command queue ----------------------------------
enum CommandType : uint8_t {
  CMD_MOVE = 0,
  CMD_PEN  = 1
};

struct Command {
  CommandType type;
  float xMm;
  float yMm;
  bool penDown;
};

const int MAX_COMMANDS = 700;
Command commandQueue[MAX_COMMANDS];
int commandCount = 0;
int commandIndex = 0;

// --------------------------- Global state ------------------------------------
AccelStepper stepperX(AccelStepper::DRIVER, stepPinX, dirPinX);
AccelStepper stepperY(AccelStepper::DRIVER, stepPinY, dirPinY);
Servo servo;

bool isHoming = false;
bool homed = false;
bool setupComplete = false;

bool penIsDown = false;
bool moveInProgress = false;

float plannerX = 0.0f;
float plannerY = 0.0f;
bool plannerPenDown = false;

// Timekeeping from manual serial set
bool timeIsSet = false;
uint32_t baseSecondsOfDay = 0;   // 0..86399
uint32_t baseMillis = 0;
int lastDrawnMinute = -1;
bool forceRedraw = false;

String serialBuffer;

// --------------------------- Utility functions -------------------------------
float degToRad(float deg) {
  return deg * (PI / 180.0f);
}

float polarToX(float radius, float angleDegClock) {
  // Clock face rotated 90° counterclockwise:
  // 12 o'clock points left
  float theta = degToRad(180.0f - angleDegClock);
  return CLOCK_CENTER_X_MM + radius * cos(theta);
}

float polarToY(float radius, float angleDegClock) {
  float theta = degToRad(180.0f - angleDegClock);
  float dy = radius * sin(theta);

  return Y_MM_INCREASES_UPWARD ? (CLOCK_CENTER_Y_MM + dy)
                               : (CLOCK_CENTER_Y_MM - dy);
}

long mmToStepsX(float mm) {
  long steps = lroundf(mm * xMmToSteps);
  return X_MM_INCREASES_RIGHTWARD ? steps : -steps;
}

long mmToStepsY(float mm) {
  return lroundf(mm * yMmToSteps);
}

void clearCommands() {
  commandCount = 0;
  commandIndex = 0;
  plannerX = 0.0f;
  plannerY = 0.0f;
  plannerPenDown = false;
}

bool enqueueMove(float xMm, float yMm) {
  if (commandCount >= MAX_COMMANDS) return false;
  commandQueue[commandCount++] = {CMD_MOVE, xMm, yMm, false};
  plannerX = xMm;
  plannerY = yMm;
  return true;
}

bool enqueuePen(bool down) {
  if (commandCount >= MAX_COMMANDS) return false;
  commandQueue[commandCount++] = {CMD_PEN, 0.0f, 0.0f, down};
  plannerPenDown = down;
  return true;
}

bool ensurePenUpPlanned() {
  if (!plannerPenDown) return true;
  return enqueuePen(false);
}

bool ensurePenDownPlanned() {
  if (plannerPenDown) return true;
  return enqueuePen(true);
}

bool travelTo(float xMm, float yMm) {
  if (!ensurePenUpPlanned()) return false;
  return enqueueMove(xMm, yMm);
}

bool lineTo(float xMm, float yMm) {
  if (!ensurePenDownPlanned()) return false;
  return enqueueMove(xMm, yMm);
}

bool finishPath() {
  return ensurePenUpPlanned();
}

void applyCoordinatedMove(float targetXmm, float targetYmm) {
  long targetXSteps = mmToStepsX(targetXmm);
  long targetYSteps = mmToStepsY(targetYmm);

  long dx = labs(targetXSteps - stepperX.currentPosition());
  long dy = labs(targetYSteps - stepperY.currentPosition());

  if (dx == 0 && dy == 0) {
    moveInProgress = false;
    return;
  }

  float scaleX = 1.0f;
  float scaleY = 1.0f;

  if (dx == 0) {
    scaleX = 0.0f;
    scaleY = 1.0f;
  } else if (dy == 0) {
    scaleX = 1.0f;
    scaleY = 0.0f;
  } else if (dx >= dy) {
    scaleX = 1.0f;
    scaleY = (float)dy / (float)dx;
  } else {
    scaleX = (float)dx / (float)dy;
    scaleY = 1.0f;
  }

  stepperX.setAcceleration(BASE_ACCEL * scaleX);
  stepperX.setMaxSpeed(BASE_MAX_SPEED * scaleX);
  stepperY.setAcceleration(BASE_ACCEL * scaleY);
  stepperY.setMaxSpeed(BASE_MAX_SPEED * scaleY);

  stepperX.moveTo(targetXSteps);
  stepperY.moveTo(targetYSteps);
  moveInProgress = true;
}

int lastServoPos = SERVO_UP;   // global variable

void moveServoSmoothly(int targetPos) {
  int degreesToMove = abs(targetPos - lastServoPos);

  servo.write(targetPos);

  // tune this number:
  int msPerDegree = 20;

  delay(degreesToMove * msPerDegree);

  lastServoPos = targetPos;
}

void penUpNow() {
  if (!penIsDown) return;
  Serial.println("PEN UP");
  moveServoSmoothly(SERVO_UP);
  penIsDown = false;
}

void penDownNow() {
  if (penIsDown) return;
  Serial.println("PEN DOWN");
  moveServoSmoothly(SERVO_DOWN);
  penIsDown = true;
}

void startHoming() {
  isHoming = true;
  moveInProgress = false;
  stepperX.stop();
  stepperY.stop();
}

bool parseTimeString(const String &input, uint32_t &secondsOut) {
  int h = -1, m = -1, s = 0;

  int firstColon = input.indexOf(':');
  if (firstColon < 0) return false;

  int secondColon = input.indexOf(':', firstColon + 1);

  String hs = input.substring(0, firstColon);
  String ms;
  String ss;

  if (secondColon < 0) {
    ms = input.substring(firstColon + 1);
  } else {
    ms = input.substring(firstColon + 1, secondColon);
    ss = input.substring(secondColon + 1);
  }

  hs.trim();
  ms.trim();
  ss.trim();

  if (hs.length() == 0 || ms.length() == 0) return false;

  h = hs.toInt();
  m = ms.toInt();
  if (secondColon >= 0) s = ss.toInt();

  if (h < 0 || h > 23 || m < 0 || m > 59 || s < 0 || s > 59) return false;

  secondsOut = (uint32_t)h * 3600UL + (uint32_t)m * 60UL + (uint32_t)s;
  return true;
}

void setClockTime(uint32_t secondsOfDay) {
  baseSecondsOfDay = secondsOfDay % 86400UL;
  baseMillis = millis();
  timeIsSet = true;
  lastDrawnMinute = -1;
  forceRedraw = true;

  uint32_t h = baseSecondsOfDay / 3600UL;
  uint32_t m = (baseSecondsOfDay % 3600UL) / 60UL;
  uint32_t s = baseSecondsOfDay % 60UL;
  Serial.printf("Time set to %02lu:%02lu:%02lu\n", h, m, s);
}

uint32_t currentSecondsOfDay() {
  if (!timeIsSet) return 0;
  uint32_t elapsed = (millis() - baseMillis) / 1000UL;
  return (baseSecondsOfDay + elapsed) % 86400UL;
}

void printTimePrompt() {
  Serial.println();
  Serial.println("Enter current time over Serial.");
  Serial.println("Accepted formats:");
  Serial.println("  HH:MM");
  Serial.println("  HH:MM:SS");
  Serial.println("Examples:");
  Serial.println("  14:37");
  Serial.println("  09:15:30");
  Serial.println();
}

void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  uint32_t seconds = 0;

  if (line.equalsIgnoreCase("D")) {
    forceRedraw = true;
    Serial.println("Redraw requested.");
    return;
  }

  if (line.equalsIgnoreCase("H")) {
    Serial.println("Re-homing requested.");
    clearCommands();
    penUpNow();
    startHoming();
    return;
  }

  if ((line.startsWith("T ") || line.startsWith("t "))) {
    String timePart = line.substring(2);
    if (parseTimeString(timePart, seconds)) {
      setClockTime(seconds);
    } else {
      Serial.println("Invalid time format. Use T HH:MM or T HH:MM:SS");
    }
    return;
  }

  if (parseTimeString(line, seconds)) {
    setClockTime(seconds);
  } else {
    Serial.println("Invalid input. Use HH:MM, HH:MM:SS, T HH:MM[:SS], D, or H");
  }
}

void serviceSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleSerialLine(serialBuffer);
      serialBuffer = "";
    } else {
      serialBuffer += c;
    }
  }
}

// --------------------------- Drawing generation ------------------------------
bool addClockCircle() {
  for (int i = 0; i <= CIRCLE_SEGMENTS; i++) {
    float a = 360.0f * ((float)i / (float)CIRCLE_SEGMENTS);
    float x = polarToX(CLOCK_RADIUS_MM, a);
    float y = polarToY(CLOCK_RADIUS_MM, a);

    if (i == 0) {
      if (!travelTo(x, y)) return false;
    }
    if (!lineTo(x, y)) return false;
  }
  return finishPath();
}

bool addHourTicks() {
  for (int h = 0; h < 12; h++) {
    float a = h * 30.0f;
    float x1 = polarToX(TICK_INNER_RADIUS_MM, a);
    float y1 = polarToY(TICK_INNER_RADIUS_MM, a);
    float x2 = polarToX(CLOCK_RADIUS_MM, a);
    float y2 = polarToY(CLOCK_RADIUS_MM, a);

    if (!travelTo(x1, y1)) return false;
    if (!lineTo(x2, y2)) return false;
    if (!finishPath()) return false;
  }
  return true;
}

bool addHand(float angleDegClock, float lengthMm) {
  float x = polarToX(lengthMm, angleDegClock);
  float y = polarToY(lengthMm, angleDegClock);

  if (!lineTo(x, y)) return false;
  if (!lineTo(CLOCK_CENTER_X_MM, CLOCK_CENTER_Y_MM)) return false;

  return true;
}

bool generateClockCommands(int hour24, int minute) {
  clearCommands();

  // Start from the center with pen up.
  if (!travelTo(CLOCK_CENTER_X_MM, CLOCK_CENTER_Y_MM)) return false;

  if (!addClockCircle()) return false;
  //if (!addHourTicks()) return false; temporarily removed to increase speed of plotting cycle

  float minuteAngle = minute * 6.0f;
  float hourAngle = ((hour24 % 12) * 30.0f) + (minute * 0.5f);

  if (!travelTo(CLOCK_CENTER_X_MM, CLOCK_CENTER_Y_MM)) return false;
  if (!ensurePenDownPlanned()) return false;

  if (!addHand(hourAngle, HOUR_HAND_LENGTH_MM)) return false;
  if (!addHand(minuteAngle, MIN_HAND_LENGTH_MM)) return false;

  if (!ensurePenUpPlanned()) return false;

    return true;
  }

// ----------------------------- Execution -------------------------------------
void processHoming() {
  if (!isHoming) return;

  stepperX.setSpeed(HOMING_SPEED);
  stepperY.setSpeed(HOMING_SPEED);

  while ((digitalRead(xLimit) == HIGH) || (digitalRead(yLimit) == HIGH)) {
    if (digitalRead(xLimit) == HIGH) stepperX.runSpeed();
    if (digitalRead(yLimit) == HIGH) stepperY.runSpeed();
    serviceSerial();
  }

  stepperX.setCurrentPosition(0);
  stepperY.setCurrentPosition(0);
  stepperX.moveTo(0);
  stepperY.moveTo(0);

  plannerX = 0.0f;
  plannerY = 0.0f;
  moveInProgress = false;
  isHoming = false;
  homed = true;

  Serial.println("Homing complete. Current position set to (0,0) mm.");
}

void executeNextCommandIfReady() {
  if (isHoming) return;

  if (moveInProgress) {
    if (stepperX.distanceToGo() == 0 && stepperY.distanceToGo() == 0) {
      moveInProgress = false;
    } else {
      return;
    }
  }

  if (commandIndex >= commandCount) return;

  Command &cmd = commandQueue[commandIndex];
  if (cmd.type == CMD_PEN) {
    if (cmd.penDown) penDownNow();
    else penUpNow();
    commandIndex++;
    return;
  }

  applyCoordinatedMove(cmd.xMm, cmd.yMm);
  commandIndex++;
}

void runSteppers() {
  if (isHoming) return;
  stepperX.run();
  stepperY.run();
}

bool drawingIsIdle() {
  return !isHoming && !moveInProgress && (commandIndex >= commandCount);
}

void scheduleClockIfNeeded() {
  if (!setupComplete || !homed || !timeIsSet) return;
  if (!drawingIsIdle()) return;

  uint32_t nowSec = currentSecondsOfDay();
  int hour24 = (int)(nowSec / 3600UL);
  int minute = (int)((nowSec % 3600UL) / 60UL);

  if (!forceRedraw && minute == lastDrawnMinute) return;

  if (!generateClockCommands(hour24, minute)) {
    Serial.println("ERROR: command queue overflow while generating clock.");
    return;
  }

  lastDrawnMinute = minute;
  forceRedraw = false;

  Serial.printf("Queued full clock for %02d:%02d (%d commands)\n",
                hour24, minute, commandCount);
}

// -------------------------------- Setup --------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(xLimit, INPUT_PULLUP);
  pinMode(yLimit, INPUT_PULLUP);
  pinMode(stepPinX, OUTPUT);
  pinMode(dirPinX, OUTPUT);
  pinMode(stepPinY, OUTPUT);
  pinMode(dirPinY, OUTPUT);

  digitalWrite(stepPinX, LOW);
  digitalWrite(stepPinY, LOW);
  digitalWrite(dirPinX, LOW);
  digitalWrite(dirPinY, LOW);

  stepperX.setAcceleration(BASE_ACCEL);
  stepperX.setMaxSpeed(BASE_MAX_SPEED);
  stepperY.setAcceleration(BASE_ACCEL);
  stepperY.setMaxSpeed(BASE_MAX_SPEED);

  servo.setPeriodHertz(50);
  servo.attach(servoPin, 1000, 2000);

  Serial.println("Lifting pen before homing...");
  servo.write(SERVO_UP);
  delay(700);
  lastServoPos = SERVO_UP;

  penIsDown = false;
  plannerPenDown = false;

  Serial.println();
  Serial.println("=====================================");
  Serial.println("XY Clock Plotter Starting");
  Serial.println("=====================================");
  Serial.printf("SERVO_UP=%d, SERVO_DOWN=%d\n", SERVO_UP, SERVO_DOWN);
  Serial.printf("Clock center: (%.1f, %.1f) mm\n", CLOCK_CENTER_X_MM, CLOCK_CENTER_Y_MM);
  Serial.printf("Clock radius: %.1f mm\n", CLOCK_RADIUS_MM);
  Serial.println("Homing now...");

  startHoming();
  processHoming();

  printTimePrompt();
  setupComplete = true;
}

// -------------------------------- Loop ---------------------------------------
void loop() {
  serviceSerial();
  processHoming();
  scheduleClockIfNeeded();
  executeNextCommandIfReady();
  runSteppers();
}
