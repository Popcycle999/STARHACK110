#include <Servo.h>
#include <IRremote.h>

// ---- Ultrasonic ----
#define TRIG 9
#define ECHO 10

// ---- Servo ----
#define SERVO_PIN 8

// ---- Buzzer & IR ----
#define BUZZER_PIN 3
#define IR_PIN A0

// ---- Tuning ----
#define SERVO_STEP_MS     30    // ms per degree
#define SERVO_SETTLE_MS   30    // ms to wait still before reading distance
#define OBSTACLE_DIST_CM  25
#define CLEAR_DIST_CM     100
#define TURN_MS           400
#define BUZZ_SETTLE_MS    20

// ---- State ----
bool paused      = false;
bool radarMode   = false;
int  volumeLevel = 50;
const int volumeMax = 100;
const int volumeMin = 1;
const int freqMin   = 200;
const int freqMax   = 2000;

// ---- Servo state ----
int           servoCurrent   = 90;
int           servoTarget    = 90;
unsigned long servoLastStep  = 0;
unsigned long servoStillSince = 0;  // when servo last stopped moving
bool          servoWasMoving  = false;

// ---- Radar sweep state machine ----
// States: 0=moving, 1=settling, 2=measure+advance
int           sweepState     = 0;
int           sweepDirection = 1;

// ---- Turn state ----
bool          turning    = false;
unsigned long turnStart  = 0;
int           turnDir    = 0;

// ---- Buzzer state ----
unsigned long buzzLast = 0;

Servo radarServo;
IRrecv irrecv(IR_PIN);
decode_results results;

// ========== SERVO ==========

void servoUpdate() {
  bool wasMoving = servoIsMoving();
  if (servoCurrent == servoTarget) {
    if (wasMoving != servoWasMoving) {
      // Just stopped — record the time
      servoStillSince = millis();
    }
    servoWasMoving = false;
    return;
  }
  servoWasMoving = true;
  if (millis() - servoLastStep >= SERVO_STEP_MS) {
    servoCurrent += (servoTarget > servoCurrent) ? 1 : -1;
    radarServo.write(servoCurrent);
    servoLastStep = millis();
  }
}

bool servoIsMoving() {
  return servoCurrent != servoTarget;
}

bool servoSettled() {
  // True when servo has been still for at least SERVO_SETTLE_MS
  return !servoIsMoving() && (millis() - servoStillSince >= SERVO_SETTLE_MS);
}

void servoMoveTo(int target) {
  servoTarget = constrain(target, 0, 180);
}

void servoMoveAndWait(int target) {
  servoMoveTo(target);
  while (!servoSettled()) servoUpdate();
}

// ========== MOTOR FUNCTIONS ==========

void forward() {
  digitalWrite(A1, LOW);  // IN2
  digitalWrite(3,  HIGH); // IN1 — note: pin 3 is also BUZZER, only use when not buzzing
  // *** If motors are wired per your latest pinout, update these ***
}

// Since you removed motor pins from this file, add them back here:
#define IN1 A2
#define IN2 A1
#define ENA 6
#define IN3 A3
#define IN4 A4
#define ENB 7

void motorForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 255); analogWrite(ENB, 255);
}

void motorBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, 255); analogWrite(ENB, 255);
}

void motorTurnLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 255 / 2); analogWrite(ENB, 255);
}

void motorTurnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, 255); analogWrite(ENB, 255 / 2);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

// ========== ULTRASONIC ==========

float getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH, 30000);
  if (duration == 0) return 999.0;
  return duration * 0.0343 / 2.0;
}

// ========== BUZZER ==========

void buzzTick(int frequency, int duration) {
  if (millis() - buzzLast < BUZZ_SETTLE_MS) return;
  tone(BUZZER_PIN, frequency, duration);
  buzzLast = millis();
}

void barkBlocking() {
  int mappedFreq = map(volumeLevel, volumeMin, volumeMax, freqMin, freqMax);
  for (int j = 0; j < 3; j++) {
    tone(BUZZER_PIN, mappedFreq, 80);
    unsigned long t = millis(); while (millis() - t < 180) servoUpdate();
    tone(BUZZER_PIN, mappedFreq / 1.5, 80);
    unsigned long t2 = millis(); while (millis() - t2 < 200) servoUpdate();
  }
  unsigned long t3 = millis(); while (millis() - t3 < 300) servoUpdate();
}

// ========== RADAR SWEEP STATE MACHINE ==========
// State 0: servo is moving → wait
// State 1: servo just settled → take reading, then move to next angle

void doRadarSweepTick() {
  servoUpdate();

  switch (sweepState) {

    case 0: // waiting for servo to settle
      if (servoSettled()) {
        sweepState = 1;
      }
      break;

    case 1: // servo settled — read distance then advance
      float dist = getDistance();
      Serial.print(servoCurrent);
      Serial.print(",");
      Serial.print(dist);
      Serial.println(".");

      // Advance angle
      if (servoCurrent >= 165) sweepDirection = -1;
      if (servoCurrent <= 15)  sweepDirection = 1;

      servoMoveTo(servoCurrent + sweepDirection);
      sweepState = 0;  // back to waiting
      break;
  }
}

// ========== OBSTACLE AVOIDANCE ==========

int getBestDirection() {
  servoMoveAndWait(0);
  float distLeft = getDistance();
  Serial.print("Left: "); Serial.print(distLeft); Serial.println(" cm");

  servoMoveAndWait(180);
  float distRight = getDistance();
  Serial.print("Right: "); Serial.print(distRight); Serial.println(" cm");

  servoMoveAndWait(90);
  return (distLeft > distRight) ? -1 : 1;
}

// ========== SETUP ==========

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  radarServo.attach(SERVO_PIN);
  servoMoveAndWait(90);

  Serial.begin(9600);
  irrecv.enableIRIn();
  stopMotors();
  Serial.println("Ready. Button 1 = toggle radar mode.");
}

// ========== MAIN LOOP ==========

void loop() {
  servoUpdate();

  // ---- IR remote ----
  if (irrecv.decode(&results)) {
    unsigned long code = results.value;

    if (code == 0xFFA25D) {
      paused = !paused;
      if (paused) {
        stopMotors();
        noTone(BUZZER_PIN);
        radarMode   = false;
        turning     = false;
        servoTarget = servoCurrent;
      }
    }
    else if (code == 0xFF629D) {
      volumeLevel = min(volumeLevel + 2, volumeMax);
    }
    else if (code == 0xFFA857) {
      volumeLevel = max(volumeLevel - 2, volumeMin);
    }
    else if (code == 0xFF30CF) {
      radarMode = !radarMode;
      if (radarMode) {
        Serial.println("Radar ON");
        sweepDirection = 1;
        sweepState     = 0;
        servoMoveTo(15);   // start position
        stopMotors();
      } else {
        Serial.println("Radar OFF");
        servoMoveAndWait(90);
      }
    }
    irrecv.resume();
  }

  if (paused) return;

  // ---- Radar mode ----
  if (radarMode) {
    doRadarSweepTick();
    return;
  }

  // ---- Handle active turn ----
  if (turning) {
    if (millis() - turnStart >= TURN_MS) {
      turning = false;
      stopMotors();
    }
    return;
  }

  // ---- Normal driving ----
  servoMoveTo(90);
  float distance = getDistance();
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");

  if (distance == 0 || distance > CLEAR_DIST_CM) {
    motorForward();
  }
  else if (distance < OBSTACLE_DIST_CM) {
    Serial.println("Obstacle!");
    stopMotors();
    barkBlocking();
    int dir = getBestDirection();
    if (dir == -1) { Serial.println("Turning left");  motorTurnLeft(); }
    else           { Serial.println("Turning right"); motorTurnRight(); }
    turning   = true;
    turnStart = millis();
    turnDir   = dir;
  }
  else {
    motorForward();
    int freq = map(distance, CLEAR_DIST_CM, OBSTACLE_DIST_CM, 400,
                   map(volumeLevel, volumeMin, volumeMax, freqMin, freqMax));
    int dur  = map(distance, CLEAR_DIST_CM, OBSTACLE_DIST_CM, 350, 100);
    buzzTick(freq, dur);
  }
}