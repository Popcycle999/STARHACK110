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
#define SERVO_STEP_MS     30    // ms per degree — increase for slower sweep
#define SERVO_SETTLE_MS   60    // ms to wait after servo reaches position
#define OBSTACLE_DIST_CM  25    // cm — stop and avoid
#define CLEAR_DIST_CM     100   // cm — consider path clear
#define TURN_MS           400   // ms for a turn
#define BUZZ_SETTLE_MS    20    // ms between buzz cycles

// ---- State ----
bool paused      = false;
bool radarMode   = false;
int  volumeLevel = 50;
const int volumeMax = 100;
const int volumeMin = 1;
const int freqMin   = 200;
const int freqMax   = 2000;

// ---- Servo state ----
int           servoCurrent  = 90;
int           servoTarget   = 90;
unsigned long servoLastStep = 0;

// ---- Motor/action timing ----
unsigned long actionStart   = 0;
unsigned long servoSettle   = 0;
bool          waitingSettle = false;

// ---- Radar sweep state ----
int  sweepAngle     = 0;
int  sweepDirection = 1;   // +1 = left→right, -1 = right→left
bool sweepWaiting   = false;

// ---- Turn state ----
bool          turning     = false;
unsigned long turnStart   = 0;
int           turnDir     = 0;   // -1 left, +1 right

// ---- Buzzer state ----
unsigned long buzzLast = 0;

Servo radarServo;
IRrecv irrecv(IR_PIN);
decode_results results;

// ========== UTILITY: non-blocking wait ==========

bool elapsed(unsigned long &timer, unsigned long duration) {
  if (millis() - timer >= duration) {
    timer = millis();
    return true;
  }
  return false;
}

// ========== SERVO ==========

void servoUpdate() {
  if (servoCurrent == servoTarget) return;
  if (millis() - servoLastStep >= SERVO_STEP_MS) {
    servoCurrent += (servoTarget > servoCurrent) ? 1 : -1;
    radarServo.write(servoCurrent);
    servoLastStep = millis();
  }
}

bool servoIsMoving() {
  return servoCurrent != servoTarget;
}

void servoMoveTo(int target) {
  servoTarget = constrain(target, 0, 180);
}

// Blocking move — only used during obstacle scan and setup
// Safe because it calls servoUpdate() internally instead of delay()
void servoMoveAndWait(int target) {
  servoMoveTo(target);
  while (servoIsMoving()) servoUpdate();
  // Non-blocking settle using millis
  unsigned long t = millis();
  while (millis() - t < SERVO_SETTLE_MS) servoUpdate();
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

// ========== BUZZER (non-blocking tone) ==========

void buzzTick(int frequency, int duration) {
  // Non-blocking: only buzz if enough time has passed
  if (millis() - buzzLast < BUZZ_SETTLE_MS) return;
  tone(BUZZER_PIN, frequency, duration);
  buzzLast = millis();
}

void barkBlocking() {
  // Bark is used only on obstacle detection — short enough to keep blocking
  int mappedFreq = map(volumeLevel, volumeMin, volumeMax, freqMin, freqMax);
  for (int j = 0; j < 3; j++) {
    tone(BUZZER_PIN, mappedFreq, 80);
    unsigned long t = millis(); while (millis() - t < 180) servoUpdate();
    tone(BUZZER_PIN, mappedFreq / 1.5, 80);
    unsigned long t2 = millis(); while (millis() - t2 < 200) servoUpdate();
  }
  unsigned long t3 = millis(); while (millis() - t3 < 300) servoUpdate();
}

// ========== RADAR SWEEP (non-blocking state machine) ==========

void doRadarSweepTick() {
  static unsigned long lastMeasure = 0;

  // Wait until enough time has passed since last move
  if (millis() - lastMeasure < SERVO_SETTLE_MS) return;

  lastMeasure = millis();

  // Measure at current angle
  float dist = getDistance();
  Serial.print("angle:");
  Serial.print(servoCurrent);
  Serial.print(" dist:");
  Serial.println(dist);

  // Decide next angle
  if (servoCurrent >= 180) sweepDirection = -1;
  if (servoCurrent <= 0)   sweepDirection = 1;

  servoMoveTo(servoCurrent + sweepDirection);
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
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  radarServo.attach(SERVO_PIN);
  servoMoveAndWait(90);

  Serial.begin(9600);
  irrecv.enableIRIn();
  Serial.println("Ready. Button 1 = toggle radar mode.");
}

// ========== MAIN LOOP ==========

void loop() {
  servoUpdate();  // always first — keeps servo stepping every loop

  // ---- IR remote ----
  if (irrecv.decode(&results)) {
    unsigned long code = results.value;

    if (code == 0xFFA25D) {
      paused = !paused;
      if (paused) {
        noTone(BUZZER_PIN);
        radarMode     = false;
        turning       = false;
        servoTarget   = servoCurrent;
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
        servoCurrent = 0;
        servoTarget = 0;
        radarServo.write(0);
        sweepDirection = 1;
      } else {
        Serial.println("Radar OFF");
        servoMoveAndWait(90);
      }
    }
    irrecv.resume();
  }

  if (paused) return;

  // ---- Radar mode (fully non-blocking state machine) ----
  if (radarMode) {
    doRadarSweepTick();
    return;
  }

  // ---- Handle active turn (non-blocking) ----
  if (turning) {
    if (millis() - turnStart >= TURN_MS) {
      turning = false;
    }
    return;
  }

  // ---- Normal driving + obstacle avoidance ----
  servoMoveTo(90);  // keep facing forward (non-blocking)

  float distance = getDistance();
  Serial.print("Distance: "); Serial.print(distance); Serial.println(" cm");

  if (distance == 0 || distance > CLEAR_DIST_CM) {
  }
  else if (distance < OBSTACLE_DIST_CM) {
    Serial.println("Obstacle! Looking for best direction...");
    barkBlocking();
    int dir = getBestDirection();
    if (dir == -1) {
      Serial.println("Turning left");
    } else {
      Serial.println("Turning right");
    }
    turning   = true;
    turnStart = millis();
    turnDir   = dir;
  }
  else {
    int freq = map(distance, CLEAR_DIST_CM, OBSTACLE_DIST_CM, 400,
                   map(volumeLevel, volumeMin, volumeMax, freqMin, freqMax));
    int dur  = map(distance, CLEAR_DIST_CM, OBSTACLE_DIST_CM, 350, 100);
    buzzTick(freq, dur);
  }
}