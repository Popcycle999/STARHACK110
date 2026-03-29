#include <IRremote.h>
#include <SPI.h>
#include <MFRC522.h>
#include <NewTone.h>

// ---- Pins ----
#define IR_PIN 2
#define BUZZER_PIN 3
#define RFID_SS_PIN 4
#define RFID_RST_PIN 5
#define TRIG 9
#define ECHO 8
#define LINE_SENSOR A0

// ---- Tuning ----
#define OBSTACLE_DIST_CM 25
#define CLEAR_DIST_CM 100
#define BUZZ_SETTLE_MS 20
#define LINE_STABLE_MS 350

// ---- State ----
bool paused = false;
int volumeLevel = 50;

const int volumeMax = 100;
const int volumeMin = 1;
const int freqMin = 200;
const int freqMax = 2000;

// ---- Buzzer / line state ----
unsigned long buzzLast = 0;

// état confirmé du capteur
int stableLineValue = -1;

// état en cours d'observation
int candidateLineValue = -1;
unsigned long candidateStartTime = 0;

// ---- IR ----
IRrecv irrecv(IR_PIN);
decode_results results;

// ---- RFID ----
MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);

// ========== ULTRASONIC ==========

float getDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duration = pulseIn(ECHO, HIGH, 12000);
  if (duration == 0) return 999.0;

  return duration * 0.0343 / 2.0;
}

// ========== BUZZER ==========

void buzzTick(int frequency, int duration) {
  if (millis() - buzzLast < BUZZ_SETTLE_MS) return;
  NewTone(BUZZER_PIN, frequency, duration);
  buzzLast = millis();
}

void stopBuzzer() {
  noNewTone(BUZZER_PIN);
}

void startContinuousBuzz(int frequency) {
  NewTone(BUZZER_PIN, frequency);
}

void rfidBeep() {
  int freq1 = 800;
  int freq2 = 1200;
  int freq3 = 1600;

  NewTone(BUZZER_PIN, freq1, 120);
  delay(150);

  noNewTone(BUZZER_PIN);
  delay(100);

  NewTone(BUZZER_PIN, freq2, 120);
  delay(150);

  noNewTone(BUZZER_PIN);
  delay(60);

  NewTone(BUZZER_PIN, freq3, 120);
  delay(150);

  noNewTone(BUZZER_PIN);
}

void lineTransitionBeep() {
  int freq1 = 2100;
  int freq2 = 2500;

  NewTone(BUZZER_PIN, freq1, 70);
  delay(90);

  noNewTone(BUZZER_PIN);
  delay(60);

  NewTone(BUZZER_PIN, freq2, 70);
  delay(90);

  noNewTone(BUZZER_PIN);
  delay(60);

  NewTone(BUZZER_PIN, freq1, 70);
  delay(90);

  noNewTone(BUZZER_PIN);
  delay(60);

  NewTone(BUZZER_PIN, freq2, 70);
  delay(90);

  noNewTone(BUZZER_PIN);
}

// ========== RFID ==========

bool checkRFIDCard() {
  if (!mfrc522.PICC_IsNewCardPresent()) {
    return false;
  }

  if (!mfrc522.PICC_ReadCardSerial()) {
    return false;
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  return true;
}

// ========== LINE SENSOR FILTER ==========

void updateLineState(int currentLineValue) {
  // initialisation
  if (stableLineValue == -1) {
    stableLineValue = currentLineValue;
    candidateLineValue = currentLineValue;
    candidateStartTime = millis();
    return;
  }

  // si on lit encore l'état déjà confirmé
  if (currentLineValue == stableLineValue) {
    candidateLineValue = currentLineValue;
    candidateStartTime = millis();
    return;
  }

  // si la lecture a changé, mais que c'est une nouvelle candidate
  if (currentLineValue != candidateLineValue) {
    candidateLineValue = currentLineValue;
    candidateStartTime = millis();
    return;
  }

  // si la candidate est restée stable assez longtemps, on valide le changement
  if (millis() - candidateStartTime >= LINE_STABLE_MS) {
    stableLineValue = candidateLineValue;
    lineTransitionBeep();
    candidateStartTime = millis();
  }
}

// ========== SETUP ==========

void setup() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LINE_SENSOR, INPUT);

  Serial.begin(9600);

  irrecv.enableIRIn();

  SPI.begin();
  mfrc522.PCD_Init();

  stopBuzzer();

  int initialLineValue = digitalRead(LINE_SENSOR);
  stableLineValue = initialLineValue;
  candidateLineValue = initialLineValue;
  candidateStartTime = millis();
}

// ========== MAIN LOOP ==========

void loop() {
  // ---- IR remote ----
  if (irrecv.decode(&results)) {
    unsigned long code = results.value;

    Serial.print("IR code: 0x");
    Serial.println(code, HEX);

    if (code == 0xFFA25D) {  // toggle pause
      paused = !paused;
      if (paused) {
        stopBuzzer();
      }
    }
    else if (code == 0xFF629D) {  // volume +
      volumeLevel = min(volumeLevel + 2, volumeMax);
    }
    else if (code == 0xFFA857) {  // volume -
      volumeLevel = max(volumeLevel - 2, volumeMin);
    }

    irrecv.resume();
  }

  if (paused) {
    stopBuzzer();
    return;
  }

  // ---- RFID ----
  if (checkRFIDCard()) {
    rfidBeep();
  }

  // ---- Read sensors ----
  float distance = getDistance();
  int lineValue = digitalRead(LINE_SENSOR);

  // ---- Filtered line transition detection ----
  updateLineState(lineValue);

  // ---- Obstacle logic ----
  if (distance < OBSTACLE_DIST_CM) {
    int alertFreq = map(volumeLevel, volumeMin, volumeMax, freqMin, freqMax);
    startContinuousBuzz(alertFreq);
  }
  else if (distance > CLEAR_DIST_CM || distance == 0) {
    stopBuzzer();
  }
  else {
    int freq = map(
      distance,
      CLEAR_DIST_CM,
      OBSTACLE_DIST_CM,
      400,
      map(volumeLevel, volumeMin, volumeMax, freqMin, freqMax)
    );

    int dur = map(distance, CLEAR_DIST_CM, OBSTACLE_DIST_CM, 350, 100);
    buzzTick(freq, dur);
  }

  // ---- Debug ----
  Serial.print("Line raw: ");
  Serial.print(lineValue);
  Serial.print(" | Line stable: ");
  Serial.println(stableLineValue);

  delay(50);
}