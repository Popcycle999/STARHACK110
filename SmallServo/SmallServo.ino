#include <Servo.h>

// ---- Pins ----
#define TRIG 12
#define ECHO 13
#define SERVO_PIN 3

Servo radarServo;

void setup() {
  Serial.begin(9600);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  radarServo.attach(SERVO_PIN);
  Serial.println("angle,distance");  // header for Serial plotter
}

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

void loop() {
  // Sweep left to right (0 to 180)
  for (int angle = 0; angle <= 180; angle += 2) {
    radarServo.write(angle);
    delay(30);  // let servo reach position
    float dist = getDistance();
    Serial.print(angle);
    Serial.print(",");
    Serial.println(dist);
  }

  // Sweep right to left (180 to 0)
  for (int angle = 180; angle >= 0; angle -= 2) {
    radarServo.write(angle);
    delay(30);
    float dist = getDistance();
    Serial.print(angle);
    Serial.print(",");
    Serial.println(dist);
  }
}