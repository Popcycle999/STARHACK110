#include <Stepper.h>

#define STEPS_PER_REV 2048

// 28BYJ-48 needs 4 pins in this order: IN1, IN3, IN2, IN4
Stepper motor(STEPS_PER_REV, 3, 8, 6, 12);

void stopMotor() {
  digitalWrite(3, LOW);
  digitalWrite(6, LOW);
  digitalWrite(8, LOW);
  digitalWrite(12, LOW);
}

void setup() {
  Serial.begin(9600);
  motor.setSpeed(10);
  Serial.println("Ready.");
}

void loop() {
  Serial.println("Forward...");
  motor.step(2048);
  stopMotor();
  delay(1000);

  Serial.println("Backward...");
  motor.step(-2048);
  stopMotor();
  delay(1000);
}