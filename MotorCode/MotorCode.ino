// Motor A (Chip 1)
#define IN1 3  // Positif moteur 1
#define IN2 8  // Négatif moteur 1
#define ENA 6   // PWM

// Motor B (Chip 1)
#define IN3 4  // Positif moteur 1
#define IN4 5  // Négatif moteur 1
#define ENB 7  // PWM

#define SPEED 150  // 0-255, adjust to your motors

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
}

// ---- Movement functions ----

void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // Left forward
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  // Right forward
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
}

void backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  // Left backward
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  // Right backward
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
}

void turnLeft() {
  // Left wheel slower, right wheel faster
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED / 2);  // Left at half speed
  analogWrite(ENB, SPEED);      // Right at full speed
}

void turnRight() {
  // Right wheel slower, left wheel faster
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED);      // Left at full speed
  analogWrite(ENB, SPEED / 2);  // Right at half speed
}

void sharpLeft() {
  // Left wheel backward, right wheel forward
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
}

void sharpRight() {
  // Left wheel forward, right wheel backward
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void loop() {
  forward();   delay(2000);
  turnLeft();  delay(800);
  forward();   delay(2000);
  turnRight(); delay(800);
  sharpLeft(); delay(500);
  sharpRight();delay(500);
  stopMotors();delay(2000);
}