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
  //pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT); pinMode(ENC, OUTPUT);
  //pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT); pinMode(END, OUTPUT);
}

// Move a motor: speed 0-255, dir true=forward false=backward
void motorMove(int in_a, int in_b, int en, int speed, bool forward) {
  digitalWrite(in_a, forward ? HIGH : LOW);
  digitalWrite(in_b, forward ? LOW : HIGH);
  analogWrite(en, speed);
}

void motorStop(int in_a, int in_b, int en) {
  digitalWrite(in_a, LOW);
  digitalWrite(in_b, LOW);
  analogWrite(en, 0);
}

void loop() {
  // All 4 motors forward at half speed
  //motorMove(IN1, IN2, ENA, 128, true);  // Motor A
  motorMove(IN3, IN4, ENB, 128, true);  // Motor B
  //motorMove(IN5, IN6, ENC, 128, true);  // Motor C
  //motorMove(IN7, IN8, END, 128, true);  // Motor D
  delay(2000);

  // Stop all
  //motorStop(IN1, IN2, ENA);
  motorStop(IN3, IN4, ENB);
  //motorStop(IN5, IN6, ENC);
  //motorStop(IN7, IN8, END);
  delay(1000);
}