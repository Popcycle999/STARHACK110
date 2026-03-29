#include <IRremote.h>

// --- Moteurs ---
#define IN1 3  // Positif moteur A
#define IN2 8  // Négatif moteur A
#define ENA 6  // PWM A

#define IN3 4  // Positif moteur B
#define IN4 5  // Négatif moteur B
#define ENB 7  // PWM B

#define SPEED  255
#define MAX_SPEED 255  

// --- Capteurs & Son ---
#define TRIG 9
#define ECHO 10
#define BUZZER 11
#define IR_PIN 2

bool paused = false;      
int volumeLevel = 50;     
const int volumeMax = 100;
const int volumeMin = 1;

const int freqMin = 200;
const int freqMax = 2000;

IRrecv irrecv(IR_PIN);
decode_results results;

// ---- Fonctions Mouvement ----
void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  
  analogWrite(ENA, SPEED); analogWrite(ENB, SPEED);

}

void backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);  
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);  
  analogWrite(ENA, SPEED); analogWrite(ENB, SPEED);
}

void turnLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED / 2); analogWrite(ENB, SPEED);      
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED); analogWrite(ENB, SPEED / 2);  
}

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  analogWrite(ENA, 0); analogWrite(ENB, 0);
}

// --- Fonctions Capteurs ---
long readDistance() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);
  return duration * 0.034 / 2; // cm
}

void buzz(int frequency, int duration) {
  long delayValue = 1000000 / frequency / 2; 
  long cycles = (long)frequency * duration / 1000;
  for (long i = 0; i < cycles; i++) {
    digitalWrite(BUZZER, HIGH);
    delayMicroseconds(delayValue);
    digitalWrite(BUZZER, LOW);
    delayMicroseconds(delayValue);
    if (paused) return; 
  }
}

void bark() {
  int mappedFreq = map(volumeLevel, volumeMin, volumeMax, freqMin, freqMax);
  for (int j = 0; j < 3; j++) {
    buzz(mappedFreq, 80);        
    delay(100);
    buzz(mappedFreq / 1.5, 80);  
    delay(120);
    if (paused) return;
  }
  delay(300);
}

void startFunction() {
  Serial.println("=== START FUNCTION TRIGGERED ===");
}

void setup() {
  // Setup moteurs
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  
  // Setup capteurs
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  pinMode(BUZZER, OUTPUT);

  Serial.begin(9600);
  irrecv.enableIRIn();
  stopMotors(); // Assure que le robot ne part pas en fou au démarrage
}

void loop() {
  // --- Vérifier la manette IR ---
  if (irrecv.decode(&results)) {
    unsigned long code = results.value;
    
    if (code == 0xFFA25D) { // Bouton rouge
      paused = !paused;
      if (paused) stopMotors(); // Coupe les moteurs immédiatement si on pause
    }
    else if (code == 0xFF629D) { // Volume +
      volumeLevel += 2;
      if (volumeLevel > volumeMax) volumeLevel = volumeMax;
    }
    else if (code == 0xFFA857) { // Volume -
      volumeLevel -= 2;
      if (volumeLevel < volumeMin) volumeLevel = volumeMin;
    }
    else if (code == 0xFF30CF) { // Bouton 1
      startFunction();
    }
    irrecv.resume();
  }

  // --- Si en pause, on ne fait rien d'autre ---
  if (paused) return;

  // --- Logique de déplacement ---
  long distance = readDistance();

  if (distance == 0 || distance > 100) {
    forward();
  } 
  else if (distance < 25) {
    stopMotors();
    bark();
  } 
  else {
    forward();
    int freq = map(distance, 100, 25, 400, map(volumeLevel, volumeMin, volumeMax, freqMin, freqMax));
    int dur  = map(distance, 100, 25, 350, 100);
    buzz(freq, dur);
    delay(20);
  }
}