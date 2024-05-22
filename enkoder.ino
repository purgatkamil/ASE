#include <Arduino.h>

// Definicja pinów dla sterowania silnikami
const int IN1 = 25;  // Pin IN1 dla kanału A
const int IN2 = 26;  // Pin IN2 dla kanału A
const int IN3 = 27;  // Pin IN3 dla kanału B
const int IN4 = 14;  // Pin IN4 dla kanału B
const int ENA = 12;  // PWM dla kanału A
const int ENB = 13;  // PWM dla kanału B
const int ENCODER_PIN1 = 0; // Pin enkodera 1 - GPIO 0 na ESP32
const int ENCODER_PIN2 = 2; // Pin enkodera 2 - GPIO 2 na ESP32

// Stała przeliczeniowa dla enkodera (ile milimetrów na impuls)
#define MM_PER_PULSE 1.35 // Przykładowa wartość, należy dostosować do rzeczywistego enkodera

// Zmienne przechowujące obecną pozycję
volatile long currentPosition1 = 0;
volatile long currentPosition2 = 0;

// Zmienne do przechowywania poprzednich wartości regulatora
float previousError1 = 0;
float previousError2 = 0;
float integral1 = 0;
float integral2 = 0;

// Parametry regulatora PID
float Kp = 1.5; // Wzmocnienie proporcjonalne
float Ki = 0.1; // Wzmocnienie całkujące
float Kd = 0.1; // Wzmocnienie różniczkujące

// Maksymalna wartość całki błędu
float maxIntegral = 100.0;

// Odległość do przejechania
float targetDistance = 1350; // Przykładowa wartość, należy dostosować do potrzeb

unsigned long lastTime = 0; // Dodana deklaracja zmiennej lastTime

// Zmienne do przechowywania poprzednich wartości sygnału sterującego
float previousPWM1 = 0;
float previousPWM2 = 0;

// Stała filtra (alpha) - zakres 0.0 do 1.0
float alpha = 0.1;

void IRAM_ATTR updateEncoder1() {
  currentPosition1++;
}

void IRAM_ATTR updateEncoder2() {
  currentPosition2++;
}

void setup() {
  // Inicjalizacja pinów dla sterowania silnikami
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  
  // Inicjalizacja pinów enkoderów
  pinMode(ENCODER_PIN1, INPUT_PULLUP);
  pinMode(ENCODER_PIN2, INPUT_PULLUP);

  // Wyzerowanie początkowej pozycji enkoderów
  currentPosition1 = 0;
  currentPosition2 = 0;

  // Inicjalizacja przerwań dla enkoderów
  attachInterrupt(ENCODER_PIN1, updateEncoder1, CHANGE);
  attachInterrupt(ENCODER_PIN2, updateEncoder2, CHANGE);

  // Inicjalizacja komunikacji szeregowej
  Serial.begin(9600);
}

void loop() {
  // Aktualny czas
  unsigned long now = millis();
  // Obliczenie czasu od ostatniego odczytu
  float dt = (now - lastTime) / 1000.0; // Czas w sekundach
  lastTime = now;

  // Odczyt obecnej pozycji z enkoderów
  long newPosition1 = currentPosition1;
  long newPosition2 = currentPosition2;

  // Obliczenie błędu regulacji dla obu enkoderów
  float error1 = targetDistance - (newPosition1 * MM_PER_PULSE);
  float error2 = targetDistance - (newPosition2 * MM_PER_PULSE);

  // Obliczenie całki błędu z ograniczeniem
  integral1 += error1 * dt;
  integral1 = constrain(integral1, -maxIntegral, maxIntegral);
  
  integral2 += error2 * dt;
  integral2 = constrain(integral2, -maxIntegral, maxIntegral);
  

  // Obliczenie różniczki błędu
  float derivative1 = (error1 - previousError1) / dt;
  float derivative2 = (error2 - previousError2) / dt;
  previousError1 = error1;
  previousError2 = error2;

  // Obliczenie sygnału sterującego PWM za pomocą regulatora PID
  float pwm1 = Kp * error1 + Ki * integral1 + Kd * derivative1;
  float pwm2 = Kp * error2 + Ki * integral2 + Kd * derivative2;
  
  pwm1 = alpha * pwm1 + (1 - alpha) * previousPWM1;
  pwm2 = alpha * pwm2 + (1 - alpha) * previousPWM2;

  // Przechowywanie bieżącej wartości sygnału PWM dla następnej iteracji
  previousPWM1 = pwm1;
  previousPWM2 = pwm2;

  // Ustawienie ograniczeń dla sygnału sterującego PWM
  pwm1 = constrain(pwm1, -255, 255); // Ograniczenie PWM do mniejszych wartości
  pwm2 = constrain(pwm2, -255, 255); // Ograniczenie PWM do mniejszych wartości

  // Ustawienie wypełnienia sygnału PWM na podstawie wyniku regulatora PID
  analogWrite(ENA, abs(pwm1));
  analogWrite(ENB, abs(pwm2));

  // Ustawienie kierunku obrotów silników
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


  // Wyświetlenie obecnej pozycji i błędu
  Serial.print("Current Position 1 (mm): ");
  Serial.println(newPosition1 * MM_PER_PULSE);
  Serial.print("Impulse Count 1: ");
  Serial.println(newPosition1);

  Serial.print("Current Position 2 (mm): ");
  Serial.println(newPosition2 * MM_PER_PULSE);
  Serial.print("Impulse Count 2: ");
  Serial.println(newPosition2);

// Sprawdzenie, czy cel został osiągnięty
if (newPosition1 * MM_PER_PULSE >= targetDistance && newPosition2 * MM_PER_PULSE >= targetDistance) {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    Serial.println("Target distance reached!");
    while (true) {} // Zatrzymanie pętli głównej
}

  delay(100); // Delikatne opóźnienie dla stabilności odczytu
}
