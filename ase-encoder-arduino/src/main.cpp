#include <Arduino.h>

// Definicja pinów dla sterowania silnikami
#define IN1 25  // Pin IN1 dla kanału A
#define IN2 26  // Pin IN2 dla kanału A
#define IN3 27  // Pin IN3 dla kanału B
#define IN4 14  // Pin IN4 dla kanału B
#define ENA 12  // PWM dla kanału A
#define ENB 13  // PWM dla kanału B
#define ENCODER_PIN1 16 // Pin enkodera 1 - GPIO 0 na ESP32
#define ENCODER_PIN2 17 // Pin enkodera 2 - GPIO 2 na ESP32

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
float targetDistance = 100; // Przykładowa wartość, należy dostosować do potrzeb

unsigned long lastTime = 0; // Dodana deklaracja zmiennej lastTime

// Zmienne do przechowywania poprzednich wartości sygnału sterującego
float previousPWM1 = 0;
float previousPWM2 = 0;

// Stała filtra (alpha) - zakres 0.0 do 1.0
float alpha = 0.1;

class PIController
{
public:
  PIController(double _k, double _i, double _i_low, double _i_high): 
    k(_k), i(_i), imin(_i_low), imax(_i_high) {

    } 

  double update(double error, double dt) {
    integral = constrain(integral + error * dt, imin, imax);
    return error*k + integral*i;
  }
private:
  double k,i;
  double integral;
  double imin,imax; // Integral min and max
};

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
  Serial.setTimeout(1000000000);
}

#define TARGET_PULSES 50

void loop() {
  // Aktualny czas
  unsigned long now = millis();
  // Obliczenie czasu od ostatniego odczytu
  double dt = (now - lastTime) / 1000.0; // Czas w sekundach
  lastTime = now;

  // Obliczenie błędu regulacji dla obu enkoderów
  uint16_t error1 = TARGET_PULSES - currentPosition1;
  uint16_t error2 = TARGET_PULSES - currentPosition2;

  // Ustawienie ograniczeń dla sygnału sterującego PWM
  // pwm1 = constrain(pwm1, 100, 255); // Ograniczenie PWM do mniejszych wartości
  // pwm2 = constrain(pwm2, 100, 255); // Ograniczenie PWM do mniejszych wartości

  // String pwmCmd = Serial.readStringUntil('\r');
  // int pwm=pwmCmd.toInt();
  // analogWrite(ENA, pwm);
  // analogWrite(ENB, pwm+10);
  // Serial.println(pwm);

  // Ustawienie wypełnienia sygnału PWM na podstawie wyniku regulatora PID
  // analogWrite(ENA, abs(pwm1));
  // analogWrite(ENB, abs(pwm2));

  // Ustawienie kierunku obrotów silników
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


  // Wyświetlenie obecnej pozycji i błędu
  // Serial.print("Imp1: ");
  // Serial.print(newPosition1);
  // Serial.print("\t");
  // Serial.print("Imp2: ");
  // Serial.println(newPosition2);

// Sprawdzenie, czy cel został osiągnięty
// if (newPosition1 * MM_PER_PULSE >= targetDistance && newPosition2 * MM_PER_PULSE >= targetDistance) {
//     analogWrite(ENA, 0);
//     analogWrite(ENB, 0);
//     digitalWrite(IN1, LOW);
//     digitalWrite(IN2, LOW);
//     digitalWrite(IN3, LOW);
//     digitalWrite(IN4, LOW);
//     Serial.println("Target distance reached!");
//     while (true) {
//       Serial.print("Imp1: ");
//       Serial.print(newPosition1);
//       Serial.print("\t");
//       Serial.print("Imp2: ");
//       Serial.println(newPosition2);
//     } // Zatrzymanie pętli głównej
// }

  // delay(100); // Delikatne opóźnienie dla stabilności odczytu
}
