#include <Arduino.h>

// Definicja pinów dla sterowania silnikami
#define IN1 25          // Pin IN1 dla kanału A
#define IN2 26          // Pin IN2 dla kanału A
#define IN3 27          // Pin IN3 dla kanału B
#define IN4 14          // Pin IN4 dla kanału B
#define ENA 12          // PWM dla kanału A
#define ENB 13          // PWM dla kanału B
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
  PIController(double _k, double _i, double _i_low, double _i_high) : k(_k), i(_i), imin(_i_low), imax(_i_high) {}

  double update(double error, double dt)
  {
    integral = constrain(integral + error * dt, imin, imax);
    return error * k + integral * i;
  }

private:
  double k, i;
  double integral;
  double imin, imax; // Integral min and max
};

int8_t dirA=1, dirB=1;

void IRAM_ATTR updateEncoder1()
{
  currentPosition1+=dirA;
}

void IRAM_ATTR updateEncoder2()
{
  currentPosition2+=dirB;
}

#define PI_K_A 1.5
#define PI_I_A 0.8
#define PI_IMIN_A -200.0 / PI_I_A
#define PI_IMAX_A 200.0 / PI_I_A

#define PI_K_B 1.5
#define PI_I_B 0.8
#define PI_IMIN_B -200.0 / PI_I_B
#define PI_IMAX_B 200.0 / PI_I_B

PIController piA(PI_K_A, PI_I_A, PI_IMIN_A, PI_IMAX_A);
PIController piB(PI_K_B, PI_I_B, PI_IMIN_B, PI_IMAX_B);

void setup()
{
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

  // Ustawienie kierunku obrotów silników
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

#define TARGET_PULSES 20

long lastTimePrinted = 0;
int prev_pwmA=0;
int prev_pwmB=0;

void loop()
{
  // Aktualny czas
  unsigned long now = millis();
  // Obliczenie czasu od ostatniego odczytu
  double dt = (now - lastTime) / 1000.0; // Czas w sekundach
  lastTime = now;

  // Obliczenie błędu regulacji dla obu enkoderów
  int16_t error1 = TARGET_PULSES - currentPosition1;
  int16_t error2 = TARGET_PULSES - currentPosition2;

  int pwmA = piA.update(error1, dt);
  int pwmB = piB.update(error2, dt);
  // Ustawienie ograniczeń dla sygnału sterującego PWM
  // pwmA = constrain(pwmA, 180, 255); // Ograniczenie PWM do mniejszych wartości
  // pwmB = constrain(pwmB, 190, 255); // Ograniczenie PWM do mniejszych wartości

  if (pwmA < 0) dirA=-1; else dirA=1;
  if (pwmB < 0) dirB=-1; else dirB=1;

  pwmA += 180/(1.34) * dirA;
  pwmB += 190/(1.34) * dirB;

  pwmA = 0.5*prev_pwmA + 0.5*pwmA;
  pwmB = 0.5*prev_pwmB + 0.5*pwmB;

  prev_pwmA = pwmA;
  prev_pwmB = pwmB;

  pwmA = constrain(pwmA, -255, 255);
  pwmB = constrain(pwmB, -255, 255);

  digitalWrite(IN1, pwmA<0);
  digitalWrite(IN2, pwmA>0);
  digitalWrite(IN3, pwmB<0);
  digitalWrite(IN4, pwmB>0);

  // Ustawienie wypełnienia sygnału PWM na podstawie wyniku regulatora PID
  analogWrite(ENA, abs(pwmA));
  analogWrite(ENB, abs(pwmB));

  // Wyświetlenie obecnej pozycji i błędu
  if (millis() - lastTimePrinted > 500)
  {
    Serial.print("Imp1: ");
    Serial.print(currentPosition1);
    Serial.print("\t");
    Serial.print("Imp2: ");
    Serial.print(currentPosition2);
    Serial.print("\t\t");
    Serial.print("errA: ");
    Serial.print(error1);
    Serial.print("\t");
    Serial.print("pwmA: ");
    Serial.print(pwmA);
    Serial.print("\t");
    Serial.print("errB: ");
    Serial.print(error2);
    Serial.print("\t");
    Serial.print("pwmB: ");
    Serial.println(pwmB);
    lastTimePrinted = millis();
  }

  delay(10); // Delikatne opóźnienie dla stabilności odczytu
}
