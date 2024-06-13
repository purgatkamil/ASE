#include <ESP32Servo.h> //Biblioteka odpowiedzialna za serwa
 
//#define trigPin 12
//#define echoPin 11

const int sensorPinLeft = 22;
const int sensorPinRight = 23;

#define trigPin 33
#define echoPin 39

Servo serwomechanizm;  //Tworzymy obiekt, dzięki któremu możemy odwołać się do serwa 
int pozycja = 45; //Aktualna pozycja serwa 0-180
int zmiana = 5; //Co ile ma się zmieniać pozycja serwa?
int kierunek = 1;

const int IN1 = 32;  // Pin IN1 dla kanału A
const int IN2 = 2;  // Pin IN2 dla kanału A
const int IN3 = 27;  // Pin IN3 dla kanału B
const int IN4 = 26;  // Pin IN4 dla kanału B
const int ENA = 12;  // pwm dla kanału A
const int ENB = 14;  // pwm dla kanału B

int dir = 1;
volatile static int turning = 0;

const int sensorPin = 15; // Pin do którego podłączony jest czujnik odbiciowy

void IRAM_ATTR handleRightSensorInterrupt() {
  noInterrupts();
  EngineTurnRight();
  turning = 1;
  
}

void IRAM_ATTR handleLeftSensorInterrupt() {
  noInterrupts();
  EngineTurnLeft();
  turning = 1;
  
}

void EngineMoveForward(){
  dir = 1;
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void EngineMoveBackward(){
  dir = -1;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void EngineFastStop(){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void EngineSoftStop(){
    //analogWrite(ENA, 0);
    //analogWrite(ENB, 0);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
}

void EngineTurnRight(){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void EngineTurnLeft(){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void EngineSetLeftSpeed(int leftSpeed)
{
  int speed = leftSpeed;
  if(leftSpeed >= 255)
  {
    speed = 250;
  }
    analogWrite(ENA, speed);

}

void EngineSetRightSpeed(int rightSpeed)
{
  int speed = rightSpeed;
  if(rightSpeed >= 255)
  {
    speed = 250;
  }
  speed = speed + (dir * 3);
    analogWrite(ENB, speed);
}

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT); //Pin, do którego podłączymy trig jako wyjście
  pinMode(echoPin, INPUT); //a echo, jako wejście

  pinMode(IN1, OUTPUT); // ustawianie wyjść 
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  EngineSetLeftSpeed(190);
  EngineSetRightSpeed(190);

  serwomechanizm.attach(25);
  serwomechanizm.write(45);

  pinMode(sensorPin, INPUT_PULLUP); // Ustawienie pinu czujnika jako wejście z wewnętrznym podciąganiem
  attachInterrupt(digitalPinToInterrupt(sensorPinRight), handleRightSensorInterrupt, RISING); // Przerwanie na zboczu opadającym
  attachInterrupt(digitalPinToInterrupt(sensorPinLeft), handleLeftSensorInterrupt, RISING);
}

long getDistance()
{
  long time, distance;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  time = pulseIn(echoPin, HIGH);
  distance = time / 58;

  return distance;
}

void loop() {
  if (pozycja <= 145 && pozycja >= 45) { //Jeśli pozycja mieści się w zakresie
    serwomechanizm.write(pozycja); //Wykonaj ruch
  } else { //Jeśli nie, to powrót na początek
    kierunek *= -1;
  }    
  
  pozycja = pozycja + (zmiana * kierunek); //Zwiększenie aktualnej pozycji serwa*/

  Serial.println(getDistance());
  
  if(turning != 1){
    EngineMoveForward();
  }
  else
  {
    delay(100);
    turning = 0;
    interrupts();
    EngineMoveForward();
        
  }
  
  /*delay(5000);
  EngineSoftStop();
  delay(2000);
 EngineMoveBackward();  
  delay(5000);
  EngineSoftStop();
  delay(1000);*/
delay(45);

}