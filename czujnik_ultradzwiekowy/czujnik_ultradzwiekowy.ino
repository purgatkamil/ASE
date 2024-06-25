#include <ESP32Servo.h> //Biblioteka odpowiedzialna za serwa

const int sensorPinLeft = 23; //odbiciowy lewy
const int sensorPinRight = 22; //odbiciowy prawy

const int trigPin = 2; //trigger ultradzwieku
const int echoPin = 17; //echo ultradzwieku

const int IN1 = 33;  // Pin IN1 dla kanału A
const int IN2 = 32;  // Pin IN2 dla kanału A
const int IN3 = 27;  // Pin IN3 dla kanału B
const int IN4 = 26;  // Pin IN4 dla kanału B
const int ENA = 12;  // pwm dla kanału A
const int ENB = 14;  // pwm dla kanału B

const int hallSensor = 1;

const int servoPin = 25;

int silniki_kierunek = 1;
volatile static int turning = 0;

void IRAM_ATTR handleRightSensorInterrupt() {
    noInterrupts();
    EngineFastStop();
    EngineTurnRight();
    //zakretWLewoPoLuku();
    turning = 1;
}

void IRAM_ATTR handleLeftSensorInterrupt() {
    noInterrupts();
    EngineFastStop();
    EngineTurnLeft();
    //zakretWPrawoPoLuku();
    turning = 1;
}

void EngineMoveBackward(){
    silniki_kierunek = 1;
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void EngineMoveForward(){
    silniki_kierunek = -1;
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
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, HIGH);
}

void EngineTurnRight(){
    EngineSetLeftSpeed(210);
    EngineSetRightSpeed(210);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void EngineTurnLeft(){
    EngineSetLeftSpeed(210);
    EngineSetRightSpeed(210);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void EngineSetRightSpeed(int leftSpeed)
{
  int speed = leftSpeed;
  if(leftSpeed >= 255)
  {
    speed = 250;
  }

  analogWrite(ENA, speed);
}

void EngineSetLeftSpeed(int rightSpeed)
{
  int speed = rightSpeed;
  if(rightSpeed >= 255)
  {
    speed = 250;
  }
  
  speed = speed + (silniki_kierunek * 3);
  analogWrite(ENB, speed);
}

void zakretWLewoPoLuku()
{
    EngineSetRightSpeed(210); //glowne
    EngineSetLeftSpeed(210);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void zakretWPrawoPoLuku()
{
    EngineSetLeftSpeed(230);  //glowne
    EngineSetRightSpeed(180);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
}

void setServoAngle(int angle) {
  // Przeliczamy kąt na czas trwania impulsu
  // Serwomechanizmy zazwyczaj potrzebują impulsu od 1ms do 2ms
  // przy kątach od 0 do 180 stopni
  int pulseWidth = map(angle, 0, 180, 544, 2400); // Przeliczamy kąt na mikroseundy
  digitalWrite(servoPin, HIGH);
  delayMicroseconds(pulseWidth);
  digitalWrite(servoPin, LOW);
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

  pinMode(hallSensor, INPUT);

  pinMode(servoPin, OUTPUT);
  

  EngineSetLeftSpeed(220);
  EngineSetRightSpeed(210);

  //pinMode(sensorPinRight, INPUT_PULLUP);
  //pinMode(sensorPinLeft, INPUT_PULLUP);

  EngineMoveForward();
  //attachInterrupt(digitalPinToInterrupt(sensorPinRight), handleRightSensorInterrupt, RISING); // Przerwanie na zboczu opadającym
  //attachInterrupt(digitalPinToInterrupt(sensorPinLeft), handleLeftSensorInterrupt, RISING);
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

int checkHallSensor()
{
  int wartosc_analogowa = 0;
  wartosc_analogowa = analogRead(hallSensor);
  int result = abs(wartosc_analogowa - 512);
  
  return result;
}

int servo_pozycja = 0;
int servo_kierunek = 1;
int servo_zmiana = 10;
int distance = 0;

void loop() {
  EngineMoveForward();
  int right = digitalRead(sensorPinRight);
  int left = digitalRead(sensorPinLeft);

  if(left == LOW)
  {
      EngineSetLeftSpeed(250);
  }
  else
  {
      EngineSetLeftSpeed(225);    
  }

  if(right == LOW)
  {
      EngineSetRightSpeed(250);
  }
  else
  {
      EngineSetRightSpeed(205);    
  }

if (servo_pozycja >= 0 && servo_pozycja <= 180)
{
    setServoAngle(servo_pozycja);
    servo_pozycja += (servo_kierunek * servo_zmiana);
    if(getDistance() < 50)
    {
      EngineFastStop();      
    }

}
else
{
    servo_kierunek *= -1;
    servo_pozycja += (servo_kierunek * servo_zmiana);
    // Aby uniknąć wyjścia poza zakres po zmianie kierunku, można dodać korekcję
    if (servo_pozycja > 180)
    {
        servo_pozycja = 180;
    }
    else if (servo_pozycja < 0)
    {
        servo_pozycja = 0;
    }
}
 //Serial.print(getDistance());
  
 delay(30);


  /*if (servo_pozycja <= 180 && servo_pozycja >= 0) //Jeśli pozycja mieści się w zakresie
  { 
    serwomechanizm.write(servo_pozycja); //Wykonaj ruch
  }
  else //Jeśli nie, to powrót na początek
  {
    servo_kierunek *= -1;
  } */   
  
 // servo_pozycja = servo_pozycja + (servo_zmiana * servo_kierunek); //Zwiększenie aktualnej pozycji serwa

  //Serial.println(getDistance());
  

  
 /* if(turning != 1)
  {
    EngineMoveForward();
  }
  else
  {
    delay(50);
    turning = 0;  
    interrupts();
    EngineMoveForward(); 
  }*/

/*int checkHall = checkHallSensor();
if(checkHall > 20);
{
  EngineFastStop();
}*/
}