#include <Servo.h> //Biblioteka odpowiedzialna za serwa
 
#define trigPin 12
#define echoPin 11

Servo serwomechanizm;  //Tworzymy obiekt, dzięki któremu możemy odwołać się do serwa 
int pozycja = 0; //Aktualna pozycja serwa 0-180
int zmiana = 24; //Co ile ma się zmieniać pozycja serwa?

void setup() {
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT); //Pin, do którego podłączymy trig jako wyjście
  pinMode(echoPin, INPUT); //a echo, jako wejście

  serwomechanizm.attach(9);
}

void loop() {
  long czas, dystans;

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  czas = pulseIn(echoPin, HIGH);
  dystans = czas / 58;
  
  Serial.print(dystans);
  Serial.println(" cm");

  if (pozycja < 180) { //Jeśli pozycja mieści się w zakresie
    serwomechanizm.write(pozycja); //Wykonaj ruch
  } else { //Jeśli nie, to powrót na początek
    pozycja = 0;
  }    
  
  pozycja = pozycja + zmiana; //Zwiększenie aktualnej pozycji serwa
  
  delay(30);
}