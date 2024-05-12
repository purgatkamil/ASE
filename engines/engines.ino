const int IN1 = 25;  // Pin IN1 dla kanału A
const int IN2 = 26;  // Pin IN2 dla kanału A
const int IN3 = 27;  // Pin IN3 dla kanału B
const int IN4 = 14;  // Pin IN4 dla kanału B
const int ENA = 12;  // pwm dla kanału A
const int ENB = 13;  // pwm dla kanału B

void EngineMoveForward(){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
}

void EngineMoveBackward(){
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
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
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
    analogWrite(ENA, 110);

}

void EngineSetRightSpeed(int leftSpeed)
{
    analogWrite(ENB, 147);
}

void setup()
{
    Serial.begin(9600);
    pinMode(IN1, OUTPUT); // ustawianie wyjść 
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);

  EngineSetLeftSpeed(10);
  EngineSetRightSpeed(10);
}

void loop()
{
    EngineMoveForward();
    delay(1000);
    EngineMoveBackward();
    delay(1000);  
}

