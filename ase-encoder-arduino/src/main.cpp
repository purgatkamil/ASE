#include <Arduino.h>
#include "MotorControl.hpp"
#include <Ticker.h>

// Motor A
#define IN1 25
#define IN2 26
#define ENA 12
#define ENCODER_PIN_RIGHT 16

// Motor B
#define IN3 27
#define IN4 14
#define ENB 13
#define ENCODER_PIN_LEFT 17

const double wheelDiameter = 0.054; // Wheel diameter in meters
const int triggersPerRevolution = 360 / 18; // 20 triggers per revolution

MotorControl motorA(IN1, IN2, ENA, ENCODER_PIN_RIGHT, wheelDiameter, triggersPerRevolution);
MotorControl motorB(IN3, IN4, ENB, ENCODER_PIN_LEFT, wheelDiameter, triggersPerRevolution);

Ticker controlLoopTicker;
Ticker printTicker;

void controlLoop() {
    motorA.update();
    motorB.update();
}

void printStatus() {
    Serial.print("Motor A Position: ");
    Serial.print(motorA.getDistance());
    Serial.println(" meters");

    Serial.print("Motor B Position: ");
    Serial.print(motorB.getDistance());
    Serial.println(" meters");
}

void setup() {
    Serial.begin(115200);
    motorA.begin();
    motorB.begin();
    motorA.setPosition(360); 
    motorA.setDirection(MotorControl::FORWARD);
    motorB.setPosition(360);
    motorB.setDirection(MotorControl::FORWARD);

    controlLoopTicker.attach_ms(10, controlLoop);
    printTicker.attach_ms(500, printStatus);
}

void loop() {
}
