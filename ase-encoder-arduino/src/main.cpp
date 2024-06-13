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

// Servo
// #define SERVO_PIN 34

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
    unsigned long currentTime = millis();
    Serial.print(currentTime); // Print time
    Serial.print(", ");
    Serial.print(motorA.getVelocity()); // Print current velocity
    Serial.print(", ");
    Serial.print(motorA.getDistance()); // Print current distance
    Serial.print(", ");
    Serial.print(motorB.getVelocity()); // Print velocity PID output
    Serial.print(", ");
    Serial.print(motorB.getDistance()); // Print velocity PID output
    Serial.println();
}


void setup() {
    Serial.begin(115200);
    motorA.begin();
    motorB.begin();
    motorA.setVelocity(7); // Example target RPM for motor A
    motorB.setVelocity(7); // Example target RPM for motor B

    controlLoopTicker.attach_ms(5, controlLoop);
    printTicker.attach_ms(50, printStatus);
}

void loop() {
}
