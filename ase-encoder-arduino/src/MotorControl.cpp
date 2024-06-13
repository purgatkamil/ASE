#include "MotorControl.hpp"

MotorControl::MotorControl(int in1, int in2, int ena, int encoderPin, double wheelDiameter, int triggersPerRevolution)
    : in1(in1), in2(in2), ena(ena), targetRPM(0), currentRPM(0), targetDegrees(0), currentDegrees(0),
      velocityPID(&currentRPM, &velocityOutput, &targetRPM, 20, 2, 4.5, DIRECT),
      encoder(encoderPin, wheelDiameter, triggersPerRevolution), direction(FORWARD) {
    // velocityPID.SetOutputLimits(-255,255);
    velocityPID.SetSampleTime(5);
}

void MotorControl::begin() {
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(ena, OUTPUT);
    encoder.begin();
    velocityPID.SetMode(AUTOMATIC);
}

void MotorControl::setVelocity(double targetRPM) {
    this->targetRPM = targetRPM;
    this->inPositionMode = false;
    velocityPID.SetMode(AUTOMATIC);
}

void MotorControl::setPosition(double targetDegrees) {
    this->targetDegrees = targetDegrees;
    this->inPositionMode = true;
}

void MotorControl::update() {
    // Update current RPM and position
    currentRPM = encoder.getVelocity() * 60; // convert from m/s to RPM
    currentDegrees = (encoder.getCount() / (double)encoder.getTriggersPerRevolution()) * 360;

    // Calculate position error and set it as targetRPM for velocity PID
    // if (this->inPositionMode) {
    //     positionError = targetDegrees - currentDegrees;
    //     targetRPM = positionError * 0.15; // Proportional control for position (P controller)
    // }

    // Compute new velocity output
    velocityPID.Compute();

    if (abs(velocityOutput) < 30) {
        velocityOutput = 0;
        currentRPM = 0;
    }

    // Set motor speed based on velocity PID output
    setMotorSpeed(velocityOutput);
}

void MotorControl::setDirection(Direction dir) {
    direction = dir;
    encoder.setDirection(dir == FORWARD ? VelocitySensor::FORWARD : VelocitySensor::BACKWARD);
}

void MotorControl::setMotorSpeed(int speed) {
    if (direction == FORWARD) {
        if (speed > 0) {
            digitalWrite(in1, HIGH);
            digitalWrite(in2, LOW);
        } else {
            // digitalWrite(in1, LOW);
            // digitalWrite(in2, HIGH);
            // speed = -speed;
        }
    } else { // direction == BACKWARD
        if (speed > 0) {
            digitalWrite(in1, LOW);
            digitalWrite(in2, HIGH);
        } else {
            // digitalWrite(in1, HIGH);
            // digitalWrite(in2, LOW);
            // speed = -speed;
        }
    }
    analogWrite(ena, speed);
}

double MotorControl::getVelocity() {
    return currentRPM;
}

double MotorControl::getDistance() {
    return (encoder.getCount() / (double)encoder.getTriggersPerRevolution()) *360;// * encoder.getWheelCircumference();
}
