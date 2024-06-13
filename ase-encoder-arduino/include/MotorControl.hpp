#ifndef MOTOR_CONTROL_HPP
#define MOTOR_CONTROL_HPP

#include <Arduino.h>
#include <PID_v1.h>
#include "VelocitySensor.hpp"

class MotorControl {
public:
    enum Direction {
        FORWARD,
        BACKWARD
    };

    MotorControl(int in1, int in2, int ena, int encoderPin, double wheelDiameter, int triggersPerRevolution);
    void begin();
    void setVelocity(double targetRPM);
    void setPosition(double targetDegrees);
    void update();
    double getVelocity();
    double getDistance();
    void setDirection(Direction dir);

private:
    int in1, in2, ena;
    double targetRPM;
    double currentRPM;
    double targetDegrees;
    double currentDegrees;

    double velocityInput, velocityOutput;
    double positionError;

    bool inPositionMode;

    PID velocityPID;
    VelocitySensor encoder;
    Direction direction;

    void setMotorSpeed(int speed);
};

#endif
