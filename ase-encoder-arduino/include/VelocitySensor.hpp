#ifndef VELOCITY_SENSOR_HPP
#define VELOCITY_SENSOR_HPP

#include <Arduino.h>

class VelocitySensor {
public:
    enum Direction {
        FORWARD,
        BACKWARD
    };

    VelocitySensor(int pin, double wheelDiameter, int triggersPerRevolution);
    void begin();
    double getVelocity();
    void setDirection(Direction dir);
    void resetCount();
    unsigned long getCount();
    double getDistance();
    double getWheelCircumference();

    int getTriggersPerRevolution() {
        return triggersPerRevolution;
    }

private:
    int sensorPin;
    double wheelCircumference;
    int triggersPerRevolution;
    volatile unsigned long lastTriggerTime;
    volatile unsigned long triggerInterval;
    volatile unsigned long pulseCount;
    Direction direction;
    void IRAM_ATTR handleInterrupt();

    static void isrWrapper(void* instance);
};

#endif
