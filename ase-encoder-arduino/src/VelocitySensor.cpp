#include "VelocitySensor.hpp"

VelocitySensor::VelocitySensor(int pin, double diameter, int triggers)
    : sensorPin(pin), wheelCircumference(diameter * PI), triggersPerRevolution(triggers),
      lastTriggerTime(0), triggerInterval(0), pulseCount(0), direction(FORWARD) {}

void VelocitySensor::begin() {
    pinMode(sensorPin, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(sensorPin), isrWrapper, this, FALLING);
}

double VelocitySensor::getVelocity() {
    if (triggerInterval > 0) {
        double timePerRevolution = (triggerInterval * triggersPerRevolution) / 1000000.0;
        return wheelCircumference / timePerRevolution;
    } else {
        return 0.0;
    }
}

void VelocitySensor::setDirection(Direction dir) {
    direction = dir;
}

void VelocitySensor::resetCount() {
    pulseCount = 0;
}

unsigned long VelocitySensor::getCount() {
    return pulseCount;
}

double VelocitySensor::getDistance() {
    return (pulseCount / (double)triggersPerRevolution) * wheelCircumference;
}

double VelocitySensor::getWheelCircumference() {
    return wheelCircumference;
}

void IRAM_ATTR VelocitySensor::handleInterrupt() {
    unsigned long currentTime = micros();
    triggerInterval = currentTime - lastTriggerTime;
    lastTriggerTime = currentTime;

    // Increment or decrement pulse count based on direction
    if (direction == FORWARD) {
        pulseCount++;
    } else {
        pulseCount--;
    }
}

void VelocitySensor::isrWrapper(void* instance) {
    ((VelocitySensor*)instance)->handleInterrupt();
}
