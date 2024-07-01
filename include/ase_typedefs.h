#ifndef ASE_TYPEDEFS_H
#define ASE_TYPEDEFS_H

// For types availability
#include "freertos/FreeRTOS.h"

typedef struct {
    int16_t angle;
    float distance;
} ultrasonic_measurement_t;

#endif