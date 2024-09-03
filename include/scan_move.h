#ifndef SCAN_MOVE_H
#define SCAN_MOVE_H

#include "freertos/FreeRTOS.h"
#include "ultrasonic_sensor.h"
#include "motor_control.h"

typedef struct {
    TaskHandle_t scn_mv_finished_task_h;
} scan_move_ctx_t;

// The function sends notification to start scan move
// manouver and block until it is finished by taking semaphore
// or when timeout is reached
void scan_move_start_and_wait(uint32_t timeout_ms);

void scan_move_task(void* pvParameters);

#endif