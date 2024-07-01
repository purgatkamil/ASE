#ifndef ASE_TYPEDEFS_H
#define ASE_TYPEDEFS_H

// For types availability
#include "freertos/FreeRTOS.h"

// Bitwise operations macros to easy
// set flags in motor control structure
#define SET_BIT(var, mask) ((var) |= (mask))
#define CLEAR_BIT(var, mask) ((var) &= ~(mask))
#define IS_BIT_SET(var, mask) (((var) & (mask)) != 0)

typedef struct
{
    int16_t angle;
    float distance;
} ultrasonic_measurement_t;

typedef enum
{
    MOTORS_CONTROL_FLAGS_ENABLE = (1 << 0),
} motors_control_mask;

typedef struct
{
    // Left and right speed are in range [-1; 1]
    double left;
    double right;
} motors_speed_t;

typedef struct
{
    motors_speed_t speed_cmd;
    uint8_t cmd_flags;
} motors_control_msg_t;

typedef enum
{
    IR_ISR_GPIO_BOT_LEFT_ID = 0u,
    IR_ISR_GPIO_BOT_CENTER_ID = 1u,
    IR_ISR_GPIO_BOT_RIGHT_ID = 2u,
} ir_isr_gpio_id;

typedef struct
{
    uint8_t gpio_id;
    gpio_num_t gpio_num;
} ir_isr_arg_t;

#endif