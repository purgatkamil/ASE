#ifndef ASE_CONFIG_H
#define ASE_CONFIG_H

////////////////////////////////////////////////
////////////// MAIN TASK CONF //////////////////
////////////////////////////////////////////////
#define MAIN_TASK_LOG_TAG "orchestrator"
////////////////////////////////////////////////
/**********************************************/
////////////////////////////////////////////////


////////////////////////////////////////////////
////////////// MOTOR CONTROL ///////////////////
////////////////////////////////////////////////
#define MOTOR_CONTROL_LOG_TAG "motor-control"
#define MOTOR_LEFT_EN_GPIO GPIO_NUM_14
#define MOTOR_LEFT_IN1_GPIO GPIO_NUM_27
#define MOTOR_LEFT_IN2_GPIO GPIO_NUM_26

#define MOTOR_RIGHT_EN_GPIO GPIO_NUM_12
#define MOTOR_RIGHT_IN1_GPIO GPIO_NUM_32
#define MOTOR_RIGHT_IN2_GPIO GPIO_NUM_33

#define MCPWM_RESOLUTION_HZ 10000000 // 10 MHz -> 100ns time resolution
#define MCPWM_PERIOD_TICKS 10000     // 10 000 ticks * time resolution 100ns -> 1 kHz
////////////////////////////////////////////////
/**********************************************/
////////////////////////////////////////////////

////////////////////////////////////////////////
////////////// LINE FOLLOWER ///////////////////
////////////////////////////////////////////////
#define LINE_FOLLOWER_LOG_TAG "line-follower"
#define IR_SENSOR_BOTTOM_LEFT_GPIO GPIO_NUM_23
#define IR_SENSOR_BOTTOM_RIGHT_GPIO GPIO_NUM_22
#define IR_SENSOR_BOTTOM_CENTER_GPIO GPIO_NUM_35

// Define motor control output range
#define MOTOR_MAX 0.9
#define MOTOR_MIN -0.9
#define MOTOR_START_THRESHOLD 0.3

// Dead time in which robot can not take another 90 deg turn.
// Units of time are milliseconds.
#define TURNING_DEAD_TIME_MS 5000
////////////////////////////////////////////////
/**********************************************/
////////////////////////////////////////////////

////////////////////////////////////////////////
////////////// SONAR & SERVO ///////////////////
////////////////////////////////////////////////
#define SONAR_SERVO_LOG_TAG "sonar-servo"
#define ULTRASONIC_TRIG_GPIO GPIO_NUM_2
#define ULTRASONIC_ECHO_GPIO GPIO_NUM_17
#define SERVO_PWM_GPIO GPIO_NUM_25

#define SONAR_LEDC_TIMER LEDC_TIMER_0
#define SONAR_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SONAR_LEDC_CHANNEL LEDC_CHANNEL_0
#define SONAR_LEDC_DUTY_RES LEDC_TIMER_20_BIT
#define SONAR_LEDC_DUTY (5 * 105)     // Set duty to 10 us (0.0001 % * 2^20 ~ 105 at 10 Hz)
#define SONAR_LEDC_FREQUENCY (5 * 10) // Frequency in Hertz

#define SERVO_LEDC_TIMER LEDC_TIMER_1
#define SERVO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL LEDC_CHANNEL_1
#define SERVO_LEDC_DUTY_RES LEDC_TIMER_13_BIT
#define SERVO_LEDC_FREQUENCY 100 // Frequency in Hertz

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE -180        // Minimum angle
#define SERVO_MAX_DEGREE 180         // Maximum angle

// Time that is waited before taking measurement after commanding
// new servo position (to make sure servo has enough time to move)
#define DELAY_AFTER_SERVO_MOVEMENT_MS 50

// Maximum distance that is propagated further onto the system
// bigger values are still measured, but they are not passed
// to queue that is then processed in main task.
// This approach should minimise amount of glitched readings.
#define SONAR_MAX_DISTANCE_CM 150

// If not commented servo will be scanning, otherwise fixed to 0 degrees.
// #define ENABLE_SERVO_MOVEMENT
////////////////////////////////////////////////
/**********************************************/
////////////////////////////////////////////////

////////////////////////////////////////////////
////////////// BLUETOOTH ///////////////////////
////////////////////////////////////////////////
#define SPP_TAG "bluetooth-com"
#define SPP_SERVER_NAME "SPP-SERVER-ASE1"
#define BT_DEVICE_NAME "EIT-ASE-GR1🤖"

// Max pin length is 16 characters
#define BT_PAIRING_PIN_LEN 4
#define BT_PAIRING_PIN \
    {                  \
        '1', '4', '3', '2'}

// ESP32 internal buffer has maximum cap of 990 bytes
// thus more can not be received at once with default configuration
#define BT_MSG_BUF_SIZE_BYTES 256

// If not commented, messages logged using ESP logging framework
// will be additionally sent to Bluetooth task's send queue.
#define LOG_OVER_BLUETOOTH
////////////////////////////////////////////////
/**********************************************/
////////////////////////////////////////////////

#endif
