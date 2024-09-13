#ifndef ASE_CONFIG_H
#define ASE_CONFIG_H

//////////////////////////////////////////
////////////// TELEPLOT //////////////////
//////////////////////////////////////////
// #define ENABLE_TELEPLOT_PRINTS
////////////////////////////////////////////////
/**********************************************/
////////////////////////////////////////////////

////////////////////////////////////////////////
////////////// MAIN TASK CONF //////////////////
////////////////////////////////////////////////
#define MAIN_TASK_LOG_TAG "orchestrator"

#define MAIN_MISSION_STATE_LOG_TAG "mission-state"
////////////////////////////////////////////////
/**********************************************/
////////////////////////////////////////////////

////////////////////////////////////////////////
////////// WANDER & AVOIDANCE //////////////////
////////////////////////////////////////////////
#define WANDER_LOG_TAG "wander & avoidance"
#define SCAN_MOVE_LOG_TAG "scan & move"

#define IR_TOP_LEFT_GPIO      GPIO_NUM_36
#define IR_TOP_RIGHT_GPIO     GPIO_NUM_39
#define IR_TOP_FRONT_GPIO     GPIO_NUM_34
#define IR_BOTTOM_LEFT_GPIO   GPIO_NUM_23
#define IR_BOTTOM_RIGHT_GPIO  GPIO_NUM_22
#define IR_BOTTOM_CENTER_GPIO GPIO_NUM_35
////////////////////////////////////////////////
/**********************************************/
////////////////////////////////////////////////

////////////////////////////////////////////////
///////////// META DETECT TASK /////////////////
////////////////////////////////////////////////
#define META_DETECTION_LOG_TAG  "meta-detector"
#define META_DETECTION_ADC_UNIT ADC_UNIT_2
#define META_DETECTION_ADC_CHAN ADC_CHANNEL_0

// How big difference in reading in regard to
// the average readings will trigger meta detection
#define META_DETECTION_TRIG_THRESHOLD 10
// How many samples should be averaged for computation
// of value that is not considered meta
#define META_DETECTION_AVG_SAMPLE_CNT 10
////////////////////////////////////////////////
/**********************************************/
////////////////////////////////////////////////

////////////////////////////////////////////////
////////////// MOTOR CONTROL ///////////////////
////////////////////////////////////////////////
#define MOTOR_CONTROL_LOG_TAG "motor-control"

#define MOTOR_LEFT_EN_GPIO  GPIO_NUM_14
#define MOTOR_LEFT_IN1_GPIO GPIO_NUM_27
#define MOTOR_LEFT_IN2_GPIO GPIO_NUM_26

#define MOTOR_RIGHT_EN_GPIO  GPIO_NUM_12
#define MOTOR_RIGHT_IN1_GPIO GPIO_NUM_32
#define MOTOR_RIGHT_IN2_GPIO GPIO_NUM_33

#define MCPWM_RESOLUTION_HZ 10000000 // 10 MHz -> 100ns time resolution
#define MCPWM_PERIOD_TICKS  10000    // 10 000 ticks * time resolution 100ns -> 1 kHz
////////////////////////////////////////////////
/**********************************************/
////////////////////////////////////////////////

////////////////////////////////////////////////
////////////// SONAR & SERVO ///////////////////
////////////////////////////////////////////////
#define SONAR_SERVO_LOG_TAG  "sonar-servo"
#define ULTRASONIC_TRIG_GPIO GPIO_NUM_2
#define ULTRASONIC_ECHO_GPIO GPIO_NUM_17
#define SERVO_PWM_GPIO       GPIO_NUM_25

#define SONAR_LEDC_TIMER     LEDC_TIMER_0
#define SONAR_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define SONAR_LEDC_CHANNEL   LEDC_CHANNEL_0
#define SONAR_LEDC_DUTY_RES  LEDC_TIMER_20_BIT
#define SONAR_LEDC_DUTY      (5 * 105) // Set duty to 10 us (0.0001 % * 2^20 ~ 105 at 10 Hz)
#define SONAR_LEDC_FREQUENCY (5 * 10)  // Frequency in Hertz

#define SERVO_LEDC_TIMER     LEDC_TIMER_1
#define SERVO_LEDC_MODE      LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL   LEDC_CHANNEL_1
#define SERVO_LEDC_DUTY_RES  LEDC_TIMER_13_BIT
#define SERVO_LEDC_FREQUENCY 100 // Frequency in Hertz

#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500 // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90  // Minimum angle
#define SERVO_MAX_DEGREE        90   // Maximum angle

// Time that is waited before taking measurement after commanding
// new servo position (to make sure servo has enough time to move)
#define DELAY_AFTER_SERVO_MOVEMENT_MS 70

// Maximum distance that is propagated further onto the system
// bigger values are still measured, but they are not passed
// to queue that is then processed in main task.
// This approach should minimise amount of glitched readings.
#define SONAR_MAX_DISTANCE_CM 300

// If not commented servo will be scanning, otherwise fixed to 0 degrees.
// #define ENABLE_SERVO_MOVEMENT
////////////////////////////////////////////////
/**********************************************/
////////////////////////////////////////////////

////////////////////////////////////////////////
////////////// BLUETOOTH ///////////////////////
////////////////////////////////////////////////
#define COMPILE_BLUETOOTH
#define SUPPORT_MACIEJ_APP
#define SPP_TAG         "bluetooth-com"
#define SPP_SERVER_NAME "SPP-SERVER-ASE1"
#define BT_DEVICE_NAME  "EIT-ASE-GR1🤖"

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
