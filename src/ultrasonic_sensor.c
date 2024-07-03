#include "ultrasonic_sensor.h"

static bool ultrasonic_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t cap_val_begin_of_sample = 0;
    static uint32_t cap_val_end_of_sample = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    if (edata->cap_edge == MCPWM_CAP_EDGE_POS)
    {
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    }
    else
    {
        cap_val_end_of_sample = edata->cap_value;
        uint32_t ticks_diff = cap_val_end_of_sample - cap_val_begin_of_sample;
        xTaskNotifyFromISR(task_to_notify, ticks_diff, eSetValueWithOverwrite, &high_task_wakeup);
    }

    return high_task_wakeup == pdTRUE;
}

static void ledc_servo_sonar_init(void)
{
    /////////////////
    // Timers init //
    /////////////////
    ESP_LOGI(SONAR_SERVO_LOG_TAG, "Initializing ultrasonic LEDC timer");
    ledc_timer_config_t sonar_ledc_timer = {
        .speed_mode = SONAR_LEDC_MODE,
        .duty_resolution = SONAR_LEDC_DUTY_RES,
        .timer_num = SONAR_LEDC_TIMER,
        .freq_hz = SONAR_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&sonar_ledc_timer));

    ESP_LOGI(SONAR_SERVO_LOG_TAG, "Initializing servo LEDC timer");
    ledc_timer_config_t servo_ledc_timer = {
        .speed_mode = SERVO_LEDC_MODE,
        .duty_resolution = SERVO_LEDC_DUTY_RES,
        .timer_num = SERVO_LEDC_TIMER,
        .freq_hz = SERVO_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&servo_ledc_timer));

    ///////////////////
    // Channels init //
    ///////////////////
    ESP_LOGI(SONAR_SERVO_LOG_TAG, "Initializing LEDC ultrasonic trigger channel (output at GPIO %d)", ULTRASONIC_TRIG_GPIO);
    ledc_channel_config_t sonar_ledc_channel = {
        .speed_mode = SONAR_LEDC_MODE,
        .channel = SONAR_LEDC_CHANNEL,
        .timer_sel = SONAR_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ULTRASONIC_TRIG_GPIO,
        .duty = SONAR_LEDC_DUTY,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&sonar_ledc_channel));

    ESP_LOGI(SONAR_SERVO_LOG_TAG, "Initializing LEDC servo channel (output at GPIO %d)", SERVO_PWM_GPIO);
    ledc_channel_config_t servo_ledc_channel = {
        .speed_mode = SERVO_LEDC_MODE,
        .channel = SERVO_LEDC_CHANNEL,
        .timer_sel = SERVO_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_PWM_GPIO,
        .duty = 0,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&servo_ledc_channel));
}

static void inline init_start_mcpwm_capture()
{
    ESP_LOGI(SONAR_SERVO_LOG_TAG, "Install capture timer");
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_timer_conf = {
        .group_id = 0,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_timer_conf, &cap_timer));

    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf = {
        .gpio_num = ULTRASONIC_ECHO_GPIO,
        .prescale = 1,
        .flags = {
            .pos_edge = true,
            .neg_edge = true,
            .pull_up = false,
            .pull_down = true}};
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

    ESP_LOGI(SONAR_SERVO_LOG_TAG, "Register capture callback");
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = ultrasonic_echo_callback,
    };

    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_LOGI(SONAR_SERVO_LOG_TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(SONAR_SERVO_LOG_TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));
}

static inline uint32_t angle_to_duty(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

void ultrasonic_sensor_task(void *pvParameters)
{
    // Init and start mcpwm capture peripherial used to
    // measure pulse width that ultrasonic sensor returns
    init_start_mcpwm_capture();

    // Configure LEDC peripherial
    ledc_servo_sonar_init();

    // Get handle to main task to be notified with measurement values
    // TaskHandle_t main_task_h = (TaskHandle_t)pvParameters;
    QueueHandle_t sonar_queue_h = (QueueHandle_t)pvParameters;

    ultrasonic_measurement_t sonar_meas;

    uint32_t tof_ticks;
    uint32_t servo_duty = 0;
    int16_t angle = 0;

    for (;;)
    {
        // LEDC_TIMER_20_BIT // 2^20 = 1048576, 100 Hz -> T=10ms
        // Convert pulse width to duty cycle value
        servo_duty = (angle_to_duty(angle) * ((1 << SERVO_LEDC_DUTY_RES) - 1)) * SERVO_LEDC_FREQUENCY / 1000000;

        ESP_LOGI(SONAR_SERVO_LOG_TAG, "Setting servo angle of %d deg (ton=%luus)", angle, servo_duty);

        ESP_ERROR_CHECK(ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, servo_duty));
        ESP_ERROR_CHECK(ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL));

        // Wait a little before actually checking the result
        // because servo takes some time to rotate before it can reliably measure
        vTaskDelay(pdMS_TO_TICKS(DELAY_AFTER_SERVO_MOVEMENT_MS));

        // There is no point of advancing with servo movement if ultrasonic is not responding
        // block until ultrasonic is responding again.
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, portMAX_DELAY) == pdTRUE)
        {
            float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
            // convert the pulse width into measure distance
            float distance = (float)pulse_width_us / 58;
            if (distance > 150)
            {
                // out of range
                continue;
            }

            sonar_meas.angle = angle;
            sonar_meas.distance = distance;

            ESP_LOGI(SONAR_SERVO_LOG_TAG, "Ultrasonic measurement at angle %d deg: %fcm",
                     angle,
                     distance);

            if (xQueueSend(sonar_queue_h, &sonar_meas, pdMS_TO_TICKS(0)) != pdTRUE)
            {
                ESP_LOGE(SONAR_SERVO_LOG_TAG, "Failed to add sonar measurement to queue!");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(300));

#ifdef ENABLE_SERVO_MOVEMENT
        static int8_t angle_dir = 1;
        static const int16_t scan_range_one_way = 80;

        angle += 20 * angle_dir;

        // if (angle >= SERVO_MAX_DEGREE)
        if (angle >= scan_range_one_way)
        {
            // angle = SERVO_MAX_DEGREE;
            angle = scan_range_one_way;
            angle_dir *= -1;
        }
        // else if (angle <= SERVO_MIN_DEGREE)
        else if (angle <= -scan_range_one_way)
        {
            // angle = SERVO_MIN_DEGREE;
            angle = -scan_range_one_way;
            angle_dir *= -1;
        }
#endif

#ifndef ENABLE_SERVO_MOVEMENT
        angle = 0;
#endif
    }
}