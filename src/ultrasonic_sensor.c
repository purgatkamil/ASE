#include "ultrasonic_sensor.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/ledc.h>
#include <driver/gpio.h>
#include <driver/mcpwm_cap.h>
#include <esp_log.h>
#include <esp_private/esp_clk.h>

static const char *TAG = "ultrasonic-sensor";

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

static void ledc_trig_init(void)
{
    ESP_LOGI(TAG, "Initializing LEDC Timer");
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ESP_LOGI(TAG, "Initializing LEDC channel (output at GPIO %d)", ULTRASONIC_TRIG_GPIO);
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ULTRASONIC_TRIG_GPIO,
        .duty = LEDC_DUTY,
        .hpoint = 0};
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void ultrasonic_sensor_task()
{
    ESP_LOGI(TAG, "Install capture timer");
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

    ESP_LOGI(TAG, "Register capture callback");
    TaskHandle_t cur_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = ultrasonic_echo_callback,
    };

    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, cur_task));

    ESP_LOGI(TAG, "Enable capture channel");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

    ESP_LOGI(TAG, "Enable and start capture timer");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    // Configure LEDC peripherial to send TRIG signal
    ledc_trig_init();

    uint32_t tof_ticks;
    for (;;)
    {
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            float pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us > 35000)
            {
                // out of range
                continue;
            }
            // convert the pulse width into measure distance
            float distance = (float)pulse_width_us / 58;
            ESP_LOGI(TAG, "Measured distance: %.2fcm", distance);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}