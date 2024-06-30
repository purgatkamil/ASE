#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#include "driver/mcpwm_prelude.h"

#include "esp_log.h"

static const char *TAG = "mcpwm-user";

static void enable_start_mcpwm_tim(mcpwm_timer_handle_t tim_h)
{
    ESP_LOGI(TAG, "Enable and start MCPWM timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(tim_h));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(tim_h, MCPWM_TIMER_START_NO_STOP));
}

static void disable_mcpwm_tim(mcpwm_timer_handle_t tim_h)
{
    ESP_LOGI(TAG, "Disable and stop MCPWM timer");
    mcpwm_timer_start_stop(tim_h, MCPWM_TIMER_START_STOP_FULL);
    ESP_ERROR_CHECK(mcpwm_timer_disable(tim_h));
}

static void create_tim_oper(mcpwm_timer_handle_t* tim_h, mcpwm_oper_handle_t* oper)
{
    ESP_LOGI(TAG, "Begin of MCPWM timer config");
    mcpwm_timer_config_t mcpwm_timer_conf = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, // 10 MHz -> 100ns
        .period_ticks = 10000,     // 10 000 ticks -> 1 kHz
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP};

    ESP_ERROR_CHECK(mcpwm_new_timer(&mcpwm_timer_conf, tim_h));
    ESP_LOGI(TAG, "Created MCPWM timer");

    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(*oper, *tim_h));
}

static void conf_mcpwm_gen_cmp(mcpwm_oper_handle_t oper, mcpwm_cmpr_handle_t *cmp_h, int gpio_num)
{
    ESP_LOGI(TAG, "Create comparator and generator from the operator");

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };

    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, cmp_h));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = gpio_num,
    };
    mcpwm_gen_handle_t generator = NULL;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *cmp_h, MCPWM_GEN_ACTION_LOW)));
}

void app_main()
{
    // Create the blink task
    // xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL);
    mcpwm_timer_handle_t tim_h = NULL;
    mcpwm_oper_handle_t oper = NULL;
    create_tim_oper(&tim_h, &oper);

    mcpwm_cmpr_handle_t cmp_hA = NULL;
    mcpwm_cmpr_handle_t cmp_hB = NULL;

    conf_mcpwm_gen_cmp(oper, &cmp_hA, GPIO_NUM_12);
    conf_mcpwm_gen_cmp(oper, &cmp_hB, GPIO_NUM_14);

    enable_start_mcpwm_tim(tim_h);

    int i = 0, j = 0;
    for (;;)
    {
        // set the initial compare value, so that the servo will spin to the center position
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_hA, 1000 + 6000 * i));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_hB, 1000 + 6000 * i++));
        if (i == 2)
            i -= 2;
        if (j++ == 5)
        {
            disable_mcpwm_tim(tim_h);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Delay for 1 second
    }
}
