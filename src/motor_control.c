#include "motor_control.h"

static const char *TAG = "mcpwm-user";

static void enable_start_mcpwm_tim(mcpwm_timer_handle_t tim_h)
{
    ESP_LOGI(TAG, "Enable and start MCPWM timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(tim_h));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(tim_h, MCPWM_TIMER_START_NO_STOP));
}

static void disable_mcpwm_tim(mcpwm_timer_handle_t tim_h)
{
    ESP_LOGI(TAG, "Stop and disable MCPWM timer");
    mcpwm_timer_start_stop(tim_h, MCPWM_TIMER_START_STOP_FULL);
    ESP_ERROR_CHECK(mcpwm_timer_disable(tim_h));
}

static void create_tim_oper(mcpwm_timer_handle_t *tim_h, mcpwm_oper_handle_t *oper)
{
    ESP_LOGI(TAG, "Begin of MCPWM timer config");
    mcpwm_timer_config_t mcpwm_timer_conf = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, // 10 MHz -> 100ns
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 10000, // 10 000 ticks -> 1 kHz
    };

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
    ESP_LOGI(TAG, "Create comparator and generator from the operator (GPIO %d)", gpio_num);

    mcpwm_comparator_config_t comparator_config = {
        .flags = {
            .update_cmp_on_tez = true,
        }};

    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, cmp_h));

    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = gpio_num,
    };
    mcpwm_gen_handle_t generator = NULL;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    ESP_LOGI(TAG, "Set generator action on timer and compare event (GPIO %d)", gpio_num);
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                                                              MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                                                                MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *cmp_h, MCPWM_GEN_ACTION_LOW)));
}

void motor_control_task()
{
    mcpwm_timer_handle_t tim_h = NULL;
    mcpwm_oper_handle_t oper = NULL;
    create_tim_oper(&tim_h, &oper);

    mcpwm_cmpr_handle_t cmp_hA = NULL;
    mcpwm_cmpr_handle_t cmp_hB = NULL;

    conf_mcpwm_gen_cmp(oper, &cmp_hA, GPIO_MOTOR_LEFT);
    conf_mcpwm_gen_cmp(oper, &cmp_hB, GPIO_MOTOR_RIGHT);

    enable_start_mcpwm_tim(tim_h);

    int i = 0, j = 0;
    for (;;)
    {
        uint32_t control_val = 1000 + 6000 * i++;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_hA, control_val));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_hB, control_val));
        if (i == 2)
            i -= 2;
        if (j++ == 10)
        {
            disable_mcpwm_tim(tim_h);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}