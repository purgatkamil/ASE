#include "motor_control.h"

static const char *TAG = "motor-control";

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
        .resolution_hz = MCPWM_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = MCPWM_PERIOD_TICKS,
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

static inline void setup_motors_dir_gpio()
{
    static gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << MOTOR_LEFT_IN1_GPIO) |
                        (1ULL << MOTOR_LEFT_IN2_GPIO) |
                        (1ULL << MOTOR_RIGHT_IN1_GPIO) |
                        (1ULL << MOTOR_RIGHT_IN2_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0};
    gpio_config(&io_conf);
}

// Based on docs:
// https://botland.com.pl/sterowniki-silnikow-moduly/3164-l298n-dwukanalowy-sterownik-silnikow-modul-12v-2a-5904422359317.html
static inline void motor_mode_set_cwise(gpio_num_t GPIO_IN1, gpio_num_t GPIO_IN2)
{
    ESP_ERROR_CHECK(gpio_set_level(GPIO_IN1, 1));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_IN2, 0));
}

static inline void motor_mode_set_ccwise(gpio_num_t GPIO_IN1, gpio_num_t GPIO_IN2)
{
    motor_mode_set_cwise(GPIO_IN2, GPIO_IN1);
}

// For now leaving it unimplemented
// void motor_mode_set_cwise(gpio_num_t GPIO_IN1, gpio_num_t GPIO_IN2)
// {

// }

// void motor_mode_set_cwise(gpio_num_t GPIO_IN1, gpio_num_t GPIO_IN2)
// {

// }

void motor_control_task()
{
    mcpwm_timer_handle_t tim_h = NULL;
    mcpwm_oper_handle_t oper = NULL;
    create_tim_oper(&tim_h, &oper);

    mcpwm_cmpr_handle_t cmp_hA = NULL;
    mcpwm_cmpr_handle_t cmp_hB = NULL;

    conf_mcpwm_gen_cmp(oper, &cmp_hA, MOTOR_LEFT_EN_GPIO);
    conf_mcpwm_gen_cmp(oper, &cmp_hB, MOTOR_RIGHT_EN_GPIO);

    setup_motors_dir_gpio();
    motor_mode_set_ccwise(MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO);
    motor_mode_set_ccwise(MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO);

    enable_start_mcpwm_tim(tim_h);

    // int i = 0, j = 0, dv = 0;
    for (;;)
    {
        //////////////////////////////
        ///////// Demo code //////////
        //////////////////////////////
        // uint32_t control_val = 5500 + 2000 * i++;
        // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_hA, control_val));
        // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(cmp_hB, control_val));
        // if (i == 2)
        // {
        //     i -= 2;
        // }

        // if (dv++ == 1)
        // {
        //     ESP_LOGI(TAG, "Setting CWISE");
        //     motor_mode_set_cwise(MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO);
        //     motor_mode_set_cwise(MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO);
        //     dv = 0;
        // }
        // else
        // {
        //     ESP_LOGI(TAG, "Setting CCWISE");
        //     motor_mode_set_ccwise(MOTOR_LEFT_IN1_GPIO, MOTOR_LEFT_IN2_GPIO);
        //     motor_mode_set_ccwise(MOTOR_RIGHT_IN1_GPIO, MOTOR_RIGHT_IN2_GPIO);
        // }

        // if (j++ == 10)
        // {
        //     disable_mcpwm_tim(tim_h);
        // }
        //////////////////////////////


        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}
