// Based on:
// https://github.com/espressif/esp-idf/blob/0479494e7abe5aef71393fba2e184b3a78ea488f/examples/peripherals/mcpwm/mcpwm_capture_hc_sr04/main/mcpwm_capture_hc_sr04.c

#include <driver/gpio.h>
#include <driver/mcpwm_cap.h>
#include <esp_private/esp_clk.h>

#include "Ticker.h"

#define HC_SR04_TRIG_GPIO GPIO_NUM_2
#define HC_SR04_ECHO_GPIO GPIO_NUM_17

const static char *TAG = "ASE_LOGGING";

Ticker trig_signal_gen_scheduler;
volatile double ultrasonic_distance = 0;

static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data) {
  static uint32_t cap_val_begin_of_sample = 0;
  static uint32_t cap_val_end_of_sample = 0;

  if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
    // store the timestamp when pos edge is detected
    cap_val_begin_of_sample = edata->cap_value;
    cap_val_end_of_sample = cap_val_begin_of_sample;
  } else {
    cap_val_end_of_sample = edata->cap_value;
    uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;
    double pulse_width_us = tof_ticks * (1000000.0 / esp_clk_apb_freq());
    ultrasonic_distance = pulse_width_us / 58.0;
  }

  return false;
}

void gen_ultrasonic_trig()
{
  digitalWrite(HC_SR04_TRIG_GPIO, LOW);
  delayMicroseconds(2);
  digitalWrite(HC_SR04_TRIG_GPIO, HIGH);
  delayMicroseconds(10);
  digitalWrite(HC_SR04_TRIG_GPIO, LOW);
}

void setup(void) {
  Serial.begin(115200);
  ESP_LOGI(TAG, "Install capture timer");
  mcpwm_cap_timer_handle_t cap_timer = NULL;
  mcpwm_capture_timer_config_t cap_timer_conf = {
    .group_id = 0,
    .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
  };
  ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_timer_conf, &cap_timer));

  mcpwm_cap_channel_handle_t cap_chan = NULL;
  mcpwm_capture_channel_config_t cap_ch_conf = {
    .gpio_num = HC_SR04_ECHO_GPIO,
    .prescale = 1,
    .flags = {
      .pos_edge = true,
      .neg_edge = true,
      .pull_up = true,
    }
  };
  ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_ch_conf, &cap_chan));

  ESP_LOGI(TAG, "Register capture callback");
  mcpwm_capture_event_callbacks_t cbs = {
    .on_cap = hc_sr04_echo_callback,
  };

  ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &cbs, NULL));

  ESP_LOGI(TAG, "Enable capture channel");
  ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));

  ESP_LOGI(TAG, "Configure Trig pin");
  gpio_config_t io_conf = {
    .pin_bit_mask = 1ULL << HC_SR04_TRIG_GPIO,
    .mode = GPIO_MODE_OUTPUT,
  };
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  // drive low by default
  ESP_ERROR_CHECK(gpio_set_level(HC_SR04_TRIG_GPIO, 0));

  ESP_LOGI(TAG, "Enable and start capture timer");
  ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
  ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

  trig_signal_gen_scheduler.attach_ms(100, gen_ultrasonic_trig);
}

void loop() {
  Serial.print("Current distance: ");
  Serial.print(ultrasonic_distance);
  Serial.println(" cm");
}