#ifndef META_DETECTION_H
#define META_DETECTION_H

#include "freertos/FreeRTOS.h"

#include <esp_adc/adc_oneshot.h>
#include <esp_log.h>

#include "ase_config.h"

void meta_detection_task(void * pvParameters);

#endif