#include "meta_detection.h"

void meta_detection_task(void *pvParameters)
{
    TaskHandle_t main_task_h = (TaskHandle_t)pvParameters;

    //-------------ADC2 Init---------------//
    adc_oneshot_unit_handle_t   adc2_handle;
    adc_oneshot_unit_init_cfg_t init_config2 = {
        .unit_id  = META_DETECTION_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

    //-------------ADC2 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, META_DETECTION_ADC_CHAN, &config));

    ESP_LOGI(META_DETECTION_LOG_TAG,
             "ADC initialized [unit=%d, channel=%d]",
             META_DETECTION_ADC_UNIT,
             META_DETECTION_ADC_CHAN);

    int     adc_out_raw       = 0;
    int     readings_average  = 0;
    size_t  i                 = 0;
    uint8_t certainty_measure = 0;
    while (1)
    {
        readings_average = 0;
        for (i = 0; i < META_DETECTION_AVG_SAMPLE_CNT; i++)
        {
            ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, META_DETECTION_ADC_CHAN, &adc_out_raw));
            readings_average += adc_out_raw;
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        readings_average /= (double)META_DETECTION_AVG_SAMPLE_CNT;
        ESP_LOGI(META_DETECTION_LOG_TAG, "Average: %d", readings_average);
        certainty_measure = 0;
        // Measure several times to make sure its really out of pocket
        for (i = 0; i < 4; i++)
        {
            ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, META_DETECTION_ADC_CHAN, &adc_out_raw));
            if (abs(adc_out_raw - readings_average) >= META_DETECTION_TRIG_THRESHOLD)
                certainty_measure++;
            vTaskDelay(pdMS_TO_TICKS(40));
        }

        if (certainty_measure > 3)
        {
            xTaskNotifyIndexed(main_task_h, MAIN_META_DETECTION_NOTIF_IDX, 0x0, eNoAction);
            ESP_LOGI(META_DETECTION_LOG_TAG,
                     "Meta detected! [average=%d, reading=%d]",
                     readings_average,
                     adc_out_raw);
        }

        // vTaskDelay(pdMS_TO_TICKS(40));
    }

    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
}