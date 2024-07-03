#ifndef BLUETOOTH_COM_H
#define BLUETOOTH_COM_H

#include "freertos/FreeRTOS.h"

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "time.h"
#include "sys/time.h"

#include "ase_config.h"

typedef struct
{
    uint8_t data[BT_MSG_BUF_SIZE_BYTES];
    size_t len;
} bt_com_msg_t;

typedef struct
{
    QueueHandle_t q_tosend_h;
    QueueHandle_t q_rcv_h;
} bt_com_task_ctx_t;

typedef enum
{
    BT_CON_DISCONNECTED = 0,
    BT_CON_CONNECTED
} bt_con_status_t;

void bluetooth_com_task(void *pvParameters);

#endif