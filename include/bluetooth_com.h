#ifndef BLUETOOTH_COM_H
#define BLUETOOTH_COM_H

#include "freertos/FreeRTOS.h"

#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "sys/time.h"
#include "time.h"

#include "ase_config.h"

typedef struct
{
    uint8_t data[BT_MSG_BUF_SIZE_BYTES];
    size_t  len;
} bt_com_msg_t;

typedef enum
{
    BT_CON_DISCONNECTED = 0,
    BT_CON_CONNECTED
} bt_con_status_t;

void start_bluetooth_task();

BaseType_t bluetooth_send(bt_com_msg_t *msg);

BaseType_t bluetooth_wait_for_msg(bt_com_msg_t *ret, uint32_t wait_ms);

void bluetooth_com_task(void *pvParameters);

#endif