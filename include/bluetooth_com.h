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

#define SPP_TAG "bluetooth com"
#define SPP_SERVER_NAME "SPP-SERVER-ASE1"
#define BT_DEVICE_NAME "EIT-ASE-GR1🤖"

// Max pin length is 16 characters
#define BT_PAIRING_PIN_LEN 4
#define BT_PAIRING_PIN     \
    {                      \
        '1', '4', '3', '2' \
    }

// ESP32 internal buffer has maximum cap of 990 bytes
// thus more can not be received at once with default configuration
#define BT_MSG_BUF_SIZE_BYTES 256

#define BT_COM_NOTIF_CON_STATE_INDEX (UBaseType_t)0u
#define BT_COM_NOTIF_WRITE_READY_INDEX (UBaseType_t)1u

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