#include "bluetooth_com.h"

#define BT_COM_NOTIF_CON_STATE_INDEX   (UBaseType_t)0u
#define BT_COM_NOTIF_WRITE_READY_INDEX (UBaseType_t)1u

static const esp_spp_mode_t esp_spp_mode              = ESP_SPP_MODE_CB;
static const bool           esp_spp_enable_l2cap_ertm = true;

static const esp_spp_sec_t  sec_mask   = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static QueueHandle_t bt_tosend_h;
static QueueHandle_t bt_rcv_h;
static uint32_t      bt_con_handle;
static TaskHandle_t  bt_task_handle;

static char *bda2str(uint8_t *bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18)
    {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    // char bda_str[18] = {0};
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS)
        {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        }
        else
        {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;

    case ESP_SPP_CLOSE_EVT:
        xTaskNotify(bt_task_handle, BT_CON_DISCONNECTED, eSetValueWithOverwrite);
        break;

    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS)
        {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%" PRIu32 " sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_dev_set_device_name(BT_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        }
        else
        {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;

    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "Open event [%d]", event);
        if (param->open.status == ESP_SPP_SUCCESS)
        {
            // ESP_LOGI(SPP_TAG, "Succesfull BT connection (event [%d])", event);
            bt_con_handle = param->open.handle;
            xTaskNotifyIndexed(bt_task_handle, BT_COM_NOTIF_CON_STATE_INDEX, BT_CON_CONNECTED, eSetValueWithOverwrite);
        }
        break;

    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%" PRIu32,
                 param->data_ind.len, param->data_ind.handle);
        bt_com_msg_t bt_msg;
        // Copy at most BT_MSG_BUF_SIZE_BYTES bytes from incoming message
        // to prevent buffer overflow errors.
        size_t takeable_msg_len =
            param->data_ind.len > BT_MSG_BUF_SIZE_BYTES ? BT_MSG_BUF_SIZE_BYTES : param->data_ind.len;
        memcpy(bt_msg.data, param->data_ind.data,
               takeable_msg_len);
        bt_msg.len = takeable_msg_len;
        if (xQueueSend(bt_rcv_h, &bt_msg, pdMS_TO_TICKS(0)) != pdTRUE)
        {
            ESP_LOGE(SPP_TAG, "Failed to put into rcv queue!");
        }
        break;

    case ESP_SPP_WRITE_EVT:
        // ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT (cong=%d)", param->write.cong);
        if (param->write.cong == false)
        {
            xTaskNotifyIndexed(bt_task_handle, BT_COM_NOTIF_WRITE_READY_INDEX,
                               1, eSetValueWithOverwrite);
        }
        break;

    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event)
    {
    case ESP_BT_GAP_AUTH_CMPL_EVT:
    {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
            ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                     bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        else
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        break;
    }
    default:
        // ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
}

#ifdef LOG_OVER_BLUETOOTH
static int dual_vprintf(const char *fmt, va_list ap)
{
    static bt_com_msg_t bt_msg;
    bt_msg.len = vsnprintf((char *)bt_msg.data, BT_MSG_BUF_SIZE_BYTES, fmt, ap);
    if (bt_msg.len > 0)
    {
        if (bluetooth_send(&bt_msg) != pdTRUE)
        {
            // ESP_LOGE(MAIN_TASK_LOG_TAG, "Can't put msg to bluetooth LOG queue!");
        }
    }
    return vprintf(fmt, ap);
}
#endif

BaseType_t bluetooth_send(bt_com_msg_t *msg)
{
    return xQueueSend(bt_tosend_h, msg, 0);
}

BaseType_t bluetooth_wait_for_msg(bt_com_msg_t *ret, uint32_t wait_ms)
{
    return xQueueReceive(bt_rcv_h, ret, pdMS_TO_TICKS(wait_ms));
}

void start_bluetooth_task()
{
    xTaskCreatePinnedToCore(&bluetooth_com_task, "bt_com", 16384, NULL, 3, NULL, 0);
}

void bluetooth_com_task(void *pvParameters)
{
    ///////////////////////////////////////
    // Create queues for sending / receiving
    ///////////////////////////////////////
    bt_rcv_h    = xQueueCreate(5, sizeof(bt_com_msg_t));
    bt_tosend_h = xQueueCreate(40, sizeof(bt_com_msg_t));
    ///////////////////////////////////////

    vTaskDelay(pdMS_TO_TICKS(1500));

#ifdef LOG_OVER_BLUETOOTH
    // Disable BT_HCI logs as they are truly useless
    esp_log_level_set("BT_HCI", ESP_LOG_NONE);
    esp_log_level_set(SPP_TAG, ESP_LOG_NONE);
    esp_log_level_set(META_DETECTION_LOG_TAG, ESP_LOG_NONE);
    esp_log_level_set(SONAR_SERVO_LOG_TAG, ESP_LOG_NONE);
    esp_log_level_set("gpio", ESP_LOG_NONE);

    // Line below enables sending app logs over bluetooth
    esp_log_set_vprintf(dual_vprintf);
#endif

    bt_task_handle = xTaskGetCurrentTaskHandle();

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
    bluedroid_cfg.ssp_en                 = false;
    if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_spp_cfg_t bt_spp_cfg = {
        .mode              = esp_spp_mode,
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
        .tx_buffer_size    = 0, /* Only used for ESP_SPP_MODE_VFS mode */
    };
    if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code = BT_PAIRING_PIN;
    esp_bt_gap_set_pin(pin_type, BT_PAIRING_PIN_LEN, pin_code);

    char bda_str[18] = {0};
    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

    bt_con_status_t     con_st         = 0;
    uint32_t            con_stat_notif = 0;
    static bt_com_msg_t tosend_msg;
    for (;;)
    {
        // Wait for connection and do not block if connection is established later
        if (xTaskNotifyWaitIndexed(BT_COM_NOTIF_CON_STATE_INDEX,
                                   0, 0, &con_stat_notif,
                                   con_st == BT_CON_CONNECTED ? pdMS_TO_TICKS(0) : portMAX_DELAY) == pdTRUE)
        {
            con_st = con_stat_notif;
            // If disconnected go back to beginning of the loop and wait
            // for notification with successfull connection
            ESP_LOGI(SPP_TAG, "BT connection status changed: [con_st=%d]", con_st);
            if (con_st != BT_CON_CONNECTED)
            {
                continue;
            }
        }

        // Wait for notification that new write can be started
        static uint32_t write_ready = 1;
        if (write_ready ||
            xTaskNotifyWaitIndexed(BT_COM_NOTIF_WRITE_READY_INDEX,
                                   0, ULONG_MAX, &write_ready, pdMS_TO_TICKS(1000)) == pdTRUE)
        {
            if (xQueueReceive(bt_tosend_h, &tosend_msg, pdMS_TO_TICKS(1000)) == pdTRUE)
            {
                // If disconnected while waiting for message to send
                // skip the iteration so connection notification is awaited
                if (con_st != BT_CON_CONNECTED)
                {
                    // Add the message back to the queue so it can be sent
                    // when connection is regained
                    // Perform insertion without waiting as nothing will
                    // read from the queue in the meantime rendering deadlock
                    xQueueSendToFront(bt_tosend_h, &tosend_msg, 0);
                    // continue;
                }

                ESP_ERROR_CHECK(
                    esp_spp_write(bt_con_handle, tosend_msg.len, tosend_msg.data));
                write_ready = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}