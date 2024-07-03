#include "bluetooth_com.h"

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static QueueHandle_t bt_tosend_h;
static QueueHandle_t bt_rcv_h;
static uint32_t bt_con_handle;
static TaskHandle_t bt_task_handle;

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
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT (cong=%d)", param->write.cong);
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

void bluetooth_com_task(void *pvParameters)
{
    bt_task_handle = xTaskGetCurrentTaskHandle();

    char bda_str[18] = {0};
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    ///////////////////////////////////////
    // Receive messaging queues from param
    ///////////////////////////////////////
    bt_com_task_ctx_t *bt_ctx = (bt_com_task_ctx_t *)pvParameters;
    bt_rcv_h = bt_ctx->q_rcv_h;
    bt_tosend_h = bt_ctx->q_tosend_h;
    ///////////////////////////////////////

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
    bluedroid_cfg.ssp_en = false;
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
        .mode = esp_spp_mode,
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
        .tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
    };
    if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK)
    {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code = BT_PAIRING_PIN;
    esp_bt_gap_set_pin(pin_type, BT_PAIRING_PIN_LEN, pin_code);

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

    bt_con_status_t con_st = 0;
    uint32_t con_stat_notif = 0;
    static bt_com_msg_t tosend_msg;
    for (;;)
    {
        // Wait for connection and do not block if connection is established later
        if (xTaskNotifyWaitIndexed(BT_COM_NOTIF_CON_STATE_INDEX,
                                   0x00, 0x00, &con_stat_notif,
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

        if (xQueueReceive(bt_tosend_h, &tosend_msg, portMAX_DELAY) == pdTRUE)
        {
            // If disconnected while waiting for message to send
            // skip the iteration so connection notification is awaited
            if (con_st != BT_CON_CONNECTED)
            {
                continue;
            }

            // Wait for notification that new write can be started
            static uint32_t write_ready = 1;
            if (write_ready ||
                xTaskNotifyWaitIndexed(BT_COM_NOTIF_WRITE_READY_INDEX,
                                       0x00, ULONG_MAX, &write_ready, portMAX_DELAY) == pdTRUE)
            {
                
                ESP_ERROR_CHECK(
                    esp_spp_write(bt_con_handle, tosend_msg.len, tosend_msg.data));
                write_ready = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}