#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "main.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "ble_main.h"
#include "prf_uuids.h"
#include "svc_ota.h"

#define LOG_TAG "OTA"

#define SVC_OTA_VERSION     "V0.1"

enum
{
    OTA_IDX_SRV,
    OTA_IDX_VERSION_PROPS,
    OTA_IDX_VERSION_VAL,
    OTA_IDX_DATA_PROPS,
    OTA_IDX_DATA_VAL,
    OTA_IDX_DATA_CCC,
    OTA_IDX_CTRL_PROPS,
    OTA_IDX_CTRL_VAL,
    OTA_IDX_CTRL_CCC,
    OTA_IDX_NUM,
};

static const char svc_ota_version[] = SVC_OTA_VERSION;

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint16_t ota_srv_uuid = SVC_OTA_SERVICE_UUID;
static const uint16_t ota_version_uuid = SVC_OTA_CHAR_VERSION_UUID;
static const uint8_t ota_version_props = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint16_t ota_data_uuid = SVC_OTA_CHAR_DATA_UUID;
static const uint8_t ota_data_props = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t ota_data_ccc[2] = { 0, 0 };
static const uint16_t ota_ctrl_uuid = SVC_OTA_CHAR_CTRL_UUID;
static const uint8_t ota_ctrl_props = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t ota_ctrl_ccc[2] = { 0, 0 };
static uint16_t ota_srv_tbl[OTA_IDX_NUM];
static esp_gatt_if_t ota_gatts_if;
static bool ota_data_can_notify;
static bool ota_ctrl_can_notify;
static svc_ota_data_write_callback_t ota_data_write_callback;
static svc_ota_ctrl_write_callback_t ota_ctrl_write_callback;

static SemaphoreHandle_t ready_sem_handle;

#define ATTR_DESC(_auto_rsp, _uuid, _perm, _max_len, _len, _val) \
{ \
    .attr_control = \
    { \
        .auto_rsp = _auto_rsp \
    }, \
    .att_desc = \
    { \
        .uuid_length = ESP_UUID_LEN_16, \
        .uuid_p = (uint8_t *)&_uuid, \
        .perm = _perm, \
        .max_length = _max_len, \
        .length = _len, \
        .value = (uint8_t *)_val \
    } \
}

static const esp_gatts_attr_db_t ota_attr_db[OTA_IDX_NUM] =
{
    [OTA_IDX_SRV] = ATTR_DESC(ESP_GATT_AUTO_RSP, primary_service_uuid, ESP_GATT_PERM_READ,
                              sizeof(uint16_t), sizeof(ota_srv_uuid), &ota_srv_uuid),

    [OTA_IDX_VERSION_PROPS] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_declaration_uuid, ESP_GATT_PERM_READ,
                                        sizeof(uint8_t), sizeof(ota_version_props), &ota_version_props),

    [OTA_IDX_VERSION_VAL] = ATTR_DESC(ESP_GATT_AUTO_RSP, ota_version_uuid, ESP_GATT_PERM_READ,
                                      SVC_GATTS_PDU_MAX, sizeof(svc_ota_version), svc_ota_version),

    [OTA_IDX_DATA_PROPS] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_declaration_uuid, ESP_GATT_PERM_READ,
                                     sizeof(uint8_t), sizeof(ota_data_props), &ota_data_props),

    [OTA_IDX_DATA_VAL] = ATTR_DESC(ESP_GATT_AUTO_RSP, ota_data_uuid, ESP_GATT_PERM_WRITE,
                                   SVC_GATTS_PDU_MAX, 0, NULL),

    [OTA_IDX_DATA_CCC] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   sizeof(ota_data_ccc), sizeof(ota_data_ccc), ota_data_ccc),

    [OTA_IDX_CTRL_PROPS] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_declaration_uuid, ESP_GATT_PERM_READ,
                                     sizeof(uint8_t), sizeof(ota_ctrl_props), &ota_ctrl_props),

    [OTA_IDX_CTRL_VAL] = ATTR_DESC(ESP_GATT_AUTO_RSP, ota_ctrl_uuid, ESP_GATT_PERM_WRITE,
                                   SVC_GATTS_PDU_MAX, 0, NULL),

    [OTA_IDX_CTRL_CCC] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                   sizeof(ota_ctrl_ccc), sizeof(ota_ctrl_ccc), ota_ctrl_ccc),
};

static void ota_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
        {
            if (param->reg.app_id == APP_OTA_ID)
            {
                ota_gatts_if = gatts_if;
                esp_ble_gatts_create_attr_tab(ota_attr_db, gatts_if, OTA_IDX_NUM, 0);
            }
            break;
        }
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        {
            if (gatts_if == ota_gatts_if)
            {
                if (param->add_attr_tab.status != ESP_GATT_OK)
                {
                    ESP_LOGE(LOG_TAG, "create service failed, status %d", param->add_attr_tab.status);
                }
                else if (param->add_attr_tab.num_handle != OTA_IDX_NUM)
                {
                    ESP_LOGE(LOG_TAG, "create service abnormally handle number %d != %d", param->add_attr_tab.num_handle, OTA_IDX_NUM);
                }
                else
                {
                    memcpy(ota_srv_tbl, param->add_attr_tab.handles, sizeof(ota_srv_tbl));
                    esp_ble_gatts_start_service(ota_srv_tbl[OTA_IDX_SRV]);
                }
            }
            break;
        }
        case ESP_GATTS_START_EVT:
        {
            if (ready_sem_handle)
            {
                xSemaphoreGive(ready_sem_handle);
            }
            break;
        }
        case ESP_GATTS_READ_EVT:
        {
            break;
        }
        case ESP_GATTS_WRITE_EVT:
        {
            if (param->write.handle == ota_srv_tbl[OTA_IDX_DATA_CCC] && param->write.len == 2)
            {
                ota_data_can_notify = !!param->write.value[0];
                ESP_LOGI(LOG_TAG, "data notify %s", ota_data_can_notify ? "enabled" : "disabled");
                memcpy(ota_data_ccc, param->write.value, 2);
            }
            else if (param->write.handle == ota_srv_tbl[OTA_IDX_DATA_VAL])
            {
                TRY_CALL(ota_data_write_callback, param->write.value, param->write.len);
            }
            else if (param->write.handle == ota_srv_tbl[OTA_IDX_CTRL_CCC] && param->write.len == 2)
            {
                ota_ctrl_can_notify = !!param->write.value[0];
                ESP_LOGI(LOG_TAG, "ctrl notify %s", ota_ctrl_can_notify ? "enabled" : "disabled");
                memcpy(ota_ctrl_ccc, param->write.value, 2);
            }
            else if (param->write.handle == ota_srv_tbl[OTA_IDX_CTRL_VAL])
            {
                TRY_CALL(ota_ctrl_write_callback, param->write.value, param->write.len);
            }
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT:
        {
            ota_data_can_notify = false;
            ota_ctrl_can_notify = false;
            memset(ota_data_ccc, 0, 2);
            memset(ota_ctrl_ccc, 0, 2);
            break;
        }
        default:
            break;
    }
}

void svc_ota_init(void)
{
    ready_sem_handle = xSemaphoreCreateBinary();
    configASSERT(ready_sem_handle);

    ble_register_gatts_callback(APP_OTA_ID, ota_gatts_event_handler);
    esp_ble_gatts_app_register(APP_OTA_ID);

    if (xSemaphoreTake(ready_sem_handle, pdMS_TO_TICKS(1000)) != pdPASS)
    {
        ESP_LOGE(LOG_TAG, "create ota service failed");
    }
    vSemaphoreDelete(ready_sem_handle);
}

uint16_t svc_ota_get_gatts_if(void)
{
    return ota_gatts_if;
}

void svc_ota_data_notify(uint8_t *data, uint16_t length)
{
    if (ota_data_can_notify)
    {
        ble_send_notify(ota_gatts_if, ota_srv_tbl[OTA_IDX_DATA_VAL], data, length);
    }
}

void svc_ota_ctrl_notify(uint8_t *data, uint16_t length)
{
    if (ota_ctrl_can_notify)
    {
        ble_send_notify(ota_gatts_if, ota_srv_tbl[OTA_IDX_CTRL_VAL], data, length);
    }
}

void svc_ota_data_write_callback_register(svc_ota_data_write_callback_t callback)
{
    ota_data_write_callback = callback;
}

void svc_ota_ctrl_write_callback_register(svc_ota_ctrl_write_callback_t callback)
{
    ota_ctrl_write_callback = callback;
}
