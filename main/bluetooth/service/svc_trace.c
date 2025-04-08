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
#include "svc_trace.h"

#define LOG_TAG "TRACE"

enum
{
    TRACE_IDX_SRV,
    TRACE_IDX_LOG_PROPS,
    TRACE_IDX_LOG_VAL,
    TRACE_IDX_LOG_CCC,
    TRACE_IDX_SHELL_PROPS,
    TRACE_IDX_SHELL_VAL,
    TRACE_IDX_SHELL_CCC,
    TRACE_IDX_LOCAL_PROPS,
    TRACE_IDX_LOCAL_VAL,
    TRACE_IDX_LOCAL_CCC,
    TRACE_IDX_NUM,
};

static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint16_t trace_srv_uuid = SVC_TRACE_SERVICE_UUID;
static const uint16_t trace_log_uuid = SVC_TRACE_CHAR_LOG_UUID;
static const uint8_t trace_log_props = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint16_t trace_shell_uuid = SVC_TRACE_CHAR_SHELL_UUID; //shell的特征
static const uint8_t trace_shell_props = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint16_t trace_local_uuid = SVC_TRACE_CHAR_LOCAL_UUID; //ESP32发给手机的日志
static const uint8_t trace_local_props = ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static uint8_t trace_log_ccc[2] = { 0, 0 };
static uint8_t trace_shell_ccc[2] = { 0, 0 };
static uint8_t trace_local_ccc[2] = { 0, 0 };
static uint16_t trace_srv_tbl[TRACE_IDX_NUM];
static esp_gatt_if_t trace_gatts_if;
static bool trace_log_can_notify;
static bool trace_shell_can_notify;
static bool trace_local_can_notify;
static svc_trace_shell_write_callback_t trace_shell_write_callback;

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

static const esp_gatts_attr_db_t trace_attr_db[TRACE_IDX_NUM] =
{
    [TRACE_IDX_SRV] = ATTR_DESC(ESP_GATT_AUTO_RSP, primary_service_uuid, ESP_GATT_PERM_READ,
                                sizeof(uint16_t), sizeof(trace_srv_uuid), &trace_srv_uuid),

    [TRACE_IDX_LOG_PROPS] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_declaration_uuid, ESP_GATT_PERM_READ,
                                      sizeof(uint8_t), sizeof(trace_log_props), &trace_log_props),

    [TRACE_IDX_LOG_VAL] = ATTR_DESC(ESP_GATT_AUTO_RSP, trace_log_uuid, ESP_GATT_PERM_WRITE,
                                    SVC_GATTS_PDU_MAX, 0, NULL),

    [TRACE_IDX_LOG_CCC] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                    sizeof(trace_log_ccc), sizeof(trace_log_ccc), trace_log_ccc),

    [TRACE_IDX_SHELL_PROPS] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_declaration_uuid, ESP_GATT_PERM_READ,
                                        sizeof(uint8_t), sizeof(trace_shell_props), &trace_shell_props),

    [TRACE_IDX_SHELL_VAL] = ATTR_DESC(ESP_GATT_AUTO_RSP, trace_shell_uuid, ESP_GATT_PERM_WRITE,
                                      SVC_GATTS_PDU_MAX, 0, NULL),

    [TRACE_IDX_SHELL_CCC] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                      sizeof(trace_shell_ccc), sizeof(trace_shell_ccc), trace_shell_ccc),

    [TRACE_IDX_LOCAL_PROPS] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_declaration_uuid, ESP_GATT_PERM_READ,
                                        sizeof(uint8_t), sizeof(trace_local_props), &trace_local_props),

    [TRACE_IDX_LOCAL_VAL] = ATTR_DESC(ESP_GATT_AUTO_RSP, trace_local_uuid, ESP_GATT_PERM_WRITE,
                                      SVC_GATTS_PDU_MAX, 0, NULL),

    [TRACE_IDX_LOCAL_CCC] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                      sizeof(trace_local_ccc), sizeof(trace_local_ccc), trace_local_ccc),
};

static void trace_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:
        {
            if (param->reg.app_id == APP_TRACE_ID)
            {
                trace_gatts_if = gatts_if;
                esp_ble_gatts_create_attr_tab(trace_attr_db, gatts_if, TRACE_IDX_NUM, 0);
            }
            break;
        }
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        {
            if (gatts_if == trace_gatts_if)
            {
                if (param->add_attr_tab.status != ESP_GATT_OK)
                {
                    ESP_LOGE(LOG_TAG, "create service failed, status %d", param->add_attr_tab.status);
                }
                else if (param->add_attr_tab.num_handle != TRACE_IDX_NUM)
                {
                    ESP_LOGE(LOG_TAG, "create service abnormally handle number %d != %d", param->add_attr_tab.num_handle, TRACE_IDX_NUM);
                }
                else
                {
                    memcpy(trace_srv_tbl, param->add_attr_tab.handles, sizeof(trace_srv_tbl));
                    esp_ble_gatts_start_service(trace_srv_tbl[TRACE_IDX_SRV]);
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
            if (param->write.handle == trace_srv_tbl[TRACE_IDX_LOG_CCC] && param->write.len == 2)
            {
                trace_log_can_notify = !!param->write.value[0];
                ESP_LOGI(LOG_TAG, "log notify %s", trace_log_can_notify ? "enabled" : "disabled");
                memcpy(trace_log_ccc, param->write.value, 2);
            }
            else if (param->write.handle == trace_srv_tbl[TRACE_IDX_SHELL_CCC] && param->write.len == 2)
            {
                trace_shell_can_notify = !!param->write.value[0];
                ESP_LOGI(LOG_TAG, "shell notify %s", trace_shell_can_notify ? "enabled" : "disabled");
                memcpy(trace_shell_ccc, param->write.value, 2);
            }
            else if (param->write.handle == trace_srv_tbl[TRACE_IDX_LOCAL_CCC] && param->write.len == 2)
            {
                trace_local_can_notify = !!param->write.value[0];
                ESP_LOGI(LOG_TAG, "local notify %s", trace_local_can_notify ? "enabled" : "disabled");
                memcpy(trace_local_ccc, param->write.value, 2);
            }

            else if (param->write.handle == trace_srv_tbl[TRACE_IDX_SHELL_VAL])
            {
                TRY_CALL(trace_shell_write_callback, param->write.value, param->write.len);
            }
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT:
        {
            trace_log_can_notify = false;
            memset(trace_log_ccc, 0, 2);
            trace_shell_can_notify = false;
            memset(trace_shell_ccc, 0, 2);
            trace_local_can_notify = false;
            memset(trace_local_ccc, 0, 2);
            break;
        }
        default:
            break;
    }
}

void svc_trace_init(void)
{
    ready_sem_handle = xSemaphoreCreateBinary();
    configASSERT(ready_sem_handle);

    ble_register_gatts_callback(APP_TRACE_ID, trace_gatts_event_handler);
    esp_ble_gatts_app_register(APP_TRACE_ID);

    if (xSemaphoreTake(ready_sem_handle, pdMS_TO_TICKS(1000)) != pdPASS)
    {
        ESP_LOGE(LOG_TAG, "create trace service failed");
    }
    vSemaphoreDelete(ready_sem_handle);
}

uint16_t svc_trace_get_gatts_if(void)
{
    return trace_gatts_if;
}

void svc_trace_log_notify(uint8_t *data, uint16_t length)
{
    if (trace_log_can_notify)
    {
        ble_send_notify(trace_gatts_if, trace_srv_tbl[TRACE_IDX_LOG_VAL], data, length);
    }
}

void svc_trace_shell_notify(uint8_t *data, uint16_t length)
{
    if (trace_shell_can_notify)
    {
        ble_send_notify(trace_gatts_if, trace_srv_tbl[TRACE_IDX_SHELL_VAL], data, length);
    }
}

void svc_trace_shell_write_callback_register(svc_trace_shell_write_callback_t callback)
{
    trace_shell_write_callback = callback;
}

void svc_trace_local_notify(uint8_t *data, uint16_t length)
{
    if (trace_local_can_notify)
    {
        ble_send_notify(trace_gatts_if, trace_srv_tbl[TRACE_IDX_LOCAL_VAL], data, length);
    }
}
