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
#include "svc_ctrl.h"

#define LOG_TAG "CTRL"

#define SVC_CTRL_VERSION     "V0.1"
 
 
 //服务表 描述的特征值
enum
{
    CTRL_IDX_SRV,
    CTRL_IDX_VERSION_PROP, //以下是VERSION_ 特征：版本号
    CTRL_IDX_VERSION_VAL,
    CTRL_IDX_NAME_PROP,    //以下是 NAME 特征：名称
    CTRL_IDX_NAME_VAL,
    CTRL_IDX_NAME_CCC, //当如果特征可以发notify的时候 会向这个属性写入00 01 的数据  这个属性会接收两个字节的数据
    CTRL_IDX_FIND_PROP,     //以下是 FIND 特征：查找
    CTRL_IDX_FIND_VAL,
    CTRL_IDX_FIND_CCC,
    CTRL_IDX_MISC_PROP,     //以下是 MISC 特征：杂项控制（自定义）
    CTRL_IDX_MISC_VAL,
    CTRL_IDX_MISC_CCC,
    CTRL_IDX_NUM,
};

static const char svc_ctrl_version[] = SVC_CTRL_VERSION;

//基础服务的设置   UUID
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

//给基础服务的特征设置具体的UUID——属性
static const uint16_t ctrl_srv_uuid = SVC_CTRL_SERVICE_UUID;
static const uint16_t ctrl_version_uuid = SVC_CTRL_CHAR_VERSION_UUID;
static const uint8_t ctrl_version_props = ESP_GATT_CHAR_PROP_BIT_READ; //给version特征设置只读的属性
static const uint16_t ctrl_name_uuid = SVC_CTRL_CHAR_NAME_UUID;
static const uint8_t ctrl_name_props = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY; //回应
static const uint16_t ctrl_find_uuid = SVC_CTRL_CHAR_FIND_UUID;
static const uint8_t ctrl_find_props = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint16_t ctrl_misc_uuid = SVC_CTRL_CHAR_MISC_UUID;
static const uint8_t ctrl_misc_props = ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_NOTIFY;

//cc属性值  CCC描述符是一个16位的值，用于配置客户端如何接收来自服务器的特征通知和指示
static uint8_t ctrl_name_ccc[2] = { 0, 0 }; //默认为0（用下面的布尔值） 不可以通知
static uint8_t ctrl_find_ccc[2] = { 0, 0 };
static uint8_t ctrl_misc_ccc[2] = { 0, 0 };

static uint16_t ctrl_srv_tbl[CTRL_IDX_NUM];
static esp_gatt_if_t ctrl_gatts_if;
static bool ctrl_name_can_notify; //与上面的变量是关联的  0和1用的布尔值
static bool ctrl_find_can_notify;
static bool ctrl_misc_can_notify;

//代码注册到协议栈中 属于应用程序  与  协议栈  之间的过度程序
static svc_ctrl_name_callback_t ctrl_name_write_callback; //特征回调：手机给NAME特征写数据的时候调用 作用：例如修改蓝牙名称 该函数就在应用程序中修改蓝牙名称
static svc_ctrl_find_callback_t ctrl_find_write_callback;//手机端写入特征值后，就执行应用程序：往STM32发送自定义的find命令
static svc_ctrl_misc_callback_t ctrl_misc_write_callback;//手机端写入特征值后，就执行应用程序：往STM32发送重启命令

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

//配置表
static const esp_gatts_attr_db_t ctrl_attr_db[CTRL_IDX_NUM] =
{
    [CTRL_IDX_SRV] = ATTR_DESC(ESP_GATT_AUTO_RSP, primary_service_uuid, ESP_GATT_PERM_READ,
                              sizeof(uint16_t), sizeof(ctrl_srv_uuid), &ctrl_srv_uuid),

    [CTRL_IDX_VERSION_PROP] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_declaration_uuid, ESP_GATT_PERM_READ,
                                       sizeof(uint8_t), sizeof(ctrl_version_props), &ctrl_version_props),

    [CTRL_IDX_VERSION_VAL] = ATTR_DESC(ESP_GATT_AUTO_RSP, ctrl_version_uuid, ESP_GATT_PERM_READ,
                                      SVC_GATTS_PDU_MAX, sizeof(svc_ctrl_version), svc_ctrl_version),

    [CTRL_IDX_NAME_PROP] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_declaration_uuid, ESP_GATT_PERM_READ,
                                     sizeof(uint8_t), sizeof(ctrl_name_props), &ctrl_name_props),

    [CTRL_IDX_NAME_VAL] = ATTR_DESC(ESP_GATT_AUTO_RSP, ctrl_name_uuid, ESP_GATT_PERM_WRITE,
                                    SVC_GATTS_PDU_MAX, 0, NULL),

    [CTRL_IDX_NAME_CCC] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                    sizeof(ctrl_name_ccc), sizeof(ctrl_name_ccc), ctrl_name_ccc),

    [CTRL_IDX_FIND_PROP] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_declaration_uuid, ESP_GATT_PERM_READ,
                                        sizeof(uint8_t), sizeof(ctrl_find_props), &ctrl_find_props),

    [CTRL_IDX_FIND_VAL] = ATTR_DESC(ESP_GATT_AUTO_RSP, ctrl_find_uuid, ESP_GATT_PERM_WRITE,
                                       SVC_GATTS_PDU_MAX, 0, NULL),

    [CTRL_IDX_FIND_CCC] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                       sizeof(ctrl_find_ccc), sizeof(ctrl_find_ccc), ctrl_find_ccc),

    [CTRL_IDX_MISC_PROP] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_declaration_uuid, ESP_GATT_PERM_READ,
                                        sizeof(uint8_t), sizeof(ctrl_misc_props), &ctrl_misc_props),

    [CTRL_IDX_MISC_VAL] = ATTR_DESC(ESP_GATT_AUTO_RSP, ctrl_misc_uuid, ESP_GATT_PERM_WRITE,
                                       SVC_GATTS_PDU_MAX, 0, NULL),

    [CTRL_IDX_MISC_CCC] = ATTR_DESC(ESP_GATT_AUTO_RSP, character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                       sizeof(ctrl_misc_ccc), sizeof(ctrl_misc_ccc), ctrl_misc_ccc),
};

//只针对GATT服务的回调事件函数
static void ctrl_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
        case ESP_GATTS_REG_EVT:  //GATTS：GATT服务器注册完毕
        {
            if (param->reg.app_id == APP_CTRL_ID)
            {
                ctrl_gatts_if = gatts_if;
                esp_ble_gatts_create_attr_tab(ctrl_attr_db, gatts_if, CTRL_IDX_NUM, 0);
            }
            break;
        }
        case ESP_GATTS_CREAT_ATTR_TAB_EVT: //特征表创建完毕
        {
            if (gatts_if == ctrl_gatts_if)
            {
                if (param->add_attr_tab.status != ESP_GATT_OK)
                {
                    ESP_LOGE(LOG_TAG, "create service failed, status %d", param->add_attr_tab.status);
                }
                else if (param->add_attr_tab.num_handle != CTRL_IDX_NUM)
                {
                    ESP_LOGE(LOG_TAG, "create attribute failed, number %d != %d", param->add_attr_tab.num_handle, CTRL_IDX_NUM);
                }
                else
                {
                    memcpy(ctrl_srv_tbl, param->add_attr_tab.handles, sizeof(ctrl_srv_tbl));
                    esp_ble_gatts_start_service(ctrl_srv_tbl[CTRL_IDX_SRV]); //表填加好之后 启动服务
                }
            }
            break;
        }
        case ESP_GATTS_START_EVT:   //GATT服务器启动完成
        {
            if (ready_sem_handle)
            {
                xSemaphoreGive(ready_sem_handle); //释放二进制的信号量  保证初始化一次
            }
            break;
        }
        case ESP_GATTS_READ_EVT: //客户端（手机）尝试读取 GATT 服务器上的特性——也就是手机读取ESP32上的数据的时候会进入这里
        {
            break;
        }
        case ESP_GATTS_WRITE_EVT://客户端向服务器写入数据——手机向ESP32写入数据
        {
            if (param->write.handle == ctrl_srv_tbl[CTRL_IDX_NAME_CCC] && param->write.len == 2)
            {
                ctrl_name_can_notify = !!param->write.value[0];
                memcpy(ctrl_name_ccc, param->write.value, 2);
                ESP_LOGI(LOG_TAG, "name notify %s", ctrl_name_can_notify ? "enabled" : "disabled");
            }
            else if (param->write.handle == ctrl_srv_tbl[CTRL_IDX_FIND_CCC] && param->write.len == 2)
            {
                ctrl_find_can_notify = !!param->write.value[0];
                memcpy(ctrl_find_ccc, param->write.value, 2);
                ESP_LOGI(LOG_TAG, "find notify %s", ctrl_find_can_notify ? "enabled" : "disabled");
            }
            else if (param->write.handle == ctrl_srv_tbl[CTRL_IDX_MISC_CCC] && param->write.len == 2)
            {
                ctrl_misc_can_notify = !!param->write.value[0];
                memcpy(ctrl_misc_ccc, param->write.value, 2);
                ESP_LOGI(LOG_TAG, "misc notify %s", ctrl_misc_can_notify ? "enabled" : "disabled");
            }
            else if (param->write.handle == ctrl_srv_tbl[CTRL_IDX_NAME_VAL])
            {
                TRY_CALL(ctrl_name_write_callback, param->write.value, param->write.len);
            }
            else if (param->write.handle == ctrl_srv_tbl[CTRL_IDX_FIND_VAL])
            {
                TRY_CALL(ctrl_find_write_callback, param->write.value, param->write.len);
            }
            else if (param->write.handle == ctrl_srv_tbl[CTRL_IDX_MISC_VAL])
            {
                TRY_CALL(ctrl_misc_write_callback, param->write.value, param->write.len);
            }
            break;
        }
        case ESP_GATTS_DISCONNECT_EVT: //服务断连之后 关闭特征清零
        {
            ctrl_name_can_notify = false;
            memset(ctrl_name_ccc, 0, 2);
            ctrl_find_can_notify = false;
            memset(ctrl_find_ccc, 0, 2);
            ctrl_misc_can_notify = false;
            memset(ctrl_misc_ccc, 0, 2);
            break;
        }
        default:
            break;
    }
}

void svc_ctrl_init(void)
{
    ready_sem_handle = xSemaphoreCreateBinary();  //每个服务都会有一个二进制信号量 防止二次初始化
    configASSERT(ready_sem_handle);

    //向协议栈中注册ctrl的ID 同时注册回调事件函数
    ble_register_gatts_callback(APP_CTRL_ID, ctrl_gatts_event_handler);
    esp_ble_gatts_app_register(APP_CTRL_ID);

    //等待回调事件 释放信号量——额就是等待初始化完成
    if (xSemaphoreTake(ready_sem_handle, pdMS_TO_TICKS(1000)) != pdPASS) //只进行一次初始化
    {
        ESP_LOGE(LOG_TAG, "create ctrl service failed");
    }
    vSemaphoreDelete(ready_sem_handle);
}

uint16_t svc_ctrl_get_gatts_if(void)
{
    return ctrl_gatts_if;
}

void svc_ctrl_name_notify(uint8_t *data, uint16_t length)
{
    CHECK_RET(ctrl_name_can_notify);

    ble_send_notify(ctrl_gatts_if, ctrl_srv_tbl[CTRL_IDX_NAME_VAL], data, length);
}

void svc_ctrl_find_notify(uint8_t *data, uint16_t length)
{
    CHECK_RET(ctrl_find_can_notify);

    ble_send_notify(ctrl_gatts_if, ctrl_srv_tbl[CTRL_IDX_FIND_VAL], data, length);
}

void svc_ctrl_misc_notify(uint8_t *data, uint16_t length)
{
    CHECK_RET(ctrl_misc_can_notify);

    ble_send_notify(ctrl_gatts_if, ctrl_srv_tbl[CTRL_IDX_MISC_VAL], data, length);
}

void svc_ctrl_name_write_callback_register(svc_ctrl_name_callback_t callback)
{
    ctrl_name_write_callback = callback;
}

void svc_ctrl_find_write_callback_register(svc_ctrl_find_callback_t callback)
{
    ctrl_find_write_callback = callback;
}

void svc_ctrl_misc_write_callback_register(svc_ctrl_misc_callback_t callback)
{
    ctrl_misc_write_callback = callback;
}
