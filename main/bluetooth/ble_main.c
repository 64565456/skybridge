#include <stdint.h>
#include <string.h>
#include "main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "svc_ctrl.h"
#include "svc_trace.h"
#include "svc_ota.h"
#include "ble_main.h"

#define LOG_TAG "BLE"

#define DEFAULT_ADV_INTERVAL_MIN        160     // 100ms
#define DEFAULT_ADV_INTERVAL_MAX        1600    // 1000ms
#define DEFAULT_CONN_INTERVAL_MIN       24      // 30ms
#define DEFAULT_CONN_INTERVAL_MAX       40      // 50ms
#define DEFAULT_CONN_SLAVE_LATENCY      0       // 0
#define DEFAULT_CONN_TIMEOUT            500     // 5000ms
#define DEFAULT_CONN_INTERVAL_FAST      12
#define DEFAULT_DEVICE_NAME             "SkyBridge"

#define ARRAY_SIZE(x)   (sizeof(x) / sizeof(x)[0])

typedef enum
{
    BLE_EVT_CONNECT = (1 << 1),
    BLE_EVT_DISCONNECT = (1 << 2),

    BLE_EVT_ADV_DATA_DONE = (1 << 8),
    BLE_EVT_SCAN_RSP_DONE = (1 << 9),
    BLE_EVT_ADV_DONE = (1 << 10),
    BLE_EVT_PPCP_DONE = (1 << 11),
} cmd_evt_t;

typedef struct
{
    bool _used;
    uint16_t app_id;
    uint16_t gatt_if;
    ble_gatts_event_handler_t callback;
} ble_srv_hdl_t;

typedef struct
{
    esp_gatt_if_t gatt_if;
    uint16_t attr_hdl;
    uint16_t length;
    uint8_t data[];
} ble_notify_t;

static ble_srv_hdl_t ble_srv_hdls[CONFIG_BT_GATT_MAX_SR_PROFILES];
static bool ble_is_connected = false;
static bool ble_conn_id = 0xffff;
static uint16_t ble_mtu_size = 23;
static uint16_t ble_gatt_if;
static esp_bd_addr_t ble_remote_bda;
static EventGroupHandle_t ble_evt_hdl;
static QueueHandle_t notify_que_handle;
static bool notify_busy;
static ble_on_connected_handler_t on_connected_cb;

static void gap_ble_adv_data_raw_set_complete_evt_handler(struct ble_adv_data_raw_cmpl_evt_param *param)
{
    if (param->status != ESP_BT_STATUS_SUCCESS)
    {
        ESP_LOGE(LOG_TAG, "adv data raw set failed, status %u", param->status);
        return;
    }

    // ESP_LOGI(LOG_TAG, "adv data raw set done");

    xEventGroupSetBits(ble_evt_hdl, BLE_EVT_ADV_DATA_DONE);
}

static void gap_ble_scan_rsp_data_raw_set_complete_evt_handler(struct ble_scan_rsp_data_raw_cmpl_evt_param *param)
{
    if (param->status != ESP_BT_STATUS_SUCCESS)
    {
        ESP_LOGE(LOG_TAG, "scan rsp raw set failed, status %u", param->status);
        return;
    }

    // ESP_LOGI(LOG_TAG, "scan rsp raw set done");

    xEventGroupSetBits(ble_evt_hdl, BLE_EVT_SCAN_RSP_DONE);
}

static void gap_ble_adv_start_complete_evt_handler(struct ble_adv_start_cmpl_evt_param *param)
{
    if (param->status != ESP_BT_STATUS_SUCCESS)
    {
        ESP_LOGE(LOG_TAG, "adv start failed, status %u", param->status);
        return;
    }

    ESP_LOGI(LOG_TAG, "adv start done");

    xEventGroupSetBits(ble_evt_hdl, BLE_EVT_ADV_DONE);
}

static void gap_ble_update_conn_params_evt_handler(struct ble_update_conn_params_evt_param *param)
{
    if (param->status != ESP_BT_STATUS_SUCCESS)
    {
        ESP_LOGE(LOG_TAG, "update conn params failed, status %u", param->status);
        return;
    }

    ESP_LOGI(LOG_TAG, "new conn param conn_int %u latency %u timeout %u",
                      param->conn_int, param->latency, param->timeout);
}

static void gap_ble_phy_update_complete_evt_handler(struct ble_phy_update_cmpl_param *param)
{
    #define PHY_STR(x)  x == 1 ? "1M" : \
                        x == 2 ? "2M" : \
                        x == 3 ? "CODED" : "NA"
    ESP_LOGI(LOG_TAG, "new tx_phy %s rx_phy %s ", PHY_STR(param->tx_phy), PHY_STR(param->rx_phy));
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    ESP_LOGD(LOG_TAG, "gap evt %d", event);
    
    //处理evnt事件
    switch (event)
    {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT: //等待广播数据（ADV_DATA）设置完毕 在代码506行完成之后执行
        {
            //发送notify 在代码507行函数ble_wait_set_adv_data()等待notify
            gap_ble_adv_data_raw_set_complete_evt_handler(&param->adv_data_raw_cmpl); 
            break;
        }
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT: //等待扫描回播或者响应（SCAN_RSP）设置完毕
        {
            gap_ble_scan_rsp_data_raw_set_complete_evt_handler(&param->scan_rsp_data_raw_cmpl);
            break;
        }
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:        //蓝牙广播启动完毕
        {
            gap_ble_adv_start_complete_evt_handler(&param->adv_start_cmpl);
            break;
        }
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:        //蓝牙连接参数启动完毕
        {
            gap_ble_update_conn_params_evt_handler(&param->update_conn_params);
            break;
        }
        case ESP_GAP_BLE_PHY_UPDATE_COMPLETE_EVT:    //蓝牙PHY设置完毕 PHY指更改数据传输时使用的物理通道，比如从 1Mbit/s 切换到 2Mbit/s 影响传输
        {
            gap_ble_phy_update_complete_evt_handler(&param->phy_update);
        }
        default:
            break;
    }
}

//设置一个全局变量的值 ： MTU
static void gatts_mtu_evt_handler(struct gatts_mtu_evt_param *param)
{
    ESP_LOGI(LOG_TAG, "new mtu size %d", param->mtu);

    ble_mtu_size = param->mtu;
}

static void gatts_connect_evt_handler(struct gatts_connect_evt_param *param)
{
    ESP_LOGI(LOG_TAG, "connected to " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(param->remote_bda));

    ble_is_connected = true;
    ble_conn_id = param->conn_id;
    memcpy(&ble_remote_bda, &param->remote_bda, sizeof(esp_bd_addr_t));

    if (on_connected_cb) {
        on_connected_cb(true);
    }

    // ble_set_ppcp(DEFAULT_CONN_INTERVAL_MIN, DEFAULT_CONN_INTERVAL_MAX, DEFAULT_CONN_SLAVE_LATENCY, DEFAULT_CONN_TIMEOUT);
}

static void gatts_disconnect_evt_handler(struct gatts_disconnect_evt_param *param)
{
    ESP_LOGI(LOG_TAG, "disconnected from " ESP_BD_ADDR_STR " reason: 0x%x", ESP_BD_ADDR_HEX(param->remote_bda), param->reason);

    if (on_connected_cb) {
        on_connected_cb(false);
    }

    ble_is_connected = false;
    ble_conn_id = 0xffff;
    ble_start_advertising();
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGD(LOG_TAG, "gatts evt %d if %d", event, gatts_if);

    if (gatts_if == ble_gatt_if || gatts_if == 0xff)
    {
        switch (event)
        {
            case ESP_GATTS_MTU_EVT: //处理MTU事件 MTU：大小决定了单次可以传输的数据量
            {
                gatts_mtu_evt_handler(&param->mtu); //设置MTU的大小
                break;
            }
            case ESP_GATTS_CONNECT_EVT: //手机与ESP32连接成功之后 触发下面的回调
            {
                gatts_connect_evt_handler(&param->connect);
                break;
            }
            case ESP_GATTS_DISCONNECT_EVT: //手机与ESP32的连接断开之后之后 触发下面的回调
            {
                gatts_disconnect_evt_handler(&param->disconnect);
                break;
            }
            case ESP_GATTS_CONGEST_EVT://当协议栈中发生冲突或者阻塞 发不出去数据的时候  触发下面的操作 
            {
                //将notify_busy置于某个值 所以蓝牙主函数中的通知任务函数中的notify_busy来自这里
                notify_busy = param->congest.congested;
                break;
            }
            default:
                break;
        }
    }

    if (event == ESP_GATTS_REG_EVT)
    {
        for (uint16_t i = 0; i < ARRAY_SIZE(ble_srv_hdls); i++)
        {
            if (ble_srv_hdls[i]._used && param->reg.app_id == ble_srv_hdls[i].app_id)
            {
                ble_srv_hdls[i].gatt_if = gatts_if;
                ble_srv_hdls[i].callback(event, gatts_if, param);
            }
        }
    }
    else
    {
        for (uint16_t i = 0; i < ARRAY_SIZE(ble_srv_hdls); i++)
        {
            if (ble_srv_hdls[i]._used && ble_srv_hdls[i].gatt_if == gatts_if)
            {
                ble_srv_hdls[i].callback(event, gatts_if, param);
            }
        }
    }
}

static void ble_gatts_notify(ble_notify_t *notify)
{
    if (ble_is_connected && !notify_busy)
    {
        if (esp_ble_gatts_send_indicate(notify->gatt_if, ble_conn_id, notify->attr_hdl,
                                        notify->length, notify->data, false) != ESP_OK)
        {
            ESP_LOGE(LOG_TAG, "notify error");
        }
    }
}

static void notify_task_handler(void *args)
{
    ble_notify_t *notify;
    for (;;)
    {
        xQueueReceive(notify_que_handle, &notify, portMAX_DELAY); //只要队列数据有数据接收取出来 

        while (notify_busy) //只要不在busy状态：0 就退出while循环
        {
            ESP_LOGW(LOG_TAG, "notify busy");
            vTaskDelay(pdMS_TO_TICKS(50)); //延时原因是因为 协议栈也有notify的队列 只要有空闲就不会进入while中 
                                         //50ms的时间并不会因为时间过长导致有空闲了但是仍在延时的情况，因为50ms内发送不完 不会出现那种状况
        }

        ble_gatts_notify(notify); //发送通知信息

        free(notify);
    }
}

void ble_send_notify(esp_gatt_if_t gatt_if, uint16_t attr_hdl, uint8_t *data, uint16_t length)
{
    uint16_t mtu = ble_get_mtu() - 3;
    while (length > 0)
    {
        uint16_t s = length > mtu ? mtu : length;
        ble_notify_t *p = malloc(sizeof(ble_notify_t) + length);
        CHECK_RET(p);

        p->gatt_if = gatt_if;
        p->attr_hdl = attr_hdl;
        p->length = s;
        memcpy(p->data, data, s);

        if (xQueueSend(notify_que_handle, &p, 0) != pdPASS)
        {
            ESP_LOGE(LOG_TAG, "notify queue full");
            break;
        }

        data += s; length -= s;
    }
}

void ble_set_adv_data(char *name, uint8_t *data, uint16_t data_length)
{
    /*
    * adv data & scan rsp
    * max 31 bytes
    *
    * refDoc: Assigned_Numbers 2.3
    * refDoc: Core_v5.4 Vol 3, Part C, 11
    * refDoc: Core_v5.4 Vol 6, Part B, 2.3.1
    *
    * note:
    * Advertising Interval (0x1A)
    * Peripheral Connection Interval Range (0x12)
    */

    #define ADV_PDU_LEN     31

    uint8_t adv_data[ADV_PDU_LEN] = { 0 }, scan_rsp[ADV_PDU_LEN] = { 0 };
    uint8_t adv_data_len = 0;

    #define ADV_DATA_ADD(tag, data, len) \
    if (adv_data_len + (len) + 2 <= ADV_PDU_LEN) \
    { \
        adv_data[adv_data_len++] = (len) + 1; \
        adv_data[adv_data_len++] = (tag); \
        memcpy(&adv_data[adv_data_len], (data), (len)); \
        adv_data_len += (len); \
    } \
    else \
    { \
        ESP_LOGE(LOG_TAG, "adv data overflow tag=%d len=%d used=%d", tag, len, adv_data_len); \
        ESP_LOG_BUFFER_HEX(LOG_TAG, data, len); \
    }

    uint8_t flag = 0x06;
    esp_bd_addr_t bd_addr;
    uint8_t addr_type;

    esp_ble_gap_set_device_name(name);
    esp_ble_gap_get_local_used_addr(bd_addr, &addr_type);

    ADV_DATA_ADD(ESP_BLE_AD_TYPE_FLAG, &flag, 1);
    ADV_DATA_ADD(ESP_BLE_AD_TYPE_NAME_CMPL, name, strlen(name));
    ADV_DATA_ADD(ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE, bd_addr, sizeof(esp_bd_addr_t));
    // ESP_LOG_BUFFER_HEX(LOG_TAG, adv_data, adv_data_len);
    esp_ble_gap_config_adv_data_raw(adv_data, adv_data_len);

    scan_rsp[0] = ADV_PDU_LEN - 1;
    scan_rsp[1] = ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE;
    scan_rsp[2] = 0xff; scan_rsp[3] = 0xff;
    memcpy(&scan_rsp[4], data, data_length < ADV_PDU_LEN - 4 ? data_length : ADV_PDU_LEN - 4);
    // ESP_LOG_BUFFER_HEX(LOG_TAG, scan_rsp, ADV_PDU_LEN);
    esp_ble_gap_config_scan_rsp_data_raw(scan_rsp, ADV_PDU_LEN);
}

 //等待GAP回调函数中设置好的notify  等待成功之后表示设置完成
bool ble_wait_set_adv_data(void)
{
    //等待标志位
    if (!xEventGroupWaitBits(ble_evt_hdl, BLE_EVT_ADV_DATA_DONE | BLE_EVT_SCAN_RSP_DONE, pdTRUE, pdTRUE, pdMS_TO_TICKS(1000)))
    {
        ESP_LOGE(LOG_TAG, "set adv data & scan rsp timeout");
        return false;
    }

    return true;
}

void ble_start_advertising(void)
{
    esp_ble_adv_params_t ble_adv_params =
    {
        .adv_int_min = DEFAULT_ADV_INTERVAL_MIN,
        .adv_int_max = DEFAULT_ADV_INTERVAL_MAX,
        .adv_type = ADV_TYPE_IND,
        .own_addr_type = BLE_ADDR_TYPE_RANDOM,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    esp_ble_gap_start_advertising(&ble_adv_params);
}

bool ble_wait_start_advertising(void)
{
    if (!xEventGroupWaitBits(ble_evt_hdl, BLE_EVT_ADV_DONE, pdTRUE, pdTRUE, pdMS_TO_TICKS(1000)))
    {
        ESP_LOGE(LOG_TAG, "start adv timeout");
        return false;
    }

    return true;
}

void ble_set_ppcp(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout)
{
    esp_ble_conn_update_params_t conn_params =
    {
        .min_int = interval_min,
        .max_int = interval_max,
        .latency = latency,
        .timeout = timeout,
    };
    memcpy(&conn_params.bda, ble_remote_bda, sizeof(esp_bd_addr_t));

    esp_ble_gap_update_conn_params(&conn_params);
}

bool ble_wait_set_ppcp(void)
{
    if (!xEventGroupWaitBits(ble_evt_hdl, BLE_EVT_PPCP_DONE, pdTRUE, pdTRUE, pdMS_TO_TICKS(1000)))
    {
        ESP_LOGE(LOG_TAG, "set ppcp timeout");
        return false;
    }

    return true;
}

void ble_main_init(void)
{
    esp_err_t ret;
    nvs_handle_t nhdl; //nvs的句柄  用于后续的操作

    /****************************配置蓝牙的地址****************************/

    //非易失性存储器  读取initila分区的数据  initial表示的是数据不是一个分区 因此不在分区表中
    if (nvs_open("initial", NVS_READONLY, &nhdl) == ESP_OK)
    {
        esp_bd_addr_t ble_addr;  //结构体里有一个数组成员
        size_t len = sizeof(ble_addr);
        if (nvs_get_blob(nhdl, "ble_addr", ble_addr, &len) == ESP_OK) //把地址存储到定义好的结构体中（数组）
        {
            ESP_LOGI(LOG_TAG, "set ble addr " ESP_BD_ADDR_STR, ESP_BD_ADDR_HEX(ble_addr));
            esp_ble_gap_set_rand_addr(ble_addr); //设置蓝牙的随机地址
            //静态地址：ESP32中蓝有唯一的静态地址  不可以设置
            //随即地址：有多个；可以自己设置
        }
        else
        {
            ESP_LOGW(LOG_TAG, "get ble addr failed");
        }
        nvs_close(nhdl);
    }
    else
    {
        ESP_LOGW(LOG_TAG, "open nvs initial failed");
    }

    ble_evt_hdl = xEventGroupCreate(); //创建事件
    configASSERT(ble_evt_hdl != NULL);
    
    /****************************配置蓝牙notify任务****************************/

    //蓝牙有三种特征类型：读   写   通知；  这里创建通知队列用于ESP32给手机端发送信息，便于管理ESP32给手机端发消息的速率、数量、内容等
    notify_que_handle = xQueueCreate(256, sizeof(ble_notify_t *)); //创建notify队列 
    configASSERT(notify_que_handle);

    //创建蓝牙通知任务
    xTaskCreate(notify_task_handler, "ble notify", 4096, NULL, 8, NULL);


    /****************************注册蓝牙GATT层和GAP层的回调函数****************************/


    //注册GAP的事件回调函数
    ret = esp_ble_gap_register_callback(gap_event_handler); //与广播有关的
    ESP_ERROR_CHECK(ret);
    
    //注册GATT的事件回调函数
    ret = esp_ble_gatts_register_callback(gatts_event_handler); //与ESP32连接手机有关的
    ESP_ERROR_CHECK(ret);


    /****************************初始化蓝牙的三个服务***************************/

    //初始化三个服务  用于  应用程序  与  协议栈  之间的过度
    //主要用于写入与STM32通信的内容  （用串口通信）
    svc_ctrl_init();
    svc_trace_init();
    svc_ota_init();

    ble_gatt_if = svc_ctrl_get_gatts_if(); //获取ctrl服务的gatt的特征ID

    //设置发射的功率 每隔一段时间发送一次连接的数据包 不要太高防止崩掉
    // esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N0);   


    /****************************配置并启动蓝牙广播****************************/

    //设置广播
    ble_start_advertising();        //启动蓝牙的广播
    ble_wait_start_advertising();   //等蓝牙广播启动完成

    //填充广播数据
    ble_set_adv_data(DEFAULT_DEVICE_NAME, NULL, 0);     //设置广播的数据：设备名 自定义数据
    ble_wait_set_adv_data();                            //等待数据设置完毕  使手机能够搜索到有效的广播数据
}

bool ble_is_connect(void)
{
    return ble_is_connected;
}

uint16_t ble_get_conn_id(void)
{
    return ble_conn_id;
}

uint16_t ble_get_mtu(void)
{
    return ble_mtu_size;
}

bool ble_register_gatts_callback(uint16_t app_id, ble_gatts_event_handler_t callback)
{
    for (uint16_t i = 0; i < ARRAY_SIZE(ble_srv_hdls); i++)
    {
        if (ble_srv_hdls[i]._used == false)
        {
            ble_srv_hdls[i]._used = true;
            ble_srv_hdls[i].app_id = app_id;
            ble_srv_hdls[i].callback = callback;
            return true;
        }
    }

    return false;
}

void ble_on_connected_register(ble_on_connected_handler_t cb)
{
    on_connected_cb = cb;
}
