#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_bt.h"
#include "esp_bt_main.h"

//蓝牙控制器初始化
void bt_dm_init(void)
{
    esp_err_t ret;
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT); //内存释放  因为没有用到经典蓝牙 因此可以释放内存块
    ESP_ERROR_CHECK(ret);

    ret = esp_bt_controller_init(&bt_cfg);                      //初始化控制器
    ESP_ERROR_CHECK(ret);

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);            ///使能控制器 BLE的控制器
    ESP_ERROR_CHECK(ret);
}
