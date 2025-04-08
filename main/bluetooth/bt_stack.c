#include "esp_err.h"
#include "esp_bt_main.h"

void bt_stack_init(void)
{
    esp_err_t ret;

    ret = esp_bluedroid_init();     //初始化蓝牙协议栈 bluedriod （EPS32中还支持nimble的协议栈） 设备不同协议栈也不同
    ESP_ERROR_CHECK(ret);

    ret = esp_bluedroid_enable();   //使能
    ESP_ERROR_CHECK(ret);
}
