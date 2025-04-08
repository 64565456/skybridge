#include "bt_dm.h"
#include "bt_stack.h"
#include "ble_main.h"

void bt_main_init(void)
{
    bt_dm_init();       //蓝牙设备管理器的初始化
    bt_stack_init();    //初始化蓝牙的协议栈  相当于协议框架
    ble_main_init();    //应用程序初始化      在框架中添加应用程序
}
