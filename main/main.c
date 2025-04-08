#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "board.h"
#include "runloop.h"
#include "com_uart.h"
#include "bt_main.h"
#include "main.h"

#define LOG_TAG "MAIN"

static int _log_vprintf(const char *fmt, va_list args)
{
    int ret;
    char log_buf[256];

    ret = vprintf(fmt, args);
    vsnprintf(log_buf, sizeof(log_buf), fmt, args);

    extern void svc_trace_local_notify(uint8_t *data, uint16_t length);
    svc_trace_local_notify((uint8_t *)log_buf, strlen(log_buf));

    return ret;
}

static void application_init(void)
{
    extern void prf_ctrl_init(void);
    extern void prf_trace_init(void);
    extern void prf_ota_init(void);

    prf_ctrl_init();
    prf_trace_init();
    prf_ota_init();
}

int app_main(void)
{
    board_init();

    com_uart_init();                    //ESP32与主控STM32F4通信的串口 
                                        //作用：ESP32模拟STM32的上位机 实现固件的升级 STM32的bootloader代码还是要提前烧录到STM32中的

    bt_main_init();                     //ESP32的蓝牙 bt协议栈初始化

    esp_log_set_vprintf(_log_vprintf);  //设置ESP32的日志重定向

    runloop_init();                     //初始化一个任务事件器
    application_init();                 //初始化应用程序

    // vTaskDelay(pdMS_TO_TICKS(5000));
    // extern void test_start_dfu(void);
    // test_start_dfu();

    return 0;
}
