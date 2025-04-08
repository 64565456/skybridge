#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/uart_hal.h"
#include "main.h"
#include "esp_pm.h"
#include "esp_log.h"
#include "com_proto.h"
#include "com_uart.h"
#include "channel_defs.h"
#include "channel_forward.h"

#define LOG_TAG "UART"


#define COM_UART_INST           UART_NUM_1  //串口的编号
#define COM_UART_BAUD           115200
#define COM_UART_FLOW           UART_HW_FLOWCTRL_DISABLE
#define COM_UART_PAD_TX         GPIO_NUM_4
#define COM_UART_PAD_RX         GPIO_NUM_5
#define COM_UART_PAD_CTS        GPIO_NUM_19
#define COM_UART_PAD_RTS        GPIO_NUM_18
#define COM_UART_TX_BUF_SIZE    1024
#define COM_UART_RX_BUF_SIZE    8192
#define RECV_BUFF_SIZE          4096
#define SEND_QUE_LEN            32

static TaskHandle_t com_recv_task = NULL;
static SemaphoreHandle_t com_recv_mux = NULL;
static bool com_send_lock = false;

static void uart_recv_handler(void *args)
{
    int frame_length = 0;
    uint8_t *frame_buff = malloc(RECV_BUFF_SIZE);
    assert(frame_buff);

    xSemaphoreTake(com_recv_mux, portMAX_DELAY);     //获取互斥锁

    for (;;)
    {
        int read_bytes;
        uint8_t channel;
        uint16_t data_length;
        uint8_t *pdata;

        // 便于com_uart_pause_internal_recv调用，临时释放锁
        xSemaphoreGive(com_recv_mux);                   //放锁
        xSemaphoreTake(com_recv_mux, portMAX_DELAY);    //再次持锁

        //在没有接收数据或没有超时的时候 会进入阻塞的状态 ； 读取到数据之后 会把数据放到frame_buff中 并返回读到的数据
        //判断协议的magic是否正确
        read_bytes = uart_read_bytes(COM_UART_INST, &frame_buff[COM_PROTO_MAGIC_OFFSET], COM_PROTO_MAGIC_SIZE, pdMS_TO_TICKS(1000));
                                    //UART的实例      存储读取数据缓冲区的指针               读取数据的字节数         超时时间
        if (read_bytes <= 0 || frame_buff[COM_PROTO_MAGIC_OFFSET] != COM_PROTO_MAGIC) //没有接收到数据就continue：1、数据为零2、第一个数据仍未0xaa
            continue;
        //continue作用是跳过当前的迭代

        //解析OTA bootloader的通信流程 
        //接收channel数据
        read_bytes = uart_read_bytes(COM_UART_INST, &frame_buff[COM_PROTO_CHANNEL_OFFSET], COM_PROTO_CHANNEL_SIZE, pdMS_TO_TICKS(100));
        if (read_bytes != COM_PROTO_CHANNEL_SIZE)
            continue;
        channel = frame_buff[COM_PROTO_CHANNEL_OFFSET];
        if (channel >= CHANNEL_NUM && channel != CHANNEL_ACK)
        {
            ESP_LOGE(LOG_TAG, "recv channel unknown %u", channel);
            continue;
        }

        //接收length
        read_bytes = uart_read_bytes(COM_UART_INST, &frame_buff[COM_PROTO_LENGTH_OFFSET], COM_PROTO_LENGTH_SIZE, pdMS_TO_TICKS(100));
        if (read_bytes != COM_PROTO_LENGTH_SIZE)
            continue;
        data_length = get_u16(&frame_buff[COM_PROTO_LENGTH_OFFSET]);
        if (data_length > RECV_BUFF_SIZE - COM_PROTO_BASE_SIZE)
        {
            ESP_LOGE(LOG_TAG, "recv data length too large %u", data_length);
            continue;
        }

        //接收data数据
        read_bytes = uart_read_bytes(COM_UART_INST, &frame_buff[COM_PROTO_DATA_OFFSET], data_length, pdMS_TO_TICKS(1000));
        if (read_bytes != data_length)
            continue;
        pdata = &frame_buff[COM_PROTO_DATA_OFFSET];


        //接收CRC数据
        read_bytes = uart_read_bytes(COM_UART_INST, &frame_buff[data_length + COM_PROTO_CRC_OFFSET], COM_PROTO_CRC_SIZE, pdMS_TO_TICKS(100));
        if (read_bytes != COM_PROTO_CRC_SIZE)
            continue;
        frame_length = COM_PROTO_BASE_SIZE + data_length;

        //进行校验
        if (!com_proto_verify(frame_buff, frame_length))
        {
            ESP_LOGE(LOG_TAG, "packet verify failed");
            continue;
        }

        // 根据channel类型 进行数据转发
        channel_forward(channel, pdata, data_length);
    }

    free(frame_buff); //由于是申请的堆内存 需要free掉  应付静态代码的扫描
}

void com_uart_init(void)
{
    esp_err_t ret;

    const uart_config_t uart_config =
    {
        .baud_rate = COM_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = COM_UART_FLOW,
        .source_clk = UART_SCLK_APB,
    };

    //设置串口
    ret = uart_param_config(COM_UART_INST, &uart_config);
    ESP_ERROR_CHECK(ret);

    //设置引脚
    ret = uart_set_pin(COM_UART_INST, COM_UART_PAD_TX, COM_UART_PAD_RX, COM_UART_PAD_RTS, COM_UART_PAD_CTS);
    ESP_ERROR_CHECK(ret);

    //注册串口的驱动
    ret = uart_driver_install(COM_UART_INST, COM_UART_RX_BUF_SIZE, COM_UART_TX_BUF_SIZE, 0, NULL, 0);
    ESP_ERROR_CHECK(ret);

    // uart_intr_config_t intr_config =
    // {
    //     .intr_enable_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
    //     .rx_timeout_thresh = 55,
    //     .rxfifo_full_thresh = 16,
    // };
    // ret = uart_intr_config(COM_UART_INST, &intr_config);
    // ESP_ERROR_CHECK(ret);

    com_recv_mux = xSemaphoreCreateMutex();  //创建互斥量
    configASSERT(com_recv_mux);

    xTaskCreate(uart_recv_handler, "com uart", 4096, NULL, 9, &com_recv_task); //创建一个串口接收的任务
}

bool com_uart_send(uint8_t *data, uint16_t length)
{
    // ESP_LOG_BUFFER_HEX("tx", data, length);

    if (!com_send_lock)
    {
        uart_write_bytes(COM_UART_INST, data, length);
    }

    return true;
}

bool com_uart_send_inlock(uint8_t *data, uint16_t length)
{
    // ESP_LOG_BUFFER_HEX("tx", data, length);

    uart_write_bytes(COM_UART_INST, data, length);

    return true;
}

void com_uart_flush(void)
{
    uart_flush(COM_UART_INST);
}

bool com_uart_recv(uint8_t *data, uint16_t length, uint32_t timeout)
{
    uart_read_bytes(COM_UART_INST, data, length, pdMS_TO_TICKS(timeout));

    // ESP_LOG_BUFFER_HEX("rx", data, length);

    return true;
}

void com_uart_lock_send(void)
{
    com_send_lock = true;
}

void com_uart_unlock_send(void)
{
    com_send_lock = false;
}

void com_uart_pause_internal_recv(bool pause)
{
    if (pause)
    {
        uint32_t priority = uxTaskPriorityGet(NULL);
        vTaskPrioritySet(NULL, uxTaskPriorityGet(com_recv_task) + 1);
        xSemaphoreTake(com_recv_mux, portMAX_DELAY);  //获取临时释放的锁
        vTaskPrioritySet(NULL, priority);

    } else
    {
        xSemaphoreGive(com_recv_mux);
    }
}
