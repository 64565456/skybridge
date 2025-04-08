#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "runloop.h"

typedef struct
{
    runloop_task_t task;
    void *args;
} runloop_id_t;

static QueueHandle_t runloop_queue_handle;

static void runloop_task(void *args)
{
    runloop_id_t rid;

    for (;;)
    {
        if (xQueueReceive(runloop_queue_handle, &rid, portMAX_DELAY))
        {
            rid.task(rid.args);
        }
    }
}

void runloop_init(void)
{
    runloop_queue_handle = xQueueCreate(10, sizeof(runloop_id_t));
    configASSERT(runloop_queue_handle);
    xTaskCreate(runloop_task, "runloop", 2048, NULL, 5, NULL);
}

void runloop_run(runloop_task_t task, void *args)
{
    runloop_id_t rid = {
        .task = task,
        .args = args,
    };

    xQueueSend(runloop_queue_handle, &rid, portMAX_DELAY);
}

void runloop_run_from_isr(runloop_task_t task, void *args)
{
    runloop_id_t rid = {
        .task = task,
        .args = args,
    };

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(runloop_queue_handle, &rid, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
