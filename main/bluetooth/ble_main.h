#ifndef __BLE_MAIN_H
#define __BLE_MAIN_H


#include <stdbool.h>
#include <stdint.h>
#include "esp_gatts_api.h"

#define SVC_GATTS_PDU_MAX     244

typedef void (*ble_gatts_event_handler_t)(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
typedef void (*ble_on_connected_handler_t)(bool connected);

void ble_main_init(void);
void ble_start_advertising(void);
bool ble_wait_start_advertising(void);
void ble_set_adv_data(char *name, uint8_t *data, uint16_t data_length);
bool ble_wait_set_adv_data(void);
void ble_set_ppcp(uint16_t interval_min, uint16_t interval_max, uint16_t latency, uint16_t timeout);
bool ble_wait_set_ppcp(void);
bool ble_is_connect(void);
uint16_t ble_get_conn_id(void);
uint16_t ble_get_mtu(void);
void ble_send_notify(esp_gatt_if_t gatt_if, uint16_t attr_hdl, uint8_t *data, uint16_t length);
bool ble_register_gatts_callback(uint16_t app_id, ble_gatts_event_handler_t callback);
void ble_on_connected_register(ble_on_connected_handler_t cb);


#endif /* __BLE_MAIN_H */
