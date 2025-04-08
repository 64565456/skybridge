#ifndef __SVC_OTA_H
#define __SVC_OTA_H


#include <stdint.h>

typedef void (*svc_ota_data_write_callback_t)(uint8_t *data, uint16_t length);
typedef void (*svc_ota_ctrl_write_callback_t)(uint8_t *data, uint16_t length);

void svc_ota_init(void);
uint16_t svc_ota_get_gatts_if(void);
void svc_ota_data_notify(uint8_t *data, uint16_t length);
void svc_ota_ctrl_notify(uint8_t *data, uint16_t length);
void svc_ota_data_write_callback_register(svc_ota_data_write_callback_t callback);
void svc_ota_ctrl_write_callback_register(svc_ota_ctrl_write_callback_t callback);


#endif /* __SVC_OTA_H */
