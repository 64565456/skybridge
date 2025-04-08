#ifndef __SVC_CTRL_H
#define __SVC_CTRL_H


#include <stdint.h>

typedef void (*svc_ctrl_name_callback_t)(uint8_t *data, uint16_t length);
typedef void (*svc_ctrl_find_callback_t)(uint8_t *data, uint16_t length);
typedef void (*svc_ctrl_misc_callback_t)(uint8_t *data, uint16_t length);

void svc_ctrl_init(void);
uint16_t svc_ctrl_get_gatts_if(void);
void svc_ctrl_name_notify(uint8_t *data, uint16_t length);
void svc_ctrl_find_notify(uint8_t *data, uint16_t length);
void svc_ctrl_misc_notify(uint8_t *data, uint16_t length);
void svc_ctrl_name_write_callback_register(svc_ctrl_name_callback_t callback);
void svc_ctrl_find_write_callback_register(svc_ctrl_find_callback_t callback);
void svc_ctrl_misc_write_callback_register(svc_ctrl_misc_callback_t callback);


#endif /* __SVC_CTRL_H */
