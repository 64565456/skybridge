#ifndef __SVC_TRACE_H
#define __SVC_TRACE_H


#include <stdint.h>

typedef void (*svc_trace_shell_write_callback_t)(uint8_t *data, uint16_t length);

void svc_trace_init(void);
uint16_t svc_trace_get_gatts_if(void);
void svc_trace_log_notify(uint8_t *data, uint16_t length);
void svc_trace_shell_notify(uint8_t *data, uint16_t length);
void svc_trace_shell_write_callback_register(svc_trace_shell_write_callback_t callback);
void svc_trace_local_notify(uint8_t *data, uint16_t length);


#endif /* __SVC_TRACE_H */
