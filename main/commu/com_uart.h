#ifndef __COMMU_UART_H
#define __COMMU_UART_H


#include <stdbool.h>
#include <stdint.h>

void com_uart_init(void);
bool com_uart_send(uint8_t *data, uint16_t length);
bool com_uart_send_inlock(uint8_t *data, uint16_t length);
void com_uart_flush(void);
bool com_uart_recv(uint8_t *data, uint16_t length, uint32_t timeout);
void com_uart_lock_send(void);
void com_uart_unlock_send(void);
void com_uart_pause_internal_recv(bool pause);


#endif /* __COMMU_UART_H */
