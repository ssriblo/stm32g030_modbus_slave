#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "main.h"

/* Configurable Modbus Parameters */
#define MODBUS_DEFAULT_SLAVE_ID 1

void Modbus_Init(void);
void Modbus_Process(void);
void Modbus_UART_IRQHandler(void);

#endif /* MODBUS_SLAVE_H */
