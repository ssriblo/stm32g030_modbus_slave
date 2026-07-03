#ifndef MODBUS_SLAVE_H
#define MODBUS_SLAVE_H

#include "main.h"

/* Configurable Modbus Parameters */
#define MODBUS_DEFAULT_SLAVE_ID 1
#define MODBUS_BUFFER_SIZE      256

/* Modbus Registers */
#define REG_FW_VERSION    0x0000
#define REG_SERIAL_NUM_HI 0x0001
#define REG_SERIAL_NUM_LO 0x0002

void Modbus_Init(void);
void Modbus_Process(void);
void Modbus_UART_IRQHandler(void);

#endif /* MODBUS_SLAVE_H */
