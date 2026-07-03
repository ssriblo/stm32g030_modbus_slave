#include "modbus_slave.h"
#include "usart.h"
#include "gpio.h"

extern UART_HandleTypeDef huart2;

static uint8_t rx_buf[MODBUS_BUFFER_SIZE];
static volatile uint16_t rx_index = 0;
static volatile uint8_t frame_ready = 0;

static uint32_t uid_32bit = 0;
static uint16_t fw_version = 0x0100; // 1.0.0

static void RS485_DE_Enable(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

static void RS485_DE_Disable(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

// CRC16 Calculation for Modbus
static uint16_t Modbus_CRC16(const uint8_t *buffer, uint16_t buffer_length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < buffer_length; i++) {
        crc ^= buffer[i];
        for (int j = 8; j != 0; j--) {
            if ((crc & 0x0001) != 0) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Modbus_Init(void) {
    // Squeeze 96-bit UID into 32-bit (XORing the 3 words)
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();
    uid_32bit = uid0 ^ uid1 ^ uid2;

    RS485_DE_Disable();

    // Enable RXNE and RTO (Receiver Timeout) interrupts
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
    
    // Enable Receiver Timeout and set to 40 bit times (approx 3.5 chars for Modbus RTU at 9600)
    HAL_UART_ReceiverTimeout_Config(&huart2, 40);
    HAL_UART_EnableReceiverTimeout(&huart2);
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RTO);
}

static void Modbus_SendResponse(uint8_t *buf, uint16_t len) {
    uint16_t crc = Modbus_CRC16(buf, len);
    buf[len++] = crc & 0xFF;
    buf[len++] = (crc >> 8) & 0xFF;

    RS485_DE_Enable();
    HAL_UART_Transmit(&huart2, buf, len, HAL_MAX_DELAY);
    RS485_DE_Disable();
}

void Modbus_Process(void) {
    if (frame_ready) {
        uint16_t len = rx_index;
        
        // Reset rx_index and frame_ready immediately to allow new receptions
        rx_index = 0;
        frame_ready = 0;

        if (len >= 8) { // Minimum Modbus RTU frame length is 8 bytes
            uint16_t crc = Modbus_CRC16(rx_buf, len - 2);
            uint16_t rx_crc = rx_buf[len - 2] | (rx_buf[len - 1] << 8);

            if (crc == rx_crc && rx_buf[0] == MODBUS_DEFAULT_SLAVE_ID) {
                uint8_t func_code = rx_buf[1];
                
                if (func_code == 0x03 || func_code == 0x04) { // Read Holding/Input Registers
                    uint16_t start_addr = (rx_buf[2] << 8) | rx_buf[3];
                    uint16_t num_regs = (rx_buf[4] << 8) | rx_buf[5];

                    if (num_regs > 0 && num_regs <= 125) {
                        uint8_t response[256];
                        response[0] = rx_buf[0];
                        response[1] = func_code;
                        response[2] = num_regs * 2; // Byte count
                        
                        uint8_t valid = 1;
                        uint16_t idx = 3;

                        for (uint16_t i = 0; i < num_regs; i++) {
                            uint16_t addr = start_addr + i;
                            uint16_t val = 0;

                            if (addr == REG_FW_VERSION) {
                                val = fw_version;
                            } else if (addr == REG_SERIAL_NUM_HI) {
                                val = (uid_32bit >> 16) & 0xFFFF;
                            } else if (addr == REG_SERIAL_NUM_LO) {
                                val = uid_32bit & 0xFFFF;
                            } else {
                                valid = 0;
                                break;
                            }
                            response[idx++] = (val >> 8) & 0xFF;
                            response[idx++] = val & 0xFF;
                        }

                        if (valid) {
                            Modbus_SendResponse(response, idx);
                        } else {
                            // Exception: Illegal Data Address
                            response[1] = func_code | 0x80;
                            response[2] = 0x02;
                            Modbus_SendResponse(response, 3);
                        }
                    } else {
                        // Exception: Illegal Data Value
                        uint8_t response[3];
                        response[0] = rx_buf[0];
                        response[1] = func_code | 0x80;
                        response[2] = 0x03;
                        Modbus_SendResponse(response, 3);
                    }
                } else {
                    // Exception: Illegal Function
                    uint8_t response[3];
                    response[0] = rx_buf[0];
                    response[1] = func_code | 0x80;
                    response[2] = 0x01;
                    Modbus_SendResponse(response, 3);
                }
            }
        }
    }
}

void Modbus_UART_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) {
        uint8_t data = (uint8_t)(huart2.Instance->RDR & 0xFF);
        if (rx_index < MODBUS_BUFFER_SIZE) {
            rx_buf[rx_index++] = data;
        }
    }
    
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RTOF) != RESET) {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_RTOF);
        if (rx_index > 0) {
            frame_ready = 1;
        }
    }

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
    }
}
