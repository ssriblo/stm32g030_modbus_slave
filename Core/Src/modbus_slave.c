#include "modbus_slave.h"
#include "nanomodbus.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

// Circular buffer for UART RX
#define RX_BUFFER_SIZE 256
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0;
static volatile uint16_t rx_tail = 0;

static nmbs_t nmbs;
static nmbs_platform_conf platform_conf;
static nmbs_callbacks callbacks;

static uint32_t uid_32bit = 0;
static uint16_t fw_version = 0x0100; // 1.0.0

static void RS485_DE_Enable(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

static void RS485_DE_Disable(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

// nanoMODBUS Platform Callbacks
static int32_t platform_read(uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {
    uint16_t bytes_read = 0;
    uint32_t start_time = HAL_GetTick();

    while (bytes_read < count) {
        if (rx_head != rx_tail) {
            buf[bytes_read++] = rx_buffer[rx_tail];
            rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
            start_time = HAL_GetTick(); // Reset timeout on successful byte read
        } else {
            if (byte_timeout_ms >= 0 && (HAL_GetTick() - start_time) > (uint32_t)byte_timeout_ms) {
                break; // Timeout
            }
        }
    }
    return bytes_read;
}

static int32_t platform_write(const uint8_t* buf, uint16_t count, int32_t byte_timeout_ms, void* arg) {
    RS485_DE_Enable();
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart2, (uint8_t*)buf, count, byte_timeout_ms > 0 ? byte_timeout_ms * count : HAL_MAX_DELAY);
    RS485_DE_Disable();
    return (status == HAL_OK) ? count : -1;
}

// nanoMODBUS Register Callbacks
static nmbs_error read_holding_registers(uint16_t address, uint16_t quantity, uint16_t* registers_out, void* arg) {
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t addr = address + i;
        
        // Old mapping (from stage 1) + New mapping for Holding Regs
        if (addr == 0x0000) {
            registers_out[i] = fw_version; // Temporarily kept here for compatibility
        } else if (addr == 0x0001) {
            registers_out[i] = (uid_32bit >> 16) & 0xFFFF; // Serial HI
        } else if (addr == 0x0002) {
            registers_out[i] = uid_32bit & 0xFFFF; // Serial LO
        } else {
            return NMBS_ERROR_ILLEGAL_DATA_ADDRESS;
        }
    }
    return NMBS_ERROR_NONE;
}

static nmbs_error read_input_registers(uint16_t address, uint16_t quantity, uint16_t* registers_out, void* arg) {
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t addr = address + i;
        
        // Stub implementations for the future modbus_reg.md map
        switch(addr) {
            case 0x0000: // Supply Voltage
            case 0x0001: // Current Consumption
            case 0x0002: // Instantaneous Power
            case 0x0003: // Accumulated Energy MSW
            case 0x0004: // Accumulated Energy LSW
            case 0x0005: // Device Uptime MSW
            case 0x0006: // Device Uptime LSW
                registers_out[i] = 0; // Stubs for next stage
                break;
            case 0x0007: // Firmware Version
                registers_out[i] = fw_version;
                break;
            case 0x0008: // Last Reset Cause
                registers_out[i] = 1; // 1 = POR
                break;
            default:
                return NMBS_ERROR_ILLEGAL_DATA_ADDRESS;
        }
    }
    return NMBS_ERROR_NONE;
}

void Modbus_Init(void) {
    // Squeeze 96-bit UID into 32-bit (XORing the 3 words)
    uint32_t uid0 = HAL_GetUIDw0();
    uint32_t uid1 = HAL_GetUIDw1();
    uint32_t uid2 = HAL_GetUIDw2();
    uid_32bit = uid0 ^ uid1 ^ uid2;

    RS485_DE_Disable();

    // Disable hardware RTO (Receiver Timeout) as nanoMODBUS handles timing
    __HAL_UART_DISABLE_IT(&huart2, UART_IT_RTO);
    
    // Enable RXNE interrupt
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);

    // Initialize nanoMODBUS platform conf
    nmbs_platform_conf_create(&platform_conf);
    platform_conf.transport = NMBS_TRANSPORT_RTU;
    platform_conf.read = platform_read;
    platform_conf.write = platform_write;
    platform_conf.arg = NULL;

    // Initialize nanoMODBUS callbacks
    nmbs_callbacks_create(&callbacks);
    callbacks.read_holding_registers = read_holding_registers;
    callbacks.read_input_registers = read_input_registers;

    // Create nanoMODBUS instance
    nmbs_server_create(&nmbs, MODBUS_DEFAULT_SLAVE_ID, &platform_conf, &callbacks);
    nmbs_set_read_timeout(&nmbs, 100); // 100ms timeout
    nmbs_set_byte_timeout(&nmbs, 20);  // 20ms byte timeout (modbus inter-character timeout)
}

void Modbus_Process(void) {
    nmbs_server_poll(&nmbs);
}

void Modbus_UART_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET) {
        uint8_t data = (uint8_t)(huart2.Instance->RDR & 0xFF);
        uint16_t next_head = (rx_head + 1) % RX_BUFFER_SIZE;
        if (next_head != rx_tail) {
            rx_buffer[rx_head] = data;
            rx_head = next_head;
        }
    }
    
    // Clear overrun error if it happens
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE) != RESET) {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_CLEAR_OREF);
    }
}
