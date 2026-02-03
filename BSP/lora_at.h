#ifndef __LORA_AT_H
#define __LORA_AT_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
// LoRa模块AT指令配置参数
#define LORA_UART_HANDLE            huart1
#define LORA_RESPONSE_TIMEOUT_MS    3000
#define LORA_CMD_DELAY_MS           100
#define LORA_MAX_RESPONSE_LEN       128

// LoRa模块默认配置参数 (根据参考代码设置)
#define LORA_DEFAULT_BANDRATE       9600
#define LORA_DEFAULT_ADDRESS        0x0001
#define LORA_DEFAULT_NETID          0
#define LORA_DEFAULT_CHANNEL        0
#define LORA_DEFAULT_TRANS_MODE     1  // 点对点传输模式
#define LORA_DEFAULT_URXT           3  // UART帧超时3字节

/* Exported types ------------------------------------------------------------*/
typedef enum {
    LORA_OK = 0,
    LORA_ERROR,
    LORA_TIMEOUT,
    LORA_INVALID_RESPONSE
} Lora_StatusTypeDef;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

// LoRa模块基础操作函数
Lora_StatusTypeDef LORA_Init(void);
Lora_StatusTypeDef LORA_TestConnection(void);
Lora_StatusTypeDef LORA_Reset(void);

// LoRa参数配置函数
Lora_StatusTypeDef LORA_SetUART(uint8_t mode, uint8_t parity);
Lora_StatusTypeDef LORA_SetAddress(uint16_t address);
Lora_StatusTypeDef LORA_SetNetID(uint8_t netid);
Lora_StatusTypeDef LORA_SetChannel(uint8_t channel);
Lora_StatusTypeDef LORA_SetTransMode(uint8_t mode);
Lora_StatusTypeDef LORA_SetURxT(uint8_t urxt);

// 辅助函数
Lora_StatusTypeDef LORA_SendATCommand(const char* cmd, char* response, uint16_t timeout_ms);
void LORA_DelayMs(uint32_t ms);
Lora_StatusTypeDef LORA_WaitForOK(uint16_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_AT_H */