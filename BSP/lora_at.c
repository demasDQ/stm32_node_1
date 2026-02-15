#include "lora_at.h"
#include "delay.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t lora_rx_buffer[LORA_MAX_RESPONSE_LEN];
static uint16_t lora_rx_index = 0;

/* Private function prototypes -----------------------------------------------*/
static void LORA_ClearRxBuffer(void);

/**
  * @brief  清空接收缓冲区
  * @param  None
  * @retval None
  */
static void LORA_ClearRxBuffer(void)
{
    memset(lora_rx_buffer, 0, sizeof(lora_rx_buffer));
    lora_rx_index = 0;
}

/**
  * @brief  延时函数
  * @param  ms: 延时毫秒数
  * @retval None
  */
void LORA_DelayMs(uint32_t ms)
{
    delay_ms(ms);
}

/**
  * @brief  等待LoRa模块返回OK响应
  * @param  timeout_ms: 超时时间(毫秒)
  * @retval Lora_StatusTypeDef: 操作状态
  */
Lora_StatusTypeDef LORA_WaitForOK(uint16_t timeout_ms)
{
    uint32_t start_time = HAL_GetTick();
    
    while((HAL_GetTick() - start_time) < timeout_ms)
    {
        // 检查是否有数据到达
        if(lora_rx_index > 0)
        {
            // 查找OK响应
            for(uint16_t i = 0; i < lora_rx_index; i++)
            {
                if(lora_rx_buffer[i] == '\r' || lora_rx_buffer[i] == '\n')
                {
                    // 检查是否包含OK
                    if(strstr((char*)lora_rx_buffer, "OK") != NULL)
                    {
                        LORA_ClearRxBuffer();
                        return LORA_OK;
                    }
                    // 检查是否包含ERROR
                    else if(strstr((char*)lora_rx_buffer, "ERROR") != NULL)
                    {
                        LORA_ClearRxBuffer();
                        return LORA_ERROR;
                    }
                }
            }
        }
        delay_ms(10); // 短暂延迟避免过度占用CPU
    }
    
    // 超时处理
    LORA_ClearRxBuffer();
    return LORA_TIMEOUT;
}

/**
  * @brief  发送AT指令并等待响应
  * @param  cmd: AT指令字符串
  * @param  response: 响应字符串指针
  * @param  timeout_ms: 超时时间(毫秒)
  * @retval Lora_StatusTypeDef: 操作状态
  */
Lora_StatusTypeDef LORA_SendATCommand(const char* cmd, char* response, uint16_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;
    uint16_t cmd_len = strlen(cmd);
    
    // 清空接收缓冲区
    LORA_ClearRxBuffer();
    
    // 发送AT指令
    hal_status = HAL_UART_Transmit(&LORA_UART_HANDLE, (uint8_t*)cmd, cmd_len, 1000);
    if(hal_status != HAL_OK)
    {
        return LORA_ERROR;
    }
    
    // 等待响应
    return LORA_WaitForOK(timeout_ms);
}

/**
  * @brief  测试LoRa模块连接
  * @param  None
  * @retval Lora_StatusTypeDef: 操作状态
  */
Lora_StatusTypeDef LORA_TestConnection(void)
{
    char response[LORA_MAX_RESPONSE_LEN];
    Lora_StatusTypeDef status;
    
    status = LORA_SendATCommand("AT\r\n", response, LORA_RESPONSE_TIMEOUT_MS);
    return status;
}

/**
  * @brief  重置LoRa模块
  * @param  None
  * @retval Lora_StatusTypeDef: 操作状态
  */
Lora_StatusTypeDef LORA_Reset(void)
{
    char response[LORA_MAX_RESPONSE_LEN];
    Lora_StatusTypeDef status;
    
    status = LORA_SendATCommand("AT+RESET\r\n", response, LORA_RESPONSE_TIMEOUT_MS);
    if(status == LORA_OK)
    {
        LORA_DelayMs(2000); // 等待模块重启
        return LORA_OK;
    }
    return LORA_ERROR;
}

/**
  * @brief  设置LoRa UART参数
  * @param  mode: UART模式 (3=9600bps)
  * @param  parity: 校验位 (0=无校验)
  * @retval Lora_StatusTypeDef: 操作状态
  */
Lora_StatusTypeDef LORA_SetUART(uint8_t mode, uint8_t parity)
{
    char cmd[32];
    char response[LORA_MAX_RESPONSE_LEN];
    Lora_StatusTypeDef status;
    
    sprintf(cmd, "AT+UART=%d,%d\r\n", mode, parity);
    status = LORA_SendATCommand(cmd, response, LORA_RESPONSE_TIMEOUT_MS);
    return status;
}

/**
  * @brief  设置LoRa地址
  * @param  address: 地址值(16位)
  * @retval Lora_StatusTypeDef: 操作状态
  */
Lora_StatusTypeDef LORA_SetAddress(uint16_t address)
{
    char cmd[32];
    char response[LORA_MAX_RESPONSE_LEN];
    Lora_StatusTypeDef status;
    
    sprintf(cmd, "AT+ADDR=%d\r\n", address);
    status = LORA_SendATCommand(cmd, response, LORA_RESPONSE_TIMEOUT_MS);
    return status;
}

/**
  * @brief  设置LoRa网络ID
  * @param  netid: 网络ID
  * @retval Lora_StatusTypeDef: 操作状态
  */
Lora_StatusTypeDef LORA_SetNetID(uint8_t netid)
{
    char cmd[32];
    char response[LORA_MAX_RESPONSE_LEN];
    Lora_StatusTypeDef status;
    
    sprintf(cmd, "AT+NETID=%d\r\n", netid);
    status = LORA_SendATCommand(cmd, response, LORA_RESPONSE_TIMEOUT_MS);
    return status;
}

/**
  * @brief  设置LoRa信道
  * @param  channel: 信道号
  * @retval Lora_StatusTypeDef: 操作状态
  */
Lora_StatusTypeDef LORA_SetChannel(uint8_t channel)
{
    char cmd[32];
    char response[LORA_MAX_RESPONSE_LEN];
    Lora_StatusTypeDef status;
    
    sprintf(cmd, "AT+CHANNEL=%d\r\n", channel);
    status = LORA_SendATCommand(cmd, response, LORA_RESPONSE_TIMEOUT_MS);
    return status;
}

/**
  * @brief  设置LoRa传输模式
  * @param  mode: 传输模式 (1=点对点)
  * @retval Lora_StatusTypeDef: 操作状态
  */
Lora_StatusTypeDef LORA_SetTransMode(uint8_t mode)
{
    char cmd[32];
    char response[LORA_MAX_RESPONSE_LEN];
    Lora_StatusTypeDef status;
    
    sprintf(cmd, "AT+TRANS=%d\r\n", mode);
    status = LORA_SendATCommand(cmd, response, LORA_RESPONSE_TIMEOUT_MS);
    return status;
}

/**
  * @brief  设置LoRa UART接收超时
  * @param  urxt: 超时字节数
  * @retval Lora_StatusTypeDef: 操作状态
  */
Lora_StatusTypeDef LORA_SetURxT(uint8_t urxt)
{
    char cmd[32];
    char response[LORA_MAX_RESPONSE_LEN];
    Lora_StatusTypeDef status;
    
    sprintf(cmd, "AT+URXT=%d\r\n", urxt);
    status = LORA_SendATCommand(cmd, response, LORA_RESPONSE_TIMEOUT_MS);
    return status;
}

/**
  * @brief  LoRa模块初始化配置
  * @param  None
  * @retval Lora_StatusTypeDef: 操作状态
  */

// M0/M1控制宏定义（如未定义请在合适位置定义）
#ifndef LORA_M0
#define LORA_M0(x) HAL_GPIO_WritePin(GPIOA, M0_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#endif
#ifndef LORA_M1
#define LORA_M1(x) HAL_GPIO_WritePin(GPIOA, M1_Pin, (x) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#endif

typedef enum {
    LORA_MODE_TRANSFER = 0,
    LORA_MODE_WOR,
    LORA_MODE_CFG,
    LORA_MODE_SLEEP
} Lora_Mode_t;

static void LORA_SetMode(Lora_Mode_t mode)
{
    switch(mode){
        case LORA_MODE_TRANSFER: LORA_M0(0); LORA_M1(0); break;
        case LORA_MODE_WOR:      LORA_M0(1); LORA_M1(0); break;
        case LORA_MODE_CFG:      LORA_M0(0); LORA_M1(1); break;
        case LORA_MODE_SLEEP:    LORA_M0(1); LORA_M1(1); break;
        default: break;
    }
    LORA_DelayMs(100);
}

Lora_StatusTypeDef LORA_Init(void)
{
    Lora_StatusTypeDef status;

    // 1. 切换到配置模式
    LORA_SetMode(LORA_MODE_CFG);
    LORA_DelayMs(100);

    // 2. 测试模块连接
    status = LORA_TestConnection();
    if(status != LORA_OK) return LORA_ERROR;

    // 3. 配置参数
    if (LORA_SetUART(3, 0) != LORA_OK) return LORA_ERROR;
    if (LORA_SetAddress(LORA_DEFAULT_ADDRESS) != LORA_OK) return LORA_ERROR;
    if (LORA_SetNetID(LORA_DEFAULT_NETID) != LORA_OK) return LORA_ERROR;
    if (LORA_SetChannel(LORA_DEFAULT_CHANNEL) != LORA_OK) return LORA_ERROR;
    if (LORA_SetTransMode(LORA_DEFAULT_TRANS_MODE) != LORA_OK) return LORA_ERROR;
    if (LORA_SetURxT(LORA_DEFAULT_URXT) != LORA_OK) return LORA_ERROR;

    // 4. 切换到传输模式
    LORA_SetMode(LORA_MODE_TRANSFER);
    LORA_DelayMs(100);

    return LORA_OK;
}