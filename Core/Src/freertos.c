/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stream_buffer.h"  // 添加流缓冲区头文件
#include <string.h>  // 添加string.h头文件以使用memcpy函数
#include <stdio.h>   // 添加stdio.h头文件以使用sprintf函数
#include "usart.h"   // 添加usart头文件以使用huart1
#include "adc.h"     // 添加ADC头文件以使用ADC功能
#include "dht11.h"   // 添加DHT11头文件以使用温湿度传感器
#include "bh1750.h"  // 添加BH1750头文件以使用光照传感器
#include "delay.h"   // 添加delay头文件以使用延时函数
#include "oled.h"    // 添加OLED头文件以使用显示屏
#include "dma.h"     // 添加DMA头文件以使用hdma_usart1_rx
#include "lora_at.h" // 添加LoRa AT指令头文件以在帧里包含LORA模块地址ID
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* External DMA handle declaration */
extern DMA_HandleTypeDef hdma_usart1_rx;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STREAM_BUFFER_SIZE 256      // 流缓冲区大小
#define TRIGGER_LEVEL 1             // 触发级别
#define MAX_FRAME_LENGTH 64         // 最大帧长度
#define CMD_FRAME_HEADER 0xAA       // 命令帧头标识
#define CMD_FRAME_TAIL 0x55         // 命令帧尾标识
#define FRAME_MIN_LENGTH 5          // 最小帧长度：帧头+命令类型+数据长度+校验和+帧尾

// 命令类型定义
#define CMD_TYPE_QUERY 0x01         // 查询命令
#define CMD_TYPE_SET   0x02         // 设置命令

// 自动控制阈值定义
#define LIGHT_THRESHOLD_LOW 200     // 光照强度低阈值(lux)，低于此值开启补光
#define LIGHT_THRESHOLD_HIGH 800    // 光照强度高阈值(lux)，高于此值关闭补光
#define SOIL_MOISTURE_THRESHOLD_LOW 1.5f    // 土壤湿度低阈值(V)，低于此值开启补水
#define SOIL_MOISTURE_THRESHOLD_HIGH 2.5f   // 土壤湿度高阈值(V)，高于此值关闭补水
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Definitions for sensorTask */
osThreadId_t sensorTaskHandle;
const osThreadAttr_t sensorTask_attributes = {
  .name = "sensorTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Definitions for oledDisplayTask */
osThreadId_t oledDisplayTaskHandle;
const osThreadAttr_t oledDisplayTask_attributes = {
  .name = "oledDisplayTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartProcessTask */
osThreadId_t uartProcessTaskHandle;
const osThreadAttr_t uartProcessTask_attributes = {
    .name = "uartProcessTask",
    .stack_size = 512 * 4,
    .priority = (osPriority_t) osPriorityAboveNormal,
};
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

StreamBufferHandle_t xStreamBufferUart;  // 串口流缓冲句柄

// DMA接收相关定义
#define UART_DMA_RX_BUFFER_SIZE 256
uint8_t dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE]; // DMA接收缓冲区
volatile uint16_t dma_last_pos = 0; // 上次处理到的位置

// 协议帧结构体定义
typedef struct {
    uint8_t header;           // 帧头
    uint8_t cmd_type;         // 命令类型
    uint8_t data_len;         // 数据长度
    uint8_t data[MAX_FRAME_LENGTH-5]; // 数据域（减去帧头、命令类型、数据长度、校验和、帧尾）
    uint8_t checksum;         // 校验和
    uint8_t tail;             // 帧尾
} ProtocolFrame_t;

// 控制模式和状态变量
uint8_t control_mode = 1;           // 控制模式：1=自动，0=手动
uint8_t manual_light_state = 0;     // 手动模式下的补光状态
uint8_t manual_water_state = 0;     // 手动模式下的补水状态
uint8_t manual_buzer_state = 0;     // 手动模式下蜂鸣器状态

// 传感器初始化结果变量
uint8_t dht11_init_result = 0;      // DHT11传感器初始化结果 (0:成功, 非0:失败)

// 传感器任务相关变量
uint16_t adc_raw_value = 0;              // ADC最新原始值（DMA单次）
float adc_voltage_value = 0.0f;          // 转换后的电压值
uint16_t adc_dma_buffer[16] = {0};       // ADC DMA采样缓冲区（多点平均）
uint8_t adc_dma_sample_count = 16;       // 采样点数
uint8_t dht11_temperature = 0;           // DHT11温度值
uint8_t dht11_humidity = 0;              // DHT11湿度值
uint32_t bh1750_light_value = 0;         // BH1750光照值
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartUartProcessTask(void *argument);
void StartSensorTask(void *argument);
uint8_t calculate_checksum(uint8_t *data, uint8_t len);
uint8_t parse_command_frame(uint8_t *buffer, size_t length, ProtocolFrame_t *frame);
void send_response_frame(uint8_t cmd_type, uint8_t *data, uint8_t data_len);
void StartOledDisplayTask(void *argument);  // 添加OLED显示任务函数原型
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    // 创建流缓冲区
    xStreamBufferUart = xStreamBufferCreate(STREAM_BUFFER_SIZE, TRIGGER_LEVEL);
    if(xStreamBufferUart == NULL){
        Error_Handler();  // 如果创建失败则报错
    }

    // 启动USART1 DMA接收
    dma_last_pos = 0;
    HAL_UART_Receive_DMA(&huart1, dma_rx_buffer, UART_DMA_RX_BUFFER_SIZE);
    // 使能IDLE中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
   /* creation of sensorTask */
  sensorTaskHandle = osThreadNew(StartSensorTask, NULL, &sensorTask_attributes);
  uartProcessTaskHandle = osThreadNew(StartUartProcessTask, NULL, &uartProcessTask_attributes);
  /* creation of oledDisplayTask */
  oledDisplayTaskHandle = osThreadNew(StartOledDisplayTask, NULL, &oledDisplayTask_attributes);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


// 传感器读取任务 - 包含ADC读取和校准
void StartSensorTask(void *argument)
{
    // ADC采样相关变量
    uint32_t adc_raw_sum = 0;
    
    uint8_t dht11_read_result = 0;    // DHT11读取结果
    static uint8_t sensor_init_done = 0; // 传感器初始化标志
    
    // 自动控制状态变量
    static uint8_t auto_light_state = 0;     // 自动补光控制状态 (0:关闭, 1:开启)
    static uint8_t auto_water_state = 0;     // 自动补水控制状态 (0:关闭, 1:开启)



    // 在开始转换之前先进行ADC校准
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        Error_Handler(); // 校准失败，进入错误处理
    }

    // 启动ADC DMA多点采集
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_dma_buffer, adc_dma_sample_count) != HAL_OK)
    {
        Error_Handler(); // 启动转换失败，进入错误处理
    }

    for(;;)
    {
        // 第一次运行时初始化传感器
        if (!sensor_init_done)
        {
            // 初始化DHT11传感器
            dht11_init_result = DHT11_Init();
            if (dht11_init_result != 0)
            {
                // DHT11初始化失败处理
                dht11_temperature = 0;
                dht11_humidity = 0;
            }
            
            // 初始化BH1750传感器
            delay_init();  // 初始化延时函数
            Init_BH1750(); // 初始化BH1750
            
            sensor_init_done = 1;
        }
        
        // 采集一组DMA数据后求平均
        adc_raw_sum = 0;
        for (uint8_t i = 0; i < adc_dma_sample_count; i++) {
            adc_raw_sum += adc_dma_buffer[i];
        }
        uint16_t avg_adc_value = adc_raw_sum / adc_dma_sample_count;
        adc_raw_value = avg_adc_value; // 记录最新平均值
        adc_voltage_value = ((float)avg_adc_value * 3.3f) / 4096.0f;
        
        // 读取DHT11温湿度传感器数据（每100ms读取一次）
        static uint32_t dht11_timer = 0;
        if (HAL_GetTick() - dht11_timer >= 100) // 100ms间隔
        {
            if (dht11_init_result == 0) // 只有初始化成功才读取
            {
                dht11_read_result = DHT11_Read_Data(&dht11_temperature, &dht11_humidity);
                if (dht11_read_result != 0)
                {
                    // 读取失败时保持原有值或设置默认值
                    dht11_temperature = 0;
                    dht11_humidity = 0;
                }
            }
            dht11_timer = HAL_GetTick();
        }
        
        // 读取BH1750光照传感器数据（每200ms读取一次）
        static uint32_t bh1750_timer = 0;
        if (HAL_GetTick() - bh1750_timer >= 200) // 200ms间隔
        {

                bh1750_light_value = Value_GY30(); // 读取光照强度值
           
            bh1750_timer = HAL_GetTick();
        }
        
        // === 控制逻辑 ===
        if (control_mode == 1) // 自动模式
        {
            // 补光自动控制逻辑 (基于BH1750光照传感器)
            if (bh1750_light_value < LIGHT_THRESHOLD_LOW && auto_light_state == 0)
            {
                // 光照不足，开启补光
                HAL_GPIO_WritePin(fill_led_GPIO_Port, fill_led_Pin, GPIO_PIN_SET);
                auto_light_state = 1;
            }
            else if (bh1750_light_value > LIGHT_THRESHOLD_HIGH && auto_light_state == 1)
            {
                // 光照充足，关闭补光
                HAL_GPIO_WritePin(fill_led_GPIO_Port, fill_led_Pin, GPIO_PIN_RESET);
                auto_light_state = 0;
            }
            
            // 补水自动控制逻辑 (基于ADC土壤湿度传感器)
            if (adc_voltage_value > 0.1f) // 确保ADC读数有效
            {
                if (adc_voltage_value < SOIL_MOISTURE_THRESHOLD_LOW && auto_water_state == 0)
                {
                    // 土壤湿度过低，开启补水
                    HAL_GPIO_WritePin(fill_water_GPIO_Port, fill_water_Pin, GPIO_PIN_SET);
                    auto_water_state = 1;
                }
                else if (adc_voltage_value > SOIL_MOISTURE_THRESHOLD_HIGH && auto_water_state == 1)
                {
                    // 土壤湿度充足，关闭补水
                    HAL_GPIO_WritePin(fill_water_GPIO_Port, fill_water_Pin, GPIO_PIN_RESET);
                    auto_water_state = 0;
                }
            }
        }
        else // 手动模式
        {
            // 根据手动状态设置GPIO
            if (manual_light_state == 1)
                HAL_GPIO_WritePin(fill_led_GPIO_Port, fill_led_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(fill_led_GPIO_Port, fill_led_Pin, GPIO_PIN_RESET);

            if (manual_water_state == 1)
                HAL_GPIO_WritePin(fill_water_GPIO_Port, fill_water_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(fill_water_GPIO_Port, fill_water_Pin, GPIO_PIN_RESET);

            if (manual_buzer_state == 1)
                HAL_GPIO_WritePin(buzer_GPIO_Port, buzer_Pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(buzer_GPIO_Port, buzer_Pin, GPIO_PIN_RESET);
        }

        // 传感器读取间隔 - 例如每100ms读取一次
        osDelay(100);
    }
}

// 新版数据处理任务 - 状态机逐字节解析帧，支持粘包/乱序/丢包
typedef enum {
    FRAME_STATE_IDLE = 0,
    FRAME_STATE_HEADER,
    FRAME_STATE_CMD,
    FRAME_STATE_LEN,
    FRAME_STATE_DATA,
    FRAME_STATE_CHECKSUM,
    FRAME_STATE_TAIL
} FrameParseState_t;

typedef struct {
    FrameParseState_t state;
    uint8_t buffer[MAX_FRAME_LENGTH];
    uint8_t length;
    uint8_t data_len;
    uint8_t checksum;
    uint8_t data_index;
} FrameParser_t;

static FrameParser_t g_frame_parser;

static void reset_frame_parser(void) {
    memset(&g_frame_parser, 0, sizeof(g_frame_parser));
    g_frame_parser.state = FRAME_STATE_IDLE;
}

void StartUartProcessTask(void *argument)
{
    uint8_t rx_buffer[64];
    int len = 0;
    reset_frame_parser();
    for(;;)
    {
        len = xStreamBufferReceive(xStreamBufferUart, rx_buffer, sizeof(rx_buffer), pdMS_TO_TICKS(20));
        if (len > 0) {
            for (int i = 0; i < len; i++) {
                uint8_t byte = rx_buffer[i];
                switch (g_frame_parser.state) {
                    case FRAME_STATE_IDLE:
                        if (byte == CMD_FRAME_HEADER) {
                            g_frame_parser.buffer[0] = byte;
                            g_frame_parser.length = 1;
                            g_frame_parser.state = FRAME_STATE_CMD;
                        }
                        break;
                    case FRAME_STATE_CMD:
                        g_frame_parser.buffer[g_frame_parser.length++] = byte;
                        g_frame_parser.state = FRAME_STATE_LEN;
                        break;
                    case FRAME_STATE_LEN:
                        g_frame_parser.buffer[g_frame_parser.length++] = byte;
                        g_frame_parser.data_len = byte;
                        g_frame_parser.data_index = 0;
                        if (g_frame_parser.data_len > (MAX_FRAME_LENGTH-5)) {
                            reset_frame_parser();
                        } else if (g_frame_parser.data_len == 0) {
                            g_frame_parser.state = FRAME_STATE_CHECKSUM;
                        } else {
                            g_frame_parser.state = FRAME_STATE_DATA;
                        }
                        break;
                    case FRAME_STATE_DATA:
                        g_frame_parser.buffer[g_frame_parser.length++] = byte;
                        g_frame_parser.data_index++;
                        if (g_frame_parser.data_index >= g_frame_parser.data_len) {
                            g_frame_parser.state = FRAME_STATE_CHECKSUM;
                        }
                        break;
                    case FRAME_STATE_CHECKSUM:
                        g_frame_parser.buffer[g_frame_parser.length++] = byte;
                        g_frame_parser.checksum = byte;
                        g_frame_parser.state = FRAME_STATE_TAIL;
                        break;
                    case FRAME_STATE_TAIL:
                        g_frame_parser.buffer[g_frame_parser.length++] = byte;
                        if (byte == CMD_FRAME_TAIL) {
                            // 校验和校验
                            uint8_t calc_sum = 0;
                            for (int k = 0; k < g_frame_parser.length-2; k++) {
                                calc_sum += g_frame_parser.buffer[k];
                            }
                            if (calc_sum == g_frame_parser.checksum) {
                                // 直接处理帧
                                uint8_t cmd_type = g_frame_parser.buffer[1];
                                uint8_t data_len = g_frame_parser.buffer[2];
                                uint8_t *data = &g_frame_parser.buffer[3];
                                if (cmd_type == CMD_TYPE_QUERY) {
                                    send_response_frame(CMD_TYPE_QUERY, NULL, 0);
                                } else if (cmd_type == CMD_TYPE_SET) {
                                    if (data_len >= 3) {
                                        manual_light_state = (data[0] > 0) ? 1 : 0;
                                        manual_water_state = (data[1] > 0) ? 1 : 0;
                                        manual_buzer_state = (data[2] > 0) ? 1 : 0;
                                        control_mode = 0;
                                        send_response_frame(CMD_TYPE_SET, NULL, 0);
                                    }
                                } else {
                                    uint8_t error_data[1] = {0xFF};
                                    send_response_frame(cmd_type, error_data, 1);
                                }
                            }
                        }
                        reset_frame_parser();
                        break;
                    default:
                        reset_frame_parser();
                        break;
                }
            }
        }
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}


// 计算校验和函数
uint8_t calculate_checksum(uint8_t *data, uint8_t len) {
    uint8_t sum = 0;
    for(int i = 0; i < len; i++) {
        sum += data[i];
    }
    return sum; // 简单的累加和校验
}


// 发送响应帧函数（带边界检查）
void send_response_frame(uint8_t cmd_type, uint8_t *data, uint8_t data_len) {
    uint8_t response_frame[MAX_FRAME_LENGTH];
    uint8_t index = 0;

    // 帧头、命令类型、占位的数据长度（稍后更新）
    response_frame[index++] = CMD_FRAME_HEADER;
    response_frame[index++] = cmd_type;
    response_frame[index++] = 0; // 占位

    if (cmd_type == CMD_TYPE_QUERY || cmd_type == CMD_TYPE_SET)
    {
        uint8_t sensor_data_len = 2 + 4 + 1 + 1 + 4 + 1 + 3; // 18
        if (sensor_data_len > (MAX_FRAME_LENGTH - 5)) {
            sensor_data_len = MAX_FRAME_LENGTH - 5;
        }
        // 更新长度字段
        response_frame[2] = sensor_data_len;

        // 节点ID (2)
        if (index + 2 <= MAX_FRAME_LENGTH - 2) {
            response_frame[index++] = (uint8_t)(LORA_DEFAULT_ADDRESS >> 8);
            response_frame[index++] = (uint8_t)(LORA_DEFAULT_ADDRESS & 0xFF);
        }

        // ADC 电压 (float -> 4 bytes)
        union { float f; uint8_t b[4]; } vu;
        vu.f = adc_voltage_value;
        for (int i = 0; i < 4 && index < MAX_FRAME_LENGTH - 2; i++) response_frame[index++] = vu.b[i];

        if (index < MAX_FRAME_LENGTH - 2) response_frame[index++] = dht11_temperature;
        if (index < MAX_FRAME_LENGTH - 2) response_frame[index++] = dht11_humidity;

        // BH1750 光照 (4 bytes)
        if (index + 4 <= MAX_FRAME_LENGTH - 2) {
            response_frame[index++] = (uint8_t)(bh1750_light_value >> 24);
            response_frame[index++] = (uint8_t)(bh1750_light_value >> 16);
            response_frame[index++] = (uint8_t)(bh1750_light_value >> 8);
            response_frame[index++] = (uint8_t)(bh1750_light_value & 0xFF);
        }

        if (index < MAX_FRAME_LENGTH - 2) response_frame[index++] = control_mode;

        // 实际GPIO状态
        uint8_t actual_light_state = (HAL_GPIO_ReadPin(fill_led_GPIO_Port, fill_led_Pin) == GPIO_PIN_SET) ? 1 : 0;
        uint8_t actual_water_state = (HAL_GPIO_ReadPin(fill_water_GPIO_Port, fill_water_Pin) == GPIO_PIN_SET) ? 1 : 0;
        uint8_t actual_buzer_state = (HAL_GPIO_ReadPin(buzer_GPIO_Port, buzer_Pin) == GPIO_PIN_SET) ? 1 : 0;
        if (index < MAX_FRAME_LENGTH - 2) response_frame[index++] = actual_light_state;
        if (index < MAX_FRAME_LENGTH - 2) response_frame[index++] = actual_water_state;
        if (index < MAX_FRAME_LENGTH - 2) response_frame[index++] = actual_buzer_state;
    }
    else
    {
        // 拷贝入参数据（受限边界）
        for (int i = 0; i < data_len && index < MAX_FRAME_LENGTH - 2; i++) {
            response_frame[index++] = data[i];
        }
        // 更新长度字段为实际拷贝的字节数
        response_frame[2] = (uint8_t)((index > 3) ? (index - 3) : 0);
    }

    // 计算并添加校验和
    uint8_t checksum = calculate_checksum(response_frame, index);
    if (index < MAX_FRAME_LENGTH - 1) {
        response_frame[index++] = checksum;
    } else {
        response_frame[MAX_FRAME_LENGTH - 2] = checksum;
    }

    // 添加帧尾
    if (index < MAX_FRAME_LENGTH) {
        response_frame[index++] = CMD_FRAME_TAIL;
    } else {
        response_frame[MAX_FRAME_LENGTH - 1] = CMD_FRAME_TAIL;
        index = MAX_FRAME_LENGTH;
    }

    HAL_UART_Transmit(&huart1, response_frame, index, HAL_MAX_DELAY);
}

// OLED显示任务 - 显示传感器数据和开关状态
void StartOledDisplayTask(void *argument)
{
    char display_buffer[32];
    static uint8_t oled_initialized = 0;
    
    // 初始化OLED显示屏
    if (!oled_initialized)
    {
        OLED_Init();
        OLED_Clear();
        OLED_Display_On();
        oled_initialized = 1;
    }
    
    for(;;)
    {
        // 清屏
        OLED_Clear();
        
        // 第一行：土壤湿度 (ADC电压值)
        sprintf(display_buffer, "Soil:%.2fV", adc_voltage_value);
        OLED_ShowString(0, 0, display_buffer, 12);
        
        // 第二行：温湿度 (DHT11)
        sprintf(display_buffer, "T:%dC H:%d%%", dht11_temperature, dht11_humidity);
        OLED_ShowString(0, 2, display_buffer, 12);
        
        // 第三行：光照强度 (BH1750)
        sprintf(display_buffer, "Light:%dlux", (int)bh1750_light_value);
        OLED_ShowString(0, 4, display_buffer, 12);
        
        // 第四行：开关状态
        uint8_t light_state = (HAL_GPIO_ReadPin(fill_led_GPIO_Port, fill_led_Pin) == GPIO_PIN_SET) ? 1 : 0;
        uint8_t water_state = (HAL_GPIO_ReadPin(fill_water_GPIO_Port, fill_water_Pin) == GPIO_PIN_SET) ? 1 : 0;
        
        sprintf(display_buffer, "LED:%s WAT:%s", 
                light_state ? "ON " : "OFF", 
                water_state ? "ON " : "OFF");
        OLED_ShowString(0, 6, display_buffer, 12);
        
        // 刷新显示
        // OLED刷新已经在ShowString中完成
        
        // 每2秒更新一次显示
        osDelay(2000);
    }
}
// USART1空闲中断处理函数：将DMA缓冲区有效数据写入FreeRTOS流缓冲区
void USART1_IdleLine_IRQHandler(void)
{
    uint16_t dma_curr_pos = UART_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); // 当前写入位置
    uint16_t data_len = 0;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (dma_curr_pos >= UART_DMA_RX_BUFFER_SIZE) dma_curr_pos = 0; // 防止越界

    if (dma_curr_pos != dma_last_pos)
    {
        if (dma_curr_pos > dma_last_pos)
        {
            // 无环绕，直接拷贝
            data_len = dma_curr_pos - dma_last_pos;
            xStreamBufferSendFromISR(xStreamBufferUart, &dma_rx_buffer[dma_last_pos], data_len, &xHigherPriorityTaskWoken);
        }
        else
        {
            // 发生环绕，先拷贝末尾，再拷贝起始
            data_len = UART_DMA_RX_BUFFER_SIZE - dma_last_pos;
            xStreamBufferSendFromISR(xStreamBufferUart, &dma_rx_buffer[dma_last_pos], data_len, &xHigherPriorityTaskWoken);
            if (dma_curr_pos > 0)
            {
                xStreamBufferSendFromISR(xStreamBufferUart, &dma_rx_buffer[0], dma_curr_pos, &xHigherPriorityTaskWoken);
            }
        }
        dma_last_pos = dma_curr_pos;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
/* USER CODE END Application */

