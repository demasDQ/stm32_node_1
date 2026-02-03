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
#include "semphr.h"  // 添加信号量头文件
#include "stream_buffer.h"  // 添加流缓冲区头文件
#include <string.h>  // 添加string.h头文件以使用memcpy函数
#include "usart.h"   // 添加usart头文件以使用huart1
#include "adc.h"     // 添加ADC头文件以使用ADC功能
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STREAM_BUFFER_SIZE 256      // 流缓冲区大小
#define TRIGGER_LEVEL 1             // 触发级别
#define MAX_FRAME_LENGTH 64         // 最大帧长度
#define CMD_FRAME_HEADER 0xAA       // 命令帧头标识
#define CMD_FRAME_TAIL 0x55         // 命令帧尾标识
#define FRAME_MIN_LENGTH 5          // 最小帧长度：帧头+命令类型+数据长度+校验和+帧尾
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
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
StreamBufferHandle_t xStreamBufferUart;  // 串口流缓冲句柄
uint8_t rx_data_buffer[1];               // 全局接收缓冲区

// 协议帧结构体定义
typedef struct {
    uint8_t header;           // 帧头
    uint8_t cmd_type;         // 命令类型
    uint8_t data_len;         // 数据长度
    uint8_t data[MAX_FRAME_LENGTH-5]; // 数据域，减去帧头+命令类型+数据长度+校验和+帧尾
    uint8_t checksum;         // 校验和
    uint8_t tail;             // 帧尾
} ProtocolFrame_t;

// 信号量用于控制发送权限和通知数据处理结果
SemaphoreHandle_t xUartSendSemaphore;
SemaphoreHandle_t xFrameParsedSemaphore;

// 用于存储解析成功的帧，以便发送任务可以访问
ProtocolFrame_t parsed_frame;

// 传感器任务相关变量
uint16_t adc_raw_value = 0;              // ADC原始值
float adc_voltage_value = 0.0f;          // 转换后的电压值
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
void StartUartSendTask(void *argument);
void StartUartReceiveTask(void *argument);
void StartUartProcessTask(void *argument);
void StartSensorTask(void *argument);
void StartUartReception(void);
uint8_t calculate_checksum(uint8_t *data, uint8_t len);
uint8_t parse_command_frame(uint8_t *buffer, size_t length, ProtocolFrame_t *frame);
void send_response_frame(uint8_t cmd_type, uint8_t *data, uint8_t data_len);
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
  

  
  // 创建二值信号量用于通知发送任务有帧已解析
  xFrameParsedSemaphore = xSemaphoreCreateBinary();
  if(xFrameParsedSemaphore == NULL){
    Error_Handler();  // 如果创建失败则报错
  }
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
  osThreadNew(StartUartProcessTask, NULL, &defaultTask_attributes);
  // 创建串口发送任务
  const osThreadAttr_t uartSendTask_attributes = {
    .name = "UartSendTask",
    .stack_size = 256 * 4,
    .priority = (osPriority_t) osPriorityBelowNormal,
  };
  osThreadNew(StartUartSendTask, NULL, &uartSendTask_attributes);
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
    uint32_t adc_raw_sum = 0;
    uint8_t adc_readings_count = 0;
    const uint8_t NUM_READINGS = 10;  // 平均值采样次数

    // 初始化ADC
    MX_ADC1_Init();

    // 在开始转换之前先进行ADC校准
    if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK)
    {
        Error_Handler(); // 校准失败，进入错误处理
    }

    // 启动ADC并开始DMA转换
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_raw_value, 1) != HAL_OK)
    {
        Error_Handler(); // 启动转换失败，进入错误处理
    }

    for(;;)
    {
        // 读取ADC值
        adc_raw_value = HAL_ADC_GetValue(&hadc1); // 从DMA缓冲区获取最新ADC值
        
        // 累加ADC值用于平均计算
        adc_raw_sum += adc_raw_value;
        adc_readings_count++;

        // 当达到指定采样数时，计算平均值并转换为电压值
        if (adc_readings_count >= NUM_READINGS)
        {
            // 计算平均ADC值
            uint16_t avg_adc_value = adc_raw_sum / NUM_READINGS;
            
            // 将ADC值转换为电压值 (假设参考电压为3.3V，分辨率为12位)
            adc_voltage_value = ((float)avg_adc_value * 3.3f) / 4096.0f;
            
            // 重置计数器
            adc_raw_sum = 0;
            adc_readings_count = 0;
            
            // 打印当前ADC值和电压值（如果需要的话，可以通过串口输出调试信息）
            // printf("ADC Raw Value: %d, Voltage: %.2f V\r\n", avg_adc_value, adc_voltage_value);
        }

        // 传感器读取间隔 - 例如每100ms读取一次
        osDelay(100);
    }
}

// 数据处理任务 - 从流缓冲区读取数据并解析帧
void StartUartProcessTask(void *argument)
{
    uint8_t temp_buffer[MAX_FRAME_LENGTH]; // 临时缓冲区用于存储可能的完整帧
    size_t received_bytes;
    ProtocolFrame_t frame;

    for(;;)
    {
        // 尝试读取至少最小帧长度的数据
        received_bytes = xStreamBufferReceive(xStreamBufferUart, temp_buffer, FRAME_MIN_LENGTH, pdMS_TO_TICKS(100));
        
        if(received_bytes >= FRAME_MIN_LENGTH)
        {
            // 检查是否有足够的数据构成完整帧（包括可能的数据字段）
            if(temp_buffer[0] == CMD_FRAME_HEADER) // 验证帧头
            {
                uint8_t expected_length = FRAME_MIN_LENGTH + temp_buffer[2]; // 帧头+命令类型+数据长度+校验和+帧尾+数据域
                
                // 如果当前接收的数据不足一帧，尝试接收剩余部分
                if(received_bytes < expected_length)
                {
                    size_t remaining_bytes = expected_length - received_bytes;
                    size_t additional_bytes = xStreamBufferReceive(xStreamBufferUart, 
                                                                 &temp_buffer[received_bytes], 
                                                                 remaining_bytes, 
                                                                 pdMS_TO_TICKS(100));
                    
                    if(additional_bytes == remaining_bytes)
                    {
                        received_bytes = expected_length;
                    }
                    else
                    {
                        // 没有接收到完整的帧，跳过这个字节并继续
                        xStreamBufferReceive(xStreamBufferUart, temp_buffer, 1, 0);
                        continue;
                    }
                }
                
                // 尝试解析命令帧
                if(parse_command_frame(temp_buffer, received_bytes, &frame))
                {
                    // 解析成功，保存帧数据并释放信号量通知发送任务
                    memcpy(&parsed_frame, &frame, sizeof(ProtocolFrame_t));
                    xSemaphoreGive(xFrameParsedSemaphore);
                }
            }
            else
            {
                // 帧头不匹配，跳过这个字节
                xStreamBufferReceive(xStreamBufferUart, temp_buffer, 1, 0);
            }
        }
        else
        {
            // 超时没有接收到足够数据，继续等待
            continue;
        }
    }
}

// 串口发送任务 - 等待数据处理任务的通知后发送响应
void StartUartSendTask(void *argument)
{
    for(;;)
    {
        // 等待数据处理任务发出的信号，表示有帧被成功解析
        if(xSemaphoreTake(xFrameParsedSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // 发送对应的响应帧
            send_response_frame(parsed_frame.cmd_type, parsed_frame.data, parsed_frame.data_len);
        }
    }
}

// 启动串口接收中断的函数
void StartUartReception(void) {
  // 启动UART接收中断，每次接收一个字节
  HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_data_buffer, 1);
}

// 重写HAL库的UART接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  
  if(huart->Instance == USART1) {
    // 将接收到的数据写入流缓冲区
    xStreamBufferSendFromISR(xStreamBufferUart, (void*)rx_data_buffer, 1, &xHigherPriorityTaskWoken);
    
    // 继续启动下一次接收
    HAL_UART_Receive_IT(&huart1, (uint8_t*)rx_data_buffer, 1);
    
    // 如果高优先级任务被唤醒，则需要执行上下文切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

// 解析命令帧函数
uint8_t parse_command_frame(uint8_t *buffer, size_t length, ProtocolFrame_t *frame) {
    if(buffer == NULL || frame == NULL || length < FRAME_MIN_LENGTH) {
        return 0; // 帧太短，不是有效帧
    }

    // 验证帧头
    if(buffer[0] != CMD_FRAME_HEADER) {
        return 0; // 帧头不匹配
    }

    // 获取数据长度
    uint8_t data_len = buffer[2];
    if(length != (size_t)(FRAME_MIN_LENGTH + data_len)) {
        return 0; // 长度不匹配
    }

    // 复制帧数据
    frame->header = buffer[0];
    frame->cmd_type = buffer[1];
    frame->data_len = buffer[2];
    for(int i = 0; i < data_len && i < MAX_FRAME_LENGTH-4; i++) {
        frame->data[i] = buffer[3+i];
    }
    frame->checksum = buffer[3+data_len]; // 最后一个是校验位

    // 验证帧尾
    if(buffer[3 + data_len] != CMD_FRAME_TAIL) {
        return 0; // 帧尾不匹配
    }

    // 复制帧数据
    frame->header = buffer[0];
    frame->cmd_type = buffer[1];
    frame->data_len = buffer[2];
    for(int i = 0; i < data_len && i < MAX_FRAME_LENGTH-5; i++) {
        frame->data[i] = buffer[3+i];
    }
    frame->checksum = buffer[3+data_len];     // 校验和
    frame->tail = buffer[4+data_len];        // 帧尾

    // 验证校验和
    uint8_t calculated_checksum = calculate_checksum(buffer, 3 + data_len);
    if(calculated_checksum != frame->checksum) {
        return 0; // 校验和错误
    }

    return 1; // 解析成功
}

// 发送响应帧函数
void send_response_frame(uint8_t cmd_type, uint8_t *data, uint8_t data_len) {
    uint8_t response_frame[MAX_FRAME_LENGTH];
    uint8_t index = 0;

    // 构建响应帧
    response_frame[index++] = CMD_FRAME_HEADER;  // 帧头
    response_frame[index++] = cmd_type + 0x80;   // 响应命令类型（约定为请求命令+0x80）
    response_frame[index++] = data_len;          // 数据长度

    // 添加数据
    for(int i = 0; i < data_len && i < MAX_FRAME_LENGTH-4; i++) {
        response_frame[index++] = data[i];
    }

    // 计算并添加校验和
    uint8_t checksum = calculate_checksum(response_frame, index);
    response_frame[index++] = checksum;
    
    // 添加帧尾
    response_frame[index++] = CMD_FRAME_TAIL;

    // 通过UART发送响应
    HAL_UART_Transmit(&huart1, response_frame, index, HAL_MAX_DELAY);
}

/* USER CODE END Application */

