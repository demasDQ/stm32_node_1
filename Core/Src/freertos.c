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
    uint8_t data[MAX_FRAME_LENGTH-5]; // 数据域（减去帧头、命令类型、数据长度、校验和、帧尾）
    uint8_t checksum;         // 校验和
    uint8_t tail;             // 帧尾
} ProtocolFrame_t;

// 控制模式和状态变量
uint8_t control_mode = 1;           // 控制模式：1=自动，0=手动
uint8_t manual_light_state = 0;     // 手动模式下的补光状态
uint8_t manual_water_state = 0;     // 手动模式下的补水状态

// 传感器初始化结果变量
uint8_t dht11_init_result = 0;      // DHT11传感器初始化结果 (0:成功, 非0:失败)

// 传感器任务相关变量
uint16_t adc_raw_value = 0;              // ADC原始值
float adc_voltage_value = 0.0f;          // 转换后的电压值
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
void StartUartReceiveTask(void *argument);
void StartUartProcessTask(void *argument);
void StartSensorTask(void *argument);
void StartUartReception(void);
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
    uint32_t adc_raw_sum = 0;
    uint8_t adc_readings_count = 0;
    const uint8_t NUM_READINGS = 10;  // 平均值采样次数
    
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

    // 启动ADC并开始DMA转换
    if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adc_raw_value, 1) != HAL_OK)
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
        }
        
        // 读取DHT11温湿度传感器数据（每2秒读取一次）
        static uint32_t dht11_timer = 0;
        if (HAL_GetTick() - dht11_timer >= 2000) // 2秒间隔
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
        
        // 读取BH1750光照传感器数据（每1秒读取一次）
        static uint32_t bh1750_timer = 0;
        if (HAL_GetTick() - bh1750_timer >= 1000) // 1秒间隔
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
            {
                HAL_GPIO_WritePin(fill_led_GPIO_Port, fill_led_Pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(fill_led_GPIO_Port, fill_led_Pin, GPIO_PIN_RESET);
            }
            
            if (manual_water_state == 1)
            {
                HAL_GPIO_WritePin(fill_water_GPIO_Port, fill_water_Pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(fill_water_GPIO_Port, fill_water_Pin, GPIO_PIN_RESET);
            }
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
                    // 根据命令类型处理
                    if (frame.cmd_type == CMD_TYPE_QUERY)
                    {
                        // 查询命令 - 发送传感器数据
                        send_response_frame(CMD_TYPE_QUERY, NULL, 0);
                    }
                    else if (frame.cmd_type == CMD_TYPE_SET)
                    {
                        // 设置命令 - 切换为手动模式并读取开关状态
                        if (frame.data_len >= 2)
                        {
                            // 读取两个开关状态
                            manual_light_state = (frame.data[0] > 0) ? 1 : 0;
                            manual_water_state = (frame.data[1] > 0) ? 1 : 0;
                            
                            // 切换为手动模式
                            control_mode = 0;
                            
                            // 发送确认响应
                            send_response_frame(CMD_TYPE_SET, NULL, 0);
                        }
                    }
                    else
                    {
                        // 未知命令类型，发送错误响应
                        uint8_t error_data[1] = {0xFF}; // 错误代码
                        send_response_frame(frame.cmd_type, error_data, 1);
                    }
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

    // 获取命令类型和数据长度
    frame->cmd_type = buffer[1];
    frame->data_len = buffer[2];
    
    // 计算期望的帧长度（包括帧尾）
    uint8_t expected_length = 4 + frame->data_len; // 帧头+命令类型+数据长度+数据域+校验和+帧尾
    
    if(length != (size_t)expected_length) {
        return 0; // 长度不匹配
    }

    // 验证帧尾
    if(buffer[3 + frame->data_len] != CMD_FRAME_TAIL) {
        return 0; // 帧尾不匹配
    }

    // 复制数据域
    for(int i = 0; i < frame->data_len && i < MAX_FRAME_LENGTH-5; i++) {
        frame->data[i] = buffer[3+i];
    }
    
    // 提取校验和
    frame->checksum = buffer[3 + frame->data_len];

    // 验证校验和（从帧头到校验和，不包含帧尾）
    uint8_t calculated_checksum = calculate_checksum(buffer, 3 + frame->data_len);
    if(calculated_checksum != frame->checksum) {
        return 0; // 校验和错误
    }

    // 设置帧头和帧尾
    frame->header = buffer[0];
    frame->tail = CMD_FRAME_TAIL;

    return 1; // 解析成功
}

// 发送响应帧函数
void send_response_frame(uint8_t cmd_type, uint8_t *data, uint8_t data_len) {
    uint8_t response_frame[MAX_FRAME_LENGTH];
    uint8_t index = 0;

    // 构建响应帧
    response_frame[index++] = CMD_FRAME_HEADER;  // 帧头
    response_frame[index++] = cmd_type;          // 命令类型（保持原样）
    response_frame[index++] = data_len;          // 数据长度

    // 如果是查询命令，添加传感器数据
    if (cmd_type == CMD_TYPE_QUERY)
    {
        // 计算传感器数据长度：节点ID(1) + ADC原始值(2) + ADC电压值(4) + DHT11温度(1) + DHT11湿度(1) + BH1750光照值(4) + 控制模式(1) + 手动状态(2) + 实际开关状态(2)
        uint8_t sensor_data_len = 1 + 2 + 4 + 1 + 1 + 4 + 1 + 2 + 2;  // 总共18字节数据
        
        // 添加节点ID (0x01)
        response_frame[index++] = 0x01;
        
        // 添加ADC原始值 (16位)
        response_frame[index++] = (uint8_t)(adc_raw_value >> 8);    // 高字节
        response_frame[index++] = (uint8_t)(adc_raw_value & 0xFF);  // 低字节
        
        // 添加ADC电压值 (32位浮点数)
        union {
            float f;
            uint8_t bytes[4];
        } voltage_union;
        voltage_union.f = adc_voltage_value;
        response_frame[index++] = voltage_union.bytes[0];
        response_frame[index++] = voltage_union.bytes[1];
        response_frame[index++] = voltage_union.bytes[2];
        response_frame[index++] = voltage_union.bytes[3];
        
        // 添加DHT11温度值
        response_frame[index++] = dht11_temperature;
        
        // 添加DHT11湿度值
        response_frame[index++] = dht11_humidity;
        
        // 添加BH1750光照值 (32位)
        response_frame[index++] = (uint8_t)(bh1750_light_value >> 24);  // 最高字节
        response_frame[index++] = (uint8_t)(bh1750_light_value >> 16);
        response_frame[index++] = (uint8_t)(bh1750_light_value >> 8);
        response_frame[index++] = (uint8_t)(bh1750_light_value & 0xFF); // 最低字节
        
        // 添加控制模式
        response_frame[index++] = control_mode;
        
        // 添加实际的GPIO开关状态
        uint8_t actual_light_state = (HAL_GPIO_ReadPin(fill_led_GPIO_Port, fill_led_Pin) == GPIO_PIN_SET) ? 1 : 0;
        uint8_t actual_water_state = (HAL_GPIO_ReadPin(fill_water_GPIO_Port, fill_water_Pin) == GPIO_PIN_SET) ? 1 : 0;
        response_frame[index++] = actual_light_state;
        response_frame[index++] = actual_water_state;
        
        data_len = sensor_data_len;
    }
    else
    {
        // 添加传入的数据
        for(int i = 0; i < data_len && i < MAX_FRAME_LENGTH-4; i++) {
            response_frame[index++] = data[i];
        }
    }

    // 计算并添加校验和
    uint8_t checksum = calculate_checksum(response_frame, index);
    response_frame[index++] = checksum;
    
    // 添加帧尾
    response_frame[index++] = CMD_FRAME_TAIL;

    // 通过UART发送响应
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
/* USER CODE END Application */

