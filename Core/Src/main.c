/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention      :
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "can.h"
#include "dma.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "other.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern DMA_HandleTypeDef hdma_uart5_rx; // DMA句柄
// 定义接收缓冲区大小
#define RX_BUFFER_SIZE 128
uint8_t rx_buffer[RX_BUFFER_SIZE]; // 接收缓冲区

// 陀螺仪数据处理相关变量
uint8_t gyro_data_ready = 0; // 数据准备标志
uint16_t rx_write_pos = 0;   // DMA写入位置
uint16_t rx_read_pos = 0;    // 数据读取位置

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USB_HOST_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_UART5_Init(); /* USER CODE BEGIN 2 */
  // 启动UART5的DMA接收
  HAL_StatusTypeDef dma_status = HAL_UART_Receive_DMA(&huart5, rx_buffer, RX_BUFFER_SIZE);
  if (dma_status != HAL_OK)
  {
    printf("DMA Init Failed: %d\r\n", dma_status);
  }
  else
  {
    printf("DMA Init Success\r\n");
  }

  // 启用UART5的IDLE中断
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);

  // 测试printf是否工作
  printf("System Init OK\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint32_t heartbeat_time = 0;
  uint16_t last_dma_pos = 0;

  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    // 定期处理DMA缓冲区数据
    Process_DMA_Buffer();

    // 处理陀螺仪数据
    if (gyro_data_ready)
    {
      gyro_data_ready = 0; // 清除标志
    } // 每隔2秒输出心跳和DMA状态
    if (HAL_GetTick() - heartbeat_time > 2000)
    {
      heartbeat_time = HAL_GetTick();
      uint16_t current_dma_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);
      printf("Heartbeat - DMA pos: %d, Last: %d\r\n", current_dma_pos, last_dma_pos);

      // 显示当前缓冲区中的数据（前10个字节）
      printf("Buffer data: ");
      for (int i = 0; i < 10 && i < current_dma_pos; i++)
      {
        printf("0x%02X ", rx_buffer[i]);
      }
      printf("\r\n");

      // 检查DMA是否有数据变化
      if (current_dma_pos != last_dma_pos)
      {
        printf("DMA data received!\r\n");
        last_dma_pos = current_dma_pos;
      }
      else
      {
        printf("No DMA data change\r\n");

        // 显示接收到的那1个字节的详细信息
        if (current_dma_pos == 1)
        {
          printf("Only 1 byte received: 0x%02X (decimal: %d, char: '%c')\r\n",
                 rx_buffer[0], rx_buffer[0],
                 (rx_buffer[0] >= 32 && rx_buffer[0] <= 126) ? rx_buffer[0] : '?');
        }
      }
    }

    HAL_Delay(1); // 避免CPU占用过高

    /* USER CODE END 3 */
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// 陀螺仪数据解析函数
void Parse_Gyro_Data(uint8_t *buffer, uint16_t start_pos)
{

  uint8_t checksum;
  int16_t data[3]; // 用于存储解析后的16位数据

  // 校验和验证
  checksum = 0x55 + buffer[(start_pos + 1) % RX_BUFFER_SIZE]; // 包头+类型
  for (int i = 0; i < 8; i++)
  {
    checksum += buffer[(start_pos + 2 + i) % RX_BUFFER_SIZE]; // 累加数据部分
  }

  if (checksum != buffer[(start_pos + 10) % RX_BUFFER_SIZE])
  {
    //!printf("Checksum failed\r\n");
    return; // 校验失败
  }

  // 检查是否为角度数据
  if (buffer[(start_pos + 1) % RX_BUFFER_SIZE] == 0x53)
  {
    // 数据解析
    for (int i = 0; i < 3; i++)
    {
      uint8_t low = buffer[(start_pos + 2 + 2 * i) % RX_BUFFER_SIZE];
      uint8_t high = buffer[(start_pos + 2 + 2 * i + 1) % RX_BUFFER_SIZE];
      data[i] = (int16_t)((int8_t)high << 8 | low);
    }

    // 转换为浮点角度并打印
    float roll = (float)data[0] / 32768.0f * 180;  // 横滚
    float pitch = (float)data[1] / 32768.0f * 180; // 俯仰
    float yaw = (float)data[2] / 32768.0f * 180;   // 航向

    printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", roll, pitch, yaw);
    gyro_data_ready = 1; // 设置数据处理完成标志
  }
}

// 在DMA缓冲区中查找陀螺仪数据包
void Process_DMA_Buffer(void)
{
  uint16_t current_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);

  // 检查是否有新数据
  while (rx_read_pos != current_pos)
  {
    // 查找包头0x55
    if (rx_buffer[rx_read_pos] == 0x55)
    {
      // 检查是否有完整的11字节数据包
      uint16_t remaining = (current_pos >= rx_read_pos) ? (current_pos - rx_read_pos) : (RX_BUFFER_SIZE - rx_read_pos + current_pos);

      if (remaining >= 11)
      {
        Parse_Gyro_Data(rx_buffer, rx_read_pos);
        rx_read_pos = (rx_read_pos + 11) % RX_BUFFER_SIZE;
      }
      else
      {
        break; // 数据不完整，等待更多数据
      }
    }
    else
    {
      rx_read_pos = (rx_read_pos + 1) % RX_BUFFER_SIZE;
    }
  }
}

// UART DMA接收完成回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    Process_DMA_Buffer();
  }
}

// UART DMA接收半完成回调函数
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    Process_DMA_Buffer();
  }
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
