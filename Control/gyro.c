#include "gyro.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

// --- Private Defines ---
#define RX_BUFFER_SIZE 128

// --- Private Variables ---
static uint8_t rx_buffer[RX_BUFFER_SIZE]; // 接收缓冲区
static uint16_t rx_read_pos = 0;          // 数据读取位置

static Gyro_Data_t gyro_data;       // 存储解析后的陀螺仪数据
static uint8_t gyro_data_ready = 0; // 数据准备好标志

// --- Private Function Prototypes ---
static void Parse_Gyro_Data(uint8_t *buffer, uint16_t start_pos);
static void Process_DMA_Buffer(void);

// --- Public Function Definitions ---

/**
 * @brief 初始化陀螺仪模块
 */
void Gyro_Init(void)
{
  // 启动UART5的DMA接收
  if (HAL_UART_Receive_DMA(&huart5, rx_buffer, RX_BUFFER_SIZE) != HAL_OK)
  {
    // 如果你需要，可以在这里处理错误
    Error_Handler();
  }
  // 启用UART5的IDLE中断
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);

  // 清零数据结构体
  memset(&gyro_data, 0, sizeof(gyro_data));
}

/**
 * @brief 陀螺仪数据处理主函数，应在主循环中调用
 */
void Gyro_Process(void)
{
  Process_DMA_Buffer();
}

/**
 * @brief 获取解析后的陀螺仪数据
 * @return Gyro_Data_t 包含角度和角速度的结构体
 */
Gyro_Data_t Gyro_GetData(void)
{
  return gyro_data;
}

/**
 * @brief 检查是否有新的陀螺仪数据
 * @return uint8_t 1:有新数据, 0:没有新数据
 */
uint8_t Gyro_IsDataReady(void)
{
  return gyro_data_ready;
}

/**
 * @brief 清除数据准备好标志
 */
void Gyro_ClearDataReadyFlag(void)
{
  gyro_data_ready = 0;
}

// --- Callback Implementations ---

void Gyro_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    Process_DMA_Buffer();
  }
}

void Gyro_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    Process_DMA_Buffer();
  }
}

// --- Private Function Definitions ---

/**
 * @brief 解析陀螺仪数据包
 */
static void Parse_Gyro_Data(uint8_t *buffer, uint16_t start_pos)
{
  uint8_t checksum;
  int16_t raw_data[3];

  // 校验和验证
  checksum = 0x55 + buffer[(start_pos + 1) % RX_BUFFER_SIZE];
  for (int i = 0; i < 8; i++)
  {
    checksum += buffer[(start_pos + 2 + i) % RX_BUFFER_SIZE];
  }

  if (checksum != buffer[(start_pos + 10) % RX_BUFFER_SIZE])
  {
    return; // 校验失败
  }

  // 解析原始数据
  for (int i = 0; i < 3; i++)
  {
    uint8_t low = buffer[(start_pos + 2 + 2 * i) % RX_BUFFER_SIZE];
    uint8_t high = buffer[(start_pos + 2 + 2 * i + 1) % RX_BUFFER_SIZE];
    raw_data[i] = (int16_t)((int8_t)high << 8 | low);
  }

  // 根据数据类型 (0x53: 角度, 0x52: 角速度) 处理数据
  if (buffer[(start_pos + 1) % RX_BUFFER_SIZE] == 0x53)
  {
    gyro_data.roll = (float)raw_data[0] / 32768.0f * 180.0f;
    gyro_data.pitch = (float)raw_data[1] / 32768.0f * 180.0f;
    gyro_data.yaw = (float)raw_data[2] / 32768.0f * 180.0f;
    printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\r\n", gyro_data.roll, gyro_data.pitch, gyro_data.yaw);
    gyro_data_ready = 1;
  }
  else if (buffer[(start_pos + 1) % RX_BUFFER_SIZE] == 0x52)
  {
    gyro_data.wx = (float)raw_data[0] / 32768.0f * 2000.0f;
    gyro_data.wy = (float)raw_data[1] / 32768.0f * 2000.0f;
    gyro_data.wz = (float)raw_data[2] / 32768.0f * 2000.0f;
    printf("Wx: %.2f, Wy: %.2f, Wz: %.2f\r\n", gyro_data.wx, gyro_data.wy, gyro_data.wz);
    gyro_data_ready = 1;
  }
}

/**
 * @brief 在DMA缓冲区中查找并处理陀螺仪数据包
 */
static void Process_DMA_Buffer(void)
{
  uint16_t current_pos = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);

  while (rx_read_pos != current_pos)
  {
    if (rx_buffer[rx_read_pos] == 0x55)
    {
      uint16_t remaining = (current_pos >= rx_read_pos) ? (current_pos - rx_read_pos) : (RX_BUFFER_SIZE - rx_read_pos + current_pos);
      if (remaining >= 11)
      {
        Parse_Gyro_Data(rx_buffer, rx_read_pos);
        rx_read_pos = (rx_read_pos + 11) % RX_BUFFER_SIZE;
      }
      else
      {
        break; // 数据不完整
      }
    }
    else
    {
      rx_read_pos = (rx_read_pos + 1) % RX_BUFFER_SIZE;
    }
  }
}
