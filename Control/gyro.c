#include "gyro.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

// --- Private Defines ---
#define RX_BUFFER_SIZE 128

// --- Private Variables ---
static uint8_t rx_buffer[RX_BUFFER_SIZE]; // ���ջ�����
static uint16_t rx_read_pos = 0;          // ���ݶ�ȡλ��

static Gyro_Data_t gyro_data;       // �洢�����������������
static uint8_t gyro_data_ready = 0; // ����׼���ñ�־

// --- Private Function Prototypes ---
static void Parse_Gyro_Data(uint8_t *buffer, uint16_t start_pos);
static void Process_DMA_Buffer(void);

// --- Public Function Definitions ---

/**
 * @brief ��ʼ��������ģ��
 */
void Gyro_Init(void)
{
  // ����UART5��DMA����
  if (HAL_UART_Receive_DMA(&huart5, rx_buffer, RX_BUFFER_SIZE) != HAL_OK)
  {
    // �������Ҫ�����������ﴦ�����
    Error_Handler();
  }
  // ����UART5��IDLE�ж�
  __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);

  // �������ݽṹ��
  memset(&gyro_data, 0, sizeof(gyro_data));
}

/**
 * @brief ���������ݴ�����������Ӧ����ѭ���е���
 */
void Gyro_Process(void)
{
  Process_DMA_Buffer();
}

/**
 * @brief ��ȡ�����������������
 * @return Gyro_Data_t �����ǶȺͽ��ٶȵĽṹ��
 */
Gyro_Data_t Gyro_GetData(void)
{
  return gyro_data;
}

/**
 * @brief ����Ƿ����µ�����������
 * @return uint8_t 1:��������, 0:û��������
 */
uint8_t Gyro_IsDataReady(void)
{
  return gyro_data_ready;
}

/**
 * @brief �������׼���ñ�־
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
 * @brief �������������ݰ�
 */
static void Parse_Gyro_Data(uint8_t *buffer, uint16_t start_pos)
{
  uint8_t checksum;
  int16_t raw_data[3];

  // У�����֤
  checksum = 0x55 + buffer[(start_pos + 1) % RX_BUFFER_SIZE];
  for (int i = 0; i < 8; i++)
  {
    checksum += buffer[(start_pos + 2 + i) % RX_BUFFER_SIZE];
  }

  if (checksum != buffer[(start_pos + 10) % RX_BUFFER_SIZE])
  {
    return; // У��ʧ��
  }

  // ����ԭʼ����
  for (int i = 0; i < 3; i++)
  {
    uint8_t low = buffer[(start_pos + 2 + 2 * i) % RX_BUFFER_SIZE];
    uint8_t high = buffer[(start_pos + 2 + 2 * i + 1) % RX_BUFFER_SIZE];
    raw_data[i] = (int16_t)((int8_t)high << 8 | low);
  }

  // ������������ (0x53: �Ƕ�, 0x52: ���ٶ�) ��������
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
 * @brief ��DMA�������в��Ҳ��������������ݰ�
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
        break; // ���ݲ�����
      }
    }
    else
    {
      rx_read_pos = (rx_read_pos + 1) % RX_BUFFER_SIZE;
    }
  }
}
