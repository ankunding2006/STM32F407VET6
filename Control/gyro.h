#ifndef __GYRO_H__
#define __GYRO_H__

#include "main.h"

/**
 * @brief  陀螺仪数据结构体，用于存放解析后的数据
 */
typedef struct
{
  float roll, pitch, yaw; // 角度
  float wx, wy, wz;       // 角速度
} Gyro_Data_t;

// --- Public Functions ---

void Gyro_Init(void);
void Gyro_Process(void);
Gyro_Data_t Gyro_GetData(void);
uint8_t Gyro_IsDataReady(void);
void Gyro_ClearDataReadyFlag(void);

// --- Callbacks to be called from main ---
void Gyro_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Gyro_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart);

#endif /* __GYRO_H__ */
