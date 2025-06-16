#ifndef __OTHER_H
#define __OTHER_H

#include <stdio.h>
#include "stm32f4xx_hal.h"

extern int fputc(int ch, FILE *f);

// 陀螺仪数据处理相关函数声明
void Parse_Gyro_Data(uint8_t *buffer, uint16_t start_pos);
void Process_DMA_Buffer(void);

#endif // !__OTHER_H
