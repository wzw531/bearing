#ifndef __TMP117_H_
#define __TMP117_H_

#include "main.h"  // 添加HAL库支持

#define TMP117_ADDR   0x48    //A0GND
//#define TMP117_ADDR   0x49    //A0V+
//#define TMP117_ADDR   0x4A    //A0SDA
//#define TMP117_ADDR   0x4B    //A0SCL

#define TMP117_TEMP_REG     0x00  //温度寄存器
#define TMP117_CONFIG_REG   0x01  //配置寄存器
#define TMP117_TLOW_REG     0x02  //温度上限寄存器
#define TMP117_THIGH_REG    0x03  //温度下限寄存器

void TMP117_Init(void);
HAL_StatusTypeDef TMP117_Write(uint8_t reg, uint16_t data);
HAL_StatusTypeDef TMP117_Read(uint8_t reg, uint16_t *data);
int16_t TMP117_Get_Temp(void);

#endif