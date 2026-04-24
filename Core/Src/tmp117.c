#include "stm32h7xx_hal.h"
#include "tmp117.h"
#include "i2c.h"

extern I2C_HandleTypeDef hi2c2;  // 使用外部定义的I2C句柄

/**********************************************************************
温度传感器初始化
**********************************************************************/
void TMP117_Init(void)
{
  // 不需要再次初始化I2C，主程序应该已经初始化
  // 配置TMP117：转换64次取平均值
  TMP117_Write(TMP117_CONFIG_REG, 0x0060);
}

/**********************************************************************
TMP117 写数据
reg: 寄存器地址
data: 16位数据
返回值: HAL状态
**********************************************************************/
HAL_StatusTypeDef TMP117_Write(uint8_t reg, uint16_t data)
{
  uint8_t buffer[3];
  buffer[0] = reg;
  buffer[1] = (data >> 8) & 0xFF;  // 高字节
  buffer[2] = data & 0xFF;         // 低字节
  
  return HAL_I2C_Master_Transmit(&hi2c2, TMP117_ADDR << 1, buffer, 3, HAL_MAX_DELAY);
}

/**********************************************************************
TMP117 读数据
reg: 寄存器地址
data: 指向存储读取数据的指针
返回值: HAL状态
**********************************************************************/
HAL_StatusTypeDef TMP117_Read(uint8_t reg, uint16_t *data)
{
  uint8_t buffer[2];
  HAL_StatusTypeDef status;
  
  // 先发送要读取的寄存器地址
  status = HAL_I2C_Master_Transmit(&hi2c2, TMP117_ADDR << 1, &reg, 1, HAL_MAX_DELAY);
  if (status != HAL_OK) {
    return status;
  }
  
  // 然后读取数据
  status = HAL_I2C_Master_Receive(&hi2c2, TMP117_ADDR << 1, buffer, 2, HAL_MAX_DELAY);
  if (status == HAL_OK) {
    *data = (buffer[0] << 8) | buffer[1];
  }
  
  return status;
}

/**********************************************************************
获取TMP117温度值
返回值: 温度值(摄氏度)
**********************************************************************/
int16_t TMP117_Get_Temp(void)
{
  int16_t value;
  float temp;
  
  if (TMP117_Read(TMP117_TEMP_REG, &value) != HAL_OK) {
    return -999.0f; // 错误值
  }
  
  // 处理负数温度
  if (value & 0x8000) {
    // 负温度，进行二进制补码转换
    value = ~value + 1;
    temp = -((float)value * 0.0078125f);
  } else {
    temp = (float)value * 0.0078125f;
  }
  
  return value;
}