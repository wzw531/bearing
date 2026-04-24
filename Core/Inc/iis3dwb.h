#ifndef      __IIS3DWB_H__
#define	     __IIS3DWB_H__

#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "iis3dwb_reg.h"

#define IIS_PIN_CTRL    0x02   //0x7fSDO上拉 0x3fSDO断开
#define IIS_FIFO_CTRL1  0x07
#define IIS_FIFO_CTRL2  0x08
#define IIS_FIFO_CTRL3  0x09
#define IIS_FIFO_CTRL4  0x0a
#define IIS_COUNTER_BDR_REG1  0x0B
#define IIS_COUNTER_BDR_REG2  0x0c
#define IIS_INT1_CTRL   0x0d
#define IIS_INT2_CTRL   0x0e
#define IIS_ID        0x0f
#define IIS_CTRL1_XL  0x10
#define IIS_CTRL3_C   0x12
#define IIS_CTRL4_C   0x13
#define IIS_CTRL5_C   0x14
#define IIS_CTRL6_C   0x15
#define IIS_CTRL7_C   0x16
#define IIS_CTRL8_XL   0x17
#define IIS_CTRL10_C   0x19
#define IIS_ALL_INT_SRC    0x1a
#define IIS_WAKE_UP_SRC    0x1b
#define IIS_STATUS_REG     0x1e
#define IIS_OUT_TEMP_L    0x20
#define IIS_OUT_TEMP_H    0x21
#define IIS_OUTX_L_A      0x28
#define IIS_OUTX_H_A      0x29
#define IIS_OUTY_L_A      0x2A
#define IIS_OUTY_H_A      0x2B
#define IIS_OUTZ_L_A      0x2C
#define IIS_OUTZ_H_A      0x2D
#define IIS_FIFO_STATUS1   0x3a
#define IIS_FIFO_STATUS2   0x3b
#define IIS_TIMESTAMP0     0x40
#define IIS_TIMESTAMP1     0x41
#define IIS_TIMESTAMP2     0x42
#define IIS_TIMESTAMP3     0x43
#define IIS_SLOPE_EN       0x56
#define IIS_INTERRUPTS_EN  0x58
#define IIS_WAKE_UP_THS    0x5b
#define IIS_WAKE_UP_DUR    0x5c
#define IIS_MD1_CFG      0x5e
#define IIS_MD2_CFG      0x5f
#define IIS_INTERNAL_FREQ_FINE   0x63
#define IIS_X_OFS_USR    0x73
#define IIS_Y_OFS_USR    0x74
#define IIS_Z_OFS_USR    0x75
// ===== 硬件引脚配置 (根据您的实际连接修改) =====
// CS引脚配置 
#define IIS3DWBTR_CS_Pin         GPIO_PIN_4
#define IIS3DWBTR_CS_GPIO_Port   GPIOE

// INT1引脚配置 
#define IIS3DWBTR_INT1_Pin       GPIO_PIN_1
#define IIS3DWBTR_INT1_GPIO_Port GPIOE

// INT2引脚配置
#define IIS3DWBTR_INT2_Pin       GPIO_PIN_3
#define IIS3DWBTR_INT2_GPIO_Port GPIOE


extern volatile uint8_t IIS3DWBTR_INT1flag;
extern volatile uint8_t IIS3DWBTR_INT2flag;
extern stmdev_ctx_t dev_ctx;
// ===== 传感器数据结构定义 =====
typedef struct {
    // 基本信息 (与KX134兼容)
    uint8_t ID;                    // 设备ID
    uint8_t SelfCheckStatus;       // 自检状态
    uint8_t DRDY;                  // 数据就绪标志 (外部代码依赖此标志)
    
    // 数据缓冲区 (格式与KX134相同)
    int16_t RawData[3];           // 原始XYZ数据 (与KX134相同的格式)
    float Data[3];                 // 转换为g的XYZ数据 (外部代码使用这个)
    
    // 内部使用 (可以添加IIS3DWB特有字段)
    uint8_t buff[8];               // 原始SPI数据缓冲区
    uint32_t SampleCnt;            // 采样计数器
} IIS3DWB_TypeDef;

// ===== 全局变量声明 =====
extern IIS3DWB_TypeDef IIS3DWB;

// ===== 函数声明 =====
uint8_t SPI_IIS3DWB_READ(uint8_t address);
void SPI_IIS3DWB_WRITE(uint8_t address, uint8_t data);
void IIS3DWB_READ_ID(void);
void IIS3DWB_READ_TEMP(void);
void IIS3DWB_READ_ALL_DATA(void);
void IIS3DWB_Init(void);
void IIS3DWB_ReadData(void);
void iis3dwb_self_test(void);
void iis3dwb_fifo(void);
void IIS3DWBTR_ReadData(void);
 void IIS3DWB_GPIO_Init(void);
// ===== 中断处理函数声明 =====
void IIS3DWB_INT1_Handler(void);  // INT1中断处理
void IIS3DWB_INT2_Handler(void);  // INT2中断处理


#endif

