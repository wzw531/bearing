#ifndef      __ADS124S08_H__
#define	     __ADS124S08_H__


#include "main.h"

#define CS_Pin GPIO_PIN_14
#define CS_GPIO_Port GPIOG
#define DRDY_Pin GPIO_PIN_13
#define DRDY_GPIO_Port GPIOG
#define DRDY_EXTI_IRQn EXTI15_10_IRQn
#define RESET_Pin GPIO_PIN_7
#define RESET_GPIO_Port GPIOD
#define START_Pin GPIO_PIN_12
#define START_GPIO_Port GPIOC

extern uint8_t ADS124S08S_NVICflag;
#pragma pack(push, 1)
typedef struct
{
	uint8_t header[12];//包头
	uint16_t adcx[2132];//振动数据存储
	int8_t ADS124S08Str1_Data[480];//应变1数据存储
	int8_t ADS124S08Str2_Data[480];//应变2数据存储
	int8_t ADS124S08Str3_Data[480];//应变3数据存储
	int8_t ADS124S08Temp_Data[13];//温度数据存储
	uint8_t pri;//校验码
	uint8_t end[2];//包尾
}Bearing_Data_Struct;
#pragma pack(pop)
// 修改双缓冲管理器，添加发送相关状态
typedef struct {
    Bearing_Data_Struct buffers[2];           //这里嵌套了Bearing_Data_Struct结构体，两个缓冲区
    volatile uint8_t active_index;            // 当前采集的缓冲区索引
    volatile uint8_t ready_index;             // 准备发送的缓冲区索引  
    volatile uint8_t buffer_status[2];        // 0=空闲, 1=采集完成, 2=发送中
    volatile uint8_t pending_transmit;        // 用于表示有缓冲区采集已完成
} Double_Buffer_Manager;

void ADS124S08_Init(void);
void ADS124S08_Reset(void);
void ADS124S08_Start(void);
void ADS124S08_GPIO_Init(void);
void ADS124S08REG_Init(void);
void ADS124S08INPMUX_Cov(uint16_t INPMUX_Flag);
void ADS124S08_RREG(uint8_t *ADS124S08RREG);
void ADS124S08_Commands(void);
void ADS124S08_Stop_it(void);
void ADS124S08_ReadData(void);
void Tdata_Process(void);

extern uint8_t INPMUX_Commands0[3];
extern uint8_t INPMUX_Commands1[3];
extern uint8_t INPMUX_Commands2[3];  //AIN5  AINCOM  应变
extern uint8_t INPMUX_Commands3[3];  //AIN10 AIN11  应变
/*************************************************************************/
//ADS124S08有关定义
//ADS124S08命令码列表 
#define ADC_CMD_NOP         0x00            /*!< 空操作   */ 
#define ADC_CMD_WAKEUP      0x02            /*!< 退出睡眠模式   */
#define ADC_CMD_POWERDN     0x04            /*掉电*/
#define ADC_CMD_RESET       0x06            /*!< 芯片复位   */  
#define ADC_CMD_START       0x08            /*开始转换*/
#define ADC_CMD_STOP        0x0A            /*停止转换*/
//校准命令
#define ADC_CMD_SYSOCAL     0x16            /*!< 系统偏移校准   */  
#define ADC_CMD_SYSGCAL     0x17            /*!< 系统增益校准   */  
#define ADC_CMD_SELFOCAL    0x19            /*!< 系统自校准   */ 
//数据读命令
#define ADC_CMD_RDATA       0x12            /*!< 单次读取数据   */  
//寄存器读写命令  
#define ADC_CMD_RREG        0x20            /*!< 读寄存器   */  
#define ADC_CMD_WREG        0x40            /*!< 写寄存器   */  
 

	 
//ADS124S08寄存器列表
#define ADC_REG_ICID        0x00
#define ADC_REG_STATUS      0x01
#define ADC_REG_INPMUX      0x02
#define ADC_REG_PGA         0x03
#define ADC_REG_DATARATE    0x04
#define ADC_REG_REF         0x05
#define ADC_REG_IDACMAG     0x06    
#define ADC_REG_IDACMUX     0x07  
#define ADC_REG_VBIAS       0x08  
#define ADC_REG_SYS         0x09  
#define ADC_REG_OFC0        0x0A
#define ADC_REG_OFC1        0x0B
#define ADC_REG_OFC2        0x0C
#define ADC_REG_FSC0        0x0D  
#define ADC_REG_FSC1        0x0E  
#define ADC_REG_FSC2        0x0F 
#define ADC_REG_GPIODAT     0x10
#define ADC_REG_GPIOCON     0x11



//ADS124S08支持的增益列表  
#define ADC_GAIN_1          0x00  
#define ADC_GAIN_2          0x01  
#define ADC_GAIN_4          0x02  
#define ADC_GAIN_8          0x03  
#define ADC_GAIN_16         0x04  
#define ADC_GAIN_32         0x05  
#define ADC_GAIN_64         0x06  
#define ADC_GAIN_128        0x07 
#define ADC_GAIN_EN         0x08

//ADS124S08支持的转换速率列表  
#define ADC_SPS_5           0x01  
#define ADC_SPS_10          0x02  
#define ADC_SPS_16_6        0x03 
#define ADC_SPS_20          0x04
#define ADC_SPS_50          0x05 
#define ADC_SPS_60          0x06 
#define ADC_SPS_100         0x07 
#define ADC_SPS_100         0x07  
#define ADC_SPS_400         0x09  
#define ADC_SPS_1000        0x0b  
#define ADC_SPS_2000        0x0c  
#define ADC_SPS_4000        0x0d 

#define POSI_AIN(x)    (x<<4)
#define NEGA_AIN(x)    (x)

//ADS124S08转换模式
#define ADC_MODE_SINGLECOV      0x00        //单次转换  
#define ADC_MODE_CONTINUOUS     0x01        //连续转换 





//定义引脚有关宏函数
#define AD_nCS_GPIO_Port    GPIOA
#define AD_nCS_Pin          GPIO_PIN_4  
#define AD_START_GPIO_Port  GPIOC
#define AD_START_Pin        GPIO_PIN_5
#define nAD_DRDY_GPIO_Port  GPIOC
#define nAD_DRDY_Pin        GPIO_PIN_4
#define   AD_nCS_LOW	HAL_GPIO_WritePin(AD_nCS_GPIO_Port,AD_nCS_Pin, GPIO_PIN_RESET)
#define   AD_nCS_HIGH	HAL_GPIO_WritePin(AD_nCS_GPIO_Port,AD_nCS_Pin, GPIO_PIN_SET)

#define   AD_START_LOW	HAL_GPIO_WritePin(AD_START_GPIO_Port,AD_START_Pin,GPIO_PIN_RESET)
#define   AD_START_HIGH	HAL_GPIO_WritePin(AD_START_GPIO_Port,AD_START_Pin,GPIO_PIN_SET)

#define   nAD_DRDY_STATE  HAL_GPIO_ReadPin( nAD_DRDY_GPIO_Port,nAD_DRDY_Pin ) 


//引进外部变量（SPI1） 
extern SPI_HandleTypeDef hspi1;
extern int32_t ADS124S08Buff;




//函数定义，供主函数调用
//读取ADS124S08中的转换数据
int32_t ADS124S08_Read(void);



//ADS124S08初始化
//void ADS124S08_Init(void);

void ADS124S08_WriteReg(unsigned char RegAddr,unsigned char *Buffer,unsigned char Length);
////读寄存器
void ADS124S08_ReadReg(unsigned char RegAddr,unsigned char *Buffer,unsigned char Length);


//复位ADS124S08
//void ADS124S08_Chg_Channel(unsigned char channel);
float ADS124S08_GetADC_Vol(void);

unsigned char ADS124S08_WaitBusy(unsigned int Timeout);
#endif
