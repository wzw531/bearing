#ifndef      __ACCELEROMETER_H__
#define	     __ACCELEROMETER_H__

#include "main.h"


#define KX134_CS_Pin GPIO_PIN_4
#define KX134_CS_GPIO_Port GPIOE

#define KX134_INT_Pin GPIO_PIN_3
#define KX134_INT_GPIO_Port GPIOE

#define  MAN_ID  		0x00
#define  PART_ID  		0x01
#define  XADP_L  		0x02
#define  XADP_H  		0x03
#define  YADP_L  		0x04
#define  YADP_H  		0x05
#define  ZADP_L  		0x06
#define  ZADP_H  		0x07
#define  XOUT_L  		0x08
#define  XOUT_H  		0x09
#define  YOUT_L  		0x0A
#define  YOUT_H  		0x0B
#define  ZOUT_L  		0x0C
#define  ZOUT_H  		0x0D
#define  COTR  			0x12
#define  WHO_AM_I  		0x13
#define  TSCP  			0x14
#define  TSPP  			0x15
#define  INS1  			0x16
#define  INS2  			0x17
#define  INS3  			0x18
#define  STATUS_REG  	0x19
#define  INT_REL  		0x1A
#define  CNTL1  		0x1B
#define  CNTL2  		0x1C
#define  CNTL3  		0x1D
#define  CNTL4  		0x1E
#define  CNTL5  		0x1F
#define  CNTL6  		0x20
#define  ODCNTL  		0x21
#define  INC1  			0x22
#define  INC2  			0x23
#define  INC3  			0x24
#define  INC4  			0x25
#define  INC5  			0x26
#define  INC6  			0x27
#define  TILT_TIMER  	0x29
#define  TDTRC  		0x2A
#define  TDTC  			0x2B
#define  TTH  			0x2C
#define  TTL  			0x2D
#define  FTD  			0x2E
#define  STD  			0x2F
#define  TLT  			0x30
#define  TWS  			0x31
#define  FFTH  			0x32
#define  FFC  			0x33
#define  FFCNTL  		0x34
#define  TILT_ANGLE_LL  0x37
#define  TILT_ANGLE_HL  0x38
#define  HYST_SET  		0x39
#define  LP_CNTL1 		0x3A
#define  LP_CNTL2  		0x3B
#define  WUFTH  		0x49
#define  BTSWUFTH  		0x4A
#define  BTSTH  		0x4B
#define  BTSC  			0x4C
#define  WUFC  			0x4D
#define  SELF_TEST  	0x5D
#define  BUF_CNTL1  	0x5E
#define  BUF_CNTL2  	0x5F
#define  BUF_STATUS_1  	0x60
#define  BUF_STATUS_2  	0x61
#define  BUF_CLEAR  	0x62
#define  BUF_READ  		0x63
#define  ADP_CNTL1  	0x64
#define  ADP_CNTL2  	0x65
#define  ADP_CNTL3  	0x66
#define  ADP_CNTL4  	0x67
#define  ADP_CNTL5  	0x68
#define  ADP_CNTL6  	0x69
#define  ADP_CNTL7  	0x6A
#define  ADP_CNTL8  	0x6B
#define  ADP_CNTL9  	0x6C
#define  ADP_CNTL10  	0x6D
#define  ADP_CNTL11  	0x6E
#define  ADP_CNTL12  	0x6F
#define  ADP_CNTL13  	0x70
#define  ADP_CNTL14  	0x71
#define  ADP_CNTL15  	0x72
#define  ADP_CNTL16  	0x73
#define  ADP_CNTL17  	0x74
#define  ADP_CNTL18  	0x75
#define  ADP_CNTL19  	0x76
#define  INTERNAL_0X7F  0x7F

void KX134_Init(void);
void KX134_WriteReg(unsigned char RegAddr,uint8_t Buffer);
void KX134_ReadReg(unsigned char RegAddr,uint8_t *Buffer,uint8_t Length);
void KX134_Start(void);
void KX134_ReadData(void);
	

#endif


