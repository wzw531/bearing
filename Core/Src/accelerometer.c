#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "accelerometer.h"
#include "stm32h7xx_hal.h"
#include "ADS124S08.h"

uint8_t KX134_testBuf[10]={0};
uint8_t KX134_CNTL1=0xaa;

uint8_t KX134_Buf[10]={0xcc};
uint8_t KX134_XOUT[2]={0}; 
uint8_t KX134_YOUT[2]={0}; 
uint8_t KX134_ZOUT[2]={0}; 
uint16_t KX134_xOUT=0; 
uint16_t KX134_yOUT=0; 
uint16_t KX134_zOUT=0;
float KX134_ConvValue[3];
uint8_t KX134_RD_flag=0;
uint16_t KX134_Cnt_flag=0;
extern Bearing_Data_Struct Bearing_Data;
uint8_t KX_start1[4]={0x99,0x77};
uint8_t KX_end1[4]={0x6A,0xBD};
uint8_t KX134_Data[9]={0x6A,0xBD,0x00,0x00,0x00,0x00,0x00,0x00,0x7C};

void KX134_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KX134_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(KX134_CS_GPIO_Port, &GPIO_InitStruct);
}


void KX134_WriteReg(unsigned char RegAddr,uint8_t Buffer) 
{
	RegAddr &= 0x7F;
	HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi4,&RegAddr,1,1000);
	HAL_SPI_Transmit(&hspi4,&Buffer,1,1000);
//	HAL_Delay(1);
	HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);
	
}

void KX134_ReadReg(unsigned char RegAddr,uint8_t *Buffer,uint8_t Length) 
{
	RegAddr |= 0x80;
	HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(1);
	HAL_SPI_Transmit(&hspi4,&RegAddr,1,1000);
	HAL_SPI_Receive(&hspi4,Buffer,Length,1000);
//	HAL_Delay(1);
	HAL_GPIO_WritePin(KX134_CS_GPIO_Port, KX134_CS_Pin, GPIO_PIN_SET);
	
}

/*
	0x06  50Hz
	0x07  100Hz
	0x08  200Hz
	0x09  400Hz
	0x0A  800Hz
	0x0B  1600Hz
	0x0C  3200Hz
	0x0D  6400Hz
	0x0E  12800Hz
	0x0F  25600Hz
*/
void KX134_Init(void) 
{
	KX134_GPIO_Init();
	HAL_Delay(20);
	KX134_WriteReg(CNTL1,0x00);
	HAL_Delay(1);
	KX134_WriteReg(INC1,0x30);
	HAL_Delay(1);
	KX134_WriteReg(INC4,0x10);   
//	KX134_WriteReg(CNTL2,0x40);
	HAL_Delay(1);
	KX134_WriteReg(ODCNTL ,0x09);  //0x0A = 800Hz
	HAL_Delay(1);
//	KX134_WriteReg(CNTL1 ,0xE0);
	
	
//	KX134_ReadReg(ODCNTL,&KX134_CNTL1,1); 
//	HAL_UART_Transmit(&huart1,KX134_testBuf,1,1000);
	
}


void KX134_Start(void) 
{
	KX134_WriteReg(CNTL1 ,0xE0);
}


void KX134_ReadData(void)
{
	  KX134_ReadReg(INS2,KX134_Buf,1); 
	  //HAL_UART_Transmit(&huart1,KX134_Buf,1,1000);
	  if(KX134_Buf[0] & 0x10)
	  {
		  KX134_RD_flag = 0;
		  
		  KX134_ReadReg(XOUT_L,KX134_Buf,6);
//		  HAL_UART_Transmit(&huart1,KX134_Buf,6,1000);
//		  KX134_XOUT[0]=KX134_Buf[1];
//		  KX134_XOUT[1]=KX134_Buf[0];
//		  KX134_YOUT[0]=KX134_Buf[3];
//		  KX134_YOUT[1]=KX134_Buf[2];
//		  KX134_ZOUT[0]=KX134_Buf[5];
//		  KX134_ZOUT[1]=KX134_Buf[4];
		  KX134_Data[2]=KX134_Buf[1];
		  KX134_Data[3]=KX134_Buf[0];
		  KX134_Data[4]=KX134_Buf[3];
		  KX134_Data[5]=KX134_Buf[2];
		  KX134_Data[6]=KX134_Buf[5];
		  KX134_Data[7]=KX134_Buf[4];
//		  KX134_xOUT=(KX134_Buf[1]<<8)|KX134_Buf[0];
//		  KX134_yOUT=(KX134_Buf[3]<<8)|KX134_Buf[2];
//		  KX134_zOUT=(KX134_Buf[5]<<8)|KX134_Buf[4];
		  //KX134_Buf[0]=0;
//		  KX134_ConvValue[0]=KX134ConvValue(KX134_xOUT,KX134_CNTL1);
//		  KX134_ConvValue[1]=KX134ConvValue(KX134_yOUT,KX134_CNTL1);
//		  KX134_ConvValue[2]=KX134ConvValue(KX134_zOUT,KX134_CNTL1);
		  Bearing_Data.adcx[KX134_Cnt_flag]=KX134_zOUT;
		  KX134_Cnt_flag++;
		//  printf("%d\r\n",KX134_Cnt_flag);
		  if(KX134_Cnt_flag == 2560){
			  KX134_Cnt_flag = 0;
		  }
		  HAL_UART_Transmit_DMA(&huart1,KX134_Data,sizeof(KX134_Data));
//		  printf("XOUT:%fg\r\n",KX134_ConvValue[0]);
//		  printf("YOUT:%fg\r\n",KX134_ConvValue[1]);
//		  printf("ZOUT:%fg\r\n",KX134_ConvValue[2]);
//		  HAL_UART_Transmit(&huart1,&KX134_XOUT,2,1000);
//		  HAL_UART_Transmit(&huart1,&KX134_YOUT,2,1000);
		  
		 // HAL_UART_Transmit(&huart1,KX_start1,2,1000);
		  //HAL_UART_Transmit_DMA(&huart1,&KX134_ZOUT[0],2);
		 // HAL_UART_Transmit(&huart1,KX_end1,2,1000);
		  
	  }	  
	
}
