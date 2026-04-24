#include "receivedata.h"
#include "usart.h"
uint8_t ReceiveBuffer[2266]={0};
uint8_t Receiveflag=0;

void Rdata_Process(void)
{
	if(Receiveflag==1)
	{
		//HAL_UART_Transmit(&huart1,(uint8_t *)ReceiveBuffer,2266,1000);	
		Receiveflag=0;

		printf("头描述符为：%x %x %x\r\n",ReceiveBuffer[0],ReceiveBuffer[1],ReceiveBuffer[2]);
		printf("版本号为：%x\r\n",ReceiveBuffer[3]);
		printf("长度为：%x %x %x %x\r\n",ReceiveBuffer[4],ReceiveBuffer[5],ReceiveBuffer[6],ReceiveBuffer[7]);
		printf("设备类型为：%x\r\n",ReceiveBuffer[8]);
		printf("校验和为：%x %x\r\n",ReceiveBuffer[9],ReceiveBuffer[10]);
		printf("震动数据为：\r\n");
		for(int i=15;i<2015;i++)  printf("%x ",ReceiveBuffer[i]);
		printf("\r\n");
		printf("应变数据为：\r\n");
		for(int i=2019;i<2259;i++) 	printf("%x ",ReceiveBuffer[i]);
		printf("\r\n");
		printf("温度数据为：%x\r\n",ReceiveBuffer[2265]);
	}
}




