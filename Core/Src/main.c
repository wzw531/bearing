/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "ADS124S08.h"
#include "dsp.h"
#include "dma.h"
#include "tim.h"
#include "receivedata.h"
#include "accelerometer.h"
#include "iis3dwb.h"
#include "iis3dwb_reg.h"
#include "wdg.h"
#include "i2c.h"

void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
void Tdata_Process(void);
extern Bearing_Data_Struct Bearing_Data;

// Ensure visibility of buffer manager and UART state (declared in ADS124S08.c)
extern Double_Buffer_Manager buffer_manager;
extern volatile uint8_t uart_busy;
extern void start_buffer_transmit(uint8_t index);


//数据处理程序，二进制转换为浮点数
double data_temp;
double string1_temp;
double string2_temp;
double ADS124S08ConvValue(uint32_t bin)    
{
	if(bin==0x800000)
	{
		return -2.5;
	}
	else
	{
		int _val;
		double adcValue;
		_val = bin&0x800000 ? (-((~bin+1)&0x7fffff)) : bin;    
		//符号位为1是负数，求补码，若为0则原码是其本身
		adcValue = 2.5*_val/8388608.0;
		return adcValue;
	}  
}

extern uint8_t KX134_CNTL1;
double KX134ConvValue(uint16_t bin, uint8_t range)    
{	
	if(bin==0x8000)
	{
		if(!(range & 0x18))
			return -8;
		if((range & 0x18) == 0x08)
			return -16;
		if((range & 0x18) == 0x10)
			return -32;
		if((range & 0x18) == 0x18)
			return -64;			
	}
	else
	{
		int _val;
		_val = bin&0x8000 ? (-((~bin+1)&0x7fff)) : bin;    
		//符号位为1是负数，求补码，若为0则原码是其本身
		if(!(range & 0x18))
			return (double)_val*0.00024;
		if((range & 0x18) == 0x08)
			return (double)_val*0.00049;
		if((range & 0x18) == 0x10)
			return (double)_val*0.00098;
		if((range & 0x18) == 0x18)
			return (double)_val*0.00195;
	}  
	return 0;
}

/*//////////////////////////////////////////////////////////////////////////
方法四：递推平均滤波法（又称滑动平均滤波法）
方法： 把连续取得的N个采样值看成一个队列，队列的长度固定为N，
       每次采样到一个新数据放入队尾，并扔掉原来队首的一次数据（先进先出原则），
       把队列中的N个数据进行算术平均运算，获得新的滤波结果。
       N值的选取：流量，N=12；压力，N=4；液面，N=4-12；温度，N=1-4。
优点：对周期性干扰有良好的抑制作用，平滑度高；
      适用于高频振荡的系统。
缺点：灵敏度低，对偶然出现的脉冲性干扰的抑制作用较差；
      不易消除由于脉冲干扰所引起的采样值偏差；
      不适用于脉冲干扰比较严重的场合；
      比较浪费RAM。
*/

//#define FILTER4_N 3
//uint32_t Temp_dataArr[FILTER4_N+1]={0};
//uint16_t Temp_dataArrflag=0;
//uint32_t Tempdata_filter() 
//{
//	if(Temp_dataArr[0]==0) return Temp_dataArr[FILTER4_N];
//  int i;
//  uint32_t filter_sum = 0;
//  for(i = 0; i < FILTER4_N; i++) 
//	{
//    Temp_dataArr[i] = Temp_dataArr[i + 1]; // 所有数据左移，低位仍掉
//    filter_sum += Temp_dataArr[i];
//  }
////	printf("%d\n",filter_sum / FILTER4_N);
//  return (uint32_t)(filter_sum / FILTER4_N);
//}
/* USER CODE END 0 */
 
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  PeriphCommonClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_DMA_Init();	
  MX_USART1_UART_Init();
  //MX_USART2_UART_Init();
  //MX_USART3_UART_Init();
  MX_I2C2_Init(); // 新增：初始化I2C2外设
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM13_Init();
  iwdg_init(IWDG_PRESCALER_64, 500);//1s时间
  /**************************——采集——**********************************/             
  ADS124S08_Init();
  //ADS124S08_Start();	//初始化
	
  //__HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);//先清除定时器标志，否则开启定时器之后会立刻进入一次中断
  //HAL_TIM_Base_Start_IT (&htim6);//开启定时器，周期400ms
	
  IIS3DWB_GPIO_Init();
  //iis3dwb_self_test();
  iis3dwb_fifo();
  
  //__HAL_TIM_CLEAR_FLAG(&htim8,TIM_FLAG_UPDATE);
  //HAL_TIM_Base_Start_IT (&htim8);//开启定时器，周期0.2ms
  
  HAL_TIM_Base_Start_IT (&htim13);//开启定时器，周期0.833ms  	
  /***********************——片上计算——*********************************/ 
  //IIRfilter_test();
  //printf("IIR Result:\r\n");
	
  //FFT_test();
  //printf("FFT Result:\r\n");
  
  //FIRfilter_test();
  //printf("FIR Result:\r\n");
  
  //StatisticsFunctions_test();
  	 
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	 
		if(ADS124S08S_NVICflag==1)
		{
			ADS124S08S_NVICflag=0;
			ADS124S08_ReadData();
			//iwdg_feed();  
		}
		//printf("running:\r\n");
		if(IIS3DWBTR_INT1flag==1)
		{
			IIS3DWBTR_INT1flag=0;
			//iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_OFF);
			IIS3DWBTR_ReadData();			
		}
//		if(IIS3DWBTR_INT2flag==1)
//		{
//			IIS3DWBTR_INT2flag=0;		
//		}		 		
  }
  /* USER CODE END 3 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//中断回调函数
{	
	if(GPIO_Pin == DRDY_Pin){	
		ADS124S08S_NVICflag=1;
	}	
	else if(GPIO_Pin == IIS3DWBTR_INT1_Pin){
		__HAL_GPIO_EXTI_CLEAR_IT(IIS3DWBTR_INT1_Pin);
		//printf("INT1\r\n");
		IIS3DWBTR_INT1flag=1;
	}
	else if(GPIO_Pin == IIS3DWBTR_INT2_Pin){
		//__HAL_GPIO_EXTI_CLEAR_IT(IIS3DWBTR_INT2_Pin);
		//printf("INT2\r\n");
		IIS3DWBTR_INT2flag=1;
	}
}

uint16_t adcx_flag=0;             //振动数据存储标志
uint16_t adcx_NVICflag=0; 
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	//printf("ADC1转换完成中断\r\n");
    if(hadc->Instance==ADC1)
    {
		adcx_NVICflag=1;
		
    }
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance==htim6.Instance)
  {
    // Use buffer_manager to avoid DMA/CPU concurrent access to the same buffer
    uint8_t idx = buffer_manager.ready_index;
    // copy current packet into ready buffer
    memcpy(&buffer_manager.buffers[idx], &Bearing_Data, sizeof(Bearing_Data_Struct));
    buffer_manager.buffer_status[idx] = 1; // ready
    buffer_manager.pending_transmit = 1;
    if (!uart_busy) {
      // start immediate transmit
      // start_buffer_transmit will clean D-Cache and call HAL_UART_Transmit_DMA
      extern void start_buffer_transmit(uint8_t index);
      start_buffer_transmit(idx);
      // swap indices so CPU fills the alternate buffer next
      buffer_manager.ready_index = buffer_manager.active_index;
      buffer_manager.active_index = idx;
    } else {
      // advance ready_index to other buffer so next fill writes the alternate buffer
      buffer_manager.ready_index = (idx == 0) ? 1 : 0;
    }
    // Recover from stuck UART DMA: if uart_busy set too long, force clear and restart pending
    extern volatile uint8_t uart_busy;
    extern volatile uint32_t uart_tx_start_ms;
    if (uart_busy && uart_tx_start_ms != 0 && (HAL_GetTick() - uart_tx_start_ms) > 5000) {
      uart_busy = 0;
      uart_tx_start_ms = 0;
      // if there is a ready buffer, start it
      for (int i = 0; i < 2; i++) {
        if (buffer_manager.buffer_status[i] == 1) {
          extern void start_buffer_transmit(uint8_t index);
          start_buffer_transmit(i);
          break;
        }
      }
    }
  }

//	if(htim->Instance==htim8.Instance)
//	{
////		KX134_ReadReg(INS2,KX134_Buf,1); 
////		if(KX134_Buf[0] & 0x10){
////			KX134_Cnt_flag++;
////		    //printf("%d\r\n",KX134_Cnt_flag);
////			KX134_RD_flag=1;
////			if(KX134_Cnt_flag == 2560){
////				KX134_Cnt_flag = 0;
////			}
////		}						
////		printf("TIM8!!!\r\n");
//	}
	
	if(htim->Instance==htim13.Instance)
	{	
		//Stopdata++;
		//HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		ADS124S08_Start();
		//iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_26k7Hz);
		//HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
		//HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&Stopdata,sizeof(Stopdata));
		//printf("T13\r\n");
//		HAL_UART_Transmit_DMA(&huart1,(uint8_t *)&Bearing_Data,sizeof(Bearing_Data));
	}
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

