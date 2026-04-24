#include "gpio.h"
#include "spi.h"
#include "usart.h"
#include "accelerometer.h"
#include "stm32h7xx_hal.h"
#include "ADS124S08.h"
#include "IIS3DWB.h"
#include "string.h"

extern uint16_t Temp_flag;
extern Double_Buffer_Manager buffer_manager;
extern volatile uint8_t data_transmit_flag;
extern volatile uint8_t transmit_in_progress;
volatile uint8_t IIS3DWBTR_INT1flag = 0;
volatile uint8_t IIS3DWBTR_INT2flag = 0;

// ===== SPI通信句柄声明 (外部引用) =====
extern SPI_HandleTypeDef hspi4;  // 确保这与您的SPI实例名称一致
extern Bearing_Data_Struct Bearing_Data;

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME        10 //ms
#define    WAIT_TIME       100 //ms
#define    FIFO_WATERMARK    5

/* Self test limits. */
#define    MIN_ST_LIMIT_mg        800.0f
#define    MAX_ST_LIMIT_mg       3200.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

/* Private variables ---------------------------------------------------------*/
static int16_t data_raw_acceleration[3];
static int16_t data_raw_temperature;
static float_t acceleration_mg[3];
static float_t temperature_degC;
static uint8_t whoamI, rst;
static uint8_t tx_buffer[1000];
static iis3dwb_fifo_out_raw_t fifo_data[FIFO_WATERMARK];
static int16_t *datax;
static int16_t *datay;
static int16_t *dataz;
static int32_t *ts;
static int16_t x_data_sum_h;
static int16_t y_data_sum_h;
static int16_t z_data_sum_h;
static int16_t x_data_sum_l;
static int16_t y_data_sum_l;
static int16_t z_data_sum_l;
int16_t tempdata[3] = {0xaabb, 0, 0xddff};
uint8_t IIS3DWB_Data[9] = {0x6A,0xBD,0x00,0x00,0x00,0x00,0x00,0x00,0x7C};
uint16_t IISDWB_Cnt_flag = 0;
 void IIS3DWB_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    /* ===== 1. CS引脚配置（沿用KX134） ===== */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);  // 初始高电平
    
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;  // 完全沿用了！
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    /* ===== 2. INT1中断引脚配置 ===== */    
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // 下降沿触发
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 中断需要高速
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;  // 下降沿触发
    GPIO_InitStruct.Pull = GPIO_NOPULL;           // 上拉
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH; // 中断需要高速
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    /* ===== 3. 配置中断优先级 ===== */
    HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);
	
	HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}


/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

stmdev_ctx_t dev_ctx;
/* Main Example --------------------------------------------------------------*/
void iis3dwb_fifo(void)
{
  iis3dwb_fifo_status_t fifo_status;
  uint8_t rst;

  /* Uncomment to configure INT 1 */
  iis3dwb_pin_int1_route_t int1_route;
	memset(&int1_route, 0, sizeof(iis3dwb_pin_int1_route_t)); 
  int1_route.fifo_th = 1;
//  int1_route.drdy_xl = 1;
  /* Uncomment to configure INT 2 */
  //iis3dwb_pin_int2_route_t int2_route;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hspi4;

  /* Check device ID */
  iis3dwb_device_id_get(&dev_ctx, &whoamI);

//  if (whoamI != IIS3DWB_ID)
//    while (1);

  /* Restore default configuration */
  iis3dwb_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    iis3dwb_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  iis3dwb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set full scale */
  iis3dwb_xl_full_scale_set(&dev_ctx, IIS3DWB_16g);

  /*
   * Set FIFO watermark (number of unread sensor data TAG + 6 bytes
   * stored in FIFO) to FIFO_WATERMARK samples
   */
  iis3dwb_fifo_watermark_set(&dev_ctx, FIFO_WATERMARK);

  /* Set FIFO batch XL ODR to 12.5Hz */
  iis3dwb_fifo_xl_batch_set(&dev_ctx, IIS3DWB_XL_BATCHED_AT_26k7Hz);

  /* Set FIFO mode to Stream mode (aka Continuous Mode) */
  iis3dwb_fifo_mode_set(&dev_ctx, IIS3DWB_STREAM_MODE);

  /* Set Output Data Rate */
  iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_26k7Hz);

  iis3dwb_xl_filt_path_on_out_set(&dev_ctx, IIS3DWB_LP_ODR_DIV_10);
  iis3dwb_int_notification_set(&dev_ctx, IIS3DWB_INT_PULSED);
  iis3dwb_pin_int1_route_set(&dev_ctx, &int1_route);  
  iis3dwb_pin_mode_set(&dev_ctx, IIS3DWB_PUSH_PULL);
  iis3dwb_pin_polarity_set(&dev_ctx, IIS3DWB_ACTIVE_LOW);

  iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_26k7Hz);
  /* Wait samples */
//  while (1) {
//    
//	uint16_t num = 0, k;
//    /* Read watermark flag */
//    iis3dwb_fifo_status_get(&dev_ctx, &fifo_status);

//    if (fifo_status.fifo_th == 1) {
//      num = fifo_status.fifo_level;
//      snprintf((char *)tx_buffer, sizeof(tx_buffer), "-- FIFO num %d \r\n", num);
//      printf("%s",tx_buffer);

//      
//    }
//  }
}

uint16_t num = 0;
void IIS3DWBTR_ReadData(void)
{
	iis3dwb_fifo_status_t fifo_status;
    uint16_t fifo_level;
    uint16_t batch;
    uint16_t i;

    iis3dwb_fifo_status_get(&dev_ctx, &fifo_status);

    // ===== 溢出自动恢复 =====
    if(fifo_status.fifo_full || fifo_status.fifo_ovr)
    {
        iis3dwb_fifo_mode_set(&dev_ctx, IIS3DWB_BYPASS_MODE);
        HAL_Delay(1);
        iis3dwb_fifo_mode_set(&dev_ctx, IIS3DWB_STREAM_MODE);
        IISDWB_Cnt_flag = 0; // 清空计数
        return;
    }

    fifo_level = fifo_status.fifo_level;
    while (fifo_level > 0U)
    {
        batch = (fifo_level > FIFO_WATERMARK) ? FIFO_WATERMARK : fifo_level;
        iis3dwb_fifo_out_multi_raw_get(&dev_ctx, fifo_data, batch);

        for (i = 0; i < batch; i++)
        {
            Bearing_Data.adcx[IISDWB_Cnt_flag++] =
                (uint16_t)(((uint16_t)fifo_data[i].data[5] << 8) | fifo_data[i].data[4]);

            if (IISDWB_Cnt_flag >= 2132U) {
                IISDWB_Cnt_flag = 0;
            }
        }

        fifo_level -= batch;
    }
/*        平均值滤波     */	
//		for(int i = 0; i < num; i++){
//			f_data = &fifo_data[i];
//			x_data_sum_h += f_data->data[1];
//			x_data_sum_l += f_data->data[0];
//			y_data_sum_h += f_data->data[3];
//			y_data_sum_l += f_data->data[2];
//			z_data_sum_h += f_data->data[5];
//			z_data_sum_l += f_data->data[4];
//		}
//			IIS3DWB_Data[2] = x_data_sum_h / 5;
//			IIS3DWB_Data[3] = x_data_sum_l / 5;
//			IIS3DWB_Data[4] = y_data_sum_h / 5;
//			IIS3DWB_Data[5] = y_data_sum_l / 5;
//			IIS3DWB_Data[6] = z_data_sum_h / 5;
//			IIS3DWB_Data[7] = z_data_sum_l / 5;
//        
//		x_data_sum_h = 0;
//		x_data_sum_l = 0;
//		y_data_sum_h = 0;
//		y_data_sum_l = 0;
//		z_data_sum_h = 0;
//		z_data_sum_l = 0;
					
	//HAL_UART_Transmit_DMA(&huart1,IIS3DWB_Data,sizeof(IIS3DWB_Data));
	
//        datay = (int16_t *)&f_data->data[2];
//        dataz = (int16_t *)&f_data->data[4];
//        ts = (int32_t *)&f_data->data[0];

//        switch (f_data->tag >> 3) {
//        case IIS3DWB_XL_TAG:
//          snprintf((char *)tx_buffer, sizeof(tx_buffer), "%d: ACC [mg]:\t%4.2f\t%4.2f\t%4.2f\r\n",
//                  k,
//                  iis3dwb_from_fs8g_to_mg(*datax),
//                  iis3dwb_from_fs8g_to_mg(*datay),
//                  iis3dwb_from_fs8g_to_mg(*dataz));
//          printf("%s",tx_buffer);
//          break;
//        default:
//          break;
        
      //}

//      snprintf((char *)tx_buffer, sizeof(tx_buffer), "------ \r\n\r\n");
//      printf("%s",tx_buffer);
}

void iis3dwb_self_test(void)
{
  stmdev_ctx_t dev_ctx;

  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.mdelay = platform_delay;
  dev_ctx.handle = &hspi4;

  /* Check device ID */
  iis3dwb_device_id_get(&dev_ctx, &whoamI);
  printf("whoamI is :%x\r\n",whoamI);

  if (whoamI != IIS3DWB_ID)
	printf("iis3dwb_device_id_get failed\r\n");

  /* Restore default configuration */
  iis3dwb_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    iis3dwb_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  iis3dwb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);

  /* Set Output Data Rate */
  iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_26k7Hz);

  /* Set full scale */
  iis3dwb_xl_full_scale_set(&dev_ctx, IIS3DWB_16g);

  /* Configure filtering chain(No aux interface)
   * Accelerometer low pass filter path
   */
  iis3dwb_xl_filt_path_on_out_set(&dev_ctx, IIS3DWB_LP_ODR_DIV_10);

  /* Read samples in polling mode (no int) */
  while (1) {
    uint8_t reg;

    /* Read output only if new xl value is available */
    iis3dwb_xl_flag_data_ready_get(&dev_ctx, &reg);

    if (reg) {
      /* Read acceleration field data */
      memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
      iis3dwb_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
		IIS3DWB_Data[2]=data_raw_acceleration[0]>>8;
		IIS3DWB_Data[3]=data_raw_acceleration[0]&0xFF;
		IIS3DWB_Data[4]=data_raw_acceleration[1]>>8;
		IIS3DWB_Data[5]=data_raw_acceleration[1]&0xFF;
		IIS3DWB_Data[6]=data_raw_acceleration[2]>>8;
		IIS3DWB_Data[7]=data_raw_acceleration[2]&0xFF;
      //HAL_UART_Transmit_DMA(&huart1,IIS3DWB_Data,sizeof(IIS3DWB_Data));
    }
	
	for(int i = 1000; i > 0; i--){
		for(int i = 100; i > 0; i--){
        __NOP();//1000000/480 ns = 208us
		}
    }

//    iis3dwb_temp_flag_data_ready_get(&dev_ctx, &reg);

//    if (reg) {
//      /* Read temperature data */
//      memset(&data_raw_temperature, 0x00, sizeof(int16_t));
//      iis3dwb_temperature_raw_get(&dev_ctx, &data_raw_temperature);
//      temperature_degC = iis3dwb_from_lsb_to_celsius(data_raw_temperature);
//      snprintf((char *)tx_buffer, sizeof(tx_buffer),
//              "Temperature [degC]:%6.2f\r\n", temperature_degC);
//      printf("%s",tx_buffer);
//    }
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len)
{
  HAL_GPIO_WritePin(IIS3DWBTR_CS_GPIO_Port, IIS3DWBTR_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(IIS3DWBTR_CS_GPIO_Port, IIS3DWBTR_CS_Pin, GPIO_PIN_SET);
  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
	 reg |= 0x80;
  HAL_GPIO_WritePin(IIS3DWBTR_CS_GPIO_Port, IIS3DWBTR_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(handle, &reg, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(handle, bufp, len, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(IIS3DWBTR_CS_GPIO_Port, IIS3DWBTR_CS_Pin, GPIO_PIN_SET);
  return 0;
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
  HAL_Delay(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
	IIS3DWB_GPIO_Init();
}
