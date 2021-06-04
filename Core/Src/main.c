/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250_I2C.h"
#include "bme280.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern I2C_HandleTypeDef hi2c1;
int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev);
int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev);
void user_delay_ms(uint32_t ms);
uint8_t humidity;
struct bme280_dev bme280;
struct bme280_data comp_data;
#define atmPress 101325
double pressure, altitude,related_altitude;
double ref_altitude;
uint8_t sendaltitude = 0;
uint8_t spi_flag = 0;
double temp_send_y_axis_acc = 0;
uint8_t send_y_axis = 0;
uint8_t sendbuff[3] = {0};
float attempt = 501;
uint8_t breakout = 0;
float i;
/* USER CODE END P TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
double ay = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

/* Definitions for Pressure_Unit */
osThreadId_t Pressure_UnitHandle;
const osThreadAttr_t Pressure_Unit_attributes = {
  .name = "Pressure_Unit",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Inertial_Unit */
osThreadId_t Inertial_UnitHandle;
const osThreadAttr_t Inertial_Unit_attributes = {
  .name = "Inertial_Unit",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Decision_Log */
osThreadId_t Decision_LogHandle;
const osThreadAttr_t Decision_Log_attributes = {
  .name = "Decision_Log",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI2_Init(void);
void Pressure_Unit_Start(void *argument);
void Inertial_Unit_Start(void *argument);
void Decision_and_Logger(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */


  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Pressure_Unit */
  Pressure_UnitHandle = osThreadNew(Pressure_Unit_Start, NULL, &Pressure_Unit_attributes);

  /* creation of Inertial_Unit */
  Inertial_UnitHandle = osThreadNew(Inertial_Unit_Start, NULL, &Inertial_Unit_attributes);

  /* creation of Decision_Log */
  Decision_LogHandle = osThreadNew(Decision_and_Logger, NULL, &Decision_Log_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */



void my_print_readings(void)
{
int8_t rslt = BME280_OK;
bme280.dev_id = BME280_I2C_ADDR_SEC;
bme280.intf = BME280_I2C_INTF;
bme280.read = (void *)user_i2c_read;
bme280.write = (void *)user_i2c_write;
bme280.delay_ms = (void *)user_delay_ms;
rslt = bme280_init(&bme280);
printf("init result %d\r\n", rslt);

printf("starting readings.\r\n");
stream_sensor_data_forced_mode(&bme280);
}


void print_sensor_data(struct bme280_data *comp_data)
{
  #ifdef BME280_FLOAT_ENABLE

  #else

  #endif
}

int8_t stream_sensor_data_forced_mode(struct bme280_dev *dev)
{
int8_t rslt;
uint8_t settings_sel;


dev->settings.osr_h = BME280_OVERSAMPLING_2X;
dev->settings.osr_p = BME280_OVERSAMPLING_4X;
dev->settings.osr_t = BME280_OVERSAMPLING_4X;
dev->settings.filter = BME280_FILTER_COEFF_2;

settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;

rslt = bme280_set_sensor_settings(settings_sel, dev);


/* Continuously stream sensor data */

  rslt = bme280_set_sensor_mode(BME280_FORCED_MODE, dev);
  /* Wait for the measurement to complete and print data @25Hz */
  dev->delay_ms(30);
  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
  print_sensor_data(&comp_data);
  pressure = comp_data.pressure;
  altitude = (44330*(1-(pow((pressure/(float)atmPress), 0.19029495718))));
  dev->delay_ms(30);
  return rslt;
}

int8_t stream_sensor_data_normal_mode(struct bme280_dev *dev)
{
int8_t rslt;
uint8_t settings_sel;
struct bme280_data comp_data;

/* Recommended mode of operation: Indoor navigation */
dev->settings.osr_h = BME280_OVERSAMPLING_1X;
dev->settings.osr_p = BME280_OVERSAMPLING_1X;
dev->settings.osr_t = BME280_OVERSAMPLING_1X;
dev->settings.filter = BME280_FILTER_COEFF_16;
dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

settings_sel = BME280_OSR_PRESS_SEL;
settings_sel |= BME280_OSR_TEMP_SEL;
settings_sel |= BME280_OSR_HUM_SEL;
settings_sel |= BME280_STANDBY_SEL;
settings_sel |= BME280_FILTER_SEL;
rslt = bme280_set_sensor_settings(settings_sel, dev);
rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);


while (1) {
  /* Delay while the sensor completes a measurement */
  dev->delay_ms(140);
  rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);
  print_sensor_data(&comp_data);
}

return rslt;
}

void user_delay_ms(uint32_t ms)
{
HAL_Delay(ms);
}

int8_t user_i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
HAL_StatusTypeDef status = HAL_OK;

status = HAL_I2C_Mem_Write(&hi2c2, dev_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 0xffff);

if (status == HAL_OK)
  return(0);
else
  return(-1);
}

int8_t user_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
HAL_StatusTypeDef status = HAL_OK;

status = HAL_I2C_Mem_Read(&hi2c2, dev_addr<<1, reg_addr, I2C_MEMADD_SIZE_8BIT, reg_data, cnt, 0xffff);

if (status == HAL_OK)
  return(0);
else
  return(-1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_Pressure_Unit_Start */
/**
  * @brief  Function implementing the Pressure_Unit thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Pressure_Unit_Start */
void Pressure_Unit_Start(void *argument)
{
  my_print_readings();
  ref_altitude = altitude;
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	my_print_readings();
	related_altitude = altitude - ref_altitude;
	sendaltitude = related_altitude/4;
	sendbuff[1] = sendaltitude;
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Inertial_Unit_Start */
/**
* @brief Function implementing the Inertial_Unit thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Inertial_Unit_Start */
void Inertial_Unit_Start(void *argument)
{
  /* USER CODE BEGIN Inertial_Unit_Start */
  /* Infinite loop */
	struct mpuSensorData sensorData;
	char outdata[100];
	struct DPL{
	int16_t h1;
	int16_t h2;
	struct mpuSensorData Data;
	};
	union{
	struct DPL Data;
	uint8_t buffer[24];
	}Packet;
  for(;;)
  {
    mpuReadSensor(&sensorData);
    Packet.Data.Data=sensorData;
	ay=sensorData.ay;
	temp_send_y_axis_acc = ay/1683.5699797160243407707910750507;
	if(ay < -1)
	{
	temp_send_y_axis_acc = (temp_send_y_axis_acc) * (-1);
	}
	send_y_axis = temp_send_y_axis_acc;
	sendbuff[0] = send_y_axis;
	osDelay(1);
  }
  /* USER CODE END Inertial_Unit_Start */
}

/* USER CODE BEGIN Header_Decision_and_Logger */
/**
* @brief Function implementing the Decision_Log thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Decision_and_Logger */
void Decision_and_Logger(void *argument)
{
	sendbuff[2] = breakout;

  /* USER CODE BEGIN Decision_and_Logger */
  /* Infinite loop */
  for(;;)
  {
	  if(ay != 0)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	  }

	  if(related_altitude>500)
	  {
		  HAL_Delay(3000);
		  breakout = 1;
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	  }

	  if(related_altitude<500 && related_altitude != 0 && altitude != 44330)
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
	  }
	  else
	  {
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	  }


	  HAL_SPI_Transmit(&hspi2, sendbuff, sizeof(sendbuff), 100);

	  if(HAL_SPI_Transmit(&hspi2, sendbuff, sizeof(sendbuff), 100)==HAL_OK)
	  {
		  spi_flag = 1;
	  }
    osDelay(1);
  }
  /* USER CODE END Decision_and_Logger */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
