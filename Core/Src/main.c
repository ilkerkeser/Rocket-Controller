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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

//BME280 and BNO055 library
#include "bno055_stm32.h"
#include "bme280.h"
#include "lwgps/lwgps.h"
#include <math.h>

//SD Card library
#include "File_Handling.h"

#include "KalmanFilter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union{
 float sayi;
 unsigned char array[4];
} float32toInt8;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
xTaskHandle UARTTaskHandle;
xTaskHandle BME280TaskHandle;
xTaskHandle BNO055TaskHandle;
/*Rocket controller tasks*/
xTaskHandle GPSTaskHandle; //GPS
xTaskHandle SDCardTaskHandle; //SD card
xTaskHandle FirstIgnTaskHandle;
xTaskHandle SecondIgnTaskHandle;

//Create Semaphore
xSemaphoreHandle Semap1;

//Kalman Filter
KalmanFilter_t kalman_filter;

//---------Sensor and Modules Variables-------------
//BNO055 variables
bno055_vector_t accel,gyro,euler;
//accelometer variables
float accel_x,accel_y,accel_z;
//gyroscope variables
float gyro_x,gyro_y,gyro_z;
//Euler angles variables
float euler_x,euler_y,euler_z;

//After kalman filters BNO055 variables
//accelometer kalman variables
float accel_x_kalman, accel_y_kalman, accel_z_kalman;

//gyroscope kalman variables
float gyro_x_kalman, gyro_y_kalman, gyro_z_kalman;

//Euler kalman angles
float euler_x_kalman, euler_y_kalman, euler_z_kalman;

//BME280 variables
float Temperature,Pressure,Humidity;

extern float Altitude;

//Kalman variable
float Altitude_Kalman,Pressure_Kalman, Temperature_Kalman, Humidity_Kalman;

//GPS variables
float gpsAlt,gpsLat,gpsLon;

//message length
char buff[60];

int sayac=0;

lwgps_t gps;

extern uint8_t rx_data[1];

//ignitions status
int ign1_status =0;
int ign2_status =0;

//velocity variables
float v0 =0.0; //başlangıç hızı
float velocity = 0.0; // Başlangıç hızı (0 kabul edelim)
float velocity_change;
float dt = 0.1; // Zaman adımı (100 ms)
float acceleration_vector;

int indx = 0;
//transmit message buffer

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SDIO_SD_Init(void);
/* USER CODE BEGIN PFP */
void StartUARTTask(void *argument);
void StartBME280Task(void *argument);
void StartBNO055Task(void *argument);
void StartGPSTask(void *argument);
void StartIgn1Task(void *argument);
void StartIgn2Task(void *argument);
void StartSDCardTask(void *argument);

float calculate_velocity(float v0,float a, float dt);
float calculate_acceleration_magnitude(float ax, float ay, float az);
//checksum function
unsigned char cs(void);
//paket mesaj
void paket(int sayac);
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
	BaseType_t status;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_FATFS_Init();
  MX_SDIO_SD_Init();
  /* USER CODE BEGIN 2 */

  //Init Kalman filter
  KalmanFilter_Init(&kalman_filter, 2.0f, 1.0f, 0.01f);

  //Config BME280
   BME280_Config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);

   //Config BNO055
   bno055_assignI2C(&hi2c2);
   bno055_setup();
   bno055_setOperationModeNDOF();

   //SD Card inits
   Mount_SD("/");
   Format_SD();
   Create_File("Rocket_Data.TXT");
   Unmount_SD("/");

   //Create UART task
   status = xTaskCreate(StartUARTTask, "UART", 128, NULL, 2, &UARTTaskHandle);

   configASSERT(status==pdPASS);

   //Create BME280 Task
   status = xTaskCreate(StartBME280Task, "BME280", 128, NULL, 2, &BME280TaskHandle);

   configASSERT(status==pdPASS);

   /*BNO055 Task*/
   status = xTaskCreate(StartBNO055Task, "BNO055", 128, NULL, 2, &BNO055TaskHandle);

   configASSERT(status==pdPASS);

   /*GPS Task*/
   status = xTaskCreate(StartGPSTask, "NEOM8N", 128, NULL, 2, &GPSTaskHandle);

   configASSERT(status==pdPASS);

   /*Ignition 1 and 2 task*/
   status = xTaskCreate(StartIgn1Task, "IGN1", 128, NULL, 2, &FirstIgnTaskHandle);

   configASSERT(status==pdPASS);

   status = xTaskCreate(StartIgn2Task, "IGN2", 128, NULL, 2, &SecondIgnTaskHandle);

   configASSERT(status==pdPASS);

   /*SD Card Task. This task has lower priority*/
   status = xTaskCreate(StartSDCardTask, "SD-Card", 128, NULL, 1, &SDCardTaskHandle);

   configASSERT(status==pdPASS);


   //Create semaphore. I used semaphore because that there are same priorites task.
   Semap1 = xSemaphoreCreateCounting(9, 0);

   if(Semap1 != NULL){
 	  //Create successfully
   }

   vTaskStartScheduler();

   xTaskNotifyGive(BME280TaskHandle);

   xTaskNotifyGive(BNO055TaskHandle);

   xTaskNotify(GPSTaskHandle,0,eNoAction);

   xTaskNotify(UARTTaskHandle,0,eNoAction);
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hi2c2.Init.ClockSpeed = 100000;
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
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IGN1_OUT_Pin|SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IGN2_OUT_GPIO_Port, IGN2_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IGN1_OUT_Pin */
  GPIO_InitStruct.Pin = IGN1_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IGN1_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IGN2_OUT_Pin */
  GPIO_InitStruct.Pin = IGN2_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IGN2_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//UART Task
void StartUARTTask(void *argument){

	for(;;){
		if(Semap1 != NULL){
			if(xSemaphoreTake(Semap1,portMAX_DELAY) == pdTRUE){
				//Send message to ground station
				if(HAL_UART_Transmit(&huart1, (uint8_t *)buff, 60, 150) != HAL_OK){
					//Error handler

				}
				xSemaphoreGive(Semap1);
			}
		}
	}
}

//BME280 task
void StartBME280Task(void *argument){

	for(;;){
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		if(Semap1 != NULL){
			if(xSemaphoreTake(Semap1,portMAX_DELAY) == pdTRUE){
				/* We were able to obtain the semaphore and can now access the
				   shared resource. */
				Temperature = BME280_Temperature();

				Pressure = BME280_Pressure();

				Humidity = BME280_Humidity();

				Altitude = BME280_Altitude();

				//vTaskDelay(pdMS_TO_TICKS(50));

				//Kalman variables
				Temperature_Kalman = BME280_Kalman_Temp(Temperature);

				Humidity_Kalman = BME280_Kalman_Hum(Humidity);

				Pressure_Kalman = BME280_Kalman_Press(Pressure);

				Altitude_Kalman = BME280_Kalman_Alt(Altitude);

				// xTaskNotify to First IGN and Second IGN@
				xTaskNotify(FirstIgnTaskHandle,0,eNoAction);

				xTaskNotify(SecondIgnTaskHandle,0,eNoAction);

				//Get relative altitude, temperature and pressure
				// research how to calculate altitude from pressure

				/* We have finished accessing the shared resource. Release the
				   semaphore. */

				xSemaphoreGive(Semap1);
			}else{
				/* We could not obtain the semaphore and can therefore not access
				   the shared resource safely. */
			}

		}
		xTaskNotifyGive(FirstIgnTaskHandle);
		xTaskNotifyGive(SecondIgnTaskHandle);
	}
}


//BNO055 task
void StartBNO055Task(void *argument){

	for(;;){
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		if(Semap1 != NULL){
			if(xSemaphoreTake(Semap1,portMAX_DELAY) == pdTRUE){
				/* We were able to obtain the semaphore and can now access the
					shared resource. */
				//Get accelerometer variables
				accel = bno055_getVectorAccelerometer();

				accel_x = accel.x;
				accel_y = accel.y;
				accel_z = accel.z;

				accel_x_kalman = updateEstimate(&kalman_filter, accel_x);
				accel_y_kalman = updateEstimate(&kalman_filter, accel_y);
				accel_z_kalman = updateEstimate(&kalman_filter, accel_z);

				//vTaskDelay(pdMS_TO_TICKS(100));

				//Get gyroscope variables
				gyro = bno055_getVectorGyroscope();

				gyro_x = gyro.x;
				gyro_y = gyro.y;
				gyro_z = gyro.z;

				gyro_x_kalman = updateEstimate(&kalman_filter, gyro_x);
				gyro_y_kalman = updateEstimate(&kalman_filter, gyro_y);
				gyro_z_kalman = updateEstimate(&kalman_filter, gyro_z);

				//vTaskDelay(pdMS_TO_TICKS(100));

				//get euler variables
				euler = bno055_getVectorEuler();

				euler_x = euler.x;
				euler_y = euler.y;
				euler_z = euler.z;

				euler_x_kalman = updateEstimate(&kalman_filter, euler_x);
				euler_y_kalman = updateEstimate(&kalman_filter, euler_y);
				euler_z_kalman = updateEstimate(&kalman_filter, euler_z);

				//vector acceleration
				acceleration_vector = calculate_acceleration_magnitude(accel_x_kalman, accel_y_kalman, accel_z_kalman);

				//calculate the derivative of velocity
				velocity_change = calculate_velocity(velocity, acceleration_vector, dt);

				//update the new velocity
				velocity += velocity_change;

				//Notify IGN1 function
				xTaskNotify(FirstIgnTaskHandle,0,eNoAction);

				//vTaskDelay(pdMS_TO_TICKS(100));

				xSemaphoreGive(Semap1);
			}
		}
		xTaskNotifyGive(FirstIgnTaskHandle);
	}

}


//GPS function
void StartGPSTask(void *argument){

	for(;;){
		if(Semap1 != NULL){
			if(xSemaphoreTake(Semap1,portMAX_DELAY) == pdTRUE){

				gpsAlt = gps.altitude;
				gpsLat = gps.latitude;
				gpsLon = gps.longitude;

				/* We have finished accessing the shared resource. Release the
				semaphore. */
				xSemaphoreGive(Semap1);
			}
		}
	}

}

//Ignition 1 function
//@NOTE: Based parameters are change of accelometer (velocity) over hight is 4000m
void StartIgn1Task(void *argument){

	for(;;){
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);

		if(ign1_status == 0){
			if(Altitude_Kalman >= 4000.0f){
				if(fabs(velocity) < 0.01 && fabs(acceleration_vector) < 0.01){
					ign1_status = 1;
					//fire the ignition output
					HAL_GPIO_WritePin(IGN1_OUT_GPIO_Port, IGN1_OUT_Pin, GPIO_PIN_SET);
				}
			}
		}
		//send a notification to BNO055 and BME280 task that bringing out of the blocked state
		xTaskNotifyGive(BNO055TaskHandle);
		xTaskNotifyGive(BME280TaskHandle);
	}

}

//Ignition 2 function
//Only altitude parameter <=500 or 600
void StartIgn2Task(void *argument){

	for(;;){
		ulTaskNotifyTake(pdTRUE,portMAX_DELAY);
		if(ign1_status == 1){
			if(ign2_status == 0){
				if(Altitude_Kalman >= 600.0f){
					//IGN2 is high
					ign2_status = 1;
					HAL_GPIO_WritePin(IGN2_OUT_GPIO_Port, IGN2_OUT_Pin, GPIO_PIN_SET);

				}
			}
		}
		xTaskNotifyGive(BME280TaskHandle);
	}

}


void StartSDCardTask(void *argument){

	for(;;){
		if(Semap1 != NULL){
			if(xSemaphoreTake(Semap1,portMAX_DELAY) == pdTRUE){
				Mount_SD("/");
				Update_File("Rocket_Data.TXT", buff);
				Unmount_SD("/");
				indx++;

				xSemaphoreGive(Semap1);
			}
		}
	}
}



//paket oluşturma
void paket(int sayac){
	buff[0] = sayac;

	//Altitude
	float32toInt8 alt_conversion;
	alt_conversion.sayi = Altitude_Kalman;
	buff[1] = alt_conversion.array[0];
	buff[2] = alt_conversion.array[1];
	buff[3] = alt_conversion.array[2];
	buff[4] = alt_conversion.array[3];

	//GPS Latitude
	float32toInt8 gpsConversion_lat;
	gpsConversion_lat.sayi = gpsLat;
	buff[5] = gpsConversion_lat.array[0];
	buff[6] = gpsConversion_lat.array[1];
	buff[7] = gpsConversion_lat.array[2];
	buff[8] = gpsConversion_lat.array[3];

	//GPS Longitude
	float32toInt8 gpsConversion_lon;
	gpsConversion_lon.sayi = gpsLon;
	buff[9] = gpsConversion_lon.array[0];
	buff[10] = gpsConversion_lon.array[1];
	buff[11] = gpsConversion_lon.array[2];
	buff[12] = gpsConversion_lon.array[3];

	float32toInt8 gpsConversion_alt;
	gpsConversion_alt.sayi = gpsAlt;
	buff[13] = gpsConversion_alt.array[0];
	buff[14] = gpsConversion_alt.array[1];
	buff[15] = gpsConversion_alt.array[2];
	buff[16] = gpsConversion_alt.array[3];

	float32toInt8 accelx_conversion;
	accelx_conversion.sayi = accel_x_kalman;
	buff[17] = accelx_conversion.array[0];
	buff[18] = accelx_conversion.array[1];
	buff[19] = accelx_conversion.array[2];
	buff[20] = accelx_conversion.array[3];


	float32toInt8 accely_conversion;
	accely_conversion.sayi = accel_y_kalman;
	buff[21] = accely_conversion.array[0];
	buff[22] = accely_conversion.array[1];
	buff[23] = accely_conversion.array[2];
	buff[24] = accely_conversion.array[3];

	float32toInt8 accelz_conversion;
	accelz_conversion.sayi = accel_z_kalman;
	buff[25] = accelz_conversion.array[0];
	buff[26] = accelz_conversion.array[1];
	buff[27] = accelz_conversion.array[2];
	buff[28] = accelz_conversion.array[3];

	float32toInt8 gyro_x_conversion;
	gyro_x_conversion.sayi = gyro_x_kalman;
	buff[29] = gyro_x_conversion.array[0];
	buff[30] = gyro_x_conversion.array[1];
	buff[31] = gyro_x_conversion.array[2];
	buff[32] = gyro_x_conversion.array[3];


	float32toInt8 gyro_y_conversion;
	gyro_y_conversion.sayi = gyro_y_kalman;
	buff[33] = gyro_y_conversion.array[0];
	buff[34] = gyro_y_conversion.array[1];
	buff[35] = gyro_y_conversion.array[2];
	buff[36] = gyro_y_conversion.array[3];


	float32toInt8 gyro_z_conversion;
	gyro_z_conversion.sayi = gyro_z_kalman;
	buff[37] = gyro_z_conversion.array[0];
	buff[38] = gyro_z_conversion.array[1];
	buff[39] = gyro_z_conversion.array[2];
	buff[40] = gyro_z_conversion.array[3];

	float32toInt8 euler_x_conversion;
	euler_x_conversion.sayi = euler_x_kalman;
	buff[41] = euler_x_conversion.array[0];
	buff[42] = euler_x_conversion.array[1];
	buff[43] = euler_x_conversion.array[2];
	buff[44] = euler_x_conversion.array[3];

	float32toInt8 euler_y_conversion;
	euler_y_conversion.sayi = euler_y_kalman;
	buff[45] = euler_y_conversion.array[0];
	buff[46] = euler_y_conversion.array[1];
	buff[47] = euler_y_conversion.array[2];
	buff[48] = euler_y_conversion.array[3];

	float32toInt8 euler_z_conversion;
	euler_z_conversion.sayi = euler_z_kalman;
	buff[49] = euler_z_conversion.array[0];
	buff[50] = euler_z_conversion.array[1];
	buff[51] = euler_z_conversion.array[2];
	buff[52] = euler_z_conversion.array[3];

	buff[53] = (uint8_t)ign1_status;
	buff[54] = (uint8_t)ign2_status;

	buff[55] = cs();
}

//checksum
unsigned char cs(){
	 int checkSum = 0;
	 for(int i=1; i<48; i++)
	 {
		 checkSum += buff[i];
	 }
	 return (unsigned char) (checkSum % 256);
}

float calculate_velocity(float v0,float a, float dt){
	return  v0 + a * dt;
}

float calculate_acceleration_magnitude(float ax, float ay, float az){
	return sqrt(ax * ax + ay * ay + az * az);
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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
