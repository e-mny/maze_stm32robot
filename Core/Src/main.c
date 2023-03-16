/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
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
#include "cmsis_os.h"
#include "math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "pid.h"
#include "ICM20948.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STRAIGHT 150
#define LEFT 105 // anything below 100 is dangerous
#define RIGHT 250 //MAX IS 270

/* Controller parameters */
#define PID_KP  1.0f // default 2.0f
#define PID_KI  0.0f // default 0.5f

#define PID_LIM_MIN  1450 //
#define PID_LIM_MAX  2900 //

#define PID_TURNING_LIM_MIN  2200 //
#define PID_TURNING_LIM_MAX  3000 //

#define SAMPLE_TIME_S 0.05f

// indoor:
//#define STRAIGHTRATIOF 1.031f // for lab. this one veers more left
//#define STRAIGHTRATIOR 1.0134f // for lab. veers more left
//#define DISTANCE_ERROR_OFFSETF -0.03f
//#define DISTANCE_ERROR_OFFSETR 0.15f

//#define STRAIGHTRATIOF 1.0258f // straight on 4 october

// outdoor:
#define STRAIGHTRATIOF 0.5f
#define STRAIGHTRATIOR 1.f // ALMOST straight, abit left. if wan to go more right, gotta decrease.
#define DISTANCE_ERROR_OFFSETF -0.035f
#define DISTANCE_ERROR_OFFSETR -0.06f

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for OLEDTask */
osThreadId_t OLEDTaskHandle;
const osThreadAttr_t OLEDTask_attributes = { .name = "OLEDTask", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityLow, };
/* Definitions for GyroRead */
osThreadId_t GyroReadHandle;
const osThreadAttr_t GyroRead_attributes = { .name = "GyroRead", .stack_size =
		512 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for ServoMotor */
osThreadId_t ServoMotorHandle;
const osThreadAttr_t ServoMotor_attributes = { .name = "ServoMotor",
		.stack_size = 512 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* USER CODE BEGIN PV */
ICM20948 imu; // Declare a variable with structure type ICM20948
uint8_t moving, moveStraight, continueMoving;

double totalAngle, actualAngle;
//double actualAngle;
//int offsetAngle=0;
//int targetAngle = 0;
uint8_t aRxBuffer[1];
uint8_t newCmdReceived;
uint8_t cmd;
uint32_t data;
uint8_t num_loops;
uint8_t remainder_dist;

float distanceTraveled = 0;

//Ultrasensor variables (refer to TIM_IC_CaptureCallback)
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
float Distance = 0;
uint8_t buff[20];
uint16_t inputAngle;

uint8_t ICMAddr = 0x68;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void oled_show(void *argument);
void gyro_read(void *argument);
void servoMotor(void *argument);

/* USER CODE BEGIN PFP */
void sendToRPI(char *msg);
void moveGyroPID(float distance, int forward);
void moveGyroPIDFast(float distance, int forward);
void moveGyroPIDOld(float distance, int forward);
void move(float distance, int forward);
void turn(uint8_t direction, uint8_t forward);
void moveUltra();
void moveUltraEnd();
void moveUltraEndLeft();
void moveUltraExtreme();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM8_Init();
	MX_TIM1_Init();
	MX_USART3_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	OLED_Init();
	/* USER CODE END 2 */

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
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of OLEDTask */
	OLEDTaskHandle = osThreadNew(oled_show, NULL, &OLEDTask_attributes);

	/* creation of GyroRead */
	GyroReadHandle = osThreadNew(gyro_read, NULL, &GyroRead_attributes);

	/* creation of ServoMotor */
	ServoMotorHandle = osThreadNew(servoMotor, NULL, &ServoMotor_attributes);

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
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 160;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1000;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 72 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 0xffff - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 65535;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 7199;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */

}

/**
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

	/* USER CODE BEGIN USART3_Init 0 */

	/* USER CODE END USART3_Init 0 */

	/* USER CODE BEGIN USART3_Init 1 */

	/* USER CODE END USART3_Init 1 */
	huart3.Instance = USART3;
	huart3.Init.BaudRate = 115200;
	huart3.Init.WordLength = UART_WORDLENGTH_8B;
	huart3.Init.StopBits = UART_STOPBITS_1;
	huart3.Init.Parity = UART_PARITY_NONE;
	huart3.Init.Mode = UART_MODE_TX_RX;
	huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart3.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE,
			OLED_SCL_Pin | OLED_SDA_Pin | OLED_RST_Pin | OLED_DC_Pin
					| UltraSensor_Trigger_Pin | LED3_Pin | CIN1_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin | AIN1_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CIN2_GPIO_Port, CIN2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
	 UltraSensor_Trigger_Pin LED3_Pin CIN1_Pin */
	GPIO_InitStruct.Pin = OLED_SCL_Pin | OLED_SDA_Pin | OLED_RST_Pin
			| OLED_DC_Pin | UltraSensor_Trigger_Pin | LED3_Pin | CIN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : AIN2_Pin AIN1_Pin */
	GPIO_InitStruct.Pin = AIN2_Pin | AIN1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : CIN2_Pin */
	GPIO_InitStruct.Pin = CIN2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CIN2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
// For own reference, this function seems to be called after
// interrupt from the HAL_UART_Receive_IT function
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//Prevent unused argument(s) compilation warning
	UNUSED(huart);
	static i = 0; //static means it doesn't get erase after every loop

	if (aRxBuffer[0] == 'n') {
		i = 0;
		newCmdReceived = 1;
	} else if (i == 0) {
		cmd = aRxBuffer[0];
		i++;
	} else if (i > 0) {
		data = data * 10 + (aRxBuffer[0] - '0');
//		data = aRxBuffer[0];
	}

	uint8_t message1[20];
	HAL_UART_Receive_IT(&huart3, (uint8_t*) aRxBuffer, 1);
//	HAL_UART_Transmit(&huart3, (uint8_t *) aRxBuffer, 1, 0xFFFF);

	sprintf(message1, "%c %d %c %d\0", aRxBuffer[0], newCmdReceived, cmd, data);
	OLED_ShowString(10, 40, message1);
	OLED_Refresh_Gram(); //Refresh Ram
}

//reference: https://controllerstech.com/hcsr04-ultrasonic-sensor-and-stm32/
void delay(uint16_t time) {
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while (__HAL_TIM_GET_COUNTER (&htim3) < time)
		;

}
void HCSR04_Read(void) {
	HAL_GPIO_WritePin(UltraSensor_Trigger_GPIO_Port, UltraSensor_Trigger_Pin,
			GPIO_PIN_SET);  // pull the TRIG pin HIGH
	delay(10);  // wait for 10 us
	HAL_GPIO_WritePin(UltraSensor_Trigger_GPIO_Port, UltraSensor_Trigger_Pin,
			GPIO_PIN_RESET);  // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim3, TIM_IT_CC1);
}

//Called whenever falling or rising edge is captured
// Let's write the callback function
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel1
			{
		if (Is_First_Captured == 0) // if the first value is not captured
				{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured == 1)   // if the first is already captured
				{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			//calculate difference in 2 timestamps
			if (IC_Val2 > IC_Val1) {
				Difference = IC_Val2 - IC_Val1;
			}

			else if (IC_Val1 > IC_Val2) {
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * 0.034 / 2;
			Is_First_Captured = 0; // set it back to false

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

void task2A(uint32_t data) {
	uint32_t offset_show[30];
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);

	//if picture detected is left tln
	if (data == 'l') {
		htim1.Instance->CCR4 = 115;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2000);
		moving = 1;
		while (moving) {
			if ((int) totalAngle >= 27) {
				break;
			}
			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}
		moveGyroPID(60, 1);
		htim1.Instance->CCR4 = RIGHT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2000);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
		continueMoving = 0;
		while (Distance > 14) {
			if (totalAngle <= -1 * 15) {
				continueMoving = 1;
// 				sendToRPI("here");
				break;
			}

			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}
		if (continueMoving) {
			moveUltraExtreme();
		}

	} else {
		htim1.Instance->CCR4 = RIGHT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2000);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
		moving = 1;
		while (moving) {
			if ((int) totalAngle <= -1 * 23) // prev was 30
					{
				break;
			}
			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}
		moveGyroPID(58, 1);
		htim1.Instance->CCR4 = LEFT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2000);
		continueMoving = 0;

		while (Distance > 14) {
			if (totalAngle >= 25) {
				continueMoving = 1;
				break;
			}
			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}
		if (continueMoving) {
			moveUltraExtreme();
		}

		// THIS PORTION IS FOR COMPLETING THE TURN NO MATTER THE ULTRASENSOR DISTANCE
// 		while (moving)
// 		{
// 			if ((int)totalAngle >= 25)
// 			{
// 				continueMoving = 1;
// 				break;
// 			}
//			 sprintf(offset_show, "angle %5d\0", (int)(totalAngle));
//			 OLED_ShowString(10,20, offset_show);
//			 OLED_Refresh_Gram();
// 		 }
// 		if(continueMoving)
// 		{
//// 			moving = 0;
//// 			sendToRPI("there");
// 			moveUltraExtreme();
// 	 		continueMoving = 0;
// 		}
	}

	// straighten STM back to be perpendicular to 2nd obstacle
	if (continueMoving) {
		if (actualAngle > 0) {
			while (actualAngle > 0) {
				moving = 1;
				htim1.Instance->CCR4 = RIGHT;
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 400);
				delay(1);
			}
		} else if (actualAngle < 0) {
			while (actualAngle < 0) {
				moving = 1;
				htim1.Instance->CCR4 = LEFT;
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 400);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
				delay(1);
			}
		}
	}

	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	htim1.Instance->CCR4 = STRAIGHT;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
}

void task2A2(uint32_t data) {
	uint32_t offset_show[30];
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	moving = 0;

	if (continueMoving) {
		//if picture detected is left Tln
		if (data == 'l') {
			htim1.Instance->CCR4 = 105;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
			moving = 1;
			while (moving) {
				if ((int) actualAngle >= 60) // was 65
						{
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			moveGyroPID(85, 1);
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
			// 		continueMoving = 0;

			while (moving) {
				if ((int) actualAngle <= -83) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}

			moving = 0;
			moveGyroPIDOld(52.0, 1); // 50.0 for indoors
			moving = 0;
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
			delay(100);
			moving = 1;
			while (moving) {
				if ((int) totalAngle <= -100) //MAYBE 105 for indoors, 100 for outdoors?
						{
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			actualAngle = 0;
			moving = 0;
			moveUltraEndLeft();

		} else {
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
			moving = 1;
			while (moving) {
				if ((int) totalAngle <= -1 * 40) // was 55. was 48. was 43.
						{
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			moveGyroPID(45.0, 1);
			htim1.Instance->CCR4 = 110;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
			while (moving) {
				if ((int) actualAngle >= 85) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			moving = 0;
			moveGyroPIDOld(50.0, 1); // prev 23
			moving = 0;
			htim1.Instance->CCR4 = 115;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
			delay(100);
			moving = 1;
			while (moving) {
				if ((int) totalAngle >= 93) // 93 is too big for outdoors. changing to 88
						{
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			actualAngle = 0;
			moveUltraEnd();
		}
	} else {
		// meaning continueMoving is 0, indicating that the car
		// stopped because ultrasonic distance hit before target angle hit
		//if picture detected is left Tln
		if (data == 'l') {
			htim1.Instance->CCR4 = 105;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
			moving = 0;
			osDelay(1);
			moving = 1;
			while (moving) {
				if ((int) actualAngle >= 55) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			moving = 0;
			moveGyroPIDOld(60, 1);
//			moveGyroPID(85,1);
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);

			while (moving) {
				if ((int) actualAngle <= -85) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}

			moving = 0;
			moveGyroPIDOld(50.0, 1); // was 60
			moving = 0;
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
			delay(100);
			moving = 0;
			osDelay(1);
			moving = 1;
			while (moving) {
				if ((int) totalAngle <= -102) //MAYBE 105 for indoors, 100 for outdoors?
						{
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			actualAngle = 0;
			moving = 0;
			moveUltraEndLeft();

		} else if (data == 'r') {
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
			moving = 1;
			while (moving) {
				if ((int) totalAngle <= -1 * 22) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			moveGyroPID(75.0, 1);
			htim1.Instance->CCR4 = 110;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2000);
			while (moving) {
				if ((int) actualAngle >= 85) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			moving = 0;
			moveGyroPIDOld(45.0, 1); // prev 23
			moving = 0;
			htim1.Instance->CCR4 = 115;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2000);
			delay(100);
			moving = 0;
			osDelay(1);
			moving = 1;
			while (moving) {
				if ((int) totalAngle >= 105) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			actualAngle = 0;
			moveUltraEnd();
		}
	}

	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	htim1.Instance->CCR4 = STRAIGHT;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
}

// 2nd obstacle after turning left
void task2A2L(uint32_t data) {
	uint32_t offset_show[30];
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	moving = 0;

	if (continueMoving) {
		//	uint8_t continueMoving;
		//if picture detected is left Yln
		if (data == 'l') {
			htim1.Instance->CCR4 = 117;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2000);
			moving = 1;
			while (moving) {
				if ((int) totalAngle >= 60) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			moveGyroPID(60, 1);
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
			// 		continueMoving = 0;

			while (moving) {
				if ((int) totalAngle <= -85) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}

			moving = 0;
			moveGyroPIDOld(50.0, 1);
			moving = 0;
			delay(100);
			moving = 1;
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);

			while (moving) {
				if ((int) totalAngle <= -90) // was 95. was 90
						{
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			actualAngle = 0;
			moving = 0;
			moveUltraEndLeft();

		} else {
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
			moving = 1;
			while (moving) {
				if ((int) totalAngle <= -1 * 60) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			moveGyroPID(45.0, 1);
			htim1.Instance->CCR4 = 110;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
			while (moving) {
				if ((int) totalAngle >= 85) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			moving = 0;
			moveGyroPIDOld(70.0, 1); // prev 23
			moving = 0;
			htim1.Instance->CCR4 = 115;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
			delay(100);
			moving = 1;
			while (moving) {
				if ((int) totalAngle >= 92) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			actualAngle = 0;
			moveUltraEnd();
		}
	} else {
		// meaning continueMoving is 0, indicating that the car
		// stopped because ultrasonic distance hit before target angle hit
		//if picture detected is left Yln
		if (data == 'l') {
			htim1.Instance->CCR4 = 115;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
			moving = 1;
			while (moving) {
				if ((int) actualAngle >= 40) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			moveGyroPID(73.0, 1);
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
			// 		continueMoving = 0;

			while (moving) {
				if ((int) actualAngle <= -1 * 90) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}

			moving = 0;
			moveGyroPIDOld(40.0, 1);
			moving = 0;
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
			delay(100);
			moving = 0;
			osDelay(1);
			moving = 1;
			while (moving) {
				if ((int) totalAngle <= -1 * 90) // note: i changed to actualAngle to try, original was totalAngle = -100
						{
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			actualAngle = 0;
			moving = 0;
			moveUltraEndLeft();

		} else if (data == 'r') {
			htim1.Instance->CCR4 = RIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
			moving = 1;
			while (moving) {
				if ((int) totalAngle <= -1 * 60) // was 65
						{
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			moving = 0;
			moveGyroPIDOld(45.0, 1); // was 55
			htim1.Instance->CCR4 = 110;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
			while (moving) {
				if ((int) actualAngle >= 83) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			moving = 0;
			moveGyroPIDOld(60.0, 1); // prev 23
			moving = 0;
			htim1.Instance->CCR4 = 115;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
			delay(100); // delay because gyro needs time to reset
			moving = 1;
			while (moving) {
				if ((int) totalAngle >= 105) {
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			actualAngle = 0;
			moveUltraEnd();
		}
	}

	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	htim1.Instance->CCR4 = STRAIGHT;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
}

void secObstL() {
//	osDelay(100);
//	left90();
//
//	htim1.Instance->CCR4 = STRAIGHT; //centre
//	osDelay(100);
//	moveGyroPID(5, 1);
//
//	osDelay(100);
//	right90();
//	totalAngle = 0;
//	right90();
//	uint8_t messageA[20];
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//
//	htim1.Instance->CCR4 = STRAIGHT; //centre
//
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
//	moving = 1;
//	osDelay(200);
//	moveGyroPID(70, 1);
//	right90();
//
//	osDelay(200);
//
//	moveGyroPID(distanceTraveled, 1);

	left90();
	osDelay(400);
	moveGyroPID(5, 1);
	osDelay(400);
	right90();
	osDelay(400);
	moveGyroPID(20, 1);
	osDelay(400);
	right90();
	osDelay(400);
	moveGyroPID(70, 1);
	osDelay(400);
	right90();
	osDelay(400);
	moveGyroPIDFast(distanceTraveled + 20, 1);
	osDelay(400);
	right90();
	osDelay(400);
	moveGyroPID(8, 1);
	osDelay(400);
	left90();
	osDelay(400);
	parkingMoveUltra();
}

void secObstR() {
	right90();
	osDelay(400);
	moveGyroPID(5, 1);
	osDelay(400);
	left90();
	osDelay(400);
	moveGyroPID(20, 1);
	osDelay(400);
	left90();
	osDelay(400);
	moveGyroPID(70, 1);
	osDelay(400);
	left90();
	osDelay(400);
	moveGyroPIDFast(distanceTraveled + 20, 1);
	osDelay(400);
	left90();
	osDelay(400);
	moveGyroPID(8, 1);
	osDelay(400);
	right90();
	osDelay(400);
	parkingMoveUltra();
}

void moveUltra() {

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(40);

	// init PID controller (left)
	PIDController pidLeft = { PID_KP, PID_KI, 3650, 3850,
	SAMPLE_TIME_S };

	// init PID controller (right)
	PIDController pidRight = { PID_KP, PID_KI, 3650, 3850,
	SAMPLE_TIME_S };

	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	//reset angle
	actualAngle = 0;

	// wheel information
	float wheel_rotationTicksL = 784; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 779; // for back right wheel

	float wheel_circumference = 21.1; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim4);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;

	float distanceError;
	float distErrOffset;

	int servo;

	//pwm values
	uint16_t pwmValA = 3750;
	uint16_t pwmValC = 3750;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);

	/*Infinite loop*/
	for (;;) {

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmValC);
		currTime = HAL_GetTick();
		moving = 1;

		if (currTime - prevTime > 60L) {
			leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			rightTick = __HAL_TIM_GET_COUNTER(&htim4);

			diffLeft = 0;
			diffRight = 0;

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
				if (leftTick < leftTick_prev)
					diffLeft = leftTick_prev - leftTick;
				else
					diffLeft = (65535 - leftTick) + leftTick_prev;
			} else {
				if (leftTick > leftTick_prev)
					diffLeft = leftTick - leftTick_prev;
				else
					diffLeft = 65535 - leftTick_prev + leftTick;
			}

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
				if (rightTick < rightTick_prev)
					diffRight = rightTick_prev - rightTick;
				else
					diffRight = (65535 - rightTick) + rightTick_prev;
			} else {
				if (rightTick > rightTick_prev)
					diffRight = rightTick - rightTick_prev;
				else
					diffRight = 65535 - rightTick_prev + rightTick;
			}

			// left measured distance
			distLeft = ((float) diffLeft / wheel_rotationTicksL)
					* wheel_circumference;
			totalDistance_left += distLeft;

			// right measured distance
			distRight = ((float) diffRight / wheel_rotationTicksR)
					* wheel_circumference;
			totalDistance_right += distRight;

			// for straight ratio, higher means right dist decrease.
			// 1.015 veer right, 1.005 veer left, 1.010 ABIT right, 1.008 almost perfect.

//#define pwmValAadjust 300;
//#define pwmValBadjust 525;
//
//			pwmValA = PIDController_Update(&pidLeft,
//					totalDistance_right * STRAIGHTRATIOF, totalDistance_left,
//					pwmValA) - pwmValAadjust
//			;
//
//			pwmValC = PIDController_Update(&pidRight, totalDistance_left,
//					totalDistance_right * STRAIGHTRATIOF, pwmValC)
//					+ pwmValBadjust
//			;
//
//#define pwmValBadjust 562.5;
//
//			pwmValA = PIDController_Update(&pidLeft,
//					totalDistance_right * STRAIGHTRATIOF, totalDistance_left,
//					pwmValA);
//
//			pwmValC = PIDController_Update(&pidRight, totalDistance_left,
//					totalDistance_right * STRAIGHTRATIOF, pwmValC)
//					+ pwmValBadjust
//			;
//
//			pwmValA = PIDController_Update(&pidLeft,
//								totalDistance_right * STRAIGHTRATIOF, totalDistance_left,
//								pwmValA);
//
//						pwmValC = PIDController_Update(&pidRight, totalDistance_left,
//								totalDistance_right * STRAIGHTRATIOF, pwmValC)
//
//						;

			if (actualAngle < -0.33) //veering right
					{

				htim1.Instance->CCR4 = 140; //left abit. prev: 138
			} else if (actualAngle > 0.33) //veering left
					{

				htim1.Instance->CCR4 = 155; //right a bit. prev: 152
			} else {
				//				 servo = STRAIGHT;
				htim1.Instance->CCR4 = STRAIGHT; //Straight
			}

			osDelay(10);

			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;

			//taking in decimal place
			if (Distance < 13.5) // prev was 10
					{
				while (Distance > 8.75) {

					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 1600);
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,1600);
				}
				break;
			}
		}
	}

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 0);
	distanceTraveled = distanceTraveled
			+ ((totalDistance_left + totalDistance_right) / 2) + 10 + 50; // travelled + err + not counting
//	offsetAngle = targetAngle - actualAngle;
	htim1.Instance->CCR4 = STRAIGHT; //centre
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);

	osDelay(10);
	return;

}

void parkingMoveUltra() {

#define PID_LIM_MIN  1800 //
#define PID_LIM_MAX  2200 //

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(100);

	// init PID controller (left)
	PIDController pidLeft = { PID_KP, PID_KI, 1900, 2100,
	SAMPLE_TIME_S };

	// init PID controller (right)
	PIDController pidRight = { PID_KP, PID_KI, 1900, 2100,
	SAMPLE_TIME_S };

	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	// wheel information
	float wheel_rotationTicksL = 784; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 779; // for back right wheel

	float wheel_circumference = 23.56f; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim4);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;

	float distanceError;

	int servo;

	//pwm values
	uint16_t pwmValA = 2000;
	uint16_t pwmValC = 2000;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);

	/*Infinite loop*/
	for (;;) {

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmValC);
		currTime = HAL_GetTick();
		moving = 1;

		if (currTime - prevTime > 60) {
			leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			rightTick = __HAL_TIM_GET_COUNTER(&htim4);

			diffLeft = 0;
			diffRight = 0;

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
				if (leftTick < leftTick_prev)
					diffLeft = leftTick_prev - leftTick;
				else
					diffLeft = (65535 - leftTick) + leftTick_prev;
			} else {
				if (leftTick > leftTick_prev)
					diffLeft = leftTick - leftTick_prev;
				else
					diffLeft = 65535 - leftTick_prev + leftTick;
			}

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
				if (rightTick < rightTick_prev)
					diffRight = rightTick_prev - rightTick;
				else
					diffRight = (65535 - rightTick) + rightTick_prev;
			} else {
				if (rightTick > rightTick_prev)
					diffRight = rightTick - rightTick_prev;
				else
					diffRight = 65535 - rightTick_prev + rightTick;
			}

			// left measured distance
			distLeft = ((float) diffLeft / wheel_rotationTicksL)
					* wheel_circumference;
			totalDistance_left += distLeft;

			// right measured distance
			distRight = ((float) diffRight / wheel_rotationTicksR)
					* wheel_circumference;
			totalDistance_right += distRight;

//#define pwmValAadjust 650;
//			pwmValA = PIDController_Update(&pidLeft,
//					totalDistance_right * STRAIGHTRATIOF, totalDistance_left,
//					pwmValA) + pwmValAadjust
//			;
//
//			pwmValC = PIDController_Update(&pidRight, totalDistance_left,
//					totalDistance_right * STRAIGHTRATIOF, pwmValC);

//#define pwmValBadjust 595;
//
//				pwmValA = PIDController_Update(&pidLeft,
//						totalDistance_right * STRAIGHTRATIOF,
//						totalDistance_left, pwmValA) + pwmValAadjust;
//
//				pwmValC = PIDController_Update(&pidRight, totalDistance_left,
//						totalDistance_right * STRAIGHTRATIOF, pwmValC) + pwmValBadjust;

//			pwmValA = PIDController_Update(&pidLeft,
//					totalDistance_right * STRAIGHTRATIOF, totalDistance_left,
//					pwmValA);
//
//			pwmValC = PIDController_Update(&pidRight, totalDistance_left,
//					totalDistance_right * STRAIGHTRATIOF, pwmValC);

			//if robot doesnt go straight, using gyro to adjust, straight = 145
			if (actualAngle < -0.50) //veering right
					{

				htim1.Instance->CCR4 = 138; //left abit
//				osDelay(10);
			}
			if (actualAngle > 0.00) //veering left
					{

				htim1.Instance->CCR4 = 155; //right a bit
//				osDelay(10);
			}
			if (actualAngle == 0.00) {
//				 servo = STRAIGHT;
				htim1.Instance->CCR4 = STRAIGHT; //Straight
//				osDelay(10);
			}

			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;

			//taking in decimal place
			if (Distance < 4) // prev was 10
					{
				while (Distance > 2) {

					__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 750);
					__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,750);
				}
				break;
			}
		}
	}

	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA / 4);
	__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, pwmValC / 4);

//	moving = 0;
	move(0, 0);
//	offsetAngle = targetAngle - actualAngle;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(10);
	return;
}

void tooClose() {
#define PID_LIM_MIN  1800 //
#define PID_LIM_MAX  2200 //

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(40);

	// init PID controller (left)
	PIDController pidLeft = { PID_KP, PID_KI, 1900, 2100,
	SAMPLE_TIME_S };

	// init PID controller (right)
	PIDController pidRight = { PID_KP, PID_KI, 1900, 2100,
	SAMPLE_TIME_S };

	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	//reset angle
	actualAngle = 0;

	// wheel information
	float wheel_rotationTicksL = 784; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 779; // for back right wheel

	float wheel_circumference = 21.1; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim4);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;

	float distanceError;
	float distErrOffset;

	int servo;

	//pwm values
	uint16_t pwmValA = 2000;
	uint16_t pwmValC = 2000;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);

	// indoor

	//		if (distance == 10)
	//			distErrOffset = -0.1515;
	//		else if (distance == 20)
	//			distErrOffset = -0.07;
	//		else if (distance == 30)
	//			distErrOffset = 0.02;
	//		else if (distance == 40)
	//			distErrOffset = 0.05;
	//		else if (distance == 50)
	//			distErrOffset = 0.075;
	//		else
	//			distErrOffset = DISTANCE_ERROR_OFFSETR;

	// outdoor

	/*Infinite loop*/
	for (;;) {

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmValC);
		currTime = HAL_GetTick();
		moving = 1;

		if (currTime - prevTime > 60L) {
			leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			rightTick = __HAL_TIM_GET_COUNTER(&htim4);

			diffLeft = 0;
			diffRight = 0;

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
				if (leftTick < leftTick_prev)
					diffLeft = leftTick_prev - leftTick;
				else
					diffLeft = (65535 - leftTick) + leftTick_prev;
			} else {
				if (leftTick > leftTick_prev)
					diffLeft = leftTick - leftTick_prev;
				else
					diffLeft = 65535 - leftTick_prev + leftTick;
			}

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
				if (rightTick < rightTick_prev)
					diffRight = rightTick_prev - rightTick;
				else
					diffRight = (65535 - rightTick) + rightTick_prev;
			} else {
				if (rightTick > rightTick_prev)
					diffRight = rightTick - rightTick_prev;
				else
					diffRight = 65535 - rightTick_prev + rightTick;
			}

			// left measured distance
			distLeft = ((float) diffLeft / wheel_rotationTicksL)
					* wheel_circumference;
			totalDistance_left += distLeft;

			// right measured distance
			distRight = ((float) diffRight / wheel_rotationTicksR)
					* wheel_circumference;
			totalDistance_right += distRight;

			// for straight ratio, higher means right dist decrease.
			// 1.015 veer right, 1.005 veer left, 1.010 ABIT right, 1.008 almost perfect.

#define pwmValAadjust 300;
#define pwmValBadjust 525;

			pwmValA = PIDController_Update(&pidLeft, totalDistance_right,
					totalDistance_left * STRAIGHTRATIOR, pwmValA)
					- pwmValAadjust
			;

			pwmValC = PIDController_Update(&pidRight,
					totalDistance_left * STRAIGHTRATIOR, totalDistance_right,
					pwmValC) + pwmValBadjust
			;

			pwmValA = PIDController_Update(&pidLeft, totalDistance_right,
					totalDistance_left * STRAIGHTRATIOR, pwmValA);

			pwmValC = PIDController_Update(&pidRight,
					totalDistance_left * STRAIGHTRATIOR, totalDistance_right,
					pwmValC);

			//if robot doesnt go straight, using gyro to adjust, straight = 145

			if (actualAngle > 0.33) //veering left
					{
				//				 servo = STRAIGHT;
				htim1.Instance->CCR4 = 138; //left abit
			} else if (actualAngle < -0.33) //veering right
					{
				//					 servo = STRAIGHT;
				htim1.Instance->CCR4 = 155; //right a bit
			} else {
				//				 servo = STRAIGHT;
				htim1.Instance->CCR4 = STRAIGHT; //Straight
			}
			osDelay(10);

			if (Distance > 4.25) {
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
				htim1.Instance->CCR4 = STRAIGHT; //centre
				break;
			}
			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;
		}
	}
	//	moving = 0;
	//	offsetAngle = targetAngle - actualAngle;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(10);
	return;
}
void moveUltraEnd() {
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(100);

	// init PID controller (left)
	PIDController pidLeft = { PID_KP, PID_KI,
	PID_LIM_MIN, PID_LIM_MAX,
	SAMPLE_TIME_S };

	// init PID controller (right)
	PIDController pidRight = { PID_KP, PID_KI,
	PID_LIM_MIN, PID_LIM_MAX,
	SAMPLE_TIME_S };

	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	// wheel information
	float wheel_rotationTicksL = 784; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 779; // for back right wheel

	float wheel_circumference = 22.4f; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim4);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;
	actualAngle = 0.00;

	float distanceError;

	int servo;

	//pwm values
	uint16_t pwmValA = 3000;
	uint16_t pwmValC = 3000;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);

	/*Infinite loop*/
	for (;;) {

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmValC);
		currTime = HAL_GetTick();
		moving = 1;

		if (currTime - prevTime > 60L) {
			leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			rightTick = __HAL_TIM_GET_COUNTER(&htim4);

			diffLeft = 0;
			diffRight = 0;

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
				if (leftTick < leftTick_prev)
					diffLeft = leftTick_prev - leftTick;
				else
					diffLeft = (65535 - leftTick) + leftTick_prev;
			} else {
				if (leftTick > leftTick_prev)
					diffLeft = leftTick - leftTick_prev;
				else
					diffLeft = 65535 - leftTick_prev + leftTick;
			}

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
				if (rightTick < rightTick_prev)
					diffRight = rightTick_prev - rightTick;
				else
					diffRight = (65535 - rightTick) + rightTick_prev;
			} else {
				if (rightTick > rightTick_prev)
					diffRight = rightTick - rightTick_prev;
				else
					diffRight = 65535 - rightTick_prev + rightTick;
			}

			// left measured distance
			distLeft = ((float) diffLeft / wheel_rotationTicksL)
					* wheel_circumference;
			totalDistance_left += distLeft;

			// right measured distance
			distRight = ((float) diffRight / wheel_rotationTicksR)
					* wheel_circumference;
			totalDistance_right += distRight;

//			 pwmValA = PIDController_Update(&pidLeft, totalDistance_right*STRAIGHTRATIOF, totalDistance_left, pwmValA);
//
//			 pwmValC = PIDController_Update(&pidRight, totalDistance_left, totalDistance_right*STRAIGHTRATIOF, pwmValC);

			//if robot doesnt go straight, using gyro to adjust, straight = 145
			if (actualAngle < 0.00) //veering right
					{

				htim1.Instance->CCR4 = 138; //left abit
				osDelay(10);
			}
			if (actualAngle > 0.00) //veering left
					{

				htim1.Instance->CCR4 = 152; //right a bit
				osDelay(10);
			}
			if (actualAngle == 0.00) {
				//				 servo = STRAIGHT;
				htim1.Instance->CCR4 = STRAIGHT; //Straight
				osDelay(10);
			}

			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;

			//taking in decimal place
			if (Distance < 2) {
				break;
			}
			sprintf(messageA, "angle %5d\0", (int) (actualAngle));
			OLED_ShowString(10, 20, messageA);
			OLED_Refresh_Gram();
		}
	}

//	moving = 0;
	move(1, 0);
//	offsetAngle = targetAngle - actualAngle;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(10);
	return;

}

void moveUltraEndLeft() {
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(100);

	// init PID controller (left)
	PIDController pidLeft = { PID_KP, PID_KI,
	PID_LIM_MIN, PID_LIM_MAX,
	SAMPLE_TIME_S };

	// init PID controller (right)
	PIDController pidRight = { PID_KP, PID_KI,
	PID_LIM_MIN, PID_LIM_MAX,
	SAMPLE_TIME_S };

	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	// wheel information
	float wheel_rotationTicksL = 784; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 779; // for back right wheel

	float wheel_circumference = 22.4f; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim4);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;

	float distanceError;

	int servo;

	//pwm values
	uint16_t pwmValA = 3000;
	uint16_t pwmValC = 3000;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);

	totalAngle = 0.0;

	/*Infinite loop*/
	for (;;) {

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmValC);
		currTime = HAL_GetTick();
		moving = 1;

		if (currTime - prevTime > 60L) {
			leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			rightTick = __HAL_TIM_GET_COUNTER(&htim4);

			diffLeft = 0;
			diffRight = 0;

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
				if (leftTick < leftTick_prev)
					diffLeft = leftTick_prev - leftTick;
				else
					diffLeft = (65535 - leftTick) + leftTick_prev;
			} else {
				if (leftTick > leftTick_prev)
					diffLeft = leftTick - leftTick_prev;
				else
					diffLeft = 65535 - leftTick_prev + leftTick;
			}

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
				if (rightTick < rightTick_prev)
					diffRight = rightTick_prev - rightTick;
				else
					diffRight = (65535 - rightTick) + rightTick_prev;
			} else {
				if (rightTick > rightTick_prev)
					diffRight = rightTick - rightTick_prev;
				else
					diffRight = 65535 - rightTick_prev + rightTick;
			}

			// left measured distance
			distLeft = ((float) diffLeft / wheel_rotationTicksL)
					* wheel_circumference;
			totalDistance_left += distLeft;

			// right measured distance
			distRight = ((float) diffRight / wheel_rotationTicksR)
					* wheel_circumference;
			totalDistance_right += distRight;

//			 pwmValA = PIDController_Update(&pidLeft, totalDistance_right*STRAIGHTRATIOF, totalDistance_left, pwmValA);
//
//			 pwmValC = PIDController_Update(&pidRight, totalDistance_left, totalDistance_right*STRAIGHTRATIOF, pwmValC);

			//if robot doesnt go straight, using gyro to adjust, straight = 145
			if (totalAngle < 0.00) //veering right
					{

				htim1.Instance->CCR4 = 138; //left abit
				osDelay(10);
			}
			if (totalAngle > 0.00) //veering left
					{

				htim1.Instance->CCR4 = 152; //right a bit
				osDelay(10);
			}
			if (totalAngle == 0.00) {
				//				 servo = STRAIGHT;
				htim1.Instance->CCR4 = STRAIGHT; //Straight
				osDelay(10);
			}

			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;

			//taking in decimal place
			if (Distance < 8) {
				break;
			}
			sprintf(messageA, "angle %5d\0", (int) (actualAngle));
			OLED_ShowString(10, 20, messageA);
			OLED_Refresh_Gram();
		}
	}

//	moving = 0;
	move(1, 0);
//	offsetAngle = targetAngle - actualAngle;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(10);
	return;

}

void moveUltraExtreme() {
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(100);

	// init PID controller (left)
	PIDController pidLeft = { PID_KP, PID_KI,
	PID_LIM_MIN, PID_LIM_MAX,
	SAMPLE_TIME_S };

	// init PID controller (right)
	PIDController pidRight = { PID_KP, PID_KI,
	PID_LIM_MIN, PID_LIM_MAX,
	SAMPLE_TIME_S };

	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	// wheel information
	float wheel_rotationTicksL = 784; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 779; // for back right wheel

	float wheel_circumference = 22.4f; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim4);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;

	float distanceError;

	int servo;

	//pwm values
	uint16_t pwmValA = 2000;
	uint16_t pwmValC = 2000;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);

	/*Infinite loop*/
	for (;;) {

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmValC);
		currTime = HAL_GetTick();
		moving = 1;

		if (currTime - prevTime > 60L) {
			leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			rightTick = __HAL_TIM_GET_COUNTER(&htim4);

			diffLeft = 0;
			diffRight = 0;

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
				if (leftTick < leftTick_prev)
					diffLeft = leftTick_prev - leftTick;
				else
					diffLeft = (65535 - leftTick) + leftTick_prev;
			} else {
				if (leftTick > leftTick_prev)
					diffLeft = leftTick - leftTick_prev;
				else
					diffLeft = 65535 - leftTick_prev + leftTick;
			}

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
				if (rightTick < rightTick_prev)
					diffRight = rightTick_prev - rightTick;
				else
					diffRight = (65535 - rightTick) + rightTick_prev;
			} else {
				if (rightTick > rightTick_prev)
					diffRight = rightTick - rightTick_prev;
				else
					diffRight = 65535 - rightTick_prev + rightTick;
			}

			// left measured distance
			distLeft = ((float) diffLeft / wheel_rotationTicksL)
					* wheel_circumference;
			totalDistance_left += distLeft;

			// right measured distance
			distRight = ((float) diffRight / wheel_rotationTicksR)
					* wheel_circumference;
			totalDistance_right += distRight;

//			 pwmValA = PIDController_Update(&pidLeft, totalDistance_right*STRAIGHTRATIOF, totalDistance_left, pwmValA);
//
//			 pwmValC = PIDController_Update(&pidRight, totalDistance_left, totalDistance_right*STRAIGHTRATIOF, pwmValC);

			//if robot doesnt go straight, using gyro to adjust, straight = 145
			if (actualAngle < 0.00) //veering right
					{

				htim1.Instance->CCR4 = 115; //left abit
				osDelay(10);
			}
			if (actualAngle > 0.00) //veering left
					{

				htim1.Instance->CCR4 = 180; //right a bit
				osDelay(10);
			}
			if (actualAngle == 0.00) {
//				 servo = STRAIGHT;
				htim1.Instance->CCR4 = STRAIGHT; //Straight
				osDelay(10);
			}

			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;

			//taking in decimal place
			if (Distance < 16) // prev was 10
					{
				break;
			}
		}
	}

//	moving = 0;
	move(1, 0);
//	offsetAngle = targetAngle - actualAngle;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(10);
	return;

}

/**
 * @brief send msg to RPI through UART after execute command
 * @param msg[] for the message to be sent
 * @retval None
 */
void sendToRPI(char *msg) {
	HAL_UART_Transmit(&huart3, (uint8_t*) msg, strlen(msg), 0xFFFF);
}

//gyro for straight
void moveGyroPID(float distance, int forward) {

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(40);

	// init PID controller (left)
	PIDController pidLeft = { PID_KP, PID_KI, 1900, 2100,
	SAMPLE_TIME_S };

	// init PID controller (right)
	PIDController pidRight = { PID_KP, PID_KI, 1900, 2100,
	SAMPLE_TIME_S };

	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	//reset angle
	actualAngle = 0;

	// wheel information
	float wheel_rotationTicksL = 784; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 779; // for back right wheel

	float wheel_circumference = 21.1; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim4);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;

	float distanceError;
	float distErrOffset;

	int servo;

	//pwm values
	uint16_t pwmValA = 2000;
	uint16_t pwmValC = 2000;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];

	if (forward) {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
	}

	// indoor
	//	if (forward) {
	//		if (distance == 10)
	//			distErrOffset = -0.154;
	//		else if (distance == 20)
	//			distErrOffset = -0.0745;
	//		else if (distance == 30)
	//			distErrOffset = -0.035;
	//		else if (distance == 40)
	//			distErrOffset = -0.02;
	//		else if (distance == 50)
	//			distErrOffset = -0.001;
	//		else
	//			distErrOffset = DISTANCE_ERROR_OFFSETF;
	//	} else {
	//		if (distance == 10)
	//			distErrOffset = -0.1515;
	//		else if (distance == 20)
	//			distErrOffset = -0.07;
	//		else if (distance == 30)
	//			distErrOffset = 0.02;
	//		else if (distance == 40)
	//			distErrOffset = 0.05;
	//		else if (distance == 50)
	//			distErrOffset = 0.075;
	//		else
	//			distErrOffset = DISTANCE_ERROR_OFFSETR;
	//	}

	// outdoor
	if (forward) {
		if (distance == 10)
			distErrOffset = -0.2;
		else if (distance == 20)
			distErrOffset = -0.15;
		else if (distance == 30)
			distErrOffset = DISTANCE_ERROR_OFFSETF;
		else if (distance == 40)
			distErrOffset = DISTANCE_ERROR_OFFSETF;
		else if (distance == 50)
			distErrOffset = 0;
		else
			distErrOffset = DISTANCE_ERROR_OFFSETF;
	} else {
		if (distance == 10)
			distErrOffset = -0.1;
		else if (distance == 20)
			distErrOffset = DISTANCE_ERROR_OFFSETR;
		else if (distance == 30)
			distErrOffset = -0.03;
		else if (distance == 40)
			distErrOffset = -0.02;
		else if (distance == 50)
			distErrOffset = 0;
		else
			distErrOffset = DISTANCE_ERROR_OFFSETR;
	}

	/*Infinite loop*/
	for (;;) {
		if (totalDistance_left >= (distance + distanceError) * 0.9
				|| totalDistance_right >= (distance + distanceError) * 0.9) {
			pwmValA = 1000;
			pwmValC = 1000;

		}

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmValC);
		currTime = HAL_GetTick();
		moving = 1;

		if (currTime - prevTime > 60L) {
			leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			rightTick = __HAL_TIM_GET_COUNTER(&htim4);

			diffLeft = 0;
			diffRight = 0;

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
				if (leftTick < leftTick_prev)
					diffLeft = leftTick_prev - leftTick;
				else
					diffLeft = (65535 - leftTick) + leftTick_prev;
			} else {
				if (leftTick > leftTick_prev)
					diffLeft = leftTick - leftTick_prev;
				else
					diffLeft = 65535 - leftTick_prev + leftTick;
			}

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
				if (rightTick < rightTick_prev)
					diffRight = rightTick_prev - rightTick;
				else
					diffRight = (65535 - rightTick) + rightTick_prev;
			} else {
				if (rightTick > rightTick_prev)
					diffRight = rightTick - rightTick_prev;
				else
					diffRight = 65535 - rightTick_prev + rightTick;
			}

			// left measured distance
			distLeft = ((float) diffLeft / wheel_rotationTicksL)
					* wheel_circumference;
			totalDistance_left += distLeft;

			// right measured distance
			distRight = ((float) diffRight / wheel_rotationTicksR)
					* wheel_circumference;
			totalDistance_right += distRight;

			// for straight ratio, higher means right dist decrease.
			// 1.015 veer right, 1.005 veer left, 1.010 ABIT right, 1.008 almost perfect.

#define pwmValAadjust 300;
#define pwmValBadjust 425;

			if (forward) {

				pwmValA = PIDController_Update(&pidLeft,
						totalDistance_right * STRAIGHTRATIOF,
						totalDistance_left, pwmValA) - pwmValAadjust
				;

				pwmValC = PIDController_Update(&pidRight, totalDistance_left,
						totalDistance_right * STRAIGHTRATIOF, pwmValC)
						+ pwmValBadjust
				;
			} else {
				pwmValA = PIDController_Update(&pidLeft, totalDistance_right,
						totalDistance_left * STRAIGHTRATIOR, pwmValA)
						- pwmValAadjust
				;

				pwmValC = PIDController_Update(&pidRight,
						totalDistance_left * STRAIGHTRATIOR,
						totalDistance_right, pwmValC) + pwmValBadjust
				;
			}

//			if (forward) {
//
//				pwmValA = PIDController_Update(&pidLeft,
//						totalDistance_right * STRAIGHTRATIOF,
//						totalDistance_left, pwmValA);
//
//				pwmValC = PIDController_Update(&pidRight, totalDistance_left,
//						totalDistance_right * STRAIGHTRATIOF, pwmValC);
//			} else {
//				pwmValA = PIDController_Update(&pidLeft, totalDistance_right,
//						totalDistance_left * STRAIGHTRATIOR, pwmValA);
//
//				pwmValC = PIDController_Update(&pidRight,
//						totalDistance_left * STRAIGHTRATIOR,
//						totalDistance_right, pwmValC);
//			}

			distanceError = distErrOffset * distance;

			//if robot doesnt go straight, using gyro to adjust, straight = 145
			if (forward) {
				if (actualAngle < -0.33) //veering right
						{

					htim1.Instance->CCR4 = 138; //left abit. prev: 138
				} else if (actualAngle > 0.33) //veering left
						{

					htim1.Instance->CCR4 = 155; //right a bit. prev: 152
				} else {
					//				 servo = STRAIGHT;
					htim1.Instance->CCR4 = STRAIGHT; //Straight
				}

			} else {
				if (actualAngle > 0.33) //veering left
						{
					//				 servo = STRAIGHT;
					htim1.Instance->CCR4 = 138; //left abit
				} else if (actualAngle < -0.33) //veering right
						{
					//					 servo = STRAIGHT;
					htim1.Instance->CCR4 = 155; //right a bit
				} else {
					//				 servo = STRAIGHT;
					htim1.Instance->CCR4 = STRAIGHT; //Straight
				}
			}
			osDelay(10);

			if (totalDistance_left >= (distance + distanceError)
					|| totalDistance_right >= (distance + distanceError)) {
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
				htim1.Instance->CCR4 = STRAIGHT; //centre
				//				 moving = 0;
				//				sprintf(messageA, "dist: %.2f\0", distance);
				//				OLED_ShowString(10, 10, messageA);
				//				sprintf(messageA, "SLeft: %2d\0", pwmValA);
				//				OLED_ShowString(10, 20, messageA);
				//				sprintf(messageA, "TLeft: %.2f\0", totalDistance_left);
				//				OLED_ShowString(10, 30, messageA);
				//
				//				sprintf(messageB, "SRight: %2d\0", pwmValC);
				//				OLED_ShowString(10, 40, messageB);
				//				sprintf(messageB, "TRight: %.2f\0", totalDistance_right);
				//				OLED_ShowString(10, 50, messageB);
				//				OLED_Refresh_Gram();
				break;
			}

			//			// OLED
			//			sprintf(messageA, "SLeft: %2d\0", diffLeft);
			//			OLED_ShowString(10, 30, messageA);
			//			sprintf(messageB, "SRight: %2d\0", diffRight);
			//			OLED_ShowString(10, 40, messageB);

			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;
		}
	}
	//	moving = 0;
	//	offsetAngle = targetAngle - actualAngle;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(10);
	return;
}

//gyro fast for straight
void moveGyroPIDFast(float distance, int forward) {

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(40);

	// init PID controller (left)
	PIDController pidLeft = { PID_KP, PID_KI, 3350, 3650,
	SAMPLE_TIME_S };

	// init PID controller (right)
	PIDController pidRight = { PID_KP, PID_KI, 3350, 3650,
	SAMPLE_TIME_S };

	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	//reset angle
	actualAngle = 0;

	// wheel information
	float wheel_rotationTicksL = 784; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 779; // for back right wheel

	float wheel_circumference = 21.1; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim4);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;

	float distanceError;
	float distErrOffset;

	int servo;

	//pwm values
	uint16_t pwmValA = 3500;
	uint16_t pwmValC = 3500;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];

	if (forward) {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
	}

	// indoor
	//	if (forward) {
	//		if (distance == 10)
	//			distErrOffset = -0.154;
	//		else if (distance == 20)
	//			distErrOffset = -0.0745;
	//		else if (distance == 30)
	//			distErrOffset = -0.035;
	//		else if (distance == 40)
	//			distErrOffset = -0.02;
	//		else if (distance == 50)
	//			distErrOffset = -0.001;
	//		else
	//			distErrOffset = DISTANCE_ERROR_OFFSETF;
	//	} else {
	//		if (distance == 10)
	//			distErrOffset = -0.1515;
	//		else if (distance == 20)
	//			distErrOffset = -0.07;
	//		else if (distance == 30)
	//			distErrOffset = 0.02;
	//		else if (distance == 40)
	//			distErrOffset = 0.05;
	//		else if (distance == 50)
	//			distErrOffset = 0.075;
	//		else
	//			distErrOffset = DISTANCE_ERROR_OFFSETR;
	//	}

	// outdoor
	if (forward) {
		if (distance == 10)
			distErrOffset = -0.2;
		else if (distance == 20)
			distErrOffset = -0.15;
		else if (distance == 30)
			distErrOffset = DISTANCE_ERROR_OFFSETF;
		else if (distance == 40)
			distErrOffset = DISTANCE_ERROR_OFFSETF;
		else if (distance == 50)
			distErrOffset = 0;
		else
			distErrOffset = DISTANCE_ERROR_OFFSETF;
	} else {
		if (distance == 10)
			distErrOffset = -0.1;
		else if (distance == 20)
			distErrOffset = DISTANCE_ERROR_OFFSETR;
		else if (distance == 30)
			distErrOffset = -0.03;
		else if (distance == 40)
			distErrOffset = -0.02;
		else if (distance == 50)
			distErrOffset = 0;
		else
			distErrOffset = DISTANCE_ERROR_OFFSETR;
	}

	/*Infinite loop*/
	for (;;) {
		if (totalDistance_left >= (distance + distanceError) * 0.9
				|| totalDistance_right >= (distance + distanceError) * 0.9) {
			pwmValA = 1000;
			pwmValC = 1000;

		}

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmValC);
		currTime = HAL_GetTick();
		moving = 1;

		if (currTime - prevTime > 60L) {
			leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			rightTick = __HAL_TIM_GET_COUNTER(&htim4);

			diffLeft = 0;
			diffRight = 0;

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
				if (leftTick < leftTick_prev)
					diffLeft = leftTick_prev - leftTick;
				else
					diffLeft = (65535 - leftTick) + leftTick_prev;
			} else {
				if (leftTick > leftTick_prev)
					diffLeft = leftTick - leftTick_prev;
				else
					diffLeft = 65535 - leftTick_prev + leftTick;
			}

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
				if (rightTick < rightTick_prev)
					diffRight = rightTick_prev - rightTick;
				else
					diffRight = (65535 - rightTick) + rightTick_prev;
			} else {
				if (rightTick > rightTick_prev)
					diffRight = rightTick - rightTick_prev;
				else
					diffRight = 65535 - rightTick_prev + rightTick;
			}

			// left measured distance
			distLeft = ((float) diffLeft / wheel_rotationTicksL)
					* wheel_circumference;
			totalDistance_left += distLeft;

			// right measured distance
			distRight = ((float) diffRight / wheel_rotationTicksR)
					* wheel_circumference;
			totalDistance_right += distRight;

			// for straight ratio, higher means right dist decrease.
			// 1.015 veer right, 1.005 veer left, 1.010 ABIT right, 1.008 almost perfect.

#define pwmValAadjust 300;
#define pwmValBadjust 425;

			if (forward) {

				pwmValA = PIDController_Update(&pidLeft,
						totalDistance_right * STRAIGHTRATIOF,
						totalDistance_left, pwmValA) - pwmValAadjust
				;

				pwmValC = PIDController_Update(&pidRight, totalDistance_left,
						totalDistance_right * STRAIGHTRATIOF, pwmValC)
						+ pwmValBadjust
				;
			} else {
				pwmValA = PIDController_Update(&pidLeft, totalDistance_right,
						totalDistance_left * STRAIGHTRATIOR, pwmValA)
						- pwmValAadjust
				;

				pwmValC = PIDController_Update(&pidRight,
						totalDistance_left * STRAIGHTRATIOR,
						totalDistance_right, pwmValC) + pwmValBadjust
				;
			}

//			if (forward) {
//
//				pwmValA = PIDController_Update(&pidLeft,
//						totalDistance_right * STRAIGHTRATIOF,
//						totalDistance_left, pwmValA);
//
//				pwmValC = PIDController_Update(&pidRight, totalDistance_left,
//						totalDistance_right * STRAIGHTRATIOF, pwmValC);
//			} else {
//				pwmValA = PIDController_Update(&pidLeft, totalDistance_right,
//						totalDistance_left * STRAIGHTRATIOR, pwmValA);
//
//				pwmValC = PIDController_Update(&pidRight,
//						totalDistance_left * STRAIGHTRATIOR,
//						totalDistance_right, pwmValC);
//			}

			distanceError = distErrOffset * distance;

			//if robot doesnt go straight, using gyro to adjust, straight = 145
			if (forward) {
				if (actualAngle < -0.33) //veering right
						{

					htim1.Instance->CCR4 = 138; //left abit. prev: 138
				} else if (actualAngle > 0.33) //veering left
						{

					htim1.Instance->CCR4 = 155; //right a bit. prev: 152
				} else {
					//				 servo = STRAIGHT;
					htim1.Instance->CCR4 = STRAIGHT; //Straight
				}

			} else {
				if (actualAngle > 0.33) //veering left
						{
					//				 servo = STRAIGHT;
					htim1.Instance->CCR4 = 138; //left abit
				} else if (actualAngle < -0.33) //veering right
						{
					//					 servo = STRAIGHT;
					htim1.Instance->CCR4 = 155; //right a bit
				} else {
					//				 servo = STRAIGHT;
					htim1.Instance->CCR4 = STRAIGHT; //Straight
				}
			}
			osDelay(10);

			if (totalDistance_left >= (distance + distanceError)
					|| totalDistance_right >= (distance + distanceError)) {
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
				htim1.Instance->CCR4 = STRAIGHT; //centre
				//				 moving = 0;
				//				sprintf(messageA, "dist: %.2f\0", distance);
				//				OLED_ShowString(10, 10, messageA);
				//				sprintf(messageA, "SLeft: %2d\0", pwmValA);
				//				OLED_ShowString(10, 20, messageA);
				//				sprintf(messageA, "TLeft: %.2f\0", totalDistance_left);
				//				OLED_ShowString(10, 30, messageA);
				//
				//				sprintf(messageB, "SRight: %2d\0", pwmValC);
				//				OLED_ShowString(10, 40, messageB);
				//				sprintf(messageB, "TRight: %.2f\0", totalDistance_right);
				//				OLED_ShowString(10, 50, messageB);
				//				OLED_Refresh_Gram();
				break;
			}

			//			// OLED
			//			sprintf(messageA, "SLeft: %2d\0", diffLeft);
			//			OLED_ShowString(10, 30, messageA);
			//			sprintf(messageB, "SRight: %2d\0", diffRight);
			//			OLED_ShowString(10, 40, messageB);

			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;
		}
	}
	//	moving = 0;
	//	offsetAngle = targetAngle - actualAngle;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(10);
	return;
}

// this is without the drastic servo movement
void moveGyroPIDOld(float distance, int forward) {
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre

	// init PID controller (left)
	PIDController pidLeft = { PID_KP, PID_KI,
	PID_LIM_MIN, PID_LIM_MAX,
	SAMPLE_TIME_S };

	// init PID controller (right)
	PIDController pidRight = { PID_KP, PID_KI,
	PID_LIM_MIN, PID_LIM_MAX,
	SAMPLE_TIME_S };

	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	// wheel information
	float wheel_rotationTicksL = 784; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 779; // for back right wheel

	float wheel_circumference = 21.1f; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim4);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;

	float distanceError;

	int servo;

	//pwm values
	uint16_t pwmValA = 2000;
	uint16_t pwmValC = 2000 * STRAIGHTRATIOF;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];

	if (forward) {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
	}

	/*Infinite loop*/
	for (;;) {

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmValC);
		currTime = HAL_GetTick();
		moving = 1;

		if (currTime - prevTime > 60L) {
			leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			rightTick = __HAL_TIM_GET_COUNTER(&htim4);

			diffLeft = 0;
			diffRight = 0;

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
				if (leftTick < leftTick_prev)
					diffLeft = leftTick_prev - leftTick;
				else
					diffLeft = (65535 - leftTick) + leftTick_prev;
			} else {
				if (leftTick > leftTick_prev)
					diffLeft = leftTick - leftTick_prev;
				else
					diffLeft = 65535 - leftTick_prev + leftTick;
			}

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
				if (rightTick < rightTick_prev)
					diffRight = rightTick_prev - rightTick;
				else
					diffRight = (65535 - rightTick) + rightTick_prev;
			} else {
				if (rightTick > rightTick_prev)
					diffRight = rightTick - rightTick_prev;
				else
					diffRight = 65535 - rightTick_prev + rightTick;
			}

			// left measured distance
			distLeft = ((float) diffLeft / wheel_rotationTicksL)
					* wheel_circumference;
			totalDistance_left += distLeft;

			// right measured distance
			distRight = ((float) diffRight / wheel_rotationTicksR)
					* wheel_circumference;
			totalDistance_right += distRight;

			// for straight ratio, higher means right dist decrease.
			// 1.015 veer right, 1.005 veer left, 1.010 ABIT right, 1.008 almost perfect.

//			 if (forward){
//
//				 pwmValA = PIDController_Update(&pidLeft, totalDistance_right*STRAIGHTRATIOF, totalDistance_left, pwmValA);
//
//				 pwmValC = PIDController_Update(&pidRight, totalDistance_left, totalDistance_right*STRAIGHTRATIOF, pwmValC);
//			 }else{
//				 pwmValA = PIDController_Update(&pidLeft, totalDistance_right, totalDistance_left*STRAIGHTRATIOR, pwmValA);
//
//				 pwmValC = PIDController_Update(&pidRight, totalDistance_left*STRAIGHTRATIOR, totalDistance_right, pwmValC);
//			 }

			if (forward) {
				distanceError = DISTANCE_ERROR_OFFSETF * distance;
			} else {
				distanceError = DISTANCE_ERROR_OFFSETR * distance;
			}

			//if robot doesnt go straight, using gyro to adjust, straight = 145
			if (forward) {
				if (totalAngle < 0.00) //veering right
						{

					htim1.Instance->CCR4 = 140; //left abit. prev: 138
				}
				if (totalAngle > 0.00) //veering left
						{

					htim1.Instance->CCR4 = 155; //right a bit. prev: 152
				}
				if (totalAngle == 0.00) {
					//				 servo = STRAIGHT;
					htim1.Instance->CCR4 = STRAIGHT; //Straight
				}

			} else {
				if (totalAngle > 0.00) //veering left
						{
					//				 servo = STRAIGHT;
					htim1.Instance->CCR4 = 140; //left abit
				}
				if (totalAngle < 0.00) //veering right
						{
					servo = STRAIGHT;
					htim1.Instance->CCR4 = 155; //right a bit
				}
				if (totalAngle == 0.00) {
					//				 servo = STRAIGHT;
					htim1.Instance->CCR4 = STRAIGHT; //Straight
				}
			}
			osDelay(10);
			if (totalDistance_left >= (distance + distanceError)
					|| totalDistance_right >= (distance + distanceError)) {
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
//				 moving = 0;
				sprintf(messageA, "dist: %.2f\0", distance);
				OLED_ShowString(10, 10, messageA);
//				sprintf(messageA, "SLeft: %2d\0", pwmValA);
//				OLED_ShowString(10, 20, messageA);
				sprintf(messageA, "TLeft: %.2f\0", totalDistance_left);
				OLED_ShowString(10, 30, messageA);
//
//				sprintf(messageB, "SRight: %2d\0", pwmValC);
//				OLED_ShowString(10, 40, messageB);
				sprintf(messageB, "TRight: %.2f\0", totalDistance_right);
				OLED_ShowString(10, 50, messageB);
				OLED_Refresh_Gram();
				break;
			}

//			// OLED
//			sprintf(messageA, "SLeft: %2d\0", diffLeft);
//			OLED_ShowString(10, 30, messageA);
//			sprintf(messageB, "SRight: %2d\0", diffRight);
//			OLED_ShowString(10, 40, messageB);
			sprintf(messageA, "dist: %.2f\0", distance);
			OLED_ShowString(10, 10, messageA);

			sprintf(messageA, "TLeft: %.2f\0", totalDistance_left);
			OLED_ShowString(10, 30, messageA);

			sprintf(messageB, "TRight: %.2f\0", totalDistance_right);
			OLED_ShowString(10, 50, messageB);
			OLED_Refresh_Gram();

			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;
		}
	}
//	moving = 0;
//	offsetAngle = targetAngle - actualAngle;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	htim1.Instance->CCR4 = STRAIGHT; //centre
	return;
}

void move(float distance, int forward) {
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(100);

	// init PID controller (left)
	PIDController pidLeft = { PID_KP, PID_KI, 500, 1500,
	SAMPLE_TIME_S };

	// init PID controller (right)
	PIDController pidRight = { PID_KP, PID_KI, 500, 1500,
	SAMPLE_TIME_S };

	PIDController_Init(&pidLeft);
	PIDController_Init(&pidRight);

	// wheel information
	float wheel_rotationTicksL = 784; // for back left wheel, around 1550-1600
	float wheel_rotationTicksR = 779; // for back right wheel

	float wheel_circumference = 22.4f; // NEED TO MEASURE AND CHANGE

	// start encoder
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //encoderA
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); //encoderB

	// wheel ticks
	int leftTick_prev = __HAL_TIM_GET_COUNTER(&htim2);
	int rightTick_prev = __HAL_TIM_GET_COUNTER(&htim4);

	// cpu ticks
	uint32_t startTime = HAL_GetTick();
	uint32_t prevTime = startTime;
	uint32_t currTime;

	// distance checkers for both wheels
	float totalDistance_left = 0;
	float totalDistance_right = 0;

	// left measured distance
	float distLeft;

	// right measured distance
	float distRight;

	int leftTick;
	int rightTick;

	int diffLeft = 0;
	int diffRight = 0;

	float distanceError;

	int servo;

	//pwm values
	uint16_t pwmValA = 2000;
	uint16_t pwmValC = 2000;

	// OLED variables for testing PID
	uint8_t messageA[20];
	uint8_t messageB[20];

	if (forward) {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
	}

	/*Infinite loop*/
	for (;;) {

		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,pwmValC);
		currTime = HAL_GetTick();
		moving = 1;

		if (currTime - prevTime > 60L) {
			leftTick = __HAL_TIM_GET_COUNTER(&htim2);
			rightTick = __HAL_TIM_GET_COUNTER(&htim4);

			diffLeft = 0;
			diffRight = 0;

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2)) {
				if (leftTick < leftTick_prev)
					diffLeft = leftTick_prev - leftTick;
				else
					diffLeft = (65535 - leftTick) + leftTick_prev;
			} else {
				if (leftTick > leftTick_prev)
					diffLeft = leftTick - leftTick_prev;
				else
					diffLeft = 65535 - leftTick_prev + leftTick;
			}

			if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4)) {
				if (rightTick < rightTick_prev)
					diffRight = rightTick_prev - rightTick;
				else
					diffRight = (65535 - rightTick) + rightTick_prev;
			} else {
				if (rightTick > rightTick_prev)
					diffRight = rightTick - rightTick_prev;
				else
					diffRight = 65535 - rightTick_prev + rightTick;
			}

			// left measured distance
			distLeft = ((float) diffLeft / wheel_rotationTicksL)
					* wheel_circumference;
			totalDistance_left += distLeft;

			// right measured distance
			distRight = ((float) diffRight / wheel_rotationTicksR)
					* wheel_circumference;
			totalDistance_right += distRight;

			// for straight ratio, higher means right dist decrease.
			// 1.015 veer right, 1.005 veer left, 1.010 ABIT right, 1.008 almost perfect.

#define pwmValAadjust 300;
#define pwmValBadjust 525;

			if (forward) {

				pwmValA = PIDController_Update(&pidLeft,
						totalDistance_right * STRAIGHTRATIOF,
						totalDistance_left, pwmValA) - pwmValAadjust
				;

				pwmValC = PIDController_Update(&pidRight, totalDistance_left,
						totalDistance_right * STRAIGHTRATIOF, pwmValC)
						+ pwmValBadjust
				;
			} else {
				pwmValA = PIDController_Update(&pidLeft, totalDistance_right,
						totalDistance_left * STRAIGHTRATIOR, pwmValA)
						- pwmValAadjust
				;

				pwmValC = PIDController_Update(&pidRight,
						totalDistance_left * STRAIGHTRATIOR,
						totalDistance_right, pwmValC) + pwmValBadjust
				;
			}

			if (forward) {
				distanceError = DISTANCE_ERROR_OFFSETF * distance;
			} else {
				distanceError = DISTANCE_ERROR_OFFSETR * distance;
			}

			if (totalDistance_left >= (distance + distanceError)
					|| totalDistance_right >= (distance + distanceError)) {
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1,0);
				__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,0);
				moving = 0;
				sprintf(messageA, "dist: %.2f\0", distance);
				OLED_ShowString(10, 10, messageA);
				sprintf(messageA, "SLeft: %2d\0", pwmValA);
				OLED_ShowString(10, 20, messageA);
				sprintf(messageA, "TLeft: %.2f\0", totalDistance_left);
				OLED_ShowString(10, 30, messageA);

				sprintf(messageB, "SRight: %2d\0", pwmValC);
				OLED_ShowString(10, 40, messageB);
				sprintf(messageB, "TRight: %.2f\0", totalDistance_right);
				OLED_ShowString(10, 50, messageB);
				OLED_Refresh_Gram();
				break;
			}

//			// OLED
//			sprintf(messageA, "SLeft: %2d\0", diffLeft);
//			OLED_ShowString(10, 30, messageA);
//			sprintf(messageB, "SRight: %2d\0", diffRight);
//			OLED_ShowString(10, 40, messageB);

			prevTime = currTime;
			leftTick_prev = leftTick;
			rightTick_prev = rightTick;
		}
	}
	moving = 0;
//	offsetAngle = targetAngle - actualAngle;
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(10);
	return;
}

void bLeft90() {
	// indoor
//	totalAngle = 0;
//	uint8_t messageA[20];
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//
//	htim1.Instance->CCR4 = STRAIGHT; //centre
//	moveGyroPID(3.6, 1);
//
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
//	moving = 1;
//	htim1.Instance->CCR4 = 101;
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 500);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2900);
//	while (moving) {
//
//		if (totalAngle < -85) {
//			break;
//		}
//		sprintf(messageA, "Langle %5d\0", (int) (totalAngle));
//		//     OLED_ShowString(10,30, messageA);
//		//     OLED_Refresh_Gram();
//	}
//	//  stop();
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
//	moving = 0;
//	move(0,0);
//	moveGyroPID(5.3, 0);

	// outdoor
	totalAngle = 0;
	uint8_t messageA[20];
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre
	moveGyroPID(5.5, 1);

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
	moving = 1;
	htim1.Instance->CCR4 = LEFT;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 500);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2900);
	while (moving) {

		if (totalAngle <= -85) {
			break;
		}
		sprintf(messageA, "Langle %5d\0", (int) (totalAngle));
		//     OLED_ShowString(10,30, messageA);
		//     OLED_Refresh_Gram();
	}
	//  stop();
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;
	move(0, 0);
	moveGyroPID(3, 0);
//	//  dir = (dir + 3) % 4 ;
//	//  x += axis[dir][0] * left_90[xydir];
//	//  xydir = (xydir + 1) % 2;
//	//  y += axis[dir][1] * left_90[xydir];

}

void bRight90() {
	// indoor
//	uint8_t messageA[20];
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//
//	htim1.Instance->CCR4 = STRAIGHT; //centre
//
//	moveGyroPID(10, 1);
//	totalAngle = 0;
//	osDelay(100);
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
//	moving = 1;
//	htim1.Instance->CCR4 = 250;
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2900);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 500);
//	while (moving) {
//
//		if (totalAngle > 82) {
//			break;
//		}
//		sprintf(messageA, "Langle %5d\0", (int) (totalAngle));
//		//     OLED_ShowString(10,30, messageA);
//		//     OLED_Refresh_Gram();
//	}
//	//  stop();
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
//	moving = 0;
//	move(0, 0);
//	moveGyroPID(1, 0);

	// outdoor
	uint8_t messageA[20];
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre

	moveGyroPID(7, 1);
	totalAngle = 0;
	osDelay(100);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
	moving = 1;
	htim1.Instance->CCR4 = 250;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2700);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 600);
	while (moving) {

		if (totalAngle > 83) {
			break;
		}
		sprintf(messageA, "Langle %5d\0", (int) (totalAngle));
		//     OLED_ShowString(10,30, messageA);
		//     OLED_Refresh_Gram();
	}
	//  stop();
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;
	move(0, 0);
	moveGyroPID(2.5, 0);

}

void left90() {
	// indoor
//	totalAngle = 0;
//	uint8_t messageA[20];
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//	htim1.Instance->CCR4 = STRAIGHT; //centre
//
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
//	moving = 1;
//	htim1.Instance->CCR4 = 100;
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 600);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
//	;
//	while (moving) {
//
//		if (totalAngle > 87.5) {
//			break;
//		}
//	}
//	//  stop();
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
//	moving = 0;
//
//	//comment for task 2
////	move(0, 0);
////	moveGyroPID(10, 0);

	// outdoor
	totalAngle = 0;
	uint8_t messageA[20];
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	moving = 1;
	htim1.Instance->CCR4 = 103;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 450);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2700);
	while (moving) {

		if (totalAngle >= 85.333) {
			break;
		}
	}
//  stop();
	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(5);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;
	//comment for task 2
//	move(0, 0);
//	moveGyroPID(12.25, 0);
}

void goRoundObsticleL() {
	//indoor
//	// set up
//	totalAngle = 0;
//	uint8_t messageA[20];
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//
//	htim1.Instance->CCR4 = STRAIGHT; //centre
//
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
//	moving = 1;
//
//	// left
//	htim1.Instance->CCR4 = 120;
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 700);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
//	while (moving) {
//
//		if (totalAngle >= 45) {
//			break;
//		}
//	}
//
//	osDelay(50);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
//	moving = 0;
//
//	// right180
//	totalAngle = 0;
//	moving = 1;
//	htim1.Instance->CCR4 = 235;
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 600);
//	while (moving) {
//
//		if (totalAngle < -97) {
//			break;
//		}
//	}
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
//	moving = 0;
//	osDelay(85);
//
//	// left
//	totalAngle = 0;
//	moving = 1;
//	htim1.Instance->CCR4 = 108;
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 400);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
//	while (moving) {
//
//		if (totalAngle >= 48) {
//			break;
//		}
//	}
//
//	osDelay(50);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
//	moving = 0;
//	totalAngle = 0;

	//outdooor
	// set up
	totalAngle = 0;
	uint8_t messageA[20];
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	moving = 1;

	// left
	htim1.Instance->CCR4 = 120;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 700);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
	while (moving) {

		if (totalAngle >= 45) {
			break;
		}
	}

	osDelay(20);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;

	// right180
	totalAngle = 0;
	moving = 1;
	htim1.Instance->CCR4 = 205;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 800);
	while (moving) {

		if (totalAngle < -97) {
			break;
		}
	}
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;
	osDelay(20);

	// left
	totalAngle = 0;
	moving = 1;
	htim1.Instance->CCR4 = 104;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 700);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
	while (moving) {

		if (totalAngle >= 44) {
			break;
		}
	}

	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(20);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;
}

void goRoundObsticleR() {
	//indoor
//	// set up
//	totalAngle = 0;
//	uint8_t messageA[20];
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//
//	htim1.Instance->CCR4 = STRAIGHT; //centre
//
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
//	moving = 1;
//
//	//right
//	totalAngle = 0;
//	htim1.Instance->CCR4 = 190;
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 800);
//	while (moving) {
//
//		if (totalAngle <= -48) {
//			break;
//		}
//	}
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
//	moving = 0;
//	osDelay(50);
//
//	// left180
//	totalAngle = 0;
//	moving = 1;
//	htim1.Instance->CCR4 = 103;
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 700);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
//	while (moving) {
//
//		if (totalAngle >= 101) {
//			break;
//		}
//	}
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
//	moving = 0;
//	osDelay(85);
//
//	// right
//	totalAngle = 0;
//	moving = 1;
//	htim1.Instance->CCR4 = 200;
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3200);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 600);
//	while (moving) {
//
//		if (totalAngle <= -46) {
//			break;
//		}
//	}
//
//	osDelay(50);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
//	moving = 0;
//	totalAngle = 0;

	//outdoor
	// set up
	totalAngle = 0;
	uint8_t messageA[20];
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	moving = 1;

	//right
	totalAngle = 0;
	htim1.Instance->CCR4 = 180;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 800);
	while (moving) {

		if (totalAngle <= -45) {
			break;
		}
	}
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;
	osDelay(20);

	// left180
	totalAngle = 0;
	moving = 1;
	htim1.Instance->CCR4 = 103;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 700);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 3000);
	while (moving) {

		if (totalAngle >= 92) {
			break;
		}
	}
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;
	osDelay(20);

	// right
	totalAngle = 0;
	moving = 1;
	htim1.Instance->CCR4 = 220;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 3000);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 800);
	while (moving) {

		if (totalAngle <= -41) {
			break;
		}
	}
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;
	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(20);

}

void right90() {
	//indoor
//	totalAngle = 0;
//	uint8_t messageA[20];
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
//	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//
//	htim1.Instance->CCR4 = STRAIGHT; //centre
//
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
//	moving = 1;
//	htim1.Instance->CCR4 = 250;
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2000);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1100);
//	while (moving) {
//
//		if (totalAngle < -88) {
//			break;
//		}
//	}
////  stop();
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
//	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
//	moving = 0;
//	//comment for task 2
////	move(0, 0);
////	moveGyroPID(10, 0);

	// outdoor
	totalAngle = 0;
	uint8_t messageA[20];
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

	htim1.Instance->CCR4 = STRAIGHT; //centre

	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
	moving = 1;
	htim1.Instance->CCR4 = 250;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2700);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 700);
	while (moving) {

		if (totalAngle < -85.333) {
			break;
		}
		sprintf(messageA, "Rangle %5d\0", (int) (totalAngle));
	}
	htim1.Instance->CCR4 = STRAIGHT; //centre
	osDelay(5);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;
	// comment for task 2
//	move(0, 0);
//	moveGyroPID(8.75, 0);
}

//using this to slowly get hold of the trend for the no. of degree vs gyro values
//mainly for task A4, require to take in angles from 90 to 360
void turn(uint8_t direction, uint8_t forward) {
//	inputAngle=data;
	uint16_t offset_show[20];
//	sprintf(totalAngleShow, "angle %d\n\rdata %d\n\r\0", (int)totalAngle, data);
//	HAL_UART_Transmit(&huart3, totalAngleShow, sizeof(totalAngleShow)-1, HAL_MAX_DELAY);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = STRAIGHT;
	//move forward
	if (forward) {
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);

		if (direction) {
//			targetAngle += 90;
//			if(targetAngle > 360){
//				targetAngle -= 360;
//			}
			//left
			htim1.Instance->CCR4 = 115;
			osDelay(100);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 800);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1500);
			moving = 1;
			while (moving) {
				if ((int) totalAngle >= 36) {
					//continue moving
//					osDelay(1);
					break;
				}
//				 sprintf(offset_show, "act angle %5d\0", (int)(offsetAngle));
//				 OLED_ShowString(10,10, offset_show);
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			htim1.Instance->CCR4 = LEFT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2000);
			while (moving) {
				//inner circle diameter:28cm
				//outer circle diameter: cm
				//1 is 90

				if ((int) totalAngle >= 84)
//				 if ((int)totalAngle >= 86 + (offsetAngle))
						{
					//continue moving
					//					osDelay(1);
					break;
				}
//				 sprintf(offset_show, "act angle %5d\0", (int)(offsetAngle));
//				 OLED_ShowString(10,10, offset_show);
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
		}
		//forward right
		else {
//			targetAngle -= 90;
//			if(targetAngle < 0) targetAngle += 360;
			//right
			move(2, 1);
			htim1.Instance->CCR4 = 200;
			osDelay(100);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2000);
			moving = 1;
			while (moving) {
				if ((int) totalAngle <= -1 * 65) {
					//continue moving
//					 osDelay(1);
					break;
				}
//				 else
//				 {
//
//					 break;
//				 }
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
			htim1.Instance->CCR4 = STRAIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			osDelay(100);
			move(10, 0);
			htim1.Instance->CCR4 = RIGHT;
			osDelay(100);

			moving = 1;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 800);
			while (moving) {
				if ((int) totalAngle <= -1 * 21.5)
//				 if ((int)totalAngle <= -1*13 - (targetAngle - actualAngle))
						{
					//continue moving
					//					 osDelay(1);
					break;
				}
				//				 else
				//				 {
				//
				//					 break;
				//				 }
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
//			 osDelay(100);
//			 HAL_Delay(100);
			htim1.Instance->CCR4 = STRAIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			move(7, 0);
		}
	}

	//backwards
	else {
//		targetAngle -= 90;
//		if(targetAngle < 0) targetAngle += 360;
		//left
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
		if (direction) {
			htim1.Instance->CCR4 = LEFT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2500);
			moving = 1;
			while (moving) {
				if (totalAngle < -1 * 16) {
					break;
				}
//				 else
//				 {
//					 break;
//				 }
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();

			}
			htim1.Instance->CCR4 = STRAIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			osDelay(500);
			move(25, 1);
			//continue reverse left
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
			htim1.Instance->CCR4 = 122;
			osDelay(100);
			moving = 1;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 2000);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 2000);
			while (moving) {
				if (totalAngle < -1 * 62)
//				 if (totalAngle < -1*62 - (targetAngle - actualAngle))
						{
					break;
				}
				//				 else
				//				 {
				//					 break;
				//				 }
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();

			}
		}
		//right
		else {
//			 targetAngle += 90;
//			 if(targetAngle > 360) targetAngle -= 360;
			htim1.Instance->CCR4 = RIGHT; //right
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1500);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1500);
			moving = 1;
			while (moving) {
				if (totalAngle >= 14) {
					break;
				}
//				 else
//				 {
//					 break;
//				 }
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
//			 HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//			 HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//			 HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
//			 HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
			//robot forward to reduce turning radius
			htim1.Instance->CCR4 = STRAIGHT;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
			osDelay(500);
			move(25, 1); //move(25,1)

			//continue reverse right
			HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
			htim1.Instance->CCR4 = 180; //180
			osDelay(100);
			moving = 1;
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1400);
			__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1000);
			while (moving) {
				if (totalAngle >= 73)
//				 if (totalAngle >= 71 + (targetAngle - actualAngle))
						{
					break;
				}
				sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
				OLED_ShowString(10, 20, offset_show);
				OLED_Refresh_Gram();
			}
		}
	}

	//Stop wheels and let wheels be straight
	htim1.Instance->CCR4 = STRAIGHT;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;
	htim1.Instance->CCR4 = STRAIGHT;
	//centre
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
//	move(1, 0); //to let robot stop immediately as robot moves forward due to momentum

	osDelay(1000);
	return;
}

void spotTurn(uint8_t direction) {
	uint16_t offset_show[20];

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	htim1.Instance->CCR4 = STRAIGHT;

	//move forward
	if (direction) // spot turn left
	{
//		targetAngle += 90;
//		if(targetAngle > 360) targetAngle -= 360;
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
		htim1.Instance->CCR4 = LEFT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 800);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1500);
		moving = 1;
		while (moving) {
			if (totalAngle >= 18) {
				break;
			}
//				 else
//				 {
//					 break;
//				 }
			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
		//robot forward left to reduce turning radius
		htim1.Instance->CCR4 = RIGHT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1500);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 500);
		while (moving) {
			if (totalAngle >= 38) {
				break;
			}
			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}
		htim1.Instance->CCR4 = STRAIGHT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
		osDelay(100);
		move(5, 1);
		osDelay(100);
		moving = 1;
		//continue forward left
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
		htim1.Instance->CCR4 = LEFT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 800);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1500);
		while (moving) {
			if (totalAngle >= 23) {
				break;
			}
			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}
		//continue reverse right
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
		htim1.Instance->CCR4 = RIGHT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1500);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 500);
		while (moving) {
			if (totalAngle >= 50) //prev:50
//			 if (totalAngle >= 53 + (targetAngle - actualAngle))//prev:50
					{
				break;
			}
			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}
		move(2, 1);
	} else //spot turn right
	{
//		targetAngle -= 90;
//		if(targetAngle < 0) targetAngle += 360;
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
		htim1.Instance->CCR4 = RIGHT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1500);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 500);
		moving = 1;
		while (moving) {
			if (totalAngle <= -1 * 20) {
				break;
			}
			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
		//robot reverse left
		htim1.Instance->CCR4 = LEFT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 800);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1500);
		while (moving) {
			if (totalAngle <= -1 * 35) {
				break;
			}
			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}
		htim1.Instance->CCR4 = STRAIGHT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
		move(9.5, 1);
		osDelay(100);
		moving = 1;
		//continue forward right
		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);
		htim1.Instance->CCR4 = RIGHT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 1500);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 500);
		while (moving) {
			if (totalAngle <= -1 * 16) {
				break;
			}
			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}

		HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_SET);
		htim1.Instance->CCR4 = LEFT;
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 800);
		__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 1500);
		while (moving) {
			if (totalAngle <= -1 * 46) //previous: 48
//			 if (totalAngle <= -1*45 - (targetAngle - actualAngle))//previous: 48
					{
				break;
			}
			sprintf(offset_show, "angle %5d\0", (int) (totalAngle));
			OLED_ShowString(10, 20, offset_show);
			OLED_Refresh_Gram();
		}
		move(0.5, 0);
	}
	//Stop wheels and let wheels be straight
	htim1.Instance->CCR4 = STRAIGHT;
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_1, 0);
	__HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3, 0);
	moving = 0;

//	 //centre
//	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOE, CIN1_Pin, GPIO_PIN_SET);
//	HAL_GPIO_WritePin(GPIOC, CIN2_Pin, GPIO_PIN_RESET);

	osDelay(100);
}

////////////////////GYRO FUNCTIONS//////////////////////////
void readByte(uint8_t addr, uint8_t *data) {
	buff[0] = addr;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr << 1, buff, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, ICMAddr << 1, data, 2, 20);

}

void writeByte(uint8_t addr, uint8_t data) {
	buff[0] = addr;
	buff[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr << 1, buff, 2, 20);
}

void gyroStart() {
	writeByte(0x07, 0x07);
	osDelayUntil(10);

	writeByte(0x07, 0x00);
	osDelayUntil(10);
}

void gyroInit() {

	writeByte(0x06, 0x00);
	osDelayUntil(10);
	writeByte(0x03, 0x80);
	osDelayUntil(10);
	writeByte(0x07, 0x07);
	osDelayUntil(10);
	writeByte(0x06, 0x01);
	osDelayUntil(10);
	writeByte(0x7F, 0x20);
	osDelayUntil(10);
	writeByte(0x01, 0x2F);
	osDelayUntil(10);
	writeByte(0x0, 0x00);
	osDelayUntil(10);
	writeByte(0x7F, 0x00);
	osDelayUntil(10);
	writeByte(0x07, 0x00);
	osDelayUntil(10);

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	uint8_t instrBuffer[40], angle;
	uint16_t i = 0;
	HAL_UART_Receive_IT(&huart3, (uint8_t*) aRxBuffer, 1);

	moving = 0;
	cmd = data = newCmdReceived = 0;

	/* Infinite loop */
	for (;;) {

		//Toggle LED just to see if the code is running
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

		htim1.Instance->CCR4 = STRAIGHT; //centre

		sprintf(instrBuffer, "Dst: %.2f\0", Distance);
		OLED_ShowString(10, 30, instrBuffer);
		sprintf(instrBuffer, "Trv: %.2f\0", distanceTraveled);
		OLED_ShowString(10, 40, instrBuffer);
		OLED_Refresh_Gram();

		// assume
		// 1: forward
		// 2: left
		// 3: right
		// 4: reverse
		// 5: shortest path
		if (newCmdReceived == 1) {
			newCmdReceived = 0;
			switch (cmd) {
			case 'w':
				num_loops = floor(data / 50);
				remainder_dist = data % 50;
				for (int i = 0; i < num_loops; i++) {
					moveGyroPID(50, 1);
				}
				if (remainder_dist != 0)
					moveGyroPID(remainder_dist, 1);
//			moveGyroPIDOld(10,1);
//			gyro_move(data, 1);
//			sendToRPI("Forward done!f\n\0");
				sendToRPI("RPI:d");
				break;
			case 'l':
//			turn(1, 1);
				left90();
//			sendToRPI("Left done!\0");
				sendToRPI("RPI:d");
				break;
			case 'r':
//			turn(0, 1);
				right90();
//			sendToRPI("Right done!\0");
				sendToRPI("RPI:d");
				break;
			case 's':
				num_loops = floor(data / 50);
				remainder_dist = data % 50;
				for (int i = 0; i < num_loops; i++) {
					moveGyroPID(50, 0);
				}
				if (remainder_dist != 0)
					moveGyroPID(remainder_dist, 0);
				sendToRPI("RPI:d");
				break;
			case 'a':
				if (Distance < 8)
					tooClose();
				sendToRPI("RPI:d");
				break;
			case 'd':
				parkingMoveUltra();
				sendToRPI("RPI:d");
				break;
			case 'L':
				bLeft90();
//			sendToRPI("Reverse Left done!\0");
				sendToRPI("RPI:d");
				break;
			case 'R':
				bRight90();
//			sendToRPI("Reverse Right done!\0");
				sendToRPI("RPI:d");
				break;
			case 'x':
				goRoundObsticleL();

				htim1.Instance->CCR4 = STRAIGHT; //centre
				osDelay(300);
				if (Distance < 8)
					tooClose();
				else
					moveUltra();

				sendToRPI("RPI:d");
				break;
			case 'X':
				goRoundObsticleR();

				htim1.Instance->CCR4 = STRAIGHT; //centre
				osDelay(1000);
				if (Distance < 8)
					tooClose();
				else
					moveUltra();
				sendToRPI("RPI:d");
				break;
			case 'u':
				moveUltra();
				sendToRPI("RPI:d");
				osDelay(50);
				sendToRPI("RPI:s"); // command for rpi to take photo
//			osDelay(1000);
				break;
			case 't': // second
				secObstL();
				sendToRPI("RPI:d"); // command for rpi to take photo
//			osDelay(1000);
				break;
			case 'T': // when first obstacle goes right-> Tln or Trn
				secObstR();
				sendToRPI("RPI:d");
				break;
			case 'Y': // when first obstacle goes left-> Yln or Yrn
				task2A2L(data);
				sendToRPI("RPI:d");
				break;
			default:
				break;
			}

			data = 0;
		}
		osDelay(50);
	}

	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_oled_show */
/**
 * @brief Function implementing the OLEDTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_oled_show */
void oled_show(void *argument) {
	/* USER CODE BEGIN oled_show */
	/* Infinite loop */

	for (;;) {

	}
	/* USER CODE END oled_show */
}

/* USER CODE BEGIN Header_gyro_read */
/**
 * @brief Function implementing the myTask09 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_gyro_read */
void gyro_read(void *argument) {
	/* USER CODE BEGIN gyro_read */
	/* Infinite loop */
	uint8_t val[2] = { 0, 0 };
	uint16_t offset_show[20];
	int16_t angularSpeed = 0;
	uint32_t tick = 0;
	int dir, offset;
	offset = 0;

	gyroInit();
	gyroStart();
	tick = HAL_GetTick();
	osDelayUntil(10);
	actualAngle = 0;

	for (;;) {

		if (HAL_GetTick() - tick >= 100L) {

			//read gyro value Z-axis
			readByte(0x37, val);
			//Angle per >100 ticks
			angularSpeed = (val[0] << 8) | val[1];

			if (moving || moveStraight) {
//			  actualAngle += (double)(angularSpeed + 0.5)*((HAL_GetTick() - tick)/16400.0)*1.02;
				totalAngle += (double) (angularSpeed + 0.5)
						* ((HAL_GetTick() - tick) / 16400.0) * 1.02;
				actualAngle += (double) (angularSpeed + 0.5)
						* ((HAL_GetTick() - tick) / 16400.0) * 1.02;

//			  if(actualAngle >= 720)
//			  {
//				  actualAngle = 0;
//			  }
				if (totalAngle >= 720) {
					totalAngle = 0;
				}
				if (actualAngle >= 720) {
					actualAngle = 0;
				}
			} else {
				totalAngle = 0;
			}

//		  to show angle in real time
//		  totalAngle += (double)(angularSpeed + 0.5)*((HAL_GetTick() - tick)/16400.0)*1.02;
//		  if(totalAngle >= 720)
//		  {
//			  totalAngle = 0;
//		  }
//

//		  sprintf(offset_show, "angle:: %5d\0", (int)(actualAngle));
//		  OLED_ShowString(10,10, offset_show);
//		  OLED_Refresh_Gram();
			tick = HAL_GetTick();
		}

	}
	/* USER CODE END gyro_read */
}

/* USER CODE BEGIN Header_servoMotor */
/**
 * @brief Function implementing the ServoMotor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_servoMotor */
void servoMotor(void *argument) {
	/* USER CODE BEGIN servoMotor */
	uint8_t message[20];
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); //will call callback function when interrupt is detected
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	for (;;) {
//		htim1.Instance -> CCR4 = RIGHT; //right
//		osDelay(2000);
//		htim1.Instance -> CCR4 = STRAIGHT; //centre
//		osDelay(2000);
//		htim1.Instance -> CCR4 = LEFT; //left
//		osDelay(2000);
//		htim1.Instance -> CCR4 = STRAIGHT; //centre
//		osDelay(2000);
		HCSR04_Read();
//		sprintf(message, "distance: %5.2f\0", Distance);
//		OLED_ShowString(10, 50, message);
//		OLED_Refresh_Gram(); //Refresh Ram
		HAL_Delay(200);
	}
	/* USER CODE END servoMotor */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
