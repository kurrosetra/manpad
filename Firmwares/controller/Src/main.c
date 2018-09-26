/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include <arm_math.h>

#include "MPU6050_register.h"
#include "tm_stm32_ahrs_imu.h"
#include "MP_controller.h"
#include "pid_controller.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* TODO Private Variables here */
#define DEBUG								1
#if DEBUG
#define IMU_DEBUG							0
#define SERVO_DEBUG							0
#define MANPAD_DEBUG						1
#endif	//if DEBUG

#define UART_BUFSIZE						1024

char tx1Data[UART_BUFSIZE];
#if DEBUG
char tx2Data[UART_BUFSIZE];
#endif	//if DEBUG

/* Private Variables for MPU6050 */
#define MPU6050_ADDRESS						0xD0
#define MPU6050_DATA_SIZE					14
//#define AHRS_INCLINATION_DEG				0.6833f
#define AHRS_INCLINATION_DEG				0.0f

uint8_t imuData[MPU6050_DATA_SIZE];
volatile uint8_t imuDataUpdated = 0;
volatile uint8_t mpuState = 0;
volatile FlagStatus finDataUpdate = 0;
#if IMU_DEBUG
volatile uint16_t counter = 0;
volatile uint16_t errCounter = 0;
#endif	//if IMU_DEBUG

typedef struct
{
	float x;
	float y;
	float z;
} MPU6050_DATATYPE;

MPU6050_DATATYPE acc, gyro;
TM_AHRSIMU_t ahrsImu;

/* Private Variables for ODROID */
#define CMD_HEADER_CHAR						'$'
#define CMD_SEPARATOR_CHAR					';'
#define CMD_DATA_TERMINATOR_CHAR			'#'
#define CMD_SERVO_PARAM_TERMINATOR_CHAR		'^'
#define CMD_FIN_PARAM_TERMINATOR_CHAR		'*'
#define CMD_DATA_TIMEOUT					1000

typedef struct _MP_ODROID_DATA_t
{
	float sigYZ;
	float roll;
	float pitch;
} MP_ODROID_DATA_t;
MP_ODROID_DATA_t MP_ODROID_INPUT = { 0.0f, 0.0f, 0.0f };
uint32_t odroid_last_command_timer = 0;
char rxBuffer[UART_BUFSIZE];
volatile FlagStatus new_data = RESET;
volatile ITStatus usart1RxReady = RESET;
volatile uint16_t uart1RxLength = 0;

/* Private Variables for SERVO */
typedef enum
{
	SERVO_ROLL,
	SERVO_PITCH
} SERVO_NAME_e;
#define SERVO_CENTER			1500
#define SERVO_ROLL_OFFSET		200
#define SERVO_PITCH_OFFSET		75

int servoRollAngle = 90;
int servoPitchAngle = 90;

volatile FlagStatus servo_new_param = RESET;

/* Private Variables for MANPAD */
float Zpid[3] = { 1.0, 0.0, 0.0 };
float Rpid[3] = { 1.0, 0.0, 0.0 };
float gainImu[2] = { 1.0, 1.0 };
float gainCom[2] = { 1.0, 1.0 };

float AyzCom = 0.0, rollCom = 0.0;
float AzImu = 0.0, rollImu = 0.0;
PIDControl zPID;
PIDControl rPID;

FlagStatus fin_start = RESET;
volatile FlagStatus fin_new_param = RESET;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* TODO Private Functions here */
static uint8_t MP_Parsing_Data(char *buf);
static void MP_Parsing_Servo_Param(char *buf);
static void MP_Parsing_Fin_Param(char *buf);
static void SERVO_set_position(int rollAngle, int pitchAngle);

static void pidInit();
static void pidUpdate();
static void finSendCommand();

static void gravityCompensateAcc(TM_AHRSIMU_t *ahrs, MPU6050_DATATYPE *acc);
static size_t map(size_t x, size_t in_min, size_t in_max, size_t out_min, size_t out_max);
static void MPU_Init();
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

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
	MX_USART2_UART_Init();
	MX_USART1_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Initialization here */

#if DEBUG
	/* clear vt100 screen & put cursor to home position (upper left) */
	sprintf(tx2Data, "\x1b[2J\x1b[H");
	HAL_UART_Transmit(&huart2, (uint8_t *) tx2Data, strlen(tx2Data), 10);
#endif	//if DEBUG

	/* MPU init */
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, SET);
	MPU_Init();
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, RESET);

	/* Init structure with 100hZ sample rate, 0.1 beta and 0.6833 inclination (+0° 41' is inclination in Bandung) on Sept, 2018 */
	/* http://www.magnetic-declination.com/Indonesia/Jakarta/1071519.html */
	TM_AHRSIMU_Init(&ahrsImu, 500, 0.1f, AHRS_INCLINATION_DEG);

	PIDModeSet(&zPID, MANUAL);
	PIDModeSet(&rPID, MANUAL);

	/* ENABLE UART1 IT_RXNE */
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	//servo ROLL & pitch zero
	SERVO_set_position(90, 90);
	// fin zero
	sprintf(tx1Data, "$CMD,0.0,0.0*");
	HAL_UART_Transmit(&huart1, (uint8_t*) tx1Data, strlen(tx1Data), 10);

	/* PID timer */
	HAL_TIM_Base_Start_IT(&htim2);
	/* IMU read timer */
	HAL_TIM_Base_Start_IT(&htim3);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int rollError = 0;
	int pitchError = 0;

	uint32_t millis = 0;
	int16_t a[3], g[3];
	char cmdIn[UART_BUFSIZE];

#if IMU_DEBUG
	uint32_t updateTimer = 0;
#endif	//if IMU_DEBUG

	while (1)
	{

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO Begin loop */
		millis = HAL_GetTick();
#if IMU_DEBUG
		if (millis >= updateTimer)
		{
			updateTimer = millis + 1000;

			HAL_UART_Transmit(&huart2, (uint8_t *) tx2Data, strlen(tx2Data), 10);

			MPU6050_DATATYPE accTem = acc;
			gravityCompensateAcc(&ahrsImu, &accTem);
			sprintf(tx1Data,
					"counter;err= %d;%d\r\n\tYPR= %.3f %.3f %.3f\r\n\tacc= %.3f %.3f %.3f\r\n\taccTem= %.3f %.3f %.3f\r\n\tgyro= %.3f %.3f %.3f\r\n",
					counter, errCounter, ahrsImu.Yaw, ahrsImu.Pitch, ahrsImu.Roll, acc.x, acc.y,
					acc.z, accTem.x, accTem.y, accTem.z, gyro.x, gyro.y, gyro.z);

			counter = 0;
			errCounter = 0;
			HAL_UART_Transmit_IT(&huart2, (uint8_t *) tx1Data, strlen(tx1Data));
		}
#endif	//if IMU_DEBUG

		if (odroid_last_command_timer && millis >= odroid_last_command_timer)
		{
			odroid_last_command_timer = 0;

			SERVO_set_position(90, 90);

			MP_ODROID_INPUT.sigYZ = 0.0f;
			MP_ODROID_INPUT.roll = 0.0f;
			MP_ODROID_INPUT.pitch = 0.0f;
		}

		if (imuDataUpdated)
		{
			mpuState = 1;
			imuDataUpdated = 0;
#if IMU_DEBUG
			counter++;
#endif	//if DEBUG

			/* parsing acc data imuData[0..5] */
			a[0] = ((int16_t) imuData[0] << 8 | imuData[1]);
			a[1] = ((int16_t) imuData[2] << 8 | imuData[3]);
			a[2] = ((int16_t) imuData[4] << 8 | imuData[5]);

			/* parsing gyro data imuData[8..13] */
			g[0] = ((int16_t) imuData[8] << 8 | imuData[9]);
			g[1] = ((int16_t) imuData[10] << 8 | imuData[11]);
			g[2] = ((int16_t) imuData[12] << 8 | imuData[13]);
			mpuState = 0;

			/* convert data */
			acc.x = (float) a[0] / 2048.0f;
			acc.y = (float) a[1] / 2048.0f;
			acc.z = (float) a[2] / 2048.0f;

			gyro.x = (float) g[0] / 16.4f;
			gyro.y = (float) g[1] / 16.4f;
			gyro.z = (float) g[2] / 16.4f;

			/* update ahrs */
			TM_AHRSIMU_UpdateIMU(&ahrsImu, AHRSIMU_DEG2RAD(gyro.x), AHRSIMU_DEG2RAD(gyro.y),
					AHRSIMU_DEG2RAD(gyro.z), acc.x, acc.y, acc.z);
		}

		if (finDataUpdate == SET)
		{
			finDataUpdate = RESET;

			if (zPID.mode == AUTOMATIC && rPID.mode == AUTOMATIC)
			{
				pidUpdate();
				/* send data to fin */
				finSendCommand();
			}
		}

		if (usart1RxReady == SET)
		{
			HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);

			memcpy(cmdIn, rxBuffer, UART_BUFSIZE);

			if (new_data == SET)
			{
				if (MP_Parsing_Data(cmdIn))
				{
					/* servo new data */
					rollError = MP_ODROID_INPUT.roll;
					pitchError = MP_ODROID_INPUT.pitch;
					SV_calculation(rollError, pitchError);
					SV_getAngle(&servoRollAngle, &servoPitchAngle);
					SERVO_set_position(servoRollAngle, servoPitchAngle);

					/* fin new data */
					MP_Conversion(&MP_ODROID_INPUT.sigYZ, &MP_ODROID_INPUT.roll, &AyzCom, &rollCom);

					new_data = RESET;

					if (zPID.mode == MANUAL || rPID.mode == MANUAL)
						pidInit();

					odroid_last_command_timer = millis + CMD_DATA_TIMEOUT;

#if SERVO_DEBUG
					sprintf(tx2Data, "INPUT= %.3f,%.3f,%.3f\r\nservo=%i %i\r\nfin=%.3f,%.3f\r\n",MP_ODROID_INPUT.sigYZ, MP_ODROID_INPUT.roll,
							MP_ODROID_INPUT.pitch, servoRollAngle, servoPitchAngle, AyzCom, rollCom);
					HAL_UART_Transmit_IT(&huart2, (uint8_t*) tx2Data, strlen(tx2Data));
#endif	//if SERVO_DEBUG

				}
			}
			else if (servo_new_param == SET)
			{
				MP_Parsing_Servo_Param(cmdIn);
				servo_new_param = RESET;
			}
			else if (fin_new_param == SET)
			{
				MP_Parsing_Fin_Param(cmdIn);
				fin_new_param = RESET;
			}

			usart1RxReady = RESET;
		}

		/* TODO End Loop */
	}
	/* USER CODE END 3 */

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Initializes the CPU, AHB and APB busses clocks
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
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1
			| RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 400000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 71;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 19999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 999;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 143;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

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
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 * Free pins are configured automatically as Analog (this feature is enabled through
 * the Code Generation settings)
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : LED_BUILTIN_Pin */
	GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC14 PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 PA5 PA6 PA7
	 PA8 PA11 PA12 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8
			| GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB1 PB2 PB10
	 PB11 PB12 PB13 PB14
	 PB15 PB3 PB4 PB5
	 PB8 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11
			| GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_3 | GPIO_PIN_4
			| GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* TODO non-hardware functions related*/
static void finSendCommand()
{
	sprintf(tx1Data, "$CMD,%.2f,%.2f*", zPID.output, rPID.output);
	HAL_UART_Transmit_IT(&huart1, (uint8_t*) tx1Data, strlen(tx1Data));
#if MANPAD_DEBUG
	char buf[UART_BUFSIZE];
	sprintf(buf, "\x1b[2J\x1b[H%s", tx1Data);
	HAL_UART_Transmit_IT(&huart2, (uint8_t *) buf, strlen(buf));
#endif	//if MANPAD_DEBUG
}

static void pidInit()
{
	zPID.setpoint = 0.0f;
	rPID.setpoint = 0.0f;
	PIDInit(&zPID, Zpid[0], Zpid[1], Zpid[2], 0.05f, -15.0f, 15.0f, AUTOMATIC, DIRECT);
	PIDInit(&rPID, Rpid[0], Rpid[1], Rpid[2], 0.05f, -15.0f, 15.0f, AUTOMATIC, DIRECT);
}

static void pidUpdate()
{
	MPU6050_DATATYPE accTem = acc;

	gravityCompensateAcc(&ahrsImu, &accTem);
	AzImu = accTem.z * gainImu[0];
	zPID.input = AzImu - AyzCom;
	PIDCompute(&zPID);

	/*
	 * author miftakur
	 *
	 * ahrsImu.Roll >= 0 : roll ke kiri (roll minus)
	 * ahrsImu.Roll < 0 : roll ke kanan (roll plus)
	 */

	if (ahrsImu.Roll >= 0)
		rollImu = ahrsImu.Roll - 180.0f;
	else
		rollImu = ahrsImu.Roll + 180.0f;
	rollImu *= gainImu[1];
	rPID.input = rollImu - rollCom;
	PIDCompute(&rPID);
}

static void SERVO_set_position(int rollAngle, int pitchAngle)
{
	uint16_t roll, pitch;

	roll = constrain(map(rollAngle, 0, 180, 2000, 1000) - SERVO_ROLL_OFFSET, 1000, 2000);
	pitch = constrain(map(pitchAngle, 0, 180, 2000, 1000) - SERVO_PITCH_OFFSET, 1000, 2000);

	htim2.Instance->CCR1 = roll;
	htim2.Instance->CCR2 = pitch;
}

static void MP_Parsing_Fin_Param(char *buf)
{
// $[Z_Kp];[Z_Ki];[Z_Kd];[R_Kp];[R_Ki];[R_Kd];[gainAzImu];[gainRollImu];[gainAzCom];[gainAyCom]*
	const uint8_t constVal = 20;
	char tem[constVal];
	char *pointer = buf;
	uint16_t a, i, pos;

	/* Z_PID */
	for ( a = 0; a < 3; a++ )
	{
		pointer++;
		memset(tem, 0, constVal);
		pos = strlen(pointer);
		for ( i = 0; i < pos; i++ )
		{
			if (*pointer != CMD_SEPARATOR_CHAR)
				tem[i] = *pointer++;
			else
				break;
		}
		Zpid[a] = atof(tem);
	}

	/* R_PID */
	for ( a = 0; a < 3; a++ )
	{
		pointer++;
		memset(tem, 0, constVal);
		pos = strlen(pointer);
		for ( i = 0; i < pos; i++ )
		{
			if (*pointer != CMD_SEPARATOR_CHAR)
				tem[i] = *pointer++;
			else
				break;
		}
		Rpid[a] = atof(tem);
	}

	/* gainImu	=	 gainAzImu	&	gainAyImu */
	for ( a = 0; a < 2; a++ )
	{
		pointer++;
		memset(tem, 0, constVal);
		pos = strlen(pointer);
		for ( i = 0; i < pos; i++ )
		{
			if (*pointer != CMD_SEPARATOR_CHAR)
				tem[i] = *pointer++;
			else
				break;
		}
		gainImu[a] = atof(tem);
	}

	/* gainCom[0]	=	gainAzCom */
	pointer++;
	memset(tem, 0, constVal);
	pos = strlen(pointer);
	for ( i = 0; i < pos; i++ )
	{
		if (*pointer != CMD_SEPARATOR_CHAR)
			tem[i] = *pointer++;
		else
			break;
	}
	gainCom[0] = atof(tem);

	/* gainCom[1]	=	gainAyCom */
	pointer++;
	memset(tem, 0, constVal);
	pos = strlen(pointer);
	for ( i = 0; i < pos; i++ )
	{
		if (*pointer != CMD_FIN_PARAM_TERMINATOR_CHAR)
			tem[i] = *pointer++;
		else
			break;
	}
	gainCom[1] = atof(tem);

}

static void MP_Parsing_Servo_Param(char *buf)
{
	float gain[2];
	int slew[2];
	const uint8_t constVal = 20;
	char tem[constVal];
	char *pointer = buf;
	uint16_t i;
	uint16_t pos;

	/* rollGain */
	pointer++;
	memset(tem, 0, constVal);
	pos = strlen(pointer);
	for ( i = 0; i < pos; i++ )
	{
		if (*pointer != CMD_SEPARATOR_CHAR)
			tem[i] = *pointer++;
		else
			break;
	}
	gain[0] = atof(tem);

	/* pitchGain */
	pointer++;
	memset(tem, 0, constVal);
	pos = strlen(pointer);
	for ( i = 0; i < pos; i++ )
	{
		if (*pointer != CMD_SEPARATOR_CHAR)
			tem[i] = *pointer++;
		else
			break;
	}
	gain[1] = atof(tem);

	/* rollSlew */
	pointer++;
	memset(tem, 0, constVal);
	pos = strlen(pointer);
	for ( i = 0; i < pos; i++ )
	{
		if (*pointer != CMD_SEPARATOR_CHAR)
			tem[i] = *pointer++;
		else
			break;
	}
	slew[0] = atoi(tem);

	/* pitchSlew */
	pointer++;
	memset(tem, 0, constVal);
	pos = strlen(pointer);
	for ( i = 0; i < pos; i++ )
	{
		if (*pointer != CMD_SERVO_PARAM_TERMINATOR_CHAR)
			tem[i] = *pointer++;
		else
			break;
	}
	slew[1] = atoi(tem);

	SV_setParameter(gain, slew);
}

static uint8_t MP_Parsing_Data(char *buf)
{
	const uint8_t constVal = 20;
	char fTem[constVal];
	char *pointer = buf;
	uint16_t i;
	uint16_t pos;

	/*
	 * author miftakur
	 *
	 *	data format: $[sigYZ],[roll],[pitch]#
	 *	example:
	 *	$1.0,1.0,1.0#
	 */

	/* MP_ODROID_INPUT.sigYZ */
	pointer++;
	memset(fTem, 0, constVal);
	pos = strlen(pointer);
	for ( i = 0; i < pos; i++ )
	{
		if (*pointer != CMD_SEPARATOR_CHAR)
			fTem[i] = *pointer++;
		else
			break;
	}
	/* not found */
	if (i == pos)
		return 0;
	MP_ODROID_INPUT.sigYZ = atof(fTem);

	/* MP_ODROID_INPUT.roll */
	pointer++;
	memset(fTem, 0, constVal);
	pos = strlen(pointer);
	for ( i = 0; i < pos; i++ )
	{
		if (*pointer != CMD_SEPARATOR_CHAR)
			fTem[i] = *pointer++;
		else
			break;
	}
	/* not found */
	if (i == pos)
		return 0;
	MP_ODROID_INPUT.roll = atof(fTem);

	/* MP_ODROID_INPUT.pitch */
	pointer++;
	memset(fTem, 0, constVal);
	pos = strlen(pointer);
	for ( i = 0; i < pos; i++ )
	{
		if (*pointer != CMD_DATA_TERMINATOR_CHAR)
			fTem[i] = *pointer++;
		else
			break;
	}
	/* not found */
	if (i == pos)
		return 0;
	MP_ODROID_INPUT.pitch = atof(fTem);

	return 1;
}

static void gravityCompensateAcc(TM_AHRSIMU_t *ahrs, MPU6050_DATATYPE *acc)
{
	float g[3];

	/* get expected direction of gravity in the sensor frame */
	g[0] = 2 * ((ahrs->_q1 * ahrs->_q3) - (ahrs->_q0 * ahrs->_q2));
	g[1] = 2 * ((ahrs->_q0 * ahrs->_q1) + (ahrs->_q2 * ahrs->_q3));
	g[2] = (ahrs->_q0 * ahrs->_q0) - (ahrs->_q1 * ahrs->_q1) - (ahrs->_q2 * ahrs->_q2)
			+ (ahrs->_q3 * ahrs->_q3);

	/* compensate accelerometer readings with the expected direction of gravity */
	acc->x += g[0];
	acc->y += g[1];
	acc->z += g[2];

}

static size_t map(size_t x, size_t in_min, size_t in_max, size_t out_min, size_t out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* TODO hardware functions related*/

void usart1_callback_IT()
{
	uint32_t isrflags = READ_REG(huart1.Instance->SR);
	char c;

	/* RXNE handler */
	if ((isrflags & USART_SR_RXNE) != RESET)
	{
		c = (char) (huart1.Instance->DR & (uint8_t) 0x00FF);
		if (c == CMD_HEADER_CHAR)
		{
			memset(rxBuffer, 0, UART_BUFSIZE);
			uart1RxLength = 0;
		}
		else if (c == CMD_DATA_TERMINATOR_CHAR)
		{
			new_data = SET;
			usart1RxReady = SET;
		}
		else if (c == CMD_SERVO_PARAM_TERMINATOR_CHAR)
		{
			servo_new_param = SET;
			usart1RxReady = SET;
		}
		else if (c == CMD_FIN_PARAM_TERMINATOR_CHAR)
		{
			fin_new_param = SET;
			usart1RxReady = SET;
		}

		rxBuffer[uart1RxLength++] = c;
	}

	/* ------------------------------------------------------------ */
	/* Other USART1 interrupts handler can go here ...             */
	/* UART in mode Transmitter ------------------------------------------------*/
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		finDataUpdate = SET;
	}

	if (htim->Instance == TIM3)
	{
		if (mpuState == 0)
		{
			/* start retrieve imu data */
#if IMU_DEBUG
			if (HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H,
							I2C_MEMADD_SIZE_8BIT, imuData, MPU6050_DATA_SIZE) != HAL_OK)
			errCounter++;
#else
			HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H,
			I2C_MEMADD_SIZE_8BIT, imuData, MPU6050_DATA_SIZE);

#endif	//if IMU_DEBUG

		}
#if IMU_DEBUG
		else
		errCounter++;
#endif	//if IMU_DEBUG
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	/* Prevent unused argument(s) compilation warning */
	UNUSED(hi2c);

	imuDataUpdated = 1;
}

static void MPU_Init()
{
	uint8_t regTem = 0x80;
#if IMU_DEBUG
	char tmp[3][64];
	char tmpString[UART_BUFSIZE];
#endif	//if IMU_DEBUG

	/* reset the whole module first */
	regTem = 1 << MPU6050_PWR1_DEVICE_RESET_BIT;
	if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
			&regTem, 1, 10) == HAL_OK)
	{
		HAL_Delay(100);
		/* PLL with X axis gyroscope reference */
		regTem = MPU6050_CLOCK_PLL_XGYRO;
		if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT,
				&regTem, 1, 10) == HAL_OK)
		{
#if IMU_DEBUG
			sprintf(tmp[0], "MPU6050 has been waken up\r\n");
#endif	//if IMU_DEBUG
		}
		/* set accelerometer range to +- 16g */
		regTem = 0x18;
		if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT,
				&regTem, 1, 10) == HAL_OK)
		{
#if IMU_DEBUG
			sprintf(tmp[1], "set accelerometer range to +-16g\r\n");
#endif	//if IMU_DEBUG
		}

		/* set gyroscope range to +-2000 degrees */
		regTem = 0x18;
		if (HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT,
				&regTem, 1, 10) == HAL_OK)
		{
#if IMU_DEBUG
			sprintf(tmp[2], "set gyroscope range to +-2000 degrees\r\n");
#endif	//if IMU_DEBUG
		}

#if IMU_DEBUG
		uint8_t val[2];
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT,
				&val[0], 1, 10);
		HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT,
				&val[1], 1, 10);
		sprintf(tmpString, "%s%s%s\r\n%X %X\r\n\r\n", tmp[0], tmp[1], tmp[2], val[0], val[1]);
		HAL_UART_Transmit_IT(&huart2, (uint8_t *) tmpString, strlen(tmpString));
#endif	//if IMU_DEBUG

	}

}
/* TODO end */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
