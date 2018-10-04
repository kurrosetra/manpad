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
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* TODO Private Variables */
#define USE_RAW_ADC						1

#define DEBUG							0
#if DEBUG
#define CMD_DEBUG						1
#endif	//if DEBUG

#define CMD_HEADER_CHAR					'$'
#define CMD_SEPARATOR_CHAR				','
#define CMD_TERMINATOR_CHAR				'*'

#define UART_BUFSIZE					1024

typedef struct
{
	char buffer[UART_BUFSIZE];
	volatile uint16_t head;
	volatile uint16_t tail;
	uint16_t size;
} Ring_Buffer_t;

Ring_Buffer_t rx1Buffer = { { 0 }, 0, 0, UART_BUFSIZE };
//char rx1Buffer[UART_BUFSIZE];
char tx1Buffer[UART_BUFSIZE];
#if DEBUG
char tx2Buffer[UART_BUFSIZE];
#endif	//if DEBUG

typedef enum
{
	M_ROLL_UP,
	M_ROLL_DOWN,
	M_PITCH_RIGHT,
	M_PITCH_LEFT
} MOTOR_NAME_e;

typedef struct
{
	MOTOR_NAME_e type;
	uint16_t pwmValue;
	/* pwm value */
	uint16_t pLow;
	uint16_t pZero;
	uint16_t pHigh;
	int16_t pDegMax;
	int16_t pDegMin;
	/* adc value */
	uint16_t aLow;
	uint16_t aZero;
	uint16_t aHigh;
} MOTOR_ID_t;

MOTOR_ID_t mUp, mDown, mRight, mLeft;

int16_t mCmd[2] = { 0, 0 };
uint16_t mPos[4];

volatile uint16_t rx1Length = 0;
volatile ITStatus rx1Ready = RESET;
volatile uint16_t uart1RxLength = 0;
volatile FlagStatus newAdcData = RESET;
volatile FlagStatus startSendLogData = RESET;
volatile uint16_t adcValue[4];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
/* TODO Private Functions here */
#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

static uint8_t parseCommand(char *buf);
static int mapInt(int x, int in_min, int in_max, int out_min, int out_max);
#if USE_RAW_ADC==0
static int16_t adcMapping(MOTOR_ID_t * mtr, uint16_t value);
#endif	//if USE_RAW_ADC==0

static void ring_buffer_write(Ring_Buffer_t *buffer, char c);
static char ring_buffer_read(Ring_Buffer_t *buffer);

static void motorInit();
static void motorSet(int16_t roll, int16_t pitch);
static void motorSetEach(MOTOR_ID_t *mtr, int16_t val);
#if USE_RAW_ADC==0
static void motorGetPos(float *pos);
#endif	//if USE_RAW_ADC==0

//static void delayIwdg(uint32_t _delayTime);
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
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	/* TODO Functions Initialization here */

#if DEBUG
//	delayIwdg(500);
	HAL_Delay(500);
//	printf("\x1b[2J\x1b[H===START===\r\n");
//	printf("===START===");
#endif	//if DEBUG

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);

	motorInit();

	/* motor Roll init */
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	//UP FIN
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);	//DOWN FIN
	/* motor Pitch init */
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);	//RIGHT FIN
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);	//LEFT FIN

	motorSet(0, 0);

	/* init adc-timer based */
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *) adcValue, 4);

	HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, RESET);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	char cmdIn[UART_BUFSIZE];
	uint8_t startSaveBuffer = 0;
	uint16_t cmdInIndex = 0;
	char c;

#if DEBUG
	uint32_t millis = 0;
	uint32_t updateTimer = 0;
#endif	//if DEBUG

	while (1)
	{

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		/* TODO Begin Loop */
#if DEBUG
		millis = HAL_GetTick();
		if (millis >= updateTimer)
		{
			updateTimer = millis + 1000;

//			printf("adcVal= %d %d %d %d\r\n", adcValue[0], adcValue[1], adcValue[2], adcValue[3]);
//			printf("$%d,%d*\r\n", mCmd[0], mCmd[1]);
		}
#endif	//if DEBUG

		if (rx1Ready == SET)
		{
			/* save rxBuffer to tem buffer */
			while (rx1Buffer.head != rx1Buffer.tail)
			{
				c = ring_buffer_read(&rx1Buffer);
				if (c == CMD_HEADER_CHAR)
				{
					memset(cmdIn, 0, UART_BUFSIZE);
					startSaveBuffer = 1;
					cmdInIndex = 0;
				}
				if (startSaveBuffer)
				{
					cmdIn[cmdInIndex++] = c;
					if (c == CMD_TERMINATOR_CHAR)
						break;
				}
			}

//			memcpy(cmdIn, rx1Buffer, UART_BUFSIZE);

			/* parse incoming command */
			if (parseCommand(cmdIn))
				LED_BUILTIN_GPIO_Port->ODR ^= LED_BUILTIN_Pin;
//				HAL_GPIO_TogglePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin);

			rx1Ready = RESET;
		}

		if (startSendLogData == SET)
		{
			/* update motor */
			motorSet(mCmd[0], mCmd[1]);

			/* TODO re-format adcValue to degrees */
#if DEBUG
			sprintf(tx1Buffer, "$%.1f,%.1f,\t%d,%d,%d,%d,\t%d,%d,%d,%d*\r\n", (float) mCmd[0] / 10,
					(float) mCmd[1] / 10, mUp.pwmValue, mRight.pwmValue, mDown.pwmValue,
					mLeft.pwmValue, adcValue[0], adcValue[2], adcValue[1], adcValue[3]);
#else
#if USE_RAW_ADC
			sprintf(tx1Buffer, "$%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d*\r\n", (float) mCmd[0] / 10,
					(float) mCmd[1] / 10, mUp.pwmValue, mRight.pwmValue, mDown.pwmValue,
					mLeft.pwmValue, adcValue[0], adcValue[2], adcValue[1], adcValue[3]);
#else
			float _f[4];
			motorGetPos(_f);
			/* format data */
			sprintf(tx1Buffer, "$%.1f,%.1f,%d,%d,%d,%d,%.1f,%.1f,%.1f,%.1f*\r\n",
					(float) mCmd[0] / 10, (float) mCmd[1] / 10, mUp.pwmValue, mDown.pwmValue,
					mRight.pwmValue, mLeft.pwmValue, _f[0], _f[1], _f[2], _f[3]);
#endif	//if USE_RAW_ADC
#endif	//if DEBUG

			HAL_UART_Transmit_IT(&huart1, (uint8_t *) tx1Buffer, strlen(tx1Buffer));

			startSendLogData = RESET;
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
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

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

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
	PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 4;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = SERVO_CENTER;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 71;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 19999;
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

	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = SERVO_CENTER;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim3);

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

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC14 PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 PA5 PA8 PA11
	 PA12 PA15 */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12
			| GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB2 PB10 PB11 PB12
	 PB13 PB14 PB15 PB4
	 PB5 PB6 PB7 PB8
	 PB9 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13
			| GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
			| GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* TODO Non-hardware related Functions */

static uint8_t parseCommand(char *buf)
{
	const uint8_t constVal = 16;
	char fTem[constVal];
	char *pointer = buf;
	float _f;
	uint16_t i, pos;
	uint16_t roll = 0;

	/*
	 * author miftakur
	 *
	 * data format: $CMD,[ROLL_ANGLE],[PITCH_ANGLE]*
	 * example:
	 * $CMD,0,0*
	 * $CMD,14.0,-15.0*
	 * $CMD,-14.0,15.0*
	 * $CMD,-14.0,-15.0*
	 *
	 */

	/* ROLL_ANGLE */
	pointer += 5;
	memset(fTem, 0, constVal);
	pos = strlen(pointer);
	if (pos > 0)
	{
		for ( i = 0; i < pos; i++ )
		{
			if (*pointer != CMD_SEPARATOR_CHAR)
				fTem[i] = *pointer++;
			else
				break;
		}
		/* not found */
		if (i >= pos)
			return 0;
	}
	else
		return 0;
	_f = atof(fTem);
	roll = (int16_t) (_f * 10);

	/* PITCH_ANGLE */
	pointer++;
	memset(fTem, 0, constVal);
	pos = strlen(pointer);
	if (pos > 0)
	{
		for ( i = 0; i < pos; i++ )
		{
			if (*pointer != CMD_TERMINATOR_CHAR)
				fTem[i] = *pointer++;
			else
				break;
		}
		/* not found */
		if (i >= pos)
			return 0;
	}
	else
		return 0;
	_f = atof(fTem);
	mCmd[1] = (int16_t) (_f * 10);
	mCmd[0] = roll;

	/* add constrain to input */
	int16_t min, max;

	/* roll angle */
	if (mUp.pDegMin >= mDown.pDegMin)
		min = mUp.pDegMin;
	else
		min = mDown.pDegMin;
	if (mUp.pDegMax >= mDown.pDegMax)
		max = mDown.pDegMax;
	else
		max = mUp.pDegMax;

	mCmd[0] = constrain(mCmd[0], min, max);

	/* pitch angle */
	if (mRight.pDegMin >= mLeft.pDegMin)
		min = mRight.pDegMin;
	else
		min = mLeft.pDegMin;
	if (mRight.pDegMax >= mLeft.pDegMax)
		max = mLeft.pDegMax;
	else
		max = mRight.pDegMax;

	mCmd[1] = constrain(mCmd[1], min, max);

	return 1;
}

static uint16_t mapPwm(MOTOR_ID_t *mtr, int16_t x)
{
	int inMin, inMax, inZero;
	int outMin, outMax, outZero;
	int mapVal;

	outMin = mtr->pLow;
	outMax = mtr->pHigh;
	inZero = 0;
	inMin = mtr->pDegMin;
	inMax = mtr->pDegMax;
	outZero = mtr->pZero;

	x = constrain(x, inMin, inMax);

	if (mtr->type == M_PITCH_LEFT)
		x = 0 - x;

	if (x < inZero)
		mapVal = constrain(mapInt((int ) x, inMin, inZero, outMin, outZero), outMin, outZero);
	else
		mapVal = constrain(mapInt((int ) x, inZero, inMax, outZero, outMax), outZero, outMax);

	return (uint16_t) mapVal;
}

static int mapInt(int x, int in_min, int in_max, int out_min, int out_max)
{
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}

#if USE_RAW_ADC==0
static int16_t adcMapping(MOTOR_ID_t * mtr, uint16_t value)
{
	int ret = 0;
	int inMin, inMax, inZero;
	int outMin, outMax;
	int leftDiff = 0;

	inMin = mtr->aLow;
	inMax = mtr->aHigh;
	inZero = mtr->aZero;
	outMin = mtr->pDegMin;
	outMax = mtr->pDegMax;

	if (mtr->type == M_PITCH_LEFT)
	{
		leftDiff = abs(inZero - value);

		if (value < inZero)
		value = inZero + leftDiff;
		else if (value > inZero)
		value = inZero - leftDiff;
	}

	if (inMin > inMax)
	value = constrain(value, inMax, inMin);
	else
	value = constrain(value, inMin, inMax);

	/* divide into 2 region, to accommodate un-linear between low-zero & zero-high */
	if (value < inZero)
	ret = mapInt((int) value, inMin, inZero, outMin, 0);
	else
	ret = mapInt((int) value, inZero, inMax, 0, outMax);

	return (int16_t) ret;
}
#endif	//if USE_RAW_ADC==0

static void ring_buffer_write(Ring_Buffer_t *buffer, char c)
{
	uint16_t i = (buffer->head + 1) % buffer->size;

	if (i != buffer->tail)
	{
		buffer->buffer[buffer->head] = c;
		buffer->head = i;
	}
}

static char ring_buffer_read(Ring_Buffer_t *buffer)
{
	if (buffer->head == buffer->tail)
		return -1;
	else
	{
		char c = buffer->buffer[buffer->tail];
		buffer->tail = (buffer->tail + 1) % buffer->size;

		return c;
	}
}

/* TODO hardware related functions */
static void motorInit()
{
	mUp.type = M_ROLL_UP;
	mUp.pZero = 1130;
	mUp.pLow = 1100;
	mUp.pHigh = 1230;
	mUp.pwmValue = mUp.pZero;
	mUp.aZero = 3370;
	mUp.aHigh = 3000;
	mUp.aLow = 3460;
	mUp.pDegMax = 100;
	mUp.pDegMin = -50;

	mDown.type = M_ROLL_DOWN;
	mDown.pZero = 1120;
	mDown.pLow = 1000;
	mDown.pHigh = 1260;
	mDown.pwmValue = mDown.pZero;
	mDown.aZero = 3480;
	mDown.aHigh = 2960;
	mDown.aLow = 3820;
	mDown.pDegMax = 150;
	mDown.pDegMin = -100;

	mRight.type = M_PITCH_RIGHT;
	mRight.pZero = 1200;
	mRight.pLow = 1050;
	mRight.pHigh = 1350;
	mRight.pwmValue = mRight.pZero;
	mRight.aZero = 3175;
	mRight.aHigh = 2590;
	mRight.aLow = 3670;
	mRight.pDegMax = 150;
	mRight.pDegMin = -150;

	mLeft.type = M_PITCH_LEFT;
	mLeft.pZero = 1130;
	mLeft.pLow = 1000;
	mLeft.pHigh = 1280;
	mLeft.pwmValue = mLeft.pZero;
	mLeft.aZero = 3460;
	mLeft.aHigh = 2840;
	mLeft.aLow = 3820;
	mLeft.pDegMax = 150;
	mLeft.pDegMin = -120;
}

static void motorSet(int16_t roll, int16_t pitch)
{
	motorSetEach(&mUp, roll);
	motorSetEach(&mDown, roll);
	motorSetEach(&mRight, pitch);
	motorSetEach(&mLeft, pitch);
}

static void motorSetEach(MOTOR_ID_t *mtr, int16_t val)
{
	mtr->pwmValue = mapPwm(mtr, val);

	if (mtr->type == M_ROLL_UP)
		htim2.Instance->CCR1 = mtr->pwmValue;
	else if (mtr->type == M_ROLL_DOWN)
		htim2.Instance->CCR3 = mtr->pwmValue;
	else if (mtr->type == M_PITCH_RIGHT)
		htim3.Instance->CCR1 = mtr->pwmValue;
	else if (mtr->type == M_PITCH_LEFT)
		htim3.Instance->CCR3 = mtr->pwmValue;

}

#if USE_RAW_ADC==0
static void motorGetPos(float *pos)
{
	float *tem = pos;

	*tem++ = (float) adcMapping(&mUp, adcValue[0]) / 10.0f;
	*tem++ = (float) adcMapping(&mDown, adcValue[1]) / 10.0f;
	*tem++ = (float) adcMapping(&mRight, adcValue[2]) / 10.0f;
	*tem++ = (float) adcMapping(&mLeft, adcValue[3]) / 10.0f;
}
#endif	//if USE_RAW_ADC==0

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM3)
	{
		/* begin sending log data */
		startSendLogData = SET;
	}
}

void usart1_callback_IT()
{
	uint32_t isrflags = READ_REG(huart1.Instance->SR);
	uint32_t cr1its = READ_REG(huart1.Instance->CR1);
	char c;

	/* RXNE handler */
	if ((isrflags & USART_SR_RXNE) != RESET)
	{
		c = (char) (huart1.Instance->DR & (uint8_t) 0x00FF);
		if (c == CMD_TERMINATOR_CHAR)
			rx1Ready = SET;

		ring_buffer_write(&rx1Buffer, c);
	}

	/* UART in mode Transmitter ------------------------------------------------*/
	if (((isrflags & USART_SR_TXE) != RESET) && ((cr1its & USART_CR1_TXEIE) != RESET))
	{
		/* Check that a Tx process is ongoing */
		if (huart1.gState == HAL_UART_STATE_BUSY_TX)
		{
			huart1.Instance->DR = (uint8_t) (*huart1.pTxBuffPtr++ & (uint8_t) 0x00FF);

			if (--huart1.TxXferCount == 0U)
			{
				/* Disable the UART Transmit Complete Interrupt */
				__HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);

				/* Enable the UART Transmit Complete Interrupt */
				__HAL_UART_ENABLE_IT(&huart1, UART_IT_TC);
			}
		}
	}

	/* UART in mode Transmitter end --------------------------------------------*/
	if (((isrflags & USART_SR_TC) != RESET) && ((cr1its & USART_CR1_TCIE) != RESET))
	{
		/* Disable the UART Transmit Complete Interrupt */
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_TC);

		/* Tx process is ended, restore huart->gState to Ready */
		huart1.gState = HAL_UART_STATE_READY;
		//		HAL_UART_TxCpltCallback(huart1);
	}

	/* ------------------------------------------------------------ */
	/* Other USART1 interrupts handler can go here ...             */
	/* UART in mode Transmitter ------------------------------------------------*/
}

//static void delayIwdg(uint32_t _delayTime)
//{
//	const uint32_t iwdg_reset_time = 100;
//
//	HAL_IWDG_Refresh(&hiwdg);
//	if (_delayTime <= iwdg_reset_time)
//		HAL_Delay(_delayTime);
//	else
//	{
//		while (_delayTime > iwdg_reset_time)
//		{
//			HAL_Delay(iwdg_reset_time);
//			HAL_IWDG_Refresh(&hiwdg);
//			_delayTime -= iwdg_reset_time;
//		}
//		HAL_Delay(_delayTime);
//	}
//	HAL_IWDG_Refresh(&hiwdg);
//}
/* TODO End */
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
