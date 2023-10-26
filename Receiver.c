/* USER CODE BEGIN Header */
/**
  **
  * @file           : main.c
  * @brief          : Main program body
  **
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  **
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "stm32f0xx.h"
#include <lcd_stm32f0.c>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
uint32_t prev_millis = 0;
uint32_t curr_millis = 0;
uint32_t delay_t = 500; // Initialise delay to 500ms
uint32_t adc_val;
int delays[] = {500, 1000}; // STORING TWO PERIODS
static uint8_t patterns[] = {0b01110000, 0b00001110};
int tempDelay;
static int isReceiving = 0; // FLAG FOR CHECKING IF THE RECEIVER IS SET TO RECEIVE MODE
int isFirstOne = 0; // FLAG TO CHECK FOR THE TRANSMISSION INDICATOR
int index = 0; // KEEPS TRACK OF THE NUMBER OF BITS OF DATA SENT
uint32_t recv_samples = 0; // SAMPLE COUNT THAT IS RECORED BY THE RECEIVER
uint32_t tran_samples; // SAMPLE COUNT THAT IS SENT BY THE TRANSMITTER

char received_bits[30]; // STORES THE 24 BITS OF DATA RECEIVED FROM THE TRANSMITTER
uint32_t pot_value; // STORES THE CONVERTED DECIMAL EQUIVALENT OF THE 1ST 12-BITS SENT BINARY VALUE
char temp[20];

int isWaiting = 1; // FLAG FOR IDLE MODE BUT READY TO RECEIVE
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
void writeLCD(char *char_in);
void initGPIO(void);
void receive(void);
void receiveData(void);
void interpretData(void);

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
    initGPIO();
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
    MX_ADC_Init();
    MX_TIM3_Init();

    /* USER CODE BEGIN 2 */
    init_LCD();

    // PWM setup
//    uint32_t CCR = 0;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Start PWM on TIM3 Channel 3
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */


	writeLCD("PRESS SW0");
	lcd_command(0xC0);
	lcd_putstring("TO RECEIVE :)");

    while (1)
    {
    	// TODO: AFTER READING 24 BITS OF DATA INTERPRET IT
		if (index == 24){
			interpretData();
		}

		// TODO: CHECKS IF THE RECIEVER WAS SET TO RECEIVE MODE BY USING THE SW0
		if (isReceiving){
			receiveData();
		}

    }
    /* USER CODE END 3 */
}

// This function initializes the GPIO pins of the microcontroller.
void initGPIO(void)
{
    // Enable the clock for the GPIOA and GPIOB peripheral.
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

    GPIOA->MODER |= 0b00000000000000000000000000000000;

    // Set the initial value of the GPIOB pins as low.
    GPIOA->ODR = 0b0000000000000000;
}

void receive(void)
{
    isReceiving = !isReceiving;
    GPIOA->IDR &= ~16; // THE DEFAULT VALUE = 8609
    lcd_command(CLEAR);
}

void receiveData(void){
	if(isWaiting){
		isWaiting = !isWaiting;
		writeLCD("AWAITING");
		lcd_command(0xC0);
		lcd_putstring("TRANSMISSION...");
	}

	//	TODO: CHECKS IF THE TRANSMITTER IS ABOUT TO SEND DATA ELSE RECEIVER REMIANS IDLE BUT READY TO RECEIVE MESSAGES
	if(((GPIOA->IDR & 16) == 16) && (isFirstOne == 0)){
		isFirstOne = 1;
		recv_samples++;
	}

	// TODO: THE RECIEVER ONLY STARTS READING DATA WHEN THE TRANSMISSION INDICATOR HAS BEEN FIRED
	else if (isFirstOne && (index < 12)){
		if (index == 0){
			writeLCD("GETTING DATA...");
			lcd_command(0xc0);
		}

		// TODO: TEMPORARILY SETTING OTHER PINS TO ZERO EXCEPT THE ONE WE ARE READING FROM, BUT THE VALUE IS STILL ONE [  00100001101*0001 = 00000000000*0000;  ];
		if((GPIOA->IDR & 16) == 16){
			received_bits[index] = '1';
			index++;
			lcd_putstring("1");
		}
		else{
			received_bits[index] = '0';
			index++;
			lcd_putstring("0");
		}
	}

	// TODO: READ THE 2ND 12-BITs OF DATA SENT
	else if (isFirstOne && (index > 11))
	{
		if (index == 12){
			writeLCD("GETTING SAMPLES:");
			lcd_command(0xc0);
		}

		if((GPIOA->IDR & 16) == 16){
			received_bits[index] = '1';
			index++;
			lcd_putstring("1");
		}
		else{
			received_bits[index] = '0';
			index++;
			lcd_putstring("0");
		}
	}

	HAL_Delay(500);
}

void interpretData(void){
	receive(); 		// WILL RESET THE SO THAT WE STOP RECIEVING DATA

	pot_value = 0;
	tran_samples = 0;

	// TODO: CONVERT THE 1ST 12-BITS TO GET THE POTENTIOMETER DECIMAL VALUE
	for (int i = 0; i < 12; i++) {
		int bit = received_bits[i] - 48;
		pot_value += (uint32_t)pow(2, 11 - i) * bit;
	}

	// TODO: CONVERT THE 2ND 12-BITS TO GET THE SAMPLE COUNT DECIMAL VALUE
	for (int i = 12; i < 24; i++) {
		int bit = received_bits[i] - 48;
		tran_samples += (uint32_t)pow(2, 23 - i) * bit;
	}

	sprintf(temp, "POT VALUE: %u", pot_value);
	writeLCD(temp);

	// TODO: CHECK IF THE SAMPLE COUNT VALUE RECORDED BY THE RECEIVER AND THE ONE SENT BY TRANSMITTER CORRESPOND
	if (recv_samples != tran_samples){
		GPIOB -> ODR = patterns[0];
		recv_samples = tran_samples;
		HAL_Delay(1000);
		GPIOB -> ODR = patterns[1];
		HAL_Delay(1000);
		GPIOB -> ODR = 0;
	}

	sprintf(temp, "SAMPLE GOT: %u", recv_samples);
	lcd_command(0xC0);
	lcd_putstring(temp);

	// RESET THE BITS READ INDEX AND RECEIVER INDICATOR RESPECTIVELY
	index = 0;
	isFirstOne = 0;

	// RESET THE RECEIVER TO IDLE MODE BUT READY TO RECEIVE
	isWaiting = 1;

	HAL_Delay(3000);

	writeLCD("PRESS SW0");
	lcd_command(0xC0);
	lcd_putstring("TO RECEIVE :)");
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
    while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_0)
    {
    }
    LL_RCC_HSI_Enable();

    /* Wait till HSI is ready */
    while (LL_RCC_HSI_IsReady() != 1)
    {
    }
    LL_RCC_HSI_SetCalibTrimming(16);
    LL_RCC_HSI14_Enable();

    /* Wait till HSI14 is ready */
    while (LL_RCC_HSI14_IsReady() != 1)
    {
    }
    LL_RCC_HSI14_SetCalibTrimming(16);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

    /* Wait till System clock is ready */
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
    {
    }
    LL_SetSystemCoreClock(8000000);

    /* Update the time base */
    if (HAL_InitTick(TICK_INT_PRIORITY) != HAL_OK)
    {
        Error_Handler();
    }
    LL_RCC_HSI14_EnableADCControl();
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

    /* USER CODE BEGIN ADC_Init 0 */
    /* USER CODE END ADC_Init 0 */

    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC_Init 1 */

    /* USER CODE END ADC_Init 1 */

    /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
    hadc.Instance = ADC1;
    hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc.Init.Resolution = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
    hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait = DISABLE;
    hadc.Init.LowPowerAutoPowerOff = DISABLE;
    hadc.Init.ContinuousConvMode = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.DMAContinuousRequests = DISABLE;
    hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure for the selected ADC regular channel to be converted.
  */
    sConfig.Channel = ADC_CHANNEL_6;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC_Init 2 */
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL)
        ;                 // Calibrate the ADC
    ADC1->CR |= (1 << 0); // Enable ADC
    while ((ADC1->ISR & (1 << 0)) == 0)
        ; // Wait for ADC ready
          /* USER CODE END ADC_Init 2 */
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */

static void MX_TIM3_Init(void)
{
    /* USER CODE BEGIN TIM3_Init 0 */

    /* USER CODE END TIM3_Init 0 */

    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    /* USER CODE BEGIN TIM3_Init 1 */

    /* USER CODE END TIM3_Init 1 */
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 47999;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN TIM3_Init 2 */

    /* USER CODE END TIM3_Init 2 */
    HAL_TIM_MspPostInit(&htim3);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    LL_EXTI_InitTypeDef EXTI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOF);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    /**/
    LL_GPIO_ResetOutputPin(LED0_GPIO_Port, LED0_Pin);
    /**/
    LL_GPIO_ResetOutputPin(LED1_GPIO_Port, LED1_Pin);
    /**/
    LL_GPIO_ResetOutputPin(LED2_GPIO_Port, LED2_Pin);
    /**/
    LL_GPIO_ResetOutputPin(LED3_GPIO_Port, LED3_Pin);
    /**/
    LL_GPIO_ResetOutputPin(LED4_GPIO_Port, LED4_Pin);
    /**/
    LL_GPIO_ResetOutputPin(LED5_GPIO_Port, LED5_Pin);
    /**/
    LL_GPIO_ResetOutputPin(LED6_GPIO_Port, LED6_Pin);
    /**/
    LL_GPIO_ResetOutputPin(LED7_GPIO_Port, LED7_Pin);

    /**/
    LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTA, LL_SYSCFG_EXTI_LINE0);

    /**/
    LL_GPIO_SetPinPull(Button0_GPIO_Port, Button0_Pin, LL_GPIO_PULL_UP);

    /**/
    LL_GPIO_SetPinMode(Button0_GPIO_Port, Button0_Pin, LL_GPIO_MODE_INPUT);

    /**/
    EXTI_InitStruct.Line_0_31 = LL_EXTI_LINE_0;
    EXTI_InitStruct.LineCommand = ENABLE;
    EXTI_InitStruct.Mode = LL_EXTI_MODE_IT;
    EXTI_InitStruct.Trigger = LL_EXTI_TRIGGER_RISING;
    LL_EXTI_Init(&EXTI_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LED0_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED0_GPIO_Port, &GPIO_InitStruct);
    /**/
    GPIO_InitStruct.Pin = LED1_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);
    /**/
    GPIO_InitStruct.Pin = LED2_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);
    /**/
    GPIO_InitStruct.Pin = LED3_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED3_GPIO_Port, &GPIO_InitStruct);
    /**/
    GPIO_InitStruct.Pin = LED4_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED4_GPIO_Port, &GPIO_InitStruct);
    /**/
    GPIO_InitStruct.Pin = LED5_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED5_GPIO_Port, &GPIO_InitStruct);
    /**/
    GPIO_InitStruct.Pin = LED6_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED6_GPIO_Port, &GPIO_InitStruct);

    /**/
    GPIO_InitStruct.Pin = LED7_Pin;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(LED7_GPIO_Port, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
    /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void EXTI0_1_IRQHandler(void)
{
    // TODO: CODE TO HANDLE THE DEBOUNCING
    curr_millis = HAL_GetTick();

    if (curr_millis - prev_millis >= 500)
    {
        prev_millis = curr_millis;

        // Call the sample() function when switch 0 is pressed.
        receive();
    }

    HAL_GPIO_EXTI_IRQHandler(Button0_Pin);
}

// TODO: Complete the writeLCD function
void writeLCD(char *char_in)
{
    // add a delay to extend time from potentiomer reads then to LCD
    delay(2500);
    // remove what is currently on LCD
    lcd_command(CLEAR);

    // set lcd cursor to line 1
    lcd_command(CURSOR_HOME);
    // write current readings to LCD
    lcd_putstring(char_in);
}

void ADC1_COMP_IRQHandler(void)
{
    adc_val = HAL_ADC_GetValue(&hadc); // read adc value
    HAL_ADC_IRQHandler(&hadc);         //Clear flags
}
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

#ifdef USE_FULL_ASSERT
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

