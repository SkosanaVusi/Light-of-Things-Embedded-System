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
int temp;
static int index;
static int isSampling = 0;
uint32_t samples_sent= 0;
char adc_chars[30];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
void EXTI0_1_IRQHandler(void);
void writeLCD(char *char_in);
uint32_t pollADC(void);
uint32_t ADCtoCCR(uint32_t adc_val);
void initGPIO(void);
char *Dec2RadixI(int, int, int);
static void sample(void);
void transmitData(void);

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
    uint32_t CCR = 0;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3); // Start PWM on TIM3 Channel 3
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */


    while (1)
    {
        // Toggle LED0
        HAL_GPIO_TogglePin(GPIOB, LED7_Pin);

        if (isSampling){
        	transmitData();
        }
        else {
            // ADC to LCD; TODO: Read POT1 value and write to LCD
        	adc_val = pollADC();

			sprintf(adc_chars, "POT VALUE: %u", adc_val);
			writeLCD(adc_chars);
        }

        // Update PWM value; TODO: Get CRR
        CCR = ADCtoCCR(adc_val);
        __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, CCR);

        // Wait for delay
        HAL_Delay(500);
    	GPIOA->ODR = 0b0000000000000000; // SWITCHES THE LED OF WHEN DONE SENDING DATA
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
    }
    /* USER CODE END 3 */
}

// This function initializes the GPIO pins of the microcontroller.
void initGPIO(void)
{
    // Enable the clock for the GPIOA and GPIOB peripheral.
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    // Set the GPIOB pins as output.
    GPIOA->MODER |= 0b00000000000000000000000100000000;
    // Set the initial value of the GPIOB pins as low.
    GPIOA->ODR = 0b0000000000000000;
}

void transmitData(void){
	//TODO: HINT THE RECEIVER FOR DATA & SAMPLES COUNT TRANSMISSION THEN SEND THE 2 MESSAGES
	samples_sent++;
	sprintf(adc_chars, "SEND POT: %u", adc_val);
	writeLCD(adc_chars);

	char *ptrADC = Dec2RadixI(adc_val, 2, 12);
	char *ptrSamples = Dec2RadixI(samples_sent, 2, 12);
	lcd_command(0xC0); // 0xC0 is the command to set the cursor to the start of the second line.
	sprintf(adc_chars, "BI: %s", ptrADC);
	lcd_putstring(adc_chars);

	GPIOA->ODR = 0b0000000000010000; // DATA TRANSMISSION INDICATOR TO SEND ADC MESSAGE

	for (int i = 0; i < 24; i++)
	{
		HAL_Delay(500);

		if (i < 12){
			// Check if the element is 0
			if (*(ptrADC + i) == '0')
			{
				GPIOA->ODR = 0b0000000000000000;
			}
			else
			{
				GPIOA->ODR = 0b0000000000010000;
			}
		}
		else{
			// Check if the element is 0

			if ((i - 12) == 0)
			{
				sprintf(adc_chars, "CHECKPOINT: %u", samples_sent);
				writeLCD(adc_chars);

				lcd_command(0xC0); // 0xC0 is the command to set the cursor to the start of the second line.
				sprintf(adc_chars, "BI: %s", ptrSamples);
				lcd_putstring(adc_chars);
			}

			if (*(ptrSamples + (i-12)) == '0')
			{
				GPIOA->ODR = 0b0000000000000000;
			}
			else
			{
				GPIOA->ODR = 0b0000000000010000;
			}
		}

	}

	//            delay(1000); // GIVE THE RECIEVER TIME TO READ THE LAST BIT SENT BY THE TRANSMITTER
	sample();
	writeLCD("SENDING COMPLETE");
	lcd_command(0xC0); // 0xC0 is the command to set the cursor to the start of the second line.
	sprintf(adc_chars, "SAMPLES SENT: %u", samples_sent);
	lcd_putstring(adc_chars);


	HAL_Delay(3000); // ALLOW THE OBSERVER TO SEE TRANSMISSION IS COMPLETE AND HOW MANY SAMPLES WERE SENT
}


char *Dec2RadixI(int decimal, int radix, int minLength)
{
    index = 0;
    int remainder;

    // Stores the radix value as a list of characters stored backwards,
    // made static so the array does not decay when returning its address
    static char convert[100];

    // Check if the inputted value is 0
    if (decimal == 0)
    {
        convert[index] = '0';
        index += 1;
    }
    else
    {
        while (decimal > 0)
        {
            remainder = decimal % radix;

            // If remainder is between 10 - 15 inclusive, store the respective letter A-F,
            // else store 0 - 9
            if (remainder >= 10)
            {
                // Convert to the respective letter A-F
                convert[index] = 'A' + (remainder - 10);
            }
            else
            {
                // +48 to make sure the integer values get converted to the correct ASCII equivalent
                convert[index] = remainder + 48;
            }

            decimal /= radix;
            index += 1;
        }
    }

    // Reverse the characters in the `convert` array and pad with zeros at the beginning
    int totalLength = index < minLength ? minLength : index;
    char *reversed = (char *)malloc(totalLength + 1); // Allocate memory for the reversed string
    if (reversed == NULL)
    {
        // Handle memory allocation error
        return NULL;
    }

    // Pad with zeros at the beginning
    for (int i = 0; i < totalLength - index; i++)
    {
        reversed[i] = '0';
    }

    int j = totalLength - index;
    for (int i = index - 1; i >= 0; i--)
    {
        reversed[j] = convert[i]; // Copy characters in reverse order
        j++;
    }

    // Null-terminate the reversed string
    reversed[totalLength] = '\0';

    // Returning the reversed array address
    return reversed;
}
static void sample(void)
{
    isSampling = !isSampling;
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
    // TODO: Add code to switch LED7 delay frequency
    curr_millis = HAL_GetTick();

    if (curr_millis - prev_millis >= 500)
    {
        /* ALTERNATE DELAY IN ARRAY */
        temp = delays[0];
        delays[0] = delays[1];
        delays[1] = temp;

        // Swap LED frequency use first element in array of delays.
        delay_t = delays[0];
        prev_millis = curr_millis;

        // Call the sample() function when switch 0 is pressed.
        sample();
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
    // write current readings to LCD
    lcd_putstring(char_in);
}

// Get ADC value
uint32_t pollADC(void)
{
    // TODO: Complete function body to get ADC val

    // initialize the sampling and resolution configurations
    HAL_ADC_Start(&hadc);
    // Read the ADC value of the analog conversion
    uint32_t val = HAL_ADC_GetValue(&hadc);
    // stop sampling
    HAL_ADC_Stop(&hadc);

    return val;
}

// Calculate PWM CCR value
//uint32_t ADCtoCCR(uint32_t adc_val)
//{
//    // TODO: Calculate CCR val using an appropriate equation
//    uint32_t val = (adc_val * (47999)) / 4096;
//    return val;
//}
uint32_t ADCtoCCR(uint32_t adc_val)
{
    // Calculate the voltage based on ADC value and desired range
    float voltage = ((float)adc_val / 4096) * (0.0f) + 0.0f;

    // Calculate the CCR value based on the voltage and timer settings
    uint32_t ccr_value = (uint32_t)((voltage / 0.01f) * htim3.Init.Period);

    return ccr_value;
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
