/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "SSD1306.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
const uint8_t degGlyph[16] = {
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x7c, /* 01111100 */
    0xc6, /* 11000110 */
    0xc6, /* 11000110 */
    0xc6, /* 11000110 */
    0xc6, /* 11000110 */
    0x7c, /* 01111100 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
};
const uint8_t heatingGlyph[16] = {
    0xFF, /* 11111111 */
    0xFF, /* 11111111 */
    0x99, /* 10011001 */
    0x99, /* 10011001 */
    0x99, /* 10011001 */
    0x81, /* 10000001 */
    0x81, /* 10000001 */
    0x99, /* 10011001 */
    0x99, /* 10011001 */
    0x99, /* 10011001 */
    0xFF, /* 11111111 */
    0xFF, /* 11111111 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
};

const uint8_t sleepingGlyph[16] = {
    0xFF, /* 11111111 */
    0xFF, /* 11111111 */
    0xC3, /* 11000011 */
    0x99, /* 10011001 */
    0x9F, /* 10011111 */
    0xC7, /* 11000111 */
    0xE3, /* 11100011 */
    0xF9, /* 11111001 */
    0x99, /* 10011001 */
    0xC3, /* 11000011 */
    0xFF, /* 11111111 */
    0xFF, /* 11111111 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
};

const uint8_t readyGlyph[16] = {
    0xFF, /* 11111111 */
    0xFF, /* 11111111 */
    0xC3, /* 11000011 */
    0x99, /* 10011001 */
    0x99, /* 10011001 */
    0x99, /* 10011001 */
    0x83, /* 10000011 */
    0x87, /* 10000111 */
    0x93, /* 10010011 */
    0x99, /* 10011001 */
    0xFF, /* 11111111 */
    0xFF, /* 11111111 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
};

const uint8_t pointerGlyph[16] = {
    0x00, /* 00000000 */
    0x40, /* 01000000 */
    0x60, /* 01100000 */
    0x70, /* 01110000 */
    0x78, /* 01111000 */
    0x7c, /* 01111100 */
    0x7e, /* 01111110 */
    0x7c, /* 01111100 */
    0x78, /* 01111000 */
    0x70, /* 01110000 */
    0x60, /* 01100000 */
    0x40, /* 01000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
    0x00, /* 00000000 */
};

enum {
    GUI_MAIN, GUI_SETTINGS
};
void GUILoop();

float temp;
float dest_temp;
uint32_t GUIState = GUI_MAIN;
uint8_t stateChanged = 1;
uint8_t cursorPosition = 0;
/* USER CODE END 0 */

int main(void) {

    /* USER CODE BEGIN 1 */
    uint32_t adc_val;
    uint32_t adc_dest_temp;
    /* USER CODE END 1 */

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_TIM3_Init();
    MX_ADC2_Init();

    /* USER CODE BEGIN 2 */
    HAL_GPIO_WritePin(Led1_GPIO_Port, Led1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Led2_GPIO_Port, Led2_Pin, GPIO_PIN_RESET);
    HAL_TIM_Base_Start_IT(&htim3);
    SSD1306_init(&hi2c1);
    SSD1306_enable(&hi2c1, 1);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while( 1 ) {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */

        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 100);
        adc_val = HAL_ADC_GetValue(&hadc1);
        HAL_ADC_Stop(&hadc1);

        HAL_ADC_Start(&hadc2);
        HAL_ADC_PollForConversion(&hadc2, 100);
        adc_dest_temp = HAL_ADC_GetValue(&hadc2);
        HAL_ADC_Stop(&hadc2);

        dest_temp = adc_dest_temp / 4029.0f;
        dest_temp *= 350;
        dest_temp += 100;
        float tmp = dest_temp - (int) dest_temp;
        if( tmp > 0.5 )
            dest_temp = (int) dest_temp + 0.5f;
        else
            dest_temp = (int) dest_temp;

        adc_val = 1750 - adc_val;
        temp = adc_val / 4.3f;
        temp += 25.0f;
        GUILoop();

    }
    /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void) {

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV5;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_PLL2;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL2_ON;
    RCC_OscInitStruct.PLL2.PLL2MUL = RCC_PLL2_MUL8;
    RCC_OscInitStruct.PLL2.HSEPrediv2Value = RCC_HSE_PREDIV2_DIV2;
    if( HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK ) {
        Error_Handler();
    }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if( HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK ) {
        Error_Handler();
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if( HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK ) {
        Error_Handler();
    }

    /**Configure the Systick interrupt time 
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick 
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /**Configure the Systick interrupt time 
    */
    __HAL_RCC_PLLI2S_ENABLE();

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void) {

    ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if( HAL_ADC_Init(&hadc1) != HAL_OK ) {
        Error_Handler();
    }

    /**Configure Regular Channel 
    */
    sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    if( HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK ) {
        Error_Handler();
    }

}

/* ADC2 init function */
static void MX_ADC2_Init(void) {

    ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    if( HAL_ADC_Init(&hadc2) != HAL_OK ) {
        Error_Handler();
    }

    /**Configure Regular Channel 
    */
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if( HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK ) {
        Error_Handler();
    }

}

/* I2C1 init function */
static void MX_I2C1_Init(void) {

    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if( HAL_I2C_Init(&hi2c1) != HAL_OK ) {
        Error_Handler();
    }

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 16000;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 4499;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if( HAL_TIM_Base_Init(&htim3) != HAL_OK ) {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if( HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK ) {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if( HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK ) {
        Error_Handler();
    }

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
static void MX_GPIO_Init(void) {

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pins : PE2 PE3 PE4 PE5
                             PE6 PE7 PE8 PE9
                             PE10 PE11 PE12 PE13
                             PE0 PE1 */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5
                          | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9
                          | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13
                          | GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : Button2_Pin JOY_U_Pin JOY_D_Pin JOY_R_Pin
                             JOY_L_Pin JOY_OK_Pin Button1_Pin */
    GPIO_InitStruct.Pin = Button2_Pin | JOY_U_Pin | JOY_D_Pin | JOY_R_Pin
                          | JOY_L_Pin | JOY_OK_Pin | Button1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PC0 PC1 PC2 PC3
                             PC4 PC10 PC11 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                          | GPIO_PIN_4 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA0 PA1 PA2 PA3
                             PA4 PA5 PA6 PA7
                             PA8 PA9 PA10 PA11
                             PA12 PA15 */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                          | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7
                          | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
                          | GPIO_PIN_12 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB1 PB2 PB10 PB11
                             PB12 PB13 PB14 PB15
                             PB3 PB4 PB5 PB8
                             PB9 */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10 | GPIO_PIN_11
                          | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15
                          | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_8
                          | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : Led1_Pin Led2_Pin */
    GPIO_InitStruct.Pin = Led1_Pin | Led2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PD8 PD9 PD10 PD11
                             PD12 PD13 PD14 PD15
                             PD0 PD1 PD2 PD3
                             PD4 PD5 PD6 PD7 */
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
                          | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15
                          | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                          | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, Led1_Pin | Led2_Pin, GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */
void GUILoop() {
    char temp_str[10];

    if( GUIState == GUI_MAIN ) {
        if( stateChanged ) {
            SSD1306_clear(&hi2c1);
            SSD1306_drawGlyph(&hi2c1, 11 * 8, 0, readyGlyph);
            SSD1306_drawGlyph(&hi2c1, 13 * 8, 0, sleepingGlyph);
            SSD1306_drawGlyph(&hi2c1, 15 * 8, 0, heatingGlyph);
            SSD1306_drawString(&hi2c1, 0, 0, "HEATING", 7);

            SSD1306_drawString(&hi2c1, 8, 16, "TEMP: ", 7);
            SSD1306_drawGlyph(&hi2c1, 13 * 8, 16, degGlyph);
            SSD1306_drawChar(&hi2c1, 14 * 8, 16, 'C');

            SSD1306_drawString(&hi2c1, 8, 32, "DEST: ", 7);
            SSD1306_drawGlyph(&hi2c1, 13 * 8, 32, degGlyph);
            SSD1306_drawChar(&hi2c1, 14 * 8, 32, 'C');

            SSD1306_drawString(&hi2c1, 8, 48, "ERR: ", 5);
            SSD1306_drawStringRight(&hi2c1, 14 * 8, 48, "1.2%", 4);
            stateChanged = 0;
        }
        sprintf(temp_str, "%5.1f", temp);
        SSD1306_drawStringRight(&hi2c1, 8 * 12, 16, temp_str, (uint8_t) strlen(temp_str));

        sprintf(temp_str, "%5.1f", dest_temp);
        SSD1306_drawStringRight(&hi2c1, 8 * 12, 32, temp_str, (uint8_t) strlen(temp_str));

        if( HAL_GPIO_ReadPin(JOY_OK_GPIO_Port, JOY_OK_Pin) == GPIO_PIN_RESET ) {
            stateChanged = 1;
            GUIState = GUI_SETTINGS;
        }
    }

    if( GUIState == GUI_SETTINGS ) {
        if( stateChanged ) {
            SSD1306_clear(&hi2c1);
            SSD1306_drawString(&hi2c1, 0, 0, "SETTINGS", 8);
            SSD1306_drawGlyph(&hi2c1, 0, 16, pointerGlyph);
            SSD1306_drawString(&hi2c1, 8, 16, "AUTO CALIBR.", 12);
            SSD1306_drawString(&hi2c1, 8, 32, "MAN. CALIBR.", 12);
            SSD1306_drawString(&hi2c1, 8, 48, "BACK", 4);
            cursorPosition = 0;
            stateChanged = 0;
        }

        if( HAL_GPIO_ReadPin(JOY_L_GPIO_Port, JOY_L_Pin) == GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(JOY_U_GPIO_Port, JOY_U_Pin) == GPIO_PIN_RESET ) {
            if( cursorPosition > 0 ) {
                cursorPosition--;
                SSD1306_drawGlyph(&hi2c1, 0, (1 + cursorPosition) * 16, pointerGlyph);
                SSD1306_drawChar(&hi2c1, 0, (2 + cursorPosition) * 16, ' ');
            }

        }
        if( HAL_GPIO_ReadPin(JOY_R_GPIO_Port, JOY_R_Pin) == GPIO_PIN_RESET ||
            HAL_GPIO_ReadPin(JOY_D_GPIO_Port, JOY_D_Pin) == GPIO_PIN_RESET ) {
            if( cursorPosition < 2 ) {
                cursorPosition++;
                SSD1306_drawGlyph(&hi2c1, 0, (1 + cursorPosition) * 16, pointerGlyph);
                SSD1306_drawChar(&hi2c1, 0, (cursorPosition) * 16, ' ');
            }
        }

        if( HAL_GPIO_ReadPin(JOY_OK_GPIO_Port, JOY_OK_Pin) == GPIO_PIN_RESET ) {
            if( cursorPosition == 2 ) {
                stateChanged = 1;
                GUIState = GUI_MAIN;
            }
        }
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while( 1 ) {
    }
    /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
