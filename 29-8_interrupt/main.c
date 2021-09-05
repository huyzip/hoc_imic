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
#include <string.h>
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t *GPIOA_MODER = (uint32_t*)0x40020000;//GPIO A moder
//uint32_t *GPIOA_IDR = (uint32_t*)0x40020010; // input data register port A
uint32_t *GPIOA_PUPDR = (uint32_t*)0x4002000c; //pull up down register

uint32_t *GPIOD_MODER = (uint32_t*)0x40020C00;//GPIO A moder
uint32_t *GPIOD_OTYPER = (uint32_t*)0x40020C04;//GPIO A otyper
uint32_t *GPIOD_ODR = (uint32_t*)0x40020C14;//output port D (led)

int last_led = 0;
void set_input()
{
	*GPIOA_MODER &= ~(0b11 << 0); //reset bit 2-3 to 0
	*GPIOA_MODER |= (0b00 << 0); // set bit 2-3 to 00 to set PA0 as GPIO input
	*GPIOA_PUPDR &= ~(0b11 << 0);//set no pull
}
void set_output()
{
	*GPIOD_MODER &= ~(0b11111111 << 24); //reset bit 24-31 to 0
	*GPIOD_MODER |= (0b01010101 << 24); // set bit 24-31 to 01 to set PD12-PD15 as GPIO output
	*GPIOD_OTYPER &= ~(0b1111 << 12); //set push-pull output port D
}
void EXTI0_IRQHandler()
{
	//__asm("nop"); // non operation
	if (last_led == 0){
		*GPIOD_ODR |= (0b1 << 13);
		last_led = 1;
	}
	else{
		*GPIOD_ODR &= ~(0b1 << 13);
		last_led = 0;
	}
	uint32_t* EXTI_PR = (uint32_t*)0x40013c14;
	*EXTI_PR |= 1;
}
void Custom_Handler()
{
	if (last_led == 0){
		*GPIOD_ODR |= (0b1 << 13);
		last_led = 1;
	}
	else{
		*GPIOD_ODR &= ~(0b1 << 13);
		last_led = 0;
	}
	uint32_t* EXTI_PR = (uint32_t*)0x40013c14;
	*EXTI_PR |= 1;
}
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
  //MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  set_input(); //set button at PA0
  set_output(); //set led at PD13-PD15

  /* configure EXTI0 (PA0)	*/
  uint32_t* EXTI_RTSR = (uint32_t*)(0x40013c08);
  *EXTI_RTSR |= 1; //rising trigger enable

  uint32_t* EXTI_IMR = (uint32_t*)(0x40013c00);
  *EXTI_IMR |= 1; //set not masked

  /*set NVIC in core ARM*/
  uint32_t* NVIC_ISER0 = (uint32_t*)(0xe000e100);
  *NVIC_ISER0 |= 1<<6;

  /*copy vector table to ram*/
  uint32_t* VTOR = (uint32_t*)0xE000ED08;
  *VTOR = 0x20000000; //ram address

  memcpy(0x20000000, 0x08000000, 384); // Move 384 byte from flash to ram

  uint32_t* px = (uint32_t*)0x20000058;
  *px = (int)Custom_Handler | 1;

  /*set TIMER1 0x40010000*/
//  __HAL_RCC_TIM1_CLK_ENABLE();
//  uint16_t* TIM1_PSC = (uint16_t*)0x40010028;//pre-scale
//  *TIM1_PSC = 3999;// scale 4000
//
//  uint16_t* TIM1_ARR = (uint16_t*)0x4001002c;
//  *TIM1_ARR = 4000;
//
//  uint16_t* TIM1_DIER = (uint16_t*)0x4001000c;
//
//  uint16_t* TIM1_CR1 = (uint16_t*)0x40010000;//control register
//  *TIM1_CR1 |= 1;// set bit0 = 1: counter enable
//
//  uint16_t* TIM1_SR = (uint16_t*)0x40010010;//status register
//  *TIM1_SR = 0;// set 0: reset status
//
//  while((*TIM1_SR & 1) != 0);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

	*GPIOD_ODR |= (0b1 << 12);
	HAL_Delay(500);
	*GPIOD_ODR &= ~(0b1 << 12);
	HAL_Delay(500);
  }
  /* USER CODE BEGIN 3 */

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
