/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

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
#define CS_HIGH			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12)
#define CS_LOW			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12)

uint8_t Buffer[2];
extern uint8_t rx_data;
extern uint8_t rx_flag;

// [block] [rotate] [shape]
uint8_t BLOCK[7][4][4] = {
		{{0x00, 0x01, 0x03, 0x02}, {0x00, 0x06, 0x03, 0x00}, {0x00, 0x01, 0x03, 0x02}, {0x00, 0x06, 0x03, 0x00}}, //
		{{0x00, 0x03, 0x03, 0x00}, {0x00, 0x03, 0x03, 0x00}, {0x00, 0x03, 0x03, 0x00}, {0x00, 0x03, 0x03, 0x00}}, // ㅁ
		{{0x02, 0x03, 0x01, 0x00}, {0x00, 0x03, 0x06, 0x00}, {0x02, 0x03, 0x01, 0x00}, {0x00, 0x03, 0x06, 0x00}}, //
		{{0x01, 0x01, 0x01, 0x01}, {0x00, 0x00, 0x0F, 0x00}, {0x01, 0x01, 0x01, 0x01}, {0x00, 0x00, 0x0F, 0x00}}, //
		{{0x00, 0x01, 0x07, 0x00}, {0x04, 0x04, 0x06, 0x00}, {0x07, 0x04, 0x00, 0x00}, {0x03, 0x01, 0x01, 0x00}}, // ㄱ
		{{0x00, 0x07, 0x01, 0x00}, {0x00, 0x06, 0x04, 0x04}, {0x00, 0x00, 0x04, 0x07}, {0x00, 0x01, 0x01, 0x03}}, // ㄴ
		{{0x02, 0x03, 0x02, 0x00}, {0x00, 0x07, 0x02, 0x00}, {0x02, 0x06, 0x02, 0x00}, {0x02, 0x07, 0x00, 0x00}}
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

int _write(int file, char* pi, int len)
{
	for(int i=0; i<len; i++)
	{
		LL_USART_TransmitData8(USART3, *(pi+i));
		HAL_Delay(1);
	}

	return len;
}

void max7219Send(uint8_t ADDR, uint8_t Data)
{
	CS_LOW;
	LL_SPI_TransmitData8(SPI2, ADDR);
	LL_SPI_TransmitData8(SPI2, Data);
	CS_HIGH;
}

void max7219SendToAll(uint8_t ADDR, uint8_t data)
{
	CS_LOW;
	for(int i=0; i<4; i++)
	{
		LL_SPI_TransmitData8(SPI2, ADDR);
		LL_SPI_TransmitData8(SPI2, data);
	}
	CS_HIGH;
}

void max7219SendDis(uint8_t ADDR, uint8_t *buf)
{
	uint8_t cnt = 4;
	CS_LOW;
	uint8_t buffer = 0;
	while(cnt)
	{
		buffer = *buf;
		LL_SPI_TransmitData8(SPI2, ADDR);
		LL_SPI_TransmitData8(SPI2, buffer);
		*buf++;
		cnt--;
	}
	CS_HIGH;
}

void max7219Init(void)
{
	max7219SendToAll(0x0B, 0x07);		// Scan Limit
	max7219SendToAll(0x09, 0x00);		// Decode Mode
	max7219SendToAll(0x0C, 0x01);		// ShutDown Register (Normal Operation)
	max7219SendToAll(0x0A, 0x00);		// Intensity (0x00 ~ 0x0F)
	max7219SendToAll(0x0F, 0x00);		// Display (Turns all LED on)
}

void max7219Clear(void)
{
	for(int i=1; i<9; i++)	max7219SendToAll(i, 0);
}


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
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  LL_SPI_Enable(SPI2);
  LL_USART_EnableIT_RXNE(USART3);


  max7219Init();
  HAL_Delay(1000);
  max7219Clear();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("EXAMPLE TETRIS\r\n");


  uint8_t buf[4];
  uint8_t i = 0, digit = 1;
  uint8_t x = 3;

  uint8_t rotate = 0;


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  buf[digit] = BLOCK[6][rotate][0]<<i;	max7219SendDis(x, buf);
	  buf[digit] = BLOCK[6][rotate][1]<<i;	max7219SendDis(x+1, buf);
	  buf[digit] = BLOCK[6][rotate][2]<<i;	max7219SendDis(x+2, buf);
	  buf[digit] = BLOCK[6][rotate][3]<<i;	max7219SendDis(x+3, buf);
	  HAL_Delay(300);
	  max7219Clear();
	  if(i++ > 5)
	  {
		  i = 0;
	  }
	  HAL_Delay(1);


	  if(rx_flag)
	  {
		  rx_flag = 0;
		  switch(rx_data)
		  {
		  	  case 97:
		  		  x--;
		  		  if(x <= 1 || x >= 250)
		  		  {
		  			  if(BLOCK[6][rotate][0] > 0)
					  {
						  x = 1;
					  }
		  			  else
		  			  {
		  				  x = 0;
		  			  }
		  		  }

		  		  break;
		  	  case 100:
		  		  x++;
		  		  if(x >= 6)
		  		  {
		  			if(BLOCK[6][rotate][3] > 0)
					{
		  				x = 7;
					}
					else
					{
						x = 6;
					}
	  			  }
		  		  break;
		  	  case 119:
		  		  rotate++;
		  		  if(rotate == 4)	rotate = 0;
		  		  break;
		  }
		  printf("x = %d\r\n", x);
	  }
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
