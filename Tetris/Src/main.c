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

// ------------------------ 도트매트릭스 CS PIN ----------------------------//
#define CS_HIGH			LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12)
#define CS_LOW			LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_12)
// -------------------------------------------------------------------- //


uint8_t digit[4];
uint8_t buf[8];

// ------------------------ 게임 관련 변수 ------------------------ //
uint8_t right_shock_flag = 0;	// 오른쪽 충돌 flag
uint8_t left_shock_flag = 0;	// 왼쪽 충돌 flag
uint8_t down_shock_flag = 0;
uint8_t digit_num = 1;		// 초기 도트매트릭스 위치
uint8_t x = 3, y = 0;		// 초기 상태 x축, y축 위치
uint8_t rotate = 0;			// 초기 블럭 회전상태
uint8_t newBlockNum = 0;	// 새로운 블럭


// ------------------------ USART 출력 변수 ------------------------ //
extern uint8_t rx_data;
extern uint8_t rx_flag;
// ------------------------------------------------------------- //

//------------------------ 게임 판 현재 상태 저장 ---------------------- //
uint8_t playPlace[16][8] = {
	{0x80, 0, 0, 0, 0, 0, 0, 0x80},		// DIGIT 1
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0x01, 0, 0, 0, 0, 0, 0, 0x01},
	{0, 0, 0, 0, 0, 0, 0, 0},		// DIGIT 0, 1
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0, 0, 0, 0, 0, 0, 0, 0},
	{0x01, 0, 0, 0, 0, 0, 0, 0x01}
};

// ------------------------ [블럭모양] [위치] 임시 저장 변수 ------------------------ //
uint8_t blockCopy[4][9];

// ------------------------ [블럭모양] [회전번호] [위치] 블럭 저장 ------------------------ //
uint8_t BLOCK[7][4][9] = {
	{
		{	0x01, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x01, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	}
	}, //
	{
		{	0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x03, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	}
	},//
	{
		{	0x02, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x02, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x03, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	}
	},//
	{
		{	0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00	},
		{	0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00	},
		{	0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	}
	},//
	{
		{	0x01, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x02, 0x02, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x07, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x03, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00	}
	}, // ?��
	{
		{	0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x03, 0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x04, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00	}
	}, // ?��
	{
		{	0x02, 0x03, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x07, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x02, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00	},
		{	0x02, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00	}
	}
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// ------------------------ printf 사용 ------------------------ //
int _write(int file, char* pi, int len)
{
	for(int i=0; i<len; i++)
	{
		LL_USART_TransmitData8(USART3, *(pi+i));
		HAL_Delay(1);
	}

	return len;
}

// ------------------------ MAX7219 사용 함수 START ------------------------ //
// MAX7219 Initiallize
void max7219Init(void)
{
	max7219SendToAll(0x0B, 0x07);		// Scan Limit
	max7219SendToAll(0x09, 0x00);		// Decode Mode
	max7219SendToAll(0x0C, 0x01);		// ShutDown Register (Normal Operation)
	max7219SendToAll(0x0A, 0x00);		// Intensity (0x00 ~ 0x0F)
	max7219SendToAll(0x0F, 0x00);		// Display (Turns all LED on)
}

// MAX7219 Clear
void max7219Clear(void)
{
	for(int i=1; i<9; i++)		max7219SendToAll(i, 0);
}

// MAX7219 Send Data (one shot)
void max7219Send(uint8_t ADDR, uint8_t Data)
{
	CS_LOW;
	LL_SPI_TransmitData8(SPI2, ADDR);
	LL_SPI_TransmitData8(SPI2, Data);
	CS_HIGH;
}

// MAX7219 Send Data (Multi shot : 총 4개)
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

// MAX7219 Send Data (특정 데이터를 원하는 위치에 출력하기 위함)
// *buf는 buf[<Digit 위치>] = Data
void max7219SendDataPos(uint8_t ADDR, uint8_t *buf)
{
	uint8_t buffer = 0;
	CS_LOW;
	for(int i=0; i<4; i++)
	{
		buffer = *buf;
		LL_SPI_TransmitData8(SPI2, ADDR);
		LL_SPI_TransmitData8(SPI2, buffer);
		*buf++;
	}
	CS_HIGH;
}
// ------------------------ MAX7219 사용 함수 END ------------------------ //

// ------------------------ 테트리스 Play 관련 함수 ------------------------ //
//현재 전체 디스플레이
void playPlaceDisplay(void)
{
	  // 적재된 블럭포함한 전체 놀이판 디스플레이
	  for(int i=0; i<8; i++)
	  {
		  for(int j=0; j<16; j++)	  digit[j/8] |= 0xFF & playPlace[j][i];
	  	  max7219SendDataPos(i+1, digit);
	  	  digit[0] = 0;
	  	  digit[1] = 0;
	  	  digit[2] = 0;
	  	  digit[3] = 0;
	  }
}

// 좌, 우측 충돌 체크
void shockCheck(uint8_t dir)
{
	if(dir == 1)		// 왼쪽 체크
	{
		if(buf[digit_num] > 0)	left_shock_flag = 1;
		else					left_shock_flag = 0;
//			  printf("left:%d, right:%d\n", left_shock_flag, right_shock_flag);
	}
	else if(dir == 0)				// 오른쪽 체크
	{
		if(buf[digit_num] > 0)	right_shock_flag = 1;
		else					right_shock_flag = 0;
//			  printf("left:%d, right:%d\n", left_shock_flag, right_shock_flag);
	}
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
  uint8_t temp_y = 0;
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // 블럭 이동과 위치 디스플레이
	  for(int i=0; i<9; i++)
	  {
		  uint8_t a;

		  int k = i-x;			// x축 이동 변수
		  if(k < 0)	k = -1;		// 0보다 작을때 unsigned -> 255가 되는 현상을 방지하기 위한 조치
		  blockCopy[rotate][i] = BLOCK[newBlockNum][rotate][i];		//현재 생성된 블럭을 복사

		  buf[digit_num] = blockCopy[rotate][k]<<y;		// 출력할 buf에 블럭을 복사 + y축

		  if(i == 0)	shockCheck(1);		// x축 좌측 충돌 확인
		  if(i == 7)	shockCheck(0);		// x축 우측 충돌 확인
		  if(i == 8)
		  {
			  a = i-x;
			  while(buf[digit_num] != 0)	// 회전하여 넘어간 경우
			  {
				  for(int i=0; i<9; i++)
				  {
					  buf[digit_num] = blockCopy[rotate][i-a]<<y;
				  }
				  x--;
			  }
		  }
		  if(buf[digit_num] > 128)
		  {
			  buf[digit_num-1] = blockCopy[rotate][k]>>(temp_y<<1);		// 블럭을 하단 블럭으로 이동
		  }

		  printf("buf=%d    y=%d    ", buf[digit_num], y);

		  max7219SendDataPos(i+1, buf);

		  buf[0] = 0;
		  buf[1] = 0;
		  buf[2] = 0;
		  buf[3] = 0;

		  HAL_Delay(30);
	  }
	  printf("\n");

	  // 키보드를 이용한 블럭 위치 조정
	  if(rx_flag)
	  {
		  max7219Clear();
		  rx_flag = 0;
		  switch(rx_data)
		  {
		  	  case 97:	if(!left_shock_flag)		x--;		break;	// 좌측 이동
		  	  case 100:	if(!right_shock_flag)		x++;		break;	// 우측 이동
		  	  case 101:	rotate++;  if(rotate == 4)	rotate = 0;	break;	// 회전
		  	  case 119:	y--;									break;	// 위 이동
		  	  case 115:	y++;									break;	// 아래 이동
		  	  case 113:	for(int i=0; i<8; i++)	blockCopy[rotate][i] = 0;  newBlockNum++;
		  	  	  	  	break;	//블럭 변경
		  	  default:	break;
		  }
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
