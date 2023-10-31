/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "button.h"
//#include "led.h" - co the se co them thu vien LED
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//khai bao trang thai hieu ung led
typedef enum
{
	LED_BLINK_ORIGIN,
	LED_BLINK_SEQ,
	LED_BLINK_ALTER,
} Led_Effect_Status;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
//---- button handle variable ----
Button_Typedef BTN1, BTN2;

//khai bao bien luu trang thai den
Led_Effect_Status led_eff_state;

//----- UART com debug variable -----
uint8_t Tx_Buffer[100]; //buffer truyen di chua gia tri chu ky hien tai cua LED + che do hien tai cua mode led effect
//---- led_time_cycle [min:max] variable + divine time (pwm)----
uint16_t max_cycle_led_time = 2000, min_cycle_led_time = 0;
uint16_t divine_time_val = 0; //bien dung de luu gia tri chia deu thoi gian hieu ung led (bam xung - pwm thu cong)

//---- Led counter effect variable -----
uint8_t count_led_alt = 0, count_led_seq = 0, count_led_blink = 0;
//---- Led counter effect variable -----
uint8_t long_press_detect = 0;
uint8_t cycle_change_mode = 0; //mode = 0: dec mode = 1 inc

//----timer count ----
uint16_t is_press_time_count = 0; //button press timer count (for longpress support purpose)
uint16_t led_timer_cnt = 0;		//timer count for led trasition time
//--------------------

uint8_t btn1_press_flag = 1;
uint8_t btn2_press_flag = 1;

uint8_t test_flag_twicepress = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void led_blink_origin(void); //effect 1
void led_blink_sequence(void); //effect 2
void led_blink_alternate(void); //effect 3

void button_shortpressing_callback_500ms(Button_Typedef *ButtonX);
void button_longpressing_callback_500ms(Button_Typedef *ButtonX);

void time_cycle_inc_callback(void);
void time_cycle_dec_callback(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void button_shortpressing_callback_500ms(Button_Typedef *ButtonX)
{
	if(ButtonX == &BTN1)
	{
		time_cycle_dec_callback();
	} else if (ButtonX == &BTN2)
	{
		switch(led_eff_state)
			{
				case LED_BLINK_ORIGIN:
					led_eff_state = LED_BLINK_SEQ;
				break;
				case LED_BLINK_SEQ:
					led_eff_state = LED_BLINK_ALTER;
				break;
				case LED_BLINK_ALTER:
					led_eff_state = LED_BLINK_ORIGIN;
				break;
			}
	}
}

void button_longpressing_callback_500ms(Button_Typedef *ButtonX)
{
	if(ButtonX == &BTN1){
		cycle_change_mode = 0;
		long_press_detect = ButtonX->is_long_press;
		btn1_press_flag = 0;
	} else if(ButtonX == &BTN2)
	{
		cycle_change_mode = 1;
		long_press_detect = ButtonX->is_long_press;
		btn2_press_flag = 0;
	}
	if(btn1_press_flag == 0 && btn2_press_flag == 0)
	{
		test_flag_twicepress = 1;
	}
}

void time_cycle_inc_callback(void)
{
	if(max_cycle_led_time == 2000)
	{
		max_cycle_led_time = 0;
	}
	max_cycle_led_time += 100;
}

void time_cycle_dec_callback(void)
{
	if(max_cycle_led_time == 0)
	{
		max_cycle_led_time = 2000;
	}
	max_cycle_led_time -= 100;
}

void led_blink_origin(void) //Callib fnc
{
	if(count_led_blink == 0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		count_led_blink = 1;
	} else if (count_led_blink == 1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		count_led_blink = 0;
	}
}

void led_blink_sequence(void) //Callib fnc
{
	if(count_led_seq == 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		count_led_seq = 1;
	} else if(count_led_seq == 1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		count_led_seq = 2;
	} else if (count_led_seq == 2)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		count_led_seq = 3;
	} else if (count_led_seq == 3)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		count_led_seq = 0;
	}
}

void led_blink_alternate(void) //Callib fnc
{
	if(count_led_alt == 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_SET); //off
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_RESET); //on
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_SET); //off
		count_led_alt = 1;
	} else if (count_led_alt == 1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3,GPIO_PIN_RESET); //on
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4,GPIO_PIN_SET); //off
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,GPIO_PIN_RESET); //on
		count_led_alt = 0;
	}
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
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  led_eff_state = LED_BLINK_ORIGIN; //ke tu luc chuong trinh bat dau se vao hieu ung blink led

  button_Init(&BTN1, GPIOB, GPIO_PIN_0); //BTN1
  button_Init(&BTN2, GPIOB,GPIO_PIN_1); //BTN2
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	//----- Phan code test debug UART gui den man hinh OLED (Uncomment 4 lines below to debug) ----
	//memset(Tx_Buffer,0,sizeof(Tx_Buffer)); //clear buffer before write
	//sprintf((char*)Tx_Buffer,"\nCYCLE:%u\nMODE:%u",max_cycle_led_time,led_eff_state);
	//HAL_UART_Transmit(&huart1,Tx_Buffer,sizeof(Tx_Buffer), 10);
	//HAL_Delay(500);
	//----- Phan code test debug UART gui den man hinh OLED -----

	//TH1: Ca 2 nut deu chua duoc nhan -> Kiem tra lien tuc xem co nut nao nhan khong
	//-> Khong quay lai TH1, co kiem tra xem co nut bam nao duoc bat.
	//Co nut bam 1
	//Xu ly nut bam 1
	// Co nut bam 2 xu ly nut bam 2
	//Co 2 nut bam
	// Xu ly 2 nut bam sau khi xu ly xong sau khi detect nut bam duoc nhả thì reset cờ
	//TH2: 1 trong 2 nut duoc nhan
	//-----> Cau hoi lam sao detect duoc trang thai cua nut nhan
	button_handle(&BTN1);
	button_handle(&BTN2);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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

  /** Initializes the CPU, AHB and APB buses clocks
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3599;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = led_TIM_counter;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
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
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 3599;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = led_TIM_counter;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LEDR_Pin|LEDG_Pin|LEDB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : BTN1_Pin BTN2_Pin */
  GPIO_InitStruct.Pin = BTN1_Pin|BTN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDR_Pin LEDG_Pin LEDB_Pin */
  GPIO_InitStruct.Pin = LEDR_Pin|LEDG_Pin|LEDB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //!WARNING: Don't change
{
	if(htim == &htim3) //behavior led function timer
	{
		if(led_timer_cnt > max_cycle_led_time)
		{
			led_timer_cnt = 0; //reset timer when timer over counter value
		} //handle for overflow timer case - temporary solution
		else {
			led_timer_cnt++;
			if(max_cycle_led_time > 0)
			{
				switch(led_eff_state)
				{
					case LED_BLINK_ORIGIN:
					{
						divine_time_val = max_cycle_led_time / 2;
						if(led_timer_cnt == divine_time_val){
							led_blink_origin();
							led_timer_cnt = 0;
						}
					break;
					}
					case LED_BLINK_SEQ:
					{
						divine_time_val = max_cycle_led_time / 4;
						if(led_timer_cnt == divine_time_val){
							led_blink_sequence();
							led_timer_cnt = 0;
						}
						break;
					}
					case LED_BLINK_ALTER:
					{
						divine_time_val = max_cycle_led_time / 2;
						if(led_timer_cnt == divine_time_val){
							led_blink_alternate();
							led_timer_cnt = 0;
						}
						break;
					}
					default:
						break;
				}
			} else if(max_cycle_led_time == 0)
			{
				max_cycle_led_time = 2000; //neu chu ky bi nut bam tuong tac giam ve 0 thi reset lai thoi gian chu ky
			}
		}
	}
	if(htim == &htim4 && long_press_detect == 1)
	{
		if(is_press_time_count++ == 200)
		{
			is_press_time_count = 0;
			if(cycle_change_mode == 0) {
				time_cycle_dec_callback();
			} else if(cycle_change_mode == 1){
				time_cycle_inc_callback();
			}
		}
	}
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
