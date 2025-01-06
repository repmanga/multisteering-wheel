/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PILOT_FINGER_TAP_SPEED 150
// #define DEBUG 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
/* Some buffers for CAN MSGs */
/* Value update frequency 20 Hz */
uint8_t TxData[8] = {0,0,0,0,0,0,0,0};      // Output buffer
uint8_t RxData_x600[8] = {0,0,0,0,0,0,0,0}; // (ID 0x600)
//0x600 {0_RPM, 1_RPM, 2_TPS, 3_IAT, 4_MAP, 5_MAP, 6_INJPW, 7_INJPW}
uint8_t RxData_x601[8] = {0,0,0,0,0,0,0,0}; // (ID 0x601)
//0x601 {0_AIN1, 1_AIN1, 2_AIN2, 3_AIN2, 4_AIN3, 5_AIN3,6_AIN4, 7_AIN4}
uint8_t RxData_x602[8] = {0,0,0,0,0,0,0,0}; // (ID 0x602)
//0x602 {0_VSPD, 1_VSPD, 2_BARO, 3_OILT, 4_OILP, 5_FUELP, 6_CLT, 7_CLT}
uint8_t RxData_x604[8] = {0,0,0,0,0,0,0,0}; // (ID 0x604)
//0x604 {0_GEAR, 1_ECUTEMP, 2_BATT, 3_BATT, 4_ERRFLAG, 5_ERRFLAG, 6_FLAGS1, 7_ETHANOL}
/* Value conversion on Nextion side */
uint32_t TxMailbox;
uint8_t msg_type = 255;
uint16_t RPM, MAP, AIN0, AIN1, AIN2, AIN3, AIN4, VSPD, CLT, BATT, ERRFLAG = 0;
uint8_t TPS, IAT, BARO, OILT, OILP, FUELP, GEAR, ECUTEMP = 0;
uint8_t flag_btn1, flag_btn2, flag_btn3, flag_btn4, flag_btn5, flag_btn6 = 0; // Some flags for buttons
uint32_t time_ms = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

/* USER CODE BEGIN PFP */
void button_handler(void);
int can_msg_handler(uint8_t typemsg);
void nextion_msg_handler(void);
void startup(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	Error_Handler();
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
  MX_CAN_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  startup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	button_handler();
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
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
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
  CAN_FilterTypeDef sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = 0x600;
  sFilterConfig.FilterIdLow = 0x600;
  sFilterConfig.FilterMaskIdHigh = 0x7F8<<5;
  sFilterConfig.FilterMaskIdLow = 0x7F8<<5;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  //sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
  Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CAN_LED_Pin */
  GPIO_InitStruct.Pin = CAN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_6_Pin */
  GPIO_InitStruct.Pin = BTN_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_1_Pin BTN_2_Pin BTN_3_Pin BTN_4_Pin */
  GPIO_InitStruct.Pin = BTN_1_Pin|BTN_2_Pin|BTN_3_Pin|BTN_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BTN_5_Pin */
  GPIO_InitStruct.Pin = BTN_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BTN_5_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void button_handler()
{
	  /* ENGINE STARTUP BUTTON HANDLER */
	  if (HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) && (HAL_GetTick() - time_ms > 150) && flag_btn1 == 0) {
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		  flag_btn1 = 1;
		  while(HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin)){
		  		  /* SEND CAN MSG ENGINE STARTUP HERE */
			  msg_type = 0;
			  while(can_msg_handler(msg_type));
		  		  /* ENGINE STARTUP SWITCH IS NOT LATCHING ! */
		  }
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  	}
	  if (!HAL_GPIO_ReadPin(BTN_1_GPIO_Port, BTN_1_Pin) && flag_btn1 == 1) {
	 		  flag_btn1 = 0;
	 		  HAL_Delay(100);
	 	}
	  /* ENGINE STOP BUTTON HANDLER */
	  if (HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) && (HAL_GetTick() - time_ms > 150) && flag_btn2 == 0) {
		  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
		  flag_btn2 = 1;
		  	  /* SEND CAN STOP ENGINE MSG HERE */
		  	  HAL_Delay(100);
		  	  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  	}
	  if (!HAL_GPIO_ReadPin(BTN_2_GPIO_Port, BTN_2_Pin) && flag_btn2 == 1) {
		  flag_btn2 = 0;
	 		  HAL_Delay(100);
	 	}
	  /* NEUTRAL GEAR BUTTON COMBINATION HANDLER */
	  HAL_Delay(PILOT_FINGER_TAP_SPEED);
	  if (HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) && HAL_GPIO_ReadPin(BTN_4_GPIO_Port, BTN_4_Pin) && (HAL_GetTick() - time_ms > 150) && flag_btn3 == 0 && flag_btn4 == 0){
	 		  flag_btn3 = 1;
	 		  flag_btn4 = 1;
	 		  /* SEND CAN NEUTRAL GEAR MSG HERE */
		  	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		  	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		  	  HAL_Delay(100);
		  	  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		  	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	  }
	  if (!HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) && !HAL_GPIO_ReadPin(BTN_4_GPIO_Port, BTN_4_Pin) && flag_btn3 == 1 && flag_btn4 == 1) {
	  	 		  flag_btn3 = 0;
	  	 		  flag_btn4 = 0;
	  	 		  HAL_Delay(100);
	  	 	}
	  /* GEAR UP BUTTON HANDLER */
	  if (HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) && !HAL_GPIO_ReadPin(BTN_4_GPIO_Port, BTN_4_Pin) && (HAL_GetTick() - time_ms > 150) && flag_btn3 == 0) {
		  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
		  flag_btn3 = 1;
		  	  /* SEND CAN GEAR UP MSG HERE */

		  HAL_Delay(100);
		  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  	}
	  if (!HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin) && flag_btn3 == 1) {
	 		  flag_btn3 = 0;
	 		  HAL_Delay(100);
	 	}
	  /* GEAR DOWN BUTTON HANDLER */
	  if (HAL_GPIO_ReadPin(BTN_4_GPIO_Port, BTN_4_Pin) && !HAL_GPIO_ReadPin(BTN_3_GPIO_Port, BTN_3_Pin)  && (HAL_GetTick() - time_ms > 150) && flag_btn4 == 0) {
		      HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
		  	  flag_btn4 = 1;
		  	  /* SEND CAN GEAR DOWN MSG HERE */

		  	  HAL_Delay(100);
		  	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	  	}
	  if (!HAL_GPIO_ReadPin(BTN_4_GPIO_Port, BTN_4_Pin) && flag_btn4 == 1) {
	 		  flag_btn4 = 0;
	 		  HAL_Delay(100);
	 	}
	  /* NEXT SCREEN BUTTON HANDLER */
	  if (HAL_GPIO_ReadPin(BTN_5_GPIO_Port, BTN_5_Pin) && (HAL_GetTick() - time_ms > 150) && flag_btn5 == 0) {
		  	  flag_btn5 = 1;
		  	  /* SEND USART NEXT SCREEN MSG HERE */

		  	  HAL_Delay(100);
	  	}
	  if (!HAL_GPIO_ReadPin(BTN_5_GPIO_Port, BTN_5_Pin) && flag_btn5 == 1) {
	 		  flag_btn5 = 0;
	 		  //HAL_Delay(100);
	 	}
	  /* PREVIOUS SCREEN BUTTON HANDLER */
	  if (HAL_GPIO_ReadPin(BTN_6_GPIO_Port, BTN_6_Pin) && (HAL_GetTick() - time_ms > 150) && flag_btn6 == 0) {
		  	  flag_btn6 = 1;
		  	  /* SEND USART PREVIOUS SCREEN MSG HERE */

		  	  HAL_Delay(100);
	  	}
	  if (!HAL_GPIO_ReadPin(BTN_6_GPIO_Port, BTN_6_Pin) && flag_btn6 == 1) {
	 		  flag_btn6 = 0;
	 		  //HAL_Delay(100);
	 	}

}
int can_msg_handler(uint8_t typemsg){
	switch (typemsg) {
		case 0:
	/* MSG START ENGINE */
	TxHeader.StdId = 0x642;
	TxData[4] = 0b00000001; //using binary system to make bit set more clear
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0); //CAN SW#0
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, TxMailbox);
	TxData[4] = 0x00;
			break;
		case 1:
	/* MSG STOP ENGINE */
	TxHeader.StdId = 0x642;
	TxData[4] = 0b00000010; //CAN SW#1
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, TxMailbox);
	TxData[4] = 0x00;
			break;
		case 2:
	/* MSG GEAR UP */
	TxHeader.StdId = 0x642;
	TxData[4] = 0b00000100; //CAN SW#2
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, TxMailbox);
	TxData[4] = 0x00;
			break;
		case 3:
	/* MSG GEAR DOWN */
	TxHeader.StdId = 0x642;
	TxData[4] = 0b00001000; //CAN SW #3
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, TxMailbox);
	TxData[4] = 0x00;
			break;
		case 4:
	/* MSG GEAR NEUTRAL */
	TxHeader.StdId = 0x642;
	TxData[4] = 0b00010000; //CAN SW#4
	while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
	HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, TxMailbox);
	TxData[4] = 0x00;
			break;
		default:
	/* MSG NONE */
			break;
		msg_type = 255; // SET NONE TYPE MSG
		return 0; // return OK value to prevent endless loop
	}
}
void startup(){
	 HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, 0);
	  TxHeader.StdId = 0x642;
	  TxHeader.ExtId = 0;
	  TxHeader.RTR = CAN_RTR_DATA; // CAN_RTR_REMOTE
	  TxHeader.IDE = CAN_ID_STD;   // USE STANDART ID
	  TxHeader.DLC = 8;
	  TxHeader.TransmitGlobalTime = 0;
	  while(HAL_CAN_Start(&hcan) == HAL_ERROR);
	  /* SOME LED BLINK FOR SUCCESSFUL STARTUP*/
	  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, 1);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, 1);
	  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin|LED4_Pin, 0);
	  HAL_Delay(200);
	  HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, 0);
	  time_ms = HAL_GetTick();
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
	  HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, 1);
	  HAL_Delay(5000);
	  HAL_NVIC_SystemReset();
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
