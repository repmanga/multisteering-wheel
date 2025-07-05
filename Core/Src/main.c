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
#include <stdbool.h>
#include "NextionAPI.h"
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* GPIO Pin Definitions */
#define BTN_GEAR_NEUTRAL_PIN      (GPIO_PIN_15)   /* PC15 */
#define BTN_GEAR_NEUTRAL_PORT     (GPIOC)

#define BTN_NEXT_PAGE_PIN         (GPIO_PIN_0)    /* PA0 */
#define BTN_NEXT_PAGE_PORT        (GPIOA)

#define BTN_ENG_STOP_PIN          (GPIO_PIN_3)    /* PA3 */
#define BTN_ENG_STOP_PORT         (GPIOA)

#define BTN_ENG_START_PIN         (GPIO_PIN_4)    /* PA4 */
#define BTN_ENG_START_PORT        (GPIOA)

#define BTN_GEAR_UP_PIN           (GPIO_PIN_6)    /* PB6 */
#define BTN_GEAR_UP_PORT          (GPIOB)

#define BTN_GEAR_DOWN_PIN         (GPIO_PIN_7)    /* PB7 */
#define BTN_GEAR_DOWN_PORT        (GPIOB)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* CAN message type enumeration */
typedef enum {
	ENGIN_START  	= 0U,
	ENGIN_STOP	 	= 1U,
	GEAR_UP		 	= 2U,
	GEAR_DOWN	 	= 3U,
	GEAR_NEUTRAL 	= 4U,
	MSG_NONE	 	= 5U
} CanMsgType;

/* CAN message buffers structure */
typedef struct {
	uint8_t buff[8U]; /* Input buffer */
	uint8_t x600[8U]; /* ID 0x600: RPM, TPS, MAP, etc. */
	uint8_t x601[8U]; /* ID 0x601: AIN1-AIN4 */
	uint8_t x602[8U]; /* ID 0x602: VSPD, BARO, CLT, etc. */
	uint8_t x604[8U]; /* ID 0x604: GEAR, BATT, etc. */
} IdMsgArray;

/* ECU values structure */
typedef struct {
	uint16_t 	RPM;
	uint16_t 	MAP;
	uint16_t 	AIN0;
	uint16_t 	AIN1;
	uint16_t 	AIN2;
	uint16_t 	AIN3;
	uint16_t 	AIN4;
	uint16_t 	VSPD;
	uint16_t 	BATT;
	uint16_t 	ERRFLAG;
	int16_t 	CLT;
	uint8_t 	TPS;
	uint8_t 	BARO;
	uint8_t 	OILT;
	uint8_t 	OILP;
	uint8_t 	FUELP;
	uint8_t 	GEAR;
	int8_t 		IAT;
	int8_t 		ECUTEMP;
	int8_t 		FAN;
} EcuValues;

/* Global variables */
static CAN_TxHeaderTypeDef TxHeader = {
		.StdId = 0U,
		.ExtId = 0U,
		.IDE = CAN_ID_STD,
		.RTR = CAN_RTR_DATA,
		.DLC = 0U,
		.TransmitGlobalTime = 0U
};

static CAN_RxHeaderTypeDef RxHeader = { 0U };
static uint8_t TxData[8U] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U };
static uint32_t TxMailbox = 0U;
static char cmd[50U] = { 0U };
static char cmd_page[50U] = { 0U };
static IdMsgArray RxData = { 0U };
static EcuValues ECU = { 0U };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void button_handler(void);
int can_msg_handler(uint8_t typemsg);
void data_update_handler(void);
void startup(void);
void data_send_handler(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	while (1) {

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void) {

	/* USER CODE BEGIN CAN_Init 0 */
	CAN_FilterTypeDef sFilterConfig;
	/* USER CODE END CAN_Init 0 */

	/* USER CODE BEGIN CAN_Init 1 */

	/* USER CODE END CAN_Init 1 */
	hcan.Instance = CAN1;
	hcan.Init.Prescaler = 4U;
	hcan.Init.Mode = CAN_MODE_NORMAL;
	hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
	hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
	hcan.Init.TimeTriggeredMode = DISABLE;
	hcan.Init.AutoBusOff = DISABLE;
	hcan.Init.AutoWakeUp = DISABLE;
	hcan.Init.AutoRetransmission = DISABLE;
	hcan.Init.ReceiveFifoLocked = DISABLE;
	hcan.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN_Init 2 */
	sFilterConfig.FilterBank = 0U;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000U;
	sFilterConfig.FilterIdLow = 0x0000U;
	sFilterConfig.FilterMaskIdHigh = 0x0000U;
	sFilterConfig.FilterMaskIdLow = 0x0000U;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	//sFilterConfig.SlaveStartFilterBank = 14;

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE END CAN_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200U;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : CAN_LED_Pin */
	GPIO_InitStruct.Pin = CAN_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(CAN_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC15 */
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA0 */
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB6 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);

	HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int can_msg_handler(uint8_t typemsg) {
	TxHeader.StdId = 0x642U;
	switch (typemsg) {
	case ENGIN_START:
		TxData[4U] = 0x01U;
		break;
	case ENGIN_STOP:
		TxData[4U] = 0x02U;
		break;
	case GEAR_UP:
		TxData[4U] = 0x04U;
		break;
	case GEAR_DOWN:
		TxData[4U] = 0x08U;
		break;
	case GEAR_NEUTRAL:
		TxData[4U] = 0x10U;
		break;
	default:
		TxData[4U] = 0x00U;
		break;
	}

	uint32_t timeout = 100U;
	while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) && (timeout > 0U)) {
		timeout--;
	}

	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
		return -1;
	}
	return 0;
}

/**
 * @brief  Update ECU data from CAN messages
 * @note   Converts raw CAN data to ECU structure format
 * @retval None
 */
void data_update_handler(void) {
	/* Macro to combine two bytes into a word (MISRA compliant version) */
	#define BYTES_TO_WORD(high, low)  ((uint16_t)(((uint16_t)(high) << 8U) | (low)))

	/* Process CAN ID 0x600 data */
	ECU.RPM = BYTES_TO_WORD(RxData.x600[1U], RxData.x600[0U]);
	ECU.TPS = RxData.x600[2U];
	ECU.MAP = BYTES_TO_WORD(RxData.x600[5U], RxData.x600[4U]);

	/* Process CAN ID 0x601 data */
	ECU.AIN1 = BYTES_TO_WORD(RxData.x601[0U], RxData.x601[1U]);
	ECU.AIN2 = BYTES_TO_WORD(RxData.x601[2U], RxData.x601[3U]);
	ECU.AIN3 = BYTES_TO_WORD(RxData.x601[4U], RxData.x601[5U]);
	ECU.AIN4 = BYTES_TO_WORD(RxData.x601[6U], RxData.x601[7U]);

	/* Process CAN ID 0x602 data */
	ECU.VSPD = 	BYTES_TO_WORD(RxData.x602[0U], RxData.x602[1U]);
	ECU.BARO = 	RxData.x602[3U];
	ECU.OILT = 	RxData.x602[4U];
	ECU.FUELP = RxData.x602[5U];
	ECU.CLT = BYTES_TO_WORD(RxData.x602[7U], RxData.x602[6U]);

	/* Process CAN ID 0x604 data */
	ECU.GEAR = RxData.x604[0U];
	ECU.BATT = BYTES_TO_WORD(RxData.x604[3U], RxData.x604[2U]);

	#undef BYTES_TO_WORD  /* Limit macro scope */
}

void data_send_handler(void) {
    const struct {
        const char *format;
        int value;
    } commands[] = {
        {"v.RP.val=%d", (int)ECU.RPM},
        {"v.GE.val=%d", (int)ECU.GEAR},
        {"v.SP.val=%d", (int)ECU.VSPD},
        {"v.BA.val=%d", (int)ECU.BATT},
        {"v.OT.val=%d", (int)ECU.OILT},
        {"v.WA.val=%d", (int)ECU.CLT}
    };

    for (size_t i = 0U; i < sizeof(commands)/sizeof(commands[0]); i++) {
        if (sprintf(cmd, commands[i].format, commands[i].value) > 0) {
            (void)nextion_send(cmd);
        }
    }
}

/**
 * @brief  Initialize CAN peripheral and perform startup sequence
 * @retval None
 */
void startup(void) {
	const uint32_t led_blink_delay = 200U;

	/* Initialize LED */
	HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_RESET);

	/* Configure CAN TX header */
	TxHeader.StdId = 0x642U;
	TxHeader.ExtId = 0x00U;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.DLC = 8U;
	TxHeader.TransmitGlobalTime = 0U;

	/* Start CAN peripheral with error handling */
	while (HAL_CAN_Start(&hcan) == HAL_ERROR) {
		/* Add timeout handling if required by MISRA Rule 17.2 */
		(void) Error_Handler();
	}

	/* Activate CAN notifications */
	(void) HAL_CAN_ActivateNotification(&hcan,
	CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR);

	/* Visual startup indication */
	HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_SET);
	HAL_Delay(led_blink_delay);
	HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_RESET);

}

/**
 * @brief  CAN RX Fifo0 callback
 * @param  hcan: Specifies the CAN instance
 * @retval None
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	uint8_t *target = NULL;

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData.buff)
			!= HAL_OK) {
		return;
	}

	switch (RxHeader.StdId) {
	case 0x600U:
		target = RxData.x600;
		break;
	case 0x601U:
		target = RxData.x601;
		break;
	case 0x602U:
		target = RxData.x602;
		break;
	case 0x604U:
		target = RxData.x604;
		break;
	default:
		/* Неизвестный ID - намеренно ничего не делаем */
		break;
	}

	if (target != NULL) {
		(void) memcpy(target, RxData.buff, 7U);
		(void) data_update_handler();
		(void) data_send_handler();
	}
}

/**
 * @brief  EXTI line detection callback
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	typedef struct {
		uint16_t pin;
		uint8_t cmd;
		bool is_engine_start;
	} ButtonAction;

	static const ButtonAction button_map[] = {
			{ BTN_ENG_STOP_PIN, 	BTN_ENG_START_PORT, 	false },
			{ BTN_GEAR_NEUTRAL_PIN,	BTN_GEAR_NEUTRAL_PORT, 	false },
			{ BTN_GEAR_UP_PIN,		BTN_GEAR_UP_PORT, 		false },
			{ BTN_GEAR_DOWN_PIN,	BTN_GEAR_DOWN_PORT, 	false },
			{ BTN_ENG_START_PIN, 	BTN_ENG_START_PORT, 	true },
			{ BTN_NEXT_PAGE_PIN, 	0x00U, 					false } /* Special case */
	};

	const size_t button_count = sizeof(button_map) / sizeof(button_map[0]);
	static uint8_t page_num = 1U;
	const uint8_t max_page = 5U;

	for (size_t i = 0U; i < button_count; i++) {
		if (GPIO_Pin == button_map[i].pin) {
			if (button_map[i].is_engine_start) {
				/* Engine start button special handling */
				if (HAL_GPIO_ReadPin(BTN_ENG_START_PORT, BTN_ENG_START_PIN)
						== GPIO_PIN_RESET) {
					(void) can_msg_handler(button_map[i].cmd);
				} else {
					TxData[4U] = 0x00U;
					while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) {
						/* Wait for mailbox availability */
					}
					(void) HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData,
							&TxMailbox);
				}
			} else if (button_map[i].pin == BTN_NEXT_PAGE_PIN) {
				/* Page switching logic */
				page_num = (page_num % max_page) + 1U;
				(void) sprintf(cmd_page, "page %u", page_num);
				(void) nextion_send(cmd_page);
			} else {
				/* Standard button handling */
				(void) can_msg_handler(button_map[i].cmd);
			}
			break;
		}
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	static const uint32_t error_blink_delay = 5000U;
	static const char error_msg[] = "v.ERRFLAG.txt=\"ERRFLAG\"";

	/* Disable all interrupts */
	__disable_irq();

	/* Infinite error handling loop */
	for (;;) {
		/* Visual error indication */
		(void) HAL_GPIO_WritePin(CAN_LED_GPIO_Port, CAN_LED_Pin, GPIO_PIN_SET);
		(void) HAL_Delay(error_blink_delay);

		/* Send error message to display */
		(void) nextion_send(error_msg);

		/* System reset */
		HAL_NVIC_SystemReset();

		/* Safety in case reset fails */
		__NOP();
	}
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
