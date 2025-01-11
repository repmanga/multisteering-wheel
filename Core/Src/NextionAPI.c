#include "NextionAPI.h"
#include "stm32f1xx_hal.h"
extern UART_HandleTypeDef huart1;

char ENDMSG[3] = {255, 255, 255};

void nextion_send(char* cmd){
	   HAL_UART_Transmit (&huart1, (uint8_t*) cmd, strlen (cmd), 50);
	   HAL_UART_Transmit (&huart1, (uint8_t*) &ENDMSG, 3, 50);
}
