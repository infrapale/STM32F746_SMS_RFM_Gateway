/*
 * Console.c
 *
 *  Created on: Jan 6, 2021
 *      Author: tom_h
 */
#include <main.h>
#include <console.h>
#include <stdio.h>
#include <string.h>

#define CONSOLE_PRIORITY_IS_OK(__PRIORITY__)  ((__PRIORITY__) >= console.priority)


struct _concole_struct {
	UART_HandleTypeDef *p_uart;
	enum console_priority priority;
	uint32_t rd_timeout;
} console;

const char newline_str[] =  "\r\n";

void ConsoleInitialize(UART_HandleTypeDef *uart_ptr){
    console.p_uart     = uart_ptr;
	console.priority   = development;
	console.rd_timeout = 5000;
}
void ConsoleSetPriority(enum console_priority c_p){
	console.priority = 	c_p;
}

void ConsoleWr(enum console_priority c_priority, char *txt, uint8_t newline){
	//if (c_priority >= console.priority){
	if CONSOLE_PRIORITY_IS_OK(c_priority){
		HAL_UART_Transmit(console.p_uart, (uint8_t*)txt, strlen(txt),HAL_MAX_DELAY);
		if (newline) {
			ConsoleNewline(c_priority);
		}
	}
}

void ConsoleWrChar(enum console_priority c_priority, char c){
	if CONSOLE_PRIORITY_IS_OK(c_priority){
		HAL_UART_Transmit(console.p_uart, &c, 1,HAL_MAX_DELAY);
	}
}

void ConsoleNewline(enum console_priority c_priority){
	if (c_priority >= console.priority){
    	HAL_UART_Transmit(console.p_uart, (uint8_t*)newline_str, strlen( newline_str), HAL_MAX_DELAY);
	}
}

void ConsoleWrDec(enum console_priority c_priority, char *pre_txt, uint16_t value ,char *post_txt, uint8_t newline){
	if (c_priority >= console.priority){
		char value_str[VALUE_TEXT_LEN];
		sprintf(value_str,"%d", value);
		HAL_UART_Transmit(console.p_uart, (uint8_t*)pre_txt, strlen(pre_txt),HAL_MAX_DELAY);
		HAL_UART_Transmit(console.p_uart, (uint8_t*)value_str, strlen(value_str),HAL_MAX_DELAY);
		HAL_UART_Transmit(console.p_uart, (uint8_t*)post_txt, strlen(post_txt),HAL_MAX_DELAY);
		if (newline) {
			ConsoleNewline(c_priority);
		}
	}
}

/**
  * @brief
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param huart
  * @param pData
  * @param Size
  * @param Timeout
  * @retval
  */
HAL_StatusTypeDef ConsoleRdChar( uint8_t *pData ){
	HAL_StatusTypeDef uart_status;
	uart_status = HAL_UART_Receive(console.p_uart, pData, 1, console.rd_timeout);
	return uart_status;
}


HAL_StatusTypeDef ConsoleRdLn(uint8_t *pData, uint8_t max_len ){
	HAL_StatusTypeDef uart_status = HAL_OK;
	uint8_t  cnt= 0;
	uint8_t  done = 0;

	while ((cnt < max_len) && (uart_status == HAL_OK) && !done) {
		uart_status = HAL_UART_Receive(console.p_uart, pData, 1, console.rd_timeout);

		if (*pData == ASCII_CR){
			*pData = 0x00;
			done = 1;
		}
		pData++;
		cnt++;
	}
}

