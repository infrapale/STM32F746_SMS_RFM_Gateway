/*
 * console.h
 *
 *  Created on: Jan 6, 2021
 *      Author: tom_h
 */

#ifndef INC_CONSOLE_H_
#define INC_CONSOLE_H

#include <main.h>

#define VALUE_TEXT_LEN 20
#define ASCII_CR 0x13
#define ASCII_LF 0x10


enum console_priority {
	top_level   = 100,
	application = 90,
	development = 50,
	debug       = 10
};

void ConsoleInitialize(UART_HandleTypeDef *uart_ptr);
void ConsoleSetPriority(enum console_priority c_p);
void ConsoleWr(enum console_priority c_priority, char *txt, uint8_t newline);
void ConsoleWrChar(enum console_priority c_priority, char c);
void ConsoleNewline(enum console_priority c_priority);
void ConsoleWrDec(enum console_priority c_priority, char *pre_txt, uint16_t value ,char *post_txt, uint8_t newline);
HAL_StatusTypeDef ConsoleRdChar( uint8_t *pData );
HAL_StatusTypeDef ConsoleRdLn(uint8_t *pData, uint8_t max_len );

#endif /* INC_CONSOLE_H_ */
