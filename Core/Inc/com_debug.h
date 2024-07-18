/*
 * com_debug.h
 *
 *  Created on: Jul 5, 2024
 *      Author: sajanduwal
 */

#ifndef INC_COM_DEBUG_H_
#define INC_COM_DEBUG_H_

#include "main.h"
#include "stdio.h"
#include "stdarg.h"
#include "stdint.h"
#include "string.h"

extern TIM_HandleTypeDef htim2;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

void myDebug(const char *fmt, ...);

void OBCDebug(const char *fmt, ...);

int bufferSize(char *buffer);

void delay_us(uint32_t us);



#endif /* INC_COM_DEBUG_H_ */
