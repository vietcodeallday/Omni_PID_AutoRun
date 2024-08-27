/*
 * wit.h
 *
 *  Created on: Jul 1, 2024
 *      Author: quocv
 */

#ifndef INC_WIT_H_
#define INC_WIT_H_

#include "main.h"
typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}angleRead_t;
extern angleRead_t angle_real;

void data_receive(uint8_t data);
void uart_handle(angleRead_t *angle);

void uart_init();

#endif /* INC_WIT_H_ */
