/*
 * wit.c
 *
 *  Created on: Jul 1, 2024
 *      Author: quocv
 */

#include "wit.h"
#define DATA_LEN 11

static uint8_t buff[DATA_LEN];
static uint8_t uart_len = 0;
static uint8_t uart_flag = 0;
static uint8_t receive_flag = 0;
angleRead_t angle_real;
//Receive and storage data from uart protocol
void data_receive(uint8_t data)
{
	if(data == 0x55 && uart_len == 0)
	{
		receive_flag = 1;
	}

	if(receive_flag)
	{
		if(uart_len == 10)
		{
			buff[uart_len] = data;
			uart_flag = 1;
		}
		else
		{
			buff[uart_len++] = data;
		}
	}
}
//==============================================//
// handling hex data to the angle of x, y, z

short data_handle(uint8_t dataH, uint8_t dataL)
{
	short data;
	data = ((short)((short)dataH<<8)|dataL);
	return data;
}
//---------------------------------------------//
void uart_handle(angleRead_t *angle)
{
	if(uart_flag)
	{
		if(buff[1] == 0x53) // so sanh xem loai du lieu gui la gi, 0x53: goc quay
		{
			angle->x = (int16_t)(data_handle(buff[3], buff[2]) / 32768.0*180.0);
			angle->y = (int16_t)(data_handle(buff[5], buff[4]) / 32768.0*180.0);
			angle->z = (int16_t)(data_handle(buff[7], buff[6]) / 32768.0*180.0);
		}
		receive_flag = 0;
		uart_len = 0;
		uart_flag = 0;
	}
}







