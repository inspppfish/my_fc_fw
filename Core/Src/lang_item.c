//
// Created by insppp on 2022/8/2.
//
#include <stdio.h>
#include "main.h"
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)

#endif


/*retargets the C library printf function to the USART*/
PUTCHAR_PROTOTYPE
{
	extern HAL_UART_StateTypeDef huart1;
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch, 1, 0xFFFF);
	return ch;
}