/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _UART_H
#define _UART_H

#include "main.h"

extern volatile uint8_t ui8_received_first_package;

void uart2_init (void);
void uart_data_clock (void);

#if __SDCC_REVISION < 9624
void putchar(char c);
#else
int putchar(int c);
#endif

#if __SDCC_REVISION < 9989
char getchar(void);
#else
int getchar(void);
#endif

#endif /* _UART_H */

