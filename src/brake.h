/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _BRAKE_H
#define _BRAKE_H

#include "main.h"

extern volatile uint8_t ui8_g_brakes_state;

void brake_init(void);
uint8_t brake_is_set(void);

#endif /* _BRAKE_H */
