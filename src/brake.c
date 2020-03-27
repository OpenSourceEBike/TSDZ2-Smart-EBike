/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_it.h"
#include "pins.h"
#include "main.h"
#include "interrupts.h"
#include "brake.h"
#include "motor.h"

volatile uint8_t ui8_g_brakes_state = 0;

void brake_init(void)
{
  //brake pin as external input pin interrupt
  GPIO_Init(BRAKE__PORT,
	    BRAKE__PIN,
	    GPIO_MODE_IN_PU_NO_IT); // input pull-up, no external interrupt
}

uint8_t brake_is_set(void)
{
  return (GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) == 0);
}


