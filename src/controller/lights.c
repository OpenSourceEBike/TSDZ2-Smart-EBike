/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2019.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "pins.h"
#include "stm8s_gpio.h"

void lights_init (void)
{
  GPIO_Init(LIGHTS__PORT, LIGHTS__PIN, GPIO_MODE_OUT_PP_LOW_SLOW);
}

void lights_set_state (uint8_t ui8_state)
{
  if (ui8_state)
  {
    GPIO_WriteHigh (LIGHTS__PORT, LIGHTS__PIN);
  }
  else
  {
    GPIO_WriteLow (LIGHTS__PORT, LIGHTS__PIN);
  }
}
