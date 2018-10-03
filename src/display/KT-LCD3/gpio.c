/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "stm8s.h"
#include "stm8s_gpio.h"
#include "pins.h"

void gpio_init (void)
{
  GPIO_Init(LCD3_BUTTON_ONOFF__PORT,
            LCD3_BUTTON_ONOFF__PIN,
            GPIO_MODE_IN_FL_NO_IT);

  GPIO_Init(LCD3_BUTTON_UP__PORT,
            LCD3_BUTTON_UP__PIN,
            GPIO_MODE_IN_PU_NO_IT);

  GPIO_Init(LCD3_BUTTON_DOWN__PORT,
            LCD3_BUTTON_DOWN__PIN,
            GPIO_MODE_IN_PU_NO_IT);

  GPIO_Init(LCD3_ENABLE_BACKLIGHT__PORT,
            LCD3_ENABLE_BACKLIGHT__PIN,
            GPIO_MODE_OUT_PP_LOW_FAST);

  GPIO_Init(LCD3_ENABLE_BACKLIGHT_POWER__PORT,
            LCD3_ENABLE_BACKLIGHT_POWER__PIN,
            GPIO_MODE_OUT_PP_LOW_FAST);

  GPIO_Init(GPIOB,
      GPIO_PIN_4,
      GPIO_MODE_OUT_PP_LOW_FAST);
}



