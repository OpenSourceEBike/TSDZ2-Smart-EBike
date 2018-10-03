/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "gpio.h"
#include "adc.h"

void adc_init (void)
{
  static uint8_t ui8_counter;

  //init GPIO for the used ADC pins
  GPIO_Init(GPIOE,
      GPIO_PIN_7,
      GPIO_MODE_IN_FL_NO_IT);

  //init ADC1 peripheral
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
      ADC1_CHANNEL_BATTERY_VOLTAGE,
      ADC1_PRESSEL_FCPU_D18,
      ADC1_EXTTRIG_TIM,
      DISABLE,
      ADC1_ALIGN_RIGHT,
      0,
      DISABLE);

  ADC1_Cmd (ENABLE);
}

uint16_t ui16_adc_read_battery_voltage_10b (void)
{
  ADC1_StartConversion ();
  while (!ADC1_GetFlagStatus(ADC1_FLAG_EOC)) ;

  return ADC1_GetConversionValue ();
}
