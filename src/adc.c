/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "pins.h"
#include "stm8s_adc1.h"
#include "adc.h"
#include "ebike_app.h"
#include "motor.h"

static void adc_trigger (void);

void adc_init (void)
{
  uint16_t ui16_counter;
  uint16_t ui16_adc_battery_current_offset;
  uint16_t ui16_adc_torque_sensor_offset;
  uint8_t ui8_i;

  //init GPIO for the used ADC pins
  GPIO_Init(GPIOB,
	    (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_3),
	    GPIO_MODE_IN_FL_NO_IT);

  //init ADC1 peripheral
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
      ADC1_CHANNEL_7,
      ADC1_PRESSEL_FCPU_D2,  // this always worked
//      ADC1_PRESSEL_FCPU_D3, // this makes reads battery voltage oscillating and lower from real value: ADC max possible frequency is 6MHz @VCC 5V. 16MHz / 3 = 5.33MHz
      ADC1_EXTTRIG_TIM,
      DISABLE,
      ADC1_ALIGN_LEFT,
      (ADC1_SCHMITTTRIG_CHANNEL3 | ADC1_SCHMITTTRIG_CHANNEL5 | ADC1_SCHMITTTRIG_CHANNEL6 | ADC1_SCHMITTTRIG_CHANNEL7),
            DISABLE);

  ADC1_ScanModeCmd(ENABLE);
  ADC1_Cmd(ENABLE);

  //********************************************************************************
  // next code is for "calibrating" the offset value of some ADC channels

  // 6s delay to wait for voltages stabilize (maybe beause capacitors on the circuit)
  // this was tested on 27.12.2019 by Casainho and lower values like 5s would not work.
  ui16_counter = TIM3_GetCounter() + 6000;
  while(TIM3_GetCounter() < ui16_counter) ;

  // read and discard few samples of ADC, to make sure the next samples are ok
  for(ui8_i = 0; ui8_i < 64; ui8_i++)
  {
    ui16_counter = TIM3_GetCounter() + 2;
    while(TIM3_GetCounter() < ui16_counter) ; // delay
    adc_trigger();
    while(!ADC1_GetFlagStatus(ADC1_FLAG_EOC)) ; // wait for end of conversion
  }

  // read and average a few values of ADC battery current
  ui16_adc_battery_current_offset = 0;
  for(ui8_i = 0; ui8_i < 16; ui8_i++)
  {
    ui16_counter = TIM3_GetCounter() + 2;
    while(TIM3_GetCounter() < ui16_counter) ; // delay
    adc_trigger();
    while(!ADC1_GetFlagStatus(ADC1_FLAG_EOC)) ; // wait for end of conversion
    ui16_adc_battery_current_offset += UI16_ADC_10_BIT_BATTERY_CURRENT;
  }
  ui16_g_adc_current_offset = ui16_adc_battery_current_offset >> 4;

  // read and average a few values of ADC torque sensor
  ui16_adc_torque_sensor_offset = 0;
  for(ui8_i = 0; ui8_i < 16; ui8_i++)
  {
    ui16_counter = TIM3_GetCounter() + 2;
    while(TIM3_GetCounter() < ui16_counter) ; // delay
    adc_trigger();
    while(!ADC1_GetFlagStatus(ADC1_FLAG_EOC)) ; // wait for end of conversion
    ui16_adc_torque_sensor_offset += UI16_ADC_10_BIT_TORQUE_SENSOR;
  }
  ui16_g_adc_torque_sensor_min_value = ui16_adc_torque_sensor_offset >> 4;
}

static void adc_trigger (void)
{
  // trigger ADC conversion of all channels (scan conversion, buffered)
  ADC1->CSR &= 0x07; // clear EOC flag first (selected also channel 7)
  ADC1->CR1 |= ADC1_CR1_ADON; // Start ADC1 conversion
}

