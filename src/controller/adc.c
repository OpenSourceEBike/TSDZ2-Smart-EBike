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
  uint8_t ui8_i;

  //init GPIO for the used ADC pins
  GPIO_Init(GPIOB,
	    (GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_3),
	    GPIO_MODE_IN_FL_NO_IT);

  //init ADC1 peripheral
  ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
      ADC1_CHANNEL_7,
      ADC1_PRESSEL_FCPU_D2,
      ADC1_EXTTRIG_TIM,
      DISABLE,
      ADC1_ALIGN_LEFT,
      (ADC1_SCHMITTTRIG_CHANNEL3 | ADC1_SCHMITTTRIG_CHANNEL5 | ADC1_SCHMITTTRIG_CHANNEL6 | ADC1_SCHMITTTRIG_CHANNEL7),
            DISABLE);

  ADC1_ScanModeCmd(ENABLE);
  ADC1_Cmd(ENABLE);



  // discard few samples of ADC, to make sure the next samples are ok ////////////////////////////////////////
  for(ui8_i = 0; ui8_i < 64; ++ui8_i)
  {
    ui16_counter = TIM3_GetCounter() + 10; // delay ~10ms
    
    // delay ~10ms
    while(TIM3_GetCounter() < ui16_counter);
    
    // trigger ADC conversion on all channels (scan conversion, buffered)
    adc_trigger();
    
    // wait for end of conversion
    while(!ADC1_GetFlagStatus(ADC1_FLAG_EOC));
  }



  // ADC battery current initialization //////////////////////////////////////////////////////////////////////
  
  uint16_t ui16_adc_battery_current_offset = 0;

  // calibrate battery current
  for (ui8_i = 0; ui8_i < 16; ++ui8_i)
  {
    ui16_counter = TIM3_GetCounter() + 10; // delay ~10ms
    
    // delay ~10ms
    while(TIM3_GetCounter() < ui16_counter);
    
    // trigger ADC conversion on all channels (scan conversion, buffered)
    adc_trigger();
    
    // wait for end of conversion
    while(!ADC1_GetFlagStatus(ADC1_FLAG_EOC));
    
    ui16_adc_battery_current_offset += UI8_ADC_BATTERY_CURRENT;
  }
  
  // calculate and set current offset
  ui16_adc_battery_current_offset >>= 4;
  ui8_adc_battery_current_offset = ui16_adc_battery_current_offset >> 2;
  ui8_g_adc_motor_phase_current_offset = ui8_adc_battery_current_offset;



  // ADC torque sensor initialization //////////////////////////////////////////////////////////////////////
  
  #define ADC_TORQUE_SENSOR_OFFSET_ADJUSTMENT   7
  
  uint16_t ui16_adc_pedal_torque_offset = 0;
  
  // calibrate torque sensor
  for (ui8_i = 0; ui8_i < 16; ++ui8_i)
  {
    ui16_counter = TIM3_GetCounter() + 10; // delay ~10ms
    
    while (TIM3_GetCounter() < ui16_counter) ; // delay ~10ms
    
    // trigger ADC conversion on all channels (scan conversion, buffered)
    adc_trigger();
    
    // wait for end of conversion
    while (!ADC1_GetFlagStatus(ADC1_FLAG_EOC));
    
    ui16_adc_pedal_torque_offset += UI8_ADC_TORQUE_SENSOR;
  }
  
  // calculate and set torque offset
  ui8_adc_pedal_torque_offset = (ui16_adc_pedal_torque_offset >> 4) + ADC_TORQUE_SENSOR_OFFSET_ADJUSTMENT;
}


static void adc_trigger (void)
{
  // trigger ADC conversion on all channels (scan conversion, buffered)
  ADC1->CSR &= 0x07; // clear EOC flag first (selected also channel 7)
  ADC1->CR1 |= ADC1_CR1_ADON; // Start ADC1 conversion
}


uint16_t ui16_adc_read_battery_current_10b (void)
{
  uint16_t temph;
  uint8_t templ;

  templ = *(uint8_t*)(0x53EB);
  temph = *(uint8_t*)(0x53EA);

  temph = ((uint16_t) temph) << 2 | ((uint16_t) templ);
  
  // we ignore low values temph < 5 to avoid issues with other consumers than the motor
  // Piecewise linear is better than a step, to avoid limit cycles.
  // in     --> outr
  // 0 -  5 --> 0 - 0
  // 5 - 15 --> 0 - 15
  if (temph <= 5)
    return 0;
  if (temph > 15)
    return temph;
  temph -= 5;
  return temph+(temph>>1);
}


uint16_t ui16_adc_read_torque_sensor_10b (void)
{
  uint16_t temph;
  uint8_t templ;

  templ = *(uint8_t*)(0x53E9);
  temph = *(uint8_t*)(0x53E8);

  return ((uint16_t) temph) << 2 | ((uint16_t) templ);
}


uint16_t ui16_adc_read_throttle_10b (void)
{
  uint16_t temph;
  uint8_t templ;

  templ = *(uint8_t*)(0x53EF);
  temph = *(uint8_t*)(0x53EE);

  return ((uint16_t) temph) << 2 | ((uint16_t) templ);
}


uint16_t ui16_adc_read_battery_voltage_10b (void)
{
  uint16_t temph;
  uint8_t templ;

  templ = *(uint8_t*)(0x53ED);
  temph = *(uint8_t*)(0x53EC);

  return ((uint16_t) temph) << 2 | ((uint16_t) templ);
}

