/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "stm8s.h"
#include "stm8s_tim2.h"

// Timer2 is used to create the pulse signal for excitation of the torque sensor circuit
// Pulse signal: period of 20us, Ton = 2us, Toff = 18us
void timer2_init (void)
{
  uint16_t ui16_i;

  // Timer2 clock = 16MHz; target: 20us period --> 50khz
  // counter period = (1 / (16000000 / prescaler)) * (159 + 1) = 20us
  TIM2_TimeBaseInit(TIM2_PRESCALER_2, 159);

  // pulse of 2us
  TIM2_OC2Init(TIM2_OCMODE_PWM1, TIM2_OUTPUTSTATE_ENABLE, 16, TIM2_OCPOLARITY_HIGH);
  TIM2_OC2PreloadConfig(ENABLE);

  TIM2_ARRPreloadConfig(ENABLE);

  TIM2_Cmd(ENABLE);

  // IMPORTANT: this software delay is needed so timer2 work after this
  for(ui16_i = 0; ui16_i < (65000); ui16_i++) { ; }
}

void timer3_init (void)
{
  uint16_t ui16_i;

  // TIM3 Peripheral Configuration
  TIM3_DeInit();
  TIM3_TimeBaseInit(TIM3_PRESCALER_16384, 0xffff); // each incremment at every ~1ms
  TIM3_Cmd(ENABLE); // TIM3 counter enable

  // IMPORTANT: this software delay is needed so timer3 work after this
  for(ui16_i = 0; ui16_i < (65000); ui16_i++) { ; }
}
