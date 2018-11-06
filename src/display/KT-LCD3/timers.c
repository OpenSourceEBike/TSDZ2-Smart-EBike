/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "stm8s.h"
#include "stm8s_tim1.h"
#include "stm8s_tim3.h"

void timer3_init (void)
{
  uint16_t ui16_i;

  // TIM3 Peripheral Configuration
  TIM3_DeInit();

  // 16MHz clock
  // prescaler = 4
  // target: 1ms (0.001)
  // 0.001 ÷ (1÷(16000000÷4)) = 4000
  TIM3_TimeBaseInit(TIM3_PRESCALER_4, 4000); // each interrupt at 1ms
  TIM3_ClearFlag(TIM3_FLAG_UPDATE); // clear TIM3 update flag
  TIM3_ITConfig(TIM3_IT_UPDATE, ENABLE); // enable update interrupt
  TIM3_Cmd(ENABLE); // TIM3 counter enable

  // IMPORTANT: this software delay is needed so timer3 work after this
  for(ui16_i = 0; ui16_i < (29000); ui16_i++) { ; }
}

// Timer1 is used to create a PWM duty-cyle signal to control LCD backlight
void timer1_init (void)
{
  uint16_t ui16_i;

  // counter period = (1 / (16000000 / 4200)) * (20 + 1) = 5ms = 100Hz
  TIM1_TimeBaseInit(4200, TIM1_COUNTERMODE_DOWN, 20, 0);

  TIM1_OC4Init(TIM1_OCMODE_PWM1, TIM1_OUTPUTSTATE_ENABLE, 0, TIM1_OCPOLARITY_HIGH, TIM1_OCIDLESTATE_RESET);
  TIM1_ARRPreloadConfig(ENABLE);
  TIM1_Cmd(ENABLE);
  TIM1_CtrlPWMOutputs(ENABLE);

  // IMPORTANT: this software delay is needed so timer2 work after this
  for(ui16_i = 0; ui16_i < (29000); ui16_i++) { ; }
}
