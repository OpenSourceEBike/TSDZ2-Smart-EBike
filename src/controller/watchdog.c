/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "stm8s.h"
#include "stm8s_clk.h"
#include "stm8s_iwdg.h"

// PLEASE NOTE: while debuging using STLinkV2, watchdog seems to be disable and to test, you will need to run without the debugger
void watchdog_init (void)
{
  IWDG_Enable ();
  IWDG_WriteAccessCmd (IWDG_WriteAccess_Enable);
  IWDG_SetPrescaler (IWDG_Prescaler_4);

//  Timeout period
//  The timeout period can be configured through the IWDG_PR and IWDG_RLR registers. It
//  is determined by the following equation:
//  T = 2 * T LSI * P * R
//  where:
//  T = Timeout period
//  T LSI = 1/f LSI
//  P = 2 (PR[2:0] + 2)
//  R = RLR[7:0]+1
//
//  0.0159 = 2 * (1 / 128000) * 4 * 255
//  R = 255
  IWDG_SetReload (255); // ~16ms
  IWDG_ReloadCounter ();
}
