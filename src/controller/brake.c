/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2019.
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


void brake_init (void)
{
  // brake pin as external input pin interrupt
  GPIO_Init(BRAKE__PORT, BRAKE__PIN, GPIO_MODE_IN_FL_IT); // with external interrupt

  // initialize the interrupt operation
  EXTI_SetExtIntSensitivity(EXTI_PORT_GPIOC, EXTI_SENSITIVITY_RISE_FALL);
}


// brake signal interrupt
void EXTI_PORTC_IRQHandler(void) __interrupt(EXTI_PORTC_IRQHANDLER)
{
/*   if (brake_is_set()) { }
  else { } */
}


BitStatus brake_is_set(void)
{
  if (GPIO_ReadInputPin(BRAKE__PORT, BRAKE__PIN) == 0) { return 1; }
  else { return 0; }
}