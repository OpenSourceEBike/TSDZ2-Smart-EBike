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
#include "timers.h"
#include "adc.h"
#include "buttons.h"
#include "lcd.h"
#include "uart.h"
#include "ht162.h"
#include "config.h"
#include "eeprom.h"

// With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
// in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
// to place the function prototype in a header file as long as the header file is included in the file that
// contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
// not be in place so the ISR will not be executed when the interrupt occurs.

// Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
// all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// when I have this code inside a function: "static uint8_t ui8_example_counter = 0;"
// and the solution was define the variable as global instead
// Another error example:
// *** buffer overflow detected ***: sdcc terminated
// Caught signal 6: SIGABRT

/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes
// UART2 Receive interrupt
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);
void TIM3_UPD_OVF_BRK_IRQHandler(void) __interrupt(TIM3_UPD_OVF_BRK_IRQHANDLER);

int main (void)
{
  uint16_t ui16_timer3_counter;
  uint16_t ui16_10ms_loop_counter = 0;

  // set clock at the max 16 MHz
  CLK_HSIPrescalerConfig (CLK_PRESCALER_HSIDIV1);
  gpio_init ();
  timer1_init ();
  timer3_init ();
  uart2_init ();
  adc_init ();
  eeprom_init ();
  lcd_init (); // must be after eeprom_init ();

  // block until users releases the buttons
  while (buttons_get_onoff_state () || buttons_get_down_state () || buttons_get_up_state ());

  enableInterrupts ();

  while (1)
  {
    // because of continue; at the end of each if code block that will stop the while (1) loop there,
    // the first if block code will have the higher priority over any others
    ui16_timer3_counter = get_timer3_counter ();
    if ((ui16_timer3_counter - ui16_10ms_loop_counter) > 10) // every 10ms
    {
      ui16_10ms_loop_counter = ui16_timer3_counter;

      buttons_clock ();
      lcd_clock ();
      uart_data_clock ();

      continue;
    }
  }

  return 0;
}