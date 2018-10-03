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
#include "lcd.h"
#include "uart.h"
#include "eeprom.h"
#include "button.h"
#include "ht162.h"
#include "config.h"

// With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
// in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
// to place the function prototype in a header file as long as the header file is included in the file that
// contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
// not be in place so the ISR will not be executed when the interrupt occurs.

// Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
// all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// when I have this code inside a function: "static uint8_t ui8_cruise_counter = 0;"
// and the solution was define the variable as global instead
// Another error example:
// *** buffer overflow detected ***: sdcc terminated
// Caught signal 6: SIGABRT

/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes
// UART2 Receive interrupt
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);

int main (void)
{
  uint16_t ui16_tim3_counter;
  uint16_t ui16_10ms_loop_counter;

  //set clock at the max 16MHz
  CLK_HSIPrescalerConfig (CLK_PRESCALER_HSIDIV1);
  gpio_init ();
  timer1_init ();
  timer3_init ();
  uart2_init ();
  adc_init ();
  eeprom_init ();
  lcd_init (); // must be after eeprom_init ();
  enableInterrupts ();

  // block until users releases the buttons
  while (get_button_onoff_state () ||
      get_button_down_state () ||
      get_button_up_state ()) ;

  ui16_tim3_counter = TIM3_GetCounter ();

  while (1)
  {
    // because of continue; at the end of each if code block that will stop the while (1) loop there,
    // the first if block code will have the higher priority over any others
    ui16_tim3_counter = TIM3_GetCounter ();
    if ((ui16_tim3_counter - ui16_10ms_loop_counter) > 10) // every 10ms
    {
      ui16_10ms_loop_counter = ui16_tim3_counter;

      clock_button ();
      clock_lcd ();
      clock_uart_data ();

      continue;
    }

#ifdef DEBUG_UART
    ui16_tim3_counter = TIM2_GetCounter ();
    if ((ui16_tim3_counter - ui16_debug_uart_counter) > 20) // every 20ms
    {
      ui16_debug_uart_counter = ui16_tim3_counter;

      // sugestion: no more than 6 variables printed (takes about 3ms to printf 6 variables)
      printf ("%d,%d,%d,%d,%d\n",
        ui8_duty_cycle_target,
        ui8_duty_cycle,
        ui16_motor_get_motor_speed_erps(),
        UI8_ADC_BATTERY_CURRENT,
        ui8_angle_correction);
      continue;
    }
#endif
  }

  return 0;
}

