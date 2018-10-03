/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "stm8s.h"
#include "stm8s_gpio.h"
#include "gpio.h"
#include "pins.h"

uint8_t ui8_buttons_events = 0;
uint8_t ui8_onoff_button_state = 0;
uint8_t ui8_onoff_button_state_counter = 0;
uint8_t ui8_down_button_state = 0;
uint8_t ui8_down_button_state_counter = 0;
uint8_t ui8_up_button_state = 0;
uint8_t ui8_up_button_state_counter = 0;

uint8_t get_button_up_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_UP__PORT, LCD3_BUTTON_UP__PIN) != 0 ? 0: 1;
}

uint8_t get_button_up_click_event (void)
{
  return (ui8_buttons_events & (1 << 4));
}

uint8_t get_button_up_long_click_event (void)
{
  return (ui8_buttons_events & (1 << 5));
}

void clear_button_up_click_event (void)
{
  ui8_buttons_events &= ~(1 << 4);
}

void clear_button_up_long_click_event (void)
{
  ui8_buttons_events &= ~((1 << 5) + (1 << 4));
}

uint8_t get_button_down_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_DOWN__PORT, LCD3_BUTTON_DOWN__PIN) != 0 ? 0: 1;
}

uint8_t get_button_down_click_event (void)
{
  return (ui8_buttons_events & (1 << 2));
}

uint8_t get_button_down_long_click_event (void)
{
  return (ui8_buttons_events & (1 << 3));
}

void clear_button_down_click_event (void)
{
  ui8_buttons_events &= ~(1 << 2);
}

void clear_button_down_long_click_event (void)
{
  ui8_buttons_events &= ~((1 << 3) + (1 << 2));
}

uint8_t get_button_onoff_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_ONOFF__PORT, LCD3_BUTTON_ONOFF__PIN) != 0 ? 1: 0;
}

uint8_t get_button_onoff_click_event (void)
{
  return (ui8_buttons_events & (1 << 0));
}

uint8_t get_button_onoff_long_click_event (void)
{
  return (ui8_buttons_events & (1 << 1));
}

void clear_button_onoff_click_event (void)
{
  ui8_buttons_events &= ~(1 << 0);
}

void clear_button_onoff_long_click_event (void)
{
  ui8_buttons_events &= ~((1 << 1) + (1 << 0));
}

uint8_t get_button_up_down_click_event (void)
{
  if (ui8_buttons_events & (1 << 6))
    return 1;
  return 0;
}

void clear_button_up_down_click_event (void)
{
  ui8_buttons_events &= ~(1 << 6);
}

uint8_t button_get_events (void)
{
  return ui8_buttons_events;
}

void button_clear_events (void)
{
  ui8_buttons_events = 0;
  ui8_onoff_button_state = 0;
  ui8_up_button_state = 0;
  ui8_down_button_state = 0;
}

void clock_button (void)
{
  switch (ui8_onoff_button_state)
  {
    case 0:
      if (!get_button_onoff_click_event () &&
          !get_button_onoff_long_click_event () &&
          get_button_onoff_state ())
        {
          ui8_onoff_button_state = 1;
        }
    break;

    case 1:
      // wait for button release; event click
      if (!get_button_onoff_state ())
      {
        ui8_onoff_button_state = 0;
        ui8_onoff_button_state_counter = 0;
        ui8_buttons_events |= (1 << 0);
      }

      // event long click
      if (ui8_onoff_button_state_counter++ > 200) // 2 seconds
      {
        ui8_onoff_button_state = 2;
        ui8_onoff_button_state_counter = 0;
        ui8_buttons_events |= (1 << 1);
      }
    break;

    case 2:
      // wait for button release
      if (!get_button_onoff_state ())
      {
        ui8_onoff_button_state = 0;
      }
    break;

    default:
      ui8_onoff_button_state = 0;
    break;
  }

  switch (ui8_down_button_state)
  {
    case 0:
      if (!(get_button_down_click_event () &&
          get_button_down_long_click_event () &&
          get_button_up_down_click_event ()) &&
          get_button_down_state ())
      {
        ui8_down_button_state = 1;
      }
    break;

    case 1:
      // wait for button release; event click
      if (!get_button_down_state ())
      {
        ui8_down_button_state = 0;
        ui8_down_button_state_counter = 0;
        ui8_buttons_events |= (1 << 2);
      }

      // event long click
      if (ui8_down_button_state_counter++ > 200) // 2 seconds
      {
        // up and down button click
        if (ui8_up_button_state == 1)
        {
          ui8_buttons_events |= (1 << 6);
          ui8_up_button_state = 2;
        }
        else
        {
          ui8_buttons_events |= (1 << 3);
        }

        ui8_down_button_state = 2;
        ui8_down_button_state_counter = 0;
      }
    break;

    case 2:
      // wait for button release
      if (!get_button_down_state ())
      {
        ui8_down_button_state = 0;
      }
    break;

    default:
      ui8_down_button_state = 0;
    break;
  }

  switch (ui8_up_button_state)
  {
    case 0:
      if (!(get_button_up_click_event () &&
          get_button_up_long_click_event () &&
          get_button_up_down_click_event ()) &&
          get_button_up_state ())
      {
        ui8_up_button_state = 1;
      }
    break;

    case 1:
      // wait for button release; event click
      if (!get_button_up_state ())
      {
        ui8_up_button_state = 0;
        ui8_up_button_state_counter = 0;
        ui8_buttons_events |= (1 << 4);
      }

      // event long click
      if (ui8_up_button_state_counter++ > 200) // 200 seconds
      {
        // up and down button click
        if (ui8_down_button_state == 1)
        {
          ui8_buttons_events |= (1 << 6);
          ui8_down_button_state = 2;
        }
        else
        {
          ui8_buttons_events |= (1 << 5);
        }

        ui8_up_button_state = 2;
        ui8_up_button_state_counter = 0;
      }
    break;

    case 2:
      // wait for button release
      if (!get_button_up_state ())
      {
        ui8_up_button_state = 0;
      }
    break;

    default:
      ui8_up_button_state = 0;
    break;
  }
}
