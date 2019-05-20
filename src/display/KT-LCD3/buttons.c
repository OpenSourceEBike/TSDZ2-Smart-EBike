/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "buttons.h"

#include "stm8s.h"
#include "stm8s_gpio.h"
#include "gpio.h"
#include "pins.h"

uint8_t ui8_onoff_button_state = 0;
uint8_t ui8_onoff_button_state_counter = 0;
uint8_t ui8_down_button_state = 0;
uint8_t ui8_down_button_state_counter = 0;
uint8_t ui8_up_button_state = 0;
uint8_t ui8_up_button_state_counter = 0;

buttons_events_t buttons_events = 0;

uint8_t buttons_get_up_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_UP__PORT, LCD3_BUTTON_UP__PIN) != 0 ? 0: 1;
}

uint8_t buttons_get_up_click_event (void)
{
  return (buttons_events & UP_CLICK) ? 1: 0;
}

uint8_t buttons_get_up_long_click_event (void)
{
  return (buttons_events & UP_LONG_CLICK) ? 1: 0;
}

void buttons_clear_up_click_event (void)
{
  buttons_events &= ~UP_CLICK;
}

uint8_t buttons_get_up_click_long_click_event(void)
{
  return (buttons_events & UP_CLICK_LONG_CLICK) ? 1: 0;
}

void buttons_clear_up_long_click_event (void)
{
  buttons_events &= ~UP_LONG_CLICK;
}

void buttons_clear_up_click_long_click_event (void)
{
  buttons_events &= ~UP_CLICK_LONG_CLICK;
}

uint8_t buttons_get_down_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_DOWN__PORT, LCD3_BUTTON_DOWN__PIN) != 0 ? 0: 1;
}

uint8_t buttons_get_down_click_event (void)
{
  return (buttons_events & DOWN_CLICK) ? 1: 0;
}

uint8_t buttons_get_down_long_click_event (void)
{
  return (buttons_events & DOWN_LONG_CLICK) ? 1: 0;
}

void buttons_clear_down_click_event (void)
{
  buttons_events &= ~DOWN_CLICK;
}

void buttons_clear_down_click_long_click_event (void)
{
  buttons_events &= ~DOWN_CLICK_LONG_CLICK;
}

uint8_t buttons_get_down_click_long_click_event (void)
{
  return (buttons_events & DOWN_CLICK_LONG_CLICK) ? 1: 0;
}

void buttons_clear_down_long_click_event (void)
{
  buttons_events &= ~DOWN_LONG_CLICK;
}

uint8_t buttons_get_onoff_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_ONOFF__PORT, LCD3_BUTTON_ONOFF__PIN) != 0 ? 1: 0;
}

uint8_t buttons_get_onoff_click_event (void)
{
  return (buttons_events & ONOFF_CLICK) ? 1: 0;
}

uint8_t buttons_get_onoff_long_click_event (void)
{
  return (buttons_events & ONOFF_LONG_CLICK) ? 1: 0;
}

uint8_t buttons_get_onoff_click_long_click_event (void)
{
  return (buttons_events & ONOFF_CLICK_LONG_CLICK) ? 1: 0;
}

void buttons_clear_onoff_click_event (void)
{
  buttons_events &= ~ONOFF_CLICK;
}

void buttons_clear_onoff_click_long_click_event (void)
{
  buttons_events &= ~ONOFF_CLICK_LONG_CLICK;
}

void buttons_clear_onoff_long_click_event (void)
{
  buttons_events &= ~ONOFF_LONG_CLICK;
}

uint8_t buttons_get_up_down_click_event (void)
{
  return (buttons_events & UPDOWN_CLICK) ? 1: 0;
}

void buttons_clear_up_down_click_event (void)
{
  buttons_events &= ~UPDOWN_CLICK;
}

buttons_events_t buttons_get_events (void)
{
  return buttons_events;
}

void buttons_set_events (buttons_events_t events)
{
  buttons_events |= events;
}

void buttons_clear_all_events (void)
{
  buttons_events = 0;
  ui8_onoff_button_state = 0;
  ui8_up_button_state = 0;
  ui8_down_button_state = 0;
}

void buttons_clock (void)
{
  // needed if the event is not cleared anywhere else
  //buttons_clear_onoff_click_long_click_event();

  switch (ui8_onoff_button_state)
  {
    case 0:
      if (!buttons_get_onoff_click_event() &&
          !buttons_get_onoff_long_click_event() &&
          !buttons_get_onoff_click_long_click_event() &&
          buttons_get_onoff_state ())
        {
          ui8_onoff_button_state_counter = 0;
          ui8_onoff_button_state = 1;
        }
    break;

    case 1:
      ui8_onoff_button_state_counter++;

      // event long click
      if (ui8_onoff_button_state_counter > 120) // 1.2 seconds
      {
        buttons_set_events(ONOFF_LONG_CLICK);
        ui8_onoff_button_state = 2;
        break;
      }

      // if button release
      if (!buttons_get_onoff_state ())
      {
        // let's validade if will be a quick click + long click
        if (ui8_onoff_button_state_counter <= 30) // 0.3 second
        {
          ui8_onoff_button_state_counter = 0;
          ui8_onoff_button_state = 3;
          break;
        }
        // event click
        else
        {
          buttons_set_events(ONOFF_CLICK);
          ui8_onoff_button_state = 0;
          break;
        }
      }
    break;

    case 2:
      // wait for button release
      if (!buttons_get_onoff_state ())
      {
        ui8_onoff_button_state = 0;
        break;
      }
    break;

    case 3:
      ui8_onoff_button_state_counter++;

      // on next step, start counting for long click
      if (buttons_get_onoff_state ())
      {
        ui8_onoff_button_state_counter = 0;
        ui8_onoff_button_state = 4;
        break;
      }

      // event click
      if (ui8_onoff_button_state_counter > 40)
      {
        buttons_set_events(ONOFF_CLICK);
        ui8_onoff_button_state = 0;
        break;
      }
    break;

    case 4:
      ui8_onoff_button_state_counter++;

      // event click, but this time it is: click + long click
      if (ui8_onoff_button_state_counter > 100)
      {
        buttons_set_events(ONOFF_CLICK_LONG_CLICK);
        ui8_onoff_button_state = 2;
        break;
      }

      // button release
      if (!buttons_get_onoff_state ())
      {
        buttons_set_events(ONOFF_CLICK);
        ui8_onoff_button_state = 0;
        break;
      }
    break;

    default:
      ui8_onoff_button_state = 0;
    break;
  }

  switch (ui8_up_button_state)
  {
    case 0:
      if (!buttons_get_up_click_event() &&
          !buttons_get_up_long_click_event() &&
          !buttons_get_up_click_long_click_event() &&
          !buttons_get_up_down_click_event() &&
          buttons_get_up_state())
        {
          ui8_up_button_state_counter = 0;
          ui8_up_button_state = 1;
        }
    break;

    case 1:
      ui8_up_button_state_counter++;

      // event long click
      if (ui8_up_button_state_counter++ > 200) // 2 seconds
      {
        // up and down button click
        if (ui8_down_button_state == 1)
        {
          buttons_set_events(UPDOWN_CLICK);
          ui8_down_button_state = 2;
        }
        else
        {
          buttons_set_events(UP_LONG_CLICK);
        }

        ui8_up_button_state = 2;
        ui8_up_button_state_counter = 0;
        break;
      }

      // if button release
      if (!buttons_get_up_state ())
      {
        // let's validade if will be a quick click + long click
        if (ui8_up_button_state_counter <= 30) // 0.3 second
        {
          ui8_up_button_state_counter = 0;
          ui8_up_button_state = 3;
          break;
        }
        // event click
        else
        {
          buttons_set_events(UP_CLICK);
          ui8_up_button_state = 0;
          break;
        }
      }
    break;

    case 2:
      // wait for button release
      if (!buttons_get_up_state ())
      {
        ui8_up_button_state = 0;
        break;
      }
    break;

    case 3:
      ui8_up_button_state_counter++;

      // on next step, start counting for long click
      if (buttons_get_up_state ())
      {
        ui8_up_button_state_counter = 0;
        ui8_up_button_state = 4;
        break;
      }

      // event click
      if (ui8_up_button_state_counter > 30)
      {
        buttons_set_events(UP_CLICK);
        ui8_up_button_state = 0;
        break;
      }
    break;

    case 4:
      ui8_up_button_state_counter++;

      // event click, but this time it is: click + long click
      if (ui8_up_button_state_counter > 100)
      {
        buttons_set_events(UP_CLICK_LONG_CLICK);
        ui8_up_button_state = 2;
        break;
      }

      // button release
      if (!buttons_get_up_state ())
      {
        buttons_set_events(UP_CLICK);
        ui8_up_button_state = 0;
        break;
      }
    break;

    default:
      ui8_up_button_state = 0;
    break;
  }

  switch (ui8_down_button_state)
  {
    case 0:
      if (!buttons_get_down_click_event() &&
          !buttons_get_down_long_click_event() &&
          !buttons_get_down_click_long_click_event() &&
          !buttons_get_up_down_click_event() &&
          buttons_get_down_state())
        {
          ui8_down_button_state_counter = 0;
          ui8_down_button_state = 1;
        }
    break;

    case 1:
      ui8_down_button_state_counter++;

      // event long click
      if (ui8_down_button_state_counter++ > 200) // 2 seconds
      {
        // up and down button click
        if (ui8_up_button_state == 1)
        {
          buttons_set_events(UPDOWN_CLICK);
          ui8_up_button_state = 2;
        }
        else
        {
          buttons_set_events(DOWN_LONG_CLICK);
        }

        ui8_down_button_state = 2;
        ui8_down_button_state_counter = 0;
        break;
      }


      // if button release
      if (!buttons_get_down_state ())
      {
        // let's validade if will be a quick click + long click
        if (ui8_down_button_state_counter <= 30) // 0.3 second
        {
          ui8_down_button_state_counter = 0;
          ui8_down_button_state = 3;
          break;
        }
        // event click
        else
        {
          buttons_set_events(DOWN_CLICK);
          ui8_down_button_state = 0;
          break;
        }
      }
    break;

    case 2:
      // wait for button release
      if (!buttons_get_down_state ())
      {
        ui8_down_button_state = 0;
        break;
      }
    break;

    case 3:
      ui8_down_button_state_counter++;

      // on next step, start counting for long click
      if (buttons_get_down_state ())
      {
        ui8_down_button_state_counter = 0;
        ui8_down_button_state = 4;
        break;
      }

      // event click
      if (ui8_down_button_state_counter > 30)
      {
        buttons_set_events(DOWN_CLICK);
        ui8_down_button_state = 0;
        break;
      }
    break;

    case 4:
      ui8_down_button_state_counter++;

      // event click, but this time it is: click + long click
      if (ui8_down_button_state_counter > 100)
      {
        buttons_set_events(DOWN_CLICK_LONG_CLICK);
        ui8_down_button_state = 2;
        break;
      }

      // button release
      if (!buttons_get_down_state ())
      {
        buttons_set_events(DOWN_CLICK);
        ui8_down_button_state = 0;
        break;
      }
    break;

    default:
      ui8_down_button_state = 0;
    break;
  }
}
