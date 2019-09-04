/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho and Leon, 2019.
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

uint8_t ONOFF_CLICK = 0;
uint8_t ONOFF_LONG_CLICK = 0;
uint8_t ONOFF_CLICK_LONG_CLICK = 0;

uint8_t UP_CLICK = 0;
uint8_t UP_LONG_CLICK = 0;
uint8_t UP_CLICK_LONG_CLICK = 0;

uint8_t DOWN_CLICK = 0;
uint8_t DOWN_LONG_CLICK = 0;
uint8_t DOWN_CLICK_LONG_CLICK = 0;

uint8_t UP_DOWN_LONG_CLICK = 0;
uint8_t ONOFF_UP_LONG_CLICK = 0;
uint8_t ONOFF_DOWN_LONG_CLICK = 0;

#define LONG_CLICK_THRESHOLD          80     // 80    ->  0.8 seconds
#define CLICK_LONG_CLICK_THRESHOLD    100    // 100   ->  1.0 seconds
#define CLICK_THRESHOLD               30     // 30    ->  0.3 seconds

uint8_t buttons_get_up_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_UP__PORT, LCD3_BUTTON_UP__PIN) != 0 ? 0: 1;
}

uint8_t buttons_get_down_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_DOWN__PORT, LCD3_BUTTON_DOWN__PIN) != 0 ? 0: 1;
}

uint8_t buttons_get_onoff_state (void)
{
  return GPIO_ReadInputPin(LCD3_BUTTON_ONOFF__PORT, LCD3_BUTTON_ONOFF__PIN) != 0 ? 1: 0;
}

uint8_t buttons_get_events (void)
{
  if (ONOFF_CLICK || 
      ONOFF_LONG_CLICK || 
      ONOFF_CLICK_LONG_CLICK ||
      UP_CLICK ||
      UP_LONG_CLICK ||
      UP_CLICK_LONG_CLICK ||
      DOWN_CLICK ||
      DOWN_LONG_CLICK ||
      DOWN_CLICK_LONG_CLICK ||
      UP_DOWN_LONG_CLICK ||
      ONOFF_UP_LONG_CLICK ||
      ONOFF_DOWN_LONG_CLICK)
  {
    return 1;
  }
  else 
  {
    return 0;
  }
}

void buttons_clock (void)
{
  // clear all button events
  ONOFF_CLICK = 0;
  ONOFF_LONG_CLICK = 0;
  ONOFF_CLICK_LONG_CLICK = 0;

  UP_CLICK = 0;
  UP_LONG_CLICK = 0;
  UP_CLICK_LONG_CLICK = 0;

  DOWN_CLICK = 0;
  DOWN_LONG_CLICK = 0;
  DOWN_CLICK_LONG_CLICK = 0;

  UP_DOWN_LONG_CLICK = 0;
  ONOFF_UP_LONG_CLICK = 0;
  ONOFF_DOWN_LONG_CLICK = 0;
  
  
  // onoff button
  switch (ui8_onoff_button_state)
  {
    case 0:
    
      if (buttons_get_onoff_state())
      {
        ui8_onoff_button_state_counter = 0;
        ui8_onoff_button_state = 1;
      }
      
    break;

    case 1:
    
      ui8_onoff_button_state_counter++;

      // long click
      if (ui8_onoff_button_state_counter > LONG_CLICK_THRESHOLD)
      {
        if (ui8_up_button_state == 1)
        {
          ONOFF_UP_LONG_CLICK = 1;
          ui8_up_button_state = 2;
        } 
        else if (ui8_down_button_state == 1)
        {
          ONOFF_DOWN_LONG_CLICK = 1;
          ui8_down_button_state = 2;
        }
        else
        {
          ONOFF_LONG_CLICK = 1;
        }

        ui8_onoff_button_state = 2;
        
        break;
      }

      // button release before long click
      if (!buttons_get_onoff_state ())
      {
        // check for click + long click
        if (ui8_onoff_button_state_counter < CLICK_THRESHOLD)
        {
          ui8_onoff_button_state_counter = 0;
          ui8_onoff_button_state = 3;
          
          break;
        }
        else
        {
          ONOFF_CLICK = 1;
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
      if (ui8_onoff_button_state_counter > CLICK_THRESHOLD)
      {
        ONOFF_CLICK = 1;
        ui8_onoff_button_state = 0;
        
        break;
      }
      
    break;

    case 4:
    
      ui8_onoff_button_state_counter++;

      // event click, but this time it is: click + long click
      if (ui8_onoff_button_state_counter > CLICK_LONG_CLICK_THRESHOLD)
      {
        ONOFF_CLICK_LONG_CLICK = 1;
        ui8_onoff_button_state = 2;
        
        break;
      }

      // button release
      if (!buttons_get_onoff_state ())
      {
        ONOFF_CLICK = 1;
        ui8_onoff_button_state = 0;
        
        break;
      }
      
    break;

    default:
    
      ui8_onoff_button_state = 0;
      
    break;
  }
  
  // up button
  switch (ui8_up_button_state)
  {
    case 0:
      
      if (buttons_get_up_state())
      {
        ui8_up_button_state_counter = 0;
        ui8_up_button_state = 1;
      }
      
    break;

    case 1:
    
      ui8_up_button_state_counter++;

      // long click
      if (ui8_up_button_state_counter > LONG_CLICK_THRESHOLD)
      {
        if (ui8_down_button_state == 1)
        {
          UP_DOWN_LONG_CLICK = 1;
          ui8_down_button_state = 2;
        }
        else if (ui8_onoff_button_state == 1)
        {
          ONOFF_UP_LONG_CLICK = 1;
          ui8_onoff_button_state = 2;
        }
        else
        {
          UP_LONG_CLICK = 1;
        }

        ui8_up_button_state = 2;
        
        break;
      }

      // button release before long click
      if (!buttons_get_up_state ())
      {
        // check for click + long click
        if (ui8_up_button_state_counter <= CLICK_THRESHOLD)
        {
          ui8_up_button_state_counter = 0;
          ui8_up_button_state = 3;
          
          break;
        }
        else
        {
          UP_CLICK = 1;
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
      if (ui8_up_button_state_counter > CLICK_THRESHOLD)
      {
        UP_CLICK = 1;
        ui8_up_button_state = 0;
        
        break;
      }
      
    break;

    case 4:
    
      ui8_up_button_state_counter++;

      // event click, but this time it is: click + long click
      if (ui8_up_button_state_counter > CLICK_LONG_CLICK_THRESHOLD)
      {
        UP_CLICK_LONG_CLICK = 1;
        ui8_up_button_state = 2;
        
        break;
      }

      // button release
      if (!buttons_get_up_state ())
      {
        UP_CLICK = 1;
        ui8_up_button_state = 0;
        
        break;
      }
      
    break;

    default:
    
      ui8_up_button_state = 0;
      
    break;
  }
  
  // down button
  switch (ui8_down_button_state)
  {
    case 0:
      
      if (buttons_get_down_state())
      {
        ui8_down_button_state_counter = 0;
        ui8_down_button_state = 1;
      }
      
    break;

    case 1:
    
      ui8_down_button_state_counter++;

      // long click
      if (ui8_down_button_state_counter > LONG_CLICK_THRESHOLD)
      {
        if (ui8_up_button_state == 1)
        {
          UP_DOWN_LONG_CLICK = 1;
          ui8_up_button_state = 2;
        }
        else if (ui8_onoff_button_state == 1)
        {
          ONOFF_DOWN_LONG_CLICK = 1;
          ui8_onoff_button_state = 2;
        }
        else
        {
          DOWN_LONG_CLICK = 1;
        }

        ui8_down_button_state = 2;
        
        break;
      }


      // button release before long click
      if (!buttons_get_down_state ())
      {
        // check for click + long click
        if (ui8_down_button_state_counter < CLICK_THRESHOLD)
        {
          ui8_down_button_state_counter = 0;
          ui8_down_button_state = 3;
          
          break;
        }
        else
        {
          DOWN_CLICK = 1;
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
      if (ui8_down_button_state_counter > CLICK_THRESHOLD)
      {
        DOWN_CLICK = 1;
        ui8_down_button_state = 0;
        
        break;
      }
      
    break;

    case 4:
    
      ui8_down_button_state_counter++;

      // event click, but this time it is: click + long click
      if (ui8_down_button_state_counter > CLICK_LONG_CLICK_THRESHOLD)
      {
        DOWN_CLICK_LONG_CLICK = 1;
        ui8_down_button_state = 2;
        
        break;
      }

      // button release
      if (!buttons_get_down_state ())
      {
        DOWN_CLICK = 1;
        ui8_down_button_state = 0;
        
        break;
      }
      
    break;

    default:
    
      ui8_down_button_state = 0;
      
    break;
  }
}
