/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <string.h>
#include "stm8s.h"
#include "stm8s_gpio.h"
#include "gpio.h"
#include "pins.h"
#include "timers.h"
#include "ht162.h"
#include "lcd.h"

void ht1622_send_bits(uint16_t ui16_data, uint8_t ui8_bits);
void ht1622_send_command(uint8_t command);

void ht1622_init (void)
{
  GPIO_WriteHigh(GPIOB, GPIO_PIN_4); // enable VDD to HT1622 ??
//  GPIO_WriteHigh(GPIOE, GPIO_PIN_3); // only needed for 24V battery pack

  GPIO_Init(LCD3_ONOFF_POWER__PORT,
            LCD3_ONOFF_POWER__PIN,
            GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_WriteHigh(LCD3_ONOFF_POWER__PORT, LCD3_ONOFF_POWER__PIN);

  GPIO_WriteHigh(LCD3_ENABLE_BACKLIGHT_POWER__PORT, LCD3_ENABLE_BACKLIGHT_POWER__PIN);
  GPIO_WriteLow(LCD3_ENABLE_BACKLIGHT__PORT, LCD3_ENABLE_BACKLIGHT__PIN);

  GPIO_Init(LCD3_HT1622_CS__PORT,
            LCD3_HT1622_CS__PIN,
            GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_WriteHigh(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);

  GPIO_Init(LCD3_HT1622_WRITE__PORT,
            LCD3_HT1622_WRITE__PIN,
            GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_WriteHigh(LCD3_HT1622_WRITE__PORT, LCD3_HT1622_WRITE__PIN);

  GPIO_Init(LCD3_HT1622_DATA__PORT,
            LCD3_HT1622_DATA__PIN,
            GPIO_MODE_OUT_PP_LOW_FAST);
  GPIO_WriteHigh(LCD3_HT1622_DATA__PORT, LCD3_HT1622_DATA__PIN);

  GPIO_Init(LCD3_HT1622_READ__PORT,
            LCD3_HT1622_READ__PIN,
            GPIO_MODE_IN_PU_NO_IT);

  ht1622_send_command(CMD_SYS_EN);
  ht1622_send_command(CMD_RC_INT);
  ht1622_send_command(CMD_LCD_ON);
}

void ht1622_send_bits(uint16_t ui16_data, uint8_t ui8_bits)
{
  static uint16_t ui16_mask;

  ui16_mask = 1 << (ui8_bits - 1);

  for (uint8_t i = ui8_bits; i > 0; i--)
  {
    GPIO_WriteLow(LCD3_HT1622_WRITE__PORT, LCD3_HT1622_WRITE__PIN);

    if (ui16_data & ui16_mask)
    {
      GPIO_WriteHigh(LCD3_HT1622_DATA__PORT, LCD3_HT1622_DATA__PIN);
    }
    else
    {
      GPIO_WriteLow(LCD3_HT1622_DATA__PORT, LCD3_HT1622_DATA__PIN);
    }

    GPIO_WriteHigh(LCD3_HT1622_WRITE__PORT, LCD3_HT1622_WRITE__PIN);

    ui16_data <<= 1;
  }
}

void ht1622_send_command(uint8_t command)
{
  GPIO_WriteLow(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);
  ht1622_send_bits(4, 3);
  ht1622_send_bits(command, 8);
  ht1622_send_bits(1, 1);
  GPIO_WriteHigh(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);
}

void ht1622_send_frame_buffer (uint8_t *p_lcd_frame_buffer)
{
  uint8_t ui8_len;
  uint8_t ui8_counter = 0;
  uint8_t ui8_data = 0;
  uint8_t ui8_lcd_frame_buffer_index = 0;

  // send first address and first 4 bits
  ui8_data = p_lcd_frame_buffer[ui8_lcd_frame_buffer_index];
  GPIO_WriteLow(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);
  ht1622_send_bits(5, 3);
  ht1622_send_bits(0, 6); // start at 0 address
  ht1622_send_bits(ui8_data, 4);
  ui8_counter++;

  // send the rest of the frame buffer
  ui8_len = (LCD_FRAME_BUFFER_SIZE << 1) - 1;
  while (ui8_len > 0)
  {
    ui8_len--;

    ui8_counter++;
    if (ui8_counter == 2)
    {
      ui8_counter = 0;
      ui8_data = p_lcd_frame_buffer[ui8_lcd_frame_buffer_index] >> 4;
    }
    else
    {
      ui8_lcd_frame_buffer_index++;
      ui8_data = p_lcd_frame_buffer[ui8_lcd_frame_buffer_index];
    }

    ht1622_send_bits(ui8_data, 4);
  }

  GPIO_WriteHigh(LCD3_HT1622_CS__PORT, LCD3_HT1622_CS__PIN);
}
