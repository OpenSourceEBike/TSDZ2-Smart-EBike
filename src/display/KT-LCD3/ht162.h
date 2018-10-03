/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _HT162_H_
#define _HT162_H_

#include "main.h"
#include "stm8s_gpio.h"

// HT1622 commands (Start with 0b100, ends with one "don't care" byte)
#define  CMD_SYS_DIS  0x00  // SYS DIS    (0000-0000-X) Turn off system oscillator, LCD bias gen [Default]
#define  CMD_SYS_EN   0x01  // SYS EN     (0000-0001-X) Turn on  system oscillator
#define  CMD_LCD_OFF  0x02  // LCD OFF    (0000-0010-X) Turn off LCD display [Default]
#define  CMD_LCD_ON   0x03  // LCD ON     (0000-0011-X) Turn on  LCD display
#define  CMD_RC_INT   0x10  // RC INT     (0001-10XX-X) System clock source, on-chip RC oscillator

void ht1622_init (void);
void ht1622_send_frame_buffer (uint8_t *ui8_lcd_frame_buffer);

#endif /* _HT162_H_ */
