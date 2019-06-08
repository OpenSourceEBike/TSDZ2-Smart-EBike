/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include "main.h"
#include "stm8s_gpio.h"

extern uint8_t ONOFF_CLICK;
extern uint8_t ONOFF_LONG_CLICK;
extern uint8_t ONOFF_CLICK_LONG_CLICK;

extern uint8_t UP_CLICK;
extern uint8_t UP_LONG_CLICK;
extern uint8_t UP_CLICK_LONG_CLICK;

extern uint8_t DOWN_CLICK;
extern uint8_t DOWN_LONG_CLICK;
extern uint8_t DOWN_CLICK_LONG_CLICK;

extern uint8_t UP_DOWN_LONG_CLICK;
extern uint8_t ONOFF_UP_LONG_CLICK;
extern uint8_t ONOFF_DOWN_LONG_CLICK;

uint8_t buttons_get_up_state (void);
uint8_t buttons_get_down_state (void);
uint8_t buttons_get_onoff_state (void);
uint8_t buttons_get_events (void);
void buttons_clock(void);

#endif /* _BUTTON_H_ */
