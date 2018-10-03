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

uint8_t get_button_up_state (void);
uint8_t get_button_up_click_event (void);
uint8_t get_button_up_long_click_event (void);
void clear_button_up_click_event (void);
void clear_button_up_long_click_event (void);
uint8_t get_button_down_state (void);
uint8_t get_button_down_click_event (void);
uint8_t get_button_down_long_click_event (void);
void clear_button_down_click_event (void);
void clear_button_down_long_click_event (void);
uint8_t get_button_onoff_state (void);
uint8_t get_button_onoff_click_event (void);
uint8_t get_button_onoff_long_click_event (void);
void clear_button_onoff_click_event (void);
void clear_button_onoff_long_click_event (void);
uint8_t get_button_up_down_click_event (void);
void clear_button_up_down_click_event (void);
void clock_button (void);
uint8_t button_get_events (void);
void button_clear_events (void);

#endif /* _BUTTON_H_ */
