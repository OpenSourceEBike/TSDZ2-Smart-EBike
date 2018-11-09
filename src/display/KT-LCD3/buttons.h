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

typedef enum
{
    ONOFF_CLICK = 1,
    ONOFF_CLICK_CLICK = 2,
    ONOFF_CLICK_AND_LONG_CLICK = 4,
    ONOFF_LONG_CLICK = 8,
    ON_CLICK = 16,
    ON_CLICK_CLICK = 32,
    ON_CLICK_AND_LONG_CLICK = 64,
    ON_LONG_CLICK = 128,
    OFF_CLICK = 256,
    OFF_CLICK_CLICK = 512,
    OFF_CLICK_AND_LONG_CLICK = 1024,
    OFF_LONG_CLICK = 2048,
} buttons_events_type_t;

extern buttons_events_type_t old_buttons_events;

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
