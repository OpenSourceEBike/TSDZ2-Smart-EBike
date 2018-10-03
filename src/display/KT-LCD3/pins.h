/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

// Battery voltage:
// 30.0V --> 447 | 0.0671 volts per each ADC unit
// 40.0V --> 595 | 0.0672 volts per each ADC unit

/* Connections:
 *
 * PIN		      | IN/OUT|Function
 * ----------------------------------------------------------
 * PD0          | HT1622 /CS
 * PA4          | HT1622 /READ
 * PB7          | HT1622 /WRITE
 * PC5          | HT1622 DATA
 *
 * PA2          | Enable power for LCD backlight (switch a very small mosfet to GND that goes to A1013 emitter pin)
 * PC4          | enables LCD backlight
 *
 * PB2          | /button down
 * PB1          | /button up
 * PB0          | /button ON/OFF

 *
 *
 * PE6 (AIN9)   | connects to header J6 (this header is not connected and has 2 pads: GND and  PE6)
 * PE7 (AIN8)   | battery voltage
 *
 * PE3          | seems to connect to a very small mosfet that enables battery voltage to some place, maybe to enable the motor controller?
 * PG1          | input digital from a mosfet output, enabled buy the LCD Power ON button
 *
 * PB4          | connects to HT1622 VDD pin 8

 *
 */

#ifndef _PINS_H_
#define _PINS_H_

#include "stm8s_gpio.h"

#define LCD3_ENABLE_BACKLIGHT__PORT             GPIOC
#define LCD3_ENABLE_BACKLIGHT__PIN              GPIO_PIN_4

#define LCD3_ENABLE_BACKLIGHT_POWER__PORT       GPIOA
#define LCD3_ENABLE_BACKLIGHT_POWER__PIN        GPIO_PIN_2

#define LCD3_BUTTON_UP__PORT                    GPIOB
#define LCD3_BUTTON_UP__PIN                     GPIO_PIN_1
#define LCD3_BUTTON_ONOFF__PORT                 GPIOG
#define LCD3_BUTTON_ONOFF__PIN                  GPIO_PIN_1
#define LCD3_BUTTON_DOWN__PORT                  GPIOB
#define LCD3_BUTTON_DOWN__PIN                   GPIO_PIN_2

#define LCD3_HT1622_CS__PORT                    GPIOD
#define LCD3_HT1622_CS__PIN                     GPIO_PIN_0
#define LCD3_HT1622_READ__PORT                  GPIOA
#define LCD3_HT1622_READ__PIN                   GPIO_PIN_4
#define LCD3_HT1622_WRITE__PORT                 GPIOB
#define LCD3_HT1622_WRITE__PIN                  GPIO_PIN_7
#define LCD3_HT1622_DATA__PORT                  GPIOC
#define LCD3_HT1622_DATA__PIN                   GPIO_PIN_5

#define LCD3_ONOFF_POWER__PORT                  GPIOA
#define LCD3_ONOFF_POWER__PIN                   GPIO_PIN_2

#endif /* _PINS_H_ */
