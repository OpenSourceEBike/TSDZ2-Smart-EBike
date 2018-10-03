/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _PINS_H_
#define _PINS_H_

#include "stm8s_gpio.h"

/* Connections:
 *
 * Motor PHASE_A: blue wire
 * Motor PHASE_B: green wire
 * Motor PHASE_C: yellow wire
 *
 * The battery_current is measured using the LM385 opamp in an non inverting configuration. The pin 1 is the output and has a low pass filter.
 * The pin 3 (+) has the signal input and pin 2 (-) has the feedback loop, composed of R1 = 11k and R2 = 1k.
 * The gain is: (R1 / R2) + 1 = (11k / 1k) + 1 = 12.
 * We know that 1 Amp of battery current is equal to 5 ADC_12bits steps, so 1 Amp = (5V / 1024) * 5 = 0.0244V.
 * Each 1 Amp at the shunt is then 0.0244V / 12 = 0.002V. This also means shunt should has 0.002 ohms resistance.
 * Since there is a transistor that has a base resistor connected throught a 1K resisitor to the shunt voltage, and also the base has
 * another connected resistor of 27K, I think the transistor will switch on at arround 0.5V on the shunt voltage and that means arround 22 amps.
 * The microcontroller should read the turned on transistor signal on PD0, to detect the battery_over_current of 22 amps. (?????)
 *
 * PIN                | IN/OUT|Function
 * ----------------------------------------------------------
 * PD0                | in  | battery_over_current (PD0 on original firmware configured as: Port D0 alternate function = TIM1_BKIN)
 * PB4  (ADC_AIN4)    | in  | torque sensor signal, this signal is amplified by the opamp
 * PB5  (ADC_AIN5)    | in  | battery_current (14 ADC bits step per 1 amp; this signal amplified by the opamp 358)
 * PB6  (ADC_AIN6)    | in  | battery_voltage (0.344V per ADC 8bits step: 17.9V --> ADC_10bits = 52; 40V --> ADC_10bits = 116; this signal atenuated by the opamp 358)
 *
 * PE5                | in  | Hall_sensor_A
 * PD2                | in  | Hall_sensor_B
 * PC5                | in  | Hall_sensor_C
 *
 * PB2  (TIM1_CH3N)   | out | PWM_phase_A_low
 * PB1  (TIM1_CH2N)   | out | PWM_phase_B_low
 * PB0  (TIM1_CH1N)   | out | PWM_phase_C_low
 * PC3  (TIM1_CH3)    | out | PWM_phase_A_high
 * PC2  (TIM1_CH2)    | out | PWM_phase_B_high
 * PC1  (TIM1_CH1)    | out | PWM_phase_C_high
 *
 * PD5  (UART2_TX)    | out | usart_tx
 * PD6  (UART2_RX)    | in  | usart_rx
 *
 * PC6                | in  | brake
 * PB7  (ADC_AIN7)    | in  | throttle
 * PD7                | in  | PAS1 (yellow wire)
 * PE0                | in  | PAS2 (blue wire)
 * PA1                | in  | wheel speed
 *
 * PD3                | out | torque sensor excitation
 * PB3  (ADC_AIN3)    | in  | ???? realted to torque sensor ???
 * PD4                | out | lights. Enable/disable 5V output of the circuit that powers the lights wire of 6V.
 *
 * PE6  (ADC_AIN9)    | in  | this signal goes to a pad of a resistor that is not assembled. If was assembled, it would measure the opamp output value related to AIN4.
 *
 * PE1  tested as input on original firmware | External interrupt enabled
 * PE2  tested as input on original firmware
 * PD1  tested as input on original firmware | External interrupt enabled
 *
 * PA2  tested as input on original firmware
 * PA4  configured as output on original firmware, original firmware seems to only put it at 1 / enable
 * PA5  tested as input on original firmware
 * PA6  configured as output on original firmware, original firmware seems to only put it at 1 / enable
 *
 */

#define HALL_SENSOR_A__PORT       GPIOE
#define HALL_SENSOR_A__PIN        GPIO_PIN_5
#define HALL_SENSOR_B__PORT       GPIOD
#define HALL_SENSOR_B__PIN        GPIO_PIN_2
#define HALL_SENSOR_C__PORT       GPIOC
#define HALL_SENSOR_C__PIN        GPIO_PIN_5

#define PMW_PHASE_A_LOW__PORT     GPIOB
#define PMW_PHASE_A_LOW__PIN      GPIO_PIN_2
#define PMW_PHASE_B_LOW__PORT     GPIOB
#define PMW_PHASE_B_LOW__PIN      GPIO_PIN_1
#define PMW_PHASE_C_LOW__PORT     GPIOB
#define PMW_PHASE_C_LOW__PIN      GPIO_PIN_0
#define PMW_PHASE_A_HIGH__PORT    GPIOC
#define PMW_PHASE_A_HIGH__PIN     GPIO_PIN_3
#define PMW_PHASE_B_HIGH__PORT    GPIOC
#define PMW_PHASE_B_HIGH__PIN     GPIO_PIN_2
#define PMW_PHASE_C_HIGH__PORT    GPIOC
#define PMW_PHASE_C_HIGH__PIN     GPIO_PIN_1

#define UART2_TX__PORT            GPIOD
#define UART2_TX__PIN             GPIO_PIN_5
#define UART2_RX__PORT            GPIOD
#define UART2_RX__PIN             GPIO_PIN_6

#define BRAKE__PORT               GPIOC
#define BRAKE__PIN                GPIO_PIN_6

#define PAS1__PORT                GPIOE
#define PAS1__PIN                 GPIO_PIN_0
#define PAS2__PORT                GPIOD
#define PAS2__PIN                 GPIO_PIN_7

#define WHEEL_SPEED_SENSOR__PORT  GPIOA
#define WHEEL_SPEED_SENSOR__PIN   GPIO_PIN_1

#define TORQUE_SENSOR_EXCITATION__PORT  GPIOD
#define TORQUE_SENSOR_EXCITATION__PIN   GPIO_PIN_3

#define TORQUE_SENSOR__PORT       GPIOB
#define TORQUE_SENSOR__PIN        GPIO_PIN_3

#define LIGHTS__PORT              GPIOD
#define LIGHTS__PIN               GPIO_PIN_4

#define THROTTLE__PORT            GPIOB
#define THROTTLE__PIN             GPIO_PIN_7

#define BATTERY_CURRENT__PORT     GPIOB
#define BATTERY_CURRENT__PIN      GPIO_PIN_5

#endif /* _PINS_H_ */
