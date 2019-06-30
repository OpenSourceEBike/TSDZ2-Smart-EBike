/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include <stdint.h>

// motor states
#define BLOCK_COMMUTATION 			                1
#define SINEWAVE_INTERPOLATION_60_DEGREES 	    2

extern volatile uint8_t ui8_g_duty_cycle;
extern volatile uint8_t ui8_g_adc_motor_phase_current_offset;
extern volatile uint8_t ui8_g_adc_battery_current;
extern volatile uint8_t ui8_g_foc_angle;

void hall_sensor_init (void); // must be called before using the motor
void motor_init (void); // must be called before using the motor

void motor_enable_PWM (void);
void motor_disable_PWM (void);

uint16_t ui16_motor_get_motor_speed_erps (void);
void motor_controller (void);

uint8_t motor_get_adc_battery_current_filtered_10b (void);
uint16_t motor_get_adc_battery_voltage_filtered_10b (void);
void motor_set_adc_battery_voltage_cut_off (uint8_t ui8_value);

void motor_enable_pwm(void);
void motor_disable_pwm(void);

#endif /* _MOTOR_H_ */
