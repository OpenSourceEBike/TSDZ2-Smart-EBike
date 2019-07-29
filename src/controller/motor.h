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


// power variables
extern volatile uint16_t ui16_controller_duty_cycle_ramp_up_inverse_step;
extern volatile uint16_t ui16_controller_duty_cycle_ramp_down_inverse_step;
extern volatile uint16_t ui16_adc_battery_voltage_filtered;
extern volatile uint8_t ui8_adc_battery_voltage_cut_off;
extern volatile uint8_t ui8_adc_battery_current_filtered;
extern volatile uint8_t ui8_controller_adc_battery_current_target;
extern volatile uint8_t ui8_g_duty_cycle;
extern volatile uint8_t ui8_controller_duty_cycle_target;
extern volatile uint8_t ui8_g_foc_angle;


// cadence sensor
extern volatile uint16_t ui16_cadence_sensor_ticks;
extern volatile uint16_t ui16_cadence_sensor_ticks_counter_min_high;
extern volatile uint16_t ui16_cadence_sensor_ticks_counter_min_low;
extern volatile uint16_t ui16_cadence_sensor_conversion_x100;
extern volatile uint16_t ui16_cadence_sensor_conversion_x100_high;
extern volatile uint16_t ui16_cadence_sensor_conversion_x100_low;


// wheel speed sensor
extern volatile uint16_t ui16_wheel_speed_sensor_ticks;
extern volatile uint32_t ui32_wheel_speed_sensor_ticks_total;


void hall_sensor_init (void); // must be called before using the motor
void motor_enable_PWM (void);
void motor_disable_PWM (void);
void motor_enable_pwm(void);
void motor_disable_pwm(void);
uint16_t ui16_motor_get_motor_speed_erps (void);
void motor_controller (void);

uint8_t motor_get_adc_battery_current_filtered_10b (void);

#endif /* _MOTOR_H_ */
