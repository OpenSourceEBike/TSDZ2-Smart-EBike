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
extern volatile uint16_t ui16_g_adc_motor_current_offset;
extern volatile uint16_t ui16_g_adc_battery_current;
extern volatile uint16_t ui16_g_adc_motor_current;
extern volatile uint8_t ui8_g_foc_angle;
extern volatile uint8_t ui8_g_pas_pedal_right;
extern volatile uint8_t ui8_g_hall_sensors_state;
extern volatile uint16_t ui16_main_loop_wdt_cnt_1;
extern volatile uint16_t ui16_g_adc_target_battery_max_current;
extern volatile uint16_t ui16_g_adc_target_battery_max_current_fw;
extern volatile uint16_t ui16_g_adc_target_motor_max_current;
extern volatile uint16_t ui16_g_adc_target_motor_max_current_fw;
extern volatile uint16_t ui16_g_adc_battery_current_filtered;
extern volatile uint16_t ui16_g_adc_motor_current_filtered;
extern volatile uint8_t ui8_g_field_weakening_angle;
extern volatile uint8_t ui8_g_field_weakening_enable;
extern volatile uint8_t ui8_g_field_weakening_enable_state;

/***************************************************************************************/
// Motor interface
void hall_sensor_init (void); // must be called before using the motor
void motor_init (void); // must be called before using the motor
void motor_enable_PWM (void);
void motor_disable_PWM (void);
void motor_set_pwm_duty_cycle_target (uint8_t value);
void motor_set_pwm_duty_cycle_ramp_up_inverse_step (uint16_t value); // each step = 64us
void motor_set_pwm_duty_cycle_ramp_down_inverse_step (uint16_t value); // each step = 64us
uint16_t ui16_motor_get_motor_speed_erps (void);
void motor_set_pwm_duty_cycle_target (uint8_t ui8_value);
void motor_controller (void);
void motor_set_adc_battery_voltage_cut_off(uint8_t ui8_value);
uint16_t motor_get_adc_battery_voltage_filtered_10b(void);
void motor_enable_pwm(void);
void motor_disable_pwm(void);
/***************************************************************************************/

#endif /* _MOTOR_H_ */
