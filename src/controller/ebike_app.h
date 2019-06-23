/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho and EndlessCadence, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EBIKE_APP_H_
#define _EBIKE_APP_H_

#include <stdint.h>
#include "main.h"
#include "common/common.h"

typedef struct _configuration_variables
{
  uint8_t ui8_assist_level_factor_x10;
  uint8_t ui8_battery_max_current;
  uint8_t ui8_motor_power_x10;
  uint16_t ui16_battery_low_voltage_cut_off_x10;
  uint16_t ui16_wheel_perimeter;
  uint8_t ui8_lights;
  uint8_t ui8_wheel_max_speed;
  uint8_t ui8_motor_type;
  uint8_t ui8_motor_assistance_startup_without_pedal_rotation;
  uint8_t ui8_target_battery_max_power_div25;
  uint8_t configuration_variables;
  uint8_t ui8_startup_motor_power_boost_feature_enabled;
  uint8_t ui8_startup_motor_power_boost_assist_level;
  uint8_t ui8_startup_motor_power_boost_state;
  uint8_t ui8_startup_motor_power_boost_limit_to_max_power;
  uint8_t ui8_startup_motor_power_boost_time;
  uint8_t ui8_startup_motor_power_boost_fade_time;
  uint8_t ui8_temperature_limit_feature_enabled;
  uint8_t ui8_motor_temperature_min_value_to_limit;
  uint8_t ui8_motor_temperature_max_value_to_limit;
  uint8_t ui8_temperature_current_limiting_value;
  uint16_t ui16_motor_temperature_x2;
  uint8_t ui8_motor_temperature;
  uint8_t ui8_ramp_up_amps_per_second;
} struct_configuration_variables;



extern volatile uint8_t ui8_adc_pedal_torque_offset;
extern volatile uint8_t ui8_adc_battery_current_offset;
extern volatile uint8_t ui8_controller_adc_battery_current_target;
extern volatile uint8_t ui8_controller_duty_cycle_target;
extern volatile uint16_t ui16_current_ramp_up_inverse_step;
extern volatile uint16_t ui16_pas_pwm_cycles_ticks;
extern volatile uint8_t ui8_g_pedaling_direction;
//extern uint8_t ui8_cadence_rpm;
extern volatile uint16_t ui16_wheel_speed_sensor_pwm_cycles_ticks;
extern volatile uint8_t ui8_wheel_speed_sensor_is_disconnected;
extern volatile uint32_t ui32_wheel_speed_sensor_tick_counter;

void ebike_app_controller (void);
struct_configuration_variables* get_configuration_variables (void);

#endif /* _EBIKE_APP_H_ */
