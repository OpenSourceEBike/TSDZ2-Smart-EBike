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

#define EBIKE_APP_STATE_MOTOR_COAST     0
#define EBIKE_APP_STATE_MOTOR_STOP      1
#define EBIKE_APP_STATE_MOTOR_STARTUP   2
#define EBIKE_APP_STATE_MOTOR_COOL      3
#define EBIKE_APP_STATE_MOTOR_RUNNING   4

#define WALK_ASSIST_PI_CONTROLLER_KP_DIVIDEND	20
#define WALK_ASSIST_PI_CONTROLLER_KP_DIVISOR	4
#define WALK_ASSIST_PI_CONTROLLER_KI_DIVIDEND	3
#define WALK_ASSIST_PI_CONTROLLER_KI_DIVISOR	6

typedef struct _configuration_variables
{
  uint8_t ui8_power_regular_state_div25;
  uint8_t ui8_battery_max_current;
  uint8_t ui8_motor_power_x10;
  uint16_t ui16_battery_low_voltage_cut_off_x10;
  uint16_t ui16_wheel_perimeter;
  uint8_t ui8_lights;
  uint8_t ui8_walk_assist;
  uint8_t ui8_walk_assist_erps;
  uint8_t ui8_walk_assist_erps_up;
  uint8_t ui8_walk_assist_erps_down;
  uint8_t ui8_offroad_mode;
  uint8_t ui8_wheel_max_speed;
  uint8_t ui8_pas_max_cadence;
  uint8_t ui8_motor_type;
  uint8_t ui8_motor_assistance_startup_without_pedal_rotation;
  uint8_t ui8_target_battery_max_power_div25;
  uint8_t ui8_cruise_control;
  uint8_t configuration_variables;
  uint8_t ui8_startup_motor_power_boost_feature_enabled;
  uint8_t ui8_startup_motor_power_boost_div25;
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
  uint8_t ui8_offroad_feature_enabled;
  uint8_t ui8_offroad_enabled_on_startup;
  uint8_t ui8_offroad_speed_limit;
  uint8_t ui8_offroad_power_limit_enabled;
  uint8_t ui8_offroad_power_limit_div25;
} struct_configuration_variables;

extern volatile uint8_t ui8_adc_torque_sensor_min_value;
extern volatile uint8_t ui8_adc_torque_sensor_max_value;
extern volatile uint8_t ui8_adc_battery_current_offset;
extern volatile uint8_t ui8_ebike_app_state;
extern volatile uint8_t ui8_adc_target_battery_max_current;

extern volatile uint16_t ui16_pas_pwm_cycles_ticks;
extern volatile uint8_t ui8_pas_direction;
extern uint8_t ui8_pas_cadence_rpm;

extern volatile uint32_t ui32_wheel_speed_sensor_pwm_cycles_ticks;
extern volatile uint8_t ui8_wheel_speed_sensor_is_disconnected;

extern volatile uint32_t ui32_wheel_speed_sensor_tick_counter;

void ebike_app_init (void);
void ebike_app_controller (void);
struct_configuration_variables* get_configuration_variables (void);

#endif /* _EBIKE_APP_H_ */
