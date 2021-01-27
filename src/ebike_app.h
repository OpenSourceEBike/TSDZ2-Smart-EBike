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

typedef struct
{
  uint16_t ui16_assist_level_factor_x1000;
  uint8_t ui8_battery_max_current;
  uint8_t ui8_motor_power_x10;
  uint16_t ui16_battery_low_voltage_cut_off_x10;
  uint16_t ui16_wheel_perimeter;
  uint8_t ui8_lights;
  uint8_t ui8_walk_assist;
  uint8_t ui8_wheel_max_speed;
  uint8_t ui8_motor_type;
  uint8_t ui8_motor_assistance_startup_without_pedal_rotation;
  uint8_t ui8_motor_current_control_mode;
  uint8_t ui8_target_battery_max_power_div25;
  uint8_t configuration_variables;
  uint8_t ui8_startup_motor_power_boost_feature_enabled;
  uint16_t ui16_startup_motor_power_boost_assist_level;
  uint8_t ui8_startup_motor_power_boost_always;
  uint8_t ui8_startup_motor_power_boost_limit_to_max_power;
  uint8_t ui8_startup_motor_power_boost_time;
  uint8_t ui8_startup_motor_power_boost_fade_time;
  uint8_t ui8_temperature_limit_feature_enabled;
  uint8_t ui8_motor_temperature_min_value_to_limit;
  uint8_t ui8_motor_temperature_max_value_to_limit;
  uint16_t ui16_motor_temperature_x2;
  uint8_t ui8_motor_temperature;
  uint8_t ui8_ramp_up_amps_per_second_x10;
  uint8_t ui8_torque_sensor_calibration_pedal_ground;
  uint8_t ui8_torque_sensor_calibration_feature_enabled;
  uint8_t ui8_battery_current_min_adc;
  uint8_t ui8_throttle_virtual;
  uint8_t ui8_torque_sensor_filter;
  uint8_t ui8_torque_sensor_adc_threshold;
} struct_config_vars;



extern volatile uint16_t ui16_g_current_ramp_up_inverse_step;
extern volatile uint16_t ui16_g_adc_torque_sensor_min_value;
extern volatile uint8_t ui8_g_ebike_app_state;
extern volatile uint8_t ui8_g_adc_coast_brake_torque_threshold;
extern volatile uint8_t ui8_g_coast_brake_enable;
extern volatile uint8_t ui8_g_pedal_cadence_fast_stop;
extern volatile uint16_t ui16_g_adc_torque_sensor_raw_horizontal;
extern volatile uint8_t ui8_g_torque_sensor_horizontal_cnt;

extern volatile uint16_t ui16_g_pas_pwm_cycles_ticks;

extern volatile uint16_t ui16_wheel_speed_sensor_pwm_cycles_ticks;
extern volatile uint8_t ui8_wheel_speed_sensor_is_disconnected;

extern volatile uint32_t ui32_wheel_speed_sensor_tick_counter;

extern uint8_t ui8_g_brake_is_set;
extern volatile uint8_t ui8_g_throttle;

extern volatile uint16_t ui16_g_adc_current_offset;

void ebike_app_controller (void);
void reset_rx_buffer(void);
struct_config_vars* get_configuration_variables (void);

#endif /* _EBIKE_APP_H_ */
