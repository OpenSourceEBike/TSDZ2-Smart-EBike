/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _LCD_H_
#define _LCD_H_

#include "main.h"
#include "stm8s_gpio.h"

//ui8_rx_buffer[2] == 8 if torque sensor
//ui8_rx_buffer[2] == 4 if motor running
typedef struct _motor_controller_data
{
  uint16_t ui16_adc_battery_voltage;
  uint8_t ui8_battery_current_x5;
  uint8_t ui8_motor_controller_state_1;
  uint8_t ui8_adc_throttle;
  uint8_t ui8_throttle;
  uint8_t ui8_adc_pedal_torque_sensor;
  uint8_t ui8_pedal_torque_sensor;
  uint8_t ui8_pedal_human_power;
  uint8_t ui8_duty_cycle;
  uint8_t ui8_error_code;
  uint16_t ui16_wheel_speed_x10;
  uint8_t ui8_motor_controller_state_2;
  uint8_t ui8_braking;
  uint8_t ui8_pedal_cadence;
  uint8_t ui8_lights;
  uint8_t ui8_walk_assist_level;
  uint8_t ui8_offroad_mode;
  uint16_t ui16_motor_speed_erps;
  uint8_t ui8_foc_angle;
  uint8_t ui8_temperature_current_limiting_value;
  uint8_t ui8_motor_temperature;
  uint32_t ui32_wheel_speed_sensor_tick_counter;
} struct_motor_controller_data;

typedef struct _configuration_variables
{
  uint8_t ui8_assist_level;
  uint8_t ui8_number_of_assist_levels;
  uint16_t ui16_wheel_perimeter;
  uint8_t ui8_wheel_max_speed;
  uint8_t ui8_units_type;
  uint32_t ui32_wh_x10_offset;
  uint32_t ui32_wh_x10_100_percent;
  uint8_t ui8_show_numeric_battery_soc;
  uint8_t ui8_odometer_field_state;
  uint8_t ui8_target_max_battery_power;
  uint8_t ui8_battery_cells_number;
  uint8_t ui8_battery_max_current;
  uint16_t ui16_battery_low_voltage_cut_off_x10;
  uint16_t ui16_battery_voltage_reset_wh_counter_x10;
  uint16_t ui16_battery_pack_resistance_x1000;
  uint8_t ui8_motor_type;
  uint8_t ui8_motor_assistance_startup_without_pedal_rotation;
  uint8_t ui8_pas_max_cadence;
  uint8_t ui8_cruise_control;
  uint8_t ui8_assist_level_power [9];
  uint8_t ui8_startup_motor_power_boost_feature_enabled;
  uint8_t ui8_startup_motor_power_boost_state;
  uint8_t ui8_startup_motor_power_boost_time;
  uint8_t ui8_startup_motor_power_boost_fade_time;
  uint8_t ui8_startup_motor_power_boost [9];
  uint16_t ui16_adc_motor_temperature_10b;
  uint8_t ui8_temperature_limit_feature_enabled;
  uint8_t ui8_motor_temperature_min_value_to_limit;
  uint8_t ui8_motor_temperature_max_value_to_limit;
  uint8_t ui8_temperature_field_config;
  uint8_t ui8_lcd_power_off_time_minutes;
  uint8_t ui8_lcd_backlight_on_brightness;
  uint8_t ui8_lcd_backlight_off_brightness;
  uint8_t ui8_offroad_feature_enabled;
  uint8_t ui8_offroad_enabled_on_startup;
  uint8_t ui8_offroad_speed_limit;
  uint8_t ui8_offroad_power_limit_enabled;
  uint8_t ui8_offroad_power_limit_div25;
  uint16_t ui16_odometer_distance_x10;
  uint32_t ui32_odometer_x10;
} struct_configuration_variables;

// LCD RAM has 32*8 bits
#define LCD_FRAME_BUFFER_SIZE 32

extern uint8_t ui8_lcd_frame_buffer[LCD_FRAME_BUFFER_SIZE];

#define ASSIST_LEVEL_FIELD     0
#define ODOMETER_FIELD         1
#define TEMPERATURE_FIELD      2
#define WHEEL_SPEED_FIELD      3
#define BATTERY_POWER_FIELD    4

// each digit needs 7 bits to be defined + 1 digit that can be another symbol like a "point"
#define ASSIST_LEVEL_DIGIT_OFFSET     1 // 8
#define ODOMETER_DIGIT_OFFSET         6
#define TEMPERATURE_DIGIT_OFFSET      8
#define WHEEL_SPEED_OFFSET            14
#define BATTERY_POWER_DIGIT_OFFSET    10

#define NUMBERS_MASK              8
#define NUMBER_0_MASK             119
#define NUMBER_1_MASK             66  // 2; 7
#define NUMBER_2_MASK             182 // 3; 2; 8; 6; 5
#define NUMBER_3_MASK             214
#define NUMBER_4_MASK             195
#define NUMBER_5_MASK             213
#define NUMBER_6_MASK             245
#define NUMBER_7_MASK             70
#define NUMBER_8_MASK             247
#define NUMBER_9_MASK             215
#define NUMBER_0_MASK_INVERTED    119
#define NUMBER_1_MASK_INVERTED    33  // 2; 7
#define NUMBER_2_MASK_INVERTED    182 // 3; 2; 8; 6; 5
#define NUMBER_3_MASK_INVERTED    181
#define NUMBER_4_MASK_INVERTED    225
#define NUMBER_5_MASK_INVERTED    213
#define NUMBER_6_MASK_INVERTED    215
#define NUMBER_7_MASK_INVERTED    49
#define NUMBER_8_MASK_INVERTED    247
#define NUMBER_9_MASK_INVERTED    245

// : from timer label ui8_lcd_frame_buffer[23] |= 8

void lcd_init (void);
void clock_lcd (void);
struct_configuration_variables* get_configuration_variables (void);
struct_motor_controller_data* lcd_get_motor_controller_data (void);
void automatic_power_off_counter_reset (void);

#endif /* _LCD_H_ */
