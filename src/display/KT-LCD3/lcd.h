/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho and Leon, 2019.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _LCD_H_
#define _LCD_H_

#include "main.h"
#include "stm8s_gpio.h"

typedef struct _motor_controller_data
{
  uint8_t ui8_riding_mode;
  uint16_t ui16_battery_voltage_x1000;
  uint8_t ui8_battery_current_x10;
  uint8_t ui8_adc_throttle;
  uint8_t ui8_throttle;
  uint16_t ui16_adc_pedal_torque_sensor;
  uint8_t ui8_duty_cycle;
  uint8_t ui8_controller_system_state;
  uint16_t ui16_wheel_speed_x10;
  uint8_t ui8_braking;
  uint8_t ui8_pedal_cadence_RPM;
  uint16_t ui16_motor_speed_erps;
  uint8_t ui8_foc_angle;
  uint8_t ui8_temperature_current_limiting_value;
  uint8_t ui8_motor_temperature;
  uint32_t ui32_wheel_speed_sensor_tick_counter;
  uint32_t ui32_wheel_speed_sensor_tick_counter_offset;
  uint16_t ui16_pedal_torque_x100;
  uint16_t ui16_pedal_power_x10;
} struct_motor_controller_data;

typedef struct _configuration_variables
{
  uint16_t ui16_cadence_sensor_pulse_high_percentage_x10;
  uint8_t ui8_assist_without_pedal_rotation_threshold;
  uint8_t ui8_light_mode;
  uint8_t ui8_lights_state;
  uint8_t ui8_lights_configuration;
  uint8_t ui8_assist_level;
  uint8_t ui8_number_of_assist_levels;
  uint8_t ui8_power_assist_function_enabled;
  uint8_t ui8_power_assist_level[9];
  uint8_t ui8_torque_assist_function_enabled;
  uint8_t ui8_torque_assist_level[9];
  uint8_t ui8_cadence_assist_function_enabled;
  uint8_t ui8_cadence_assist_level[9];
  uint8_t ui8_eMTB_assist_function_enabled;
  uint8_t ui8_eMTB_assist_sensitivity;
  uint8_t ui8_walk_assist_function_enabled;
  uint8_t ui8_walk_assist_button_bounce_time;
  uint8_t ui8_walk_assist_level[9];
  uint8_t ui8_cruise_function_enabled;
  uint8_t ui8_cruise_function_set_target_speed_enabled;
  uint8_t ui8_cruise_function_target_speed_kph;
  uint8_t ui8_cruise_function_target_speed_mph;
  uint16_t ui16_wheel_perimeter;
  uint8_t ui8_wheel_max_speed;
  uint8_t ui8_wheel_max_speed_imperial;
  uint8_t ui8_units_type;
  uint32_t ui32_wh_x10_offset;
  uint32_t ui32_wh_x10_100_percent;
  uint8_t ui8_battery_SOC_function_enabled;
  uint8_t ui8_odometer_field_state;
  uint8_t ui8_time_measurement_field_state;
  uint8_t ui8_total_second_TTM;
  uint16_t ui8_total_minute_TTM;
  uint16_t ui16_total_hour_TTM;
  uint8_t ui8_odometer_sub_field_state_0;
  uint8_t ui8_odometer_sub_field_state_1;
  uint8_t ui8_odometer_sub_field_state_2;
  uint8_t ui8_odometer_sub_field_state_3;
  uint8_t ui8_odometer_sub_field_state_4;
  uint8_t ui8_odometer_sub_field_state_5;
  uint8_t ui8_odometer_sub_field_state_6;
  uint8_t ui8_odometer_show_field_number;
  uint8_t ui8_target_max_battery_power_div25;
  uint8_t ui8_battery_cells_number;
  uint8_t ui8_battery_max_current;
  uint16_t ui16_battery_low_voltage_cut_off_x10;
  uint16_t ui16_battery_voltage_reset_wh_counter_x10;
  uint16_t ui16_battery_pack_resistance_x1000;
  uint8_t ui8_motor_type;
  uint8_t ui8_pedal_torque_per_10_bit_ADC_step_x100;
  uint16_t ui16_adc_motor_temperature_10b;
  uint8_t ui8_optional_ADC_function;
  uint8_t ui8_motor_temperature_min_value_to_limit;
  uint8_t ui8_motor_temperature_max_value_to_limit;
  uint8_t ui8_temperature_field_state;
  uint8_t ui8_lcd_power_off_time_minutes;
  uint8_t ui8_lcd_backlight_on_brightness;
  uint8_t ui8_lcd_backlight_off_brightness;
  uint8_t ui8_street_mode_function_enabled;
  uint8_t ui8_street_mode_enabled;
  uint8_t ui8_street_mode_speed_limit;
  uint8_t ui8_street_mode_power_limit_enabled;
  uint8_t ui8_street_mode_power_limit_div25;
  uint8_t ui8_street_mode_throttle_enabled;
  uint8_t ui8_street_mode_cruise_enabled;
  uint16_t ui16_distance_since_power_on_x10;
  uint32_t ui32_odometer_x10;
  uint32_t ui32_trip_x10;
  uint8_t ui8_motor_acceleration;
  uint8_t ui8_cadence_sensor_mode;
  uint8_t ui8_show_cruise_function_set_target_speed;
  uint8_t ui8_wheel_speed_field_state;
  uint8_t ui8_show_distance_data_odometer_field;
  uint8_t ui8_show_battery_state_odometer_field;
  uint8_t ui8_show_pedal_data_odometer_field;
  uint8_t ui8_show_time_measurement_odometer_field;
  uint8_t ui8_show_wheel_speed_odometer_field;
  uint8_t ui8_show_energy_data_odometer_field;
  uint8_t ui8_show_motor_temperature_odometer_field;
  uint8_t ui8_show_battery_SOC_odometer_field;
  uint8_t ui8_main_screen_power_menu_enabled;
} struct_configuration_variables;



// menu definitions
#define MAIN_MENU               0
#define POWER_MENU              1
#define CONFIGURATION_MENU      2

#define LCD_FRAME_BUFFER_SIZE   32 // LCD RAM has 32*8 bits

extern uint8_t ui8_lcd_frame_buffer[LCD_FRAME_BUFFER_SIZE];

#define ASSIST_LEVEL_FIELD      0
#define ODOMETER_FIELD          1
#define TEMPERATURE_FIELD       2
#define WHEEL_SPEED_FIELD       3
#define BATTERY_POWER_FIELD     4
#define TIME_SECOND_FIELD       5
#define TIME_MINUTE_FIELD       6


// each digit needs 7 bits to be defined + 1 digit that can be another symbol like a "point"
#define ASSIST_LEVEL_DIGIT_OFFSET     1 // 8
#define ODOMETER_DIGIT_OFFSET         6
#define TEMPERATURE_DIGIT_OFFSET      8
#define WHEEL_SPEED_OFFSET            14
#define BATTERY_POWER_DIGIT_OFFSET    10
#define SECOND_DIGIT_OFFSET           18
#define MINUTE_DIGIT_OFFSET           20


#define NUMBERS_MASK                  8
#define NUMBER_0_MASK                 119
#define NUMBER_1_MASK                 66  // 2; 7
#define NUMBER_2_MASK                 182 // 3; 2; 8; 6; 5
#define NUMBER_3_MASK                 214
#define NUMBER_4_MASK                 195
#define NUMBER_5_MASK                 213
#define NUMBER_6_MASK                 245
#define NUMBER_7_MASK                 70
#define NUMBER_8_MASK                 247
#define NUMBER_9_MASK                 215
#define NUMBER_0_MASK_INVERTED        119
#define NUMBER_1_MASK_INVERTED        33  // 2; 7
#define NUMBER_2_MASK_INVERTED        182 // 3; 2; 8; 6; 5
#define NUMBER_3_MASK_INVERTED        181
#define NUMBER_4_MASK_INVERTED        225
#define NUMBER_5_MASK_INVERTED        213
#define NUMBER_6_MASK_INVERTED        215
#define NUMBER_7_MASK_INVERTED        49
#define NUMBER_8_MASK_INVERTED        247
#define NUMBER_9_MASK_INVERTED        245


// : from timer label ui8_lcd_frame_buffer[23] |= 8


void lcd_init (void);
void lcd_clock (void);
struct_configuration_variables* get_configuration_variables (void);
struct_motor_controller_data* lcd_get_motor_controller_data (void);
uint16_t get_timer3_counter(void);

#endif /* _LCD_H_ */
