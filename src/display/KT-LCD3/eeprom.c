/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "eeprom.h"

#include <stdint.h>

#include "stm8s.h"
#include "stm8s_flash.h"
#include "main.h"
#include "lcd.h"

static uint8_t array_default_values[EEPROM_BYTES_STORED] = {
  KEY,
  DEFAULT_VALUE_ASSIST_LEVEL,
  DEFAULT_VALUE_WHEEL_PERIMETER_0,
  DEFAULT_VALUE_WHEEL_PERIMETER_1,
  DEFAULT_VALUE_WHEEL_MAX_SPEED,
  DEFAULT_VALUE_UNITS_TYPE,
  DEFAULT_VALUE_WH_OFFSET,
  DEFAULT_VALUE_WH_OFFSET,
  DEFAULT_VALUE_WH_OFFSET,
  DEFAULT_VALUE_WH_OFFSET,
  DEFAULT_VALUE_HW_X10_100_PERCENT,
  DEFAULT_VALUE_HW_X10_100_PERCENT,
  DEFAULT_VALUE_HW_X10_100_PERCENT,
  DEFAULT_VALUE_HW_X10_100_PERCENT,
  DEFAULT_VALUE_BATTERY_SOC_FUNCTION_ENABLED,
  DEFAULT_VALUE_ODOMETER_FIELD_STATE,
  DEFAULT_VALUE_BATTERY_MAX_CURRENT,
  DEFAULT_VALUE_TARGET_MAX_BATTERY_POWER,
  DEFAULT_VALUE_BATTERY_CELLS_NUMBER,
  DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0,
  DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1,
  DEFAULT_VALUE_CONFIG_0,                                             // 21 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_FUNCTION_ENABLED,                        // 22 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_1,                                 // 23 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_2,                                 // 24 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_3,                                 // 25 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_4,                                 // 26 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_5,                                 // 27 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_6,                                 // 28 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_7,                                 // 29 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_8,                                 // 30 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_9,                                 // 31 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_NUMBER_OF_ASSIST_LEVELS,                              // 32 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_FEATURE_ENABLED,            // 33 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_STATE,                      // 34 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1,             // 35 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_2,             // 36 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_3,             // 37 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_4,             // 38 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_5,             // 39 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_6,             // 40 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_7,             // 41 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_8,             // 42 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_9,             // 43 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_TIME,                       // 44 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_FADE_TIME,                  // 45 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT,                    // 46 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT,                    // 47 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0,               // 48 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1,               // 49 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_LCD_POWER_OFF_TIME,                                   // 50 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS,                          // 51 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS,                         // 52 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_0,                            // 53 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_1,                            // 54 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_FUNCTION_ENABLED,                         // 55 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_ENABLED_ON_STARTUP,                       // 56 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_SPEED_LIMIT,                              // 57 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_POWER_LIMIT_ENABLED,                      // 58 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_POWER_LIMIT_DIV25,                        // 59 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_X10,                                         // 60 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_X10,                                         // 61 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_X10,                                         // 62 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TRIP_X10,                                             // 63 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TRIP_X10,                                             // 64 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TRIP_X10,                                             // 65 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_0,                           // 66 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_1,                           // 67 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_2,                           // 68 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_3,                           // 69 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_4,                           // 70 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_5,                           // 71 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_6,                           // 72 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_MAX_WHEEL_SPEED_IMPERIAL,                             // 73 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TIME_MEASUREMENT_FIELD_STATE,                         // 74 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TOTAL_SECOND_TTM,                                     // 75 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TOTAL_MINUTE_TTM,                                     // 76 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TOTAL_HOUR_TTM_0,                                     // 77 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TOTAL_HOUR_TTM_1,                                     // 78 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_RAMP_UP_AMPS_PER_SECOND_X10,                          // 79 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_FUNCTION_ENABLED,                         // 80 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_1,                                  // 81 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_2,                                  // 82 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_3,                                  // 83 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_4,                                  // 84 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_5,                                  // 85 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_6,                                  // 86 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_7,                                  // 87 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_8,                                  // 88 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_9,                                  // 89 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CRUISE_FUNCTION_ENABLED,                              // 90 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED,             // 91 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CRUISE_FUNCTION_TARGET_SPEED_KPH,                     // 92 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CRUISE_FUNCTION_TARGET_SPEED_MPH,                     // 93 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED,                // 94 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WHEEL_SPEED_FIELD_STATE,                              // 95 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_DISTANCE_DATA_ODOMETER_FIELD,                    // 96 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_BATTERY_STATE_ODOMETER_FIELD,                    // 97 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_PEDAL_DATA_ODOMETER_FIELD,                       // 98 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD,                 // 99 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_WHEEL_SPEED_ODOMETER_FIELD,                      // 100 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_ENERGY_DATA_ODOMETER_FIELD,                      // 101 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD,                // 102 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_BATTERY_SOC_ODOMETER_FIELD,                      // 103 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_MAIN_SCREEN_POWER_MENU_ENABLED,                       // 104 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_THROTTLE_ENABLED,                         // 105 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_RPM_MIN                                       // 106 + EEPROM_BASE_ADDRESS (Array index)
};


static void eeprom_write_array (uint8_t *p_array_data, uint8_t ui8_len);
static void eeprom_read_values_to_variables (void);
static void variables_to_array (uint8_t *ui8_array);


void eeprom_init (void)
{
  uint8_t ui8_data;

  // start by reading address 0 and see if value is different from our key,
  // if it is different it means that EEPROM memory is "empty" and we need to populate.
  // This should always happen after erasing the microcontroller.
  
  ui8_data = FLASH_ReadByte (ADDRESS_KEY);
  if (ui8_data != KEY) // verify if our key exists
  {
    // write default values
    eeprom_write_array(array_default_values, ((uint8_t) EEPROM_BYTES_STORED));
  }
}

void eeprom_init_variables (void)
{
  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables ();

  eeprom_read_values_to_variables ();

//  // now verify if any EEPROM saved value is out of valid range and if so, write correct ones and read again
//  if ((p_configuration_variables->ui8_number_of_assist_levels < 1) ||
//      (p_configuration_variables->ui8_number_of_assist_levels > 9) ||
//      (p_configuration_variables->ui16_wheel_perimeter > 3000) ||
//      (p_configuration_variables->ui16_wheel_perimeter < 750) ||
//      (p_configuration_variables->ui8_wheel_max_speed > 99) ||
//      (p_configuration_variables->ui8_units_type > 1) ||
//      (p_configuration_variables->ui32_wh_x10_offset > 99900) ||
//      (p_configuration_variables->ui32_wh_x10_100_percent > 99900) ||
//      (p_configuration_variables->ui8_battery_SOC_function_enabled > 2) ||
//      (p_configuration_variables->ui8_odometer_field_state > 4) ||
//      (p_configuration_variables->ui8_battery_max_current > 100) ||
//      (p_configuration_variables->ui8_target_max_battery_power_div10 > 195) ||
//      (p_configuration_variables->ui8_startup_motor_power_boost_state > 3) ||
//      (p_configuration_variables->ui8_battery_cells_number > 15) ||
//      (p_configuration_variables->ui8_battery_cells_number < 6) ||
//      (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 > 630) ||
//      (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 < 160) ||
//      (p_configuration_variables->ui8_motor_type > 2) ||
//      (p_configuration_variables->ui8_motor_temperature_min_value_to_limit < 124) ||
//      (p_configuration_variables->ui8_motor_temperature_max_value_to_limit < 124))
//  {
//    eeprom_write_array (array_default_values);
//    eeprom_read_values_to_variables ();
//  }
}

static void eeprom_read_values_to_variables (void)
{
  uint8_t ui8_temp;
  uint16_t ui16_temp;
  uint32_t ui32_temp;
  uint8_t ui8_index;

  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables ();

  // assist level
  p_configuration_variables->ui8_assist_level = FLASH_ReadByte(ADDRESS_ASSIST_LEVEL);
  
  
  // number of assist levels
  p_configuration_variables->ui8_number_of_assist_levels = FLASH_ReadByte(ADDRESS_NUMBER_OF_ASSIST_LEVELS);
  
  
  // power assist
  p_configuration_variables->ui8_power_assist_function_enabled = FLASH_ReadByte(ADDRESS_POWER_ASSIST_FUNCTION_ENABLED);
  for (ui8_index = 0; ui8_index < 9; ui8_index++)
  {
    p_configuration_variables->ui8_power_assist_level[ui8_index] = FLASH_ReadByte(ADDRESS_POWER_ASSIST_LEVEL_1 + ui8_index);
  }
  
  
  // walk assist
  p_configuration_variables->ui8_walk_assist_function_enabled = FLASH_ReadByte(ADDRESS_WALK_ASSIST_FUNCTION_ENABLED);
  for (ui8_index = 0; ui8_index < 10; ui8_index++)
  {
    p_configuration_variables->ui8_walk_assist_level[ui8_index] = FLASH_ReadByte(ADDRESS_WALK_ASSIST_LEVEL_1 + ui8_index);
  }
  
  
  // cruise function
  p_configuration_variables->ui8_cruise_function_enabled = FLASH_ReadByte (ADDRESS_CRUISE_FUNCTION_ENABLED);
  p_configuration_variables->ui8_cruise_function_set_target_speed_enabled = FLASH_ReadByte (ADDRESS_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED);
  p_configuration_variables->ui8_cruise_function_target_speed_kph = FLASH_ReadByte (ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_KPH);
  p_configuration_variables->ui8_cruise_function_target_speed_mph = FLASH_ReadByte (ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_MPH);
  p_configuration_variables->ui8_show_cruise_function_set_target_speed = FLASH_ReadByte (ADDRESS_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED);    


  // wheel perimeter
  ui16_temp = FLASH_ReadByte (ADDRESS_WHEEL_PERIMETER_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_WHEEL_PERIMETER_1);
  ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
  p_configuration_variables->ui16_wheel_perimeter = ui16_temp;


  // max wheel speed
  p_configuration_variables->ui8_wheel_max_speed = FLASH_ReadByte (ADDRESS_MAX_WHEEL_SPEED);
  p_configuration_variables->ui8_wheel_max_speed_imperial = FLASH_ReadByte (ADDRESS_MAX_WHEEL_SPEED_IMPERIAL);
  
  
  // units
  p_configuration_variables->ui8_units_type = FLASH_ReadByte (ADDRESS_UNITS_TYPE);

  ui32_temp = FLASH_ReadByte (ADDRESS_HW_X10_OFFSET_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_OFFSET_1);
  ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_OFFSET_2);
  ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_OFFSET_3);
  ui32_temp += (((uint32_t) ui8_temp << 24) & 0xff000000);
  p_configuration_variables->ui32_wh_x10_offset = ui32_temp;

  ui32_temp = FLASH_ReadByte (ADDRESS_HW_X10_100_PERCENT_OFFSET_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_100_PERCENT_OFFSET_1);
  ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_100_PERCENT_OFFSET_2);
  ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
  ui8_temp = FLASH_ReadByte (ADDRESS_HW_X10_100_PERCENT_OFFSET_3);
  ui32_temp += (((uint32_t) ui8_temp << 24) & 0xff000000);
  p_configuration_variables->ui32_wh_x10_100_percent = ui32_temp;

  // battery SOC function
  p_configuration_variables->ui8_battery_SOC_function_enabled = FLASH_ReadByte (ADDRESS_BATTERY_SOC_FUNCTION_ENABLED);
  
  p_configuration_variables->ui8_odometer_field_state = FLASH_ReadByte (ADDRESS_ODOMETER_FIELD_STATE);
  p_configuration_variables->ui8_odometer_sub_field_state_0 = FLASH_ReadByte (ADDRESS_ODOMETER_SUB_FIELD_STATE_0);
  p_configuration_variables->ui8_odometer_sub_field_state_1 = FLASH_ReadByte (ADDRESS_ODOMETER_SUB_FIELD_STATE_1);
  p_configuration_variables->ui8_odometer_sub_field_state_2 = FLASH_ReadByte (ADDRESS_ODOMETER_SUB_FIELD_STATE_2);
  p_configuration_variables->ui8_odometer_sub_field_state_3 = FLASH_ReadByte (ADDRESS_ODOMETER_SUB_FIELD_STATE_3);
  p_configuration_variables->ui8_odometer_sub_field_state_4 = FLASH_ReadByte (ADDRESS_ODOMETER_SUB_FIELD_STATE_4); 
  p_configuration_variables->ui8_odometer_sub_field_state_5 = FLASH_ReadByte (ADDRESS_ODOMETER_SUB_FIELD_STATE_5); 
  p_configuration_variables->ui8_odometer_sub_field_state_6 = FLASH_ReadByte (ADDRESS_ODOMETER_SUB_FIELD_STATE_6);
  
  
  // time measurement variables
  p_configuration_variables->ui8_time_measurement_field_state = FLASH_ReadByte (ADDRESS_TIME_MEASUREMENT_FIELD_STATE);
  p_configuration_variables->ui8_total_second_TTM = FLASH_ReadByte (ADDRESS_TOTAL_SECOND_TTM);
  p_configuration_variables->ui8_total_minute_TTM = FLASH_ReadByte (ADDRESS_TOTAL_MINUTE_TTM);
  ui16_temp = FLASH_ReadByte (ADDRESS_TOTAL_HOUR_TTM_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_TOTAL_HOUR_TTM_1);
  ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
  p_configuration_variables->ui16_total_hour_TTM = ui16_temp;
  
  
  // battery
  p_configuration_variables->ui8_battery_max_current = FLASH_ReadByte (ADDRESS_BATTERY_MAX_CURRENT);
  p_configuration_variables->ui8_target_max_battery_power_div25 = FLASH_ReadByte (ADDRESS_TARGET_MAX_BATTERY_POWER);
  p_configuration_variables->ui8_battery_cells_number = FLASH_ReadByte (ADDRESS_BATTERY_CELLS_NUMBER);

  ui16_temp = FLASH_ReadByte (ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1);
  ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
  p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 = ui16_temp;


  // motor type, temperature limit enabled, tempereture field state
  ui8_temp = FLASH_ReadByte (ADDRESS_CONFIG_0);
  p_configuration_variables->ui8_motor_type = ui8_temp & 3;
  p_configuration_variables->ui8_temperature_limit_feature_enabled = (ui8_temp & 24) >> 3;
  p_configuration_variables->ui8_temperature_field_state = (ui8_temp & 224) >> 5;


  p_configuration_variables->ui8_startup_motor_power_boost_feature_enabled = FLASH_ReadByte (ADDRESS_STARTUP_MOTOR_POWER_BOOST_FEATURE_ENABLED);
  p_configuration_variables->ui8_startup_motor_power_boost_state = FLASH_ReadByte (ADDRESS_STARTUP_MOTOR_POWER_BOOST_STATE);
  p_configuration_variables->ui8_startup_motor_power_boost_time = FLASH_ReadByte (ADDRESS_STARTUP_MOTOR_POWER_BOOST_TIME);
  for (ui8_index = 0; ui8_index < 9; ui8_index++)
  {
    p_configuration_variables->ui8_startup_motor_power_boost_factor [ui8_index] = FLASH_ReadByte (ADDRESS_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1 + ui8_index);
  }

  p_configuration_variables->ui8_startup_motor_power_boost_fade_time = FLASH_ReadByte (ADDRESS_STARTUP_MOTOR_POWER_BOOST_FADE_TIME);

  p_configuration_variables->ui8_motor_temperature_min_value_to_limit = FLASH_ReadByte (ADDRESS_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT);
  p_configuration_variables->ui8_motor_temperature_max_value_to_limit = FLASH_ReadByte (ADDRESS_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT);

  ui16_temp = FLASH_ReadByte (ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1);
  ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
  p_configuration_variables->ui16_battery_voltage_reset_wh_counter_x10 = ui16_temp;

  p_configuration_variables->ui8_lcd_power_off_time_minutes = FLASH_ReadByte (ADDRESS_LCD_POWER_OFF_TIME);
  p_configuration_variables->ui8_lcd_backlight_on_brightness = FLASH_ReadByte (ADDRESS_LCD_BACKLIGHT_ON_BRIGHTNESS);
  p_configuration_variables->ui8_lcd_backlight_off_brightness = FLASH_ReadByte (ADDRESS_LCD_BACKLIGHT_OFF_BRIGHTNESS);

  ui16_temp = FLASH_ReadByte (ADDRESS_BATTERY_PACK_RESISTANCE_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_BATTERY_PACK_RESISTANCE_1);
  ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
  p_configuration_variables->ui16_battery_pack_resistance_x1000 = ui16_temp;

  // street mode variables
  p_configuration_variables->ui8_street_mode_function_enabled = FLASH_ReadByte (ADDRESS_STREET_MODE_FUNCTION_ENABLED);
  p_configuration_variables->ui8_street_mode_enabled_on_startup = FLASH_ReadByte (ADDRESS_STREET_MODE_ENABLED_ON_STARTUP);
  p_configuration_variables->ui8_street_mode_speed_limit = FLASH_ReadByte (ADDRESS_STREET_MODE_SPEED_LIMIT);
  p_configuration_variables->ui8_street_mode_power_limit_enabled = FLASH_ReadByte (ADDRESS_STREET_MODE_POWER_LIMIT_ENABLED);
  p_configuration_variables->ui8_street_mode_power_limit_div25 = FLASH_ReadByte (ADDRESS_STREET_MODE_POWER_LIMIT_DIV25);
  p_configuration_variables->ui8_street_mode_throttle_enabled = FLASH_ReadByte (ADDRESS_STREET_MODE_THROTTLE_ENABLED);
  
  
  // odometer variable
  ui32_temp = FLASH_ReadByte (ADDRESS_ODOMETER_X10_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_ODOMETER_X10_1);
  ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
  ui8_temp = FLASH_ReadByte (ADDRESS_ODOMETER_X10_2);
  ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
  p_configuration_variables->ui32_odometer_x10 = ui32_temp;
  
  
  // trip distance variable
  ui32_temp = FLASH_ReadByte (ADDRESS_TRIP_X10_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_TRIP_X10_1);
  ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
  ui8_temp = FLASH_ReadByte (ADDRESS_TRIP_X10_2);
  ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
  p_configuration_variables->ui32_trip_x10 = ui32_temp;
  
  
  // ramp up amps per second
  p_configuration_variables->ui8_ramp_up_amps_per_second_x10 = FLASH_ReadByte (ADDRESS_RAMP_UP_AMPS_PER_SECOND_X10);
  
  
  // wheel speed measurement
  p_configuration_variables->ui8_wheel_speed_field_state = FLASH_ReadByte (ADDRESS_WHEEL_SPEED_FIELD_STATE);
  
  
  // show variables in odometer field
  p_configuration_variables->ui8_show_distance_data_odometer_field = FLASH_ReadByte (ADDRESS_SHOW_DISTANCE_DATA_ODOMETER_FIELD);
  p_configuration_variables->ui8_show_battery_state_odometer_field = FLASH_ReadByte (ADDRESS_SHOW_BATTERY_STATE_ODOMETER_FIELD);
  p_configuration_variables->ui8_show_pedal_data_odometer_field = FLASH_ReadByte (ADDRESS_SHOW_PEDAL_DATA_ODOMETER_FIELD);
  p_configuration_variables->ui8_show_time_measurement_odometer_field = FLASH_ReadByte (ADDRESS_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD);
  p_configuration_variables->ui8_show_wheel_speed_odometer_field = FLASH_ReadByte (ADDRESS_SHOW_WHEEL_SPEED_ODOMETER_FIELD);
  p_configuration_variables->ui8_show_energy_data_odometer_field = FLASH_ReadByte (ADDRESS_SHOW_ENERGY_DATA_ODOMETER_FIELD);
  p_configuration_variables->ui8_show_motor_temperature_odometer_field = FLASH_ReadByte (ADDRESS_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD);
  p_configuration_variables->ui8_show_battery_SOC_odometer_field = FLASH_ReadByte (ADDRESS_SHOW_BATTERY_SOC_ODOMETER_FIELD);
  
  
  // main screen power menu enable 
  p_configuration_variables->ui8_main_screen_power_menu_enabled = FLASH_ReadByte(ADDRESS_MAIN_SCREEN_POWER_MENU_ENABLED);
  
  
  // cadence rpm min
  p_configuration_variables->ui8_cadence_rpm_min = FLASH_ReadByte(ADDRESS_CADENCE_RPM_MIN);
}


void eeprom_write_variables (void)
{
  uint8_t array_variables[EEPROM_BYTES_STORED];
  variables_to_array(array_variables);
  eeprom_write_array(array_variables, ((uint8_t) EEPROM_BYTES_STORED));
}


static void variables_to_array (uint8_t *ui8_array)
{
  uint8_t ui8_index;

  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables ();
  
  // write various parameters
  ui8_array [0] = KEY;
  ui8_array[ADDRESS_ASSIST_LEVEL - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_assist_level;
  ui8_array [2] = p_configuration_variables->ui16_wheel_perimeter & 255;
  ui8_array [3] = (p_configuration_variables->ui16_wheel_perimeter >> 8) & 255;
  ui8_array [4] = p_configuration_variables->ui8_wheel_max_speed;
  ui8_array [5] = p_configuration_variables->ui8_units_type;
  ui8_array [6] = p_configuration_variables->ui32_wh_x10_offset & 255;
  ui8_array [7] = (p_configuration_variables->ui32_wh_x10_offset >> 8) & 255;
  ui8_array [8] = (p_configuration_variables->ui32_wh_x10_offset >> 16) & 255;
  ui8_array [9] = (p_configuration_variables->ui32_wh_x10_offset >> 24) & 255;
  ui8_array [10] = p_configuration_variables->ui32_wh_x10_100_percent & 255;
  ui8_array [11] = (p_configuration_variables->ui32_wh_x10_100_percent >> 8) & 255;
  ui8_array [12] = (p_configuration_variables->ui32_wh_x10_100_percent >> 16) & 255;
  ui8_array [13] = (p_configuration_variables->ui32_wh_x10_100_percent >> 24) & 255;
  ui8_array [14] = p_configuration_variables->ui8_battery_SOC_function_enabled;
  ui8_array [15] = p_configuration_variables->ui8_odometer_field_state;
  ui8_array [16] = p_configuration_variables->ui8_battery_max_current;
  ui8_array [17] = p_configuration_variables->ui8_target_max_battery_power_div25;
  ui8_array [18] = p_configuration_variables->ui8_battery_cells_number;
  ui8_array [19] = p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 & 255;
  ui8_array [20] = (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 >> 8) & 255;
  
  
  // write motor type, temperature limit enabled, tempereture field state
  ui8_array [21] = (p_configuration_variables->ui8_motor_type & 3) |
                  ((p_configuration_variables->ui8_temperature_limit_feature_enabled & 3) << 3) |
                  ((p_configuration_variables->ui8_temperature_field_state & 7) << 5);
                  
                  
  // power assist
  ui8_array[22] = p_configuration_variables->ui8_power_assist_function_enabled;
  for (ui8_index = 0; ui8_index < 9; ui8_index++)
  {
    ui8_array[23 + ui8_index] = p_configuration_variables->ui8_power_assist_level[ui8_index];
  }
  
  
  // number of assist levels
  ui8_array[ADDRESS_NUMBER_OF_ASSIST_LEVELS - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_number_of_assist_levels;
  
  
  // write motor parameters
  ui8_array [33] = p_configuration_variables->ui8_startup_motor_power_boost_feature_enabled;
  ui8_array [34] = p_configuration_variables->ui8_startup_motor_power_boost_state;
  for (ui8_index = 0; ui8_index < 9; ui8_index++)
  {
    ui8_array [35 + ui8_index] = p_configuration_variables->ui8_startup_motor_power_boost_factor [ui8_index];
  }
  ui8_array [44] = p_configuration_variables->ui8_startup_motor_power_boost_time;
  ui8_array [45] = p_configuration_variables->ui8_startup_motor_power_boost_fade_time;
  ui8_array [46] = p_configuration_variables->ui8_motor_temperature_min_value_to_limit;
  ui8_array [47] = p_configuration_variables->ui8_motor_temperature_max_value_to_limit;
  
  
  // write battery parameters
  ui8_array [48] = p_configuration_variables->ui16_battery_voltage_reset_wh_counter_x10 & 255;
  ui8_array [49] = (p_configuration_variables->ui16_battery_voltage_reset_wh_counter_x10 >> 8) & 255;
  
  
  // write display parameters
  ui8_array [50] = p_configuration_variables->ui8_lcd_power_off_time_minutes;
  ui8_array [51] = p_configuration_variables->ui8_lcd_backlight_on_brightness;
  ui8_array [52] = p_configuration_variables->ui8_lcd_backlight_off_brightness;


  // write battery parameters
  ui8_array [53] = p_configuration_variables->ui16_battery_pack_resistance_x1000 & 255;
  ui8_array [54] = (p_configuration_variables->ui16_battery_pack_resistance_x1000 >> 8) & 255;


  // write street mode parameters
  ui8_array [55] = p_configuration_variables->ui8_street_mode_function_enabled;
  ui8_array [56] = p_configuration_variables->ui8_street_mode_enabled_on_startup;
  ui8_array [57] = p_configuration_variables->ui8_street_mode_speed_limit;
  ui8_array [58] = p_configuration_variables->ui8_street_mode_power_limit_enabled;
  ui8_array [59] = p_configuration_variables->ui8_street_mode_power_limit_div25;
  
  
  // write odometer variable
  ui8_array [60] = p_configuration_variables->ui32_odometer_x10 & 255;
  ui8_array [61] = (p_configuration_variables->ui32_odometer_x10 >> 8) & 255;
  ui8_array [62] = (p_configuration_variables->ui32_odometer_x10 >> 16) & 255;
  
  
  // write trip distance variable
  ui8_array [63] = p_configuration_variables->ui32_trip_x10 & 255;
  ui8_array [64] = (p_configuration_variables->ui32_trip_x10 >> 8) & 255;
  ui8_array [65] = (p_configuration_variables->ui32_trip_x10 >> 16) & 255;


  // write sub menu states so user can resume since last power on
  ui8_array [66] = p_configuration_variables->ui8_odometer_sub_field_state_0;
  ui8_array [67] = p_configuration_variables->ui8_odometer_sub_field_state_1;
  ui8_array [68] = p_configuration_variables->ui8_odometer_sub_field_state_2;
  ui8_array [69] = p_configuration_variables->ui8_odometer_sub_field_state_3;
  ui8_array [70] = p_configuration_variables->ui8_odometer_sub_field_state_4;
  ui8_array [71] = p_configuration_variables->ui8_odometer_sub_field_state_5;
  ui8_array [72] = p_configuration_variables->ui8_odometer_sub_field_state_6;
  
  
  // write max wheel speed in imperial units
  ui8_array [73] = p_configuration_variables->ui8_wheel_max_speed_imperial;
  
  
  // write time measurement field state
  ui8_array [74] = p_configuration_variables->ui8_time_measurement_field_state;


  // write time measurement values
  ui8_array [75] = p_configuration_variables->ui8_total_second_TTM;
  ui8_array [76] = p_configuration_variables->ui8_total_minute_TTM;
  ui8_array [77] = p_configuration_variables->ui16_total_hour_TTM & 255;
  ui8_array [78] = (p_configuration_variables->ui16_total_hour_TTM >> 8) & 255;
  
  
  // write ramp up amps per second
  ui8_array [79] = p_configuration_variables->ui8_ramp_up_amps_per_second_x10;
  
  
  // write walk assist function variables
  ui8_array[80] = p_configuration_variables->ui8_walk_assist_function_enabled;
  for (ui8_index = 0; ui8_index < 10; ui8_index++)
  {
    ui8_array[81 + ui8_index] = p_configuration_variables->ui8_walk_assist_level[ui8_index];
  }
  
  
  // write cruise function variables
  ui8_array [90] = p_configuration_variables->ui8_cruise_function_enabled;
  ui8_array [91] = p_configuration_variables->ui8_cruise_function_set_target_speed_enabled;
  ui8_array [92] = p_configuration_variables->ui8_cruise_function_target_speed_kph;
  ui8_array [93] = p_configuration_variables->ui8_cruise_function_target_speed_mph;
  ui8_array [94] = p_configuration_variables->ui8_show_cruise_function_set_target_speed;  
  
  
  // write wheel speed field state
  ui8_array [95] = p_configuration_variables->ui8_wheel_speed_field_state;
  
  
  // write show odometer variables
  ui8_array [96] = p_configuration_variables->ui8_show_distance_data_odometer_field;
  ui8_array [97] = p_configuration_variables->ui8_show_battery_state_odometer_field;
  ui8_array [98] = p_configuration_variables->ui8_show_pedal_data_odometer_field;
  ui8_array [99] = p_configuration_variables->ui8_show_time_measurement_odometer_field;
  ui8_array [100] = p_configuration_variables->ui8_show_wheel_speed_odometer_field;
  ui8_array [101] = p_configuration_variables->ui8_show_energy_data_odometer_field;
  ui8_array [102] = p_configuration_variables->ui8_show_motor_temperature_odometer_field;
  ui8_array [103] = p_configuration_variables->ui8_show_battery_SOC_odometer_field;
  
  // write main screen power menu enable variable
  ui8_array [104] = p_configuration_variables->ui8_main_screen_power_menu_enabled;
  
  // write street mode parameters
  ui8_array [105] = p_configuration_variables->ui8_street_mode_throttle_enabled;

  // write cadence min
  ui8_array [106] = p_configuration_variables->ui8_cadence_rpm_min;
}


static void eeprom_write_array (uint8_t *p_array, uint8_t ui8_len)
{
  uint8_t ui8_i;
  uint8_t ui8_data;
  uint8_t *p_array_data;
  uint8_t array_data_read_back [EEPROM_BYTES_STORED];
  uint8_t ui8_data_written_correctly = 0;

  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_TPROG);
  
  // unlock memory
  FLASH_Unlock (FLASH_MEMTYPE_DATA);
  
  while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET) { } // Wait until Data EEPROM area unlocked flag is set

  // on next loop, we write the array to EEPROM and then read again and compare the values,
  // if they are different, we keep writing until they are equal
  do
  {
    // write the full array
    p_array_data = p_array;
    for (ui8_i = 0; ui8_i < ui8_len; ui8_i++)
    {
      FLASH_ProgramByte (((uint32_t) EEPROM_BASE_ADDRESS) + ((uint32_t) ui8_i), *p_array_data++);
    }

    // read back the full array
    variables_to_array (array_data_read_back);

    // compare each byte of read array to see if the values were correctly written
    ui8_data_written_correctly = 1;
    p_array_data = p_array;
    for (ui8_i = 0; ui8_i < ui8_len; ui8_i++)
    {
      ui8_data = *p_array_data++;
      if (ui8_data != array_data_read_back[ui8_i])
      {
        ui8_data_written_correctly = 0;
        break;
      }
    }
  }
  while (ui8_data_written_correctly == 0); // loop until data is written correctly

  // lock memory
  FLASH_Lock (FLASH_MEMTYPE_DATA);
}

void eeprom_erase_key_value (void)
{
  uint8_t ui8_data;
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_TPROG);

  // unlock memory
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  
  while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET) { } // Wait until Data EEPROM area unlocked flag is set

  do
  {
    // set the key to 0
    FLASH_ProgramByte(ADDRESS_KEY, 0);

    // read back the KEY value
    ui8_data = FLASH_ReadByte(ADDRESS_KEY);
  }
  while (ui8_data != 0); // loop until key value is stored as 0
  
  // lock memory
  FLASH_Lock (FLASH_MEMTYPE_DATA);
}
