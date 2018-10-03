/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_flash.h"
#include "eeprom.h"
#include "main.h"
#include "lcd.h"

static uint8_t array_default_values [EEPROM_BYTES_STORED] = {
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
    DEAFULT_VALUE_SHOW_NUMERIC_BATTERY_SOC,
    DEFAULT_VALUE_ODOMETER_FIELD_STATE,
    DEFAULT_VALUE_BATTERY_MAX_CURRENT,
    DEFAULT_VALUE_TARGET_MAX_BATTERY_POWER,
    DEFAULT_VALUE_BATTERY_CELLS_NUMBER,
    DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0,
    DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1,
    DEFAULT_VALUE_PAS_MAX_CADENCE,
    DEFAULT_VALUE_CONFIG_0,
    DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_1,
    DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_2,
    DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_3,
    DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_4,
    DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_5,
    DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_6,
    DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_7,
    DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_8,
    DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_9,
    DEFAULT_VALUE_NUMBER_OF_ASSIST_LEVELS,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_STATE,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_2,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_3,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_4,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_5,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_6,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_7,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_8,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_9,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_TIME,
    DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_FADE_TIME,
    DEFAULT_VALUE_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT,
    DEFAULT_VALUE_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT,
    DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0,
    DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1,
    DEFAULT_VALUE_LCD_POWER_OFF_TIME,
    DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS,
    DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS,
    DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_0,
    DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_1,
    DEFAULT_VALUE_OFFROAD_FUNC_ENABLED,
    DEFAULT_VALUE_OFFROAD_MODE_ENABLED_ON_STARTUP,
    DEFAULT_VALUE_OFFROAD_SPEED_LIMIT,
    DEFAULT_VALUE_OFFROAD_POWER_LIMIT_ENABLED,
    DEFAULT_VALUE_OFFROAD_POWER_LIMIT_DIV25,
    DEFAULT_VALUE_ODOMETER_X10,
    DEFAULT_VALUE_ODOMETER_X10,
    DEFAULT_VALUE_ODOMETER_X10
  };

static void eeprom_write_array (uint8_t *array, uint8_t ui8_len);
static void eeprom_read_values_to_variables (void);
static void variables_to_array (uint8_t *ui8_array);

void eeprom_init (void)
{
  uint8_t ui8_data;

  // start by reading address 0 and see if value is different from our key,
  // if so mean that eeprom memory is clean and we need to populate: should happen after erasing the microcontroller
  ui8_data = FLASH_ReadByte (ADDRESS_KEY);
  if (ui8_data != KEY) // verify if our key exist
  {
    eeprom_write_array (array_default_values, ((uint32_t) EEPROM_BYTES_STORED));
  }
}

void eeprom_init_variables (void)
{
  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables ();

  eeprom_read_values_to_variables ();

//  // now verify if any EEPROM saved value is out of valid range and if so,
//  // write correct ones and read again
//  if ((p_configuration_variables->ui8_number_of_assist_levels < 1) ||
//      (p_configuration_variables->ui8_number_of_assist_levels > 9) ||
//      (p_configuration_variables->ui16_wheel_perimeter > 3000) ||
//      (p_configuration_variables->ui16_wheel_perimeter < 750) ||
//      (p_configuration_variables->ui8_wheel_max_speed > 99) ||
//      (p_configuration_variables->ui8_units_type > 1) ||
//      (p_configuration_variables->ui32_wh_x10_offset > 99900) ||
//      (p_configuration_variables->ui32_wh_x10_100_percent > 99900) ||
//      (p_configuration_variables->ui8_show_numeric_battery_soc > 2) ||
//      (p_configuration_variables->ui8_odometer_field_state > 4) ||
//      (p_configuration_variables->ui8_battery_max_current > 100) ||
//      (p_configuration_variables->ui8_target_max_battery_power_div10 > 195) ||
//      (p_configuration_variables->ui8_startup_motor_power_boost_state > 3) ||
//      (p_configuration_variables->ui8_battery_cells_number > 15) ||
//      (p_configuration_variables->ui8_battery_cells_number < 6) ||
//      (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 > 630) ||
//      (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 < 160) ||
//      (p_configuration_variables->ui8_cruise_control > 1) ||
//      (p_configuration_variables->ui8_motor_voltage_type > 1) ||
//      (p_configuration_variables->ui8_motor_temperature_min_value_to_limit < 124) ||
//      (p_configuration_variables->ui8_motor_temperature_max_value_to_limit < 124) ||
//      (p_configuration_variables->ui8_motor_assistance_startup_without_pedal_rotation > 1))
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

  p_configuration_variables->ui8_assist_level = FLASH_ReadByte (ADDRESS_ASSIST_LEVEL);

  ui16_temp = FLASH_ReadByte (ADDRESS_WHEEL_PERIMETER_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_WHEEL_PERIMETER_1);
  ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
  p_configuration_variables->ui16_wheel_perimeter = ui16_temp;

  p_configuration_variables->ui8_wheel_max_speed = FLASH_ReadByte (ADDRESS_MAX_WHEEL_SPEED);
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

  p_configuration_variables->ui8_show_numeric_battery_soc = FLASH_ReadByte (ADDRESS_SHOW_NUMERIC_BATTERY_SOC);
  p_configuration_variables->ui8_odometer_field_state = FLASH_ReadByte (ADDRESS_ODOMETER_FIELD_STATE);
  p_configuration_variables->ui8_battery_max_current = FLASH_ReadByte (ADDRESS_BATTERY_MAX_CURRENT);
  p_configuration_variables->ui8_target_max_battery_power = FLASH_ReadByte (ADDRESS_TARGET_MAX_BATTERY_POWER);
  p_configuration_variables->ui8_battery_cells_number = FLASH_ReadByte (ADDRESS_BATTERY_CELLS_NUMBER);

  ui16_temp = FLASH_ReadByte (ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1);
  ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
  p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 = ui16_temp;

  p_configuration_variables->ui8_pas_max_cadence = FLASH_ReadByte (ADDRESS_PAS_MAX_CADENCE);

  ui8_temp = FLASH_ReadByte (ADDRESS_CONFIG_0);
  p_configuration_variables->ui8_motor_voltage_type = ui8_temp & 1;
  p_configuration_variables->ui8_motor_assistance_startup_without_pedal_rotation = (ui8_temp & 2) >> 1;
  p_configuration_variables->ui8_throttle_adc_measures_motor_temperature = (ui8_temp & 4) >> 2;
  p_configuration_variables->ui8_temperature_field_config = (ui8_temp & 24) >> 3;

  p_configuration_variables->ui8_number_of_assist_levels = FLASH_ReadByte (ADDRESS_NUMBER_OF_ASSIST_LEVELS);
  for (ui8_index = 0; ui8_index < 9; ui8_index++)
  {
    p_configuration_variables->ui8_assist_level_power [ui8_index] = FLASH_ReadByte (ADDRESS_ASSIST_LEVEL_FACTOR_1 + ui8_index);
  }

  p_configuration_variables->ui8_startup_motor_power_boost_state = FLASH_ReadByte (ADDRESS_STARTUP_MOTOR_POWER_BOOST_STATE);
  p_configuration_variables->ui8_startup_motor_power_boost_time = FLASH_ReadByte (ADDRESS_STARTUP_MOTOR_POWER_BOOST_TIME);
  for (ui8_index = 0; ui8_index < 9; ui8_index++)
  {
    p_configuration_variables->ui8_startup_motor_power_boost [ui8_index] = FLASH_ReadByte (ADDRESS_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1 + ui8_index);
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

  p_configuration_variables->ui8_offroad_func_enabled = FLASH_ReadByte (ADDRESS_DEFAULT_VALUE_OFFROAD_FUNC_ENABLED);
  p_configuration_variables->ui8_offroad_enabled_on_startup = FLASH_ReadByte (ADDRESS_DEFAULT_VALUE_OFFROAD_MODE_ENABLED_ON_STARTUP);
  p_configuration_variables->ui8_offroad_speed_limit = FLASH_ReadByte (ADDRESS_DEFAULT_VALUE_OFFROAD_SPEED_LIMIT);
  p_configuration_variables->ui8_offroad_power_limit_enabled = FLASH_ReadByte (ADDRESS_DEFAULT_VALUE_OFFROAD_POWER_LIMIT_ENABLED);
  p_configuration_variables->ui8_offroad_power_limit_div25 = FLASH_ReadByte (ADDRESS_DEFAULT_VALUE_OFFROAD_POWER_LIMIT_DIV25);

  ui32_temp = FLASH_ReadByte (ADDRESS_ODOMETER_X10_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_ODOMETER_X10_1);
  ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
  ui8_temp = FLASH_ReadByte (ADDRESS_ODOMETER_X10_2);
  ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
  p_configuration_variables->ui32_odometer_x10 = ui32_temp;
}

void eeprom_write_variables (void)
{
  uint8_t array_variables [EEPROM_BYTES_STORED];
  variables_to_array (array_variables);
  eeprom_write_array (array_variables, ((uint8_t) EEPROM_BYTES_STORED));
}

static void variables_to_array (uint8_t *ui8_array)
{
  uint8_t ui8_index;

  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables ();

  ui8_array [0] = KEY;
  ui8_array [1] = p_configuration_variables->ui8_assist_level;
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
  ui8_array [14] = p_configuration_variables->ui8_show_numeric_battery_soc;
  ui8_array [15] = p_configuration_variables->ui8_odometer_field_state;
  ui8_array [16] = p_configuration_variables->ui8_battery_max_current;
  ui8_array [17] = p_configuration_variables->ui8_target_max_battery_power;
  ui8_array [18] = p_configuration_variables->ui8_battery_cells_number;
  ui8_array [19] = p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 & 255;
  ui8_array [20] = (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 >> 8) & 255;
  ui8_array [21] = p_configuration_variables->ui8_pas_max_cadence;
  ui8_array [22] = (p_configuration_variables->ui8_motor_voltage_type & 1) |
                      ((p_configuration_variables->ui8_motor_assistance_startup_without_pedal_rotation & 1) << 1) |
                      ((p_configuration_variables->ui8_throttle_adc_measures_motor_temperature & 1) << 2) |
                      ((p_configuration_variables->ui8_temperature_field_config & 3) << 3);

  for (ui8_index = 0; ui8_index < 9; ui8_index++)
  {
    ui8_array [23 + ui8_index] = p_configuration_variables->ui8_assist_level_power [ui8_index];
  }
  ui8_array [32] = p_configuration_variables->ui8_number_of_assist_levels;

  ui8_array [33] = p_configuration_variables->ui8_startup_motor_power_boost_state;
  for (ui8_index = 0; ui8_index < 9; ui8_index++)
  {
    ui8_array [34 + ui8_index] = p_configuration_variables->ui8_startup_motor_power_boost [ui8_index];
  }
  ui8_array [43] = p_configuration_variables->ui8_startup_motor_power_boost_time;
  ui8_array [44] = p_configuration_variables->ui8_startup_motor_power_boost_fade_time;
  ui8_array [45] = p_configuration_variables->ui8_motor_temperature_min_value_to_limit;
  ui8_array [46] = p_configuration_variables->ui8_motor_temperature_max_value_to_limit;

  ui8_array [47] = p_configuration_variables->ui16_battery_voltage_reset_wh_counter_x10 & 255;
  ui8_array [48] = (p_configuration_variables->ui16_battery_voltage_reset_wh_counter_x10 >> 8) & 255;

  ui8_array [49] = p_configuration_variables->ui8_lcd_power_off_time_minutes;
  ui8_array [50] = p_configuration_variables->ui8_lcd_backlight_on_brightness;
  ui8_array [51] = p_configuration_variables->ui8_lcd_backlight_off_brightness;

  ui8_array [52] = p_configuration_variables->ui16_battery_pack_resistance_x1000 & 255;
  ui8_array [53] = (p_configuration_variables->ui16_battery_pack_resistance_x1000 >> 8) & 255;

  ui8_array [54] = p_configuration_variables->ui8_offroad_func_enabled;
  ui8_array [55] = p_configuration_variables->ui8_offroad_enabled_on_startup;
  ui8_array [56] = p_configuration_variables->ui8_offroad_speed_limit;
  ui8_array [57] = p_configuration_variables->ui8_offroad_power_limit_enabled;
  ui8_array [58] = p_configuration_variables->ui8_offroad_power_limit_div25;

  ui8_array [59] = p_configuration_variables->ui32_odometer_x10 & 255;
  ui8_array [60] = (p_configuration_variables->ui32_odometer_x10 >> 8) & 255;
  ui8_array [61] = (p_configuration_variables->ui32_odometer_x10 >> 16) & 255;
}

static void eeprom_write_array (uint8_t *array, uint8_t ui8_len)
{
  uint8_t ui8_i;

  if (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == 0)
  {
    FLASH_Unlock (FLASH_MEMTYPE_DATA);
  }

  for (ui8_i = 0; ui8_i < ui8_len; ui8_i++)
  {
    FLASH_ProgramByte (((uint32_t) EEPROM_BASE_ADDRESS) + ((uint32_t) ui8_i), *array++);
  }

  FLASH_Lock (FLASH_MEMTYPE_DATA);
}

void eeprom_erase_key_value (void)
{
  uint8_t array_variables [1] = { 0 }; // key value is the first element, let's keep at 0
  eeprom_write_array (array_variables, 1);
}
