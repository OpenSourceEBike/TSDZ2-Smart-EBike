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

static const uint8_t ui8_default_array[EEPROM_BYTES_STORED] = 
{
  DEFAULT_VALUE_KEY,                                                  // 0 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ASSIST_LEVEL,                                         // 1 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_NUMBER_OF_ASSIST_LEVELS,                              // 2 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WHEEL_PERIMETER_0,                                    // 3 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WHEEL_PERIMETER_1,                                    // 4 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WHEEL_MAX_SPEED,                                      // 5 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_UNITS_TYPE,                                           // 6 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WH_OFFSET,                                            // 7 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WH_OFFSET,                                            // 8 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WH_OFFSET,                                            // 9 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WH_OFFSET,                                            // 1 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_HW_X10_100_PERCENT,                                   // 11 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_HW_X10_100_PERCENT,                                   // 12 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_HW_X10_100_PERCENT,                                   // 13 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_HW_X10_100_PERCENT,                                   // 14 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_SOC_FUNCTION_ENABLED,                         // 15 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_FIELD_STATE,                                 // 16 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_MAX_CURRENT,                                  // 17 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TARGET_MAX_BATTERY_POWER,                             // 18 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_CELLS_NUMBER,                                 // 19 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0,                    // 20 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1,                    // 21 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_MOTOR_TYPE,                                           // 22 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_FUNCTION_ENABLED,                        // 23 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_1,                                 // 24 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_2,                                 // 25 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_3,                                 // 26 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_4,                                 // 27 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_5,                                 // 28 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_6,                                 // 29 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_7,                                 // 30 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_8,                                 // 31 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_9,                                 // 32 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_OPTIONAL_ADC_FUNCTION,                                // 33 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT,                    // 34 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT,                    // 35 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0,               // 36 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1,               // 37 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_LCD_POWER_OFF_TIME,                                   // 38 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS,                          // 39 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS,                         // 40 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_0,                            // 41 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_1,                            // 42 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_FUNCTION_ENABLED,                         // 43 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_SPEED_LIMIT,                              // 44 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_POWER_LIMIT_ENABLED,                      // 45 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_POWER_LIMIT_DIV25,                        // 46 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_THROTTLE_ENABLED,                         // 47 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_STREET_MODE_CRUISE_ENABLED,                           // 48 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_X10,                                         // 49 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_X10,                                         // 50 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_X10,                                         // 51 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TRIP_X10,                                             // 52 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TRIP_X10,                                             // 53 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TRIP_X10,                                             // 54 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_0,                           // 55 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_1,                           // 56 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_2,                           // 57 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_3,                           // 58 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_4,                           // 59 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_5,                           // 60 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_6,                           // 61 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_MAX_WHEEL_SPEED_IMPERIAL,                             // 62 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TIME_MEASUREMENT_FIELD_STATE,                         // 63 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TOTAL_SECOND_TTM,                                     // 64 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TOTAL_MINUTE_TTM,                                     // 65 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TOTAL_HOUR_TTM_0,                                     // 66 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TOTAL_HOUR_TTM_1,                                     // 67 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_MOTOR_ACCELERATION,                                   // 68 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_FUNCTION_ENABLED,                         // 69 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_1,                                  // 70 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_2,                                  // 71 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_3,                                  // 72 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_4,                                  // 73 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_5,                                  // 74 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_6,                                  // 75 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_7,                                  // 76 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_8,                                  // 77 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_9,                                  // 78 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CRUISE_FUNCTION_ENABLED,                              // 79 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED,             // 80 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CRUISE_FUNCTION_TARGET_SPEED_KPH,                     // 81 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CRUISE_FUNCTION_TARGET_SPEED_MPH,                     // 82 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED,                // 83 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_WHEEL_SPEED_FIELD_STATE,                              // 84 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_DISTANCE_DATA_ODOMETER_FIELD,                    // 85 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_BATTERY_STATE_ODOMETER_FIELD,                    // 86 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_PEDAL_DATA_ODOMETER_FIELD,                       // 87 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD,                 // 88 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_WHEEL_SPEED_ODOMETER_FIELD,                      // 89 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_ENERGY_DATA_ODOMETER_FIELD,                      // 90 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD,                // 91 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_SHOW_BATTERY_SOC_ODOMETER_FIELD,                      // 92 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_MAIN_SCREEN_POWER_MENU_ENABLED,                       // 93 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TORQUE_ASSIST_FUNCTION_ENABLED,                       // 94 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_1,                                // 95 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_2,                                // 96 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_3,                                // 97 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_4,                                // 98 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_5,                                // 99 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_6,                                // 100 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_7,                                // 101 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_8,                                // 102 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_9,                                // 103 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_ASSIST_FUNCTION_ENABLED,                      // 104 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_1,                               // 105 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_2,                               // 106 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_3,                               // 107 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_4,                               // 108 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_5,                               // 109 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_6,                               // 110 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_7,                               // 111 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_8,                               // 112 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_9,                               // 113 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_SENSOR_MODE,                                  // 114 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_EMTB_ASSIST_FUNCTION_ENABLED,                         // 115 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_EMTB_ASSIST_SENSITIVITY,                              // 116 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100,                // 117 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TEMPERATURE_FIELD_STATE,                              // 118 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_0,           // 119 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_1            // 120 + EEPROM_BASE_ADDRESS (Array index)
};


void eeprom_init(void)
{
  volatile uint32_t ui32_delay_counter = 0;
  
  // deinitialize EEPROM
  FLASH_DeInit();
  
  // time delay
  for (ui32_delay_counter = 0; ui32_delay_counter < 160000; ++ui32_delay_counter) {}
  
  // select and set programming time mode
  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD); // standard programming (erase and write) time mode
  //FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_TPROG); // fast programming (write only) time mode
  
  // time delay
  for (ui32_delay_counter = 0; ui32_delay_counter < 160000; ++ui32_delay_counter) {}
  
  // read key
  volatile uint8_t ui8_saved_key = FLASH_ReadByte(ADDRESS_KEY);
  
  // check if key is valid
  if (ui8_saved_key != DEFAULT_VALUE_KEY)
  {
    // set to default values
    EEPROM_controller(SET_TO_DEFAULT);
  }
  
  // read from EEPROM
  EEPROM_controller(READ_FROM_MEMORY);
}



void EEPROM_controller(uint8_t ui8_operation)
{
  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables();
  
  uint8_t ui8_array[EEPROM_BYTES_STORED];
  uint8_t ui8_temp;
  uint16_t ui16_temp;
  uint32_t ui32_temp;
  uint8_t ui8_i;

  // unlock memory
  FLASH_Unlock(FLASH_MEMTYPE_DATA);
  
  // wait until data EEPROM area unlocked flag is set
  while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET) {}
  
  // select EEPROM operation
  switch (ui8_operation)
  {
    
    
    /********************************************************************************************************************************************************/
    
    
    case SET_TO_DEFAULT:
    
      // write array of variables to EEPROM, write key last
      for (ui8_i = EEPROM_BYTES_STORED; ui8_i > 0; ui8_i--)
      {
        // get address
        uint32_t ui32_default_address = (uint32_t) ui8_i - 1 + EEPROM_BASE_ADDRESS;
        
        // get value
        uint8_t ui8_default_variable_value = ui8_default_array[ui8_i - 1];
        
        // write variable value to EEPROM
        FLASH_ProgramByte(ui32_default_address, ui8_default_variable_value);
        
        // read value from EEPROM for validation
        volatile uint8_t ui8_saved_default_value = FLASH_ReadByte(ui32_default_address);
        
        // if write was not successful, rewrite
        if (ui8_saved_default_value != ui8_default_variable_value) { ui8_i = EEPROM_BYTES_STORED; }
      }
      
    break;
    
    
    /********************************************************************************************************************************************************/
    
    
    case READ_FROM_MEMORY:
    
      // assist level
      p_configuration_variables->ui8_assist_level = FLASH_ReadByte(ADDRESS_ASSIST_LEVEL);
      p_configuration_variables->ui8_number_of_assist_levels = FLASH_ReadByte(ADDRESS_NUMBER_OF_ASSIST_LEVELS);
      
      // power assist
      p_configuration_variables->ui8_power_assist_function_enabled = FLASH_ReadByte(ADDRESS_POWER_ASSIST_FUNCTION_ENABLED);
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        p_configuration_variables->ui8_power_assist_level[ui8_i] = FLASH_ReadByte(ADDRESS_POWER_ASSIST_LEVEL_1 + ui8_i);
      }
      
      // torque assist
      p_configuration_variables->ui8_torque_assist_function_enabled = FLASH_ReadByte(ADDRESS_TORQUE_ASSIST_FUNCTION_ENABLED);
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        p_configuration_variables->ui8_torque_assist_level[ui8_i] = FLASH_ReadByte(ADDRESS_TORQUE_ASSIST_LEVEL_1 + ui8_i);
      }
      
      // cadence assist
      p_configuration_variables->ui8_cadence_assist_function_enabled = FLASH_ReadByte(ADDRESS_CADENCE_ASSIST_FUNCTION_ENABLED);
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        p_configuration_variables->ui8_cadence_assist_level[ui8_i] = FLASH_ReadByte(ADDRESS_CADENCE_ASSIST_LEVEL_1 + ui8_i);
      }
      
      // eMTB assist
      p_configuration_variables->ui8_eMTB_assist_function_enabled = FLASH_ReadByte(ADDRESS_EMTB_ASSIST_FUNCTION_ENABLED);
      p_configuration_variables->ui8_eMTB_assist_sensitivity = FLASH_ReadByte(ADDRESS_EMTB_ASSIST_SENSITIVITY);
      
      // walk assist
      p_configuration_variables->ui8_walk_assist_function_enabled = FLASH_ReadByte(ADDRESS_WALK_ASSIST_FUNCTION_ENABLED);
      for (ui8_i = 0; ui8_i < 10; ui8_i++)
      {
        p_configuration_variables->ui8_walk_assist_level[ui8_i] = FLASH_ReadByte(ADDRESS_WALK_ASSIST_LEVEL_1 + ui8_i);
      }
      
      // cruise
      p_configuration_variables->ui8_cruise_function_enabled = FLASH_ReadByte(ADDRESS_CRUISE_FUNCTION_ENABLED);
      p_configuration_variables->ui8_cruise_function_set_target_speed_enabled = FLASH_ReadByte(ADDRESS_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED);
      p_configuration_variables->ui8_cruise_function_target_speed_kph = FLASH_ReadByte(ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_KPH);
      p_configuration_variables->ui8_cruise_function_target_speed_mph = FLASH_ReadByte(ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_MPH);
      p_configuration_variables->ui8_show_cruise_function_set_target_speed = FLASH_ReadByte(ADDRESS_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED);    

      // wheel perimeter
      ui16_temp = FLASH_ReadByte(ADDRESS_WHEEL_PERIMETER_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_WHEEL_PERIMETER_1);
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_wheel_perimeter = ui16_temp;

      // max wheel speed
      p_configuration_variables->ui8_wheel_max_speed = FLASH_ReadByte(ADDRESS_MAX_WHEEL_SPEED);
      p_configuration_variables->ui8_wheel_max_speed_imperial = FLASH_ReadByte(ADDRESS_MAX_WHEEL_SPEED_IMPERIAL);
      
      // units
      p_configuration_variables->ui8_units_type = FLASH_ReadByte(ADDRESS_UNITS_TYPE);

      // odometer sub field states
      p_configuration_variables->ui8_odometer_field_state = FLASH_ReadByte(ADDRESS_ODOMETER_FIELD_STATE);
      p_configuration_variables->ui8_odometer_sub_field_state_0 = FLASH_ReadByte(ADDRESS_ODOMETER_SUB_FIELD_STATE_0);
      p_configuration_variables->ui8_odometer_sub_field_state_1 = FLASH_ReadByte(ADDRESS_ODOMETER_SUB_FIELD_STATE_1);
      p_configuration_variables->ui8_odometer_sub_field_state_2 = FLASH_ReadByte(ADDRESS_ODOMETER_SUB_FIELD_STATE_2);
      p_configuration_variables->ui8_odometer_sub_field_state_3 = FLASH_ReadByte(ADDRESS_ODOMETER_SUB_FIELD_STATE_3);
      p_configuration_variables->ui8_odometer_sub_field_state_4 = FLASH_ReadByte(ADDRESS_ODOMETER_SUB_FIELD_STATE_4); 
      p_configuration_variables->ui8_odometer_sub_field_state_5 = FLASH_ReadByte(ADDRESS_ODOMETER_SUB_FIELD_STATE_5); 
      p_configuration_variables->ui8_odometer_sub_field_state_6 = FLASH_ReadByte(ADDRESS_ODOMETER_SUB_FIELD_STATE_6);
      
      // time measurement
      p_configuration_variables->ui8_time_measurement_field_state = FLASH_ReadByte(ADDRESS_TIME_MEASUREMENT_FIELD_STATE);
      p_configuration_variables->ui8_total_second_TTM = FLASH_ReadByte(ADDRESS_TOTAL_SECOND_TTM);
      p_configuration_variables->ui8_total_minute_TTM = FLASH_ReadByte(ADDRESS_TOTAL_MINUTE_TTM);
      ui16_temp = FLASH_ReadByte(ADDRESS_TOTAL_HOUR_TTM_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_TOTAL_HOUR_TTM_1);
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_total_hour_TTM = ui16_temp;

      // motor type
      p_configuration_variables->ui8_motor_type = FLASH_ReadByte(ADDRESS_MOTOR_TYPE);
      
      // optional ADC function
      p_configuration_variables->ui8_optional_ADC_function = FLASH_ReadByte(ADDRESS_OPTIONAL_ADC_FUNCTION);
      
      // tempereture field state
      p_configuration_variables->ui8_temperature_field_state = FLASH_ReadByte(ADDRESS_TEMPERATURE_FIELD_STATE);
      
      // motor temperature protection
      p_configuration_variables->ui8_motor_temperature_min_value_to_limit = FLASH_ReadByte(ADDRESS_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT);
      p_configuration_variables->ui8_motor_temperature_max_value_to_limit = FLASH_ReadByte(ADDRESS_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT);
      
      // display
      p_configuration_variables->ui8_lcd_power_off_time_minutes = FLASH_ReadByte(ADDRESS_LCD_POWER_OFF_TIME);
      p_configuration_variables->ui8_lcd_backlight_on_brightness = FLASH_ReadByte(ADDRESS_LCD_BACKLIGHT_ON_BRIGHTNESS);
      p_configuration_variables->ui8_lcd_backlight_off_brightness = FLASH_ReadByte(ADDRESS_LCD_BACKLIGHT_OFF_BRIGHTNESS);
      
      // battery
      p_configuration_variables->ui8_battery_max_current = FLASH_ReadByte(ADDRESS_BATTERY_MAX_CURRENT);
      p_configuration_variables->ui8_target_max_battery_power_div25 = FLASH_ReadByte(ADDRESS_TARGET_MAX_BATTERY_POWER);
      p_configuration_variables->ui8_battery_cells_number = FLASH_ReadByte(ADDRESS_BATTERY_CELLS_NUMBER);
      
      ui16_temp = FLASH_ReadByte(ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1);
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 = ui16_temp;
      
      ui16_temp = FLASH_ReadByte(ADDRESS_BATTERY_PACK_RESISTANCE_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_BATTERY_PACK_RESISTANCE_1);
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_battery_pack_resistance_x1000 = ui16_temp;
      
      p_configuration_variables->ui8_battery_SOC_function_enabled = FLASH_ReadByte(ADDRESS_BATTERY_SOC_FUNCTION_ENABLED);
      
      ui32_temp = FLASH_ReadByte(ADDRESS_HW_X10_OFFSET_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_HW_X10_OFFSET_1);
      ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
      ui8_temp = FLASH_ReadByte(ADDRESS_HW_X10_OFFSET_2);
      ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
      ui8_temp = FLASH_ReadByte(ADDRESS_HW_X10_OFFSET_3);
      ui32_temp += (((uint32_t) ui8_temp << 24) & 0xff000000);
      p_configuration_variables->ui32_wh_x10_offset = ui32_temp;

      ui32_temp = FLASH_ReadByte(ADDRESS_HW_X10_100_PERCENT_OFFSET_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_HW_X10_100_PERCENT_OFFSET_1);
      ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
      ui8_temp = FLASH_ReadByte(ADDRESS_HW_X10_100_PERCENT_OFFSET_2);
      ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
      ui8_temp = FLASH_ReadByte(ADDRESS_HW_X10_100_PERCENT_OFFSET_3);
      ui32_temp += (((uint32_t) ui8_temp << 24) & 0xff000000);
      p_configuration_variables->ui32_wh_x10_100_percent = ui32_temp;
      
      ui16_temp = FLASH_ReadByte(ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1);
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_battery_voltage_reset_wh_counter_x10 = ui16_temp;

      // street mode
      p_configuration_variables->ui8_street_mode_function_enabled = FLASH_ReadByte(ADDRESS_STREET_MODE_FUNCTION_ENABLED);
      p_configuration_variables->ui8_street_mode_speed_limit = FLASH_ReadByte(ADDRESS_STREET_MODE_SPEED_LIMIT);
      p_configuration_variables->ui8_street_mode_power_limit_enabled = FLASH_ReadByte(ADDRESS_STREET_MODE_POWER_LIMIT_ENABLED);
      p_configuration_variables->ui8_street_mode_power_limit_div25 = FLASH_ReadByte(ADDRESS_STREET_MODE_POWER_LIMIT_DIV25);
      p_configuration_variables->ui8_street_mode_throttle_enabled = FLASH_ReadByte(ADDRESS_STREET_MODE_THROTTLE_ENABLED);
      p_configuration_variables->ui8_street_mode_cruise_enabled = FLASH_ReadByte(ADDRESS_STREET_MODE_CRUISE_ENABLED);
      
      
      // odometer variable
      ui32_temp = FLASH_ReadByte(ADDRESS_ODOMETER_X10_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_ODOMETER_X10_1);
      ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
      ui8_temp = FLASH_ReadByte(ADDRESS_ODOMETER_X10_2);
      ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
      p_configuration_variables->ui32_odometer_x10 = ui32_temp;
      
      
      // trip distance variable
      ui32_temp = FLASH_ReadByte(ADDRESS_TRIP_X10_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_TRIP_X10_1);
      ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
      ui8_temp = FLASH_ReadByte(ADDRESS_TRIP_X10_2);
      ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
      p_configuration_variables->ui32_trip_x10 = ui32_temp;
      
      
      // motor acceleration
      p_configuration_variables->ui8_motor_acceleration = FLASH_ReadByte(ADDRESS_MOTOR_ACCELERATION);
      
      
      // wheel speed measurement
      p_configuration_variables->ui8_wheel_speed_field_state = FLASH_ReadByte(ADDRESS_WHEEL_SPEED_FIELD_STATE);
      
      
      // show variables in odometer field
      p_configuration_variables->ui8_show_distance_data_odometer_field = FLASH_ReadByte(ADDRESS_SHOW_DISTANCE_DATA_ODOMETER_FIELD);
      p_configuration_variables->ui8_show_battery_state_odometer_field = FLASH_ReadByte(ADDRESS_SHOW_BATTERY_STATE_ODOMETER_FIELD);
      p_configuration_variables->ui8_show_pedal_data_odometer_field = FLASH_ReadByte(ADDRESS_SHOW_PEDAL_DATA_ODOMETER_FIELD);
      p_configuration_variables->ui8_show_time_measurement_odometer_field = FLASH_ReadByte(ADDRESS_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD);
      p_configuration_variables->ui8_show_wheel_speed_odometer_field = FLASH_ReadByte(ADDRESS_SHOW_WHEEL_SPEED_ODOMETER_FIELD);
      p_configuration_variables->ui8_show_energy_data_odometer_field = FLASH_ReadByte(ADDRESS_SHOW_ENERGY_DATA_ODOMETER_FIELD);
      p_configuration_variables->ui8_show_motor_temperature_odometer_field = FLASH_ReadByte(ADDRESS_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD);
      p_configuration_variables->ui8_show_battery_SOC_odometer_field = FLASH_ReadByte(ADDRESS_SHOW_BATTERY_SOC_ODOMETER_FIELD);
      
      
      // main screen power menu enable 
      p_configuration_variables->ui8_main_screen_power_menu_enabled = FLASH_ReadByte(ADDRESS_MAIN_SCREEN_POWER_MENU_ENABLED);
      
      
      // pedal torque conversion
      p_configuration_variables->ui8_pedal_torque_per_10_bit_ADC_step_x100 = FLASH_ReadByte(ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100);
      
      
      // cadence sensor mode
      p_configuration_variables->ui8_cadence_sensor_mode = FLASH_ReadByte(ADDRESS_CADENCE_SENSOR_MODE);
      ui16_temp = FLASH_ReadByte(ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_1);
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_cadence_sensor_pulse_high_percentage_x10 = ui16_temp;
      
    break;
    
    
    /********************************************************************************************************************************************************/
    
    
    case WRITE_TO_MEMORY:
    
      // key
      ui8_array[ADDRESS_KEY - EEPROM_BASE_ADDRESS] = DEFAULT_VALUE_KEY;
      
      // assist level
      ui8_array[ADDRESS_ASSIST_LEVEL - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_assist_level;
      
      // wheel perimeter
      ui8_array[ADDRESS_WHEEL_PERIMETER_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_wheel_perimeter & 255;
      ui8_array[ADDRESS_WHEEL_PERIMETER_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_wheel_perimeter >> 8) & 255;
      
      // max wheel speed
      ui8_array[ADDRESS_MAX_WHEEL_SPEED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_wheel_max_speed;
      ui8_array[ADDRESS_MAX_WHEEL_SPEED_IMPERIAL - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_wheel_max_speed_imperial;
      
      // units
      ui8_array[ADDRESS_UNITS_TYPE - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_units_type;

      // motor type
      ui8_array[ADDRESS_MOTOR_TYPE - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_motor_type;
      
      // optional ADC function
      ui8_array[ADDRESS_OPTIONAL_ADC_FUNCTION - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_optional_ADC_function;
      
      // temperature field state
      ui8_array[ADDRESS_TEMPERATURE_FIELD_STATE - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_temperature_field_state;
      
      // number of assist levels
      ui8_array[ADDRESS_NUMBER_OF_ASSIST_LEVELS - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_number_of_assist_levels;
      
      // power assist
      ui8_array[ADDRESS_POWER_ASSIST_FUNCTION_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_power_assist_function_enabled;
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        ui8_array[ADDRESS_POWER_ASSIST_LEVEL_1 - EEPROM_BASE_ADDRESS + ui8_i] = p_configuration_variables->ui8_power_assist_level[ui8_i];
      }
      
      // torque assist
      ui8_array[ADDRESS_TORQUE_ASSIST_FUNCTION_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_torque_assist_function_enabled;
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        ui8_array[ADDRESS_TORQUE_ASSIST_LEVEL_1 - EEPROM_BASE_ADDRESS + ui8_i] = p_configuration_variables->ui8_torque_assist_level[ui8_i];
      }
      
      // cadence assist
      ui8_array[ADDRESS_CADENCE_ASSIST_FUNCTION_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_cadence_assist_function_enabled;
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        ui8_array[ADDRESS_CADENCE_ASSIST_LEVEL_1 - EEPROM_BASE_ADDRESS + ui8_i] = p_configuration_variables->ui8_cadence_assist_level[ui8_i];
      }
      
      // eMTB assist function variables
      ui8_array[ADDRESS_EMTB_ASSIST_FUNCTION_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_eMTB_assist_function_enabled;
      ui8_array[ADDRESS_EMTB_ASSIST_SENSITIVITY - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_eMTB_assist_sensitivity;
      
      // walk assist
      ui8_array[ADDRESS_WALK_ASSIST_FUNCTION_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_walk_assist_function_enabled;
      for (ui8_i = 0; ui8_i < 10; ui8_i++)
      {
        ui8_array[ADDRESS_WALK_ASSIST_LEVEL_1 - EEPROM_BASE_ADDRESS + ui8_i] = p_configuration_variables->ui8_walk_assist_level[ui8_i];
      }
      
      // cruise
      ui8_array[ADDRESS_CRUISE_FUNCTION_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_cruise_function_enabled;
      ui8_array[ADDRESS_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_cruise_function_set_target_speed_enabled;
      ui8_array[ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_KPH - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_cruise_function_target_speed_kph;
      ui8_array[ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_MPH - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_cruise_function_target_speed_mph;
      ui8_array[ADDRESS_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_show_cruise_function_set_target_speed; 
      
      // motor temperature protection
      ui8_array[ADDRESS_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_motor_temperature_min_value_to_limit;
      ui8_array[ADDRESS_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_motor_temperature_max_value_to_limit;
      
      // display
      ui8_array[ADDRESS_LCD_POWER_OFF_TIME - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_lcd_power_off_time_minutes;
      ui8_array[ADDRESS_LCD_BACKLIGHT_ON_BRIGHTNESS - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_lcd_backlight_on_brightness;
      ui8_array[ADDRESS_LCD_BACKLIGHT_OFF_BRIGHTNESS - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_lcd_backlight_off_brightness;
      
      // battery
      ui8_array[ADDRESS_BATTERY_MAX_CURRENT - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_battery_max_current;
      ui8_array[ADDRESS_TARGET_MAX_BATTERY_POWER - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_target_max_battery_power_div25;
      ui8_array[ADDRESS_BATTERY_CELLS_NUMBER - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_battery_cells_number;
      
      ui8_array[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 & 255;
      ui8_array[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 >> 8) & 255;
      
      ui8_array[ADDRESS_BATTERY_PACK_RESISTANCE_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_battery_pack_resistance_x1000 & 255;
      ui8_array[ADDRESS_BATTERY_PACK_RESISTANCE_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_battery_pack_resistance_x1000 >> 8) & 255;
      
      ui8_array[ADDRESS_BATTERY_SOC_FUNCTION_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_battery_SOC_function_enabled;
      
      ui8_array[ADDRESS_HW_X10_OFFSET_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui32_wh_x10_offset & 255;
      ui8_array[ADDRESS_HW_X10_OFFSET_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui32_wh_x10_offset >> 8) & 255;
      ui8_array[ADDRESS_HW_X10_OFFSET_2 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui32_wh_x10_offset >> 16) & 255;
      ui8_array[ADDRESS_HW_X10_OFFSET_3 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui32_wh_x10_offset >> 24) & 255;
      ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui32_wh_x10_100_percent & 255;
      ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui32_wh_x10_100_percent >> 8) & 255;
      ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_2 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui32_wh_x10_100_percent >> 16) & 255;
      ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_3 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui32_wh_x10_100_percent >> 24) & 255;
      
      ui8_array[ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_battery_voltage_reset_wh_counter_x10 & 255;
      ui8_array[ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_battery_voltage_reset_wh_counter_x10 >> 8) & 255;

      // street mode
      ui8_array[ADDRESS_STREET_MODE_FUNCTION_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_street_mode_function_enabled;
      ui8_array[ADDRESS_STREET_MODE_SPEED_LIMIT - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_street_mode_speed_limit;
      ui8_array[ADDRESS_STREET_MODE_POWER_LIMIT_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_street_mode_power_limit_enabled;
      ui8_array[ADDRESS_STREET_MODE_POWER_LIMIT_DIV25 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_street_mode_power_limit_div25;
      ui8_array[ADDRESS_STREET_MODE_THROTTLE_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_street_mode_throttle_enabled;
      ui8_array[ADDRESS_STREET_MODE_CRUISE_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_street_mode_cruise_enabled;
      
      // odometer
      ui8_array[ADDRESS_ODOMETER_X10_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui32_odometer_x10 & 255;
      ui8_array[ADDRESS_ODOMETER_X10_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui32_odometer_x10 >> 8) & 255;
      ui8_array[ADDRESS_ODOMETER_X10_2 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui32_odometer_x10 >> 16) & 255;
      
      // trip distance
      ui8_array[ADDRESS_TRIP_X10_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui32_trip_x10 & 255;
      ui8_array[ADDRESS_TRIP_X10_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui32_trip_x10 >> 8) & 255;
      ui8_array[ADDRESS_TRIP_X10_2 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui32_trip_x10 >> 16) & 255;

      // odometer sub field states
      ui8_array[ADDRESS_ODOMETER_FIELD_STATE - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_odometer_field_state;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_odometer_sub_field_state_0;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_1 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_odometer_sub_field_state_1;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_2 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_odometer_sub_field_state_2;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_3 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_odometer_sub_field_state_3;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_4 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_odometer_sub_field_state_4;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_5 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_odometer_sub_field_state_5;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_6 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_odometer_sub_field_state_6;
      
      // time measurement
      ui8_array[ADDRESS_TIME_MEASUREMENT_FIELD_STATE - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_time_measurement_field_state;
      ui8_array[ADDRESS_TOTAL_SECOND_TTM - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_total_second_TTM;
      ui8_array[ADDRESS_TOTAL_MINUTE_TTM - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_total_minute_TTM;
      ui8_array[ADDRESS_TOTAL_HOUR_TTM_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_total_hour_TTM & 255;
      ui8_array[ADDRESS_TOTAL_HOUR_TTM_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_total_hour_TTM >> 8) & 255;
      
      // motor acceleration
      ui8_array[ADDRESS_MOTOR_ACCELERATION - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_motor_acceleration; 
      
      // wheel speed field state
      ui8_array[ADDRESS_WHEEL_SPEED_FIELD_STATE - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_wheel_speed_field_state;
      
      // show odometer variables
      ui8_array[ADDRESS_SHOW_DISTANCE_DATA_ODOMETER_FIELD - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_show_distance_data_odometer_field;
      ui8_array[ADDRESS_SHOW_BATTERY_STATE_ODOMETER_FIELD - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_show_battery_state_odometer_field;
      ui8_array[ADDRESS_SHOW_PEDAL_DATA_ODOMETER_FIELD - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_show_pedal_data_odometer_field;
      ui8_array[ADDRESS_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_show_time_measurement_odometer_field;
      ui8_array[ADDRESS_SHOW_WHEEL_SPEED_ODOMETER_FIELD - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_show_wheel_speed_odometer_field;
      ui8_array[ADDRESS_SHOW_ENERGY_DATA_ODOMETER_FIELD - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_show_energy_data_odometer_field;
      ui8_array[ADDRESS_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_show_motor_temperature_odometer_field;
      ui8_array[ADDRESS_SHOW_BATTERY_SOC_ODOMETER_FIELD - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_show_battery_SOC_odometer_field;

      // main screen power menu enable
      ui8_array[ADDRESS_MAIN_SCREEN_POWER_MENU_ENABLED - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_main_screen_power_menu_enabled;

      // pedal torque conversion
      ui8_array[ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_pedal_torque_per_10_bit_ADC_step_x100;
      
      // cadence sensor mode
      ui8_array[ADDRESS_CADENCE_SENSOR_MODE - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_cadence_sensor_mode;
      ui8_array[ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_cadence_sensor_pulse_high_percentage_x10 & 255;
      ui8_array[ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_cadence_sensor_pulse_high_percentage_x10 >> 8) & 255;
      
      // write array of variables to EEPROM
      for (ui8_i = EEPROM_BYTES_STORED; ui8_i > 0; ui8_i--)
      {
        // get address
        uint32_t ui32_address = (uint32_t) ui8_i - 1 + EEPROM_BASE_ADDRESS;
        
        // get value
        uint8_t ui8_variable_value = ui8_array[ui8_i - 1];
        
        // write variable value to EEPROM
        FLASH_ProgramByte(ui32_address, ui8_variable_value);
        
        // read value from EEPROM for validation
        volatile uint8_t ui8_saved_value = FLASH_ReadByte(ui32_address);
        
        // if write was not successful, rewrite
        if (ui8_saved_value != ui8_variable_value) { ui8_i = EEPROM_BYTES_STORED; }
      }
      
    break;
  }
  
  // lock memory
  FLASH_Lock(FLASH_MEMTYPE_DATA);
}
