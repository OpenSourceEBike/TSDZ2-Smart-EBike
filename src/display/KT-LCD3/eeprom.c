/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho and Leon, 2019.
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
  DEFAULT_VALUE_KEY,                                                  // 0
  DEFAULT_VALUE_ASSIST_LEVEL,                                         // 1
  DEFAULT_VALUE_NUMBER_OF_ASSIST_LEVELS,                              // 2
  DEFAULT_VALUE_WHEEL_PERIMETER_0,                                    // 3
  DEFAULT_VALUE_WHEEL_PERIMETER_1,                                    // 4
  DEFAULT_VALUE_WHEEL_MAX_SPEED,                                      // 5
  DEFAULT_VALUE_UNITS_TYPE,                                           // 6
  DEFAULT_VALUE_WH_OFFSET,                                            // 7
  DEFAULT_VALUE_WH_OFFSET,                                            // 8
  DEFAULT_VALUE_WH_OFFSET,                                            // 9
  DEFAULT_VALUE_WH_OFFSET,                                            // 10
  DEFAULT_VALUE_HW_X10_100_PERCENT,                                   // 11
  DEFAULT_VALUE_HW_X10_100_PERCENT,                                   // 12
  DEFAULT_VALUE_HW_X10_100_PERCENT,                                   // 13
  DEFAULT_VALUE_HW_X10_100_PERCENT,                                   // 14
  DEFAULT_VALUE_BATTERY_SOC_FUNCTION_ENABLED,                         // 15
  DEFAULT_VALUE_ODOMETER_FIELD_STATE,                                 // 16
  DEFAULT_VALUE_BATTERY_MAX_CURRENT,                                  // 17
  DEFAULT_VALUE_TARGET_MAX_BATTERY_POWER,                             // 18
  DEFAULT_VALUE_BATTERY_CELLS_NUMBER,                                 // 19
  DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0,                    // 20
  DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1,                    // 21
  DEFAULT_VALUE_MOTOR_TYPE,                                           // 22
  DEFAULT_VALUE_POWER_ASSIST_FUNCTION_ENABLED,                        // 23
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_1,                                 // 24
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_2,                                 // 25
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_3,                                 // 26
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_4,                                 // 27
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_5,                                 // 28
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_6,                                 // 29
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_7,                                 // 30
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_8,                                 // 31
  DEFAULT_VALUE_POWER_ASSIST_LEVEL_9,                                 // 32
  DEFAULT_VALUE_OPTIONAL_ADC_FUNCTION,                                // 33
  DEFAULT_VALUE_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT,                    // 34
  DEFAULT_VALUE_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT,                    // 35
  DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0,               // 36
  DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1,               // 37
  DEFAULT_VALUE_LCD_POWER_OFF_TIME,                                   // 38
  DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS,                          // 39
  DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS,                         // 40
  DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_0,                            // 41
  DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_1,                            // 42
  DEFAULT_VALUE_STREET_MODE_FUNCTION_ENABLED,                         // 43
  DEFAULT_VALUE_STREET_MODE_SPEED_LIMIT,                              // 44
  DEFAULT_VALUE_STREET_MODE_POWER_LIMIT_ENABLED,                      // 45
  DEFAULT_VALUE_STREET_MODE_POWER_LIMIT_DIV25,                        // 46
  DEFAULT_VALUE_STREET_MODE_THROTTLE_ENABLED,                         // 47
  DEFAULT_VALUE_STREET_MODE_CRUISE_ENABLED,                           // 48
  DEFAULT_VALUE_ODOMETER_X10,                                         // 49
  DEFAULT_VALUE_ODOMETER_X10,                                         // 50
  DEFAULT_VALUE_ODOMETER_X10,                                         // 51
  DEFAULT_VALUE_TRIP_X10,                                             // 52
  DEFAULT_VALUE_TRIP_X10,                                             // 53
  DEFAULT_VALUE_TRIP_X10,                                             // 54
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_0,                           // 55
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_1,                           // 56
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_2,                           // 57
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_3,                           // 58
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_4,                           // 59
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_5,                           // 60
  DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_6,                           // 61
  DEFAULT_VALUE_MAX_WHEEL_SPEED_IMPERIAL,                             // 62
  DEFAULT_VALUE_TIME_MEASUREMENT_FIELD_STATE,                         // 63
  DEFAULT_VALUE_TOTAL_SECOND_TTM,                                     // 64
  DEFAULT_VALUE_TOTAL_MINUTE_TTM,                                     // 65
  DEFAULT_VALUE_TOTAL_HOUR_TTM_0,                                     // 66
  DEFAULT_VALUE_TOTAL_HOUR_TTM_1,                                     // 67
  DEFAULT_VALUE_MOTOR_ACCELERATION,                                   // 68
  DEFAULT_VALUE_WALK_ASSIST_FUNCTION_ENABLED,                         // 69
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_1,                                  // 70
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_2,                                  // 71
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_3,                                  // 72
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_4,                                  // 73
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_5,                                  // 74
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_6,                                  // 75
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_7,                                  // 76
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_8,                                  // 77
  DEFAULT_VALUE_WALK_ASSIST_LEVEL_9,                                  // 78
  DEFAULT_VALUE_CRUISE_FUNCTION_ENABLED,                              // 79
  DEFAULT_VALUE_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED,             // 80
  DEFAULT_VALUE_CRUISE_FUNCTION_TARGET_SPEED_KPH,                     // 81
  DEFAULT_VALUE_CRUISE_FUNCTION_TARGET_SPEED_MPH,                     // 82
  DEFAULT_VALUE_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED,                // 83
  DEFAULT_VALUE_WHEEL_SPEED_FIELD_STATE,                              // 84
  DEFAULT_VALUE_SHOW_DISTANCE_DATA_ODOMETER_FIELD,                    // 85
  DEFAULT_VALUE_SHOW_BATTERY_STATE_ODOMETER_FIELD,                    // 86
  DEFAULT_VALUE_SHOW_PEDAL_DATA_ODOMETER_FIELD,                       // 87
  DEFAULT_VALUE_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD,                 // 88
  DEFAULT_VALUE_SHOW_WHEEL_SPEED_ODOMETER_FIELD,                      // 89
  DEFAULT_VALUE_SHOW_ENERGY_DATA_ODOMETER_FIELD,                      // 90
  DEFAULT_VALUE_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD,                // 91
  DEFAULT_VALUE_SHOW_BATTERY_SOC_ODOMETER_FIELD,                      // 92
  DEFAULT_VALUE_MAIN_SCREEN_POWER_MENU_ENABLED,                       // 93
  DEFAULT_VALUE_TORQUE_ASSIST_FUNCTION_ENABLED,                       // 94
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_1,                                // 95
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_2,                                // 96
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_3,                                // 97
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_4,                                // 98
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_5,                                // 99
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_6,                                // 100
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_7,                                // 101
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_8,                                // 102
  DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_9,                                // 103
  DEFAULT_VALUE_CADENCE_ASSIST_FUNCTION_ENABLED,                      // 104
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_1,                               // 105
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_2,                               // 106
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_3,                               // 107
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_4,                               // 108
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_5,                               // 109
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_6,                               // 110
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_7,                               // 111
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_8,                               // 112
  DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_9,                               // 113
  DEFAULT_VALUE_CADENCE_SENSOR_MODE,                                  // 114
  DEFAULT_VALUE_EMTB_ASSIST_FUNCTION_ENABLED,                         // 115
  DEFAULT_VALUE_EMTB_ASSIST_SENSITIVITY,                              // 116
  DEFAULT_VALUE_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100,                // 117
  DEFAULT_VALUE_TEMPERATURE_FIELD_STATE,                              // 118
  DEFAULT_VALUE_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_0,           // 119
  DEFAULT_VALUE_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_1,           // 120
  DEFAULT_VALUE_LIGHTS_MODE,                                          // 121
  DEFAULT_VALUE_LIGHTS_STATE,                                         // 122
  DEFAULT_VALUE_ASSIST_WITHOUT_PEDAL_ROTATION_THRESHOLD,              // 123
  DEFAULT_VALUE_LIGHTS_CONFIGURATION,                                 // 124
  DEFAULT_VALUE_WALK_ASSIST_BUTTON_BOUNCE_TIME                        // 125
};


void EEPROM_init(void)
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
  uint8_t ui8_saved_key = FLASH_ReadByte(ADDRESS_KEY + EEPROM_BASE_ADDRESS);
  
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
        
        // wait until end of programming (write or erase operation) flag is set
        while (FLASH_GetFlagStatus(FLASH_FLAG_EOP) == RESET) {}
        
        // read value from EEPROM for validation
        volatile uint8_t ui8_saved_default_value = FLASH_ReadByte(ui32_default_address);
        
        // if write was not successful, rewrite
        if (ui8_saved_default_value != ui8_default_variable_value) { ui8_i = EEPROM_BYTES_STORED; }
      }
      
    break;
    
    
    /********************************************************************************************************************************************************/
    
    
    case READ_FROM_MEMORY:
    
      // read from EEPROM and set in array
      for (ui8_i = EEPROM_BYTES_STORED; ui8_i > 0; ui8_i--)
      {
        // get address
        uint32_t ui32_address = (uint32_t) ui8_i - 1 + EEPROM_BASE_ADDRESS;
        
        // read from EEPROM
        uint8_t ui8_EEPROM_value = FLASH_ReadByte(ui32_address);
        
        // save value in array
        ui8_array[ui8_i - 1] = ui8_EEPROM_value;
      }
      
      // assist level
      p_configuration_variables->ui8_assist_level = ui8_array[ADDRESS_ASSIST_LEVEL];
      p_configuration_variables->ui8_number_of_assist_levels = ui8_array[ADDRESS_NUMBER_OF_ASSIST_LEVELS];
      
      // power assist
      p_configuration_variables->ui8_power_assist_function_enabled = ui8_array[ADDRESS_POWER_ASSIST_FUNCTION_ENABLED];
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        p_configuration_variables->ui8_power_assist_level[ui8_i] = ui8_array[ADDRESS_POWER_ASSIST_LEVEL_1 + ui8_i];
      }
      
      // torque assist
      p_configuration_variables->ui8_torque_assist_function_enabled = ui8_array[ADDRESS_TORQUE_ASSIST_FUNCTION_ENABLED];
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        p_configuration_variables->ui8_torque_assist_level[ui8_i] = ui8_array[ADDRESS_TORQUE_ASSIST_LEVEL_1 + ui8_i];
      }
      
      // cadence assist
      p_configuration_variables->ui8_cadence_assist_function_enabled = ui8_array[ADDRESS_CADENCE_ASSIST_FUNCTION_ENABLED];
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        p_configuration_variables->ui8_cadence_assist_level[ui8_i] = ui8_array[ADDRESS_CADENCE_ASSIST_LEVEL_1 + ui8_i];
      }
      
      // eMTB assist
      p_configuration_variables->ui8_eMTB_assist_function_enabled = ui8_array[ADDRESS_EMTB_ASSIST_FUNCTION_ENABLED];
      p_configuration_variables->ui8_eMTB_assist_sensitivity = ui8_array[ADDRESS_EMTB_ASSIST_SENSITIVITY];
      
      // walk assist
      p_configuration_variables->ui8_walk_assist_function_enabled = ui8_array[ADDRESS_WALK_ASSIST_FUNCTION_ENABLED];
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        p_configuration_variables->ui8_walk_assist_level[ui8_i] = ui8_array[ADDRESS_WALK_ASSIST_LEVEL_1 + ui8_i];
      }
      
      // cruise
      p_configuration_variables->ui8_cruise_function_enabled = ui8_array[ADDRESS_CRUISE_FUNCTION_ENABLED];
      p_configuration_variables->ui8_cruise_function_set_target_speed_enabled = ui8_array[ADDRESS_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED];
      p_configuration_variables->ui8_cruise_function_target_speed_kph = ui8_array[ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_KPH];
      p_configuration_variables->ui8_cruise_function_target_speed_mph = ui8_array[ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_MPH];
      p_configuration_variables->ui8_show_cruise_function_set_target_speed = ui8_array[ADDRESS_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED]; 

      // wheel perimeter
      ui16_temp = ui8_array[ADDRESS_WHEEL_PERIMETER_0];
      ui8_temp = ui8_array[ADDRESS_WHEEL_PERIMETER_1];
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_wheel_perimeter = ui16_temp;

      // max wheel speed
      p_configuration_variables->ui8_wheel_max_speed = ui8_array[ADDRESS_MAX_WHEEL_SPEED];
      p_configuration_variables->ui8_wheel_max_speed_imperial = ui8_array[ADDRESS_MAX_WHEEL_SPEED_IMPERIAL];
      
      // units
      p_configuration_variables->ui8_units_type = ui8_array[ADDRESS_UNITS_TYPE];

      // odometer sub field states
      p_configuration_variables->ui8_odometer_field_state = ui8_array[ADDRESS_ODOMETER_FIELD_STATE];
      p_configuration_variables->ui8_odometer_sub_field_state_0 = ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_0];
      p_configuration_variables->ui8_odometer_sub_field_state_1 = ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_1];
      p_configuration_variables->ui8_odometer_sub_field_state_2 = ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_2];
      p_configuration_variables->ui8_odometer_sub_field_state_3 = ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_3];
      p_configuration_variables->ui8_odometer_sub_field_state_4 = ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_4];
      p_configuration_variables->ui8_odometer_sub_field_state_5 = ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_5];
      p_configuration_variables->ui8_odometer_sub_field_state_6 = ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_6];
      
      // time measurement
      p_configuration_variables->ui8_time_measurement_field_state = ui8_array[ADDRESS_TIME_MEASUREMENT_FIELD_STATE];
      p_configuration_variables->ui8_total_second_TTM = ui8_array[ADDRESS_TOTAL_SECOND_TTM];
      p_configuration_variables->ui8_total_minute_TTM = ui8_array[ADDRESS_TOTAL_MINUTE_TTM];
      ui16_temp = ui8_array[ADDRESS_TOTAL_HOUR_TTM_0];
      ui8_temp = ui8_array[ADDRESS_TOTAL_HOUR_TTM_1];
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_total_hour_TTM = ui16_temp;

      // motor type
      p_configuration_variables->ui8_motor_type = ui8_array[ADDRESS_MOTOR_TYPE];
      
      // optional ADC function
      p_configuration_variables->ui8_optional_ADC_function = ui8_array[ADDRESS_OPTIONAL_ADC_FUNCTION];
      
      // temperature field state
      p_configuration_variables->ui8_temperature_field_state = ui8_array[ADDRESS_TEMPERATURE_FIELD_STATE];
      
      // motor temperature protection
      p_configuration_variables->ui8_motor_temperature_min_value_to_limit = ui8_array[ADDRESS_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT];
      p_configuration_variables->ui8_motor_temperature_max_value_to_limit = ui8_array[ADDRESS_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT];
      
      // display
      p_configuration_variables->ui8_lcd_power_off_time_minutes = ui8_array[ADDRESS_LCD_POWER_OFF_TIME];
      p_configuration_variables->ui8_lcd_backlight_on_brightness = ui8_array[ADDRESS_LCD_BACKLIGHT_ON_BRIGHTNESS];
      p_configuration_variables->ui8_lcd_backlight_off_brightness = ui8_array[ADDRESS_LCD_BACKLIGHT_OFF_BRIGHTNESS];
      
      // battery
      p_configuration_variables->ui8_battery_max_current = ui8_array[ADDRESS_BATTERY_MAX_CURRENT];
      p_configuration_variables->ui8_target_max_battery_power_div25 = ui8_array[ADDRESS_TARGET_MAX_BATTERY_POWER];
      p_configuration_variables->ui8_battery_cells_number = ui8_array[ADDRESS_BATTERY_CELLS_NUMBER];
      
      ui16_temp = ui8_array[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0];
      ui8_temp = ui8_array[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1];
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 = ui16_temp;
      
      ui16_temp = ui8_array[ADDRESS_BATTERY_PACK_RESISTANCE_0];
      ui8_temp = ui8_array[ADDRESS_BATTERY_PACK_RESISTANCE_1];
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_battery_pack_resistance_x1000 = ui16_temp;
      
      p_configuration_variables->ui8_battery_SOC_function_enabled = ui8_array[ADDRESS_BATTERY_SOC_FUNCTION_ENABLED];
      
      ui32_temp = ui8_array[ADDRESS_HW_X10_OFFSET_0];
      ui8_temp = ui8_array[ADDRESS_HW_X10_OFFSET_1];
      ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
      ui8_temp = ui8_array[ADDRESS_HW_X10_OFFSET_2];
      ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
      ui8_temp = ui8_array[ADDRESS_HW_X10_OFFSET_3];
      ui32_temp += (((uint32_t) ui8_temp << 24) & 0xff000000);
      p_configuration_variables->ui32_wh_x10_offset = ui32_temp;

      ui32_temp = ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_0];
      ui8_temp = ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_1];
      ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
      ui8_temp = ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_2];
      ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
      ui8_temp = ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_3];
      ui32_temp += (((uint32_t) ui8_temp << 24) & 0xff000000);
      p_configuration_variables->ui32_wh_x10_100_percent = ui32_temp;
      
      ui16_temp = ui8_array[ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0];
      ui8_temp = ui8_array[ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1];
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_battery_voltage_reset_wh_counter_x10 = ui16_temp;

      // street mode
      p_configuration_variables->ui8_street_mode_function_enabled = ui8_array[ADDRESS_STREET_MODE_FUNCTION_ENABLED];
      p_configuration_variables->ui8_street_mode_speed_limit = ui8_array[ADDRESS_STREET_MODE_SPEED_LIMIT];
      p_configuration_variables->ui8_street_mode_power_limit_enabled = ui8_array[ADDRESS_STREET_MODE_POWER_LIMIT_ENABLED];
      p_configuration_variables->ui8_street_mode_power_limit_div25 = ui8_array[ADDRESS_STREET_MODE_POWER_LIMIT_DIV25];
      p_configuration_variables->ui8_street_mode_throttle_enabled = ui8_array[ADDRESS_STREET_MODE_THROTTLE_ENABLED];
      p_configuration_variables->ui8_street_mode_cruise_enabled = ui8_array[ADDRESS_STREET_MODE_CRUISE_ENABLED];
      
      
      // odometer variable
      ui32_temp = ui8_array[ADDRESS_ODOMETER_X10_0];
      ui8_temp = ui8_array[ADDRESS_ODOMETER_X10_1];
      ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
      ui8_temp = ui8_array[ADDRESS_ODOMETER_X10_2];
      ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
      p_configuration_variables->ui32_odometer_x10 = ui32_temp;
      
      
      // trip distance variable
      ui32_temp = ui8_array[ADDRESS_TRIP_X10_0];
      ui8_temp = ui8_array[ADDRESS_TRIP_X10_1];
      ui32_temp += (((uint32_t) ui8_temp << 8) & 0xff00);
      ui8_temp = ui8_array[ADDRESS_TRIP_X10_2];
      ui32_temp += (((uint32_t) ui8_temp << 16) & 0xff0000);
      p_configuration_variables->ui32_trip_x10 = ui32_temp;
      
      
      // motor acceleration
      p_configuration_variables->ui8_motor_acceleration = ui8_array[ADDRESS_MOTOR_ACCELERATION];
      
      
      // wheel speed measurement
      p_configuration_variables->ui8_wheel_speed_field_state = ui8_array[ADDRESS_WHEEL_SPEED_FIELD_STATE];
      
      
      // show variables in odometer field
      p_configuration_variables->ui8_show_distance_data_odometer_field = ui8_array[ADDRESS_SHOW_DISTANCE_DATA_ODOMETER_FIELD];
      p_configuration_variables->ui8_show_battery_state_odometer_field = ui8_array[ADDRESS_SHOW_BATTERY_STATE_ODOMETER_FIELD];
      p_configuration_variables->ui8_show_pedal_data_odometer_field = ui8_array[ADDRESS_SHOW_PEDAL_DATA_ODOMETER_FIELD];
      p_configuration_variables->ui8_show_time_measurement_odometer_field = ui8_array[ADDRESS_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD];
      p_configuration_variables->ui8_show_wheel_speed_odometer_field = ui8_array[ADDRESS_SHOW_WHEEL_SPEED_ODOMETER_FIELD];
      p_configuration_variables->ui8_show_energy_data_odometer_field = ui8_array[ADDRESS_SHOW_ENERGY_DATA_ODOMETER_FIELD];
      p_configuration_variables->ui8_show_motor_temperature_odometer_field = ui8_array[ADDRESS_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD];
      p_configuration_variables->ui8_show_battery_SOC_odometer_field = ui8_array[ADDRESS_SHOW_BATTERY_SOC_ODOMETER_FIELD];
      
      
      // main screen power menu enable 
      p_configuration_variables->ui8_main_screen_power_menu_enabled = ui8_array[ADDRESS_MAIN_SCREEN_POWER_MENU_ENABLED];
      
      
      // pedal torque conversion
      p_configuration_variables->ui8_pedal_torque_per_10_bit_ADC_step_x100 = ui8_array[ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100];
      
      
      // cadence sensor mode
      p_configuration_variables->ui8_cadence_sensor_mode = ui8_array[ADDRESS_CADENCE_SENSOR_MODE];
      ui16_temp = ui8_array[ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_0];
      ui8_temp = ui8_array[ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_1];
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_cadence_sensor_pulse_high_percentage_x10 = ui16_temp;
      
      
      // assist without pedal rotation threshold
      p_configuration_variables->ui8_assist_without_pedal_rotation_threshold = ui8_array[ADDRESS_ASSIST_WITHOUT_PEDAL_ROTATION_THRESHOLD];
      
      
      // lights
      p_configuration_variables->ui8_light_mode = ui8_array[ADDRESS_LIGHTS_MODE];
      p_configuration_variables->ui8_lights_state = ui8_array[ADDRESS_LIGHTS_STATE];
      p_configuration_variables->ui8_lights_configuration = ui8_array[ADDRESS_LIGHTS_CONFIGURATION];
      
      // walk assist button bounce time
      p_configuration_variables->ui8_walk_assist_button_bounce_time = ui8_array[ADDRESS_WALK_ASSIST_BUTTON_BOUNCE_TIME];
      
    break;
    
    
    /********************************************************************************************************************************************************/
    
    
    case WRITE_TO_MEMORY:
    
      // key
      ui8_array[ADDRESS_KEY] = DEFAULT_VALUE_KEY;
      
      // assist level
      ui8_array[ADDRESS_ASSIST_LEVEL] = p_configuration_variables->ui8_assist_level;
      
      // wheel perimeter
      ui8_array[ADDRESS_WHEEL_PERIMETER_0] = p_configuration_variables->ui16_wheel_perimeter & 255;
      ui8_array[ADDRESS_WHEEL_PERIMETER_1] = (p_configuration_variables->ui16_wheel_perimeter >> 8) & 255;
      
      // max wheel speed
      ui8_array[ADDRESS_MAX_WHEEL_SPEED] = p_configuration_variables->ui8_wheel_max_speed;
      ui8_array[ADDRESS_MAX_WHEEL_SPEED_IMPERIAL] = p_configuration_variables->ui8_wheel_max_speed_imperial;
      
      // units
      ui8_array[ADDRESS_UNITS_TYPE] = p_configuration_variables->ui8_units_type;

      // motor type
      ui8_array[ADDRESS_MOTOR_TYPE] = p_configuration_variables->ui8_motor_type;
      
      // optional ADC function
      ui8_array[ADDRESS_OPTIONAL_ADC_FUNCTION] = p_configuration_variables->ui8_optional_ADC_function;
      
      // temperature field state
      ui8_array[ADDRESS_TEMPERATURE_FIELD_STATE] = p_configuration_variables->ui8_temperature_field_state;
      
      // number of assist levels
      ui8_array[ADDRESS_NUMBER_OF_ASSIST_LEVELS] = p_configuration_variables->ui8_number_of_assist_levels;
      
      // power assist
      ui8_array[ADDRESS_POWER_ASSIST_FUNCTION_ENABLED] = p_configuration_variables->ui8_power_assist_function_enabled;
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        ui8_array[ADDRESS_POWER_ASSIST_LEVEL_1 + ui8_i] = p_configuration_variables->ui8_power_assist_level[ui8_i];
      }
      
      // torque assist
      ui8_array[ADDRESS_TORQUE_ASSIST_FUNCTION_ENABLED] = p_configuration_variables->ui8_torque_assist_function_enabled;
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        ui8_array[ADDRESS_TORQUE_ASSIST_LEVEL_1 + ui8_i] = p_configuration_variables->ui8_torque_assist_level[ui8_i];
      }
      
      // cadence assist
      ui8_array[ADDRESS_CADENCE_ASSIST_FUNCTION_ENABLED] = p_configuration_variables->ui8_cadence_assist_function_enabled;
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        ui8_array[ADDRESS_CADENCE_ASSIST_LEVEL_1 + ui8_i] = p_configuration_variables->ui8_cadence_assist_level[ui8_i];
      }
      
      // eMTB assist function variables
      ui8_array[ADDRESS_EMTB_ASSIST_FUNCTION_ENABLED] = p_configuration_variables->ui8_eMTB_assist_function_enabled;
      ui8_array[ADDRESS_EMTB_ASSIST_SENSITIVITY] = p_configuration_variables->ui8_eMTB_assist_sensitivity;
      
      // walk assist
      ui8_array[ADDRESS_WALK_ASSIST_FUNCTION_ENABLED] = p_configuration_variables->ui8_walk_assist_function_enabled;
      for (ui8_i = 0; ui8_i < 9; ui8_i++)
      {
        ui8_array[ADDRESS_WALK_ASSIST_LEVEL_1 + ui8_i] = p_configuration_variables->ui8_walk_assist_level[ui8_i];
      }
      
      // cruise
      ui8_array[ADDRESS_CRUISE_FUNCTION_ENABLED] = p_configuration_variables->ui8_cruise_function_enabled;
      ui8_array[ADDRESS_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED] = p_configuration_variables->ui8_cruise_function_set_target_speed_enabled;
      ui8_array[ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_KPH] = p_configuration_variables->ui8_cruise_function_target_speed_kph;
      ui8_array[ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_MPH] = p_configuration_variables->ui8_cruise_function_target_speed_mph;
      ui8_array[ADDRESS_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED] = p_configuration_variables->ui8_show_cruise_function_set_target_speed; 
      
      // motor temperature protection
      ui8_array[ADDRESS_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT] = p_configuration_variables->ui8_motor_temperature_min_value_to_limit;
      ui8_array[ADDRESS_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT] = p_configuration_variables->ui8_motor_temperature_max_value_to_limit;
      
      // display
      ui8_array[ADDRESS_LCD_POWER_OFF_TIME] = p_configuration_variables->ui8_lcd_power_off_time_minutes;
      ui8_array[ADDRESS_LCD_BACKLIGHT_ON_BRIGHTNESS] = p_configuration_variables->ui8_lcd_backlight_on_brightness;
      ui8_array[ADDRESS_LCD_BACKLIGHT_OFF_BRIGHTNESS] = p_configuration_variables->ui8_lcd_backlight_off_brightness;
      
      // battery
      ui8_array[ADDRESS_BATTERY_MAX_CURRENT] = p_configuration_variables->ui8_battery_max_current;
      ui8_array[ADDRESS_TARGET_MAX_BATTERY_POWER] = p_configuration_variables->ui8_target_max_battery_power_div25;
      ui8_array[ADDRESS_BATTERY_CELLS_NUMBER] = p_configuration_variables->ui8_battery_cells_number;
      
      ui8_array[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0] = p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 & 255;
      ui8_array[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1] = (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 >> 8) & 255;
      
      ui8_array[ADDRESS_BATTERY_PACK_RESISTANCE_0] = p_configuration_variables->ui16_battery_pack_resistance_x1000 & 255;
      ui8_array[ADDRESS_BATTERY_PACK_RESISTANCE_1] = (p_configuration_variables->ui16_battery_pack_resistance_x1000 >> 8) & 255;
      
      ui8_array[ADDRESS_BATTERY_SOC_FUNCTION_ENABLED] = p_configuration_variables->ui8_battery_SOC_function_enabled;
      
      ui8_array[ADDRESS_HW_X10_OFFSET_0] = p_configuration_variables->ui32_wh_x10_offset & 255;
      ui8_array[ADDRESS_HW_X10_OFFSET_1] = (p_configuration_variables->ui32_wh_x10_offset >> 8) & 255;
      ui8_array[ADDRESS_HW_X10_OFFSET_2] = (p_configuration_variables->ui32_wh_x10_offset >> 16) & 255;
      ui8_array[ADDRESS_HW_X10_OFFSET_3] = (p_configuration_variables->ui32_wh_x10_offset >> 24) & 255;
      ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_0] = p_configuration_variables->ui32_wh_x10_100_percent & 255;
      ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_1] = (p_configuration_variables->ui32_wh_x10_100_percent >> 8) & 255;
      ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_2] = (p_configuration_variables->ui32_wh_x10_100_percent >> 16) & 255;
      ui8_array[ADDRESS_HW_X10_100_PERCENT_OFFSET_3] = (p_configuration_variables->ui32_wh_x10_100_percent >> 24) & 255;
      
      ui8_array[ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0] = p_configuration_variables->ui16_battery_voltage_reset_wh_counter_x10 & 255;
      ui8_array[ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1] = (p_configuration_variables->ui16_battery_voltage_reset_wh_counter_x10 >> 8) & 255;

      // street mode
      ui8_array[ADDRESS_STREET_MODE_FUNCTION_ENABLED] = p_configuration_variables->ui8_street_mode_function_enabled;
      ui8_array[ADDRESS_STREET_MODE_SPEED_LIMIT] = p_configuration_variables->ui8_street_mode_speed_limit;
      ui8_array[ADDRESS_STREET_MODE_POWER_LIMIT_ENABLED] = p_configuration_variables->ui8_street_mode_power_limit_enabled;
      ui8_array[ADDRESS_STREET_MODE_POWER_LIMIT_DIV25] = p_configuration_variables->ui8_street_mode_power_limit_div25;
      ui8_array[ADDRESS_STREET_MODE_THROTTLE_ENABLED] = p_configuration_variables->ui8_street_mode_throttle_enabled;
      ui8_array[ADDRESS_STREET_MODE_CRUISE_ENABLED] = p_configuration_variables->ui8_street_mode_cruise_enabled;
      
      // odometer
      ui8_array[ADDRESS_ODOMETER_X10_0] = p_configuration_variables->ui32_odometer_x10 & 255;
      ui8_array[ADDRESS_ODOMETER_X10_1] = (p_configuration_variables->ui32_odometer_x10 >> 8) & 255;
      ui8_array[ADDRESS_ODOMETER_X10_2] = (p_configuration_variables->ui32_odometer_x10 >> 16) & 255;
      
      // trip distance
      ui8_array[ADDRESS_TRIP_X10_0] = p_configuration_variables->ui32_trip_x10 & 255;
      ui8_array[ADDRESS_TRIP_X10_1] = (p_configuration_variables->ui32_trip_x10 >> 8) & 255;
      ui8_array[ADDRESS_TRIP_X10_2] = (p_configuration_variables->ui32_trip_x10 >> 16) & 255;

      // odometer sub field states
      ui8_array[ADDRESS_ODOMETER_FIELD_STATE] = p_configuration_variables->ui8_odometer_field_state;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_0] = p_configuration_variables->ui8_odometer_sub_field_state_0;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_1] = p_configuration_variables->ui8_odometer_sub_field_state_1;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_2] = p_configuration_variables->ui8_odometer_sub_field_state_2;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_3] = p_configuration_variables->ui8_odometer_sub_field_state_3;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_4] = p_configuration_variables->ui8_odometer_sub_field_state_4;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_5] = p_configuration_variables->ui8_odometer_sub_field_state_5;
      ui8_array[ADDRESS_ODOMETER_SUB_FIELD_STATE_6] = p_configuration_variables->ui8_odometer_sub_field_state_6;
      
      // time measurement
      ui8_array[ADDRESS_TIME_MEASUREMENT_FIELD_STATE] = p_configuration_variables->ui8_time_measurement_field_state;
      ui8_array[ADDRESS_TOTAL_SECOND_TTM] = p_configuration_variables->ui8_total_second_TTM;
      ui8_array[ADDRESS_TOTAL_MINUTE_TTM] = p_configuration_variables->ui8_total_minute_TTM;
      ui8_array[ADDRESS_TOTAL_HOUR_TTM_0] = p_configuration_variables->ui16_total_hour_TTM & 255;
      ui8_array[ADDRESS_TOTAL_HOUR_TTM_1] = (p_configuration_variables->ui16_total_hour_TTM >> 8) & 255;
      
      // motor acceleration
      ui8_array[ADDRESS_MOTOR_ACCELERATION] = p_configuration_variables->ui8_motor_acceleration; 
      
      // wheel speed field state
      ui8_array[ADDRESS_WHEEL_SPEED_FIELD_STATE] = p_configuration_variables->ui8_wheel_speed_field_state;
      
      // show odometer variables
      ui8_array[ADDRESS_SHOW_DISTANCE_DATA_ODOMETER_FIELD] = p_configuration_variables->ui8_show_distance_data_odometer_field;
      ui8_array[ADDRESS_SHOW_BATTERY_STATE_ODOMETER_FIELD] = p_configuration_variables->ui8_show_battery_state_odometer_field;
      ui8_array[ADDRESS_SHOW_PEDAL_DATA_ODOMETER_FIELD] = p_configuration_variables->ui8_show_pedal_data_odometer_field;
      ui8_array[ADDRESS_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD] = p_configuration_variables->ui8_show_time_measurement_odometer_field;
      ui8_array[ADDRESS_SHOW_WHEEL_SPEED_ODOMETER_FIELD] = p_configuration_variables->ui8_show_wheel_speed_odometer_field;
      ui8_array[ADDRESS_SHOW_ENERGY_DATA_ODOMETER_FIELD] = p_configuration_variables->ui8_show_energy_data_odometer_field;
      ui8_array[ADDRESS_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD] = p_configuration_variables->ui8_show_motor_temperature_odometer_field;
      ui8_array[ADDRESS_SHOW_BATTERY_SOC_ODOMETER_FIELD] = p_configuration_variables->ui8_show_battery_SOC_odometer_field;

      // main screen power menu enable
      ui8_array[ADDRESS_MAIN_SCREEN_POWER_MENU_ENABLED] = p_configuration_variables->ui8_main_screen_power_menu_enabled;

      // pedal torque conversion
      ui8_array[ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100] = p_configuration_variables->ui8_pedal_torque_per_10_bit_ADC_step_x100;
      
      // cadence sensor mode
      ui8_array[ADDRESS_CADENCE_SENSOR_MODE] = p_configuration_variables->ui8_cadence_sensor_mode;
      ui8_array[ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_0] = p_configuration_variables->ui16_cadence_sensor_pulse_high_percentage_x10 & 255;
      ui8_array[ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_1] = (p_configuration_variables->ui16_cadence_sensor_pulse_high_percentage_x10 >> 8) & 255;
      
      // assist without pedal rotation threshold
      ui8_array[ADDRESS_ASSIST_WITHOUT_PEDAL_ROTATION_THRESHOLD] = p_configuration_variables->ui8_assist_without_pedal_rotation_threshold;
      
      // lights
      ui8_array[ADDRESS_LIGHTS_MODE] = p_configuration_variables->ui8_light_mode;
      ui8_array[ADDRESS_LIGHTS_STATE] = p_configuration_variables->ui8_lights_state;
      ui8_array[ADDRESS_LIGHTS_CONFIGURATION] = p_configuration_variables->ui8_lights_configuration;

      // walk assist button bounce time
      ui8_array[ADDRESS_WALK_ASSIST_BUTTON_BOUNCE_TIME] = p_configuration_variables->ui8_walk_assist_button_bounce_time;
      
      // write array of variables to EEPROM
      for (ui8_i = EEPROM_BYTES_STORED; ui8_i > 0; ui8_i--)
      {
        // get address
        uint32_t ui32_address = (uint32_t) ui8_i - 1 + EEPROM_BASE_ADDRESS;
        
        // get value
        uint8_t ui8_variable_value = ui8_array[ui8_i - 1];
        
        // write variable value to EEPROM
        FLASH_ProgramByte(ui32_address, ui8_variable_value);
        
        // wait until end of programming (write or erase operation) flag is set
        while (FLASH_GetFlagStatus(FLASH_FLAG_EOP) == RESET) {}
        
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