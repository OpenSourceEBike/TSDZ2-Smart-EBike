/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_flash.h"
#include "eeprom.h"
#include "ebike_app.h"

static uint8_t array_default_values [EEPROM_BYTES_STORED] = 
{
  KEY,
  DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_X10,                // 1 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_CONFIG_0,                               // 2 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_BATTERY_MAX_CURRENT,                    // 3 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_TARGET_BATTERY_MAX_POWER_X10,           // 4 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0,      // 5 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1,      // 6 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_WHEEL_PERIMETER_0,                      // 7 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_WHEEL_PERIMETER_1,                      // 8 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_WHEEL_SPEED_MAX,                        // 9 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_MOTOR_TYPE,                             // 10 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100   // 11 + EEPROM_BASE_ADDRESS
};



static void eeprom_read_values_to_variables (void);
static void eeprom_write_array (uint8_t *array_values);
static void variables_to_array (uint8_t *ui8_array);



void eeprom_init (void)
{
  uint8_t ui8_data;

  // start by reading address 0 and see if value is different from our key,
  // if so mean that eeprom memory is clean and we need to populate: should happen after erasing the microcontroller
  ui8_data = FLASH_ReadByte (ADDRESS_KEY);
  
  // verify our key exists
  if (ui8_data != KEY) 
  {
    eeprom_write_array (array_default_values);
  }
}


void eeprom_init_variables (void)
{
  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables ();

  eeprom_read_values_to_variables ();

  // now verify if any EEPROM saved value is out of valid range and if so,
  // write correct ones and read again
  if ((p_configuration_variables->ui8_battery_max_current > 100) ||
      (p_configuration_variables->ui8_motor_power_x10 > 195) ||
      (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 > 630) ||
      (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 < 160) ||
      (p_configuration_variables->ui16_wheel_perimeter > 3000) ||
      (p_configuration_variables->ui16_wheel_perimeter < 750))
  {
    eeprom_write_array (array_default_values);
    eeprom_read_values_to_variables ();
  }
}


static void eeprom_read_values_to_variables (void)
{
  static uint8_t ui8_temp;
  static uint16_t ui16_temp;

  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables ();

  p_configuration_variables->ui8_assist_level_factor_x10 = FLASH_ReadByte (ADDRESS_ASSIST_LEVEL_FACTOR_X10);

  ui8_temp = FLASH_ReadByte (ADDRESS_CONFIG_0);
  p_configuration_variables->ui8_lights = ui8_temp & 1 ? 1 : 0;

  p_configuration_variables->ui8_battery_max_current = FLASH_ReadByte (ADDRESS_BATTERY_MAX_CURRENT);
  p_configuration_variables->ui8_motor_power_x10 = FLASH_ReadByte (ADDRESS_MOTOR_POWER_X10);
  
  // battery low voltage cut off 
  ui16_temp = FLASH_ReadByte (ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1);
  ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
  p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 = ui16_temp;
  
  // wheel perimeter
  ui16_temp = FLASH_ReadByte (ADDRESS_WHEEL_PERIMETER_0);
  ui8_temp = FLASH_ReadByte (ADDRESS_WHEEL_PERIMETER_1);
  ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
  p_configuration_variables->ui16_wheel_perimeter = ui16_temp;

  p_configuration_variables->ui8_wheel_speed_max = FLASH_ReadByte(ADDRESS_WHEEL_SPEED_MAX);

  p_configuration_variables->ui8_motor_type = FLASH_ReadByte(ADDRESS_MOTOR_TYPE);
  
  p_configuration_variables->ui8_pedal_torque_per_10_bit_ADC_step_x100 = FLASH_ReadByte(ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100);
}


void eeprom_write_variables (void)
{
  uint8_t array_variables [EEPROM_BYTES_STORED];
  variables_to_array (array_variables);
  eeprom_write_array (array_variables);
}


static void variables_to_array (uint8_t *ui8_array)
{
  struct_configuration_variables *p_configuration_variables;
  p_configuration_variables = get_configuration_variables ();

  ui8_array [0] = KEY;
  ui8_array [ADDRESS_ASSIST_LEVEL_FACTOR_X10 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_assist_level_factor_x10;
  ui8_array [ADDRESS_CONFIG_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_lights;
  ui8_array [ADDRESS_BATTERY_MAX_CURRENT - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_battery_max_current;
  ui8_array [ADDRESS_MOTOR_POWER_X10 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_motor_power_x10;
  ui8_array [ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 & 255;
  ui8_array [ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 >> 8) & 255;
  ui8_array [ADDRESS_WHEEL_PERIMETER_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_wheel_perimeter & 255;
  ui8_array [ADDRESS_WHEEL_PERIMETER_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_wheel_perimeter >> 8) & 255;
  ui8_array [ADDRESS_WHEEL_SPEED_MAX - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_wheel_speed_max;
  ui8_array [ADDRESS_MOTOR_TYPE - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_motor_type;
  ui8_array [ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_pedal_torque_per_10_bit_ADC_step_x100;
}


static void eeprom_write_array (uint8_t *array)
{
  uint8_t ui8_i;

  FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);
  
  // unlock data memory 
  FLASH_Unlock (FLASH_MEMTYPE_DATA); 
  
  // wait until data EEPROM area unlocked flag is set
  while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET) { } 

  for (ui8_i = 0; ui8_i < EEPROM_BYTES_STORED; ui8_i++)
  {
    FLASH_ProgramByte (EEPROM_BASE_ADDRESS + ui8_i, *array++);
  }
  
  // lock data memory 
  FLASH_Lock (FLASH_MEMTYPE_DATA);
}


void eeprom_write_if_values_changed (void)
{
// 2018.08.29:
// NOTE: the next code gives a problem with motor, when we exchange assist level on LCD3 and
// all the variables all written to EEPROM. As per datasheet, seems each byte takes ~6ms to be written
// I am not sure the issue is the amount of time...

//  uint8_t ui8_index;
//
//  uint8_t array_variables [EEPROM_BYTES_STORED];
//  variables_to_array (array_variables);
//
//  ui8_index = 1; // do not verify the first byte: ADDRESS_KEY
//  while (ui8_index < EEPROM_BYTES_STORED)
//  {
//    if (array_variables [ui8_index] != FLASH_ReadByte (EEPROM_BASE_ADDRESS + ui8_index))
//    {
//      eeprom_write_array (array_variables);
//      break; // exit the while loop
//    }
//
//    ui8_index++;
//  }
}
