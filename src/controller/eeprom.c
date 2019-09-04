/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho and Leon, 2019.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include "stm8s.h"
#include "stm8s_flash.h"
#include "eeprom.h"
#include "ebike_app.h"


static const uint8_t ui8_default_array[EEPROM_BYTES_STORED] = 
{
  DEFAULT_VALUE_KEY,                                          // 0 + EEPROM_BASE_ADDRESS (Array index)
  DEFAULT_VALUE_TARGET_BATTERY_MAX_POWER_X10,                 // 1 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0,            // 2 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1,            // 3 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_WHEEL_PERIMETER_0,                            // 4 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_WHEEL_PERIMETER_1,                            // 5 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_WHEEL_SPEED_MAX,                              // 6 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_MOTOR_TYPE,                                   // 7 + EEPROM_BASE_ADDRESS
  DEFAULT_VALUE_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100         // 8 + EEPROM_BASE_ADDRESS
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
      
      p_configuration_variables->ui8_motor_power_x10 = FLASH_ReadByte(ADDRESS_MOTOR_POWER_X10);
      
      ui16_temp = FLASH_ReadByte(ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1);
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 = ui16_temp;
      
      ui16_temp = FLASH_ReadByte(ADDRESS_WHEEL_PERIMETER_0);
      ui8_temp = FLASH_ReadByte(ADDRESS_WHEEL_PERIMETER_1);
      ui16_temp += (((uint16_t) ui8_temp << 8) & 0xff00);
      p_configuration_variables->ui16_wheel_perimeter = ui16_temp;

      p_configuration_variables->ui8_wheel_speed_max = FLASH_ReadByte(ADDRESS_WHEEL_SPEED_MAX);

      p_configuration_variables->ui8_motor_type = FLASH_ReadByte(ADDRESS_MOTOR_TYPE);
      
      p_configuration_variables->ui8_pedal_torque_per_10_bit_ADC_step_x100 = FLASH_ReadByte(ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100);
      
    break;
    
    
    /********************************************************************************************************************************************************/
    
    
    case WRITE_TO_MEMORY:
    
      ui8_array[0] = DEFAULT_VALUE_KEY;
      
      ui8_array[ADDRESS_MOTOR_POWER_X10 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_motor_power_x10;
      
      ui8_array[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 & 255;
      ui8_array[ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_battery_low_voltage_cut_off_x10 >> 8) & 255;
      
      ui8_array[ADDRESS_WHEEL_PERIMETER_0 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui16_wheel_perimeter & 255;
      ui8_array[ADDRESS_WHEEL_PERIMETER_1 - EEPROM_BASE_ADDRESS] = (p_configuration_variables->ui16_wheel_perimeter >> 8) & 255;
      
      ui8_array[ADDRESS_WHEEL_SPEED_MAX - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_wheel_speed_max;
      
      ui8_array[ADDRESS_MOTOR_TYPE - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_motor_type;
      
      ui8_array[ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100 - EEPROM_BASE_ADDRESS] = p_configuration_variables->ui8_pedal_torque_per_10_bit_ADC_step_x100;
      
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