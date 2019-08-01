/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "main.h"



#define KEY                                                 0xcd

#define EEPROM_BASE_ADDRESS                                 0x4000

#define ADDRESS_KEY                                         EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_MAX_CURRENT                         1 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_POWER_X10                             2 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0           3 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1           4 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_PERIMETER_0                           5 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_PERIMETER_1                           6 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_SPEED_MAX                             7 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_TYPE                                  8 + EEPROM_BASE_ADDRESS
#define ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100       9 + EEPROM_BASE_ADDRESS
#define EEPROM_BYTES_STORED                                 10


void eeprom_init (void);
void eeprom_init_variables (void);
void eeprom_write_if_values_changed (void);

#endif /* _EEPROM_H_ */
