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

#define KEY                                         0xcc

#define EEPROM_BASE_ADDRESS                         0x4000
#define ADDRESS_KEY                                 EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL_FACTOR_X10             1 + EEPROM_BASE_ADDRESS
#define ADDRESS_CONFIG_0                            2 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_MAX_CURRENT                 3 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_POWER_X10                     4 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0   5 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1   6 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_PERIMETER_0                   7 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_PERIMETER_1                   8 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_MAX_SPEED                     9 + EEPROM_BASE_ADDRESS
#define ADDRESS_PAS_MAX_CADENCE                     10 + EEPROM_BASE_ADDRESS
#define ADDRESS_CONFIG_1                            11 + EEPROM_BASE_ADDRESS
#define ADDRESS_OFFROAD_CONFIG                      12 + EEPROM_BASE_ADDRESS
#define ADDRESS_OFFROAD_SPEED_LIMIT                 13 + EEPROM_BASE_ADDRESS
#define ADDRESS_OFFROAD_POWER_LIMIT_DIV25           14 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_ERPS                    15 + EEPROM_BASE_ADDRESS
#define EEPROM_BYTES_STORED                         16

void eeprom_init (void);
void eeprom_init_variables (void);
void eeprom_write_if_values_changed (void);
void eeprom_write_value_if_changed (uint32_t address, uint8_t value);

#endif /* _EEPROM_H_ */
