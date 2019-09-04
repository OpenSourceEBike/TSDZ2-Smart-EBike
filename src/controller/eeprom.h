/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho and Leon, 2019.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "main.h"


#define EEPROM_BASE_ADDRESS                                 0x4000

#define ADDRESS_KEY                                         0 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_POWER_X10                             1 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0           2 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1           3 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_PERIMETER_0                           4 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_PERIMETER_1                           5 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_SPEED_MAX                             6 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_TYPE                                  7 + EEPROM_BASE_ADDRESS
#define ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100       8 + EEPROM_BASE_ADDRESS
#define EEPROM_BYTES_STORED                                 9


#define DEFAULT_VALUE_KEY     204
#define SET_TO_DEFAULT        0
#define READ_FROM_MEMORY      1
#define WRITE_TO_MEMORY       2


void EEPROM_init(void);
void EEPROM_controller(uint8_t ui8_operation);

#endif /* _EEPROM_H_ */
