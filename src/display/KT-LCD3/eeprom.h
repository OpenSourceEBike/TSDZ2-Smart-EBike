/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "lcd.h"

#define KEY                                                                 0xe3

#define EEPROM_BASE_ADDRESS                                                 0x4000
#define ADDRESS_KEY                                                         0 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL                                                1 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_PERIMETER_0                                           2 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_PERIMETER_1                                           3 + EEPROM_BASE_ADDRESS
#define ADDRESS_MAX_WHEEL_SPEED                                             4 + EEPROM_BASE_ADDRESS
#define ADDRESS_UNITS_TYPE                                                  5 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_0                                             6 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_1                                             7 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_2                                             8 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_3                                             9 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_0                                 10 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_1                                 11 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_2                                 12 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_3                                 13 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_NUMERIC_BATTERY_SOC                                    14 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_FIELD_STATE                                        15 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_MAX_CURRENT                                         16 + EEPROM_BASE_ADDRESS
#define ADDRESS_TARGET_MAX_BATTERY_POWER                                    17 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_CELLS_NUMBER                                        18 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0                           19 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1                           20 + EEPROM_BASE_ADDRESS
#define ADDRESS_PAS_MAX_CADENCE                                             21 + EEPROM_BASE_ADDRESS
#define ADDRESS_CONFIG_0                                                    22 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL_FACTOR_1                                       23 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL_FACTOR_2                                       24 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL_FACTOR_3                                       25 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL_FACTOR_4                                       26 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL_FACTOR_5                                       27 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL_FACTOR_6                                       28 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL_FACTOR_7                                       29 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL_FACTOR_8                                       30 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL_FACTOR_9                                       31 + EEPROM_BASE_ADDRESS
#define ADDRESS_NUMBER_OF_ASSIST_LEVELS                                     32 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_FEATURE_ENABLED                   33 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_STATE                             34 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1                    35 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_2                    36 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_3                    37 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_4                    38 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_5                    39 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_6                    40 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_7                    41 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_8                    42 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_9                    43 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_TIME                              44 + EEPROM_BASE_ADDRESS
#define ADDRESS_STARTUP_MOTOR_POWER_BOOST_FADE_TIME                         45 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT                           46 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT                           47 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0                      48 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1                      49 + EEPROM_BASE_ADDRESS
#define ADDRESS_LCD_POWER_OFF_TIME                                          50 + EEPROM_BASE_ADDRESS
#define ADDRESS_LCD_BACKLIGHT_ON_BRIGHTNESS                                 51 + EEPROM_BASE_ADDRESS
#define ADDRESS_LCD_BACKLIGHT_OFF_BRIGHTNESS                                52 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_PACK_RESISTANCE_0                                   53 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_PACK_RESISTANCE_1                                   54 + EEPROM_BASE_ADDRESS
#define ADDRESS_OFFROAD_FEATURE_ENABLED                                     55 + EEPROM_BASE_ADDRESS
#define ADDRESS_OFFROAD_MODE_ENABLED_ON_STARTUP                             56 + EEPROM_BASE_ADDRESS
#define ADDRESS_OFFROAD_SPEED_LIMIT                                         57 + EEPROM_BASE_ADDRESS
#define ADDRESS_OFFROAD_POWER_LIMIT_ENABLED                                 58 + EEPROM_BASE_ADDRESS
#define ADDRESS_OFFROAD_POWER_LIMIT_DIV25                                   59 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_X10_0                                              60 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_X10_1                                              61 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_X10_2                                              62 + EEPROM_BASE_ADDRESS
#define EEPROM_BYTES_STORED                                                 63

void eeprom_init (void);
void eeprom_init_variables (void);
void eeprom_write_variables (void);
void eeprom_erase_key_value (void);

#endif /* _EEPROM_H_ */
