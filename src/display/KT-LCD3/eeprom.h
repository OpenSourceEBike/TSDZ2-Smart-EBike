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


#define KEY                                                                 0xe4

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
#define ADDRESS_BATTERY_SOC_FUNCTION_ENABLED                                14 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_FIELD_STATE                                        15 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_MAX_CURRENT                                         16 + EEPROM_BASE_ADDRESS
#define ADDRESS_TARGET_MAX_BATTERY_POWER                                    17 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_CELLS_NUMBER                                        18 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0                           19 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1                           20 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_TYPE                                                  21 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_FUNCTION_ENABLED                               22 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_1                                        23 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_2                                        24 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_3                                        25 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_4                                        26 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_5                                        27 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_6                                        28 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_7                                        29 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_8                                        30 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_9                                        31 + EEPROM_BASE_ADDRESS
#define ADDRESS_NUMBER_OF_ASSIST_LEVELS                                     32 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT                           33 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT                           34 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0                      35 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1                      36 + EEPROM_BASE_ADDRESS
#define ADDRESS_LCD_POWER_OFF_TIME                                          37 + EEPROM_BASE_ADDRESS
#define ADDRESS_LCD_BACKLIGHT_ON_BRIGHTNESS                                 38 + EEPROM_BASE_ADDRESS
#define ADDRESS_LCD_BACKLIGHT_OFF_BRIGHTNESS                                39 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_PACK_RESISTANCE_0                                   40 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_PACK_RESISTANCE_1                                   41 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_FUNCTION_ENABLED                                42 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_SPEED_LIMIT                                     43 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_POWER_LIMIT_ENABLED                             44 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_POWER_LIMIT_DIV25                               45 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_X10_0                                              46 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_X10_1                                              47 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_X10_2                                              48 + EEPROM_BASE_ADDRESS
#define ADDRESS_TRIP_X10_0                                                  49 + EEPROM_BASE_ADDRESS
#define ADDRESS_TRIP_X10_1                                                  50 + EEPROM_BASE_ADDRESS
#define ADDRESS_TRIP_X10_2                                                  51 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_0                                  52 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_1                                  53 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_2                                  54 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_3                                  55 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_4                                  56 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_5                                  57 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_6                                  58 + EEPROM_BASE_ADDRESS
#define ADDRESS_MAX_WHEEL_SPEED_IMPERIAL                                    59 + EEPROM_BASE_ADDRESS
#define ADDRESS_TIME_MEASUREMENT_FIELD_STATE                                60 + EEPROM_BASE_ADDRESS
#define ADDRESS_TOTAL_SECOND_TTM                                            61 + EEPROM_BASE_ADDRESS
#define ADDRESS_TOTAL_MINUTE_TTM                                            62 + EEPROM_BASE_ADDRESS
#define ADDRESS_TOTAL_HOUR_TTM_0                                            63 + EEPROM_BASE_ADDRESS
#define ADDRESS_TOTAL_HOUR_TTM_1                                            64 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_ACCELERATION                                          65 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_FUNCTION_ENABLED                                66 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_1                                         67 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_2                                         68 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_3                                         69 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_4                                         70 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_5                                         71 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_6                                         72 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_7                                         73 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_8                                         74 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_9                                         75 + EEPROM_BASE_ADDRESS
#define ADDRESS_CRUISE_FUNCTION_ENABLED                                     76 + EEPROM_BASE_ADDRESS
#define ADDRESS_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED                    77 + EEPROM_BASE_ADDRESS
#define ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_KPH                            78 + EEPROM_BASE_ADDRESS
#define ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_MPH                            79 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED                       80 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_SPEED_FIELD_STATE                                     81 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_DISTANCE_DATA_ODOMETER_FIELD                           82 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_BATTERY_STATE_ODOMETER_FIELD                           83 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_PEDAL_DATA_ODOMETER_FIELD                              84 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD                        85 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_WHEEL_SPEED_ODOMETER_FIELD                             86 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_ENERGY_DATA_ODOMETER_FIELD                             87 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD                       88 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_BATTERY_SOC_ODOMETER_FIELD                             89 + EEPROM_BASE_ADDRESS
#define ADDRESS_MAIN_SCREEN_POWER_MENU_ENABLED                              90 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_THROTTLE_ENABLED                                91 + EEPROM_BASE_ADDRESS
#define ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100                       92 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_FUNCTION_ENABLED                              93 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_1                                       94 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_2                                       95 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_3                                       96 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_4                                       97 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_5                                       98 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_6                                       99 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_7                                       100 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_8                                       101 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_9                                       102 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_FUNCTION_ENABLED                             103 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_1                                      104 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_2                                      105 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_3                                      106 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_4                                      107 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_5                                      108 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_6                                      109 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_7                                      110 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_8                                      111 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_9                                      112 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_SENSOR_MODE                                         113 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_CRUISE_ENABLED                                  114 + EEPROM_BASE_ADDRESS
#define ADDRESS_EMTB_ASSIST_FUNCTION_ENABLED                                115 + EEPROM_BASE_ADDRESS
#define ADDRESS_EMTB_ASSIST_SENSITIVITY                                     116 + EEPROM_BASE_ADDRESS
#define ADRESS_OPTIONAL_ADC_FUNCTION                                        117 + EEPROM_BASE_ADDRESS
#define ADRESS_TEMPERATURE_FIELD_STATE                                      118 + EEPROM_BASE_ADDRESS
#define EEPROM_BYTES_STORED                                                 119


void eeprom_init(void);
void eeprom_init_variables(void);
void eeprom_write_variables(void);
void eeprom_erase_key_value(void);

#endif /* _EEPROM_H_ */
