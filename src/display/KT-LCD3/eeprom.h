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


#define EEPROM_BASE_ADDRESS                                                 0x4000
#define ADDRESS_KEY                                                         0 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_LEVEL                                                1 + EEPROM_BASE_ADDRESS
#define ADDRESS_NUMBER_OF_ASSIST_LEVELS                                     2 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_PERIMETER_0                                           3 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_PERIMETER_1                                           4 + EEPROM_BASE_ADDRESS
#define ADDRESS_MAX_WHEEL_SPEED                                             5 + EEPROM_BASE_ADDRESS
#define ADDRESS_UNITS_TYPE                                                  6 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_0                                             7 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_1                                             8 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_2                                             9 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_OFFSET_3                                             10 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_0                                 11 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_1                                 12 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_2                                 13 + EEPROM_BASE_ADDRESS
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_3                                 14 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_SOC_FUNCTION_ENABLED                                15 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_FIELD_STATE                                        16 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_MAX_CURRENT                                         17 + EEPROM_BASE_ADDRESS
#define ADDRESS_TARGET_MAX_BATTERY_POWER                                    18 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_CELLS_NUMBER                                        19 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0                           20 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1                           21 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_TYPE                                                  22 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_FUNCTION_ENABLED                               23 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_1                                        24 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_2                                        25 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_3                                        26 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_4                                        27 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_5                                        28 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_6                                        29 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_7                                        30 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_8                                        31 + EEPROM_BASE_ADDRESS
#define ADDRESS_POWER_ASSIST_LEVEL_9                                        32 + EEPROM_BASE_ADDRESS
#define ADDRESS_OPTIONAL_ADC_FUNCTION                                       33 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT                           34 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT                           35 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0                      36 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1                      37 + EEPROM_BASE_ADDRESS
#define ADDRESS_LCD_POWER_OFF_TIME                                          38 + EEPROM_BASE_ADDRESS
#define ADDRESS_LCD_BACKLIGHT_ON_BRIGHTNESS                                 39 + EEPROM_BASE_ADDRESS
#define ADDRESS_LCD_BACKLIGHT_OFF_BRIGHTNESS                                40 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_PACK_RESISTANCE_0                                   41 + EEPROM_BASE_ADDRESS
#define ADDRESS_BATTERY_PACK_RESISTANCE_1                                   42 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_FUNCTION_ENABLED                                43 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_SPEED_LIMIT                                     44 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_POWER_LIMIT_ENABLED                             45 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_POWER_LIMIT_DIV25                               46 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_THROTTLE_ENABLED                                47 + EEPROM_BASE_ADDRESS
#define ADDRESS_STREET_MODE_CRUISE_ENABLED                                  48 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_X10_0                                              49 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_X10_1                                              50 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_X10_2                                              51 + EEPROM_BASE_ADDRESS
#define ADDRESS_TRIP_X10_0                                                  52 + EEPROM_BASE_ADDRESS
#define ADDRESS_TRIP_X10_1                                                  53 + EEPROM_BASE_ADDRESS
#define ADDRESS_TRIP_X10_2                                                  54 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_0                                  55 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_1                                  56 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_2                                  57 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_3                                  58 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_4                                  59 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_5                                  60 + EEPROM_BASE_ADDRESS
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_6                                  61 + EEPROM_BASE_ADDRESS
#define ADDRESS_MAX_WHEEL_SPEED_IMPERIAL                                    62 + EEPROM_BASE_ADDRESS
#define ADDRESS_TIME_MEASUREMENT_FIELD_STATE                                63 + EEPROM_BASE_ADDRESS
#define ADDRESS_TOTAL_SECOND_TTM                                            64 + EEPROM_BASE_ADDRESS
#define ADDRESS_TOTAL_MINUTE_TTM                                            65 + EEPROM_BASE_ADDRESS
#define ADDRESS_TOTAL_HOUR_TTM_0                                            66 + EEPROM_BASE_ADDRESS
#define ADDRESS_TOTAL_HOUR_TTM_1                                            67 + EEPROM_BASE_ADDRESS
#define ADDRESS_MOTOR_ACCELERATION                                          68 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_FUNCTION_ENABLED                                69 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_1                                         70 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_2                                         71 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_3                                         72 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_4                                         73 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_5                                         74 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_6                                         75 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_7                                         76 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_8                                         77 + EEPROM_BASE_ADDRESS
#define ADDRESS_WALK_ASSIST_LEVEL_9                                         78 + EEPROM_BASE_ADDRESS
#define ADDRESS_CRUISE_FUNCTION_ENABLED                                     79 + EEPROM_BASE_ADDRESS
#define ADDRESS_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED                    80 + EEPROM_BASE_ADDRESS
#define ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_KPH                            81 + EEPROM_BASE_ADDRESS
#define ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_MPH                            82 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED                       83 + EEPROM_BASE_ADDRESS
#define ADDRESS_WHEEL_SPEED_FIELD_STATE                                     84 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_DISTANCE_DATA_ODOMETER_FIELD                           85 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_BATTERY_STATE_ODOMETER_FIELD                           86 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_PEDAL_DATA_ODOMETER_FIELD                              87 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD                        88 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_WHEEL_SPEED_ODOMETER_FIELD                             89 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_ENERGY_DATA_ODOMETER_FIELD                             90 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD                       91 + EEPROM_BASE_ADDRESS
#define ADDRESS_SHOW_BATTERY_SOC_ODOMETER_FIELD                             92 + EEPROM_BASE_ADDRESS
#define ADDRESS_MAIN_SCREEN_POWER_MENU_ENABLED                              93 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_FUNCTION_ENABLED                              94 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_1                                       95 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_2                                       96 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_3                                       97 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_4                                       98 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_5                                       99 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_6                                       100 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_7                                       101 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_8                                       102 + EEPROM_BASE_ADDRESS
#define ADDRESS_TORQUE_ASSIST_LEVEL_9                                       103 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_FUNCTION_ENABLED                             104 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_1                                      105 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_2                                      106 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_3                                      107 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_4                                      108 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_5                                      109 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_6                                      110 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_7                                      111 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_8                                      112 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_ASSIST_LEVEL_9                                      113 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_SENSOR_MODE                                         114 + EEPROM_BASE_ADDRESS
#define ADDRESS_EMTB_ASSIST_FUNCTION_ENABLED                                115 + EEPROM_BASE_ADDRESS
#define ADDRESS_EMTB_ASSIST_SENSITIVITY                                     116 + EEPROM_BASE_ADDRESS
#define ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100                       117 + EEPROM_BASE_ADDRESS
#define ADDRESS_TEMPERATURE_FIELD_STATE                                     118 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_0                  119 + EEPROM_BASE_ADDRESS
#define ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_1                  120 + EEPROM_BASE_ADDRESS
#define ADDRESS_LIGHTS_CONFIGURATION                                        121 + EEPROM_BASE_ADDRESS
#define ADDRESS_LIGHTS_STATE                                                122 + EEPROM_BASE_ADDRESS
#define ADDRESS_ASSIST_WITHOUT_PEDAL_ROTATION_THRESHOLD                     123 + EEPROM_BASE_ADDRESS
#define EEPROM_BYTES_STORED                                                 124


#define DEFAULT_VALUE_KEY     202
#define SET_TO_DEFAULT        0
#define READ_FROM_MEMORY      1
#define WRITE_TO_MEMORY       2


void EEPROM_init(void);
void EEPROM_controller(uint8_t ui8_operation);


#endif /* _EEPROM_H_ */
