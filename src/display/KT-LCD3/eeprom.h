/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho and Leon, 2019.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_

#include "lcd.h"


#define EEPROM_BASE_ADDRESS                                                 0x4000
#define ADDRESS_KEY                                                         0
#define ADDRESS_ASSIST_LEVEL                                                1
#define ADDRESS_NUMBER_OF_ASSIST_LEVELS                                     2
#define ADDRESS_WHEEL_PERIMETER_0                                           3
#define ADDRESS_WHEEL_PERIMETER_1                                           4
#define ADDRESS_MAX_WHEEL_SPEED                                             5
#define ADDRESS_UNITS_TYPE                                                  6
#define ADDRESS_HW_X10_OFFSET_0                                             7
#define ADDRESS_HW_X10_OFFSET_1                                             8
#define ADDRESS_HW_X10_OFFSET_2                                             9
#define ADDRESS_HW_X10_OFFSET_3                                             10
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_0                                 11
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_1                                 12
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_2                                 13
#define ADDRESS_HW_X10_100_PERCENT_OFFSET_3                                 14
#define ADDRESS_BATTERY_SOC_FUNCTION_ENABLED                                15
#define ADDRESS_ODOMETER_FIELD_STATE                                        16
#define ADDRESS_BATTERY_MAX_CURRENT                                         17
#define ADDRESS_TARGET_MAX_BATTERY_POWER                                    18
#define ADDRESS_BATTERY_CELLS_NUMBER                                        19
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0                           20
#define ADDRESS_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1                           21
#define ADDRESS_MOTOR_TYPE                                                  22
#define ADDRESS_POWER_ASSIST_FUNCTION_ENABLED                               23
#define ADDRESS_POWER_ASSIST_LEVEL_1                                        24
#define ADDRESS_POWER_ASSIST_LEVEL_2                                        25
#define ADDRESS_POWER_ASSIST_LEVEL_3                                        26
#define ADDRESS_POWER_ASSIST_LEVEL_4                                        27
#define ADDRESS_POWER_ASSIST_LEVEL_5                                        28
#define ADDRESS_POWER_ASSIST_LEVEL_6                                        29
#define ADDRESS_POWER_ASSIST_LEVEL_7                                        30
#define ADDRESS_POWER_ASSIST_LEVEL_8                                        31
#define ADDRESS_POWER_ASSIST_LEVEL_9                                        32
#define ADDRESS_OPTIONAL_ADC_FUNCTION                                       33
#define ADDRESS_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT                           34
#define ADDRESS_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT                           35
#define ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0                      36
#define ADDRESS_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1                      37
#define ADDRESS_LCD_POWER_OFF_TIME                                          38
#define ADDRESS_LCD_BACKLIGHT_ON_BRIGHTNESS                                 39
#define ADDRESS_LCD_BACKLIGHT_OFF_BRIGHTNESS                                40
#define ADDRESS_BATTERY_PACK_RESISTANCE_0                                   41
#define ADDRESS_BATTERY_PACK_RESISTANCE_1                                   42
#define ADDRESS_STREET_MODE_FUNCTION_ENABLED                                43
#define ADDRESS_STREET_MODE_SPEED_LIMIT                                     44
#define ADDRESS_STREET_MODE_POWER_LIMIT_ENABLED                             45
#define ADDRESS_STREET_MODE_POWER_LIMIT_DIV25                               46
#define ADDRESS_STREET_MODE_THROTTLE_ENABLED                                47
#define ADDRESS_STREET_MODE_CRUISE_ENABLED                                  48
#define ADDRESS_ODOMETER_X10_0                                              49
#define ADDRESS_ODOMETER_X10_1                                              50
#define ADDRESS_ODOMETER_X10_2                                              51
#define ADDRESS_TRIP_X10_0                                                  52
#define ADDRESS_TRIP_X10_1                                                  53
#define ADDRESS_TRIP_X10_2                                                  54
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_0                                  55
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_1                                  56
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_2                                  57
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_3                                  58
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_4                                  59
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_5                                  60
#define ADDRESS_ODOMETER_SUB_FIELD_STATE_6                                  61
#define ADDRESS_MAX_WHEEL_SPEED_IMPERIAL                                    62
#define ADDRESS_TIME_MEASUREMENT_FIELD_STATE                                63
#define ADDRESS_TOTAL_SECOND_TTM                                            64
#define ADDRESS_TOTAL_MINUTE_TTM                                            65
#define ADDRESS_TOTAL_HOUR_TTM_0                                            66
#define ADDRESS_TOTAL_HOUR_TTM_1                                            67
#define ADDRESS_MOTOR_ACCELERATION                                          68
#define ADDRESS_WALK_ASSIST_FUNCTION_ENABLED                                69
#define ADDRESS_WALK_ASSIST_LEVEL_1                                         70
#define ADDRESS_WALK_ASSIST_LEVEL_2                                         71
#define ADDRESS_WALK_ASSIST_LEVEL_3                                         72
#define ADDRESS_WALK_ASSIST_LEVEL_4                                         73
#define ADDRESS_WALK_ASSIST_LEVEL_5                                         74
#define ADDRESS_WALK_ASSIST_LEVEL_6                                         75
#define ADDRESS_WALK_ASSIST_LEVEL_7                                         76
#define ADDRESS_WALK_ASSIST_LEVEL_8                                         77
#define ADDRESS_WALK_ASSIST_LEVEL_9                                         78
#define ADDRESS_CRUISE_FUNCTION_ENABLED                                     79
#define ADDRESS_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED                    80
#define ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_KPH                            81
#define ADDRESS_CRUISE_FUNCTION_TARGET_SPEED_MPH                            82
#define ADDRESS_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED                       83
#define ADDRESS_WHEEL_SPEED_FIELD_STATE                                     84
#define ADDRESS_SHOW_DISTANCE_DATA_ODOMETER_FIELD                           85
#define ADDRESS_SHOW_BATTERY_STATE_ODOMETER_FIELD                           86
#define ADDRESS_SHOW_PEDAL_DATA_ODOMETER_FIELD                              87
#define ADDRESS_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD                        88
#define ADDRESS_SHOW_WHEEL_SPEED_ODOMETER_FIELD                             89
#define ADDRESS_SHOW_ENERGY_DATA_ODOMETER_FIELD                             90
#define ADDRESS_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD                       91
#define ADDRESS_SHOW_BATTERY_SOC_ODOMETER_FIELD                             92
#define ADDRESS_MAIN_SCREEN_POWER_MENU_ENABLED                              93
#define ADDRESS_TORQUE_ASSIST_FUNCTION_ENABLED                              94
#define ADDRESS_TORQUE_ASSIST_LEVEL_1                                       95
#define ADDRESS_TORQUE_ASSIST_LEVEL_2                                       96
#define ADDRESS_TORQUE_ASSIST_LEVEL_3                                       97
#define ADDRESS_TORQUE_ASSIST_LEVEL_4                                       98
#define ADDRESS_TORQUE_ASSIST_LEVEL_5                                       99
#define ADDRESS_TORQUE_ASSIST_LEVEL_6                                       100
#define ADDRESS_TORQUE_ASSIST_LEVEL_7                                       101
#define ADDRESS_TORQUE_ASSIST_LEVEL_8                                       102
#define ADDRESS_TORQUE_ASSIST_LEVEL_9                                       103
#define ADDRESS_CADENCE_ASSIST_FUNCTION_ENABLED                             104
#define ADDRESS_CADENCE_ASSIST_LEVEL_1                                      105
#define ADDRESS_CADENCE_ASSIST_LEVEL_2                                      106
#define ADDRESS_CADENCE_ASSIST_LEVEL_3                                      107
#define ADDRESS_CADENCE_ASSIST_LEVEL_4                                      108
#define ADDRESS_CADENCE_ASSIST_LEVEL_5                                      109
#define ADDRESS_CADENCE_ASSIST_LEVEL_6                                      110
#define ADDRESS_CADENCE_ASSIST_LEVEL_7                                      111
#define ADDRESS_CADENCE_ASSIST_LEVEL_8                                      112
#define ADDRESS_CADENCE_ASSIST_LEVEL_9                                      113
#define ADDRESS_CADENCE_SENSOR_MODE                                         114
#define ADDRESS_EMTB_ASSIST_FUNCTION_ENABLED                                115
#define ADDRESS_EMTB_ASSIST_SENSITIVITY                                     116
#define ADDRESS_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100                       117
#define ADDRESS_TEMPERATURE_FIELD_STATE                                     118
#define ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_0                  119
#define ADDRESS_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_1                  120
#define ADDRESS_LIGHTS_MODE                                                 121
#define ADDRESS_LIGHTS_STATE                                                122
#define ADDRESS_ASSIST_WITHOUT_PEDAL_ROTATION_THRESHOLD                     123
#define ADDRESS_LIGHTS_CONFIGURATION                                        124
#define ADDRESS_WALK_ASSIST_BUTTON_BOUNCE_TIME                              125
#define EEPROM_BYTES_STORED                                                 126


#define DEFAULT_VALUE_KEY     202
#define SET_TO_DEFAULT        0
#define READ_FROM_MEMORY      1
#define WRITE_TO_MEMORY       2


void EEPROM_init(void);
void EEPROM_controller(uint8_t ui8_operation);


#endif /* _EEPROM_H_ */
