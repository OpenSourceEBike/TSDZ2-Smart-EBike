/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#define EXTI_PORTA_IRQHANDLER 3
#define EXTI_PORTC_IRQHANDLER 5
#define EXTI_PORTD_IRQHANDLER 6
#define EXTI_PORTE_IRQHANDLER 7
#define TIM1_CAP_COM_IRQHANDLER 12
#define TIM2_UPD_OVF_TRG_BRK_IRQHANDLER 13
#define TIM3_UPD_OVF_BRK_IRQHANDLER 15
#define UART2_IRQHANDLER 21
#define ADC1_IRQHANDLER 22

// *************************************************************************** //
// EEPROM memory variables default values
#define DEFAULT_VALUE_ASSIST_LEVEL                                  3
#define DEFAULT_VALUE_NUMBER_OF_ASSIST_LEVELS                       5
#define DEFAULT_VALUE_WHEEL_PERIMETER_0                             2 // 26'' wheel: 2050mm perimeter (2 + (8 << 8))
#define DEFAULT_VALUE_WHEEL_PERIMETER_1                             8
#define DEFAULT_VALUE_WHEEL_MAX_SPEED                               50 // 50 kph
#define DEFAULT_VALUE_MAX_WHEEL_SPEED_IMPERIAL                      20 // 20 mph
#define DEFAULT_VALUE_UNITS_TYPE                                    0 // 0 = km/h and km

#define DEFAULT_VALUE_WH_OFFSET                                     0
#define DEFAULT_VALUE_HW_X10_100_PERCENT                            0
#define DEAFULT_VALUE_SHOW_NUMERIC_BATTERY_SOC                      0
#define DEFAULT_VALUE_ODOMETER_FIELD_STATE                          0

#define DEFAULT_VALUE_BATTERY_MAX_CURRENT                           16 // 16 amps
#define DEFAULT_VALUE_TARGET_MAX_BATTERY_POWER                      0 // e.g. 20 = 20 * 25 = 500, 0 is disabled
#define DEFAULT_VALUE_BATTERY_CELLS_NUMBER                          13 // 13 --> 48V
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0             134 // 48v battery, LVC = 39.0 (3.0 * 13): (134 + (1 << 8))
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1             1
#define DEFAULT_VALUE_CONFIG_0                                      0 // ui8_motor_type = 0; ui8_motor_assistance_startup_config = 0

#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_1                         3 // 0.3
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_2                         6
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_3                         9
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_4                         12
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_5                         15
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_6                         18
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_7                         21
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_8                         24
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_9                         30

#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_FEATURE_ENABLED     0
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_STATE               1
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_1      4
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_2      7
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_3      10
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_4      13
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_5      16
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_6      19
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_7      22
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_8      25
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_ASSIST_LEVEL_9      28
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_TIME                20 // 2.0 seconds
#define DEFAULT_VALUE_STARTUP_MOTOR_POWER_BOOST_FADE_TIME           35 // 3.5 seconds

#define DEFAULT_VALUE_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT             75 // 75 degrees celsius
#define DEFAULT_VALUE_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT             85
#define DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0        30 // 48v battery, 54.2 volts fully charged = 54.2: (30 + (2 << 8))
#define DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1        2
#define DEFAULT_VALUE_LCD_POWER_OFF_TIME                            15 // 15 minutes, each unit 1 minute
#define DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS                   16 // 16 = 80%
#define DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS                  1 // 1 = 5%
#define DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_0                     130 // 48v battery, 13S5P measured 130 milli ohms
#define DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_1                     0
#define DEFAULT_VALUE_OFFROAD_FEATURE_ENABLED                       0
#define DEFAULT_VALUE_OFFROAD_MODE_ENABLED_ON_STARTUP               0
#define DEFAULT_VALUE_OFFROAD_SPEED_LIMIT                           25
#define DEFAULT_VALUE_OFFROAD_POWER_LIMIT_ENABLED                   0
#define DEFAULT_VALUE_OFFROAD_POWER_LIMIT_DIV25                     10 //10 * 25 = 250W
#define DEFAULT_VALUE_ODOMETER_X10                                  0
#define DEFAULT_VALUE_TRIP_X10                                      0

// default values for the sub field menus for every odometer field state
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_0                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_1                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_2                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_3                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_4                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_5                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_6                    0

// default values for time measurement
#define DEFAULT_VALUE_TIME_MEASUREMENT_FIELD_STATE                  1 // 1 = display time measurement since last power on (TM)
#define DEFAULT_VALUE_TOTAL_SECOND_TTM                              0
#define DEFAULT_VALUE_TOTAL_MINUTE_TTM                              0
#define DEFAULT_VALUE_TOTAL_HOUR_TTM_0                              0
#define DEFAULT_VALUE_TOTAL_HOUR_TTM_1                              0

// default values for ADC battery current ramp up inverse step
#define DEFAULT_VALUE_ADC_BATTERY_CURRENT_RAMP_UP_INVERSE_STEP_0    161 // 1953, see note below, (161 + (7 << 8)) = 1953
#define DEFAULT_VALUE_ADC_BATTERY_CURRENT_RAMP_UP_INVERSE_STEP_1    7

/*---------------------------------------------------------
  NOTE: regarding ADC_BATTERY_CURRENT_RAMP_UP_INVERSE_STEP

  Target: ramp 5 amps per second

  Every second has 15625 PWM cycles interrupts,
  one ADC battery current step --> 0.625 amps:

  5 / 0.625 = 8 (we need to do 8 steps ramp up per second)

  Therefore:

  15625 / 8 = 1953 (our default value)
---------------------------------------------------------*/

// default values for walk assist function
#define DEFAULT_VALUE_WALK_ASSIST_FUNCTION_ENABLED                  0 // disabled
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_0                    0
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_1                    20
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_2                    25
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_3                    30
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_4                    35
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_5                    40
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_6                    45
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_7                    50
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_8                    55
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_FACTOR_9                    60

// default values wheel speed measurement
#define DEFAULT_VALUE_WHEEL_SPEED_FIELD_STATE                       0 // 0 = display wheel speed, 1 = display average wheel speed, 2 = display max measured wheel speed

// default values for cruise function
#define DEFAULT_VALUE_CRUISE_FUNCTION_ENABLED                       0 // disabled

// *************************************************************************** //

// Torque sensor value found experimentaly
// measuring with a cheap digital hook scale, we found that each torque sensor unit is equal to 0.556 Nm
// using the scale, was found that each 1kg was measured as 3 torque sensor units
// Force (Nm) = Kg * 9.18 * 0.17 (arm cranks size)
#define TORQUE_SENSOR_FORCE_SCALE_X1000 556

// *************************************************************************** //
// BATTERY

// ADC Battery voltage
// 0.344 per ADC_8bits step: 17.9V --> ADC_8bits = 52; 40V --> ADC_8bits = 116; this signal atenuated by the opamp 358
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512 44
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X256 (ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512 >> 1)
#define ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP 0.344

// ADC Battery current
// 1A per 5 steps of ADC_10bits
#define ADC_BATTERY_CURRENT_PER_ADC_STEP_X512 102
// *************************************************************************** //

#endif // _MAIN_H_
