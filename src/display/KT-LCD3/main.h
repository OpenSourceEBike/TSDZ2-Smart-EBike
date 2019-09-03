/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho and Leon, 2019.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#define EXTI_PORTA_IRQHANDLER                     3
#define EXTI_PORTC_IRQHANDLER                     5
#define EXTI_PORTD_IRQHANDLER                     6
#define EXTI_PORTE_IRQHANDLER                     7
#define TIM1_CAP_COM_IRQHANDLER                   12
#define TIM2_UPD_OVF_TRG_BRK_IRQHANDLER           13
#define TIM3_UPD_OVF_BRK_IRQHANDLER               15
#define UART2_IRQHANDLER                          21
#define ADC1_IRQHANDLER                           22



// default values for assist levels
#define DEFAULT_VALUE_ASSIST_LEVEL                                  0
#define DEFAULT_VALUE_NUMBER_OF_ASSIST_LEVELS                       9



// default values for bike wheel parameters
#define DEFAULT_VALUE_WHEEL_PERIMETER_0                             2   // 26 inch wheel: 2050 mm perimeter (2 + (8 << 8))
#define DEFAULT_VALUE_WHEEL_PERIMETER_1                             8
#define DEFAULT_VALUE_WHEEL_MAX_SPEED                               50  // 50 kph
#define DEFAULT_VALUE_MAX_WHEEL_SPEED_IMPERIAL                      20  // 20 mph



// default value for system units
#define DEFAULT_VALUE_UNITS_TYPE                                    0   // 0 = km/h and kilometer, 1 mph and miles



// default values for battery capacity variables
#define DEFAULT_VALUE_WH_OFFSET                                     0
#define DEFAULT_VALUE_HW_X10_100_PERCENT                            0
#define DEFAULT_VALUE_BATTERY_SOC_FUNCTION_ENABLED                  0



// default values for battery parameters
#define DEFAULT_VALUE_BATTERY_MAX_CURRENT                           16  // 16 amps
#define DEFAULT_VALUE_TARGET_MAX_BATTERY_POWER                      10  // 10 -> 10 * 25 = 250 watts
#define DEFAULT_VALUE_BATTERY_CELLS_NUMBER                          13  // 13 -> 48 V
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0             134 // 48 V battery, LVC = 39.0 (3.0 * 13): (134 + (1 << 8))
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1             1



// default value motor type
#define DEFAULT_VALUE_MOTOR_TYPE                                    0 // 0 = 48 volt motor



// default value optional ADC function
#define DEFAULT_VALUE_OPTIONAL_ADC_FUNCTION                         0



// default value temperature field state
#define DEFAULT_VALUE_TEMPERATURE_FIELD_STATE                       0



// default values for power assist
#define DEFAULT_VALUE_POWER_ASSIST_FUNCTION_ENABLED                 1
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_0                          0   // 0
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_1                          3   // 0.3
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_2                          6   // 0.6
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_3                          9
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_4                          12
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_5                          15
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_6                          18
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_7                          21
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_8                          24
#define DEFAULT_VALUE_POWER_ASSIST_LEVEL_9                          30



// default values for motor temperature limit function
#define DEFAULT_VALUE_MOTOR_TEMPERATURE_MIN_VALUE_LIMIT             65  // 70 degrees celsius
#define DEFAULT_VALUE_MOTOR_TEMPERATURE_MAX_VALUE_LIMIT             70



// default values for battery voltage 
#define DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_0        30  // 48 V battery, 54.2 volts fully charged = 54.2: (30 + (2 << 8))
#define DEFAULT_VALUE_BATTERY_VOLTAGE_RESET_WH_COUNTER_X10_1        2



// default values for screen parameters
#define DEFAULT_VALUE_LCD_POWER_OFF_TIME                            10  // 10 -> 10 minutes
#define DEFAULT_VALUE_LCD_BACKLIGHT_ON_BRIGHTNESS                   16  // 16 = 80 %
#define DEFAULT_VALUE_LCD_BACKLIGHT_OFF_BRIGHTNESS                  1   // 1 = 5%



// default values for internal resistance of battery
#define DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_0                     140
#define DEFAULT_VALUE_BATTERY_PACK_RESISTANCE_1                     0



// default values for street mode function
#define DEFAULT_VALUE_STREET_MODE_FUNCTION_ENABLED                  0
#define DEFAULT_VALUE_STREET_MODE_SPEED_LIMIT                       25
#define DEFAULT_VALUE_STREET_MODE_POWER_LIMIT_ENABLED               0
#define DEFAULT_VALUE_STREET_MODE_POWER_LIMIT_DIV25                 10  // 10 * 25 = 250 W
#define DEFAULT_VALUE_STREET_MODE_THROTTLE_ENABLED                  0   // throttle is disabled in street mode by default
#define DEFAULT_VALUE_STREET_MODE_CRUISE_ENABLED                    0   // cruise is disabled in street mode by default



// default values for distance measurement
#define DEFAULT_VALUE_ODOMETER_X10                                  0
#define DEFAULT_VALUE_TRIP_X10                                      0



// default values for the odometer field and sub field states
#define DEFAULT_VALUE_ODOMETER_FIELD_STATE                          0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_0                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_1                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_2                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_3                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_4                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_5                    0
#define DEFAULT_VALUE_ODOMETER_SUB_FIELD_STATE_6                    0



// default values for time measurement
#define DEFAULT_VALUE_TIME_MEASUREMENT_FIELD_STATE                  1   // 1 = display time measurement since power on (TM)
#define DEFAULT_VALUE_TOTAL_SECOND_TTM                              0
#define DEFAULT_VALUE_TOTAL_MINUTE_TTM                              0
#define DEFAULT_VALUE_TOTAL_HOUR_TTM_0                              0
#define DEFAULT_VALUE_TOTAL_HOUR_TTM_1                              0



// default value motor acceleration
#define DEFAULT_VALUE_MOTOR_ACCELERATION                            0



// default values for walk assist function
#define DEFAULT_VALUE_WALK_ASSIST_FUNCTION_ENABLED                  0   // disabled by default
#define DEFAULT_VALUE_WALK_ASSIST_BUTTON_BOUNCE_TIME                0   // 0 milliseconds
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_1                           20
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_2                           25
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_3                           30
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_4                           35
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_5                           40
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_6                           42
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_7                           44
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_8                           46
#define DEFAULT_VALUE_WALK_ASSIST_LEVEL_9                           48



// default values for cruise function
#define DEFAULT_VALUE_CRUISE_FUNCTION_ENABLED                       0   // disabled by default
#define DEFAULT_VALUE_CRUISE_FUNCTION_SET_TARGET_SPEED_ENABLED      0   // disabled by default
#define DEFAULT_VALUE_CRUISE_FUNCTION_TARGET_SPEED_KPH              25  // 25 kph
#define DEFAULT_VALUE_CRUISE_FUNCTION_TARGET_SPEED_MPH              15  // 15 mph
#define DEFAULT_VALUE_SHOW_CRUISE_FUNCTION_SET_TARGET_SPEED         0   // disabled by default



// default values wheel speed field state
#define DEFAULT_VALUE_WHEEL_SPEED_FIELD_STATE                       0   // 0 = display wheel speed, 1 = display average wheel speed, 2 = display max measured wheel speed



// default values for showing odometer variables
#define DEFAULT_VALUE_SHOW_DISTANCE_DATA_ODOMETER_FIELD             1
#define DEFAULT_VALUE_SHOW_BATTERY_STATE_ODOMETER_FIELD             1
#define DEFAULT_VALUE_SHOW_PEDAL_DATA_ODOMETER_FIELD                1
#define DEFAULT_VALUE_SHOW_TIME_MEASUREMENT_ODOMETER_FIELD          0
#define DEFAULT_VALUE_SHOW_WHEEL_SPEED_ODOMETER_FIELD               0
#define DEFAULT_VALUE_SHOW_ENERGY_DATA_ODOMETER_FIELD               1
#define DEFAULT_VALUE_SHOW_MOTOR_TEMPERATURE_ODOMETER_FIELD         0
#define DEFAULT_VALUE_SHOW_BATTERY_SOC_ODOMETER_FIELD               1



// default value for the main screen power menu
#define DEFAULT_VALUE_MAIN_SCREEN_POWER_MENU_ENABLED                1



// default value pedal torque conversion
#define DEFAULT_VALUE_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100         67



// default value cadence sensor mode
#define DEFAULT_VALUE_CADENCE_SENSOR_MODE                           0
#define DEFAULT_VALUE_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_0    244
#define DEFAULT_VALUE_CADENCE_SENSOR_PULSE_HIGH_PERCENTAGE_X10_1    1



// default value for torque assist
#define DEFAULT_VALUE_TORQUE_ASSIST_FUNCTION_ENABLED                0
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_1                         10
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_2                         20
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_3                         30
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_4                         40
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_5                         50
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_6                         60
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_7                         70
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_8                         80
#define DEFAULT_VALUE_TORQUE_ASSIST_LEVEL_9                         90



// default value for cadence assist
#define DEFAULT_VALUE_CADENCE_ASSIST_FUNCTION_ENABLED               0
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_1                        30
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_2                        40
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_3                        50
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_4                        60
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_5                        70
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_6                        80
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_7                        90
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_8                        100
#define DEFAULT_VALUE_CADENCE_ASSIST_LEVEL_9                        120



// default value for eMTB assist
#define DEFAULT_VALUE_EMTB_ASSIST_FUNCTION_ENABLED                  0
#define DEFAULT_VALUE_EMTB_ASSIST_SENSITIVITY                       10



// default value assist without pedal rotation threshold
#define DEFAULT_VALUE_ASSIST_WITHOUT_PEDAL_ROTATION_THRESHOLD       0



// default value lights
#define DEFAULT_VALUE_LIGHTS_MODE                                   0
#define DEFAULT_VALUE_LIGHTS_STATE                                  0
#define DEFAULT_VALUE_LIGHTS_CONFIGURATION                          0



#endif // _MAIN_H_