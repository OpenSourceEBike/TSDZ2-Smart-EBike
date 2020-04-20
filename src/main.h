/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include "config.h"

//#define DISABLE_PWM_CHANNELS_1_3

#define PWM_CYCLES_COUNTER_MAX                    3800    // 5 erps minimum speed; 1/5 = 200ms; 200ms/52.6us = 3800
#define PWM_CYCLES_SECOND                         19011L  // 1 / 64us(PWM period)
#define PWM_DUTY_CYCLE_MAX                        254
#define PWM_DUTY_CYCLE_MIN                        20
#define MIDDLE_PWM_DUTY_CYCLE_MAX                 (PWM_DUTY_CYCLE_MAX/2)

#define MOTOR_ROTOR_ANGLE_90                      (63  + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_150                     (106 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_210                     (148 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_270                     (191 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_330                     (233 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_30                      (20  + MOTOR_ROTOR_OFFSET_ANGLE)

// motor maximum rotation
#define MOTOR_OVER_SPEED_ERPS                     700 // motor max speed, protection max value | 27 points for the sinewave at max speed

// throttle
#define THROTTLE_FILTER_COEFFICIENT               1   // see note below
#define ADC_THROTTLE_THRESHOLD                    10  // value in ADC 8 bits step

/*---------------------------------------------------------
  NOTE: regarding throttle

  Possible values: 0, 1, 2, 3, 4, 5, 6
  0 equals to no filtering and no delay, higher values 
  will increase filtering but will also add a bigger delay.
---------------------------------------------------------*/



// walk assist and cruise
#define WALK_ASSIST_CRUISE_THRESHOLD_SPEED_X10    80    // 8.0 km/h
#define CRUISE_PID_KP                             7    // 48 volt motor: 6, 36 volt motor: 7
#define CRUISE_PID_KI                             0.35   // 48 volt motor: 0.5, 36 volt motor: 0.35
#define CRUISE_PID_INTEGRAL_LIMIT                 1000
#define CRUISE_PID_KD                             0



// throttle ADC values
#define ADC_THROTTLE_MIN_VALUE                    47
#define ADC_THROTTLE_MAX_VALUE                    176

/*---------------------------------------------------------
  NOTE: regarding throttle ADC values

  Max voltage value for throttle, in ADC 8 bits step, 
  each ADC 8 bits step = (5 V / 256) = 0.0195
---------------------------------------------------------*/


// torque sensor
/*---------------------------------------------------------
  NOTE: regarding torque sensor

  Force (Nm) = weight Kg * 9.81 * 0.17 (0.17 = arm cranks size)
---------------------------------------------------------*/
#define TORQUE_SENSOR_WEIGHT_TO_FORCE_X10  17

/*---------------------------------------------------------
  NOTE: regarding torque sensor

  NOTE: the following information is incorrect as the torque sensor output is not linear.

  Torque (force) value found experimentaly.

  Measured with a cheap digital hook scale, we found that
  each torque sensor unit is equal to 0.52 Nm. Using the
  scale it was found that 0.33 kg was measured as 1 torque
  sensor unit.

  Force (Nm) = 1 Kg * 9.18 * 0.17 (0.17 = arm cranks size)
---------------------------------------------------------*/
#define PEDAL_TORQUE_X100 52


// PAS
#define PAS_NUMBER_MAGNETS                                        20 // see note below
#define PAS_NUMBER_MAGNETS_X2                                     (PAS_NUMBER_MAGNETS * 2)
#define PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS                  (7625 / PAS_NUMBER_MAGNETS)   // max hard limit to 150 RPM PAS cadence, see note below
#define PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS                  (114375 / PAS_NUMBER_MAGNETS)  // min hard limit to 10 RPM PAS cadence, see note below

/*---------------------------------------------------------
  NOTE: regarding PAS
  
  PAS_NUMBER_MAGNETS = 20, was validated on August 2018 
  by Casainho and jbalat

  x = (1/(150RPM/60)) / (0.000064)
  
  PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS = 
  (x / PAS_NUMBER_MAGNETS)
---------------------------------------------------------*/



// Wheel speed sensor
#define WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS                    165   // something like 200 m/h with a 6'' wheel
#define WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS                    39976 // could be a bigger number but will make for a slow detection of stopped wheel speed

// default values for ramp up
#define DEFAULT_VALUE_RAMP_UP_AMPS_PER_SECOND_X10                 50  // 5.0 amps per second ramp up


// ADC battery voltage measurement
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512               44
#define ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X256               (ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X512 >> 1)
#define ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP_INVERSE_X256        (ADC10BITS_BATTERY_VOLTAGE_PER_ADC_STEP_X256 << 2)

/*---------------------------------------------------------
  NOTE: regarding ADC battery voltage measurement

  0.344 per ADC_8bits step:
  
  17.9 V -->  ADC_8bits  = 52; 
  40 V   -->  ADC_8bits  = 116; 
  
  This signal is atenuated by the opamp 358.
---------------------------------------------------------*/

// ADC battery current measurement and filter
#define ADC10BITS_BATTERY_CURRENT_PER_ADC_STEP_X512               80 // 1A per 6.4 steps of ADC_10bits (0.156A per each ADC step)
#define READ_BATTERY_CURRENT_FILTER_COEFFICIENT                   2
#define READ_MOTOR_CURRENT_FILTER_COEFFICIENT                     2
#define READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT                   2

/*---------------------------------------------------------
  NOTE: regarding ADC battery current measurement and 
  filter coefficients 

  Possible values: 0, 1, 2, 3, 4, 5, 6
  0 equals to no filtering and no delay, higher values 
  will increase filtering but will also add a bigger delay.
---------------------------------------------------------*/



// motor temperature filter coefficient 
#define READ_MOTOR_TEMPERATURE_FILTER_COEFFICIENT                 5



#endif // _MAIN_H_
