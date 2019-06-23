/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_


//#define DEBUG_UART

#define PWM_CYCLES_COUNTER_MAX                    3125    // 5 erps minimum speed; 1/5 = 200ms; 200ms/64us = 3125
#define PWM_CYCLES_SECOND                         15625L  // 1 / 64us(PWM period)
#define PWM_DUTY_CYCLE_MAX                        254
#define PWM_DUTY_CYCLE_MIN                        20
#define MIDDLE_PWM_DUTY_CYCLE_MAX                 (PWM_DUTY_CYCLE_MAX/2)


// motor rotor offset 
#define MOTOR_ROTOR_OFFSET_ANGLE                  10

/*---------------------------------------------------------
  NOTE: regarding motor rotor offset 
  
  The motor rotor offset should be as close to 0 as 
  possible. You can try to tune with the wheel in the air,
  full throttle and look at the batttery current. Adjust 
  for the lowest battery current possible.
---------------------------------------------------------*/

#define MOTOR_ROTOR_ANGLE_90                      (63  + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_150                     (106 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_210                     (148 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_270                     (191 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_330                     (233 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_30                      (20  + MOTOR_ROTOR_OFFSET_ANGLE)



// motor phase current
#define ADC_MOTOR_PHASE_CURRENT_MAX               48 // 30 amps (0.625 amps each unit)



// battery current
#define ADC_BATTERY_CURRENT_MAX                   29 // 18 amps (0.625 amps each step)

/*---------------------------------------------------------
  NOTE: regarding ADC_BATTERY_CURRENT_MAX
  
  This is the maximum current in ADC steps that the motor 
  will be able to draw from the battery. A higher value 
  will give higher torque figures but the limit of the 
  controller is 16 A and it should not be exceeded.
---------------------------------------------------------*/



// motor maximum rotation
#define MOTOR_OVER_SPEED_ERPS                     520 // motor max speed, protection max value | 30 points for the sinewave at max speed
#define MOTOR_OVER_SPEED_ERPS_EXPERIMENTAL        700 // experimental motor speed to allow a higher cadence



// motor start interpolation
#define MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES           10

/*---------------------------------------------------------
  NOTE: regarding motor start interpolation
  
  This value is the ERPS speed after which a transition 
  happens from sinewave and no interpolation to 
  interpolation 60 degrees. Must be found experimentally 
  but a value of 25 may be good.
---------------------------------------------------------*/



// duty cycle (PWM) ramping
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP       20  // 20 -> 20 * 64 us for every duty cycle increment
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP     20

/*---------------------------------------------------------
  NOTE: regarding duty cycle (PWM) ramping
  
  Choose appropriate duty cycle (PWM) ramp up/down step. 
  A higher value will make the motor acceleration slower.
  
  For a 24 V battery, 25 for ramp up seems ok. For a 
  higher voltage battery, this values should be higher.
---------------------------------------------------------*/



// throttle
#define THROTTLE_FILTER_COEFFICIENT               1   // see note below
#define ADC_THROTTLE_THRESHOLD                    10  // value in ADC 8 bits step


/*---------------------------------------------------------
  NOTE: regarding throttle

  Possible values: 0, 1, 2, 3, 4, 5, 6
  0 equals to no filtering and no delay, higher values 
  will increase filtering but will also add a bigger delay.
---------------------------------------------------------*/



// throttle ADC values
#define ADC_THROTTLE_MIN_VALUE                    47
#define ADC_THROTTLE_MAX_VALUE                    176

/*---------------------------------------------------------
  NOTE: regarding throttle ADC values

  Max voltage value for throttle, in ADC 8 bits step, 
  each ADC 8 bits step = (5 V / 256) = 0.0195
---------------------------------------------------------*/



// PAS
#define PAS_NUMBER_MAGNETS                                        20 // see note below
#define PAS_NUMBER_MAGNETS_X2                                     (PAS_NUMBER_MAGNETS * 2)
#define PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS                  (6250 / PAS_NUMBER_MAGNETS)   // max hard limit to 150 RPM PAS cadence, see note below
#define PAS_ABSOLUTE_MIN_CADENCE_PWM_CYCLE_TICKS                  (93750 / PAS_NUMBER_MAGNETS)  // min hard limit to 10 RPM PAS cadence, see note below

/*---------------------------------------------------------
  NOTE: regarding PAS
  
  PAS_NUMBER_MAGNETS = 20, was validated on August 2018 
  by Casainho and jbalat

  x = (1/(150RPM/60)) / (0.000064)
  
  PAS_ABSOLUTE_MAX_CADENCE_PWM_CYCLE_TICKS = 
  (x / PAS_NUMBER_MAGNETS)
---------------------------------------------------------*/



// Wheel speed sensor
#define WHEEL_SPEED_SENSOR_MAX_PWM_CYCLE_TICKS                    135   // something like 200 m/h with a 6'' wheel
#define WHEEL_SPEED_SENSOR_MIN_PWM_CYCLE_TICKS                    32767 // could be a bigger number but will make for a slow detection of stopped wheel speed



// EEPROM memory variables default values
#define DEFAULT_VALUE_ASSIST_LEVEL_FACTOR_X10                     40  // 4.0
#define DEFAULT_VALUE_CONFIG_0                                    0
#define DEFAULT_VALUE_BATTERY_MAX_CURRENT                         10  // 10 amps
#define DEFAULT_VALUE_TARGET_BATTERY_MAX_POWER_X10                50  // 500 watts
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0           134 // 48 V battery, LVC = 39.0 (3.0 * 13): (134 + (1 << 8)) = 390
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1           1
#define DEFAULT_VALUE_WHEEL_PERIMETER_0                           2   // 26'' wheel: 2050 mm perimeter (2 + (8 << 8))
#define DEFAULT_VALUE_WHEEL_PERIMETER_1                           8
#define DEFAULT_VALUE_WHEEL_MAX_SPEED                             50  // 50 km/h
#define DEFAULT_VALUE_CONFIG_1                                    0



// default values for ramp up
#define DEFAULT_VALUE_RAMP_UP_AMPS_PER_SECOND                     5  // 5 amps per second ramp up


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
#define ADC_BATTERY_CURRENT_PER_ADC_STEP_X512                     102 // 1 A per 5 steps of ADC_10bits
#define ADC_BATTERY_VOLTAGE_MIN                                   (uint8_t) ((float) (BATTERY_LI_ION_CELLS_NUMBER * LI_ION_CELL_VOLTS_0) / ADC8BITS_BATTERY_VOLTAGE_PER_ADC_STEP)
#define READ_BATTERY_CURRENT_FILTER_COEFFICIENT                   2
#define READ_BATTERY_VOLTAGE_FILTER_COEFFICIENT                   2

/*---------------------------------------------------------
  NOTE: regarding ADC battery current measurement and 
  filter coefficients 

  Possible values: 0, 1, 2, 3, 4, 5, 6
  0 equals to no filtering and no delay, higher values 
  will increase filtering but will also add a bigger delay.
---------------------------------------------------------*/



// motor temperature filter coefficient 
#define READ_MOTOR_TEMPERATURE_FILTER_COEFFICIENT                 4



#endif // _MAIN_H_
