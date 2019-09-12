/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho and Leon, 2019.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _MAIN_H_
#define _MAIN_H_


//#define DEBUG_UART



// motor 
#define PWM_CYCLES_COUNTER_MAX                                    3125    // 5 erps minimum speed -> 1/5 = 200 ms; 200 ms / 64 us = 3125
#define PWM_CYCLES_SECOND                                         15625   // 1 / 64us(PWM period)
#define PWM_DUTY_CYCLE_MAX                                        254
#define MIDDLE_PWM_DUTY_CYCLE_MAX                                 (PWM_DUTY_CYCLE_MAX / 2)

#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_DEFAULT               160     // 160 -> 160 * 64 us for every duty cycle increment
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP_MIN                   20      // 20 -> 20 * 64 us for every duty cycle increment

#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_DEFAULT             40      // 40 -> 40 * 64 us for every duty cycle decrement
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP_MIN                 8       // 8 -> 8 * 64 us for every duty cycle decrement

/*---------------------------------------------------------
  NOTE: regarding duty cycle (PWM) ramping
  
  Do not change these values if not sure of the effects!
  
  A lower value of the duty cycle inverse step will mean
  a faster acceleration. Be careful not to choose too
  low values for acceleration.
---------------------------------------------------------*/



#define MOTOR_ROTOR_OFFSET_ANGLE                                  10
#define MOTOR_ROTOR_ANGLE_90                                      (63  + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_150                                     (106 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_210                                     (148 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_270                                     (191 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_330                                     (233 + MOTOR_ROTOR_OFFSET_ANGLE)
#define MOTOR_ROTOR_ANGLE_30                                      (20  + MOTOR_ROTOR_OFFSET_ANGLE)

/*---------------------------------------------------------
  NOTE: regarding motor rotor offset 
  
  The motor rotor offset should be as close to 0 as 
  possible. You can try to tune with the wheel in the air,
  full throttle and look at the batttery current. Adjust 
  for the lowest battery current possible.
---------------------------------------------------------*/



#define MOTOR_OVER_SPEED_ERPS                                     520     // motor max speed, protection max value | 30 points for the sinewave at max speed
#define MOTOR_OVER_SPEED_ERPS_EXPERIMENTAL                        700     // experimental motor speed to allow a higher cadence

#define MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES           10

/*---------------------------------------------------------
  NOTE: regarding motor start interpolation
  
  This value is the ERPS speed after which a transition 
  happens from sinewave and no interpolation to 
  interpolation 60 degrees. Must be found experimentally 
  but a value of 25 may be good.
---------------------------------------------------------*/



#define ADC_10_BIT_BATTERY_CURRENT_MAX                            106     // 18 amps
#define ADC_10_BIT_MOTOR_PHASE_CURRENT_MAX                        177     // 30 amps

/*---------------------------------------------------------
  NOTE: regarding ADC battery current max
  
  This is the maximum current in ADC steps that the motor 
  will be able to draw from the battery. A higher value 
  will give higher torque figures but the limit of the 
  controller is 16 A and it should not be exceeded.
---------------------------------------------------------*/



// throttle ADC values
#define ADC_THROTTLE_MIN_VALUE                                    47
#define ADC_THROTTLE_MAX_VALUE                                    176

/*---------------------------------------------------------
  NOTE: regarding throttle ADC values

  Max voltage value for throttle, in ADC 8 bits step, 
  each ADC 8 bits step = (5 V / 256) = 0.0195

---------------------------------------------------------*/



// cadence sensor
#define CADENCE_SENSOR_NUMBER_MAGNETS                             20
#define CADENCE_SENSOR_NUMBER_MAGNETS_X2                          (CADENCE_SENSOR_NUMBER_MAGNETS * 2)

#define CADENCE_SENSOR_TICKS_COUNTER_MAX                          300
#define CADENCE_SENSOR_TICKS_COUNTER_MIN                          10000

#define CADENCE_SENSOR_PULSE_PERCENTAGE_X10_DEFAULT               500
#define CADENCE_SENSOR_PULSE_PERCENTAGE_X10_MAX                   800
#define CADENCE_SENSOR_PULSE_PERCENTAGE_X10_MIN                   200

/*-------------------------------------------------------------------------------
  NOTE: regarding the cadence sensor
  
  CADENCE_SENSOR_NUMBER_MAGNETS = 20, this is the number of magnets used for
  the cadence sensor. Was validated on August 2018 by Casainho and jbalat
  
  x = (1/(150RPM/60)) / (0.000064)
  
  6250 / CADENCE_SENSOR_NUMBER_MAGNETS ≈ 313 -> 150 RPM
  
  93750 / CADENCE_SENSOR_NUMBER_MAGNETS ≈ 4688 -> 10 RPM
  
  CADENCE_SENSOR_TICKS_COUNTER_MAX = x / CADENCE_SENSOR_NUMBER_MAGNETS
  

  
  CADENCE_SENSOR_NUMBER_MAGNETS_X2 = 40, this is the number of transitions 
  in one crank revolution
  
  x = (1/(150RPM/60)) / (0.000064)
  
  6250 / CADENCE_SENSOR_NUMBER_MAGNETS_X2 ≈ 156 -> 150 RPM
  
  93750 / CADENCE_SENSOR_NUMBER_MAGNETS_X2 ≈ 2344 -> 10 RPM, or 5 RPM if set to around 4600
  
  CADENCE_SENSOR_TICKS_COUNTER_MAX = x / CADENCE_SENSOR_NUMBER_MAGNETS_X2
  
  
  
  Cadence is calculated by counting how much time passes between two 
  transitions. Depending on if all transitions are measured or simply 
  transitions of the same kind it is important to adjust the calculation of 
  pedal cadence. If measuring all transistions it is also important to 
  adjust for the different spacings between the transitions.
-------------------------------------------------------------------------------*/



// Wheel speed sensor
#define WHEEL_SPEED_SENSOR_TICKS_COUNTER_MAX                      135   // something like 200 m/h with a 6'' wheel
#define WHEEL_SPEED_SENSOR_TICKS_COUNTER_MIN                      32767 // could be a bigger number but will make for a slow detection of stopped wheel speed



// default values
#define DEFAULT_VALUE_BATTERY_CURRENT_MAX                         10  // 10 amps
#define DEFAULT_VALUE_TARGET_BATTERY_MAX_POWER_X10                50  // 500 watts
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_0           134 // 48 V battery, LVC = 39.0 (3.0 * 13): (134 + (1 << 8)) = 390
#define DEFAULT_VALUE_BATTERY_LOW_VOLTAGE_CUT_OFF_X10_1           1
#define DEFAULT_VALUE_WHEEL_PERIMETER_0                           2   // 26'' wheel: 2050 mm perimeter (2 + (8 << 8))
#define DEFAULT_VALUE_WHEEL_PERIMETER_1                           8
#define DEFAULT_VALUE_WHEEL_SPEED_MAX                             50  // 50 km/h
#define DEFAULT_VALUE_MOTOR_TYPE                                  0
#define DEFAULT_VALUE_PEDAL_TORQUE_PER_10_BIT_ADC_STEP_X100       67

/*---------------------------------------------------------

  NOTE: regarding the torque sensor output values

  Torque (force) value needs to be found experimentaly.
  
  One torque sensor ADC 10 bit step is equal to 0.38 kg
  
  Force (Nm) = 1 Kg * 9.81 * 0.17 (0.17 = arm cranks size)
---------------------------------------------------------*/



// ADC battery voltage measurement
#define BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X512                  44
#define BATTERY_VOLTAGE_PER_10_BIT_ADC_STEP_X1000                 87  // conversion value verified with a cheap power meter

/*---------------------------------------------------------
  NOTE: regarding ADC battery voltage measurement

  0.344 per ADC 8 bit step:
  
  17.9 V -->  ADC 8 bits value  = 52; 
  40 V   -->  ADC 8 bits value  = 116; 
  
  This signal is atenuated by the opamp 358.
---------------------------------------------------------*/



// ADC battery current measurement
#define BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X512                  102
#define BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100                  17  // conversion value verified with a cheap power meter



#endif // _MAIN_H_