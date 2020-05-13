/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _CONFIG_H_
#define _CONFIG_H_

// This file is the firmware configuration for the TSDZ2 motor controller,
// to run the 2 different available motors of 36V or 48V motor,
// and from 24V battery (7S) up to 52V battery pack (14S).

// The limit of max battery current on original firmware is 16 amps!!
#define ADC_BATTERY_CURRENT_MAX 128 // 20 amps (0.156 amps each unit) - note that on original firmware is 16 amps
#define ADC_MOTOR_CURRENT_MAX 192 // 30 amps (0.156 amps each unit)

// *************************************************************************** //
// MOTOR CONTROLLER

// Choose PWM ramp up/down step (higher value will make the motor acceleration slower)
//
// For a 24V battery, 25 for ramp up seems ok. For an higher voltage battery, this values should be higher
#define PWM_DUTY_CYCLE_RAMP_UP_INVERSE_STEP 44
#define PWM_DUTY_CYCLE_RAMP_DOWN_INVERSE_STEP 34

// The following value were tested by Casainho on 2020.04.23 
#define FIELD_WEAKENING_RAMP_UP_INVERSE_STEP 600
#define FIELD_WEAKENING_RAMP_DOWN_INVERSE_STEP 600

// *************************************************************************** //
// MOTOR

// Choose some parameters for your motor (if you don't know, just keep the following original values because they should work ok)
//
// This value should be near 0.
// You can try to tune with the whell on the air, full throttle and look at batttery current: adjust for lower battery current
#define MOTOR_ROTOR_OFFSET_ANGLE 10

// This value is ERPS speed after which a transition happens from sinewave no interpolation to have
// interpolation 60 degrees and must be found experimentally but a value of 25 may be good
#define MOTOR_ROTOR_ERPS_START_INTERPOLATION_60_DEGREES 10

#endif /* _CONFIG_H_ */
