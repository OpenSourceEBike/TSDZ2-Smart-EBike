/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef COMMON_COMMON_H_
#define COMMON_COMMON_H_

// error codes
#define NO_ERROR                                0
#define ERROR_MOTOR_BLOCKED                     1
#define ERROR_TORQUE_APPLIED_DURING_POWER_ON    2
#define ERROR_BRAKE_APPLIED_DURING_POWER_ON     3
#define ERROR_THROTTLE_APPLIED_DURING_POWER_ON  4
#define ERROR_NO_SPEED_SENSOR_DETECTED          5
#define ERROR_LOW_CONTROLLER_VOLTAGE            6 // controller works with no less than 15 V so give error code if voltage is too low

// walk assist
#define WALK_ASSIST_THRESHOLD_SPEED_X10         80 // 80 -> 8.0 kph, this is the maximum speed limit in which walk assist can be activated

// cruise
#define CRUISE_THRESHOLD_SPEED_X10              90 // 90 -> 9.0 kph, this is the minimum speed limit in which cruise can be activated

// optional ADC channel function
#define NOT_IN_USE                              0
#define TEMPERATURE_CONTROL                     1
#define THROTTLE_CONTROL                        2

#endif /* COMMON_COMMON_H_ */
