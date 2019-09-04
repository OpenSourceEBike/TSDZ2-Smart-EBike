/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho and Leon, 2019.
 *
 * Released under the GPL License, Version 3
 */

#ifndef COMMON_COMMON_H_
#define COMMON_COMMON_H_


// riding modes
#define OFF_MODE                                  0
#define POWER_ASSIST_MODE                         1
#define TORQUE_ASSIST_MODE                        2
#define CADENCE_ASSIST_MODE                       3
#define eMTB_ASSIST_MODE                          4
#define WALK_ASSIST_MODE                          5
#define CRUISE_MODE                               6
#define CADENCE_SENSOR_CALIBRATION_MODE           7


// error codes
#define NO_ERROR                                  0
#define ERROR_MOTOR_BLOCKED                       1
#define ERROR_TORQUE_SENSOR                       2
#define ERROR_BRAKE_APPLIED_DURING_POWER_ON       3
#define ERROR_THROTTLE_APPLIED_DURING_POWER_ON    4
#define ERROR_NO_SPEED_SENSOR_DETECTED            5
#define ERROR_LOW_CONTROLLER_VOLTAGE              6   // controller works with no less than 15 V so give error code if voltage is too low
#define ERROR_CADENCE_SENSOR_CALIBRATION          7


// walk assist
#define WALK_ASSIST_THRESHOLD_SPEED_X10           80  // 80 -> 8.0 kph, this is the maximum speed limit from which walk assist can be activated


// cruise
#define CRUISE_THRESHOLD_SPEED_X10                90  // 90 -> 9.0 kph, this is the minimum speed limit from which cruise can be activated


// optional ADC function
#define NOT_IN_USE                                0
#define TEMPERATURE_CONTROL                       1
#define THROTTLE_CONTROL                          2


// cadence sensor
#define STANDARD_MODE                             0
#define ADVANCED_MODE                             1
#define CALIBRATION_MODE                          2


int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t map_inverse (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
uint8_t ui8_max (uint8_t value_a, uint8_t value_b);
uint8_t ui8_min (uint8_t value_a, uint8_t value_b);
uint32_t filter(uint32_t ui32_new_value, uint32_t ui32_old_value, uint8_t ui8_alpha);
void ui8_limit_max (uint8_t *ui8_p_value, uint8_t ui8_max_value);
void crc16(uint8_t ui8_data, uint16_t* ui16_crc);


#endif /* COMMON_COMMON_H_ */
