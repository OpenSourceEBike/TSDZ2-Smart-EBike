/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _ADC_H
#define _ADC_H

#include "main.h"

// for AIN6: 0x53E0 + 2*6 = 0x53E8
#define UI8_ADC_BATTERY_VOLTAGE 			(*(uint8_t*)(0x53EC)) // AIN6
#define UI8_ADC_BATTERY_CURRENT				(*(uint8_t*)(0x53EA)) // AIN5
#define UI8_ADC_THROTTLE 				      (*(uint8_t*)(0x53EE)) // AIN7
#define UI8_ADC_TORQUE_SENSOR         (*(uint8_t*)(0x53E8)) // AIN4


#define UI16_ADC_10_BIT_BATTERY_VOLTAGE 	(((*(uint8_t*)(0x53EC)) << 2) | (*(uint8_t*)(0x53ED)))
#define UI16_ADC_10_BIT_BATTERY_CURRENT	  (((*(uint8_t*)(0x53EA)) << 2) | (*(uint8_t*)(0x53EB)))
#define UI16_ADC_10_BIT_THROTTLE 				  (((*(uint8_t*)(0x53EE)) << 2) | (*(uint8_t*)(0x53EF)))
#define UI16_ADC_10_BIT_TORQUE_SENSOR     (((*(uint8_t*)(0x53E8)) << 2) | (*(uint8_t*)(0x53E9)))


void adc_init (void);
uint16_t ui16_adc_read_battery_current_10b (void);
uint16_t ui16_adc_read_battery_voltage_10b (void);
uint16_t ui16_adc_read_torque_sensor_10b (void);
uint16_t ui16_adc_read_throttle_10b (void);

#endif /* _ADC_H */