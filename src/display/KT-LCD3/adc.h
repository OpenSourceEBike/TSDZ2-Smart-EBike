/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _ADC_H
#define _ADC_H

#define ADC1_CHANNEL_BATTERY_VOLTAGE  ADC1_CHANNEL_8

void adc_init (void);
uint16_t ui16_adc_read_battery_voltage_10b (void);

#endif /* _ADC_H */
