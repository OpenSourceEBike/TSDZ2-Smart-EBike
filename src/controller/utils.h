/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _UTILS_H
#define _UTILS_H

#include "main.h"

int32_t map (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t map_inverse (int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
uint8_t ui8_max (uint8_t value_a, uint8_t value_b);
uint8_t ui8_min (uint8_t value_a, uint8_t value_b);
void ui8_filter(uint8_t *ui8_new_value, uint8_t *ui8_old_value, uint8_t ui8_alpha);
void ui16_filter(uint16_t *ui16_new_value, uint16_t *ui16_old_value, uint16_t ui16_alpha);
void ui8_limit_max (uint8_t *ui8_p_value, uint8_t ui8_max_value);
void crc16(uint8_t ui8_data, uint16_t* ui16_crc);

#endif /* _UTILS_H */
