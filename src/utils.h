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

typedef struct pi_controller_state
{
  uint8_t ui8_current_value;
  uint8_t ui8_target_value;
  uint8_t ui8_controller_output_value;
  uint8_t ui8_kp_dividend;
  uint8_t ui8_kp_divisor;
  uint8_t ui8_ki_dividend;
  uint8_t ui8_ki_divisor;
  int16_t i16_i_term;
} struct_pi_controller_state;

int32_t map(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
int32_t map_inverse(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max);
uint8_t ui8_max(uint8_t value_a, uint8_t value_b);
uint8_t ui8_min(uint8_t value_a, uint8_t value_b);
uint16_t ui16_min(uint16_t value_a, uint16_t value_b);
void pi_controller(struct_pi_controller_state *pi_controller_state);
void pi_controller_reset(struct_pi_controller_state *pi_controller);
void crc16(uint8_t ui8_data, uint16_t* ui16_crc);
void ui8_limit_max(uint8_t *ui8_p_value, uint8_t ui8_max_value);
void ui16_limit_max(uint16_t *ui16_p_value, uint16_t ui16_max_value);

#endif /* _UTILS_H */
