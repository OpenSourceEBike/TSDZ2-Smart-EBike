/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#include "stm8s.h"
#include "stm8s_gpio.h"
#include "pins.h"
#include "torque_sensor.h"

void torque_sensor_init (void)
{
  GPIO_Init(TORQUE_SENSOR_EXCITATION__PORT, TORQUE_SENSOR_EXCITATION__PIN, GPIO_MODE_OUT_OD_HIZ_FAST);
}