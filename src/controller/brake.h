/*
 * TongSheng TSDZ2 motor controller firmware/
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _BRAKE_H
#define _BRAKE_H

#include "main.h"

void brake_init (void);
BitStatus brake_is_set (void);

#endif /* _BRAKE_H */
