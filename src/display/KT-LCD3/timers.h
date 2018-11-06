/*
 * LCD3 firmware
 *
 * Copyright (C) Casainho, 2018.
 *
 * Released under the GPL License, Version 3
 */

#ifndef _TIMERS_H_
#define _TIMERS_H_

#include "stm8s.h"
#include "stm8s_tim1.h"
#include "stm8s_tim3.h"

void timer3_init (void);
void timer1_init (void);

#endif /* _TIMERS_H_ */
