/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s_gpio.h"
#include "stm8s_tim1.h"
#include "stm8s.h"
#include "stm8s_tim2.h"
#include "motor.h"
#include "gpio.h"


void timer2_init (void)
{
  // TIM2 Peripheral Configuration
  TIM2_DeInit();

  TIM2_TimeBaseInit(TIM2_PRESCALER_16, 0xFFFF); //1us timer with 65ms period
  TIM2_ITConfig(TIM2_IT_UPDATE,ENABLE);

  TIM2_Cmd(ENABLE); // TIM2 counter enable

}

