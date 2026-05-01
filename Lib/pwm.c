/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho,Björn Schmidt 2015, 2106, 2017, 2019
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s_gpio.h"
#include "stm8s_tim1.h"
#include "motor.h"
#include "gpio.h"
#include "motor.h"
#include "pwm.h"
#include "config.h"



void pwm_init(void)
{
	// TIM1 Peripheral Configuration
	TIM1_DeInit();

	TIM1_TimeBaseInit(0, // TIM1_Prescaler = 0
					  TIM1_COUNTERMODE_CENTERALIGNED1,
					  PWM_PERIOD,
					  1); // will fire the TIM1_IT_UPDATE at every PWM period cycle

	// #define DISABLE_PWM_CHANNELS_1_3

	TIM1_OC1Init(TIM1_OCMODE_PWM1,
#ifdef DISABLE_PWM_CHANNELS_1_3
				 TIM1_OUTPUTSTATE_DISABLE,
				 TIM1_OUTPUTNSTATE_DISABLE,
#else
				 TIM1_OUTPUTSTATE_ENABLE,
				 TIM1_OUTPUTNSTATE_ENABLE,
#endif
				 0, // initial duty_cycle value
				 TIM1_OCPOLARITY_HIGH,
				 TIM1_OCNPOLARITY_LOW,
				 TIM1_OCIDLESTATE_RESET,
				 TIM1_OCNIDLESTATE_SET);

	TIM1_OC2Init(TIM1_OCMODE_PWM1,
				 TIM1_OUTPUTSTATE_ENABLE,
				 TIM1_OUTPUTNSTATE_ENABLE,
				 0, // initial duty_cycle value
				 TIM1_OCPOLARITY_HIGH,
				 TIM1_OCNPOLARITY_LOW,
				 TIM1_OCIDLESTATE_RESET,
				 TIM1_OCNIDLESTATE_SET);

	TIM1_OC3Init(TIM1_OCMODE_PWM1,
#ifdef DISABLE_PWM_CHANNELS_1_3
				 TIM1_OUTPUTSTATE_DISABLE,
				 TIM1_OUTPUTNSTATE_DISABLE,
#else
				 TIM1_OUTPUTSTATE_ENABLE,
				 TIM1_OUTPUTNSTATE_ENABLE,
#endif
				 0, // initial duty_cycle value
				 TIM1_OCPOLARITY_HIGH,
				 TIM1_OCNPOLARITY_LOW,
				 TIM1_OCIDLESTATE_RESET,
				 TIM1_OCNIDLESTATE_SET);

	TIM1_OC1PreloadConfig(ENABLE);
	TIM1_OC2PreloadConfig(ENABLE);
	TIM1_OC3PreloadConfig(ENABLE);

	// break, dead time and lock configuration
	TIM1_BDTRConfig(TIM1_OSSISTATE_ENABLE,
					TIM1_LOCKLEVEL_OFF,
					// hardware nees a dead time of 1us
					16, // DTG = 0; dead time in 62.5 ns steps; 1us/62.5ns = 16
					TIM1_BREAK_DISABLE,
					TIM1_BREAKPOLARITY_LOW,
					TIM1_AUTOMATICOUTPUT_DISABLE);

	TIM1_ITConfig(TIM1_IT_UPDATE, ENABLE);

	TIM1_Cmd(ENABLE);			  // TIM1 counter enable
	TIM1_CtrlPWMOutputs(DISABLE); // main Output disable for start up
}
