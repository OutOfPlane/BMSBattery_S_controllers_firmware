/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho,Björn Schmidt 2015, 2106, 2017, 2019
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s_iwdg.h"
#include "stm8s_gpio.h"
#include "stm8s_tim1.h"
#include "motor.h"
#include "gpio.h"
#include "pwm.h"
#include "config.h"
#include "adc.h"



/* Local only */
static uint8_t ui8_half_rotation_flag = 0;
static uint8_t ui8_foc_enable_flag = 0;
static uint8_t ui8_assumed_motor_position = 0;
static uint8_t ui8_motor_rotor_hall_position = 0; // in 360/256 degrees
static uint8_t ui8_sinetable_precalc = 0;
static uint8_t ui8_interpolation_start_position = 0;
static uint8_t ui8_interpolation_angle = 0;
static int8_t hall_sensors_last = 0;
static uint16_t ui16_ADC_iq_current_accumulated = 4096;

// Local except for diagnostics (main)
uint16_t ui16_PWM_cycles_counter = 0;
uint16_t ui16_PWM_cycles_counter_6 = 0;
uint16_t ui16_PWM_cycles_counter_total = 0;
uint16_t ui16_ADC_iq_current = 0; // main plus BO
int8_t hall_sensors; // main only; plus it's an int8, should be fine.
uint8_t ui8_possible_motor_state = 0;
uint8_t ui8_dynamic_motor_state = 0;
uint8_t ui8_position_correction_value = 127; // in 360/256 degrees
// Only for diagnostics
uint8_t uint8_t_60deg_pwm_cycles[6];
uint8_t uint8_t_hall_case[7];


// Motor->PWM (we call pwm; same context.)
uint8_t ui8_sinetable_position = 0; // in 360/256 degrees

// Slow loop -> motor (by the motor_slow_update_post)
static uint16_t BatteryCurrent;

// Motor-> slow loop (_pre)
static uint16_t motor_speed_erps;



void hall_sensor_init(void) {

	GPIO_Init(HALL_SENSORS__PORT,
			(GPIO_Pin_TypeDef) (HALL_SENSOR_A__PIN | HALL_SENSOR_B__PIN | HALL_SENSOR_C__PIN),
			GPIO_MODE_IN_FL_NO_IT);
}

void watchdog_init(void) {
	IWDG_Enable();
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_16);

	//  Timeout period
	//  The timeout period can be configured through the IWDG_PR and IWDG_RLR registers. It
	//  is determined by the following equation:
	//  T = 2 * T LSI * P * R
	//  where:
	//  T = Timeout period
	//  T LSI = 1/f LSI
	//  P = 2 (PR[2:0] + 2)
	//  R = RLR[7:0]+1
	//
	//  0.02 = 2 * (1 / 128000) * 16 * R
	// R = 0.02 * 128000/(2*16)
	//  R = 80
	//  R = 80 means a value of reload register = 79
	IWDG_SetReload(79);
	IWDG_ReloadCounter();
}

