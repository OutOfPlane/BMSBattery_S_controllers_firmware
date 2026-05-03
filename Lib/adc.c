/*
 * EGG OpenSource EBike firmware
 *
 * Copyright (C) Casainho, 2015, 2106, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "gpio.h"
#include "stm8s_adc1.h"
#include "adc.h"
#include "timers.h"

void adc_init(void) {   
    //init GPIO for the used ADC pins
    GPIO_Init(GPIOB,
            (THROTTLE__PIN || CURRENT_PHASE_B__PIN || CURRENT_MOTOR_TOTAL__PIN || REGEN_THROTTLE__PIN),
            GPIO_MODE_IN_FL_NO_IT);

    GPIO_Init(GPIOE,
            (CURRENT_MOTOR_TOTAL_FILTERED__PIN || BATTERY_VOLTAGE__PIN),
            GPIO_MODE_IN_FL_NO_IT);

    /*
    PB4 CH4 THROTTLE
    PB5 CH5 CURRENT_PHASE_B
    PB6 CH6 CURRENT_MOTOR_TOTAL
    PB7 CH7 REGEN_THROTTLE
    PE7 CH8 CURRENT_MOTOR_TOTAL_FILTERED
    PE6 CH9 BATTERY_VOLTAGE
    */

    //de-Init ADC peripheral
    ADC1_DeInit();

    //init ADC1 peripheral
    ADC1_Init(ADC1_CONVERSIONMODE_SINGLE,
            ADC1_CHANNEL_9,
            ADC1_PRESSEL_FCPU_D2,
            ADC1_EXTTRIG_TIM,
            DISABLE,
            ADC1_ALIGN_LEFT,
            (ADC1_SCHMITTTRIG_CHANNEL4 || ADC1_SCHMITTTRIG_CHANNEL5 || ADC1_SCHMITTTRIG_CHANNEL6 || ADC1_SCHMITTTRIG_CHANNEL7 || ADC1_SCHMITTTRIG_CHANNEL8 || ADC1_SCHMITTTRIG_CHANNEL9),
            DISABLE);

    ADC1_ScanModeCmd(ENABLE);
    ADC1_Cmd(ENABLE);        
}

inline void adc_trigger(void) //inline ?!
{
    ADC1->CSR &= 0x09; // in single scan mode channels are converted from AIN0 to AIN CSR[0:3]
    ADC1_StartConversion();
}

