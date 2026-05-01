/*
 * BMSBattery S series motor controllers firmware
 *
 * Copyright (C) Casainho, 2017.
 *
 * Released under the GPL License, Version 3
 */

#include <stdint.h>
#include <stdio.h>
#include "stm8s.h"
#include "gpio.h"
#include "stm8s_itc.h"
#include "interrupts.h"
#include "eeprom.h"
#include "motor.h"
#include "uart.h"
#include "adc.h"
#include "timers.h"
#include "pwm.h"
// #include "update_setpoint.h"
#include "config.h"
#include "TinyFOC.h"

// wiring:
/*
Red: Battery +
Black: Battery -
Blue: Ignition

Yellow: Display TX
Green: Display RX

*/

/////////////////////////////////////////////////////////////////////////////////////////////
//// Local Variables
volatile uint16_t microsh = 0;

//tinyFOC storage classes
FOCMotor bldcMotor;
FOCDriver PWMDriver;
Sensor hallSensor;
CurrentSense dcCurrentSense;


/////////////////////////////////////////////////////////////////////////////////////////////
//// Functions prototypes

// main -- start of firmware and main loop
int main(void);

// With SDCC, interrupt service routine function prototypes must be placed in the file that contains main ()
// in order for an vector for the interrupt to be placed in the the interrupt vector space.  It's acceptable
// to place the function prototype in a header file as long as the header file is included in the file that
// contains main ().  SDCC will not generate any warnings or errors if this is not done, but the vector will
// not be in place so the ISR will not be executed when the interrupt occurs.

// Calling a function from interrupt not always works, SDCC manual says to avoid it. Maybe the best is to put
// all the code inside the interrupt

// Local VS global variables
// Sometimes I got the following error when compiling the firmware: motor.asm:750: Error: <r> relocation error
// and the solution was to avoid using local variables and define them as global instead

// UART2 receivce handler
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);

// Timer1/PWM period interrupt
void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM1_UPD_OVF_TRG_BRK_IRQHANDLER)
{
	adc_trigger();
	// clear the interrupt pending bit for TIM1
	TIM1_ClearITPendingBit(TIM1_IT_UPDATE);
}

void TIM2_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM2_UPD_OVF_TRG_BRK_IRQHANDLER)
{
	microsh++;
	// clear the interrupt pending bit for TIM2
	TIM2_ClearITPendingBit(TIM2_IT_UPDATE);
}

uint8_t ui8_rx_buffer[13];
uint8_t ui8_rx_buffer_counter = 0;
uint8_t ui8_UARTCounter = 0;
uint16_t ui16_uart_throttle = 0;

#define PKT_MASK 0xC0
#define PKT_PEDAL 0x00
#define PKT_BRAKE 0x40

void uartInputHandler(void)
{
	uart_fill_rx_packet_buffer(ui8_rx_buffer, 1, &ui8_UARTCounter);
	if (ui8_UARTCounter)
	{
		if ((ui8_rx_buffer[0] & PKT_MASK) == PKT_PEDAL)
		{
			// pedal packet
			ui16_uart_throttle = (ui8_rx_buffer[0] & ~PKT_MASK) << 2; // map 6 bit value to 8 bit value
		}
		else if ((ui8_rx_buffer[0] & PKT_MASK) == PKT_BRAKE)
		{
			// brake packet
			// if(ui8_rx_buffer[0] & ~PKT_MASK > 0) {
			// 	brake_set();
			// } else {
			// 	brake_clear();
			// }
		}
	}
	ui8_UARTCounter = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////Implement TinyFOC library Stubs//////////////////////////////////
uint32_t _micros(void)
{
	return ((uint32_t)microsh) << 16 | (((uint32_t)TIM2->CNTRH) << 8) | ((uint32_t)TIM2->CNTRL);
}

void FOCDriver_ll_setPwm(void *params, FIXP dcA, FIXP dcB, FIXP dcC)
{
	// set final duty_cycle value
	TIM1_SetCompare1((uint16_t)FIX_MUL(dcA, PWM_PERIOD));
	TIM1_SetCompare2((uint16_t)FIX_MUL(dcB, PWM_PERIOD));
	TIM1_SetCompare3((uint16_t)FIX_MUL(dcC, PWM_PERIOD));
}

bool FOCDriver_ll_init(void *params)
{
	pwm_init();
	return true;
}

void FOCDriver_ll_enable(void *params)
{
	TIM1_CtrlPWMOutputs(ENABLE);
}

void FOCDriver_ll_disable(void *params)
{
	TIM1_CtrlPWMOutputs(DISABLE);
}

void Sensor_ll_init(void *param)
{
	hall_sensor_init();
}

void Sensor_ll_read(void *param)
{
	Sensor *sns = (Sensor *)param;
	sns->new_hall_state = (GPIO_ReadInputData(HALL_SENSORS__PORT) & (HALL_SENSORS_MASK));
}

bool CurrentSense_ll_init(void *param)
{
	return true;
}

void CurrentSense_ll_readcurrents(void *params, FIXP *phA, FIXP *phB, FIXP *phC)
{
	// BLDCState *state = (BLDCState *)params;
	// *phA = FIX_FROM_FLOAT(state->current_a);
	// *phB = FIX_FROM_FLOAT(state->current_b);
	// *phC = FIX_FROM_FLOAT(state->current_c);
}

void dbg_write(char val)
{
	printf("%c", val);
}

void dbg_newline(void)
{
	printf("\n");
}

void dbg_print(char* msg)
{
	printf(msg);
}

void dbg_print_f(FIXP val, int digits)
{
	printf("%.*f", digits, FIX_TO_FLOAT(val));
}

int dbg_read(char* val, int len)
{
	return 0;
}

int main(void)
{
	// set clock at the max 16MHz
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	gpio_init();
	uart_init();
	debug_pin_init();
	light_pin_init();
	timer2_init();
	eeprom_init();
	adc_init();


	//setup tinyFOC
	FOCMotor_load_default(&bldcMotor);
    bldcMotor.pole_pairs = 15;
    bldcMotor.KV_rating = FIX_FROM_FLOAT(10.0f);
    bldcMotor.axis_inductance.d = FIX_FROM_FLOAT(0.005f);
    bldcMotor.axis_inductance.q = FIX_FROM_FLOAT(0.005f);
    bldcMotor.phase_resistance = FIX_FROM_FLOAT(0.2f);

    FOCDriver_load_default(&PWMDriver);
    
    CurrentSense_load_default(&dcCurrentSense);
    CurrentSense_linkDriver(&dcCurrentSense, &PWMDriver);
    

    
    Sensor_load_default(&hallSensor);
    hallSensor.pp = bldcMotor.pole_pairs;
    hallSensor.params = &hallSensor; //needed to write back the current hall state

    FOCMotor_linkCurrentSense(&bldcMotor, &dcCurrentSense);
    FOCMotor_linkDriver(&bldcMotor, &PWMDriver);
    FOCMotor_linkSensor(&bldcMotor, &hallSensor);
    bldcMotor.sensor_direction = Direction_UNKNOWN;
    bldcMotor.zero_electric_angle = NOT_SET;
    bldcMotor.controller = MotionControlType_velocity;
    bldcMotor.torque_controller = TorqueControlType_voltage;

    Sensor_init(&hallSensor);
    FOCDriver_init(&PWMDriver);
    FOCMotor_init(&bldcMotor);
    CurrentSense_init(&dcCurrentSense);
    FOCMotor_updateVoltageLimit(&bldcMotor, FIX_FROM_FLOAT(8.0f));
    FOCMotor_updateVelocityLimit(&bldcMotor, FIX_FROM_FLOAT(100.0f));
    FOCMotor_initFOC(&bldcMotor);

    FOCMotor_move(&bldcMotor, FIX_FROM_FLOAT(5.0f)); // Move to 5 rad/s


	enableInterrupts();

	watchdog_init(); // init watchdog after enabling interrupt to have fast loop running already

#ifdef DIAGNOSTICS
	printf("System initialized\r\n");
#endif

	while (1)
	{
		FOCMotor_loopFOC(&bldcMotor);
		uart_send_if_avail();
		// reset watchdog
		IWDG->KR = IWDG_KEY_REFRESH; // we are still alive
	} // end of while(1) loop
}
