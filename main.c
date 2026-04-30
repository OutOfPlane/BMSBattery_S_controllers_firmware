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
#include "stm8s_gpio.h"
#include "interrupts.h"
#include "stm8s_tim2.h"
#include "motor.h"
#include "main.h"
#include "uart.h"
#include "adc.h"
#include "brake.h"
#include "timers.h"
#include "pwm.h"
#include "PAS.h"
#include "SPEED.h"
// #include "update_setpoint.h"
#include "ACAsetPoint.h"
#include "config.h"
#include "display.h"
#include "display_kingmeter.h"
#include "ACAcontrollerState.h"
#include "BOdisplay.h"
#include "ACAeeprom.h"
#include "ACAcommons.h"

// wiring:
/*
Red: Battery +
Black: Battery -
Blue: Ignition

Yellow: Display TX
Green: Display RX

*/

// uint16_t ui16_LPF_angle_adjust = 0;
// uint16_t ui16_LPF_angle_adjust_temp = 0;

uint16_t ui16_log1 = 0;
uint8_t ui8_slowloop_flag = 0;
uint8_t ui8_veryslowloop_counter = 0;
uint8_t ui8_ultraslowloop_counter = 0;
uint16_t ui16_log2 = 0;
uint8_t ui8_log = 0;
uint8_t ui8_i = 0; // counter for ... next loop

float float_kv = 0;
float float_R = 0;
uint8_t a = 0; // loop counter

static int16_t i16_deziAmps;

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

// Brake signal interrupt
void EXTI_PORTA_IRQHandler(void) __interrupt(EXTI_PORTA_IRQHANDLER);
// Speed signal interrupt
void EXTI_PORTC_IRQHandler(void) __interrupt(EXTI_PORTC_IRQHANDLER);
// PAS signal interrupt
void EXTI_PORTD_IRQHandler(void) __interrupt(EXTI_PORTD_IRQHANDLER);

// Timer1/PWM period interrupt
void TIM1_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM1_UPD_OVF_TRG_BRK_IRQHANDLER);

// Timer2/slow control loop
void TIM2_UPD_OVF_TRG_BRK_IRQHandler(void) __interrupt(TIM2_UPD_OVF_TRG_BRK_IRQHANDLER);

// UART2 receivce handler
void UART2_IRQHandler(void) __interrupt(UART2_IRQHANDLER);

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
	if(ui8_UARTCounter){
		if((ui8_rx_buffer[0] & PKT_MASK) == PKT_PEDAL) {
			// pedal packet
			ui16_uart_throttle = (ui8_rx_buffer[0] & ~PKT_MASK) << 2; // map 6 bit value to 8 bit value
		} else if((ui8_rx_buffer[0] & PKT_MASK) == PKT_BRAKE) {
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

void updateRequestedTorque(void) {
	ui16_momentary_throttle = ui16_uart_throttle;
	ui16_sum_throttle = ui16_uart_throttle;
}
/////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////

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
	pwm_init();
	hall_sensor_init();
	adc_init();
	printf("PAS\r\n");
	
	enableInterrupts();

	watchdog_init(); // init watchdog after enabling interrupt to have fast loop running already

#if (SVM_TABLE == SVM)
	TIM1_SetCompare1(126 << 1);
	TIM1_SetCompare2(126 << 1);
	TIM1_SetCompare3(126 << 1);
#elif (SVM_TABLE == SINE) || (SVM_TABLE == SINE_SVM)
	TIM1_SetCompare1(126 << 2);
	TIM1_SetCompare2(126 << 2);
	TIM1_SetCompare3(126 << 2);
#endif

	hall_sensors_read_and_action(); // needed to start the motor
	// printf("Back in Main.c\n");

#ifdef DIAGNOSTICS
	printf("System initialized\r\n");
#endif
	while (1)
	{
		uart_send_if_avail();

	} // end of while(1) loop
}
