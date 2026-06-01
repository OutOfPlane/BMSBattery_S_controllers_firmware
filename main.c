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
volatile uint16_t adc_VBat = 0;
volatile uint16_t adc_IBat = 0;
volatile uint16_t adc_IBat_filt = 0;
volatile uint16_t adc_throttle = 0;

uint32_t lastmicros;
uint32_t motormicros;

enum motorstate
{
	MOTOR_OFF,
	MOTOR_CALIBRATING,
	MOTOR_READY
};
enum motorstate state = MOTOR_READY;
uint8_t motor_dir = 0; // 0 fwd, 1 rev
#define CAL_REVOLUTIONS 20

uint8_t ui8_rx;
uint8_t uart_throttle = 0;
uint8_t motor_cmd = 0;
#define PKT_MASK 0xC0
#define PKT_PEDAL 0x00
#define PKT_CMD 0x40
#define CMD_CAL 0x01
#define CMD_FWD 0x02
#define CMD_REV 0x03

// hall goes   1    5    4    6    2    3
//           001  101  100  110  010  011
//  		   0°  60° 120° 180° 240° 300°
const uint8_t hall_sector[] = {6, 4, 2, 3, 0, 5, 1, 6};
const uint8_t commutation_phases_en[] = {5, 3, 6, 5, 3, 6, 0};
const uint8_t commutation_phases_fwd[] = {4, 2, 2, 1, 1, 4, 0};
const uint8_t commutation_phases_rev[] = {1, 1, 4, 4, 2, 2, 0};
uint32_t last_hall_update = 0;
uint8_t last_hall_state = 0;
uint32_t hall_dt = 0;
uint8_t new_hall_cnt = 0;

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
	adc_VBat = ((uint16_t)ADC1->DB9RH) << 2 | ADC1->DB9RL;
	adc_IBat = ((uint16_t)ADC1->DB6RH) << 2 | ADC1->DB6RL;
	adc_IBat_filt = ((uint16_t)ADC1->DB8RH) << 2 | ADC1->DB8RL;
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

void uartInputHandler(void)
{
	if (byte_avail_at_position()) // uart has data
	{
		ui8_rx = uart_get_buffered();
		if ((ui8_rx & PKT_MASK) == PKT_PEDAL)
		{
			// pedal packet
			uart_throttle = (ui8_rx & ~PKT_MASK) << 2; // map 6 bit value to 7 bit value
		}
		else if ((ui8_rx & PKT_MASK) == PKT_CMD)
		{
			// command packet
			motor_cmd = (ui8_rx & ~PKT_MASK);
		}
	}
}

void uartWrite(uint8_t c)
{
	// Write a character to the UART2
	UART2_SendData8(c);

	// Loop until the end of transmission
	while (UART2_GetFlagStatus(UART2_FLAG_TXE) == RESET)
		;
}

/////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////Implement TinyFOC library Stubs//////////////////////////////////
uint32_t _micros(void)
{
	uint32_t tmp = 0;
	disableInterrupts();
	tmp = ((uint32_t)microsh) << 16 | (((uint32_t)TIM2->CNTRH) << 8) | ((uint32_t)TIM2->CNTRL);
	enableInterrupts();
	return tmp;
}

void setPWM(uint8_t dcA, uint8_t dcB, uint8_t dcC)
{
	// set final duty_cycle value
	TIM1_SetCompare1(((uint32_t)dcA * PWM_PERIOD) >> 8);
	TIM1_SetCompare2(((uint32_t)dcB * PWM_PERIOD) >> 8);
	TIM1_SetCompare3(((uint32_t)dcC * PWM_PERIOD) >> 8);
}

void enablePWM(void)
{
	TIM1_CtrlPWMOutputs(ENABLE);
}

void disablePWM(void)
{
	TIM1_CtrlPWMOutputs(DISABLE);
}

uint8_t getHallState(void)
{
	uint8_t new_hall_state = (GPIO_ReadInputData(HALL_SENSORS__PORT) & (HALL_SENSORS_MASK));
	if (last_hall_state != new_hall_state)
	{
		new_hall_cnt++;
		if (new_hall_cnt == 10)
		{
			new_hall_cnt = 0;
			hall_dt = _micros() - last_hall_update;
			last_hall_update += hall_dt;
			if (hall_dt > 16000)
			{
				hall_dt = 0; // no speed available
			}
			last_hall_state = new_hall_state;
		}
	}
	else
	{
		new_hall_cnt = 0;
	}
	return new_hall_state;
}

void setPWMPower(uint8_t val, uint8_t dir, uint8_t hallstate)
{
	uint8_t sector, ch_en, ch_pwrd, chA, chB, chC;
	sector = hall_sector[hallstate];
	ch_en = commutation_phases_en[sector];
	chA = 0;
	chB = 0;
	chC = 0;

	if (dir == 0)
		ch_pwrd = commutation_phases_fwd[sector];
	else
		ch_pwrd = commutation_phases_rev[sector];

	if (ch_en & 4)
		TIM1_CCxCmd(TIM1_CHANNEL_1, ENABLE);
	else
		TIM1_CCxCmd(TIM1_CHANNEL_1, DISABLE);

	if (ch_en & 2)
		TIM1_CCxCmd(TIM1_CHANNEL_2, ENABLE);
	else
		TIM1_CCxCmd(TIM1_CHANNEL_2, DISABLE);

	if (ch_en & 1)
		TIM1_CCxCmd(TIM1_CHANNEL_3, ENABLE);
	else
		TIM1_CCxCmd(TIM1_CHANNEL_3, DISABLE);

	if (ch_pwrd & 4)
		chA = val;
	if (ch_pwrd & 2)
		chB = val;
	if (ch_pwrd & 1)
		chC = val;

	setPWM(chA, chB, chC);
}

int main(void)
{
	uint8_t hall_state = 0;
	// set clock at the max 16MHz
	CLK_HSIPrescalerConfig(CLK_PRESCALER_HSIDIV1);
	gpio_init();
	uart_init();
	timer2_init();
	eeprom_init();
	adc_init();
	pwm_init();
	hall_sensor_init();

	enableInterrupts();

	watchdog_init(); // init watchdog after enabling interrupt to have fast loop running already

#ifdef DIAGNOSTICS
	printf("System initialized\r\n");
#endif

	setPWM(0, 0, 0);

	enablePWM();

	while (1)
	{
		hall_state = getHallState();
		uartInputHandler();
		if (_micros() - lastmicros > 100000ul)
		{
			lastmicros = _micros();
			if (_micros() - last_hall_update > 16000)
			{
				hall_dt = 0; // no speed available
			}

			uartWrite(adc_IBat_filt >> 8);
			uartWrite(adc_IBat_filt & 0xFF);

			uartWrite(hall_dt >> 8);
			uartWrite(hall_dt & 0xFF);
		}

		if (state == MOTOR_READY)
		{
			// do the pwm controller as fast as possible
			if (uart_throttle < 20 && hall_dt == 0)
			{
				// activate hard brake
				setPWM(0, 0, 0);
				TIM1_CCxCmd(TIM1_CHANNEL_1, ENABLE);
				TIM1_CCxCmd(TIM1_CHANNEL_2, ENABLE);
				TIM1_CCxCmd(TIM1_CHANNEL_3, ENABLE);
			}
			else
			{
				if (motor_dir == 1)
				{
					setPWMPower((uart_throttle), 0, hall_state);
				}
				else
				{
					setPWMPower((uart_throttle), 1, hall_state);
				}
			}
		}
		// reset watchdog
		IWDG->KR = IWDG_KEY_REFRESH; // we are still alive
	} // end of while(1) loop
}
