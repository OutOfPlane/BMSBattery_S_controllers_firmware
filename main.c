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
uint8_t motor_dir = 0; //0 fwd, 1 rev
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
//  		   0°  60° 120° 180° 240° 300° #255 = 360°
//             0   42   85  127  170  212
//						INV  1  2    3    4   5   6    INV
uint8_t hall_lookup[] = {0, 0, 170, 212, 85, 42, 127, 0};
const uint8_t hall_order[] = {1, 5, 4, 6, 2, 3};
uint32_t last_hall_update = 0;
uint8_t last_hall_state = 0;
uint32_t hall_dt = 0;


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
	if(byte_avail_at_position()) //uart has data
	{
		ui8_rx = uart_get_buffered();
		if ((ui8_rx & PKT_MASK) == PKT_PEDAL)
		{
			// pedal packet
			uart_throttle = (ui8_rx & ~PKT_MASK) << 1; // map 6 bit value to 7 bit value
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
	if(last_hall_state != new_hall_state)
	{
		hall_dt = _micros() - last_hall_update;
		last_hall_update += hall_dt;
		if(hall_dt > 16000)
		{
			hall_dt = 0; // no speed available
		}
		last_hall_state = new_hall_state;
	}
	return new_hall_state;
}
uint8_t hall_offset = 0;
uint8_t getHallAngle(void)
{
	return hall_lookup[getHallState()] - hall_offset;
}

const int8_t sine_array[] = {
	0, 3, 6, 9, 12, 15, 18, 22, 25, 28, 31, 34, 37, 40, 43, 46,
	49, 52, 55, 57, 60, 63, 66, 68, 71, 74, 76, 79, 81, 84, 86,
	88, 90, 93, 95, 97, 99, 101, 103, 104, 106, 108, 109, 111, 113, 114, 115,
	117, 118, 119, 120, 121, 122, 123, 123, 124, 125, 125, 126, 126, 126, 126, 126, 127};

int8_t sin(uint8_t angle)
{
	if (angle < 64)
	{
		return sine_array[angle];
	}
	else if (angle < 128)
	{
		return sine_array[127 - angle];
	}
	else if (angle < 192)
	{
		return -sine_array[angle - 128];
	}
	else
	{
		return -sine_array[255 - angle];
	}
}

int8_t cos(uint8_t angle)
{
	return sin(angle + 64);
}

void setPWMAngleQ(int8_t val, uint8_t angle)
{
	// val, sin and cos is ranging from -127 to 127

	int16_t sin_a = sin(angle);
	int16_t cos_a = cos(angle);

	// Inverse park transform
	int16_t alpha = -sin_a * (int16_t)val; // -sin(angle) * Uq;
	int16_t beta = cos_a * (int16_t)val;   //  cos(angle) * Uq;
	beta /= 8;							   // 7/8 = 0.875 is close enough to sqrt(3)/2 = 0.866
	beta *= 7;
	// values alpha and beta now range +- 16384

	// Clarke transform
	uint16_t Ua = alpha + (INT16_MAX / 2);
	uint16_t Ub = -(alpha >> 1) + beta + (INT16_MAX / 2);
	uint16_t Uc = -(alpha >> 1) - beta + (INT16_MAX / 2);

	Ua = Ua >> 7;
	Ub = Ub >> 7;
	Uc = Uc >> 7;
	setPWM(Ua, Ub, Uc);
}

void setPWMAngleD(int8_t val, uint8_t angle)
{
	// val, sin and cos is ranging from -127 to 127

	int16_t sin_a = sin(angle);
	int16_t cos_a = cos(angle);

	// Inverse park transform
	int16_t alpha = cos_a * (int16_t)val; // -sin(angle) * Uq;
	int16_t beta = sin_a * (int16_t)val;   //  cos(angle) * Uq;
	beta /= 8;							   // 7/8 = 0.875 is close enough to sqrt(3)/2 = 0.866
	beta *= 7;
	// values alpha and beta now range +- 16384

	// Clarke transform
	uint16_t Ua = alpha + (INT16_MAX / 2);
	uint16_t Ub = -(alpha >> 1) + beta + (INT16_MAX / 2);
	uint16_t Uc = -(alpha >> 1) - beta + (INT16_MAX / 2);

	Ua = Ua >> 7;
	Ub = Ub >> 7;
	Uc = Uc >> 7;
	setPWM(Ua, Ub, Uc);
}



int main(void)
{
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

	setPWM(127, 127, 127);

	enablePWM();

	uint8_t curr_angle = 0;
	uint16_t cal_revs = 0;
	uint8_t prev_hall = getHallAngle();
	uint8_t curr_hall = prev_hall;
	uint16_t hall_lookup_sum[8] = {0};
	uint16_t hall_lookup_cnt[8] = {0};

	while (1)
	{
		uartInputHandler();
		if (_micros() - lastmicros > 100000ul)
		{
			lastmicros = _micros();
			uartWrite(adc_IBat_filt>>8);
			uartWrite(adc_IBat_filt & 0xFF);

			uartWrite(hall_dt>>8);
			uartWrite(hall_dt&0xFF);
			
		}

		if(motor_cmd == CMD_CAL)
		{
			motor_cmd = 0;
			cal_revs = 0;
			for (size_t i = 0; i < 6; i++)
			{
				uint8_t j = hall_order[i];
				hall_lookup_cnt[j] = 0;
				hall_lookup_sum[j] = 0;
			}
			hall_offset = 0;
			state = MOTOR_CALIBRATING;
		}

		if (state == MOTOR_CALIBRATING)
		{
			setPWMAngleQ(40, getHallAngle());

			if (_micros() - motormicros > 1000000)
			{
				motormicros = _micros();
				hall_offset++;
				if(hall_offset == 64){
					state = MOTOR_READY;
				}
			}
			// 	curr_hall = getHallState();
			// 	setPWMAngleD(60, cal_revs);
			// 	cal_revs++;

			// 	if (prev_hall != curr_hall)
			// 	{
			// 		//we hit a switchpoint
			// 		hall_lookup_sum[curr_hall] += (cal_revs & 0xFF);
			// 		hall_lookup_cnt[curr_hall]++;
			// 	}

			// 	if (cal_revs >= (CAL_REVOLUTIONS * 255))
			// 	{
			// 		disablePWM();
			// 		for (size_t i = 0; i < 6; i++)
			// 		{
			// 			uint8_t j = hall_order[i];
			// 			if(hall_lookup_cnt[j])
			// 				hall_lookup_sum[j] /= hall_lookup_cnt[j];
			// 		}
			// 		uint8_t hall_delta, idx0, idx1;

			// 		for (size_t i = 0; i < 6; i++)
			// 		{
			// 			idx0 = hall_order[i];
			// 			idx1 = hall_order[(i+1)%6];
			// 			hall_delta = hall_lookup_sum[idx1] - hall_lookup_sum[idx0];
			// 			if(hall_delta < 128)
			// 			{
			// 				//normal order
			// 				hall_lookup[hall_order[idx0]] = hall_lookup_sum[hall_order[idx0]] + (hall_delta >> 1);
			// 			}else{
			// 				//reversed order
			// 				hall_delta = hall_lookup_sum[idx0] - hall_lookup_sum[idx1];
			// 				hall_lookup[hall_order[idx1]] = hall_lookup_sum[hall_order[idx1]] + (hall_delta >> 1);
			// 			}
			// 		}
					
			// 		state = MOTOR_READY;
			// 		enablePWM();
			// 	}
			// 	prev_hall = curr_hall;
			// }
		}
		if (state == MOTOR_READY)
		{
			//do the pwm controller as fast as possible
			if(motor_dir == 0)
			{
				setPWMAngleQ(-(uart_throttle), getHallAngle());
			}else{
				setPWMAngleQ(uart_throttle, getHallAngle());
			}
		}
		// reset watchdog
		IWDG->KR = IWDG_KEY_REFRESH; // we are still alive
	} // end of while(1) loop
}
