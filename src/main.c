#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>
#include <string.h>
#include "bootloader.h"
#include "timer.h"
#include "config.h"
#include "uart.h"
#include "lcd.h"

void putchar(char c) { UART0_send_char(c); }

void InitPinADC (unsigned char portno, unsigned char pinno)
{
	unsigned char mask;
	
	mask=1<<pinno;

	SFRPAGE = 0x00;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
	 return ((ADC_at_Pin(pin)*VDD)/0b_0011_1111_1111_1111);
}

// Map a voltage (0 to VDD) to a byte (0 to 255)
unsigned char volt_to_byte(float v)
{
	int val = (int)(v / VDD * 255.0f);
	if (val < 0)   val = 0;
	if (val > 255) val = 255;
	return (unsigned char)val;
}

// ---- Main ----
void main (){
	float joystick_x = 0;
	float joystick_y = 0;
	unsigned char x_byte, y_byte;

	int debug = 0;

	init_pin_input();
	TIMER0_Init();
	TIMER2_Init();
	UART0_init();

	InitPinADC(1, 4); // Configure Joystick_Y as analog input
	InitPinADC(1, 5); // Configure Joystick_X as analog input
	InitADC();

	
	LCD_4BIT();
	LCDprint("Hello", 1, 1);

	printf("start\r\n");
	while(1){
		joystick_x = Volts_at_Pin(JOYSTICK_X);
		joystick_y = Volts_at_Pin(JOYSTICK_Y);

		x_byte = volt_to_byte(joystick_x);
		y_byte = volt_to_byte(joystick_y);

		IR_Send(IR_CMD_JOYSTICK_X, x_byte, IR_ADDR);
		while (fsm_state == FSM_IDLE);
		while (fsm_state != FSM_IDLE);

		IR_Send(IR_CMD_JOYSTICK_Y, y_byte, IR_ADDR);
		while (fsm_state == FSM_IDLE);
		while (fsm_state != FSM_IDLE);

		/*
		IR_Send(IR_CMD_JOYSTICK_Y, y_byte, IR_ADDR);
		while (fsm_state == FSM_IDLE);
		while (fsm_state != FSM_IDLE);

		IR_Send(IR_CMD_JOYSTICK_X, x_byte, IR_ADDR);
		while (fsm_state == FSM_IDLE);
		while (fsm_state != FSM_IDLE);
		*/

		printf("X=%3u Y=%3u\r\n", (unsigned int)x_byte, (unsigned int)y_byte);
	}
}
