#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>
#include <string.h>
#include "bootloader.h"
#include "timer.h"
#include "ir_tx.h"
#include "ir_rx.h"
#include "config.h"
#include "uart.h"
#include "lcd.h"
#include "lcd_fsm.h"

void putchar(char c) { UART0_send_char(c); }

// ---- IR loopback debug --------------------------------------------------
static void IR_debug(void)
{
    IR_Frame_t f;
    //send_ir_packet((uint8_t)IR_CMD_START, (uint16_t)0x0000, (uint8_t)IR_ADDR1);
    while (IR_RX_get(&f))
        if (f.addr == IR_ADDR2)
            printf("cmd=%u val=0x%04X addr=0x%X\r\n",
                   (unsigned int)f.cmd,
                   (unsigned int)f.val,
                   (unsigned int)f.addr);
}

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
	lcd_state_t prev_lcd_state = LCD_NUM_STATES;

	init_pin_input();
	TIMER0_Init();
	TIMER2_Init();
	UART0_init();
	UART1_init();
	IR_RX_init();

	InitPinADC(1, 4); // Configure Joystick_Y as analog input
	InitPinADC(1, 5); // Configure Joystick_X as analog input
	InitADC();

	LCD_4BIT();
	LCD_FSM_init();

	printf("start\r\n");
	while(1){
		IR_debug();
		/*
		joystick_x = Volts_at_Pin(JOYSTICK_X);
		joystick_y = Volts_at_Pin(JOYSTICK_Y);

		x_byte = volt_to_byte(joystick_x);
		y_byte = volt_to_byte(joystick_y);

		LCD_FSM_update(x_byte, y_byte);

		// On transition into a running/pause state, send IR command
		if (lcd_state != prev_lcd_state) {
			if (lcd_state == LCD_S7) {
				// Paused
				IR_Send(IR_CMD_PAUSE, 0xFF, IR_ADDR);
				while (fsm_state == FSM_IDLE);
				while (fsm_state != FSM_IDLE);
			} else if (lcd_state == LCD_S5) {
				// Auto mode: send mode, path, then start
				IR_Send(IR_CMD_MODE, active_mode, IR_ADDR);
				while (fsm_state == FSM_IDLE);
				while (fsm_state != FSM_IDLE);

				IR_Send(IR_CMD_PATH, active_path, IR_ADDR);
				while (fsm_state == FSM_IDLE);
				while (fsm_state != FSM_IDLE);

				IR_Send(IR_CMD_START, 0xFF, IR_ADDR);
				while (fsm_state == FSM_IDLE);
				while (fsm_state != FSM_IDLE);
			} else if (lcd_state == LCD_S6) {
				// Remote mode: send mode, then start
				IR_Send(IR_CMD_MODE, active_mode, IR_ADDR);
				while (fsm_state == FSM_IDLE);
				while (fsm_state != FSM_IDLE);

				IR_Send(IR_CMD_START, 0xFF, IR_ADDR);
				while (fsm_state == FSM_IDLE);
				while (fsm_state != FSM_IDLE);
			}
			prev_lcd_state = lcd_state;
		}

		// Send joystick only in remote mode
		if (lcd_state == LCD_S6) {
			IR_Send(IR_CMD_JOYSTICK_X, x_byte, IR_ADDR);
			while (fsm_state == FSM_IDLE);
			while (fsm_state != FSM_IDLE);

			IR_Send(IR_CMD_JOYSTICK_Y, y_byte, IR_ADDR);
			while (fsm_state == FSM_IDLE);
			while (fsm_state != FSM_IDLE);
		}
			*/
		
	}
}
