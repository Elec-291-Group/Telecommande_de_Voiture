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

static bit bluetooth_stream_enabled = 0;

static void Bluetooth_handle_commands(void)
{
    static char cmd_buf[24];
    static unsigned char cmd_len = 0;

    while (UART1_available())
    {
        char c = UART1_read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            cmd_buf[cmd_len] = '\0';

            if (strcmp(cmd_buf, "STREAM_ON") == 0)
            {
                bluetooth_stream_enabled = 1;
                UART1_send_string("OK,STREAM_ON\r\n");
            }
            else if (strcmp(cmd_buf, "STREAM_OFF") == 0)
            {
                bluetooth_stream_enabled = 0;
                UART1_send_string("OK,STREAM_OFF\r\n");
            }
            else if (strcmp(cmd_buf, "STATUS") == 0)
            {
                UART1_send_string(bluetooth_stream_enabled ? "STATUS,STREAM_ON\r\n"
                                                           : "STATUS,STREAM_OFF\r\n");
            }
            else if (cmd_len > 0)
            {
                UART1_send_string("ERR,UNKNOWN_CMD\r\n");
            }

            cmd_len = 0;
            continue;
        }

        if (cmd_len < (sizeof(cmd_buf) - 1))
            cmd_buf[cmd_len++] = c;
        else
            cmd_len = 0;
    }
}

static void IR_forward_imu_to_bluetooth(void)
{
    IR_Frame_t f;
    char ble_buf[40];

    while (IR_RX_get(&f))
    {
        if (f.addr != IR_ADDR2)
            continue;

        printf("cmd=%u val=0x%04X addr=0x%X\r\n",
               (unsigned int)f.cmd,
               (unsigned int)f.val,
               (unsigned int)f.addr);

        if (bluetooth_stream_enabled &&
            f.cmd >= IMU_CMD_BASE && f.cmd < IMU_CMD_BASE + IMU_REG_COUNT)
        {
            sprintf(ble_buf, "imu,%u,%u\r\n",
                    (unsigned int)(f.cmd - IMU_CMD_BASE),
                    (unsigned int)f.val);
            UART1_send_string(ble_buf);
        }
    }
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
		Bluetooth_handle_commands();
		IR_forward_imu_to_bluetooth();
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
				if (prev_lcd_state == LCD_S7) {
					// Resume from pause
					IR_Send(IR_CMD_RESUME, 0xFF, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);
				} else {
					// Fresh start: send mode, path, then start
					IR_Send(IR_CMD_MODE, active_mode, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);

					IR_Send(IR_CMD_PATH, active_path, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);

					IR_Send(IR_CMD_START, 0xFF, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);
				}
			} else if (lcd_state == LCD_S6) {
				if (prev_lcd_state == LCD_S7) {
					// Resume from pause
					IR_Send(IR_CMD_RESUME, 0xFF, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);
				} else {
					// Fresh start: send mode, then start
					IR_Send(IR_CMD_MODE, active_mode, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);

					IR_Send(IR_CMD_START, 0xFF, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);
				}
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
