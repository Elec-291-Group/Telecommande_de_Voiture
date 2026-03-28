#include <stdlib.h>
#include <EFM8LB1.h>
#include "bootloader.h"
#include "timer.h"
#include "ir_tx.h"
#include "ir_rx.h"
#include "config.h"
#include "uart.h"
#include "lcd.h"
#include "lcd_fsm.h"
#include "data_buffers.h"
#include "bluetooth.h"

unsigned char crossing_action = 0;

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

static void uart0_send_hex_byte(unsigned char v)
{
    unsigned char n;
    n = (v >> 4) & 0x0Fu;
    UART0_send_char((char)(n < 10u ? '0' + n : 'A' + n - 10u));
    n = v & 0x0Fu;
    UART0_send_char((char)(n < 10u ? '0' + n : 'A' + n - 10u));
}

void IR_debug(void)
{
    IR_Frame_t f;
    send_ir_packet((uint8_t)IR_CMD_START, (uint16_t)0x0000, (uint8_t)IR_ADDR1);
    while (IR_RX_get(&f)) {
        if (f.addr == IR_ADDR2) {
            UART0_send_string("cmd=");
            uart0_send_hex_byte(f.cmd);
            UART0_send_string(" val=0x");
            uart0_send_hex_byte((unsigned char)(f.val >> 8));
            uart0_send_hex_byte((unsigned char)(f.val & 0xFFu));
            UART0_send_string(" addr=0x");
            uart0_send_hex_byte(f.addr);
            UART0_send_string("\r\n");
        }
    }
}

void IR_RX_decode_command(const IR_Frame_t *frame)
{
    switch (frame->cmd)
    {
        case IR_RX_CMD_DATA_RECEIVED:
            if (lcd_state == LCD_S10)
                lcd_state = LCD_S12;
            break;

        case IR_RX_CMD_CROSSING_ACTION:
            crossing_action = (unsigned char)(frame->val & 0xFF);
            break;

        default:
            break;
    }
}

static bit pb_start_latch  = 1;
static bit pb_pause_latch  = 1;
static bit pb_reset_latch  = 1;
static bit pbtxcmd_latch   = 1;
static bit pbsw_s12_latch  = 1;

static void handle_pb_start(void)
{
    if (PB_START == 0) {
        if (pb_start_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_START, 0x0000, IR_ADDR);
            pb_start_latch = 0;
        }
    } else {
        pb_start_latch = 1;
    }
}

static void handle_pb_pause(void)
{
    if (PB_PAUSE == 0) {
        if (pb_pause_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_PAUSE, 0x0000, IR_ADDR);
            if (lcd_state == LCD_S5  || lcd_state == LCD_S6 ||
                lcd_state == LCD_S9  || lcd_state == LCD_S11)
                LCD_FSM_pause(lcd_state);
            pb_pause_latch = 0;
        }
    } else {
        pb_pause_latch = 1;
    }
}

static void handle_pb_reset(void)
{
    if (PB_RESET == 0) {
        if (pb_reset_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_RESET, 0x0000, IR_ADDR);
            lcd_state = LCD_S0;
            pb_reset_latch = 0;
        }
    } else {
        pb_reset_latch = 1;
    }
}

static void send_path_waypoints(void)
{
    unsigned char count = PathBuffer_get_count();
    unsigned char i;
    Waypoint_t wpt;

    for (i = 0; i < count; i++) {
        if (!PathBuffer_get(i, &wpt))
            continue;
        send_ir_packet(
            (uint8_t)(IR_CMD_PATH_WPT_BASE + i),
            (uint16_t)(((uint16_t)(wpt.x_cm & 0xFF) << 8) | (uint16_t)(wpt.y_cm & 0xFF)),
            IR_ADDR
        );
        while (fsm_state == FSM_IDLE);
        while (fsm_state != FSM_IDLE);
    }
}

static void handle_s10_buttons(void)
{
    if (lcd_state != LCD_S10) {
        pbtxcmd_latch = 1;
        return;
    }

    /* PB_TXCMD: transmit all waypoints via IR */
    if (PB_TXCMD == 0) {
        if (pbtxcmd_latch && fsm_state == FSM_IDLE) {
            send_path_waypoints();
            pbtxcmd_latch = 0;
        }
    } else {
        pbtxcmd_latch = 1;
    }
}

static void handle_s12_buttons(void)
{
    if (lcd_state != LCD_S12) {
        pbsw_s12_latch = 1;
        return;
    }

    /* PB_TXCMD: re-transmit all waypoints via IR */
    if (PB_TXCMD == 0) {
        if (pbtxcmd_latch && fsm_state == FSM_IDLE) {
            send_path_waypoints();
            pbtxcmd_latch = 0;
        }
    } else {
        pbtxcmd_latch = 1;
    }

    /* JoyStick_SW: send IR_CMD_START and enter Running (Path) */
    if (JoyStick_SW == 0) {
        if (pbsw_s12_latch && fsm_state == FSM_IDLE) {
            send_ir_packet(IR_CMD_START, 0x0000, IR_ADDR);
            lcd_state = LCD_S11;
            pbsw_s12_latch = 0;
        }
    } else {
        pbsw_s12_latch = 1;
    }
}

// ---- Main ----
void main (){
	static xdata float joystick_x;
	static xdata float joystick_y;
	unsigned char x_byte, y_byte;
	lcd_state_t prev_lcd_state = LCD_NUM_STATES;

	init_pin_input();
	TIMER0_Init();
	TIMER2_Init();
	UART0_init();
	UART1_init();
	IR_RX_init();
	PathBuffer_reset();
	IMUBuffer_reset();

	InitPinADC(1, 4); // Configure Joystick_Y as analog input
	InitPinADC(1, 5); // Configure Joystick_X as analog input
	InitADC();

	LCD_4BIT();
	LCD_FSM_init();

	UART0_send_string("start\r\n");
	while(1){
		Bluetooth_handle_commands();
		Bluetooth_forward_imu();

		joystick_x = Volts_at_Pin(JOYSTICK_X);
		joystick_y = Volts_at_Pin(JOYSTICK_Y);

		x_byte = volt_to_byte(joystick_x);
		y_byte = volt_to_byte(joystick_y);

		LCD_FSM_update(x_byte, y_byte);

		handle_pb_start();
		handle_pb_pause();
		handle_pb_reset();
		handle_s10_buttons();
		handle_s12_buttons();

		// On transition into a running/pause state, send IR command
		if (lcd_state != prev_lcd_state) {
			if (lcd_state == LCD_S7) {
				// Paused
				send_ir_packet(IR_CMD_PAUSE, 0xFF, IR_ADDR);
				while (fsm_state == FSM_IDLE);
				while (fsm_state != FSM_IDLE);
			} else if (lcd_state == LCD_S5) {
				if (prev_lcd_state == LCD_S7) {
					// Resume from pause
					send_ir_packet(IR_CMD_START, 0xFF, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);
				} else {
					// Fresh start: send mode, path, then start
					send_ir_packet(IR_CMD_MODE, active_mode, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);

					send_ir_packet(IR_CMD_PATH, active_path, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);

					send_ir_packet(IR_CMD_START, 0xFF, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);
				}
			} else if (lcd_state == LCD_S6) {
				if (prev_lcd_state == LCD_S7) {
					// Resume from pause
					send_ir_packet(IR_CMD_START, 0xFF, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);
				} else {
					// Fresh start: send mode, then start
					send_ir_packet(IR_CMD_MODE, active_mode, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);

					send_ir_packet(IR_CMD_START, 0xFF, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);
				}
			} else if (lcd_state == LCD_S9) {
				if (prev_lcd_state == LCD_S7) {
					// Resume from pause
					send_ir_packet(IR_CMD_START, 0xFF, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);
				} else {
					// Fresh start: send mode, waypoints, then start
					send_ir_packet(IR_CMD_MODE, active_mode, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);

					send_path_waypoints();

					send_ir_packet(IR_CMD_START, 0xFF, IR_ADDR);
					while (fsm_state == FSM_IDLE);
					while (fsm_state != FSM_IDLE);
				}
			}
			prev_lcd_state = lcd_state;
		}

		// Send joystick only in remote mode
		if (lcd_state == LCD_S6) {
			send_ir_packet(IR_CMD_JOYSTICK_X, x_byte, IR_ADDR);
			while (fsm_state == FSM_IDLE);
			while (fsm_state != FSM_IDLE);

			send_ir_packet(IR_CMD_JOYSTICK_Y, y_byte, IR_ADDR);
			while (fsm_state == FSM_IDLE);
			while (fsm_state != FSM_IDLE);
		}
		
	}
}

