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
#include "data_buffers.h"

static bit bluetooth_stream_enabled = 0;
static char xdata bluetooth_cmd_buf[24];
static char xdata bluetooth_parse_buf[24];
static IR_Frame_t xdata bluetooth_rx_frame;
static char xdata uart_digits[5];

static int parse_signed_int(const char *text, unsigned char *ok)
{
	int sign = 1;
	int value = 0;
	char c;

	*ok = 0;
	if ((text == 0) || (*text == '\0'))
		return 0;

	if (*text == '-') {
		sign = -1;
		text++;
	} else if (*text == '+') {
		text++;
	}

	if (*text == '\0')
		return 0;

	while ((c = *text++) != '\0') {
		if ((c < '0') || (c > '9'))
			return 0;
		value = (value * 10) + (c - '0');
	}

	*ok = 1;
	return sign * value;
}

static void Bluetooth_send_rx_cmd_echo(const char *cmd_text)
{
	UART1_send_string("RX_CMD,");
	UART1_send_string(cmd_text);
	UART1_send_string("\r\n");
}

static void UART1_send_uint(unsigned int value)
{
	unsigned char count = 0;

	if (value == 0) {
		UART1_send_char('0');
		return;
	}

	while ((value > 0) && (count < sizeof(uart_digits))) {
		uart_digits[count++] = (char)('0' + (value % 10u));
		value /= 10u;
	}

	while (count > 0)
		UART1_send_char(uart_digits[--count]);
}

static void Bluetooth_stream_imu_frame(const IR_Frame_t *frame)
{
	UART1_send_string("imu,");
	UART1_send_uint((unsigned int)(frame->cmd - IMU_CMD_BASE));
	UART1_send_char(',');
	UART1_send_uint(frame->val);
	UART1_send_string("\r\n");
}

static void Bluetooth_handle_path_command(const char *cmd_text)
{
	char *path_token;
	char *path_arg1;
	char *path_arg2;
	char *path_arg3;
	unsigned char path_ok1;
	unsigned char path_ok2;
	unsigned char path_ok3;
	int path_value1;
	int path_value2;
	int path_value3;

	strcpy(bluetooth_parse_buf, cmd_text);
	path_token = strtok(bluetooth_parse_buf, ",");
	if (path_token == 0)
		return;

	if (strcmp(path_token, "PATH_BEGIN") == 0)
	{
		path_arg1 = strtok(0, ",");
		if (path_arg1 == 0)
		{
			UART1_send_string("PATH_ACK,ERROR\r\n");
			return;
		}

		path_value1 = parse_signed_int(path_arg1, &path_ok1);
		if (!path_ok1 || !PathBuffer_begin((unsigned char)path_value1))
		{
			UART1_send_string("PATH_ACK,ERROR\r\n");
			return;
		}

		UART1_send_string("PATH_ACK,BEGIN\r\n");
		return;
	}

	if (strcmp(path_token, "WPT") == 0)
	{
		path_arg1 = strtok(0, ",");
		path_arg2 = strtok(0, ",");
		path_arg3 = strtok(0, ",");
		if ((path_arg1 == 0) || (path_arg2 == 0) || (path_arg3 == 0))
			return;

		path_value1 = parse_signed_int(path_arg1, &path_ok1);
		path_value2 = parse_signed_int(path_arg2, &path_ok2);
		path_value3 = parse_signed_int(path_arg3, &path_ok3);
		if (!path_ok1 || !path_ok2 || !path_ok3)
			return;

		if (PathBuffer_store((unsigned char)path_value1, path_value2, path_value3))
			Bluetooth_send_rx_cmd_echo(cmd_text);
		return;
	}

	if (strcmp(path_token, "PATH_END") == 0)
	{
		if (PathBuffer_commit())
			UART1_send_string("PATH_ACK,LOADED\r\n");
		else
			UART1_send_string("PATH_ACK,ERROR\r\n");
	}
}

static void Bluetooth_handle_commands(void)
{
    static unsigned char cmd_len = 0;

    while (UART1_available())
    {
        char c = UART1_read();

        if (c == '\r')
            continue;

        if (c == '\n')
        {
            bluetooth_cmd_buf[cmd_len] = '\0';

            if (strcmp(bluetooth_cmd_buf, "STREAM_ON") == 0)
            {
                bluetooth_stream_enabled = 1;
                UART1_send_string("OK,STREAM_ON\r\n");
            }
            else if (strcmp(bluetooth_cmd_buf, "STREAM_OFF") == 0)
            {
                bluetooth_stream_enabled = 0;
                UART1_send_string("OK,STREAM_OFF\r\n");
            }
            else if (strcmp(bluetooth_cmd_buf, "STATUS") == 0)
            {
                UART1_send_string(bluetooth_stream_enabled ? "STATUS,STREAM_ON\r\n"
                                                           : "STATUS,STREAM_OFF\r\n");
            }
            else if ((strncmp(bluetooth_cmd_buf, "PATH_BEGIN,", 11) == 0) ||
                     (strncmp(bluetooth_cmd_buf, "WPT,", 4) == 0) ||
                     (strcmp(bluetooth_cmd_buf, "PATH_END") == 0))
            {
                Bluetooth_handle_path_command(bluetooth_cmd_buf);
            }
            else if (cmd_len > 0)
            {
                UART1_send_string("ERR,UNKNOWN_CMD\r\n");
            }

            cmd_len = 0;
            continue;
        }

        if (cmd_len < (sizeof(bluetooth_cmd_buf) - 1))
            bluetooth_cmd_buf[cmd_len++] = c;
        else
            cmd_len = 0;
    }
}

static void IR_forward_imu_to_bluetooth(void)
{
    while (IR_RX_get(&bluetooth_rx_frame))
    {
        if (bluetooth_rx_frame.addr != IR_ADDR2)
            continue;

        IMUBuffer_push_frame(&bluetooth_rx_frame);

        if (bluetooth_stream_enabled &&
            bluetooth_rx_frame.cmd >= IMU_CMD_BASE &&
            bluetooth_rx_frame.cmd < IMU_CMD_BASE + IMU_REG_COUNT)
        {
            Bluetooth_stream_imu_frame(&bluetooth_rx_frame);
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

void IR_debug() {
	IR_Frame_t f;
    send_ir_packet((uint8_t)IR_CMD_START, (uint16_t)0x0000, (uint8_t)IR_ADDR1);
    while (IR_RX_get(&f))
        if (f.addr == IR_ADDR2)
            printf("cmd=%u val=0x%04X addr=0x%X\r\n",
                   (unsigned int)f.cmd,
                   (unsigned int)f.val,
                   (unsigned int)f.addr);
}

// ---- Main ----
void main (){
	static xdata float joystick_x;
	static xdata float joystick_y;
	unsigned char x_byte, y_byte;
	lcd_state_t prev_lcd_state = LCD_NUM_STATES;
	static xdata char ble_buf[17];

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
		IR_forward_imu_to_bluetooth();
		IR_debug();

		joystick_x = Volts_at_Pin(JOYSTICK_X);
		joystick_y = Volts_at_Pin(JOYSTICK_Y);

		x_byte = volt_to_byte(joystick_x);
		y_byte = volt_to_byte(joystick_y);

		LCD_FSM_update(x_byte, y_byte);

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
					send_ir_packet(IR_CMD_RESUME, 0xFF, IR_ADDR);
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

