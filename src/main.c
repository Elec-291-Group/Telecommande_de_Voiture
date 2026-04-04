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

unsigned char crossing_action   = 0;
static unsigned char intersection_num = 0;
static bit           crossing_updated = 0;
signed char left_power  = 0;
signed char right_power = 0;

/* ---- Stop-and-wait TX command queue ------------------------------------ */
typedef struct {
    uint8_t  cmd;
    uint16_t val;
    uint8_t  addr;
} TX_Cmd_t;

#define TX_Q_SIZE  16
#define TX_Q_MASK  (TX_Q_SIZE - 1)

static TX_Cmd_t xdata tx_q[TX_Q_SIZE];
static unsigned char tx_q_head = 0;
static unsigned char tx_q_tail = 0;

static void tx_q_push(uint8_t cmd, uint16_t val, uint8_t addr)
{
    unsigned char next = (tx_q_head + 1) & TX_Q_MASK;
    if (next == tx_q_tail) return;   /* full — drop */
    tx_q[tx_q_head].cmd  = cmd;
    tx_q[tx_q_head].val  = val;
    tx_q[tx_q_head].addr = addr;
    tx_q_head = next;
}

static unsigned char tx_q_pop(TX_Cmd_t *out)
{
    unsigned char t;
    if (tx_q_head == tx_q_tail) return 0;
    t = tx_q_tail;
    out->cmd  = tx_q[t].cmd;
    out->val  = tx_q[t].val;
    out->addr = tx_q[t].addr;
    tx_q_tail = (t + 1) & TX_Q_MASK;
    return 1;
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

static void uart0_send_hex_byte(unsigned char v)
{
    unsigned char n;
    n = (v >> 4) & 0x0Fu;
    UART0_send_char((char)(n < 10u ? '0' + n : 'A' + n - 10u));
    n = v & 0x0Fu;
    UART0_send_char((char)(n < 10u ? '0' + n : 'A' + n - 10u));
}

static void IR_TX_debug_print(unsigned char cmd, unsigned int val)
{
    UART0_send_string("TX cmd=");
    uart0_send_hex_byte(cmd);
    UART0_send_string(" val=0x");
    uart0_send_hex_byte((unsigned char)(val >> 8));
    uart0_send_hex_byte((unsigned char)(val & 0xFFu));
    UART0_send_string("\r\n");
}
/*
void IR_debug(void)
{
    IR_Frame_t f;
    send_ir_packet((uint8_t)IR_CMD_START, (uint16_t)0x0000, (uint8_t)IR_ADDR1);
    while (fsm_state == FSM_IDLE);
    while (fsm_state != FSM_IDLE);
    IR_TX_debug_print(IR_CMD_START, 0x0000);
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

static void IR_RX_debug_print(const IR_Frame_t *frame)
{
    UART0_send_string("RX cmd=");
    uart0_send_hex_byte(frame->cmd);
    UART0_send_string(" val=0x");
    uart0_send_hex_byte((unsigned char)(frame->val >> 8));
    uart0_send_hex_byte((unsigned char)(frame->val & 0xFFu));
    UART0_send_string(" addr=0x");
    uart0_send_hex_byte(frame->addr);
    UART0_send_string("\r\n");
}
*/

void IR_RX_decode_command(const IR_Frame_t *frame)
{
    switch (frame->cmd)
    {
        case IR_RX_CMD_DATA_RECEIVED:
            break;

        case IR_RX_CMD_CROSSING_ACTION:
            crossing_action = (unsigned char)(frame->val & 0xFF);
            if (lcd_state == LCD_S5) {
                intersection_num++;
                crossing_updated = 1;
            }
            break;

        case IR_RX_CMD_LEFT_POWER:
        {
            unsigned char byte1 = (frame->val >> 8) & 0xFF;
            unsigned char byte2 = frame->val & 0xFF;
            left_power = (byte1 > 0) ? (signed char)byte1 : -((signed char)byte2);
            break;
        }

        case IR_RX_CMD_RIGHT_POWER:
        {
            unsigned char byte1 = (frame->val >> 8) & 0xFF;
            unsigned char byte2 = frame->val & 0xFF;
            right_power = (byte1 > 0) ? (signed char)byte1 : -((signed char)byte2);
            break;
        }

        case IR_RX_CMD_IS_CONFIG:
           // UART1_send_string("btn,reset\r\n");
            lcd_state = LCD_S0;
            break;
        
        default:
            break;
    }
}

static bit pb_start_latch    = 1;
static bit pb_pause_latch    = 1;
static bit pb_reset_latch    = 1;
static bit pbtxcmd_s13_latch = 1;
static bit pbstart_s13_latch = 1;
static bit pbtxcmd_s14_latch = 1;
static bit pbstart_s14_latch = 1;
static bit pbtxcmd_s15_latch = 1;
static bit pbstart_s15_latch = 1;
static bit pbtxcmd_s18_latch = 1;
static bit pbstart_s18_latch = 1;
static bit pb0_s7_latch      = 1;
static bit pb0_s8_latch      = 1;
static lcd_state_t s7_resume_state = LCD_S0; // which running state S7 should resume to

static void handle_pb_start(void)
{
    if (PB_START == 0) {
        if (pb_start_latch) {
            tx_q_push(IR_CMD_START, 0x0000, IR_ADDR);
            IR_TX_debug_print(IR_CMD_START, 0x0000);
            UART1_send_string("btn,start\r\n");
            pb_start_latch = 0;
        }
    } else {
        pb_start_latch = 1;
    }
}

static void handle_pb_pause(void)
{
    if (PB_PAUSE == 0) {
        if (pb_pause_latch) {
            tx_q_push(IR_CMD_PAUSE, 0x0000, IR_ADDR);
            IR_TX_debug_print(IR_CMD_PAUSE, 0x0000);
            UART1_send_string("btn,pause\r\n");
            if (lcd_state == LCD_S5  || lcd_state == LCD_S6 ||
                lcd_state == LCD_S9  || lcd_state == LCD_S11) {
                s7_resume_state = lcd_state;
                LCD_FSM_pause(lcd_state);
            }
            pb_pause_latch = 0;
        }
    } else {
        pb_pause_latch = 1;
    }
}

static void handle_pb_reset(void)
{
    if (PB_RESET == 0) {
        if (pb_reset_latch) {
            tx_q_push(IR_CMD_RESET, 0x0000, IR_ADDR);
            IR_TX_debug_print(IR_CMD_RESET, 0x0000);
            UART1_send_string("btn,reset\r\n");
            lcd_state = LCD_S0;
            pb_reset_latch = 0;
        }
    } else {
        pb_reset_latch = 1;
    }
}

static void queue_path_waypoints(void)
{
    unsigned char count = PathBuffer_get_count();
    unsigned char i;
    Waypoint_t wpt;

    for (i = 0; i < count; i++) {
        if (!PathBuffer_get(i, &wpt))
            continue;
        tx_q_push(
            (uint8_t)(IR_CMD_PATH_WPT_BASE + i),
            (uint16_t)(((uint16_t)(wpt.x_cm & 0xFF) << 8) | (uint16_t)(wpt.y_cm & 0xFF)),
            IR_ADDR
        );
    }
}

static void handle_s13_buttons(void)
{
    if (lcd_state != LCD_S13) { pbtxcmd_s13_latch = 1; pbstart_s13_latch = 1; return; }

    /* PB_TXCMD: queue MODE + PATH to car */
    if (PB_TXCMD == 0) {
        if (pbtxcmd_s13_latch) {
            tx_q_push(IR_CMD_MODE, active_mode, IR_ADDR);
            tx_q_push(IR_CMD_PATH, active_path + 1, IR_ADDR);
            IR_TX_debug_print(IR_CMD_MODE, active_mode);
            IR_TX_debug_print(IR_CMD_PATH, active_path + 1);
            pbtxcmd_s13_latch = 0;
        }
    } else {
        pbtxcmd_s13_latch = 1;
    }

    /* PB_START: queue start command and go to S5 */
    if (PB_START == 0) {
        if (pbstart_s13_latch) {
            tx_q_push(IR_CMD_START, 0x0000, IR_ADDR);
            IR_TX_debug_print(IR_CMD_START, 0x0000);
            intersection_num = 0;
            crossing_updated = 0;
            lcd_state = LCD_S5;
            pbstart_s13_latch = 0;
        }
    } else {
        pbstart_s13_latch = 1;
    }
}

static void handle_s14_buttons(void)
{
    if (lcd_state != LCD_S14) { pbtxcmd_s14_latch = 1; pbstart_s14_latch = 1; return; }

    /* PB_TXCMD: queue MODE to car */
    if (PB_TXCMD == 0) {
        if (pbtxcmd_s14_latch) {
            tx_q_push(IR_CMD_MODE, active_mode, IR_ADDR);
            IR_TX_debug_print(IR_CMD_MODE, active_mode);
            pbtxcmd_s14_latch = 0;
        }
    } else {
        pbtxcmd_s14_latch = 1;
    }

    /* PB_START: queue start command and go to S6 */
    if (PB_START == 0) {
        if (pbstart_s14_latch) {
            tx_q_push(IR_CMD_START, 0x0000, IR_ADDR);
            IR_TX_debug_print(IR_CMD_START, 0x0000);
            lcd_state = LCD_S6;
            pbstart_s14_latch = 0;
        }
    } else {
        pbstart_s14_latch = 1;
    }
}

static void handle_s15_buttons(void)
{
    if (lcd_state != LCD_S15) { pbtxcmd_s15_latch = 1; pbstart_s15_latch = 1; return; }

    /* PB_TXCMD: queue MODE (pathfind) + all waypoints + zero_yaw */
    if (PB_TXCMD == 0) {
        if (pbtxcmd_s15_latch) {
            tx_q_push(IR_CMD_MODE, 0x0002, IR_ADDR);
            queue_path_waypoints();
            tx_q_push(IR_CMD_ZERO_YAW, 0x0012, IR_ADDR);
            IR_TX_debug_print(IR_CMD_MODE, 0x0002);
            IR_TX_debug_print(IR_CMD_ZERO_YAW, 0x0012);
            pbtxcmd_s15_latch = 0;
        }
    } else {
        pbtxcmd_s15_latch = 1;
    }

    /* PB_START: queue start command and go to S11 */
    if (PB_START == 0) {
        if (pbstart_s15_latch) {
            tx_q_push(IR_CMD_START, 0x0000, IR_ADDR);
            IR_TX_debug_print(IR_CMD_START, 0x0000);
            lcd_state = LCD_S11;
            pbstart_s15_latch = 0;
        }
    } else {
        pbstart_s15_latch = 1;
    }
}

static void handle_s18_buttons(void)
{
    if (lcd_state != LCD_S18) { pbtxcmd_s18_latch = 1; pbstart_s18_latch = 1; return; }

    /* PB_TXCMD: queue MODE(0=auto) + PATH + manul_path packed 16-bit */
    if (PB_TXCMD == 0) {
        if (pbtxcmd_s18_latch) {
            tx_q_push(IR_CMD_MODE, 0x0000, IR_ADDR);
            tx_q_push(IR_CMD_PATH, 0x0004, IR_ADDR);
            tx_q_push(IR_CMD_MANUL_PATH, manual_path_buf, IR_ADDR);
            tx_q_push(IR_CMD_MANUL_PATH, manual_path_buf, IR_ADDR);
            IR_TX_debug_print(IR_CMD_MODE, 0x0000);
            IR_TX_debug_print(IR_CMD_PATH, 0x0004);
            IR_TX_debug_print(IR_CMD_MANUL_PATH, manual_path_buf);
            pbtxcmd_s18_latch = 0;
        }
    } else {
        pbtxcmd_s18_latch = 1;
    }

    /* PB_START: queue start command and go to S5 */
    if (PB_START == 0) {
        if (pbstart_s18_latch) {
            tx_q_push(IR_CMD_START, 0x0000, IR_ADDR);
            IR_TX_debug_print(IR_CMD_START, 0x0000);
            lcd_state = LCD_S5;
            pbstart_s18_latch = 0;
        }
    } else {
        pbstart_s18_latch = 1;
    }
}

static void handle_s7_buttons(void)
{
    if (lcd_state != LCD_S7) { pb0_s7_latch = 1; return; }

    /* JoyStick_SW: resume — queue IR start and return to previous running state */
    if (JoyStick_SW == 0) {
        if (pb0_s7_latch) {
            tx_q_push(IR_CMD_START, 0xFF, IR_ADDR);
            IR_TX_debug_print(IR_CMD_START, 0xFF);
            lcd_state = s7_resume_state;
            pb0_s7_latch = 0;
        }
    } else {
        pb0_s7_latch = 1;
    }
}

static void handle_s8_buttons(void)
{
    if (lcd_state != LCD_S8) { pb0_s8_latch = 1; return; }

    /* JoyStick_SW: go to S9 (no IR — pathfind uses BT path via S15) */
    if (JoyStick_SW == 0) {
        if (pb0_s8_latch) {
            active_mode = selected_mode;
            active_path = selected_path;
            lcd_state = LCD_S9;
            pb0_s8_latch = 0;
        }
    } else {
        pb0_s8_latch = 1;
    }
}

// ---- Crossing display (Auto/Field mode S5) ----
static void update_crossing_display(void)
{
    static xdata char line1[17];
    unsigned char path_num = active_path + 1; // 0-indexed → 1-3
    unsigned char i = 0;

    line1[i++] = 'P';
    line1[i++] = (char)('0' + path_num);
    line1[i++] = ' ';
    line1[i++] = 'I'; line1[i++] = 'n'; line1[i++] = 't'; line1[i++] = ':';
    if (intersection_num >= 10)
        line1[i++] = (char)('0' + intersection_num / 10);
    line1[i++] = (char)('0' + intersection_num % 10);
    while (i < 16) line1[i++] = ' ';
    line1[16] = '\0';
    LCDprint(line1, 1, 0);

    switch (crossing_action) {
        case 0: LCDprint("Forward         ", 2, 0); break;
        case 1: LCDprint("Left            ", 2, 0); break;
        case 2: LCDprint("Right           ", 2, 0); break;
        case 3: LCDprint("Stop            ", 2, 0); break;
        default: break;
    }
}

/* ---- Stop-and-wait half-duplex coordinator ------------------------------ */
/*
 * Protocol: EFM8 TX → STM32 RX → STM32 TX → EFM8 RX → EFM8 TX → ...
 * - After receiving a valid frame from STM32 (addr=0x7), immediately TX next.
 * - If no RX within 50 ms, TX anyway (deadlock fallback / bootstrap).
 * - Never TX while RX FSM is mid-frame.
 * - Queued commands have priority; joystick X/Y sent round-robin in S6 mode.
 */
static bit saw_tx_ready    = 0;

static void saw_step(void)
{
    static IR_Frame_t xdata pp_frame;
    TX_Cmd_t txc;

    /* --- Drain all pending RX frames ------------------------------------ */
    while (IR_RX_get(&pp_frame))
    {
        if (pp_frame.addr == IR_ADDR2)
        {
            IR_RX_decode_command(&pp_frame);
            IMUBuffer_push_frame(&pp_frame);
            saw_tx_ready   = 1;      /* STM32 spoke — our turn next        */
            pp_idle_ticks = 0;      /* reset deadlock timer                */
        }
    }

    /* --- Deadlock / bootstrap timeout (50 ms) --------------------------- */
    if (pp_idle_ticks >= PP_TIMEOUT_TICKS)
    {
        saw_tx_ready   = 1;
        pp_idle_ticks = 0;
    }

    /* --- Transmit one packet if it's our turn --------------------------- */
    if (saw_tx_ready && fsm_state == FSM_IDLE && !IR_RX_is_busy())
    {
        /* Priority 1: queued commands (buttons, config, waypoints) */
        if (tx_q_pop(&txc))
        {
            send_ir_packet(txc.cmd, txc.val, txc.addr);
        }
        /* Priority 2: NOP heartbeat — keep the exchange alive */
        else
        {
            send_ir_packet(IR_CMD_NOP, 0x0000, IR_ADDR);
        }

        saw_tx_ready = 0;
    }
}

// ---- Main ----
void main (){
	static xdata float joystick_x;
	static xdata float joystick_y;
	unsigned char x_byte, y_byte;

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
        // poBluetooth_debug_drive_sequencewer debug 
       // Bluetooth_debug_drive_sequence();
        
		Bluetooth_handle_commands();
		Bluetooth_forward_imu();
		joystick_x = Volts_at_Pin(JOYSTICK_X);
		joystick_y = Volts_at_Pin(JOYSTICK_Y);

		x_byte = volt_to_byte(joystick_x);
		y_byte = volt_to_byte(joystick_y);

		LCD_FSM_update(x_byte, y_byte);
        // printf ("JOYSTICK: %c  %c", x_byte, y_byte);
        UART0_send_string("JOY X=");
        uart0_send_hex_byte(x_byte);
        UART0_send_string(" Y=");
        uart0_send_hex_byte(y_byte);
        UART0_send_string("\r\n");

		// In joystick mode, refill queue with fresh X/Y when drained
		if (lcd_state == LCD_S6 && tx_q_head == tx_q_tail) {
			tx_q_push(IR_CMD_JOYSTICK_X, x_byte, IR_ADDR);
			tx_q_push(IR_CMD_JOYSTICK_Y, y_byte, IR_ADDR);
		}

		handle_pb_start();
		handle_pb_pause();
		handle_pb_reset();
		handle_s7_buttons();
		handle_s8_buttons();
		handle_s13_buttons();
		handle_s14_buttons();
		handle_s15_buttons();
		handle_s18_buttons();

		// Update intersection display in auto mode
		if (lcd_state == LCD_S5 && crossing_updated) {
			crossing_updated = 0;
			update_crossing_display();
		}
		// Debug: print any addr=0x7 frame received
        /*
		if (dbg_frame_ready) {
			dbg_frame_ready = 0;
			IR_RX_debug_print(&dbg_frame);
		}
        */
		// Debug: print individual imu_reg when it changes
/*
		{
			static unsigned int xdata imu_prev[IMU_REG_COUNT];
			static bit imu_prev_init = 0;
			unsigned char ri;

			if (!imu_prev_init) {
				for (ri = 0; ri < IMU_REG_COUNT; ri++)
					imu_prev[ri] = imu_regs[ri];
				imu_prev_init = 1;
			}

			for (ri = 0; ri < IMU_REG_COUNT; ri++) {
				if (imu_regs[ri] != imu_prev[ri]) {
					UART0_send_string("IMU[");
					uart0_send_hex_byte(ri);
					UART0_send_string("]=");
					uart0_send_hex_byte((unsigned char)(imu_regs[ri] >> 8));
					uart0_send_hex_byte((unsigned char)(imu_regs[ri] & 0xFFu));
					UART0_send_string("\r\n");
					imu_prev[ri] = imu_regs[ri];
				}
			}
		}
    */
		// Debug: print left/right power whenever they change
    /*
		{
			static signed char prev_lp = 0;
			static signed char prev_rp = 0;
			if (left_power != prev_lp || right_power != prev_rp) {
				UART0_send_string("PWR: L=");
				if (left_power < 0) {
					UART0_send_char('-');
					uart0_send_hex_byte((unsigned char)(-(left_power)));
				} else {
					uart0_send_hex_byte((unsigned char)left_power);
				}
				UART0_send_string(" R=");
				if (right_power < 0) {
					UART0_send_char('-');
					uart0_send_hex_byte((unsigned char)(-(right_power)));
				} else {
					uart0_send_hex_byte((unsigned char)right_power);
				}
				UART0_send_string("\r\n");
				prev_lp = left_power;
				prev_rp = right_power;
			}
		}*/
		/* Stop-and-wait coordinator: drain RX, handle deadlock timeout,
		 * send one queued command or joystick packet per turn. */
		saw_step();
		
	}
}

