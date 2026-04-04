#ifndef IR_TX_H
#define IR_TX_H

// Portable integer types (C51 does not provide <stdint.h>)
typedef unsigned char  uint8_t;
typedef unsigned int   uint16_t;

// Timer 0: 38 kHz carrier
// Half-period = SYSCLK / 38000 / 2 = 72000000 / 38000 / 2 ≈ 947 counts
// RELOAD = 65536 - 947 = 64589  (original 64588 also fine, kept as-is)
#define TIMER0_RELOAD   64588U

// Timer 2: one SIRC "T" unit = 7 carrier cycles at 38 kHz = 184.21 µs
// Counts = 72e6 * 7/38000 = 13263 → RELOAD = 65536 - 13263 = 52273
#define TIMER2_RELOAD   52273U

// FSM states
#define FSM_IDLE        0x00   // waiting for next send_ir_packet()
#define FSM_SENDING     0x01   // stepping through unit array

// Frame: start(4) + 8-bit cmd(max 32) + 16-bit data(max 64) + 4-bit addr(max 16) + stop(1) = 117 worst case
// Start = 2T burst + 2T space (4 units). Bit 0 = [1,0,0] (3), Bit 1 = [1,0,0,0] (4), MSB first
#define SIRC_MAX_UNITS      117U
// Trailing silence after the last unit before going IDLE (~1 ms)
#define SIRC_TRAIL_UNITS    4U
// Stop-and-wait deadlock timeout: 35 ms / 184.21 µs ≈ 190 ticks
// Asymmetric with STM32's 50 ms timeout to prevent synchronized collisions
#define PP_TIMEOUT_TICKS    190U

extern volatile unsigned char fsm_state;
extern volatile unsigned int  pp_idle_ticks;

void TIMER0_Init(void);
void TIMER2_Init(void);

// Non-blocking 28-bit packet: 8-bit cmd | 16-bit data | 4-bit addr.
// Frame: 2T+2T start | 8-bit cmd | 16-bit val | 4-bit addr | 1T stop
// Returns immediately; Timer 2 ISR drives the actual transmission.
// Drops silently if FSM is busy (fsm_state != FSM_IDLE).
void send_ir_packet(uint8_t cmd, uint16_t val, uint8_t addr);

#endif
