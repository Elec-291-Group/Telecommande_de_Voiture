#ifndef TIMER_H
#define TIMER_H

// Timer 0: 38 kHz carrier
// Half-period = SYSCLK / 38000 / 2 = 72000000 / 38000 / 2 ≈ 947 counts
// RELOAD = 65536 - 947 = 64589  (original 64588 also fine, kept as-is)
#define TIMER0_RELOAD   64588U

// Timer 2: one SIRC "T" unit = 10 carrier cycles at 38 kHz = 263.16 µs
// Counts = 72e6 * 10/38000 = 18947 → RELOAD = 65536 - 18947 = 46589
#define TIMER2_RELOAD   46589U

// FSM states
#define FSM_IDLE        0x00   // waiting for next IR_Send()
#define FSM_SENDING     0x01   // stepping through unit array

// Frame: start(6) + 4-bit cmd(max 16) + 8-bit payload(max 32) + 4-bit addr(max 16) + stop(1) = 71 worst case
// Bit 0 = [1,0,0] (3 units), Bit 1 = [1,0,0,0] (4 units), MSB first
#define SIRC_MAX_UNITS      71U
// 45 ms repeat period: 45 ms / 263.16 µs = 171 units
#define SIRC_PERIOD_UNITS   171U

extern volatile unsigned char fsm_state;

void TIMER0_Init(void);
void TIMER2_Init(void);

// Build frame into the unit buffer.
// cmd_name : 4-bit command field (lower nibble)
// payload  : 8-bit data byte
// address  : 4-bit device address (lower nibble), fixed 0xB
void PrepareFrame(unsigned char cmd_name, unsigned char payload, unsigned char address);

// Trigger one IR transmission.  FSM returns to IDLE after the 45 ms period.
void IR_Send(unsigned char cmd_name, unsigned char payload, unsigned char address);

void Timer3us(unsigned char us);
void waitms(unsigned int ms);

#endif
