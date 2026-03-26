#ifndef IR_RX_H
#define IR_RX_H

/*
 * ir_rx.h — IR receiver driver for EFM8LB1
 *
 * Hardware:
 *   P0.7  — TSOP33x demodulated output (active-low), via Port Match interrupt
 *   Timer 3 — free-running 1-tick ≈ 0.167 µs counter (SYSCLK/12 = 6 MHz)
 *
 * Protocol: Pulse-Distance, MSB first, active-low
 *   Start : 4T LOW burst  (~1052 µs), detected on rising edge
 *   Bit 0 : 3T falling-to-falling (~789  µs)
 *   Bit 1 : 4T falling-to-falling (~1052 µs)
 *   Threshold 3.5T (~921 µs) separates 0 from 1
 *
 * Frame (16 bits, MSB first):
 *   [15:12] 4-bit cmd    [11:4] 8-bit data    [3:0] 4-bit addr
 */

/* Decoded IR frame */
typedef struct {
    unsigned char cmd;    /* 4-bit command  [15:12] */
    unsigned char data;   /* 8-bit payload  [11:4]  */
    unsigned char addr;   /* 4-bit address  [ 3:0]  */
} IR_Frame_t;

/* Configure P0.7, Timer 3, and Port Match interrupt.
 * Call once before enabling interrupts (EA=1 handled internally). */
void IR_RX_init(void);

/* Returns 1 and fills *out if a complete valid frame is waiting, else 0.
 * Safe to call from main loop. */
unsigned char IR_RX_get(IR_Frame_t *out);

#endif /* IR_RX_H */
