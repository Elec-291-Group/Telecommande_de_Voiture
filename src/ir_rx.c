/*
 * ir_rx.c — IR receiver for EFM8LB1, P0.7, Port Match interrupt
 *
 * Timer 3 runs free at SYSCLK/12 = 6 MHz → 1 tick ≈ 0.167 µs
 *
 * T = 263 µs  →  1578 ticks
 * 3.5T threshold = 921 µs  →  5526 ticks   (separates 0 / 1 / start)
 *
 * Signal is ACTIVE-LOW (TSOP demodulator output):
 *   LOW  = carrier burst   (active)
 *   HIGH = space / idle
 *
 * Edge strategy:
 *   Rising edge  → measure LOW-pulse duration; if ≥ 3.5T it's the start burst
 *   Falling edge → measure falling-to-falling interval to decode bits
 */

#include <EFM8LB1.h>
#include "ir_rx.h"
#include "config.h"

/* ---------- timing constants ---------------------------------------------- */
#define TICKS_PER_US    6U                              /* SYSCLK/12 = 6 MHz   */
#define THRESH_3_5T     ((unsigned int)(921U * TICKS_PER_US))   /* 5526 ticks  */

/* ---------- P0.7 bit masks ------------------------------------------------- */
#define IR_PIN_MASK     0x80U   /* P0.7 */

/* ---------- decoder FSM states --------------------------------------------- */
#define RX_IDLE             0   /* waiting for a start pulse                   */
#define RX_WAIT_FIRST_FALL  1   /* start seen, waiting for first bit edge      */
#define RX_BITS             2   /* collecting 16 bits                          */

/* ---------- module state --------------------------------------------------- */
static volatile unsigned char  rx_state    = RX_IDLE;
static volatile unsigned int   last_fall   = 0;    /* time of last falling edge */
static volatile unsigned int   rx_frame    = 0;    /* bits shift in here        */
static volatile unsigned char  bit_count   = 0;
static volatile unsigned char  frame_ready = 0;
static IR_Frame_t              pending;             /* written only in ISR       */

/* ---------- helpers -------------------------------------------------------- */

/* Read Timer 3: reading TMR3L first latches TMR3H (EFM8 hardware guarantee). */
static unsigned int read_t3(void)
{
    unsigned char lo = TMR3L;
    return ((unsigned int)TMR3H << 8) | lo;
}

/* ---------- Port Match ISR (EFM8LB1 interrupt vector 8) ------------------- */
void PMATCH_ISR(void) __interrupt(8)
{
    unsigned int  now      = read_t3();
    unsigned char pin_hi   = IR_IN;    /* 1 = HIGH (rising), 0 = LOW (falling) */
    unsigned int  interval;

    /*
     * Re-arm Port Match: update P0MAT to match current pin state so the
     * next change (opposite edge) triggers the interrupt again.
     */
    if (pin_hi)
        P0MAT |=  IR_PIN_MASK;   /* pin is HIGH → arm for next falling edge  */
    else
        P0MAT &= ~IR_PIN_MASK;   /* pin is LOW  → arm for next rising edge   */

    if (!pin_hi)
    {
        /* ================================================================
         * FALLING EDGE — start of a carrier burst
         * ================================================================ */
        switch (rx_state)
        {
            case RX_IDLE:
                last_fall = now;
                break;

            case RX_WAIT_FIRST_FALL:
                /* First bit's falling edge, right after the start burst.
                 * Record it as the reference for bit-interval measurement. */
                last_fall = now;
                rx_frame  = 0;
                bit_count = 0;
                rx_state  = RX_BITS;
                break;

            case RX_BITS:
                interval  = now - last_fall;   /* falling-to-falling span     */
                last_fall = now;

                rx_frame <<= 1;
                if (interval >= THRESH_3_5T)
                    rx_frame |= 1U;            /* long interval → bit '1'     */

                if (++bit_count == 16)
                {
                    /* Validate address nibble [3:0] against IR_ADDR2 (0x7). */
                    if ((rx_frame & 0x000FU) == (unsigned int)IR_ADDR2)
                    {
                        pending.cmd  = (unsigned char)((rx_frame >> 12) & 0x0FU);
                        pending.data = (unsigned char)((rx_frame >>  4) & 0xFFU);
                        pending.addr = (unsigned char)( rx_frame        & 0x0FU);
                        frame_ready  = 1;
                    }
                    rx_state = RX_IDLE;
                }
                break;
        }
    }
    else
    {
        /* ================================================================
         * RISING EDGE — end of a carrier burst
         * ================================================================ */
        interval = now - last_fall;   /* = duration of the LOW pulse          */

        if (interval >= THRESH_3_5T)
        {
            /* Long burst (≥ 3.5T) → this is the 4T start pulse.
             * Accept from any state (re-syncs on noise or mid-frame errors). */
            rx_state = RX_WAIT_FIRST_FALL;
        }
        /* Short burst (1T, part of a data bit) — no action needed here.      */
    }
}

/* ---------- IR_RX_init ----------------------------------------------------- */
void IR_RX_init(void)
{
    /* --- P0.7: digital input (high-impedance) ---------------------------- */
    P0MDIN  |=  IR_PIN_MASK;   /* digital mode                               */
    P0MDOUT &= ~IR_PIN_MASK;   /* open-drain → acts as input                 */
    P0      |=  IR_PIN_MASK;   /* write 1 → high-impedance                   */
    P0SKIP  |=  IR_PIN_MASK;   /* skip crossbar; P0.7 used as GPIO           */

    /* --- Timer 3: free-running at SYSCLK/12 = 6 MHz --------------------- */
    CKCON0  &= ~0x40U;         /* T3ML=0 → Timer 3 uses SYSCLK/12            */
    TMR3RL   = 0x0000;         /* reload to 0 (free-running full 16-bit)     */
    TMR3     = 0x0000;         /* start count from 0                         */
    TMR3CN0  = 0x04;           /* TR3=1, start; no overflow interrupt        */

    /* --- Port Match: watch P0.7, fire on any change --------------------- */
    P0MASK   = (unsigned char)(~IR_PIN_MASK);  /* 0x7F: bit 7 watched, others masked */
    P0MAT    = 0xFFU;          /* initially expect HIGH (idle line)          */

    EIE1    |= 0x01U;          /* EMAT=1: enable Port Match interrupt        */
    EA       = 1;
}

/* ---------- IR_RX_get ------------------------------------------------------ */
unsigned char IR_RX_get(IR_Frame_t *out)
{
    if (!frame_ready) return 0;

    EIE1 &= ~0x01U;   /* disable Port Match interrupt — brief critical section */
    *out = pending;
    frame_ready = 0;
    EIE1 |=  0x01U;   /* re-enable                                            */

    return 1;
}
