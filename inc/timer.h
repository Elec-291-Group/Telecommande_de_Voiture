#ifndef TIMER_H
#define TIMER_H

#define TIMER0_RELOAD 64588
#define TIMER2_RELOAD 46588

#define FSM_IDLE 11  // Sentinel value: FSM is idle, waiting for IR_Send()

extern volatile unsigned char fsm_state;
extern volatile unsigned char ir_byte;
extern volatile unsigned int  burst_count;

void TIMER0_Init(void);
void TIMER2_Init(void);

// Trigger a full IR transmission of one byte.
// The FSM runs through states 0-10 on subsequent Timer 2 interrupts.
void IR_Send(unsigned char byte);

void Timer3us(unsigned char us);
void waitms(unsigned int ms);

#endif
