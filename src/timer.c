#include <EFM8LB1.h>
#include "timer.h"
#include "config.h"

// ---- Timer 3 — microsecond delay ------------------------------------

void Timer3us(unsigned char us)
{
	unsigned char i;

	CKCON0 |= 0b_0100_0000;             // Timer 3 clocked by SYSCLK
	TMR3RL  = (-(SYSCLK) / 1000000L);  // overflow every 1 µs
	TMR3    = TMR3RL;
	TMR3CN0 = 0x04;                     // start Timer 3

	for (i = 0; i < us; i++)
	{
		while (!(TMR3CN0 & 0x80));  // wait for overflow
		TMR3CN0 &= ~(0x80);         // clear overflow flag
	}
	TMR3CN0 = 0;
}

void waitms(unsigned int ms)
{
	unsigned int  j;
	unsigned char k;
	for (j = 0; j < ms; j++)
		for (k = 0; k < 4; k++) Timer3us(250);
}
