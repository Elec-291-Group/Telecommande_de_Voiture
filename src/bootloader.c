#include <EFM8LB1.h>
#include "bootloader.h"
#include "config.h"

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN |= 0x80;       // enable VDD monitor
	RSTSRC = 0x02;  // Enable reset on missing clock detector and VDD

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif

	SFRPAGE = 0x00;
	P0MDOUT |= 0x14; // Enable UART0 TX as push-pull output P0.4, P0.2
	P0SKIP = 0xC3;
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)
	XBR1     = 0x00;
	XBR2     = 0x41; // Enable crossbar (bit6), enable UART1 (bit0) on 
	SFRPAGE = 0x00;

	return 0;
}

// init p2.2 and p2.3 as input pin
void init_pin_input(void){
    P2MDIN |= 0b_0000_0110; // enable p2.1, p2.2 to be digital pin
    P2MDOUT |= 0b_0000_0110; // enable p2.1, p2.2 to be output pin 
    P2SKIP |= 0b_0000_0010;
	P3MDIN |= 0b_0000_0001; // set p3.0 to be digital pin
	P3MDOUT &= 0b_1111_1110; // enable p3.0 to be output pin
	P3 |= 0b_0000_0001;
	XBR2 &= ~0x80;
    XBR2 |= 0x40;
	P2_1 = 0;
}

