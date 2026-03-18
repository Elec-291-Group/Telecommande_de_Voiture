#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>
#include <string.h>
#include "bootloader.h"
#include "timer.h"
#include "config.h"

// ---- Global state variables ----
volatile unsigned char ir_state = 1; // variable for ir message. 0 = wave, 1 = constant
volatile bit last_ir_state = 0;

void debounce(void){
	if(P3_0 == 0){
		waitms(50);
		if(P3_0 == 0){
			while(P3_0 == 0);
			ir_state = !ir_state;
		}
	}
}

void switch_ir_mode(void){
	if(ir_state){
		// turn OFF IR
        TR2 = 0;
        ET2 = 0;

        TR0 = 0;
        ET0 = 0;
        P2_1 = 0;
	}
	else{
		// turn ON IR
        TH0 = (TIMER0_RELOAD >> 8) & 0xFF;
        TL0 = TIMER0_RELOAD & 0xFF;
        TF0 = 0;
        ET0 = 1;
        TR0 = 1;

        TMR2H = (TIMER2_RELOAD >> 8) & 0xFF;
        TMR2L = TIMER2_RELOAD & 0xFF;
        TF2H = 0;
        TF2L = 0;
        ET2 = 1;
        TR2 = 1;
	}
}

// ---- Main ----
void main (){
	int i = 0;
	init_pin_input();
	TIMER0_Init();
	TIMER2_Init();
	

	IR_Send(0); // default: send 0 on startup

	while(1){
		/*
		if(ir_state != last_ir_state){
			IR_Send(ir_state); // send 1 when toggled on, 0 when toggled off
			last_ir_state = ir_state;
		}
			*/
		waitms(500);
		
		for (i=0; i<10; i++) {
			IR_Send(ir_state);
		}
	}
}
