;--------------------------------------------------------
; File Created by C51
; Version 1.0.0 #1170 (Feb 16 2022) (MSVC)
; This file was generated Mon Mar 09 10:37:24 2026
;--------------------------------------------------------
$name cmd
$optc51 --model-small
	R_DSEG    segment data
	R_CSEG    segment code
	R_BSEG    segment bit
	R_XSEG    segment xdata
	R_PSEG    segment xdata
	R_ISEG    segment idata
	R_OSEG    segment data overlay
	BIT_BANK  segment data overlay
	R_HOME    segment code
	R_GSINIT  segment code
	R_IXSEG   segment xdata
	R_CONST   segment code
	R_XINIT   segment code
	R_DINIT   segment code

;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	public _cli_cmd_count
	public _cli_commands
	public _cmd_vol
	public _cmd_help
	public _cmd_psize
	public _cmd_enable
	public _cmd_disable
	public _cmd_info
	public _cmd_reset
	public _cli_parse_and_dispatch
;--------------------------------------------------------
; Special Function Registers
;--------------------------------------------------------
_ACC            DATA 0xe0
_ADC0ASAH       DATA 0xb6
_ADC0ASAL       DATA 0xb5
_ADC0ASCF       DATA 0xa1
_ADC0ASCT       DATA 0xc7
_ADC0CF0        DATA 0xbc
_ADC0CF1        DATA 0xb9
_ADC0CF2        DATA 0xdf
_ADC0CN0        DATA 0xe8
_ADC0CN1        DATA 0xb2
_ADC0CN2        DATA 0xb3
_ADC0GTH        DATA 0xc4
_ADC0GTL        DATA 0xc3
_ADC0H          DATA 0xbe
_ADC0L          DATA 0xbd
_ADC0LTH        DATA 0xc6
_ADC0LTL        DATA 0xc5
_ADC0MX         DATA 0xbb
_B              DATA 0xf0
_CKCON0         DATA 0x8e
_CKCON1         DATA 0xa6
_CLEN0          DATA 0xc6
_CLIE0          DATA 0xc7
_CLIF0          DATA 0xe8
_CLKSEL         DATA 0xa9
_CLOUT0         DATA 0xd1
_CLU0CF         DATA 0xb1
_CLU0FN         DATA 0xaf
_CLU0MX         DATA 0x84
_CLU1CF         DATA 0xb3
_CLU1FN         DATA 0xb2
_CLU1MX         DATA 0x85
_CLU2CF         DATA 0xb6
_CLU2FN         DATA 0xb5
_CLU2MX         DATA 0x91
_CLU3CF         DATA 0xbf
_CLU3FN         DATA 0xbe
_CLU3MX         DATA 0xae
_CMP0CN0        DATA 0x9b
_CMP0CN1        DATA 0x99
_CMP0MD         DATA 0x9d
_CMP0MX         DATA 0x9f
_CMP1CN0        DATA 0xbf
_CMP1CN1        DATA 0xac
_CMP1MD         DATA 0xab
_CMP1MX         DATA 0xaa
_CRC0CN0        DATA 0xce
_CRC0CN1        DATA 0x86
_CRC0CNT        DATA 0xd3
_CRC0DAT        DATA 0xcb
_CRC0FLIP       DATA 0xcf
_CRC0IN         DATA 0xca
_CRC0ST         DATA 0xd2
_DAC0CF0        DATA 0x91
_DAC0CF1        DATA 0x92
_DAC0H          DATA 0x85
_DAC0L          DATA 0x84
_DAC1CF0        DATA 0x93
_DAC1CF1        DATA 0x94
_DAC1H          DATA 0x8a
_DAC1L          DATA 0x89
_DAC2CF0        DATA 0x95
_DAC2CF1        DATA 0x96
_DAC2H          DATA 0x8c
_DAC2L          DATA 0x8b
_DAC3CF0        DATA 0x9a
_DAC3CF1        DATA 0x9c
_DAC3H          DATA 0x8e
_DAC3L          DATA 0x8d
_DACGCF0        DATA 0x88
_DACGCF1        DATA 0x98
_DACGCF2        DATA 0xa2
_DERIVID        DATA 0xad
_DEVICEID       DATA 0xb5
_DPH            DATA 0x83
_DPL            DATA 0x82
_EIE1           DATA 0xe6
_EIE2           DATA 0xf3
_EIP1           DATA 0xbb
_EIP1H          DATA 0xee
_EIP2           DATA 0xed
_EIP2H          DATA 0xf6
_EMI0CN         DATA 0xe7
_FLKEY          DATA 0xb7
_HFO0CAL        DATA 0xc7
_HFO1CAL        DATA 0xd6
_HFOCN          DATA 0xef
_I2C0ADM        DATA 0xff
_I2C0CN0        DATA 0xba
_I2C0DIN        DATA 0xbc
_I2C0DOUT       DATA 0xbb
_I2C0FCN0       DATA 0xad
_I2C0FCN1       DATA 0xab
_I2C0FCT        DATA 0xf5
_I2C0SLAD       DATA 0xbd
_I2C0STAT       DATA 0xb9
_IE             DATA 0xa8
_IP             DATA 0xb8
_IPH            DATA 0xf2
_IT01CF         DATA 0xe4
_LFO0CN         DATA 0xb1
_P0             DATA 0x80
_P0MASK         DATA 0xfe
_P0MAT          DATA 0xfd
_P0MDIN         DATA 0xf1
_P0MDOUT        DATA 0xa4
_P0SKIP         DATA 0xd4
_P1             DATA 0x90
_P1MASK         DATA 0xee
_P1MAT          DATA 0xed
_P1MDIN         DATA 0xf2
_P1MDOUT        DATA 0xa5
_P1SKIP         DATA 0xd5
_P2             DATA 0xa0
_P2MASK         DATA 0xfc
_P2MAT          DATA 0xfb
_P2MDIN         DATA 0xf3
_P2MDOUT        DATA 0xa6
_P2SKIP         DATA 0xcc
_P3             DATA 0xb0
_P3MDIN         DATA 0xf4
_P3MDOUT        DATA 0x9c
_PCA0CENT       DATA 0x9e
_PCA0CLR        DATA 0x9c
_PCA0CN0        DATA 0xd8
_PCA0CPH0       DATA 0xfc
_PCA0CPH1       DATA 0xea
_PCA0CPH2       DATA 0xec
_PCA0CPH3       DATA 0xf5
_PCA0CPH4       DATA 0x85
_PCA0CPH5       DATA 0xde
_PCA0CPL0       DATA 0xfb
_PCA0CPL1       DATA 0xe9
_PCA0CPL2       DATA 0xeb
_PCA0CPL3       DATA 0xf4
_PCA0CPL4       DATA 0x84
_PCA0CPL5       DATA 0xdd
_PCA0CPM0       DATA 0xda
_PCA0CPM1       DATA 0xdb
_PCA0CPM2       DATA 0xdc
_PCA0CPM3       DATA 0xae
_PCA0CPM4       DATA 0xaf
_PCA0CPM5       DATA 0xcc
_PCA0H          DATA 0xfa
_PCA0L          DATA 0xf9
_PCA0MD         DATA 0xd9
_PCA0POL        DATA 0x96
_PCA0PWM        DATA 0xf7
_PCON0          DATA 0x87
_PCON1          DATA 0xcd
_PFE0CN         DATA 0xc1
_PRTDRV         DATA 0xf6
_PSCTL          DATA 0x8f
_PSTAT0         DATA 0xaa
_PSW            DATA 0xd0
_REF0CN         DATA 0xd1
_REG0CN         DATA 0xc9
_REVID          DATA 0xb6
_RSTSRC         DATA 0xef
_SBCON1         DATA 0x94
_SBRLH1         DATA 0x96
_SBRLL1         DATA 0x95
_SBUF           DATA 0x99
_SBUF0          DATA 0x99
_SBUF1          DATA 0x92
_SCON           DATA 0x98
_SCON0          DATA 0x98
_SCON1          DATA 0xc8
_SFRPAGE        DATA 0xa7
_SFRPGCN        DATA 0xbc
_SFRSTACK       DATA 0xd7
_SMB0ADM        DATA 0xd6
_SMB0ADR        DATA 0xd7
_SMB0CF         DATA 0xc1
_SMB0CN0        DATA 0xc0
_SMB0DAT        DATA 0xc2
_SMB0FCN0       DATA 0xc3
_SMB0FCN1       DATA 0xc4
_SMB0FCT        DATA 0xef
_SMB0RXLN       DATA 0xc5
_SMB0TC         DATA 0xac
_SMOD1          DATA 0x93
_SP             DATA 0x81
_SPI0CFG        DATA 0xa1
_SPI0CKR        DATA 0xa2
_SPI0CN0        DATA 0xf8
_SPI0DAT        DATA 0xa3
_SPI0FCN0       DATA 0x9a
_SPI0FCN1       DATA 0x9b
_SPI0FCT        DATA 0xf7
_SPI0PCF        DATA 0xdf
_TCON           DATA 0x88
_TH0            DATA 0x8c
_TH1            DATA 0x8d
_TL0            DATA 0x8a
_TL1            DATA 0x8b
_TMOD           DATA 0x89
_TMR2CN0        DATA 0xc8
_TMR2CN1        DATA 0xfd
_TMR2H          DATA 0xcf
_TMR2L          DATA 0xce
_TMR2RLH        DATA 0xcb
_TMR2RLL        DATA 0xca
_TMR3CN0        DATA 0x91
_TMR3CN1        DATA 0xfe
_TMR3H          DATA 0x95
_TMR3L          DATA 0x94
_TMR3RLH        DATA 0x93
_TMR3RLL        DATA 0x92
_TMR4CN0        DATA 0x98
_TMR4CN1        DATA 0xff
_TMR4H          DATA 0xa5
_TMR4L          DATA 0xa4
_TMR4RLH        DATA 0xa3
_TMR4RLL        DATA 0xa2
_TMR5CN0        DATA 0xc0
_TMR5CN1        DATA 0xf1
_TMR5H          DATA 0xd5
_TMR5L          DATA 0xd4
_TMR5RLH        DATA 0xd3
_TMR5RLL        DATA 0xd2
_UART0PCF       DATA 0xd9
_UART1FCN0      DATA 0x9d
_UART1FCN1      DATA 0xd8
_UART1FCT       DATA 0xfa
_UART1LIN       DATA 0x9e
_UART1PCF       DATA 0xda
_VDM0CN         DATA 0xff
_WDTCN          DATA 0x97
_XBR0           DATA 0xe1
_XBR1           DATA 0xe2
_XBR2           DATA 0xe3
_XOSC0CN        DATA 0x86
_DPTR           DATA 0x8382
_TMR2RL         DATA 0xcbca
_TMR3RL         DATA 0x9392
_TMR4RL         DATA 0xa3a2
_TMR5RL         DATA 0xd3d2
_TMR0           DATA 0x8c8a
_TMR1           DATA 0x8d8b
_TMR2           DATA 0xcfce
_TMR3           DATA 0x9594
_TMR4           DATA 0xa5a4
_TMR5           DATA 0xd5d4
_SBRL1          DATA 0x9695
_PCA0           DATA 0xfaf9
_PCA0CP0        DATA 0xfcfb
_PCA0CP1        DATA 0xeae9
_PCA0CP2        DATA 0xeceb
_PCA0CP3        DATA 0xf5f4
_PCA0CP4        DATA 0x8584
_PCA0CP5        DATA 0xdedd
_ADC0ASA        DATA 0xb6b5
_ADC0GT         DATA 0xc4c3
_ADC0           DATA 0xbebd
_ADC0LT         DATA 0xc6c5
_DAC0           DATA 0x8584
_DAC1           DATA 0x8a89
_DAC2           DATA 0x8c8b
_DAC3           DATA 0x8e8d
;--------------------------------------------------------
; special function bits
;--------------------------------------------------------
_ACC_0          BIT 0xe0
_ACC_1          BIT 0xe1
_ACC_2          BIT 0xe2
_ACC_3          BIT 0xe3
_ACC_4          BIT 0xe4
_ACC_5          BIT 0xe5
_ACC_6          BIT 0xe6
_ACC_7          BIT 0xe7
_TEMPE          BIT 0xe8
_ADGN0          BIT 0xe9
_ADGN1          BIT 0xea
_ADWINT         BIT 0xeb
_ADBUSY         BIT 0xec
_ADINT          BIT 0xed
_IPOEN          BIT 0xee
_ADEN           BIT 0xef
_B_0            BIT 0xf0
_B_1            BIT 0xf1
_B_2            BIT 0xf2
_B_3            BIT 0xf3
_B_4            BIT 0xf4
_B_5            BIT 0xf5
_B_6            BIT 0xf6
_B_7            BIT 0xf7
_C0FIF          BIT 0xe8
_C0RIF          BIT 0xe9
_C1FIF          BIT 0xea
_C1RIF          BIT 0xeb
_C2FIF          BIT 0xec
_C2RIF          BIT 0xed
_C3FIF          BIT 0xee
_C3RIF          BIT 0xef
_D1SRC0         BIT 0x88
_D1SRC1         BIT 0x89
_D1AMEN         BIT 0x8a
_D01REFSL       BIT 0x8b
_D3SRC0         BIT 0x8c
_D3SRC1         BIT 0x8d
_D3AMEN         BIT 0x8e
_D23REFSL       BIT 0x8f
_D0UDIS         BIT 0x98
_D1UDIS         BIT 0x99
_D2UDIS         BIT 0x9a
_D3UDIS         BIT 0x9b
_EX0            BIT 0xa8
_ET0            BIT 0xa9
_EX1            BIT 0xaa
_ET1            BIT 0xab
_ES0            BIT 0xac
_ET2            BIT 0xad
_ESPI0          BIT 0xae
_EA             BIT 0xaf
_PX0            BIT 0xb8
_PT0            BIT 0xb9
_PX1            BIT 0xba
_PT1            BIT 0xbb
_PS0            BIT 0xbc
_PT2            BIT 0xbd
_PSPI0          BIT 0xbe
_P0_0           BIT 0x80
_P0_1           BIT 0x81
_P0_2           BIT 0x82
_P0_3           BIT 0x83
_P0_4           BIT 0x84
_P0_5           BIT 0x85
_P0_6           BIT 0x86
_P0_7           BIT 0x87
_P1_0           BIT 0x90
_P1_1           BIT 0x91
_P1_2           BIT 0x92
_P1_3           BIT 0x93
_P1_4           BIT 0x94
_P1_5           BIT 0x95
_P1_6           BIT 0x96
_P1_7           BIT 0x97
_P2_0           BIT 0xa0
_P2_1           BIT 0xa1
_P2_2           BIT 0xa2
_P2_3           BIT 0xa3
_P2_4           BIT 0xa4
_P2_5           BIT 0xa5
_P2_6           BIT 0xa6
_P3_0           BIT 0xb0
_P3_1           BIT 0xb1
_P3_2           BIT 0xb2
_P3_3           BIT 0xb3
_P3_4           BIT 0xb4
_P3_7           BIT 0xb7
_CCF0           BIT 0xd8
_CCF1           BIT 0xd9
_CCF2           BIT 0xda
_CCF3           BIT 0xdb
_CCF4           BIT 0xdc
_CCF5           BIT 0xdd
_CR             BIT 0xde
_CF             BIT 0xdf
_PARITY         BIT 0xd0
_F1             BIT 0xd1
_OV             BIT 0xd2
_RS0            BIT 0xd3
_RS1            BIT 0xd4
_F0             BIT 0xd5
_AC             BIT 0xd6
_CY             BIT 0xd7
_RI             BIT 0x98
_TI             BIT 0x99
_RB8            BIT 0x9a
_TB8            BIT 0x9b
_REN            BIT 0x9c
_CE             BIT 0x9d
_SMODE          BIT 0x9e
_RI1            BIT 0xc8
_TI1            BIT 0xc9
_RBX1           BIT 0xca
_TBX1           BIT 0xcb
_REN1           BIT 0xcc
_PERR1          BIT 0xcd
_OVR1           BIT 0xce
_SI             BIT 0xc0
_ACK            BIT 0xc1
_ARBLOST        BIT 0xc2
_ACKRQ          BIT 0xc3
_STO            BIT 0xc4
_STA            BIT 0xc5
_TXMODE         BIT 0xc6
_MASTER         BIT 0xc7
_SPIEN          BIT 0xf8
_TXNF           BIT 0xf9
_NSSMD0         BIT 0xfa
_NSSMD1         BIT 0xfb
_RXOVRN         BIT 0xfc
_MODF           BIT 0xfd
_WCOL           BIT 0xfe
_SPIF           BIT 0xff
_IT0            BIT 0x88
_IE0            BIT 0x89
_IT1            BIT 0x8a
_IE1            BIT 0x8b
_TR0            BIT 0x8c
_TF0            BIT 0x8d
_TR1            BIT 0x8e
_TF1            BIT 0x8f
_T2XCLK0        BIT 0xc8
_T2XCLK1        BIT 0xc9
_TR2            BIT 0xca
_T2SPLIT        BIT 0xcb
_TF2CEN         BIT 0xcc
_TF2LEN         BIT 0xcd
_TF2L           BIT 0xce
_TF2H           BIT 0xcf
_T4XCLK0        BIT 0x98
_T4XCLK1        BIT 0x99
_TR4            BIT 0x9a
_T4SPLIT        BIT 0x9b
_TF4CEN         BIT 0x9c
_TF4LEN         BIT 0x9d
_TF4L           BIT 0x9e
_TF4H           BIT 0x9f
_T5XCLK0        BIT 0xc0
_T5XCLK1        BIT 0xc1
_TR5            BIT 0xc2
_T5SPLIT        BIT 0xc3
_TF5CEN         BIT 0xc4
_TF5LEN         BIT 0xc5
_TF5L           BIT 0xc6
_TF5H           BIT 0xc7
_RIE            BIT 0xd8
_RXTO0          BIT 0xd9
_RXTO1          BIT 0xda
_RFRQ           BIT 0xdb
_TIE            BIT 0xdc
_TXHOLD         BIT 0xdd
_TXNF1          BIT 0xde
_TFRQ           BIT 0xdf
;--------------------------------------------------------
; overlayable register banks
;--------------------------------------------------------
	rbank0 segment data overlay
;--------------------------------------------------------
; internal ram data
;--------------------------------------------------------
	rseg R_DSEG
_cli_parse_and_dispatch_input_1_69:
	ds 3
_cli_parse_and_dispatch_in_arg_1_70:
	ds 1
;--------------------------------------------------------
; overlayable items in internal ram 
;--------------------------------------------------------
	rseg R_OSEG
;--------------------------------------------------------
; indirectly addressable internal ram data
;--------------------------------------------------------
	rseg R_ISEG
;--------------------------------------------------------
; absolute internal ram data
;--------------------------------------------------------
	DSEG
;--------------------------------------------------------
; bit data
;--------------------------------------------------------
	rseg R_BSEG
;--------------------------------------------------------
; paged external ram data
;--------------------------------------------------------
	rseg R_PSEG
;--------------------------------------------------------
; external ram data
;--------------------------------------------------------
	rseg R_XSEG
_cli_parse_and_dispatch_argv_1_70:
	ds 24
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	XSEG
;--------------------------------------------------------
; external initialized ram data
;--------------------------------------------------------
	rseg R_IXSEG
	rseg R_HOME
	rseg R_GSINIT
	rseg R_CSEG
;--------------------------------------------------------
; Interrupt vectors
;--------------------------------------------------------
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	rseg R_HOME
	rseg R_GSINIT
	rseg R_GSINIT
;--------------------------------------------------------
; data variables initialization
;--------------------------------------------------------
	rseg R_DINIT
	; The linker places a 'ret' at the end of segment R_DINIT.
;--------------------------------------------------------
; code
;--------------------------------------------------------
	rseg R_CSEG
;------------------------------------------------------------
;Allocation info for local variables in function 'cmd_help'
;------------------------------------------------------------
;argv                      Allocated to stack - offset -5
;argc                      Allocated to registers 
;i                         Allocated to registers r2 
;------------------------------------------------------------
;	src/cmd.c:6: void cmd_help(unsigned char argc, char *argv[]) reentrant {
;	-----------------------------------------
;	 function cmd_help
;	-----------------------------------------
_cmd_help:
	using	0
	push	_bp
	mov	_bp,sp
;	src/cmd.c:8: UART1_send_string("Usage: tibo <cmd> <args>\r\n");
	mov	dptr,#__str_0
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:9: UART1_send_string("Available commands:\r\n");
	mov	dptr,#__str_1
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:10: for (i = 0; i < cli_cmd_count; i++)
	mov	r2,#0x00
L002001?:
	mov	dptr,#_cli_cmd_count
	clr	a
	movc	a,@a+dptr
	mov	r3,a
	clr	c
	mov	a,r2
	subb	a,r3
	jc	L002010?
	ljmp	L002005?
L002010?:
;	src/cmd.c:12: UART1_send_string("  tibo ");
	mov	dptr,#__str_2
	mov	b,#0x80
	push	ar2
	lcall	_UART1_send_string
	pop	ar2
;	src/cmd.c:13: UART1_send_string(cli_commands[i].name);
	mov	a,r2
	mov	b,#0x0B
	mul	ab
	mov	r3,a
	mov	r4,b
	add	a,#_cli_commands
	mov	r5,a
	mov	a,r4
	addc	a,#(_cli_commands >> 8)
	mov	r6,a
	mov	dpl,r5
	mov	dph,r6
	inc	dptr
	inc	dptr
	inc	dptr
	clr	a
	movc	a,@a+dptr
	mov	r5,a
	inc	dptr
	clr	a
	movc	a,@a+dptr
	mov	r6,a
	inc	dptr
	clr	a
	movc	a,@a+dptr
	mov	r7,a
	mov	dpl,r5
	mov	dph,r6
	mov	b,r7
	push	ar2
	push	ar3
	push	ar4
	lcall	_UART1_send_string
;	src/cmd.c:14: UART1_send_string(" - ");
	mov	dptr,#__str_3
	mov	b,#0x80
	lcall	_UART1_send_string
	pop	ar4
	pop	ar3
;	src/cmd.c:15: UART1_send_string(cli_commands[i].help);
	mov	a,r3
	add	a,#_cli_commands
	mov	r3,a
	mov	a,r4
	addc	a,#(_cli_commands >> 8)
	mov	r4,a
	mov	a,#0x06
	add	a,r3
	mov	dpl,a
	clr	a
	addc	a,r4
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	r3,a
	inc	dptr
	clr	a
	movc	a,@a+dptr
	mov	r4,a
	inc	dptr
	clr	a
	movc	a,@a+dptr
	mov	r5,a
	mov	dpl,r3
	mov	dph,r4
	mov	b,r5
	lcall	_UART1_send_string
;	src/cmd.c:16: UART1_send_string("\r\n");
	mov	dptr,#__str_4
	mov	b,#0x80
	lcall	_UART1_send_string
	pop	ar2
;	src/cmd.c:10: for (i = 0; i < cli_cmd_count; i++)
	inc	r2
	ljmp	L002001?
L002005?:
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'cmd_psize'
;------------------------------------------------------------
;argv                      Allocated to stack - offset -5
;argc                      Allocated to registers r2 
;------------------------------------------------------------
;	src/cmd.c:20: void cmd_psize(unsigned char argc, char *argv[]) reentrant {
;	-----------------------------------------
;	 function cmd_psize
;	-----------------------------------------
_cmd_psize:
	push	_bp
	mov	_bp,sp
	mov	r2,dpl
;	src/cmd.c:21: if (argc < 2) {
	cjne	r2,#0x02,L003014?
L003014?:
	jnc	L003002?
;	src/cmd.c:22: UART1_send_string("Usage: tibo psize -b|-s\r\n");
	mov	dptr,#__str_5
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:23: return;
	ljmp	L003009?
L003002?:
;	src/cmd.c:25: if (strcmp(argv[1], "-b") == 0) {
	mov	a,_bp
	add	a,#0xfb
	mov	r0,a
	mov	a,#0x03
	add	a,@r0
	mov	r2,a
	clr	a
	inc	r0
	addc	a,@r0
	mov	r3,a
	inc	r0
	mov	ar4,@r0
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r5,a
	inc	dptr
	lcall	__gptrget
	mov	r6,a
	inc	dptr
	lcall	__gptrget
	mov	r7,a
	mov	_strcmp_PARM_2,#__str_6
	mov	(_strcmp_PARM_2 + 1),#(__str_6 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dpl,r5
	mov	dph,r6
	mov	b,r7
	push	ar2
	push	ar3
	push	ar4
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	pop	ar4
	pop	ar3
	pop	ar2
	orl	a,b
	jnz	L003007?
;	src/cmd.c:26: UART_send_string("!psize:b\n");   // to Python via UART0
	mov	dptr,#__str_7
	mov	b,#0x80
	lcall	_UART_send_string
	sjmp	L003009?
L003007?:
;	src/cmd.c:27: } else if (strcmp(argv[1], "-s") == 0) {
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r2,a
	inc	dptr
	lcall	__gptrget
	mov	r3,a
	inc	dptr
	lcall	__gptrget
	mov	r4,a
	mov	_strcmp_PARM_2,#__str_8
	mov	(_strcmp_PARM_2 + 1),#(__str_8 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	orl	a,b
	jnz	L003004?
;	src/cmd.c:28: UART_send_string("!psize:s\n");   // to Python via UART0
	mov	dptr,#__str_9
	mov	b,#0x80
	lcall	_UART_send_string
	sjmp	L003009?
L003004?:
;	src/cmd.c:30: UART1_send_string("Usage: tibo psize -b|-s\r\n");
	mov	dptr,#__str_5
	mov	b,#0x80
	lcall	_UART1_send_string
L003009?:
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'cmd_enable'
;------------------------------------------------------------
;argv                      Allocated to stack - offset -5
;argc                      Allocated to registers r2 
;------------------------------------------------------------
;	src/cmd.c:34: void cmd_enable(unsigned char argc, char *argv[]) __reentrant {
;	-----------------------------------------
;	 function cmd_enable
;	-----------------------------------------
_cmd_enable:
	push	_bp
	mov	_bp,sp
	mov	r2,dpl
;	src/cmd.c:35: if (argc < 2) {
	cjne	r2,#0x02,L004014?
L004014?:
	jnc	L004002?
;	src/cmd.c:36: UART1_send_string("Usage: tibo enable -1|-2\r\n");
	mov	dptr,#__str_10
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:37: return;
	ljmp	L004009?
L004002?:
;	src/cmd.c:39: if (strcmp(argv[1], "-1") == 0) {
	mov	a,_bp
	add	a,#0xfb
	mov	r0,a
	mov	a,#0x03
	add	a,@r0
	mov	r2,a
	clr	a
	inc	r0
	addc	a,@r0
	mov	r3,a
	inc	r0
	mov	ar4,@r0
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r5,a
	inc	dptr
	lcall	__gptrget
	mov	r6,a
	inc	dptr
	lcall	__gptrget
	mov	r7,a
	mov	_strcmp_PARM_2,#__str_11
	mov	(_strcmp_PARM_2 + 1),#(__str_11 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dpl,r5
	mov	dph,r6
	mov	b,r7
	push	ar2
	push	ar3
	push	ar4
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	pop	ar4
	pop	ar3
	pop	ar2
	orl	a,b
	jnz	L004007?
;	src/cmd.c:40: UART_send_string("!ch1:on\n");
	mov	dptr,#__str_12
	mov	b,#0x80
	lcall	_UART_send_string
;	src/cmd.c:41: UART1_send_string("CH1: enabled\r\n");
	mov	dptr,#__str_13
	mov	b,#0x80
	lcall	_UART1_send_string
	sjmp	L004009?
L004007?:
;	src/cmd.c:42: } else if (strcmp(argv[1], "-2") == 0) {
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r2,a
	inc	dptr
	lcall	__gptrget
	mov	r3,a
	inc	dptr
	lcall	__gptrget
	mov	r4,a
	mov	_strcmp_PARM_2,#__str_14
	mov	(_strcmp_PARM_2 + 1),#(__str_14 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	orl	a,b
	jnz	L004004?
;	src/cmd.c:43: UART_send_string("!ch2:on\n");
	mov	dptr,#__str_15
	mov	b,#0x80
	lcall	_UART_send_string
;	src/cmd.c:44: UART1_send_string("CH2: enabled\r\n");
	mov	dptr,#__str_16
	mov	b,#0x80
	lcall	_UART1_send_string
	sjmp	L004009?
L004004?:
;	src/cmd.c:46: UART1_send_string("Usage: tibo enable -1|-2\r\n");
	mov	dptr,#__str_10
	mov	b,#0x80
	lcall	_UART1_send_string
L004009?:
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'cmd_disable'
;------------------------------------------------------------
;argv                      Allocated to stack - offset -5
;argc                      Allocated to registers r2 
;------------------------------------------------------------
;	src/cmd.c:50: void cmd_disable(unsigned char argc, char *argv[]) reentrant {
;	-----------------------------------------
;	 function cmd_disable
;	-----------------------------------------
_cmd_disable:
	push	_bp
	mov	_bp,sp
	mov	r2,dpl
;	src/cmd.c:51: if (argc < 2) {
	cjne	r2,#0x02,L005014?
L005014?:
	jnc	L005002?
;	src/cmd.c:52: UART1_send_string("Usage: tibo disable -1|-2\r\n");
	mov	dptr,#__str_17
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:53: return;
	ljmp	L005009?
L005002?:
;	src/cmd.c:55: if (strcmp(argv[1], "-1") == 0) {
	mov	a,_bp
	add	a,#0xfb
	mov	r0,a
	mov	a,#0x03
	add	a,@r0
	mov	r2,a
	clr	a
	inc	r0
	addc	a,@r0
	mov	r3,a
	inc	r0
	mov	ar4,@r0
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r5,a
	inc	dptr
	lcall	__gptrget
	mov	r6,a
	inc	dptr
	lcall	__gptrget
	mov	r7,a
	mov	_strcmp_PARM_2,#__str_11
	mov	(_strcmp_PARM_2 + 1),#(__str_11 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dpl,r5
	mov	dph,r6
	mov	b,r7
	push	ar2
	push	ar3
	push	ar4
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	pop	ar4
	pop	ar3
	pop	ar2
	orl	a,b
	jnz	L005007?
;	src/cmd.c:56: UART_send_string("!ch1:off\n");
	mov	dptr,#__str_18
	mov	b,#0x80
	lcall	_UART_send_string
;	src/cmd.c:57: UART1_send_string("CH1: disabled\r\n");
	mov	dptr,#__str_19
	mov	b,#0x80
	lcall	_UART1_send_string
	sjmp	L005009?
L005007?:
;	src/cmd.c:58: } else if (strcmp(argv[1], "-2") == 0) {
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r2,a
	inc	dptr
	lcall	__gptrget
	mov	r3,a
	inc	dptr
	lcall	__gptrget
	mov	r4,a
	mov	_strcmp_PARM_2,#__str_14
	mov	(_strcmp_PARM_2 + 1),#(__str_14 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	orl	a,b
	jnz	L005004?
;	src/cmd.c:59: UART_send_string("!ch2:off\n");
	mov	dptr,#__str_20
	mov	b,#0x80
	lcall	_UART_send_string
;	src/cmd.c:60: UART1_send_string("CH2: disabled\r\n");
	mov	dptr,#__str_21
	mov	b,#0x80
	lcall	_UART1_send_string
	sjmp	L005009?
L005004?:
;	src/cmd.c:62: UART1_send_string("Usage: tibo disable -1|-2\r\n");
	mov	dptr,#__str_17
	mov	b,#0x80
	lcall	_UART1_send_string
L005009?:
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'cmd_vol'
;------------------------------------------------------------
;argv                      Allocated to stack - offset -5
;argc                      Allocated to registers r2 
;------------------------------------------------------------
;	src/cmd.c:66: void cmd_vol (unsigned char argc, char *argv[]) reentrant {
;	-----------------------------------------
;	 function cmd_vol
;	-----------------------------------------
_cmd_vol:
	push	_bp
	mov	_bp,sp
	mov	r2,dpl
;	src/cmd.c:67: if (argc < 2) {
	cjne	r2,#0x02,L006014?
L006014?:
	jnc	L006002?
;	src/cmd.c:68: UART1_send_string("Usage: tibo vol -vrms|vpp-2\r\n");
	mov	dptr,#__str_22
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:69: return;
	ljmp	L006009?
L006002?:
;	src/cmd.c:71: if (strcmp(argv[1], "-vrms") == 0) {
	mov	a,_bp
	add	a,#0xfb
	mov	r0,a
	mov	a,#0x03
	add	a,@r0
	mov	r2,a
	clr	a
	inc	r0
	addc	a,@r0
	mov	r3,a
	inc	r0
	mov	ar4,@r0
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r5,a
	inc	dptr
	lcall	__gptrget
	mov	r6,a
	inc	dptr
	lcall	__gptrget
	mov	r7,a
	mov	_strcmp_PARM_2,#__str_23
	mov	(_strcmp_PARM_2 + 1),#(__str_23 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dpl,r5
	mov	dph,r6
	mov	b,r7
	push	ar2
	push	ar3
	push	ar4
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	pop	ar4
	pop	ar3
	pop	ar2
	orl	a,b
	jnz	L006007?
;	src/cmd.c:72: rms_vpp_signal = 0;
	clr	_rms_vpp_signal
	sjmp	L006009?
L006007?:
;	src/cmd.c:73: } else if (strcmp(argv[1], "-vpp") == 0) {
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	__gptrget
	mov	r2,a
	inc	dptr
	lcall	__gptrget
	mov	r3,a
	inc	dptr
	lcall	__gptrget
	mov	r4,a
	mov	_strcmp_PARM_2,#__str_24
	mov	(_strcmp_PARM_2 + 1),#(__str_24 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	orl	a,b
	jnz	L006004?
;	src/cmd.c:74: rms_vpp_signal = 1;
	setb	_rms_vpp_signal
	sjmp	L006009?
L006004?:
;	src/cmd.c:76: UART1_send_string("Usage: tibo vol -vrms|vpp-2\r\n");
	mov	dptr,#__str_22
	mov	b,#0x80
	lcall	_UART1_send_string
L006009?:
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'cmd_info'
;------------------------------------------------------------
;argv                      Allocated to stack - offset -5
;argc                      Allocated to registers 
;------------------------------------------------------------
;	src/cmd.c:80: void cmd_info(unsigned char argc, char *argv[]) reentrant {
;	-----------------------------------------
;	 function cmd_info
;	-----------------------------------------
_cmd_info:
	push	_bp
	mov	_bp,sp
;	src/cmd.c:81: UART1_send_string("EFM8 Frequency measurement using Timer/Counter 0.\r\n");
	mov	dptr,#__str_25
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:82: UART1_send_string("File: ");
	mov	dptr,#__str_26
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:83: UART1_send_string(__FILE__);
	mov	dptr,#__str_27
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:84: UART1_send_string("\r\nCompiled: ");
	mov	dptr,#__str_28
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:85: UART1_send_string(__DATE__);
	mov	dptr,#__str_29
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:86: UART1_send_string(", ");
	mov	dptr,#__str_30
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:87: UART1_send_string(__TIME__);
	mov	dptr,#__str_31
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:88: UART1_send_string("\r\n");
	mov	dptr,#__str_4
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:89: return;
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'cmd_reset'
;------------------------------------------------------------
;argv                      Allocated to stack - offset -5
;argc                      Allocated to registers 
;------------------------------------------------------------
;	src/cmd.c:92: void cmd_reset(unsigned char argc, char *argv[]) reentrant {
;	-----------------------------------------
;	 function cmd_reset
;	-----------------------------------------
_cmd_reset:
	push	_bp
	mov	_bp,sp
;	src/cmd.c:93: UART1_send_string("System Resetting...\r\n");
	mov	dptr,#__str_32
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:94: RSTSRC = 0x10;
	mov	_RSTSRC,#0x10
	pop	_bp
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'cli_parse_and_dispatch'
;------------------------------------------------------------
;input                     Allocated with name '_cli_parse_and_dispatch_input_1_69'
;argc                      Allocated to registers r5 
;i                         Allocated to registers r0 
;in_arg                    Allocated with name '_cli_parse_and_dispatch_in_arg_1_70'
;argv                      Allocated with name '_cli_parse_and_dispatch_argv_1_70'
;------------------------------------------------------------
;	src/cmd.c:109: void cli_parse_and_dispatch(char *input) {
;	-----------------------------------------
;	 function cli_parse_and_dispatch
;	-----------------------------------------
_cli_parse_and_dispatch:
	mov	_cli_parse_and_dispatch_input_1_69,dpl
	mov	(_cli_parse_and_dispatch_input_1_69 + 1),dph
	mov	(_cli_parse_and_dispatch_input_1_69 + 2),b
;	src/cmd.c:110: unsigned char argc = 0;
	mov	r5,#0x00
;	src/cmd.c:113: char in_arg = 0;
	mov	_cli_parse_and_dispatch_in_arg_1_70,#0x00
;	src/cmd.c:115: for (i = 0; input[i] != '\0'; i++) {
	mov	r7,#0x00
	mov	r0,#0x00
L009017?:
	mov	a,r0
	add	a,_cli_parse_and_dispatch_input_1_69
	mov	r1,a
	clr	a
	addc	a,(_cli_parse_and_dispatch_input_1_69 + 1)
	mov	r6,a
	mov	r2,(_cli_parse_and_dispatch_input_1_69 + 2)
	mov	dpl,r1
	mov	dph,r6
	mov	b,r2
	lcall	__gptrget
	mov	r3,a
	jz	L009039?
;	src/cmd.c:116: if (input[i] == ' ') {
	cjne	r3,#0x20,L009006?
;	src/cmd.c:117: input[i] = '\0';
	mov	dpl,r1
	mov	dph,r6
	mov	b,r2
	clr	a
	lcall	__gptrput
;	src/cmd.c:118: in_arg = 0;
	mov	_cli_parse_and_dispatch_in_arg_1_70,#0x00
	sjmp	L009019?
L009006?:
;	src/cmd.c:120: else if (!in_arg) {
	mov	a,_cli_parse_and_dispatch_in_arg_1_70
	jnz	L009019?
;	src/cmd.c:121: if (argc < 8) {
	cjne	r7,#0x08,L009044?
L009044?:
	jnc	L009002?
;	src/cmd.c:122: argv[argc++] = &input[i];
	mov	ar3,r7
	inc	r7
	mov	ar5,r7
	mov	a,r3
	mov	b,#0x03
	mul	ab
	add	a,#_cli_parse_and_dispatch_argv_1_70
	mov	dpl,a
	clr	a
	addc	a,#(_cli_parse_and_dispatch_argv_1_70 >> 8)
	mov	dph,a
	mov	a,r1
	movx	@dptr,a
	inc	dptr
	mov	a,r6
	movx	@dptr,a
	inc	dptr
	mov	a,r2
	movx	@dptr,a
L009002?:
;	src/cmd.c:124: in_arg = 1;
	mov	_cli_parse_and_dispatch_in_arg_1_70,#0x01
L009019?:
;	src/cmd.c:115: for (i = 0; input[i] != '\0'; i++) {
	inc	r0
	sjmp	L009017?
L009039?:
;	src/cmd.c:128: if (argc == 0) return;
	mov	a,r7
	mov	r5,a
	jnz	L009009?
	ret
L009009?:
;	src/cmd.c:130: if (strcmp(argv[0], "tibo") != 0) {
	mov	dptr,#_cli_parse_and_dispatch_argv_1_70
	movx	a,@dptr
	mov	r2,a
	inc	dptr
	movx	a,@dptr
	mov	r3,a
	inc	dptr
	movx	a,@dptr
	mov	r4,a
	mov	_strcmp_PARM_2,#__str_33
	mov	(_strcmp_PARM_2 + 1),#(__str_33 >> 8)
	mov	(_strcmp_PARM_2 + 2),#0x80
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	push	ar5
	push	ar7
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	pop	ar7
	pop	ar5
	orl	a,b
	jz	L009011?
;	src/cmd.c:131: UART1_send_string("Usage: tibo <cmd> <args>\r\n");
	mov	dptr,#__str_0
	mov	b,#0x80
;	src/cmd.c:132: return;
	ljmp	_UART1_send_string
L009011?:
;	src/cmd.c:135: if (argc < 2) {
	cjne	r7,#0x02,L009048?
L009048?:
	jnc	L009038?
;	src/cmd.c:136: UART1_send_string("Usage: tibo <cmd> <args>\r\n");
	mov	dptr,#__str_0
	mov	b,#0x80
;	src/cmd.c:137: return;
	ljmp	_UART1_send_string
;	src/cmd.c:140: for (i = 0; i < cli_cmd_count; i++) {
L009038?:
	mov	r2,#0x00
L009021?:
	mov	dptr,#_cli_cmd_count
	clr	a
	movc	a,@a+dptr
	mov	r3,a
	clr	c
	mov	a,r2
	subb	a,r3
	jc	L009050?
	ljmp	L009024?
L009050?:
;	src/cmd.c:141: if ((strcmp(argv[0], cli_commands[i].cli_name) == 0) &&
	mov	dptr,#_cli_parse_and_dispatch_argv_1_70
	movx	a,@dptr
	mov	r3,a
	inc	dptr
	movx	a,@dptr
	mov	r4,a
	inc	dptr
	movx	a,@dptr
	mov	r6,a
	mov	a,r2
	mov	b,#0x0B
	mul	ab
	mov	r7,a
	mov	b,a
	mov	dptr,#_cli_commands
	movc	a,@a+dptr
	mov	_strcmp_PARM_2,a
	inc	dptr
	mov	a,b
	movc	a,@a+dptr
	mov	(_strcmp_PARM_2 + 1),a
	inc	dptr
	mov	a,b
	movc	a,@a+dptr
	mov	(_strcmp_PARM_2 + 2),a
	mov	dpl,r3
	mov	dph,r4
	mov	b,r6
	push	ar2
	push	ar5
	push	ar7
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	pop	ar7
	pop	ar5
	pop	ar2
	orl	a,b
	jz	L009051?
	ljmp	L009023?
L009051?:
;	src/cmd.c:142: (strcmp(argv[1], cli_commands[i].name) == 0))
	mov	dptr,#(_cli_parse_and_dispatch_argv_1_70 + 0x0003)
	movx	a,@dptr
	mov	r3,a
	inc	dptr
	movx	a,@dptr
	mov	r4,a
	inc	dptr
	movx	a,@dptr
	mov	r6,a
	mov	a,r7
	add	a,#_cli_commands
	mov	r0,a
	clr	a
	addc	a,#(_cli_commands >> 8)
	mov	r1,a
	mov	dpl,r0
	mov	dph,r1
	inc	dptr
	inc	dptr
	inc	dptr
	clr	a
	movc	a,@a+dptr
	mov	_strcmp_PARM_2,a
	inc	dptr
	clr	a
	movc	a,@a+dptr
	mov	(_strcmp_PARM_2 + 1),a
	inc	dptr
	clr	a
	movc	a,@a+dptr
	mov	(_strcmp_PARM_2 + 2),a
	mov	dpl,r3
	mov	dph,r4
	mov	b,r6
	push	ar2
	push	ar5
	push	ar7
	lcall	_strcmp
	mov	a,dpl
	mov	b,dph
	pop	ar7
	pop	ar5
	pop	ar2
	orl	a,b
	jnz	L009023?
;	src/cmd.c:144: cli_commands[i].handler(argc - 1, &argv[1]);
	mov	a,r7
	add	a,#_cli_commands
	mov	r7,a
	clr	a
	addc	a,#(_cli_commands >> 8)
	mov	r3,a
	mov	a,#0x09
	add	a,r7
	mov	dpl,a
	clr	a
	addc	a,r3
	mov	dph,a
	clr	a
	movc	a,@a+dptr
	mov	r3,a
	inc	dptr
	clr	a
	movc	a,@a+dptr
	mov	r4,a
	mov	a,r5
	dec	a
	mov	dpl,a
	push	ar3
	push	ar4
	mov	a,#(_cli_parse_and_dispatch_argv_1_70 + 0x0003)
	push	acc
	mov	a,#((_cli_parse_and_dispatch_argv_1_70 + 0x0003) >> 8)
	push	acc
	clr	a
	push	acc
	mov	a,#L009053?
	push	acc
	mov	a,#(L009053? >> 8)
	push	acc
	push	ar3
	push	ar4
	ret
L009053?:
	dec	sp
	dec	sp
	dec	sp
	pop	ar4
	pop	ar3
;	src/cmd.c:145: return;
	ret
L009023?:
;	src/cmd.c:140: for (i = 0; i < cli_cmd_count; i++) {
	inc	r2
	ljmp	L009021?
L009024?:
;	src/cmd.c:148: UART1_send_string("Unknown command: ");
	mov	dptr,#__str_34
	mov	b,#0x80
	lcall	_UART1_send_string
;	src/cmd.c:149: UART1_send_string(argv[1]);
	mov	dptr,#(_cli_parse_and_dispatch_argv_1_70 + 0x0003)
	movx	a,@dptr
	mov	r2,a
	inc	dptr
	movx	a,@dptr
	mov	r3,a
	inc	dptr
	movx	a,@dptr
	mov	r4,a
	mov	dpl,r2
	mov	dph,r3
	mov	b,r4
	lcall	_UART1_send_string
;	src/cmd.c:150: UART1_send_string("\r\n");
	mov	dptr,#__str_4
	mov	b,#0x80
	ljmp	_UART1_send_string
	rseg R_CSEG

	rseg R_XINIT

	rseg R_CONST
__str_0:
	db 'Usage: tibo <cmd> <args>'
	db 0x0D
	db 0x0A
	db 0x00
__str_1:
	db 'Available commands:'
	db 0x0D
	db 0x0A
	db 0x00
__str_2:
	db '  tibo '
	db 0x00
__str_3:
	db ' - '
	db 0x00
__str_4:
	db 0x0D
	db 0x0A
	db 0x00
__str_5:
	db 'Usage: tibo psize -b|-s'
	db 0x0D
	db 0x0A
	db 0x00
__str_6:
	db '-b'
	db 0x00
__str_7:
	db '!psize:b'
	db 0x0A
	db 0x00
__str_8:
	db '-s'
	db 0x00
__str_9:
	db '!psize:s'
	db 0x0A
	db 0x00
__str_10:
	db 'Usage: tibo enable -1|-2'
	db 0x0D
	db 0x0A
	db 0x00
__str_11:
	db '-1'
	db 0x00
__str_12:
	db '!ch1:on'
	db 0x0A
	db 0x00
__str_13:
	db 'CH1: enabled'
	db 0x0D
	db 0x0A
	db 0x00
__str_14:
	db '-2'
	db 0x00
__str_15:
	db '!ch2:on'
	db 0x0A
	db 0x00
__str_16:
	db 'CH2: enabled'
	db 0x0D
	db 0x0A
	db 0x00
__str_17:
	db 'Usage: tibo disable -1|-2'
	db 0x0D
	db 0x0A
	db 0x00
__str_18:
	db '!ch1:off'
	db 0x0A
	db 0x00
__str_19:
	db 'CH1: disabled'
	db 0x0D
	db 0x0A
	db 0x00
__str_20:
	db '!ch2:off'
	db 0x0A
	db 0x00
__str_21:
	db 'CH2: disabled'
	db 0x0D
	db 0x0A
	db 0x00
__str_22:
	db 'Usage: tibo vol -vrms|vpp-2'
	db 0x0D
	db 0x0A
	db 0x00
__str_23:
	db '-vrms'
	db 0x00
__str_24:
	db '-vpp'
	db 0x00
__str_25:
	db 'EFM8 Frequency measurement using Timer/Counter 0.'
	db 0x0D
	db 0x0A
	db 0x00
__str_26:
	db 'File: '
	db 0x00
__str_27:
	db 'src/cmd.c'
	db 0x00
__str_28:
	db 0x0D
	db 0x0A
	db 'Compiled: '
	db 0x00
__str_29:
	db 'Mar  9 2026'
	db 0x00
__str_30:
	db ', '
	db 0x00
__str_31:
	db '10:37:24'
	db 0x00
__str_32:
	db 'System Resetting...'
	db 0x0D
	db 0x0A
	db 0x00
_cli_commands:
	db __str_33,(__str_33 >> 8),0x80
	db _str_35,(_str_35 >> 8),0x80
	db _str_36,(_str_36 >> 8),0x80
	db _cmd_help,(_cmd_help >> 8)
	db __str_33,(__str_33 >> 8),0x80
	db _str_37,(_str_37 >> 8),0x80
	db _str_38,(_str_38 >> 8),0x80
	db _cmd_psize,(_cmd_psize >> 8)
	db __str_33,(__str_33 >> 8),0x80
	db _str_39,(_str_39 >> 8),0x80
	db _str_40,(_str_40 >> 8),0x80
	db _cmd_enable,(_cmd_enable >> 8)
	db __str_33,(__str_33 >> 8),0x80
	db _str_41,(_str_41 >> 8),0x80
	db _str_42,(_str_42 >> 8),0x80
	db _cmd_disable,(_cmd_disable >> 8)
	db __str_33,(__str_33 >> 8),0x80
	db _str_43,(_str_43 >> 8),0x80
	db _str_44,(_str_44 >> 8),0x80
	db _cmd_vol,(_cmd_vol >> 8)
	db __str_33,(__str_33 >> 8),0x80
	db _str_45,(_str_45 >> 8),0x80
	db _str_46,(_str_46 >> 8),0x80
	db _cmd_info,(_cmd_info >> 8)
	db __str_33,(__str_33 >> 8),0x80
	db _str_47,(_str_47 >> 8),0x80
	db _str_48,(_str_48 >> 8),0x80
	db _cmd_reset,(_cmd_reset >> 8)
_cli_cmd_count:
	db 0x07	; 7 
__str_33:
	db 'tibo'
	db 0x00
__str_34:
	db 'Unknown command: '
	db 0x00
_str_35:
	db '--help'
	db 0x00
_str_36:
	db 'Show all commands'
	db 0x00
_str_37:
	db 'psize'
	db 0x00
_str_38:
	db 'Modify the size of Window'
	db 0x00
_str_39:
	db 'enable'
	db 0x00
_str_40:
	db 'Display CH1/CH2'
	db 0x00
_str_41:
	db 'disable'
	db 0x00
_str_42:
	db 'Shutdown CH1/CH2 display'
	db 0x00
_str_43:
	db 'vol'
	db 0x00
_str_44:
	db 'Toggle Vrms/Vpp display'
	db 0x00
_str_45:
	db 'info'
	db 0x00
_str_46:
	db 'Show device info'
	db 0x00
_str_47:
	db '--reset'
	db 0x00
_str_48:
	db 'Reset the System'
	db 0x00

	CSEG

end
