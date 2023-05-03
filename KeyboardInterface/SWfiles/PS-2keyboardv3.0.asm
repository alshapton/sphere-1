/*
 * PS_2keyboardv3.asm
 *
 *  Created: 4/11/2014 8:21:36 AM
 *   Author: mwillega
 */ 

 	.LIST
;	.include "tn2313def.inc"
	.LISTMAC

; r0 
	.DEF	bitcount=r0			;used to count in recieve PS2 bits
; r1
	.DEF	parity=r1			; used to maintain parity
; r2 
	.DEF	datain=r2			; input register to background process
; r3 
	.DEF	ps2leds=r3			; PS2 keyboard LED status
; r4
	.DEF	ps2outdata=r4		; PS2 keyboard CMD for output
; r5							; additional key state register
	.DEF	kstate = r5
; r6
	.DEF	clrstate=r6			; use to track clear (apple-1) sequence
;r7
	.DEF	cal_val=r7
;r8								; counters for timers
	.DEF	timer1=r8
;r9
	.DEF	timer2=r9
;r10
	.DEF	timer3=r10
;r11
	.DEF	ps2initialized=r11	; set when PS/2 initiallized
; r15 is used as zero
	.DEF	zero=r15

; r16 state
	.DEF	rststate=r16
; r17 available for use
	.DEF	rxstate=r17
; r18 - ps2 data accumulator
	.DEF	ps2data=r18			;accumulator for PS2 characters
; R19 - used in interrupt handling code, only
	.DEF	intscratch = r19
; r20,r21,r22,r23 scratch, scratch1, scratch2, scratch3
	.DEF	scratch = r20
	.DEF	scratch1 = r21
	.DEF	scratch2 = r22
; 
	.DEF	bdataout = r23
	.DEF	ddataout = r24
; r25 - state register
	.DEF	state = r25

; r26 and r27 is X (XL and XH)

; r28 and r29 is Y (YL and YH) - used for output digit pointer

; r30 and r31 is Z (ZL and ZH) - used for output digit lookup

;
; ps2leds=r3
;
	.EQU	scroll_lock = 0
	.EQU	num_lock = 1
	.EQU	caps_lock = 2
;
; r5 - additional key states
;
	.EQU	delete = 0		;0xe0, 0x71
	.EQU    delete_num = 1		;0x71
;
; r17 rx state
	.EQU	ps2_data_avail	=	0
	.EQU	cmd_out	 = 1
;
; r25 state register states
	.EQU	lft_alt = 0
	.EQU	rht_alt = 1
	.EQU	lft_shift =	2
	.EQU	rht_shift = 3
	.EQU	rht_control = 4
	.EQU	lft_control = 5
	.EQU	isup = 6
	.EQU	extended = 7

;
;
; DATA PORT to Apple 
;
	
	.EQU	BIT0_B0 	=	0x01 		; D0 ASCII bit - PB0
	.EQU	BIT0_B0_N	=	0

	.EQU	BIT1_B1		=	0x02		; D1 ASCII bit - PB1
	.EQU	BIT1_B1_N	=	1

	.EQU	BIT2_B2		=	0x04		; D2 ASCII bit - PB2
	.EQU	BIT2_B2_N	=	2

	.EQU	BIT3_B3		=	0x08		; D3 ASCII bit - PB3
	.EQU	BIT3_B3_N	=	3

	.EQU	BIT4_B4		=	0x10		; D4 ASCII bit - PB4
	.EQU	BIT4_B4_N	=	4

	.EQU	BIT5_B5		=	0x20		; D5 ASCII bit - PB5
	.EQU	BIT5_B5_N	=	5

	.EQU	BIT6_B6		=	0x40		; D6 ASCII bit - PB6
	.EQU	BIT6_B6_N	=	6

	.EQU	RESET_A0	=	0x01		; RESET - LOW resets motherboard - PA0
	.EQU	RESET_A0_N	=	0

	.EQU	STROBE_D3	=	0x08		; STROBE Rising edge indicates data available - PD3
	.EQU	STROBE_D3_N	=	3

	.EQU	CLEAR_A1	=	0x02		; CLEAR - HIGH clears Apple 1 screen -PA1
	.EQU	CLEAR_A1_N	=	1



;
; reset strobe sequence
;
	.EQU	RST_R		= 0x12			; reset via control-R, control-S, control-T, in order
	.EQU	RST_S		= 0x13
	.EQU	RST_T		= 0x14

;
; clear  strobe(apple 1) sequence	control-C, control-L, control-R
;
	.EQU	CLR_C		= 0x3
	.EQU	CLR_L		= 0xc
	.EQU	CLR_R		= 0x12

	.EQU	A2_FEAT_D6 	=	0x40		; input control map arrows to apple II ESC-char sequence
	.EQU	A2_FEAT_D6_N =	6				

	.EQU	AUTORESET_D5	=	0x20		; input control auto reset = 0, else no auto reset
	.EQU	AUTORESET_D5_N	=	5		;

	.EQU	CLRRST_D4	=	0x10		; input control support cntrl-RST and cntrl-clr sequences
	.EQU	CLRRST_D4_N	=	4		;

;
; define commonly used macros
;


	.LIST
	.DSEG

	.cseg
	.org  0x0000;
 	rjmp	start			; 0x0 reset
	rjmp	interrupt0		; 0x1 extern int 0
	rjmp	error0		; 0x2 extern int 1
	rjmp	error2			; 0x3 timer 1 capture
	rjmp	timer1_compA	; 0x4 timer 1 comp A
	rjmp	error3	 		; 0x5 timer 1 overflow
	rjmp	error4			; 0x6 timer 0 overflow
	rjmp	error5			; 0x7
	rjmp	error6			; 0x8
	rjmp	error7			; 0x9
	rjmp	error8			; 0xa
	rjmp	error9			; 0xb
	rjmp	error1	; 0xc timer1_compA
	rjmp	error10			; 0xd timer0_compA
	rjmp	error11			; 0xe timer0_compB
	rjmp	error12			; 0xf
	rjmp	error13			; 0x10
	rjmp	error14			; 0x11
	rjmp	error15				; 0xb12


;
; initialize variables and clock
;
start:
;
; initialize registers
;
;
; stack pointer
;
	clr	zero
	clr XH
	ldi scratch,RAMEND				; stack pointer
	out SPL,scratch
;
; PORT D has configuration jumper inputs (input with pull up) and PS2 clock
;
	ldi	scratch, AUTORESET_D5 | CLRRST_D4 | A2_FEAT_D6				; 
	out	PORTD,scratch					; all config inputs are pulled up
	ldi scratch,STROBE_D3					; inputs, except for D3-strobe
	out DDRD,scratch					; must have data as zero, before
								; switching - assume come
								; here only from reset
;
;
; port A  reset and clear
;

; if auto reset (RESET_A0 grouned), clr scratch else ldi scratch,RESET_A0
	ldi 	scratch,RESET_A0			; outputs set low, except for reset
	sbis	PIND,AUTORESET_D5_N			; test auto-reset signal (auto reset if grounded)
	clr 	scratch						; reset low (enabled)
	out		PORTA,scratch				; clear data bits

	ldi		scratch,RESET_A0 | CLEAR_A1 ; set dir resister to outputs for clear and reset
	sbis	PIND,CLRRST_D4_N			; test CTRL-CLEAR & CNTRL-RST SUPPORT 
	ldi		scratch,RESET_A0			; set dir resister to output for reset only - clear
										; is repurposed as strobe ack input
	out		DDRA,scratch				; write PORT A direction register
;
; port B  7 keyboard data bits plus PS2 data
;	
	clr scratch							; outputs set low
	out PORTB,scratch					; clear data bits
	ldi scratch,BIT0_B0 | BIT1_B1 | BIT2_B2 | BIT3_B3 | BIT4_B4 | BIT5_B5 | BIT6_B6
	out DDRB,scratch					; set outputs

;
; initialize UART
;
;
; 7 bits data even parity - 1 stop
;
	ldi scratch,0x24
	out UCSRC,scratch
	ldi scratch,25		; 9600 baud, given 4.0 MHZ clock
	out UBRRL,scratch
	clr	scratch
	out	UBRRH,scratch
	ldi scratch,(1<<RXEN)|(1<<TXEN)
	out UCR,scratch
;
; optional - calibrate clock
;
;	.EQU CALIB=1
.IFDEF CALIB
; read eeprom
; scratch is address
; data is returned in scratch
	ldi		scratch,EEPROMEND			; last location in eeprom has cal value
	rcall	eeprom_rd
	cpi		scratch,0xff
	brne	cal_start
;
; use default value
;
	ldi 	scratch,0x62
	mov		cal_val,scratch
;	rjmp calibrate_cont
;
; use stored value
;
cal_start:
	mov		cal_val,scratch
calibrate:
	out	OSCCAL,cal_val				; tweak clock calibration
	rcall	transmit
;
; send square wave out in order to calibrate - 1000 cycles per iteration
; 	results in 125 usec long levels - 250 usec long wave
;
calibrate_cont:
	ldi		scratch2,248		;1
calibrate2:						; 248 * 4 = 992
	nop							;1
	dec		scratch2			;1
	brne	calibrate2			;2/1
	nop							;1
	nop							;1
	in		scratch1,PORTB		;1
	com		scratch1			;1
	out		PORTB,scratch1		;1

	sbis	USR,RXC				; handle UART data even if keyboard not initialized
	rjmp 	calibrate_cont		;2 no,wait
;
; now allow user to adjust
;
	in		scratch,UDR
	cpi		scratch,'I'
	brne	calibrate_chk_d
; increase speed
	dec		cal_val
	rjmp	calibrate
; descrease speed, unless done
calibrate_chk_d:
	cpi		scratch,'D'
	brne	calibrate_chk_wait
	inc		cal_val
	rjmp	calibrate

; descrease speed, unless wait detected
calibrate_chk_wait:
	cpi		scratch,'W'
	breq	calibrate
	inc		cal_val

calibrate_chk_done:
	cpi		scratch,'Q'
	brne	calibrate

calibrate_done:
	mov		datain,cal_val
	rcall	dumpit

; scratch is address
; scratch1 is data
	mov		scratch1,cal_val
	ldi		scratch,EEPROMEND			; last location in eeprom has cal value
	rcall	eeprom_wrt
.ENDIF
;.IFDEF SAVECALIB
	ldi		scratch,EEPROMEND			; last location in eeprom has cal value
	rcall	eeprom_rd
	out		OSCCAL,scratch				; output clock calibration
;.ENDIF
;
;
; initialize timer1 - 666 ticks per second
; 5 digits = each digit  666 times per second
; 4000000/256/24=666
;
								; Set Clear on Compare Match,
	ldi	scratch,0x4 | 0x8		; set clock divisor to 64
			
	out	TCCR1B,scratch
	ldi  scratch,0		; compare register is 24 (decimal)
	ldi  scratch1,72	; with scaler of 256 = 666 int per second  (make 222)
;MJW for debug-simulate
;	ldi  scratch,0		; 
;	ldi  scratch1,1	;
	out	OCR1AH,scratch
	out	OCR1AL,scratch1

; initialize PS-2 input interrupt
; interupt on both edges
;
	ldi scratch,0x01
	out	MCUCR,scratch
	ldi	scratch, 0x40
	out	GIMSK,scratch
;
; send clear pulse (Apple 1 function)
;
	sbis	PIND,CLRRST_D4_N				; test CTRL-CLEAR & CNTRL-RST SUPPORT
	rjmp	initchkautoreset				; grounded - not enabled, skip auto clear, even if auto reset enabled
;
;
; if auto reset, call appleclear, else do not
	sbis	PIND,AUTORESET_D5_N			; test auto-reset signal (auto reset if grounded)
	rcall	appleclear					; skip
;
; if auto reset, deassert reset output, else do not
initchkautoreset:
	sbis	PIND,AUTORESET_D5_N			; test auto-reset signal (auto reset if grounded)
	sbi		PORTA,RESET_A0_N			; auto reset, remove reset				

	ldi	scratch,1<<num_lock | 1<<caps_lock ;default numlock - capslock on		; 
	mov	ps2leds,scratch

	ldi		rststate,RST_R			; initialize reset sequence
	ldi		scratch,CLR_C
	mov		clrstate,scratch		; initialize clear sequence

;
; wait for AA self test complete indication
;
;
; this is interrupt driven
;
begin:
	cli
	ldi 	scratch,RAMEND				; reinit stack pointer
	out 	SPL,scratch
	SEI

	clr		ps2initialized				;indicate PS/2 not initialized

;	ldi		scratch,'1'
;	rcall	transmit

	rcall	delay250us

	nop
	nop

	cli
	rcall	ps2_cleanup2
	clr		state
	clr		kstate
	sei


wait4selftestcomplete:
	rcall	timed_wait_ps2_data			; wait for PS/2 data (returned in scratch)
	cpi		scratch,0xaa	
	breq	setcapslock

;	ldi		scratch,'2'
;	rcall	transmit
;
; not good reset and repeat	
send_rst:

;	ldi		scratch,'3'
;	rcall	transmit

	cli

	sbr		rxstate,1<<cmd_out 	; set command out processing active
	ldi		scratch,0xff		; send reset command 0xff
	mov		ps2outdata,scratch
;
;	lower clock, this will kick off interrupt processing, in order to transmit 
;	command
;
	cbi		PORTD,PIND2		; set output low
	sbi		DDRD,PIND2		; now set direction
	sei
;
; wait for ack
;
	rcall	timed_wait_ps2_data

	cpi		scratch,0xfa				
	breq	wait4selftestcomplete	; ack - wait for self test complete indication
	
;	cpi		scratch,0xfe		; cmd error				; 
	rjmp	send_rst			; resend reset


timed_wait_ps2_data:
	clr	timer1
	clr	timer2
	ldi	scratch,8
	mov	timer3,scratch

timed_wait_ps2_data_1:
	inc		timer1					;1
	brne	timed_wait_ps2_data_2	;1/2
	inc		timer2					;1
	brne	timed_wait_ps2_data_2 ;1/2
	dec		timer3
	breq	timed_wait_ps2_data_timeout
timed_wait_ps2_data_2:
	sbic	USR,RXC					; 1/2 handle UART data even if keyboard not initialized
	rcall 	receive					; uart data present yes, call recieve handler

	sbrs	rxstate,ps2_data_avail 	; 1/2 has new character arrived
	rjmp	timed_wait_ps2_data_1	; 1 wait for data

;	rcall	dumpit

	mov		scratch,datain
	cbr		rxstate,1<<ps2_data_avail ; clear rx bit
	ret
;
; 1 second timeout
;
timed_wait_ps2_data_timeout:

	rcall	ps2_cleanup2
	ldi		scratch,0xfe
	ret
;
; 
;

setcapslock:
	ser		scratch
	mov		ps2initialized,scratch	; indicated PS/2 initialized

	rcall	delay250us
	cli

	ldi 	scratch,RAMEND			; reinit stack pointer
	out 	SPL,scratch

	rcall	ps2_cleanup2
	clr		state
	clr		kstate

	sei

	rcall	start_ps2_cmd			; set keyboard leds

;
; idle loop - wait for incoming data
;
main:
	rcall	wait_ps2_data		; wait 4 PS2 or RS232 data
	rcall	decode				; decode recieved character
	rjmp	main				; no, continue to wait

;
; wait 4 PS/2 data, but allow RS232 input (new data returned in scratch)
;
wait_ps2_data:
	sbic	USR,RXC					; handle UART data even if keyboard not initialized
	rcall 	receive					; uart data present yes, call recieve handler

	sbrs	rxstate,ps2_data_avail 	; has new character arrived
	rjmp	wait_ps2_data			; wait for data

;	rcall	dumpit

	mov		scratch,datain
	cbr		rxstate,1<<ps2_data_avail ; clear rx bit
	ret
;
; send data to UART
;
transmit:
	sei
	nop
	cli
	sbis	USR,UDRE 			;ready to send?
	rjmp 	transmit
	out		UDR,scratch
	sei
	ret

;
; receive a byte from the serial port
; since it should be in ASCII format
; echo out TX and send to APPLE
;
receive:
	sbis	USR,RXC				; recieve complete
	rjmp 	receivedone				; no,wait
	in		scratch,UDR
;
; got char
;
gotchar:
;
; check to see if Cntrl-rst and Cntrl-clr are enabled
;
	sbis	PIND,CLRRST_D4_N		; test CTRL-CLEAR & CNTRL-RST SUPPORT 
	rjmp	recievenotclear			; grounded out - disabled - skip this check	
;
	cp		scratch,rststate	; check to see if it is next reset char
	brne	recievenotreset			;
;
;  next reset sequence char: cntrl-R, cntrl-S, cntrl-T
;
	inc		rststate
;
; RESET when cntrl-R, cntrl-S, contrl-T sequence is recieved
;
	cpi		rststate,RST_T+1	; last reset char?
	brlt	check_clear_cmd
;
; sequence complete - now reset
;
	rcall	applereset			; reset now
;
	ldi		rststate,RST_R		; reinitialize reset sequence
	rjmp	receivedone			;

recievenotreset:
	ldi		rststate,RST_R		; reinitialize reset sequence
;
; check for clear command
;
check_clear_cmd:
	cp		scratch,clrstate	; check to see if it is next clear char
	brne	recievenotclear			;
;
;  next reset sequence char: cntrl-C, cntrl-L, cntrl-R
;
	mov		scratch1,clrstate
	cpi		scratch1,CLR_C
	brne	check_clear_L
	ldi		scratch1,CLR_L
	rjmp	recievechar

check_clear_L:
	cpi		scratch1,CLR_L
	brne	clear_R
	ldi		scratch1,CLR_R
	rjmp	recievechar
;
; CLEAR when cntrl-C, cntrl-L, contrl-R sequence is recieved
;
;
; sequence complete - now reset
;
clear_R:
	rcall	appleclear			; clear now
;
	ldi		scratch1,CLR_C		; reinitialize clear sequence
	mov		clrstate,scratch1
	rjmp	receivedone			;
;
;
;
recievenotclear:
	ldi		scratch1,CLR_C		; reinitialize clear sequence
	mov		clrstate,scratch1
;
; regular character
;
recievechar:
	mov		clrstate,scratch1	; save clear state
	rcall	appleoutput			; send to apple
	rcall	transmit
receivedone:
	ret
;
; reset appkle, don't echo or foreward character
;
applereset:
	cbi		PORTA,RESET_A0_N	; engage reset
	ldi		scratch1,4
applereset1:
	ldi		scratch2,250		; wait 4*1000 * 1/4 micro-second (1 milli second)	
applereset2:
	nop
	nop
	dec		scratch2
	brne	applereset2
	dec		scratch1
	brne	applereset1
;
;remove reset signal
;
	sbi		PORTA,RESET_A0_N
	ret

;
; clear apple 1 screen
;
; must hold this signal for 1 entire screen
; at 30 hertz, that is 33 milliseconds
; for extra goodness, I'm doubling that
;
appleclear:
	sbi		PORTA,CLEAR_A1_N	; engage reset
	ldi		scratch1,128			; hold for 66 milliseconds
appleclear1:
	ldi		scratch2,250		; wait  for 1 milliseconde 4*1000 * 1/4 micro-second cycle = 1000 microseconds (1 milli second)	
appleclear2:
	nop
	nop
	dec		scratch2
	brne	appleclear2
	dec		scratch1
	brne	appleclear1
;
;remove clear signal
;
	cbi		PORTA,CLEAR_A1_N
	ret

;
; dumpit
; 
dumpit:
	ldi		scratch2,0x30
	mov		scratch,datain
	lsr		scratch
	lsr		scratch
	lsr		scratch
	lsr		scratch
	cpi		scratch,0xa
	brlt	dumpit1
	ldi		scratch2,0x7
	add		scratch,scratch2
dumpit1:
	ldi		scratch2,0x30
	add		scratch,scratch2
	rcall 	transmit

	mov		scratch,datain
	ldi		scratch1,0x0f
	and		scratch,scratch1
	cpi		scratch,0xa
	brlt	dumpit2
	ldi		scratch2,0x7
	add		scratch,scratch2
dumpit2:
	ldi		scratch2,0x30
	add		scratch,scratch2
	rcall	transmit
	ret
;
; output cmd to PS/2 keyboard
;
start_ps2_cmd:

;	ldi		scratch,'1'		; debug
;	rcall	transmit

	cli

	sbr		rxstate,1<<cmd_out 	; set command out processing active
	ldi		scratch,0xed
	mov		ps2outdata,scratch						; block interrupts
;
;	lower clock, this will kick off interrupt processing, in order to transmit 
;	command
;
	cbi		PORTD,PIND2		; set output low
	sbi		DDRD,PIND2		; now just set direction
	sei

;
;	wait for command complete
temp_wait:
	sbrc	rxstate,cmd_out
	rjmp	temp_wait

;
; wait for ack
;
	rcall	timed_wait_ps2_data	; wait 4 PS2 or RS232 data
	cpi		scratch,0xfa				
	breq	send_LED			; ack - send LED settings
	cpi		scratch,0xfe		; cmd error
	brne	send_error6			; yes echo error	
	rjmp	error5				; no ack - send error code 5
send_error6:
	rjmp	error6

send_LED:

	cli

	sbr		rxstate,1<<cmd_out 	; set command out processing active
	mov		scratch,ps2leds
	mov		ps2outdata,scratch
;
;	lower clock, this will kick off interrupt processing, in order to transmit 
;	command
;
	cbi		PORTD,PIND2		; set output low
	sbi		DDRD,PIND2		; now set direction
	sei
;
; wait for ack
;

	rcall	timed_wait_ps2_data		; wait 4 PS2 or RS232 data
	cpi		scratch,0xfa				
	breq	start_ps2_cmd_wait3		; ack - done
	
	cpi		scratch,0xfe		; cmd error
	brne	send_error8				; 	
	rjmp	error7				; no ack - send error code 5
send_error8:
	rjmp	error8	
start_ps2_cmd_wait3:
;	ldi		scratch,'5'
;	rcall	transmit
	ret	

;
; output character to apple
;
appleoutput:
	push 	scratch				; save data (in case we need to invert)

	mov		bdataout,scratch	; move to working register

	andi	bdataout,BIT0_B0 | BIT1_B1 | BIT2_B2 | BIT3_B3 | BIT4_B4 | BIT5_B5 | BIT6_B6

;
;preserve PS2 data bit setting
;
;
;preserve PS2 clock/data bit settings - these registers are manupulated in interrupt
; handling, so we must block interrupts around this
;
	cli

	in		scratch2,PORTB
	andi	scratch2,1<<PINB7
	or		bdataout,scratch2

;
; load registers and then clock
;
	out		PORTB,bdataout

	sei


	sbi		PORTD,STROBE_D3_N	; clock motherboard flip-flop

	ldi		scratch,100			; 100 iterations, 1 usec per iteration = 100 usec
datastrobe:
	nop
	nop
	dec		scratch
	brne	datastrobe
;
;IF CLR RST config jumper installed, wait for clear input to be asserted.
;
	sbic		PIND,CLRRST_D4_N	; test yes CTRL-CLEAR & CNTRL-RST SUPPORT 
	rjmp		datastrobexit		; yes, don’t wait for ack on clear line
;
; wait for ack on clear line, which is configured as input
;
datastrobewaitack:
	sbis		PINA,CLEAR_A1_N
	rjmp		datastrobewaitack	 

datastrobexit:
	cbi		PORTD,STROBE_D3_N

	pop 	scratch				; restore non-inverted data 

	ret
;
; decode recieved char from PS2 keyboard interface
;
decode:
;
; if this is a selftest done indication, then 
; someone must have just plugged in a keyboard
; set LEDS and start over in this case
;
	cpi		scratch,0xaa	
	brne	check_extended
	rjmp	setcapslock
;
; check for extended char coming - this can come with
; either keyup character in buffer or not
;
check_extended:
	cpi		scratch,0xe0
	brne	check_isup
	sbr		state,1<<extended
	rjmp	decode_done
;
; are we in key up or down state
;
check_isup:
	sbrc	state,isup	
	rjmp	keyup_inprogress	;branch key up being processed
;
; 	this is a key down
;	key up indicator (0xf0) has not been recieved
;
;
; check for delete
;
	cpi		scratch,0x71			; delete character code
	brne		not_delete			; no, continue
;
; extended (normal) or not (from keypad) 
	mov		scratch1,kstate			; prefetch state
	sbrs		state,extended			; extended flag (normal del)
	rjmp		not_extended_delete

	sbr		scratch1,1<<delete		; indicate delete depressed
	rjmp		delete_chk_done			;
not_extended_delete:					;
	sbr		scratch1,1<<delete_num		; indicate delete (keypad) depressed
delete_chk_done:
	mov		kstate,scratch1			; save kstate
	rjmp		decode_lookup			; delete is also passed to host system

not_delete:
;
; check for alt
;
	cpi		scratch,0x11			; alt character (left or right)
	brne	chk_control
	sbrs	state,extended			; extended flag means right
	rjmp	left_alt
	sbr		state,1<<rht_alt		; right alt depressed
	rjmp	keyup_done
left_alt:
	sbr		state,1<<lft_alt		; left alt depressed
	rjmp	keyup_done
;
; check for control
;
chk_control:
	cpi		scratch,0x14			;control character (left or right)
	brne	not_control
	sbrs	state,extended			; extended flag means right
	rjmp	control_left
	sbr		state,1<<rht_control	; right
	rjmp	keyup_done
control_left:
	sbr		state,1<<lft_control	; left
	rjmp	keyup_done
not_control:
;
; check for numlock - commented out - numlock always on
;
;	cpi		scratch,0x77
;	brne	not_numlock:
;	sbr		state,1<<numlock
;	rjmp	keyup_done
;notnumlock
;
; check for shift lock
;
	cpi		scratch,0x58
	brne	check_shift
	sbrc	ps2leds,caps_lock
	rjmp	clear_shiftlock
; set shift lock
	mov		scratch,ps2leds
	sbr		scratch,1<<caps_lock
	mov		ps2leds,scratch
	rcall	start_ps2_cmd
	rjmp	keyup_done

clear_shiftlock:
	mov		scratch,ps2leds
	cbr		scratch,1<<caps_lock
	mov		ps2leds,scratch
	rcall	start_ps2_cmd
	rjmp	keyup_done
;
; check for shift "0x12" or "0x59" or keyup "0xf0"
;
check_shift:
	cpi		scratch,0x12
	breq	left_shift
	cpi		scratch,0x59
	brne	chk_keyup
	sbr		state,1<<rht_shift
	rjmp	keyup_done
left_shift:
	sbr		state,1<<lft_shift
	rjmp	keyup_done

chk_keyup:
	cpi		scratch,0xf0		;key up indicator
	brne	chk_arrow
	sbr		state,1<<isup
	rjmp	decode_done

;
; check for arrow keys - emulate esc-char sequence for apple II
; 
chk_arrow:
;these are all extended keycodes - make sure extended bit is set
	sbrs	state,extended	
	rjmp	decode_lookup
	sbic	PIND,A2_FEAT_D6_N	; is Apple arrow mapping enabled?
	rjmp	decode_lookup		; no, do normal ASCII mapping
	cpi		scratch,0x75		; up arrow
	brne	chk_dn_arrow		; no, check down
	ldi		scratch,'D'			; simulate with ESC-D
	rjmp	emulate_cursor_controls
chk_dn_arrow:
	cpi		scratch,0x72		; down arrow
	brne	chk_lft_arrow		; no, check left
	ldi		scratch,'C'
	rjmp	emulate_cursor_controls
chk_lft_arrow:
	cpi		scratch,0x6b		; left arrow
	brne	chk_rht_arrow		;no, check right
	ldi		scratch,'B'
	rjmp	emulate_cursor_controls
chk_rht_arrow:
	cpi		scratch,0x74		; right arrow
	brne	decode_lookup		;no, decode normally
	ldi		scratch,0x15
	rjmp	emulate_cursor_no_esc
emulate_cursor_controls:
	push	scratch
	ldi		scratch,0x1b		; send ESC
	rcall	appleoutput
	rcall	delay250us			; give apple time to process
	pop		scratch
emulate_cursor_no_esc:
	rcall	appleoutput			; send second char to move cursor
	rjmp	keyup_done
;
; not shift/or keyup - then do character lookup
;
decode_lookup:
;
; check control, extentened, caps lock, shifted, normal table, in that order
;
	ldi		ZH,high(controlchar<<1)
	ldi		ZL,low(controlchar<<1)
	sbrc	state,rht_control
	rjmp	decode_now
	sbrc	state,lft_control
	rjmp	decode_now

 	ldi		ZH,high(extend<<1)
	ldi		ZL,low(extend<<1)
	sbrc	state,extended
	rjmp	decode_now

	ldi		ZH,high(caps<<1)
	ldi		ZL,low(caps<<1)
	sbrs	ps2leds,caps_lock
	rjmp	decode_check_shifted

decode_check_caps_lock:
	lpm		scratch1,Z+
	lpm		scratch2,Z+
	cp		scratch1,zero
	breq	decode_check_shifted		;not found - check shifted or unahifted
	cp		scratch,scratch1
	brne	decode_check_caps_lock
	rjmp	decode_found_it

decode_check_shifted:
	ldi		ZH,high(shifted<<1)
	ldi		ZL,low(shifted<<1)
	sbrc	state,lft_shift
	rjmp	decode_now
	sbrc	state,rht_shift
	rjmp	decode_now

;
; normal, unshifted, no extended or control
;
	ldi		ZH,high(unshifted<<1)
	ldi		ZL,low(unshifted<<1)
decode_now:
;
; fetch entry from table
;
decode_chk_nxt:
	lpm		scratch1,Z+
	lpm		scratch2,Z+
	cp		scratch1,zero
	breq	lookup_done			;not found - drop it
	cp		scratch,scratch1
	brne	decode_chk_nxt
;
; found it 
;
decode_found_it:
	mov		scratch,scratch2	; found it move to scratch register and forward
	rcall	gotchar			; to ascii interface and serial port
;
; clear flags
;
lookup_finished:
lookup_done:
	cbr		state,1<<extended
	rjmp	decode_done	
;
; keyup processing
;
keyup_inprogress:
	cbr		state,1<<isup		; clear key up indicator
;
; is delete character released	;
;
	cpi		scratch,0x71			;delete character
	brne	keyup_not_delete

;
; extended (normal) or not (from keypad) 
	mov		scratch1,kstate			; prefetch state

	sbrs		state,extended			; extended flag 
	rjmp		keyup_not_extended_delete

	cbr		scratch1,1<<delete		; indicate delete (normal del) released
	rjmp		keyup_delete_chk_done		;

keyup_not_extended_delete:				;
	cbr		scratch1,1<<delete_num		; indicate delete (keypad) released
keyup_delete_chk_done:
	mov		kstate,scratch1
	rjmp		keyup_done

keyup_not_delete:

; check for alt key released
;
	cpi		scratch,0x11			;alt character
	brne	keyup_not_alt

	sbrs	state,extended			; extended flag means right
	rjmp	keyup_alt_left
	cbr		state,1<<rht_alt		; right
	rjmp	keyup_done
keyup_alt_left:
	cbr		state,1<<lft_alt		; left
	rjmp	keyup_done
keyup_not_alt:

;
; check for control
;
	cpi		scratch,0x14			;control character
	brne	keyup_not_control

	sbrs	state,extended			; extended flag means right
	rjmp	keyup_control_left
	cbr		state,1<<rht_control	; right
	rjmp	keyup_done
keyup_control_left:
	cbr		state,1<<lft_control	; left
	rjmp	keyup_done
keyup_not_control:
;
; check for numlock - commented out - numlock is always on
;
;	cpi		scratch,0x77
;	brne	keyup_not_numlock
;	cbr		state,1<<numlock
;	rjmp	keyup_done
;keyup_not_numlock:
;
; check for shift
;
	cpi		scratch,0x12
	breq	keyup_shiftleft
	cpi		scratch,0x59
	brne	keyup_no_shift
	cbr		state,1<<rht_shift
	rjmp	keyup_done

keyup_shiftleft:
	cbr	state,1<<lft_shift			; recieved shift indicator remove, it
;
keyup_no_shift:
;
keyup_done:
	cbr	state,1<<extended
;
decode_done:
;
; before we exit - check for control-alt-del
;
	mov		scratch,state
	andi	scratch,1<<lft_alt | 1<<rht_alt			;either alt ?
	breq	decode_exit					;no, just exit
	mov		scratch,state
	andi	scratch,1<<lft_control | 1<<rht_control		;either control?
	breq	decode_exit					; no, just exit
	mov		scratch,kstate					
	andi	scratch,1<<delete | 1 << delete_num		;either delete?
	breq	decode_exit					; no, just exit
;
;control-alt-delete all pressed - triggers reset
;
	rcall	applereset
;
; reset state flags, so we don't double reset
;
	mov		scratch,state
	andi	scratch,~(1<<lft_control | 1<<rht_control | 1<<lft_alt | 1<<rht_alt)
	mov     state,scratch
	clr		kstate
decode_exit:
	ret

;
; PS-2 interrupt - caused by either edge (positive or negative)
;		used to caputer PS2- character input
;
interrupt0:
interrupt1:
	in		intscratch,SREG		; save status register
	push	intscratch
;
; first figure out whether this is command out or data in
;
	sbrc	rxstate,cmd_out
	rjmp	intcmd	
;
;
; check whether interrupt is rising or falling edge
;
	sbis	PIND,PIND2			; current state indicates
	rjmp 	intfalling			; rising or falling edge
;
; rising edge here
; check for all bits recieved
; if so then decode incoming character
; and indicate to background loop that there is
; a character ready to send
;
	in		intscratch,TIMSK	; if timer not running, ignore for now
	sbrs	intscratch,OCIE1A	; running?
	rjmp	intdone				; just exit, if not			; 


	dec		bitcount			; count bit
	brne	intdone				; not done, just continue
;
; character recieved - cleanup and tell BG task that char arrived

	mov		datain,ps2data		; move to background process
	sbr		rxstate,1<<ps2_data_avail ;set data avail flag

int_processing_done_ok:
	clr		intscratch			; stop timer interrupt
	out		TIMSK,intscratch
;
	ldi		intscratch,11		; reinitialize bit counter
	mov		bitcount,intscratch ;
	ser		intscratch			; initialize parity to odd
	mov		parity,intscratch
	rjmp	intdone				; done, let's get out
;
; falling edge here
; clock in next bit - ignore  bits 11 and 2-0
; bit 10 - 3 is data
;
intfalling:
	mov		intscratch,bitcount
	cpi		intscratch,3			; don't accumulate data 
	brlt	intparity_stop			; when parity bit (2) and stop bit (1) arrive

	cpi 	intscratch,11			; start bit (11) start timer
	breq	startbit			; 

	lsr		ps2data				; make room for next bit
	
	sbis	PINB,PINB7			; read state of data line
	rjmp	intdone				; zero - do nothing
	sbr		ps2data,1<<7		; set high bit in accumulator
	eor		parity,ps2data		; accumulate parity in MSB of parity register			
	rjmp	intdone
;
; start 1.5 MS timer here and check start bit
;
startbit:

	rcall	start_timer			; start timer
	sbic	PINB,PINB7			; read state of data line
	rjmp	ps2_cleanup			; 1st bit has to be zero - overwise abort- cleanup
	rjmp	intdone				; good start bit, exit

start_timer:
								; preset counter
	clr		intscratch
	out		TCNT1H,intscratch	; to zero
	out		TCNT1L,intscratch	;

	ser		intscratch			; clear any pending interrupts
	out		TIFR,intscratch		;

	ldi		intscratch,1<<OCIE1A ; enable intererupt
	out		TIMSK,intscratch

	ret

intparity_stop:
	cpi		intscratch,2		; stop bit (2) or parity (1)
	brne	intdone_check_stop	; count is (2), must be parity
;
; parity check here
;
	mov		intscratch,parity	; grab current parity state
	sbic	PINB,PINB7			; read state of data line
	rjmp	parity_set			; jump if parity bit set

	sbrc	intscratch,7		; MSB parity register should be clear
	rjmp	ps2_cleanup			; it is an error if set - cleanup
	rjmp	intdone				; no parity error

parity_set:
	sbrs	intscratch,7		; MSB parity register should be set
	rjmp	ps2_cleanup			; no - parity error  - cleanup
	rjmp	intdone				;  ok - wait for stop bit

intdone_check_stop:
	sbis	PINB,PINB7			; read state of data line
	rjmp	ps2_cleanup			; stop bit has to be one - if not abort input - cleanup
	rjmp	intdone				; ok - return from interrupt
;
; Timer compare interrupt 0 
;		used to timeout failed PS-2 keyboard transaction
;
timer1_compA:
timer0_compA:
timer0_compB:
	rjmp	errort

ps2_cleanup:
	rcall	ps2_cleanup2
;
; now return from int
;
intdone:
	pop		intscratch
	out		SREG,intscratch
	reti		

ps2_cleanup2:
	sbi		PORTD,PIND2			; set data bit high to enable pull ups
	cbi		DDRD,PIND2			; now switch clock driver off, now rx with pull up

	sbi		PORTB,PINB7			; set data bit high to enable pull ups
	cbi		DDRB,PINB7			; now switch data driver off; now rx with pull up:

	clr		rxstate
	ldi		intscratch,11		; reinitialize bit counter
	mov		bitcount,intscratch ;

	ldi		intscratch,1<<INT0	; clear any lingering interrupt
	out		EIFR,intscratch

; must mask timer interrupt
; timer is free running, we will restart and enable mask when next char comes in
;
ps2_stop_timer:
	clr		intscratch			; stop timer interrupt
	out		TIMSK,intscratch
	ret							;  return

;
; start processing command out 
;
intcmd:
; background process sets data direction register for clock bit
; when initiating command sequence
;
	sbic	DDRD,PIND2
	rjmp	intcmd_low_clock		; initial sequence - lowered clock
;
; keyboard is driving clock, so are now sending data or waiting ack
; then we drive data
;
	sbic	DDRB,PINB7				; we are driving data lines so data rx in progress
	rjmp	intcmd_data
;
; driving neither clock nor data - waiting for ack
;
	rjmp	cmd_wait_ack
;
; this is first phase, we have lowered clock to alert keyboard
;
intcmd_low_clock:
	mov		ps2data,ps2outdata	; get data register ready
;
; wait min 100 micro seconds
;
	rcall	delay40			; delay for at least 100 usec
	rcall	delay40			; this should inhit keyboard from sending
	rcall	delay40
	
	rcall	start_timer			; start timer

	ldi		intscratch,10		; reinitialize bit counter
	mov		bitcount,intscratch ;
	ser		intscratch			; initialize parity to odd
	mov		parity,intscratch
;
;
; lower data bit and raise and release clock 
;
	cbi		PORTB,PINB7		; set data bit low (start bit) (data register first)
	sbi		DDRB,PINB7		; set data bit low (start bit) (then direction register)
	sbi		PORTD,PIND2		; raise clock
	cbi		DDRD,PIND2		; now switch clock driver off
;
; this sequience causes an interrupt - clear it
;
	ldi		intscratch,1<<INT0
	out		EIFR,intscratch
; now device will clock in data on rising edge (use interrupt to handle from here)

	rjmp	intdone
;
;  sending data - clock out command on rising edge - set up data on lower edge
; 
intcmd_data:

	sbic	PIND,PIND2			; 
	rjmp	cmd_wait_ack		; raising edge - check for ack

	dec		bitcount			; check for stop bit
	brne	intcmd_not_stop		; not stop cont
;
; stop bit here
;
	sbi		PORTB,PINB7			; raise data 
	cbi		DDRB,PINB7			; now switch data driver off
	rjmp	intdone
;
intcmd_not_stop:
	mov		intscratch,bitcount
	cpi		intscratch,2	; check for parity
	brge	intcmd2_data	; branch if still sending command

;
; send parity bit
;
	mov		intscratch,parity ;
	sbrs	parity,0
	rjmp	cmd_parity_zero
	
	sbi		PORTB,PINB7		; set data bit high 
	rjmp	dataone

cmd_parity_zero:
	cbi		PORTB,PINB7		; set data bit high
	rjmp	intdone
;
; data
intcmd2_data:
	sbrs	ps2data,0		; check LSBit
	rjmp	datazero

	mov		intscratch,parity
	ldi		XL,0x01
	eor		intscratch,XL		; adjust parity
	mov		parity,intscratch
	sbi		PORTB,PINB7		; set data bit high 
	rjmp	dataone

datazero:
	cbi		PORTB,PINB7		; set data bit low

dataone:
	lsr		ps2data
	rjmp	intdone
;
; sent data and parity - wait for device to acknowlage - come here on falling edge	
;
cmd_wait_ack:
	mov 	intscratch,bitcount
	cpi		intscratch,0
	brne	cmd_wait_done

	sbic	PIND,PIND2		; falling edge - check for ack
cmd_wait_done:
	rjmp	intdone			; 

	sbic	PINB,PINB7		; data is low - then we are done
	rjmp	ps2_cleanup		; error clean up and prepare for next transaction
;
; done - with no error
;
	cbr		rxstate,1<<cmd_out		; clear command out flag 
	rjmp	int_processing_done_ok	; clean up and exit

;
; delay for 40 micro seconds (160 cycles)
;
delay40:
	push	scratch

	ldi		scratch,38			;1 cycle  - repeat 38 times/4 cycles per iteration
delay40_cont:
	nop
	dec		scratch				;1 cycle
	brne	delay40_cont		;1/2 cycles

	pop		scratch
	ret

delay250us:
	push	scratch
	ldi		scratch,248
delay250us_cont:
	nop
	dec		scratch
	brne	delay250us_cont
	pop		scratch
	ret
;
; error handling
;
error0:
	ldi		scratch1,'0'
	rjmp	error
error1:
	ldi		scratch1,'1'
	rjmp	error
error2:
	ldi		scratch1,'2'
	rjmp	error
error3:
	ldi		scratch1,'3'
	rjmp	error
error4:
	ldi		scratch1,'4'
	rjmp	error
error5:
	ldi		scratch1,'5'
	rjmp	error
error6:
	ldi		scratch1,'6'
	rjmp	error
error7:
	ldi		scratch1,'7'
	rjmp	error
error8:
	ldi		scratch1,'8'
	rjmp	error
error9:
	ldi		scratch1,'9'
	rjmp	error
error10:
	ldi		scratch1,'A'
	rjmp	error
error11:
	ldi		scratch1,'B'
	rjmp	error
error12:
	ldi		scratch1,'C'
	rjmp	error
error13:
	ldi		scratch1,'D'
	rjmp	error
error14:
	ldi		scratch1,'E'
	rjmp	error
error15:
	ldi		scratch1,'F'
	rjmp	error
errort:
	ldi		scratch1,'T'
	rjmp	error
error:
	tst		ps2initialized		;if PS/2 not initialized, don't send errors
	breq	errorwait

	push	scratch1
	ldi		scratch,'R'
	rcall	gotchar
	pop		scratch
	rcall	gotchar
errorwait:
	sbis	USR,UDRE 			;ready to send?
	rjmp 	errorwait
	rjmp	begin				; start over


; write eeprom
; scratch is address
; scratch1 is data
eeprom_wrt:
	sbic	EECR,EEPE
	rjmp	EEPROM_wrt
	out		EEAR,scratch		;save in I/O register
	out		EEDR,scratch1		;write data
	sbi		EECR,EEMPE
	sbi		EECR,EEPE
	ret

; read eeprom
; scratch is address
; data is returned in scratch
eeprom_rd:
	sbic	EECR,EEWE				;make sure eeprom is ready to read;
	rjmp	eeprom_rd
	out		EEAR,scratch			;set address
	sbi		EECR,EERE
	in		scratch,EEDR			;read value from eeprom location
	ret

		.include "scancodes.asm"

;	.ESEG
;	.ORG	0x7f
;	.DB		0x63




