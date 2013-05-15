;************************************************************
;**  	p31_audio.asm	27.04.13	           			   **
;*															*
;*		HW: LR Board mit ATM88-20 	    					*
;*		läuft unter AVR Studio4,einfachst HW, int RC Oszil. *
;*				    			    						*
;*		Clone aus p25_audio.			    				*
;*							    							*
;*	PWM Audio-Synthese mit Atm88:		    		*
;*							    							*
;*	Fkt.: 						    						*
;*							    							*
;*	Hist: 						    						*
;*	260413: init Proj. 		*
;*	 		*
;*	190710: einf. Playmodes Jumper 1/0 (=PD4/2)				*
;*			11 PM0:											*
;*				basic mode: 2x VCO in 15.xHz steps + Ringmod*
;*			10 PM1:											*
;*				1 VCO + Ringmod + autoplay tune				*
;*			01 PM2:											*
;*				autoplay beat polyphone + Ringmod			*
;*			00 PM3:											*
;*				clk pattern: 								*
;*				Ain6: mute/run -> rise up tone 				*
;*				Ain7: rise up clk freq.						*
;*							    							*
;*							    							*
;*	- no JTAG / int RC Oszillator. 							*
;*	-Boot-Sektor reserviert				    				*
;*	-kein umlegen der IRQ-vektoren			    			*
;*  FUSES: high-0xdf / low-0xe2
;*							    							*
;*	* timer0 ov-IRQ in phase corr. PWM mode (up+down cnt)	*
;*	 - 8MHz/prescaler 1, ticks to ov: 510: 63,75µs IRQ 		*
;*	 - 255µs: ADC-service / (debounce services)           	*
;*	          255+ 63,75 phase shift: start ADC				*
;*	 - 25ms:  (98x 255µs= 24,99ms)	    					*
;*	 - 0.25s:  LEDs etc			    						*
;*							    							*
;*							    							*
;*	* timer0:					    						*
;*	  PWMa: audio sin generation / main IRQ	ref AN314  		*
;*		1inc OCR2 := 15.686Hz: Synth A/B					*
;*	* timer1:					    						*
;*	  not used												*
;*	* timer2:					    						*
;*	  PWM out for Ring-Modulator							*
;*	  TOP= OCR2A aus compl(ADC0): Mod-Frequenz				*
;*	  OCR2B aus ADC1, aber:									*
;*	  - immer < ADC0/2 										*
;*	  - wenn ADC0 > eps: OCR2B immer >= 2 (einschalten Ring-*
;*	    Modulator via ADC0 = Frequenzvorgabe)				*
;*							    							*
;*							    							*				
;*							    							*
;*	* ADC: Vref= 5V, service in timer IRQ		    		*
;*	  Ain0: pot.1: Frequenzvorgabe PWM für Ringmodulation	*
;*	  Ain1: pot.2: DuCy PWM für Ringmodulation	  			*
;*	  Ain6: pot.3: Oszil.1 (Loockup Tab)	    			*
;*	  Ain3: pot.4: Oszil.2 (LT)								*
;*	  Ain4: extin1											*
;*	  Ain5: extin2											*
;*	  no mean-val calc, shrink to 8 bit, 63.75 for mux se.	*
;*	  						    							*
;*							    							*
;*	* RS232:					    						*
;*	  4800baud, 8bit 1stop no par			    			*
;*	  (2.08ms per 10bit frame)			    				*
;*	  byte-periode > 2ms				    				*
;*	  codes:					    						*
;*		> tbd.				    *
;*															*
;*							    							*
;* ;* Belegung der Ports:				    				*
;*							    							*
;*							    							*
;* PORTB:						    						*
;*		PB0	:out	status LED				    			*
;*		PB1	:out	OC1A Out 	 		    				*
;*		PB2	:out	nu			 			    			*
;*		PB3	:out	SPI master data out, MOSI  				*
;*		PB4	:in		SPI master data in, MISO   				*
;*		PB5	:out	SPI master Sclk out    					*
;*		(PB6/7:    	Xtal)									*
;*										    				*
;* PORTC:						    						*
;*		PC0	:ain	adc0			    					*
;*		PC1	:ain	adc1			    					*
;*		PC2	:ain	adc2				    			*
;*		PC3	:ain	adc3				    			*
;*		PC4	:ain	adc4			    					*
;*		PC5	:ain	adc5									*
;*		(PC6 :in.RES)			    						*
;*															*
;* PORTD:						    						*
;*		PD0	:in		RS232 RX 	    						*
;*		PD1	:out	RS232 TX		    					*
;*		PD2	:in.pu	SW1										*
;*		PD3	:out	Sq/PWM out OC2B							*
;*		PD4	:in.pu  SW2 	    							*
;*		PD5	:out	PWM out:OC0B           					*
;*		PD6	:out	PWM out:OC0A    						*
;*		PD7	:in.pu	SW3 				 					*
;*							    							*
;*							    							*
;* Memory-map:						    					*  
;*							    							*
;*	Registers:	see declarations		    				*
;*							    							*
;*	int. SRAM:	0x100-	void			    				*
;*							    							*
;*			<-0x45f	stack			    					*
;*							    							*
;*	int.EEPROM:					    						*
;*			0x000-				    						*	
;*			0x1ff					    					*
;*							    							*
;*	spec. flags:	r27 -> fla:			    				*
;*			bit7: 	will be set every 0.5s        			*
;*					in IRQ-serv 		   					*
;*			bit6:	will be set every 25ms	    			*
;*			bit5:	will be set every 255µs	    			*
;*			bit4:	act. not used		    				*
;*			bit3:			    							*
;*			bit2:		    								*
;*			bit1:	flag oszil_b							*
;*			bit0:	flag oszil_a   							*
;*							    							*
;*							    							*
;*							    							*
;*							    							*
;************************************************************
.INCLUDE "m88def.inc"

;		 r0 	;LPM data fetch
.DEF	ain0bu	=r1		;buffer Ain0
.DEF	ain1bu	=r2		;buffer Ain1
.DEF	ain2bu	=r3		;buffer Ain2
.DEF	ain3bu	=r4		;buffer Ain3
.DEF	ptr_Ltmp	=r5	;
.DEF	ptr_Htmp	=r6	;
.DEF	ptr_La		=r7	;
.DEF	ptr_Ha		=r8	;
.DEF	adda_o		=r9	;
.DEF	x_SWa		=r10;
.DEF	ptr_Lb		=r11;
.DEF	ptr_Hb		=r12;
.DEF	addb_o		=r13;
.DEF	x_SWb		=r14;
.DEF	appoi		=r15; autoplay-pointer

.DEF	acc	=r16		;main register, low
.DEF	ach	=r17		;high byte for accu in 16bit ops.
.DEF	acx	=r18		;auxilary reg.
.DEF	axh	=r19		;auxilary reg. high
.DEF	auxcnt	=r20	;com. counter for loops etc.
.DEF	rsloop	=r21	;Rs232 loop cnt
.DEF	adcpoi	=r22	;...

.DEF	ucnt	=r26	;counter for 63,75µs ticks
.DEF	cnt		=r27	;counter for 255µs ticks
.DEF	mscnt	=r28	;counter T=25ms
.DEF	fla		=r29	;flag register
.DEF	Zl		=r30	;pointer Z, low byte
.DEF	Zh		=r31	;pointer Z, high byte

.EQU	ti0val	=5 		;250 Timer0 ticks to IRQ (2ms)
.EQU	t01val	=98		;98 x 255µs for 25ms
.EQU	t1val	=10		;10 25ms's for 0.25s
.EQU	Led1= PB0		;
;.EQU	Led2= PD4		; 

.EQU	eps1= 3		;thereshold for FSR1 ant Ain2 (Osz.1)
.EQU	eps2= 3		;thereshold for FSR2 ant Ain3 (Osz.2)
.EQU	eps3= 3		;thereshold for FSR3 ant Ain0 (Ring.mod.freq)





;*==============================

.CSEG
.ORG 0x000		;reset vector
	rjmp start
.ORG 0x010		;Timer0 overflow IRQ service routine
	rjmp t0_irq

.ORG 0x020

start:
	ldi acc,0x04	;init stack pointer to end of int.SRAM
	ldi ach,0xff
	out SPL,ach
	out SPH,acc

	rcall bas_init

	ldi acc,0x01	;enable IRQ: Timer0!!! 
	sts TIMSK0,acc
	sei
 

;*******   M A I N    ***************************************
	 
main:				
	;rjmp pm1			;skip playmodes sel
		;playmodes:
	in acc,PIND
	swap acc
	lsl acc
	bst acc,7
	bld acc,0
	andi acc,0x03
	cpi acc,2
	breq pm1
	cpi acc,1
	brne main1
	rjmp pm2
main1:
	cpi acc,0
	brne pm0
	rjmp pm3

pm0:				;PM0: basic fct.
	clr appoi
	sbrs fla,6		;25ms tick
	rjmp main
	cbr fla,0x40	;clear 25ms flag

						;Synth_A: read ADC2
	mov acc, ain2bu
	cpi acc,eps1
	brsh pm04
	ori fla,0x01		;set Synth_A mute if inp. level below Th
	rjmp pm05
pm04:
	;subi acc,9			; subtract offset FSR sensor
	mov x_SWa,acc
	sbi PORTB,Led1
pm05:					;Synth_B: read ADC3
	mov acc, ain3bu
	cpi acc,eps2
	brsh pm06
	ori fla,0x02		;set Synth_B mute if inp. level below Th
	rjmp pm07
pm06:
	;subi acc,9			; subtract offset FSR sensor
	mov x_SWb,acc
	;sbi PORTD,Led2

pm07:

					;START of Ring mod service
					;serve Timer2 PWM: ADC0-> Top-> det. Freq.
	mov acc,ain0bu	;ADC0-> Top-> det. Freq.
	mov ach,ain1bu	;ADC1-> Duty-cycle
	cpi acc,eps3
	brsh pm01
	clr acc			;switch off Ringmod
	sts OCR2A,acc
	sts OCR2B,acc
	rjmp pm02
pm01:
	;subi acc,20		;subtract offset FSR sensor
	com acc			;OCR val determines periode 
	cp acc,ach
	brsh pm03
	mov ach,acc
pm03:
	lsr ach
	inc ach
	sts OCR2A,acc
	sts OCR2B,ach	;END of Ring mod service
pm02:		
	rjmp main


pm1:				;PM1: autoplay tune
	sbrs fla,7		;0.25s tick
	rjmp pm1
	sbi PORTB,Led1
	cbr fla,0xc0	;clear flags
	ori fla,0x01	;set Synth_A mute for x25ms
pm10:
	sbrs fla,6		;25ms tick
	rjmp pm10
	cbr fla,0x40	;clear 25ms flag			
					;Z-access & lpm must be done imediatly
	ldi    ZL,low(play_tbl*2)
    ldi    ZH,high(play_tbl*2)
    clr    acc
	add    ZL,appoi
    adc    ZH,acc           
    lpm
	;sbic PINA,7
	;lsl r0
	mov x_SWa,r0
	inc appoi		;lpm low byte access
	ldi acc,4		;length of sample
	;ldi acc,48		;length of sample
	cp appoi,acc
	brlo pm10
	clr appoi
	;sbi PORTD,Led2
pm11:
	rjmp pm05	; go ahead with Synth_B & Ringmod service


pm2:				;PM2: autoplay tune polyphon
	sbrs fla,7		;0.25s tick
	rjmp pm2
	sbi PORTB,Led1
	cbr fla,0xc0	;clear flags
	ori fla,0x03	;set Synth_A & _B mute for x25ms
pm20:
	sbrs fla,6		;25ms tick
	rjmp pm20
	cbr fla,0x40	;clear 25ms flag			
					;Z-access & lpm must be done imediatly
	ldi    ZL,low(pla2_tbl*2)
    ldi    ZH,high(pla2_tbl*2)
    clr    acc
	add    ZL,appoi
    adc    ZH,acc           
    lpm
	mov x_SWa,r0
	inc appoi
	inc ZL
	adc ZH,acc
	lpm
	mov x_SWb,r0
	inc appoi
	ldi acc,48		;length of sample
	;ldi acc,24		;length of sample
	cp appoi,acc
	brlo pm21
	clr appoi
	;sbi PORTD,Led2
pm21:
	rjmp pm07	; go ahead with Ringmod service

pm3:				;PM3: beat box
	mov acc, ain2bu
	cpi acc,10
	brsh pm34
	ori fla,0x03		;set Synths mute if inp. level below Th
	rjmp pm0
pm34:
	com acc
	lsr acc
	cpi acc,2
	brsh pm33
	ldi acc,2
pm33:
	cpi acc,20
	brlo pm39
	ldi acc,20
pm39:
	sbrs fla,6		;25ms ticks
	rjmp pm33
	cbr fla,0xc0	;clear flags
	dec acc
	brne pm33
	ori fla,0x03	;set Synth_A & _B mute for x25ms
pm30:
	sbrs fla,6		;25ms tick
	rjmp pm30
	cbr fla,0x40	;clear 25ms flag			
					;Z-access & lpm must be done imediatly
	ldi    ZL,low(play_be*2)
    ldi    ZH,high(play_be*2)
    clr    acc
	add    ZL,appoi
    adc    ZH,acc           
    lpm
	tst r0
	brne pm32
	ori fla,0x03	;set Synth_A & _B mute	
pm32:
	mov ach, ain3bu
	lsr ach
	add r0,ach
	mov x_SWa,r0
	ldi acc,4
	add r0,acc
	mov x_SWb,r0
	inc appoi		;lpm low byte access
	ldi acc,6		;length of sample
	cp appoi,acc
	brlo pm31
	clr appoi
	;sbi PORTD,Led2
pm31:
	rjmp pm07	; go ahead with Ringmod service




	rjmp pm0


;************************************************************
;*********** I R Q 's SERVICE *******************************
;************************************************************

;**** timer0 overflow irq- service routine ******************
t0_irq:			
	
	push r0
	push acx
	push ach
	push acc
	push Zl
	push Zh
	in acc, SREG		;context save
	push acc

	sbi PORTB,1		;%TEST
						; no reload of timer2 in PWM mode...
						; PWM-sin service:
						; frequency A
   	sbrs fla,0			; goto mute state flag?
	rjmp t2irq1
	mov acc,adda_o		; switch off Synth near zero phase
	cpi acc,72
	brsh t2irq1			
	cpi acc,55
	brlo t2irq1
	clr x_SWa			; set synth_A mute
	clr ptr_La
	clr ptr_Ha
	cbi PORTB,Led1
	cbr fla,0x01		;clear flag
t2irq1:
   mov ptr_Ltmp,ptr_La
   mov ptr_Htmp,ptr_Ha
   rcall getsample      ; read from sample table
   mov adda_o,r0        ; adda_out = high frquency value
   add ptr_La,x_SWa
   clr acc              ; (acc is cleared, but not the carry flag)
   adc ptr_Ha,acc  
						; frequency B

	sbrs fla,1			; goto mute state flag?
	rjmp t2irq2
	mov acc,addb_o			; switch off Synth near zero phase
	cpi acc,72
	brsh t2irq2			
	cpi acc,55
	brlo t2irq2
	clr x_SWb			; set synth_A mute
	clr ptr_Lb
	clr ptr_Hb
	;cbi PORTD,Led2
	cbr fla,0x02		;clear flag
t2irq2:
   mov ptr_Ltmp,ptr_Lb
   mov ptr_Htmp,ptr_Hb
   rcall getsample      ; read from sample table
   mov addb_o,r0        ; adda_out = high frquency value
   add ptr_Lb,x_SWb
   clr acc              ; (acc is cleared, but not the carry flag)
   adc ptr_Hb,acc  
						; add freq. A & B contents
	mov acc,adda_o
    add acc, addb_o
    out OCR0a,acc

	inc ucnt
	cpi ucnt,1			;63.75µs für sattleing Mux passed?
	brne irq0
	lds acc,ADCSRA		;starten ADC
	ori acc,0x40
	sts ADCSRA,acc
irq0:
	cpi ucnt,4
	brne irqend
						;255µs tick
	clr ucnt
	rcall adchand		;get & store ADC 
	ori fla,0x20		;255µs flag
	
	inc cnt
	cpi cnt,t01val		;25ms tick?
	brne irqend
	clr cnt
	ori fla,0x40
irq1:	
	inc mscnt
	cpi mscnt,t1val		;0.25s tick?
	brne irqend
	
	clr mscnt
	ori fla,0x80	

irqend:
	cbi PORTB,1	;test&&&

	pop acc
	out SREG,acc
	pop Zh
	pop Zl
	pop acc
	pop ach
	pop acx
	pop r0

	reti


;************ end of timer0-overflow-irq-service-routine ****


;**************************************************************
;****	SUBs reffered by IRQ- service routines:   *************
;**************************************************************

;******************   GET SAMPLE   ******************************
;****************************************************************
getsample:
   ldi    acc,0x0f
   and    ptr_Htmp,acc

; ROUND	
   ldi    acc,4
   add    ptr_Ltmp,acc
   clr    acc
   adc    ptr_Htmp,acc

;shift (divide by eight):
   lsr    ptr_Htmp
   ror    ptr_Ltmp
   lsr    ptr_Htmp
   ror    ptr_Ltmp
   lsr    ptr_Htmp
   ror    ptr_Ltmp

   ldi    acc,0x7f
   and    ptr_Ltmp,acc                  ; module 128 (samples number sine table)

   ldi    ZL,low(sine_tbl*2)
   ldi    ZH,high(sine_tbl*2)
   add    ZL,ptr_Ltmp
   clr    acc
   adc    ZH,acc                        ; Z is a pointer to the correct
                                        ; sine_tbl value
   lpm
   ret

;**** adchand: ***********************************************
;* get & store ADC result routine for single channel   *******
;* test: ADC0-3												 *
adchand:	
				;ADCH 8bit LAR readout
	lds acc,ADCH		;
	inc adcpoi
	clr Zh
	mov Zl,adcpoi	;offset pointer/adr(r)=1
	st Z,acc
	cpi adcpoi,4
	brlo adchel
	clr adcpoi
adchel:	
	mov ach,adcpoi
adcfl:
	ori ach,0x20
	sts ADMUX,ach		;set mux
	
	ret		;************   end of adchand   *****


;************************************************************
;*********** S U B 's ***************************************
;************************************************************

;*** power-on initialisation:   *****************************

bas_init:
				;init Ports:
					;no PORTA!
	ldi r17,0x2f
	out DDRB,r17	;PortB
	ldi r17,0x00	; set SPI /CS, EA-dog Register sel.
	out PORTB,r17
	ldi r17,0x00
	out DDRC,r17	;PortC
	ldi r17,0x00	;no pull up's/ 
	out PORTC,r17
	ldi r17,0x6a
	out DDRD,r17	;PortD
	ldi r17,0x94	;set pull up's
	out PORTD,r17

			;clear counters / flags / pointers
	clr fla
	clr ucnt
	clr cnt
	clr mscnt
	clr adcpoi
	clr rsloop
	clr Zh
	clr Zl
	clr ptr_La	
	clr ptr_Ha	
	clr x_SWa
	clr ptr_Lb	
	clr ptr_Hb	
	clr x_SWb
	
				;init. timer0
	ldi acc,0x0f		;§§ Test
	out OCR0A,acc
	clr acc
	out TCNT0,acc
	ldi acc,0xa1		;ph_cor, non-inv. PWM; prescaler: 1
	out TCCR0A,acc		;PWM Freq.: 15.686kHz
	ldi acc,0x01
	out TCCR0B,acc
				;init Timer 2 phase corrected PWM-Mode
	ldi acc,0xff		;§§ Test
	sts OCR2A,acc
	ldi acc,0x80		;§§ Test
	sts OCR2B,acc
	clr acc
	sts TCNT2,acc
	sts ASSR,acc		;sync. clock mode
	ldi acc,0x21		;ph_cor, non-inv. PWM; TOP= OCR2A; prescaler: 64
	sts TCCR2A,acc		;PWM Freq.: 62.5kHz/Top -> Freq.min= 244Hz
	ldi acc,0x0c
	sts TCCR2B,acc
			
				;initialisation of ADC:
	ldi r16,0b10000110	;single conversation, no IRQs, 108µs conversion time
	sts ADCSRA,r16
	ldi r16,0b00000000	;no auto-trigger functions
	sts ADCSRB,r16
	ldi r16,0b00000011	;disable dig.inp.buf. PC0,1
	sts DIDR0,r16
	ldi r16,0b00100110	;ext.Uref, left right adj.,single ended ADC6
	sts ADMUX,r16
						;PRADC-bit in PRR is clear by preset

	ldi r16,0b11000110	;starten ADC	
	sts ADCSRA,r16

		
	
				;init SPI-Master:
	;ldi acc,0x52		;no SPI-IRQ/SPI enable/MSB first/
	;out SPCR,acc		;Master/rising edge/64'divider 64µs frames @ 8MHz	

				;initialisation of the RS232:
	ldi r16,103
	sts UBRR0L,r16		;4k8baud @ 8MHz
	ldi r16,0b00000000	;normal speed/ no MPCM
	sts UCSR0A,r16
	ldi r16,0b00011000	;no RS232-irq's, enable RX/TX, 8bit char.
	sts UCSR0B,r16	
	ldi r16,0b00000110	;asy-mode/no par/1 stop bit/8 bit frame/
	sts UCSR0C,r16		



	ret		;*************** end of bas_init ****************

	
;**** wait acc x (8x0.125)us   *******************************************
wait:
	mov acc,acc
	mov acc,acc
	mov acc,acc
	mov acc,acc
	mov acc,acc
	dec acc
	brne wait
	
	ret
		;****************** end of wait ************************
	
;*** stdel ***********************************************
;* standart delay time routine: del.time:                 *
;* zscnt(225µs)*[r18 +/-1] (r18 min:2!) r18=acx		  *
;
stdel:		
	clr axh

lblst1:
	sbrs fla,5
	rjmp lblst1
	inc axh
	cbr fla,0x20	
	cp axh,acx
	brne lblst1

	ret		;**** time is gone... *********	

;**** tx_byte: ********************************************
tx_byte:
	lds ach,UCSR0A
	sbrs ach,UDRE0		;transmitter buffer emty?
	rjmp tx_byte
	sts UDR0,acx		;start byte transmission

	ret		;*************** end of tx_byte **
	
	
;*** tx_a: ***********************************************
;* 't' command received:		                  *
;* send 'A' adc0, adc1, adc2, adc3			  *
;* 							  *		
tx_a:		
	ldi acx,65	;Ack received command
	rcall tx_byte
	
	mov acx,ain0bu
	rcall tx_byte
	mov acx,ain1bu
	rcall tx_byte
	mov acx,ain2bu
	rcall tx_byte
	mov acx,ain3bu
	rcall tx_byte
	
trgel:	clr acc
	ret
		;****************** end of tx_a ************************	
	
;*************************** SIN TABLE *************************************
; Samples table : one period sampled on 128 samples and
; quantized on 7 bit
;******************************************************************************
sine_tbl:
.db 64,67
.db 70,73
.db 76,79
.db 82,85
.db 88,91
.db 94,96
.db 99,102
.db 104,106
.db 109,111
.db 113,115
.db 117,118
.db 120,121
.db 123,124
.db 125,126
.db 126,127
.db 127,127
.db 127,127
.db 127,127
.db 126,126
.db 125,124
.db 123,121
.db 120,118
.db 117,115
.db 113,111
.db 109,106
.db 104,102
.db 99,96
.db 94,91
.db 88,85
.db 82,79
.db 76,73
.db 70,67
.db 64,60
.db 57,54
.db 51,48
.db 45,42
.db 39,36
.db 33,31
.db 28,25
.db 23,21
.db 18,16
.db 14,12
.db 10,9
.db 7,6
.db 4,3
.db 2,1
.db 1,0
.db 0,0
.db 0,0
.db 0,0
.db 1,1
.db 2,3
.db 4,6
.db 7,9
.db 10,12
.db 14,16
.db 18,21
.db 23,25
.db 28,31
.db 33,36
.db 39,42
.db 45,48
.db 51,54
.db 57,60
;****************************************************************
; play-data
		

play_be:
.db 10,10,13,10,00,00


pla2_tbl:
.db 33,25,00,25,00,25,33,25
.db 42,25,33,25,42,25,33,25
.db 25,19,00,19,00,19,25,19
.db 16,19,25,19,16,19,25,19
.db 33,21,00,21,00,21,33,21
.db 25,21,16,21,25,21,16,21


play_tbl:
.db 33,17,37,20,42,33,42,33
.db 25,00,00,25,16,25,16,25
.db 33,00,00,33,25,16,25,16

play_alt3:
.db 67,33,47,33,67,33,47,33
.db 59,33,47,33,53,33,47,33
.db 50,33,47,33,50,33,47,33
.db 50,33,47,33,50,33,47,33

play_alt2:
.db 22,25,22,31,28,25,22,28
.db 28,28,37,33,32,28,28,28
.db 28,32,28,25,00,00,00,00

play_alt1:
.db 19,19,21,24,25,24,21,19
.db 14,16,14,16,17,19,17,16
.db 17,16,17,19,21,19,25,24
.db 21,19,25,21,19,17,16,14
.db 25,00,00,00
;*****************************************************************************


.EXIT
