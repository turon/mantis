;  This file is part of MANTIS OS, Operating System
;  See http//mantis.cs.colorado.edu/
;
;  Copyright (C) 2003-2007 University of Colorado, Boulder
;
;  This program is free software; you can redistribute it and/or
;  modify it under the terms of the mos license (see file LICENSE)



;  Project Mantis
;  File: flash.S
;  Author: Jeff Rose
;  Date: 1-21-03

;  Implements functions to read and write flash memory on Atmega128.

#include <avr/io.h>

#define PAGESIZE 256
	
		;; Read a word from program memory
.global flash_read_word
flash_read_word:
	MOV     ZH,R25     ;Move addr to Z (R31=ZH R30=ZL)
        MOV     ZL,R24     
        ELPM                ;read LSB        
        MOV     R25,R0      ;read LSB           
        INC     ZL
        ELPM                ;read LSB        
        MOV     R24,R0      ;read MSB (ignored when reading lockbits)
	RET

	;; Read a fuse setting from flash
.global flash_read_fuse
flash_read_fuse:
	PUSH	R18

	MOV     ZH,R25     ;Move addr to Z (R31=ZH R30=ZL)
        MOV     ZL,R24
	LDI	R18,0x09    ; Set the BLB & SPMEN bits
	STS	SPMCSR,R18  
        ELPM                ; Read the fuse
        MOV     R24,R0      

	POP	R18
	RET

	;; flash_write_page(void *flash_addr, uint8_t *buf)
.global flash_write_page
flash_write_page:
	PUSH	R0
	PUSH	R1
	PUSH	R18
	PUSH	R19
	PUSH	R22
	PUSH	R23
	PUSH	R24
	PUSH	R25
	PUSH	R28
	PUSH	R29
	PUSH	R30
	PUSH	R31
	
	MOV	ZH,R25		; Load the flash addr into Z
	MOV     ZL,R24
	MOV	R29,R23		; Load buf addr into Y
	MOV	R28,R22

	;; First erase the page in flash
	LDI	R18,0x03	; Load PGERS and SPMEN
	RCALL	do_spm
	
	;; Now write the data into the temporary page buffer
	LDI	R22,0x00       ; Init loop variable w/ 0x00 for PAGESIZE = 256
	
write_loop:
	LD	R0,Y+		; Copy the word from RAM to R1:R0
	LD	R1,Y+
	LDI	R18,0x01	; Load SPMEN
spm_write:
	LDS     R19,SPMCSR	; Loop until the spm bit is clear
        SBRC    R19,0
        RJMP	spm_write    
	STS	SPMCSR,R18
	SPM			; Execute the write
	
	INC	ZL		; Increment Z (flash addr)
	INC	ZL	
	SUBI	R22,2		; Decrement the loop variable
	BRNE	write_loop	; Loop until loop variable is zero

	;; Now write the temporary buffer into flash
	MOV	ZH,R25		; Load the flash addr into Z
	MOV     ZL,R24
	LDI	R18,0x05	; Load PGWRT and SPMEN
	RCALL	do_spm		; Write the page
	
	POP	R31
	POP	R30
	POP	R29
	POP	R28
	POP	R25
	POP	R24
	POP	R23
	POP	R22
	POP	R19
	POP	R18
	POP	R1
	POP	R0
	
	RET

.global flash_enable_rww
flash_enable_rww:	
		;; Re-enable the RWW section
	LDI	R18,0x11	; Load RWWSRE and SPMEN
	;; Now we just fall into do_spm and return...
	
		
	;; Execute some SPM operation after making sure SPMEN is clear
do_spm:
	;; First make sure any previous SPM operation has completed
	LDS     R19,SPMCSR
        SBRC    R19,0
        RJMP    do_spm

	;; Now do the SPM operation
	STS	SPMCSR,R18
	SPM
      	RET
