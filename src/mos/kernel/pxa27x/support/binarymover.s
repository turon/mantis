@@@@@@@@@@@@@@@@@@@@@@@@@
@ Author:		Josh Herbach
@ Revision:	1.0
@ Date:		09/02/2005
@@@@@@@@@@@@@@@@@@@@@@@@@@	

	.equ FLASH_READARRAY,(0x00FF)
	.equ FLASH_CFIQUERY,(0x0098)
	.equ FLASH_READSTATUS,(0x0070)
	.equ FLASH_CLEARSTATUS,(0x0050)
	.equ FLASH_PROGRAMWORD,(0x0040)
	.equ FLASH_PROGRAMBUFFER,(0x00E8)
	.equ FLASH_ERASEBLOCK,(0x0020)
	.equ FLASH_DLOCKBLOCK,(0x0060)
	.equ FLASH_PROGRAMBUFFERCONF,(0x00D0)
	.equ FLASH_LOCKCONF,(0x0001)
	.equ FLASH_UNLOCKCONF,(0x00D0)
	.equ FLASH_ERASECONF,(0x00D0)
	.equ FLASH_OP_NOT_SUPPORTED,(0x10)
	.equ ARM_CPSR_INT_MASK,(0xC0)

	.global __Binary_Mover
	.global __Binary_Mover_true_end

__Binary_Mover:
.func __Binary_Mover	@r0 = addr (move to r1)
	                @r1 = size (move to r2)

	mov r2,r1			@Put size into r2
	mov r1,r0			@Put addr into r1

	ldr r5,=ARM_CPSR_INT_MASK		@Store int mask value
	mrs r4,CPSR				@Store Core Program Status Reg
	orr r4,r4,r5				@Add int mask
	msr CPSR,r4				@Replaces previous CPSR value
	
	ldr r5,=0x40000000
	add r5,r5,#0xe00000
	add r5,r5,#0x0100
	add r5,r5,#0x0c				@r5 contains 0x40e0 010c, address of GPDR3
	ldr r6,[r5]
	ldr r7,=0x7
	orr r6,r6,r7,LSL #7
	str r6,[r5]				@should set GPIO 103-105 to output
	
	ldr r6,=0x18				@offset between GPDR3 and GPCR3
	add r5,r5,r6				@r5 contains 0x40e0 0124
	ldr r6,=0x7
	mov r6,r6,LSL #7
	str r6,[r5]				@Turn on 3 LEDs
	
	sub r9,r5,#0xC				@r9 will store GPSR3, 0x40e0 0118
	
	ldr r4,=0x0				@Counter
	cmp r4,r2
	beq __Binary_Mover_end		@If imagesize is 0 then skip a bunch of stuff
	
	ldr r5,=0x200000
	ldr r7,=0x2000000			@Final flash addr 0x0200 0000
	@preset all partitions to ReadArray
	__Binary_Mover_RA_setup:
	ldr r6,=FLASH_READARRAY
	strh r6,[r4]				@Readarray
	ldrh r6,[r4]				@Latch?
	add r4,r4,r5				@Increment
	cmp r4,r7
	blo __Binary_Mover_RA_setup	@Repeat
	
	ldr r6,=0x1				@switching to 2 LEDs
	mov r6,r6,LSL #7
	str r6,[r9]				@Turn off 1 LED
	
	ldr r4,=0x0				@Counter
	ldr r5,=0x8000				@Block size 0x0000 8000 initially...moves up to 0x0002 0000
	__Binary_Mover_Erase_loop:
	ldr r6,=FLASH_CLEARSTATUS		@Unlock block
	strh r6,[r4]				@Clear Status
	ldr r6,=FLASH_DLOCKBLOCK
	strh r6,[r4]				@Change lock
	ldr r6,=FLASH_UNLOCKCONF
	strh r6,[r4]				@Confirm Unlock
	ldr r6,=FLASH_READARRAY
	strh r6,[r4]				@Return to read array
	ldrh r6,[r4]				@latch?
		
	ldr r6,=FLASH_CLEARSTATUS
	strh r6,[r4]				@Clear Status register
	ldr r6,=FLASH_ERASEBLOCK
	strh r6,[r4]				@Send EraseBlock command
	ldr r6,=FLASH_ERASECONF
	strh r6,[r4]				@Confirm Erase Block
	EraseBlockSpin:
	ldrh r6,[r4]				@Read Block Status
	and r6,r6,#0x80
	cmp r6,#0x0				@Check if Erase is complete
	beq EraseBlockSpin			@Spin if not complete
	ldr r6,=FLASH_READARRAY
	strh r6,[r4]				@Set Block back to normal read mode
	ldrh r6,[r4]				@latch?
	
	add r4,r4,r5				@Increment to next block
	
	ldr r6,=0x20				@Check if Block size needs to increase
	mov r6,r6,LSL #0xc			@starting address for larger size
	cmp r4,r6
	bne __Binary_Mover_Erase_loop_end	@if r4!=r6 continue
	mov r5,r5,LSL #0x2			@Times 4 making 0x8000 become 0x2 0000
	
	__Binary_Mover_Erase_loop_end:
	cmp r4,r2
	blo __Binary_Mover_Erase_loop		@Repeat
	
	ldr r6,=0x1				@switching to 1 LEDs
	mov r6,r6,LSL #8
	str r6,[r9]				@Turn off 1 LED

	ldr r4,=0x0				@Reset counter for writing
	__Binary_Mover_Write_loop:
	ldrh r6,[r1, r4]			@Get data
	
	ldr r5,=FLASH_CLEARSTATUS		@Write word start
	strh r5,[r4]				@Clear Status register
	ldr r5,=FLASH_PROGRAMWORD
	strh r5,[r4]				@Send Program Word Command
	strh r6,[r4]				@Write Word
	WriteWordSpin:
	ldrh r5,[r4]				@Read Block Status
	and r5,r5,#0x80
	cmp r5,#0x0				@Check if Write is complete
	beq WriteWordSpin			@Spin if not complete
	ldr r5,=FLASH_READARRAY
	strh r5,[r4]				@Set Block back to normal read mode
	ldrh r5,[r4]				@Word written
	
	add r4,r4,#0x2				@Increment Counter
	cmp r4,r2
	blo __Binary_Mover_Write_loop		@Repeat
	
	__Binary_Mover_end:
	ldr r6,=0x1				@switching to 0 LEDs
	mov r6,r6,LSL #9
	str r6,[r9]				@Turn off 1 LED
	
	
	ldr r5,=0x40000000
	add r5,r5,#0xa00000
	add r5,r5,#0x18				@r5 contains 0x40a0 0018, OWER
	ldr r6,=0x1
	str r6,[r5]				@enables watchdog timer reset
	
	sub r5,r5,#0xC				@r5 contains 0x40a0 000c OSMR3
	add r4,r5,#0x4				@r4 contains 0x40a0 0010 OSCR0
	ldr r6,[r4]
	add r6,r6,#0x10000
	str r6,[r5]				@osmr3 = oscr0 + 0x10000
	__Binary_Mover_spin:
	b __Binary_Mover_spin		@spin until watchdog timer resets this
	
	ldr PC,=0x0					@Reset
	__Binary_Mover_true_end:
	nop
.endfunc
