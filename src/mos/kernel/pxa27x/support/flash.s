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


	.global __Flash_Erase
	.global __Flash_Program_Word
	.global __Flash_Program_Buffer
	.global __Flash_Erase_true_end
	.global __Flash_Program_Word_true_end
	.global __Flash_Program_Buffer_true_end
	

__Flash_Erase:
.func __Flash_Erase @r0 (return) = status
					@r0 = addr (move to r1)
					
	STMFD R13!, {R1, R4 - R5, LR} @I'm being conservative and saving registers R1-R3 which might not be necessary with function calls
	mov r1,r0
	ldr r4,=0x0					@Flash Base Addr
	ldr r5,=FLASH_CLEARSTATUS
	strh r5,[r4]				@Clear Status register
	ldr r5,=FLASH_ERASEBLOCK
	strh r5,[r1]				@Send EraseBlock command
	ldr r5,=FLASH_ERASECONF
	strh r5,[r1]				@Confirm Erase Block
	_goEraseFlashSpin:
	ldrh r5,[r1]				@Read Block Status
	and r5,r5,#0x80
	cmp r5,#0x0					@Check if Erase is complete
	beq _goEraseFlashSpin		@Spin if not complete
	ldrh r0,[r1]				@Read / return status
	ldr r5,=FLASH_READARRAY
	strh r5,[r1]				@Set Block back to normal read mode
	ldrh r5,[r1]
	LDMFD R13!, {R1, R4 - R5, PC}
	__Flash_Erase_true_end:
	nop;
.endfunc

__Flash_Program_Word:
.func __Flash_Program_Word	@r0 (return) = status
							@r0 (move to r1) = addr
							@r1 (move to r2) = dataword
							
	STMFD R13!, {R1 - R2, R4 - R5, LR} @I'm being conservative and saving registers R1-R3 which might not be necessary with function calls
	mov r2,r1
	mov r1,r0	
	ldr r4,=0x0					@Flash Base Addr
	ldr r5,=FLASH_CLEARSTATUS
	strh r5,[r4]				@Clear Status register
	ldr r5,=FLASH_PROGRAMWORD
	strh r5,[r1]				@Send Program Word Command
	strh r2,[r1]				@Write Word
	_goProgramWordSpin:
	ldrh r5,[r1]				@Read Block Status
	and r5,r5,#0x80
	cmp r5,#0x0					@Check if Write is complete
	beq _goProgramWordSpin		@Spin if not complete
	ldrh r0,[r1]				@Read / return status
	ldr r5,=FLASH_READARRAY
	strh r5,[r1]				@Set Block back to normal read mode
	ldrh r5,[r1]
	LDMFD R13!, {R1 - R2, R4 - R5, PC}
	__Flash_Program_Word_true_end:
	nop;
.endfunc

__Flash_Program_Buffer:
.func __Flash_Program_Buffer	@r0 (return) = status
								@r0 (move to r1) = addr
								@r1 (move to r2) = datadata
								@r2 (move to r3) = datalen
					 
	STMFD R13!, {R1 - R12, LR} @I'm being conservative and saving registers R1-R3 which might not be necessary with function calls
	mov r3,r2
	mov r2,r1
	mov r1,r0
	ldr r4,=0x0					@Flash Base Addr
	ldr r5,=FLASH_CLEARSTATUS
	strh r5,[r4]				@Clear Status register
	ldr r5,=FLASH_PROGRAMBUFFER
	strh r5,[r1]				@Send Program Buffer Command
	ldrh r5,[r1]				@Read Block Status
	and r5,r5,#0x80
	cmp r5,#0x0					@Check if Program Buffer works with this flash
	beq _goProgramBufferNS		@Program Buffer Not Supported, Jump

	strh r3, [r1]				@Send number of words to write
	_goProgramBufferLoop:
	ldrh r5, [r2,r4]			@Temporarily Store Word
	strh r5, [r1,r4]			@Write Word
	add r4,r4,#2				@Increment Counter
	cmp r4, r3, LSL #1			@Check if all words written
	ble _goProgramBufferLoop	@If all words written, continue
	ldr r5,=FLASH_PROGRAMBUFFERCONF @Confirm Program Buffer
	strh r5,[r1]
	_goProgramBufferSpin:
	ldrh r5,[r1]				@Read Block Status
	and r5,r5,#0x80
	cmp r5,#0x0					@Check if Write is complete
	beq _goProgramBufferSpin	@Spin if not complete
	ldrh r0,[r1]				@Read / return status
	b _goProgramBufferEnd
	_goProgramBufferNS:
	ldrh r0,=FLASH_OP_NOT_SUPPORTED		@Program Buffer is not supported
	mov r0,r0,LSL #1					@Return operation not supported
	_goProgramBufferEnd:
	ldr r5,=FLASH_READARRAY
	strh r5,[r1]				@Set Block back to normal read mode
	ldrh r5,[r1]
	LDMFD R13!, {R1 - R12, PC}
	__Flash_Program_Buffer_true_end:
	nop;
.endfunc
