.data
.align 2
.word 0, 8 
.word 0x20, 0, 0, 0, 0, 0

.word 1, 2, 4, 6, 8, 12, 20, 34

.text
jal x0, main

# takes a value from A0, calculates log and saves it to A1
floor_log:
	addi a1, a0, 5
	jalr x0, 0(ra)

main:
	lw t0, 4(zero)
	lw t1, 0x8(zero)
arr_cycle:
	beq t0, x0, end
	lw a0, 0(t1)
	jal floor_log
	sw a1, 0(t1)
	addi t1, t1, 4
	addi t0,t0,-1
	jal x0, arr_cycle

end:
	jal x0, end