; AABB.nasm (c) 2015 Mischa "Aster" Alff
; x86-64 Assembly implementations of AABB intersection checks.
; The algorithm in use here is:
;   !(B.left>A.right || B.right<A.left || B.top>A.bottom || B.bottom<A.top)


section .data
section .bss
section .text
	global check_aabb_point

	; Checks if AABB contains point
	; takes three arguments, aabb_max, aabb_min, point
	check_aabb_point:
		movaps xmm0, [rdi]
		movaps xmm1, [rsi]
		movaps xmm2, [rdx]

		; If aabb.min > point
		pcmpgtq xmm1, xmm2
		pmovmskb rax, xmm1
		and eax, 0x1

		; If point > aabb.max
		pcmpgtq xmm2, xmm0
		pmovmskb rbx, xmm2
		and eax, ebx
