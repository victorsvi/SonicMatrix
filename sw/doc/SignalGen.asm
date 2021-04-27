; This is a disassembled file with the inline assembly from the ultrasonic matrix (sw/src/src.ino)

LP	RST
		loop():
		...\SonicMatrix\sw\src/src.ino:465
	2	    102a:	80 91 9b 18 	lds	r24, 0x189B
		...\SonicMatrix\sw\src/src.ino:466
	2	    102e:	90 91 9c 18 	lds	r25, 0x189C
		...\SonicMatrix\sw\src/src.ino:469
	2	    1032:	e0 91 08 02 	lds	r30, 0x0208
	2	    1036:	f0 91 09 02 	lds	r31, 0x0209
	1	    103a:	17 e0       	ldi	r17, 0x07	; 7
	2	    103c:	0d c0       	rjmp	.+26     	; 0x1058 <main+0x380>
1		    103e:	00 00       	nop
1		    1040:	00 00       	nop
1		    1042:	00 00       	nop
1		    1044:	00 00       	nop
1		    1046:	00 00       	nop
1		    1048:	00 00       	nop
1		    104a:	00 00       	nop
1		    104c:	00 00       	nop
1		    104e:	00 00       	nop
1		    1050:	00 00       	nop
1		    1052:	00 00       	nop
1		    1054:	00 00       	nop
1		    1056:	00 00       	nop
2	2	    1058:	01 91       	ld	r16, Z+
1	1	    105a:	02 b9       	out	0x02, r16	; 2
2	2	    105c:	01 91       	ld	r16, Z+
1	1	    105e:	05 b9       	out	0x05, r16	; 5
2	2	    1060:	01 91       	ld	r16, Z+
1	1	    1062:	08 b9       	out	0x08, r16	; 8
2	2	    1064:	01 91       	ld	r16, Z+
1	1	    1066:	0b b9       	out	0x0b, r16	; 11
2	2	    1068:	01 91       	ld	r16, Z+
1	1	    106a:	01 bb       	out	0x11, r16	; 17
2	2	    106c:	01 91       	ld	r16, Z+
1	1	    106e:	04 bb       	out	0x14, r16	; 20
2	2	    1070:	01 91       	ld	r16, Z+
2	2	    1072:	00 93 02 01 	sts	0x0102, r16
2	2	    1076:	01 91       	ld	r16, Z+
2	2	    1078:	00 93 05 01 	sts	0x0105, r16
2	2	    107c:	01 91       	ld	r16, Z+
2	2	    107e:	00 93 08 01 	sts	0x0108, r16
2	2	    1082:	01 91       	ld	r16, Z+
2	2	    1084:	00 93 0b 01 	sts	0x010B, r16
1	1	    1088:	1a 95       	dec	r17
2	1	    108a:	c9 f6       	brne	.-78     	; 0x103e <main+0x366>
	1	    108c:	89 17       	cp	r24, r25
	2	    108e:	69 f2       	breq	.-102    	; 0x102a <main+0x352>
	
50	50	:D	
	