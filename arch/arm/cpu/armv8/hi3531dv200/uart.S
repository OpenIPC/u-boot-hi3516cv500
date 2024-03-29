/*
 * Copyright (c) 2018 HiSilicon Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */


#include <config.h>

//******************************************************************************
//
//  void uart_early_init(void);
//
.text
.align	4
.global	uart_early_init
.type	uart_early_init, %function
uart_early_init:
#ifndef CONFIG_HISI_DISABLE_CONSOLE
	ldr	x1, =SYS_CTRL_REG_BASE
	ldr	w2, [x1, #REG_SYSSTAT]
	lsr	w2, w2, #29
	cmp	w2, #1
	beq	for_vo_bt1120_cascade

	ldr	x4, =UATR_IO_CFG_BASE
	ldr	w3, =0x531
	str	w3, [x4, #UART0_RXD_IOCFG_OFST]
	ldr	w3, =0x431
	str	w3, [x4, #UART0_TXD_IOCFG_OFST]
	b	uart_ctrl_init

for_vo_bt1120_cascade:
	ldr	x4, =REG_VO_BT1120_CLK
	ldr	w3, =0x5f1
	str	w3, [x4]
	ldr	x4, =REG_VO_BT1120_DATA0
	ldr	w3, =0x471
	str	w3, [x4]

uart_ctrl_init:
	ldr	x4, uart_base_addr_L0
	mov	w3, #0
	/* Disable UART */
	str	w3, [x4, #48]
	/* Set baud rate to 115200, uart clock:24M */
	add	w3, w3, #13
	str	w3, [x4, #36]
	mov	w3, #1
	str	w3, [x4, #40]
	/* Set the UART to be 8 bits, 1 stop bit, no parity, fifo enabled. */
	ldr	w3, =112
	str	w3, [x4, #44]
	/* Enable UART */
	ldr	w3, =769
	str	w3, [x4, #48]
#endif
	ret
.align	4
uart_base_addr_L0:
	.quad CONFIG_CUR_UART_BASE

//******************************************************************************
//
//  void uart_early_puts(const char *ss);
//
.align	4
.global	uart_early_puts
.type	uart_early_puts, %function
uart_early_puts:
#ifndef CONFIG_HISI_DISABLE_CONSOLE
#if !defined(CONFIG_SUPPORT_CA_RELEASE)
	ldr	x2, uart_base_addr_L1
	b	next_char
output:
	ldr	w4, [x2, #24]
	tst	w4, #32
	bne	output
	str	w3, [x2, #0]
	add	x0, x0, #1
next_char:
	ldrb	w3, [x0]
	cmp	w3, #0
	bne	output
#endif /* CONFIG_SUPPORT_CA_RELEASE */
#endif /* CONFIG_HISI_DISABLE_CONSOLE */
	ret
.align	4
uart_base_addr_L1:
	.quad CONFIG_CUR_UART_BASE

//******************************************************************************
//
//  void uart_early_putc(int chr);
//
//  call example:
//    mov	w0, #'A'
//    bl	uart_early_putc
//
.align	4
.global	uart_early_putc
.type	uart_early_putc, %function
uart_early_putc:
#ifndef CONFIG_HISI_DISABLE_CONSOLE
#if !defined(CONFIG_SUPPORT_CA_RELEASE)
	ldr	x2, uart_base_addr_L3
wait3:
	ldr	w4, [x2, #24]
	tst	w4, #32
	bne	wait3
	str	w0, [x2, #0]

#endif /* CONFIG_SUPPORT_CA_RELEASE */
#endif /* CONFIG_HISI_DISABLE_CONSOLE */
	ret
.align	4
uart_base_addr_L3:
	.quad CONFIG_CUR_UART_BASE
