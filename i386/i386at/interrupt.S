/* 
 * Copyright (c) 1995 Shantanu Goel
 * All Rights Reserved.
 * 
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 * 
 * THE AUTHOR ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  THE AUTHOR DISCLAIMS ANY LIABILITY OF ANY KIND FOR
 * ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 */

#include <mach/machine/asm.h>

#include <i386/ipl.h>
#include "imps/apic.h"
#include <i386/i386asm.h>

/*
 * Generic interrupt handler.
 *
 * On entry, %eax contains the irq number.
 */
ENTRY(interrupt)
	pushl	%eax			/* save irq number */
	movl	%eax,%ecx		/* copy irq number */
	cmpl	$255,%eax		/* was this a spurious intr? */
	je	0f			/* if so, null handler */
	shll	$2,%ecx			/* irq * 4 */
	call	spl7			/* set ipl */
	movl	EXT(iunit)(%ecx),%edx	/* get device unit number */
	pushl	%eax			/* push previous ipl */
	pushl	%edx			/* push unit number */
	call	*EXT(ivect)(%ecx)	/* call interrupt handler */
	addl	$4,%esp			/* pop unit number */
	call	splx_cli		/* restore previous ipl */
	addl	$4,%esp			/* pop previous ipl */

	cli				/* XXX no more nested interrupts */
	popl	%eax			/* restore irq number */
	movl	%eax,%ecx		/* copy irq number */
	call	EXT(lapic_eoi)		/* EOI */
	ret				/* return */
0:
	popl	%eax			/* restore irq number */
	movl	%eax,%ecx		/* copy irq number */
	ret				/* return */
END(interrupt)
