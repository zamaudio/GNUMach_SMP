/* 
 * Copyright (c) 1995 Shantanu Goel
 * Copyright (c) 2019 Damien Zammit
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

/*
 * spl routines for the i386at.
 */

#include <mach/machine/asm.h>
#include <i386/ipl.h>
#include <i386/i386asm.h>

#if NCPUS > 1
#define mb lock; addl $0,(%esp)
#else
#define mb
#endif

/*
 * Set IPL to the specified value.
 */
#define SETIPL(level)			\
	mb;				\
	movl	$(level),%edx;		\
	cmpl	EXT(curr_ipl),%edx;	\
	jne	spl;			\
	movl	%edx,%eax;		\
	ret

/*
 * Program PICs with mask in %eax.
 */
#define SETMASK()				\
	cmpl	EXT(curr_pic_mask),%eax;	\
	je	9f;				\
	outb	%al,$(PIC_MASTER_OCW);		\
	movl	%eax,EXT(curr_pic_mask);	\
	movb	%ah,%al;			\
	outb	%al,$(PIC_SLAVE_OCW);		\
9:

ENTRY(spl0)
	mb;
	movl	EXT(curr_ipl),%eax	/* save current ipl */
	pushl	%eax
	cli				/* disable interrupts */
	cmpl	$0,softclkpending	/* softclock pending? */
	je	1f			/* no, skip */
	movl	$0,softclkpending	/* clear flag */
	call	EXT(spl1)		/* block further interrupts */
	call	EXT(softclock)		/* go handle interrupt */
	cli				/* disable interrupts */
1:
	cmpl	$(SPL0),EXT(curr_ipl)	/* are we at spl0? */
	je	1f			/* yes, all done */
	movl	$(SPL0),EXT(curr_ipl)	/* set ipl */
	movl	EXT(pic_mask)+SPL0*4,%eax
					/* get PIC mask */
	SETMASK()			/* program PICs with new mask */
1:
	sti				/* enable interrupts */
	popl	%eax			/* return previous mask */
	ret

Entry(splsoftclock)
ENTRY(spl1)
	SETIPL(SPL1)

ENTRY(spl2)
	SETIPL(SPL2)

ENTRY(spl3)
	SETIPL(SPL3)

Entry(splnet)
Entry(splhdw)
ENTRY(spl4)
	SETIPL(SPL4)

Entry(splbio)
Entry(spldcm)
ENTRY(spl5)
	SETIPL(SPL5)

Entry(spltty)
Entry(splimp)
Entry(splvm)
ENTRY(spl6)
	SETIPL(SPL6)

Entry(splclock)
Entry(splsched)
Entry(splhigh)
Entry(splhi)
ENTRY(spl7)
	mb;
	/* ipl7 just clears IF */
	movl	$SPL7,%eax
	xchgl	EXT(curr_ipl),%eax
	cli
	ret

ENTRY(splx)
	movl	S_ARG0,%edx		/* get ipl */

	testl	%edx,%edx		/* spl0? */
	jz	EXT(spl0)		/* yes, handle specially */
	cmpl	EXT(curr_ipl),%edx	/* same ipl as current? */
	jne	spl			/* no */
	cmpl	$SPL7,%edx		/* spl7? */
	je	1f			/* to ipl7, don't enable interrupts */
	sti				/* ensure interrupts are enabled */
1:
	movl	%edx,%eax		/* return previous ipl */
	ret

/*
 * Like splx() but returns with interrupts disabled and does
 * not return the previous ipl.  This should only be called
 * when returning from an interrupt.
 */
	.align	TEXT_ALIGN
	.globl	splx_cli
splx_cli:
	movl	S_ARG0,%edx		/* get ipl */
	cli				/* disable interrupts */
	testl	%edx,%edx		/* spl0? */
	jnz	2f			/* no, skip */
	cmpl	$0,softclkpending	/* softclock pending? */
	je	1f			/* no, skip */
	movl	$0,softclkpending	/* clear flag */
	call	EXT(spl1)		/* block further interrupts */
	call	EXT(softclock)		/* go handle interrupt */
	cli				/* disable interrupts */
1:
	xorl	%edx,%edx		/* edx = ipl 0 */
2:
	cmpl	EXT(curr_ipl),%edx	/* same ipl as current? */
	je	1f			/* yes, all done */
	movl	%edx,EXT(curr_ipl)	/* set ipl */
	movl	EXT(pic_mask)(,%edx,4),%eax
					/* get PIC mask */
	SETMASK()			/* program PICs with new mask */
1:
	ret

/*
 * NOTE: This routine must *not* use %ecx, otherwise
 * the interrupt code will break.
 */
	.align	TEXT_ALIGN
	.globl	spl
spl:
	cmpl	$SPL7,%edx		/* spl7? */
	je	EXT(spl7)		/* yes, handle specially */
	movl	EXT(pic_mask)(,%edx,4),%eax
					/* get PIC mask */
	cli				/* disable interrupts */
	xchgl	EXT(curr_ipl),%edx	/* set ipl */
	SETMASK()			/* program PICs with new mask */
	sti				/* enable interrupts */
	movl	%edx,%eax		/* return previous ipl */
	ret

ENTRY(sploff)
	pushfl
	popl	%eax
	cli
	ret

ENTRY(splon)
	pushl	4(%esp)
	popfl
	ret

	.data
	.align	DATA_ALIGN
softclkpending:
	.long	0
	.text

ENTRY(setsoftclock)
	incl	softclkpending
	ret
