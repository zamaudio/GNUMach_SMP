/*
 * Copyright (c) 2019 Damien Zammit
 */

#include <sys/types.h>
#include <i386/ipl.h>
#include <i386/pic.h>
#include <i386/fpu.h>
#include <i386/hardclock.h>
#include <i386at/kd.h>
#include "imps/apic.h"

#define NINTR 25

void (*ivect[NINTR])() = {
	/* 00 */	hardclock,	/* always */
	/* 01 */	kdintr,		/* kdintr, ... */
	/* 02 */	intnull,
	/* 03 */	intnull,	/* lnpoll, comintr, ... */

	/* 04 */	intnull,	/* comintr, ... */
	/* 05 */	intnull,	/* comintr, wtintr, ... */
	/* 06 */	intnull,	/* fdintr, ... */
	/* 07 */	prtnull,	/* qdintr, ... */

	/* 08 */	intnull,
	/* 09 */	intnull,	/* ether */
	/* 10 */	intnull,
	/* 11 */	intnull,

	/* 12 */	intnull,
	/* 13 */	fpintr,		/* always */
	/* 14 */	intnull,	/* hdintr, ... */
	/* 15 */	intnull,	/* ??? */

	/* 16 */	intnull,	/* PIRQA */
	/* 17 */	intnull,	/* PIRQB */
	/* 18 */	intnull,	/* PIRQC */
	/* 19 */	intnull,	/* PIRQD */
	/* 20 */	intnull,	/* PIRQE */
	/* 21 */	intnull,	/* PIRQF */
	/* 22 */	intnull,	/* PIRQG */
	/* 23 */	intnull,	/* PIRQH */
	
	/* 255 */	intnull,	/* spurious */
};
