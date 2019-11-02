/*
 * Copyright (c) 2019 Damien Zammit
 */

#include <sys/types.h>
#include <i386/ipl.h>
#include <i386/fpu.h>
#include <i386/hardclock.h>
#include <i386at/kd.h>
#include <i386at/idt.h>
#include "imps/apic.h"

#define NINTR (IOAPIC_NINTR + 1)

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

static uint32_t
ioapic_read(uint8_t apic, uint8_t reg)
{
    uint32_t volatile *hw = (uint32_t volatile *)ioapic[apic].addr;
    hw[0] = reg;
    return hw[4];
}

static void
ioapic_write(uint8_t apic, uint8_t reg, uint32_t value)
{
    uint32_t volatile *hw = (uint32_t volatile *)ioapic[apic].addr;
    hw[0] = reg;
    hw[4] = value;
}

static struct ioapic_route_entry
ioapic_read_entry(int apic, int pin)
{
    union ioapic_route_entry_union entry;

    entry.lo = ioapic_read(apic, APIC_IO_REDIR_LOW(pin));
    entry.hi = ioapic_read(apic, APIC_IO_REDIR_HIGH(pin));

    return entry.both;
}

/* Write the high word first because mask bit is in low word */
static void
ioapic_write_entry(int apic, int pin, struct ioapic_route_entry e)
{
    union ioapic_route_entry_union entry = {{0, 0}};

    entry.both = e;
    ioapic_write(apic, APIC_IO_REDIR_HIGH(pin), entry.hi);
    ioapic_write(apic, APIC_IO_REDIR_LOW(pin), entry.lo);
}

/* When toggling the interrupt via mask, write low word only */
static void
ioapic_toggle_entry(int apic, int pin, int mask)
{
    union ioapic_route_entry_union entry;

    entry.both = ioapic_read_entry(apic, pin);
    entry.both.mask = mask & 0x1;
    ioapic_write(apic, APIC_IO_REDIR_LOW(pin), entry.lo);
}

static void
lapic_enable_ioapic(void)
{
    apic_local_unit.spurious_vector.r
	|= LAPIC_ENABLE_SPURIOUS | IOAPIC_SPURIOUS_BASE;
}

void
ioapic_toggle(int pin, int mask)
{
    int apic = 0;

    ioapic_toggle_entry(apic, pin, mask);
}

void
lapic_eoi(void)
{
    apic_local_unit.eoi.r = 0;
}


void
ioapic_configure(void)
{
    int i, pin, apic = 0;
    union ioapic_route_entry_union entry = {{0, 0}};

    entry.both.delvmode = IOAPIC_FIXED;
    entry.both.destmode = IOAPIC_PHYSICAL;
    entry.both.mask = IOAPIC_MASK_ENABLED;
    entry.both.dest = ioapic[apic].apic_id & 0xf;

    /* ISA legacy IRQs */
    entry.both.polarity = IOAPIC_ACTIVE_HIGH;
    entry.both.trigger = IOAPIC_EDGE_TRIGGER;

    for (pin = 0; pin < 16; pin++) {
        entry.both.vector = IOAPIC_INT_BASE + pin;
	ioapic_write_entry(apic, pin, entry.both);
    }

    /* PCI IRQs PIRQ A-H */
    entry.both.polarity = IOAPIC_ACTIVE_LOW;
    entry.both.trigger = IOAPIC_LEVEL_TRIGGER;

    for (pin = 16; pin < 24; pin++) {
        entry.both.vector = IOAPIC_INT_BASE + pin;
	ioapic_write_entry(apic, pin, entry.both);
    }

    lapic_enable_ioapic();
}
