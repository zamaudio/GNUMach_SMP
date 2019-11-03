/*
 * Copyright (c) 2019 Damien Zammit
 */

#include <sys/types.h>
#include <i386/ipl.h>
#include <i386/fpu.h>
#include <i386/hardclock.h>
#include <i386at/kd.h>
#include <i386at/idt.h>
#include <i386/pio.h>
#include <i386/pit.h>
#include <kern/printf.h>
#include "imps/apic.h"

spl_t	curr_ipl;
int	pic_mask[NSPL] = {0};
int	curr_pic_mask;

int	iunit[NINTR] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
			16, 17, 18, 19, 20, 21, 22, 23, 24};

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

int intpri[NINTR] = {
	/* 00 */   	0,	SPL6,	0,	0,
	/* 04 */	0,	0,	0,	0,
	/* 08 */	0,	0,	0,	0,
	/* 12 */	0,	SPL1,	0,	0,
	/* 16 */	0,	0,	0,	0,
	/* 20 */	0,	0,	0,	0,
	/* 255 */	0,
};

void
picdisable(void)
{
	asm("cli");

	curr_ipl = SPLHI;
	curr_pic_mask = pic_mask[SPLHI];

	outb(0xa1, 0xff);
	outb(0x21, 0xff);

	/* Disable PIC */
	outb(IMCR_SELECT, MODE_IMCR);	/* select IMCR */
	outb(IMCR_DATA, IMCR_USE_APIC);	/* force NMI and INTR through APIC */
}

void
form_pic_mask(void)
{
    /* empty */
}

void
intnull(int unit_dev)
{
    printf("intnull(%d)\n", unit_dev);
}

int prtnull_count = 0;

void
prtnull(int unit)
{
	++prtnull_count;
}

static uint32_t
ioapic_read(uint8_t apic, uint8_t reg)
{
    ioapic->select.r = reg;
    return ioapic->window.r;
}

static void
ioapic_write(uint8_t apic, uint8_t reg, uint32_t value)
{
    ioapic->select.r = reg;
    ioapic->window.r = value;
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
global_enable_apic(void)
{
	uint32_t val = 0;
	uint32_t msr = 0x1b;

	__asm__ __volatile__("rdmsr"
			    : "=A" (val)
			    : "c" (msr));
        val |= (1 << 11);

	__asm__ __volatile__("wrmsr"
			    : /* no Outputs */
			    : "c" (msr), "A" (val));
}

static uint32_t
pit_measure_apic_hz(void)
{
    uint32_t start = 0xffffffff;

    /* Prepare accurate delay for 10ms */
    pit_prepare_sleep(10000);

    /* Set APIC timer */
    lapic->init_count.r = start;

    /* zZz */
    pit_sleep();

    /* Stop APIC timer */
    lapic->lvt_timer.r = LAPIC_DISABLE;

    return start - lapic->cur_count.r;
}

static void
lapic_enable_timer_ioapic(void)
{
    uint32_t magic_timer_value_hz = 0;

    lapic->dest_format.r = 0xffffffff;
    lapic->logical_dest.r = (lapic->logical_dest.r & 0xffffff) | 0x1;
    lapic->lvt_timer.r = LAPIC_DISABLE;
    lapic->lvt_performance_monitor.r = LAPIC_NMI;
    lapic->lvt_lint0.r = LAPIC_DISABLE;
    lapic->lvt_lint1.r = LAPIC_DISABLE;
    lapic->task_pri.r = 0;

    global_enable_apic();

    /* Enable IOAPIC interrupts and spurious interrupt */
    lapic->spurious_vector.r |= LAPIC_ENABLE | IOAPIC_SPURIOUS_BASE;

    /* Set one-shot timer */
    lapic->lvt_timer.r = IOAPIC_INT_BASE;
    lapic->divider_config.r = LAPIC_TIMER_DIVIDE_16;

    /* Measure number of APIC timer ticks in 10ms */
    lapic->init_count.r = pit_measure_apic_hz();

    /* Set the timer to interrupt periodically */
    lapic->lvt_timer.r = IOAPIC_INT_BASE | LAPIC_TIMER_PERIODIC;

    /* Some buggy hardware requires this set again */
    lapic->divider_config.r = LAPIC_TIMER_DIVIDE_16;
}

void
ioapic_toggle(int pin, int mask)
{
    int apic = 0;

    if (pin < IOAPIC_NINTR) {
        ioapic_toggle_entry(apic, pin, mask);
    }
}

void
lapic_eoi(void)
{
    lapic->eoi.r = 0;
}

#ifndef LINUX_DEV
void
enable_irq(unsigned int irq)
{
    asm("cli");
    ioapic_toggle(irq, IOAPIC_MASK_ENABLED);
}

void
disable_irq(unsigned int irq)
{
    asm("cli");
    ioapic_toggle(irq, IOAPIC_MASK_DISABLED);
}
#endif

void
ioapic_configure(void)
{
    /* Assume first IO APIC maps to GSI base 0 */
    int i, pin, apic = 0;

    union ioapic_route_entry_union entry = {{0, 0}};

    entry.both.delvmode = IOAPIC_FIXED;
    entry.both.destmode = IOAPIC_PHYSICAL;
    entry.both.mask = IOAPIC_MASK_ENABLED;
    entry.both.dest = ioapics[apic].apic_id & 0xf;

    /* ISA legacy IRQs */
    entry.both.polarity = IOAPIC_ACTIVE_HIGH;
    entry.both.trigger = IOAPIC_EDGE_TRIGGERED;

    for (pin = 0; pin < 16; pin++) {
        entry.both.vector = IOAPIC_INT_BASE + pin;
	ioapic_write_entry(apic, pin, entry.both);
    }

    /* PCI IRQs PIRQ A-H */
    entry.both.polarity = IOAPIC_ACTIVE_LOW;
    entry.both.trigger = IOAPIC_LEVEL_TRIGGERED;

    for (pin = 16; pin < 24; pin++) {
        entry.both.vector = IOAPIC_INT_BASE + pin;
	ioapic_write_entry(apic, pin, entry.both);
    }

    lapic_enable_timer_ioapic();
}
