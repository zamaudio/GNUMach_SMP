/* 
 * Mach Operating System
 * Copyright (c) 1991,1990 Carnegie Mellon University
 * All Rights Reserved.
 * 
 * Permission to use, copy, modify and distribute this software and its
 * documentation is hereby granted, provided that both the copyright
 * notice and this permission notice appear in all copies of the
 * software, derivative works or modified versions, and any portions
 * thereof, and that both notices appear in supporting documentation.
 * 
 * CARNEGIE MELLON ALLOWS FREE USE OF THIS SOFTWARE IN ITS "AS IS"
 * CONDITION.  CARNEGIE MELLON DISCLAIMS ANY LIABILITY OF ANY KIND FOR
 * ANY DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE.
 * 
 * Carnegie Mellon requests users of this software to return to
 * 
 *  Software Distribution Coordinator  or  Software.Distribution@CS.CMU.EDU
 *  School of Computer Science
 *  Carnegie Mellon University
 *  Pittsburgh PA 15213-3890
 * 
 * any improvements or extensions that they make and grant Carnegie Mellon
 * the rights to redistribute these changes.
 */

#if	NCPUS > 1

#include <string.h> 

#include <kern/cpu_number.h>
#include <kern/debug.h>
#include <kern/startup.h>
#include <mach/machine.h>
#include <mach/xen.h>
#include <vm/vm_kern.h>

#include <i386/mp_desc.h>
#include <i386/lock.h>
#include <machine/ktss.h>
#include <machine/tss.h>
#include <machine/io_perm.h>
#include <machine/vm_param.h>

/*
 * The i386 needs an interrupt stack to keep the PCB stack from being
 * overrun by interrupts.  All interrupt stacks MUST lie at lower addresses
 * than any thread`s kernel stack.
 */

/*
 * Addresses of bottom and top of interrupt stacks.
 */
vm_offset_t	interrupt_stack[NCPUS];
vm_offset_t	int_stack_top[NCPUS];

/*
 * Barrier address.
 */
vm_offset_t	int_stack_high;

/*
 * First cpu`s interrupt stack.
 */
char		intstack[];	/* bottom */
char		eintstack[];	/* top */

/*
 * Multiprocessor i386/i486 systems use a separate copy of the
 * GDT, IDT, LDT, and kernel TSS per processor.  The first three
 * are separate to avoid lock contention: the i386 uses locked
 * memory cycles to access the descriptor tables.  The TSS is
 * separate since each processor needs its own kernel stack,
 * and since using a TSS marks it busy.
 */

/*
 * Allocated descriptor tables.
 */
struct mp_desc_table	*mp_desc_table[NCPUS] = { 0 };

/*
 * Pointer to TSS for access in load_context.
 */
struct task_tss		*mp_ktss[NCPUS] = { 0 };

/*
 * Pointer to GDT to reset the KTSS busy bit.
 */
struct real_descriptor	*mp_gdt[NCPUS] = { 0 };

/*
 * Boot-time tables, for initialization and master processor.
 */
extern struct real_gate		idt[IDTSZ];
extern struct real_descriptor	gdt[GDTSZ];
extern struct real_descriptor	ldt[LDTSZ];

/*
 * Allocate and initialize the per-processor descriptor tables.
 */

struct mp_desc_table *
mp_desc_init(int mycpu)
{
	struct mp_desc_table *mpt;

	if (mycpu == master_cpu) {
		/*
		 * Master CPU uses the tables built at boot time.
		 * Just set the TSS and GDT pointers.
		 */
		mp_ktss[mycpu] = (struct task_tss *) &ktss;
		mp_gdt[mycpu] = gdt;
		return 0;
	}
	else {
		/*
		 * Other CPUs allocate the table from the bottom of
		 * the interrupt stack.
		 */
		mpt = (struct mp_desc_table *) interrupt_stack[mycpu];

		mp_desc_table[mycpu] = mpt;
		mp_ktss[mycpu] = &mpt->ktss;
		mp_gdt[mycpu] = mpt->gdt;

		/*
		 * Copy the tables
		 */
		memcpy(mpt->idt,
		  idt,
		  sizeof(idt));
		memcpy(mpt->gdt,
		  gdt,
		  sizeof(gdt));
		memcpy(mpt->ldt,
		  ldt,
		  sizeof(ldt));
		memset(&mpt->ktss, 0, 
		  sizeof(struct task_tss));

		/*
		 * Fix up the entries in the GDT to point to
		 * this LDT and this TSS.
		 */
#ifdef	MACH_RING1
		panic("TODO %s:%d\n",__FILE__,__LINE__);
#else	/* MACH_RING1 */
		_fill_gdt_sys_descriptor(mpt->gdt, KERNEL_LDT,
			(unsigned)&mpt->ldt,
			LDTSZ * sizeof(struct real_descriptor) - 1,
			ACC_P|ACC_PL_K|ACC_LDT, 0);
		_fill_gdt_sys_descriptor(mpt->gdt, KERNEL_TSS,
			(unsigned)&mpt->ktss,
			sizeof(struct task_tss) - 1,
			ACC_P|ACC_PL_K|ACC_TSS, 0);

		mpt->ktss.tss.ss0 = KERNEL_DS;
		mpt->ktss.tss.io_bit_map_offset = IOPB_INVAL;
		mpt->ktss.barrier = 0xFF;
#endif	/* MACH_RING1 */

		return mpt;
	}
}

static void send_ipi(unsigned icr_h, unsigned icr_l)
{
    lapic->icr_high.r = icr_h;
    lapic->icr_low.r = icr_l;
}


void startup_cpu(uint32_t apic_id)
{
    unsigned icr_h = 0;
    unsigned icr_l = 0;

    //send INIT Assert IPI
    icr_h = (apic_id << 24);
    icr_l = (INIT << 8) | (ASSERT << 14) | (LEVEL << 15);
    send_ipi(icr_h, icr_l);

    //wait until IPI is sent
    delay(10000);
    while( ( (lapic->icr_low.r >> 12) & 1) == SEND_PENDING);

    //Send INIT De-Assert IPI
    icr_h = 0;
    icr_l = 0;
    icr_h = (apic_id << 24);
    icr_l = (INIT << 8) | (DE_ASSERT << 14) | (LEVEL << 15);
    send_ipi(icr_h, icr_l);

    //wait until IPI is sent
    delay(10000);
    while( ( (lapic->icr_low.r >> 12) & 1) == SEND_PENDING);

    //Send StartUp IPI
    icr_h = 0;
    icr_l = 0;
    icr_h = (apic_id << 24);
    icr_l = (STARTUP << 8) | ((AP_BOOT_ADDR >>12) & 0xff);
    send_ipi(icr_h, icr_l);

    //wait until IPI is sent
    delay(1000);
    while( ( (lapic->icr_low.r >> 12) & 1) == SEND_PENDING);

    //Send second StartUp IPI
    icr_h = 0;
    icr_l = 0;
    icr_h = (apic_id << 24);
    icr_l = (STARTUP << 8) | ((AP_BOOT_ADDR >>12) & 0xff);
    send_ipi(icr_h, icr_l);

    //wait until IPI is sent
    delay(1000);
    while( ( (lapic->icr_low.r >> 12) & 1) == SEND_PENDING);

}

int
cpu_setup()
{

    int i = 1;
    int kernel_id = 0;

    kmutex_init(&ap_config_lock);
    kmutex_lock(&ap_config_lock, FALSE);

    while(i < ncpu && (machine_slot[i].running == TRUE)) i++;

    /* assume Pentium 4, Xeon, or later processors */
    //unsigned apic_id = (((ApicLocalUnit*)phystokv(lapic_addr))->apic_id.r >> 24) & 0xff;
    unsigned apic_id = lapic->apic_id.r;

    /* panic? */
    if(i >= ncpu)
        return -1;

    /*TODO: Move this code to a separate function*/



    /* Update apic2kernel and machine_slot with the newest apic_id */
    if(apic2kernel[machine_slot[i].apic_id] == i)
        {
            apic2kernel[machine_slot[i].apic_id] = -1;
        }

    apic2kernel[apic_id] = i;
    machine_slot[i].apic_id =  apic_id;

    /* Initialize machine_slot fields with the cpu data */
    machine_slot[i].running = TRUE;
    machine_slot[i].cpu_subtype = CPU_SUBTYPE_AT386;

    int cpu_type = discover_x86_cpu_type ();

    switch (cpu_type)
        {
        default:
            printf("warning: unknown cpu type %d, assuming i386\n", cpu_type);

        case 3:
            machine_slot[i].cpu_type = CPU_TYPE_I386;
            break;

        case 4:
            machine_slot[i].cpu_type = CPU_TYPE_I486;
            break;

        case 5:
            machine_slot[i].cpu_type = CPU_TYPE_PENTIUM;
            break;
        case 6:
        case 15:
            machine_slot[i].cpu_type = CPU_TYPE_PENTIUMPRO;
            break;
        }

    /*
     * Initialize and activate the real i386 protected-mode structures.
     */
    gdt_init();
    idt_init();
    ldt_init();
    ktss_init();

    /* Add cpu to the kernel */
    slave_main();

    kmutex_unlock(&ap_config_lock);

    return 0;
}

void paging_setup(){

#if PAE
    set_cr3(pdpbase_addr);
#ifndef	MACH_HYP
    if (!CPU_HAS_FEATURE(CPU_FEATURE_PAE))
        set_cr4(get_cr4() | CR4_PAE);
#endif	/* MACH_HYP */
#else
    set_cr3(kernel_page_dir_addr);
#endif	/* PAE */
#ifndef	MACH_HYP
    /* Turn paging on.
     * Also set the WP bit so that on 486 or better processors
     * page-level write protection works in kernel mode.
     */
    set_cr0(get_cr0() | CR0_PG | CR0_WP);
    set_cr0(get_cr0() & ~(CR0_CD | CR0_NW));

    if (CPU_HAS_FEATURE(CPU_FEATURE_PGE))
        set_cr4(get_cr4() | CR4_PGE);

#endif	/* MACH_HYP */

    flush_instr_queue();
    flush_tlb();

}

void
cpu_ap_main(void)
{
    cpu_setup();
}


/*
 * Called after all CPUs have been found, but before the VM system
 * is running.  The machine array must show which CPUs exist.
 */
void
interrupt_stack_alloc(void)
{
	int		i;
	int		cpu_count;
	vm_offset_t	stack_start;

	/*
	 * Count the number of CPUs.
	 */
	cpu_count = 0;
	for (i = 0; i < NCPUS; i++)
	    if (machine_slot[i].is_cpu)
		cpu_count++;

	/*
	 * Allocate an interrupt stack for each CPU except for
	 * the master CPU (which uses the bootstrap stack)
	 */
	if (!init_alloc_aligned(INTSTACK_SIZE*(cpu_count-1), &stack_start))
		panic("not enough memory for interrupt stacks");
	stack_start = phystokv(stack_start);

	/*
	 * Set up pointers to the top of the interrupt stack.
	 */
	for (i = 0; i < NCPUS; i++) {
	    if (i == master_cpu) {
		interrupt_stack[i] = (vm_offset_t) intstack;
		int_stack_top[i]   = (vm_offset_t) eintstack;
	    }
	    else if (machine_slot[i].is_cpu) {
		interrupt_stack[i] = stack_start;
		int_stack_top[i]   = stack_start + INTSTACK_SIZE;

		stack_start += INTSTACK_SIZE;
	    }
	}

	/*
	 * Set up the barrier address.  All thread stacks MUST
	 * be above this address.
	 */
	int_stack_high = stack_start;
}

/* XXX should be adjusted per CPU speed */
int simple_lock_pause_loop = 100;

unsigned int simple_lock_pause_count = 0;	/* debugging */

void
simple_lock_pause(void)
{
	static volatile int dummy;
	int i;

	simple_lock_pause_count++;

	/*
	 * Used in loops that are trying to acquire locks out-of-order.
	 */

	for (i = 0; i < simple_lock_pause_loop; i++)
	    dummy++;	/* keep the compiler from optimizing the loop away */
}

kern_return_t
cpu_control(int cpu, const int *info, unsigned int count)
{
	printf("cpu_control(%d, %p, %d) not implemented\n",
	       cpu, info, count);
	return KERN_FAILURE;
}

void
interrupt_processor(int cpu)
{
	printf("interrupt cpu %d\n",cpu);
}

kern_return_t
cpu_start(int cpu)
{
	if (machine_slot[cpu].running)
		return KERN_FAILURE;

	return intel_startCPU(cpu);
}

void
start_other_cpus(void)
{
	int cpu;
	for (cpu = 0; cpu < NCPUS; cpu++)
		if (cpu != cpu_number())
			cpu_start(cpu);
}

#endif	/* NCPUS > 1 */
