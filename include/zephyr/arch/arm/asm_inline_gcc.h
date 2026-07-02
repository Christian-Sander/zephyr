/* ARM AArch32 GCC specific public inline assembler functions and macros */

/*
 * Copyright (c) 2015, Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Either public functions or macros or invoked by public functions */

#ifndef ZEPHYR_INCLUDE_ARCH_ARM_ASM_INLINE_GCC_H_
#define ZEPHYR_INCLUDE_ARCH_ARM_ASM_INLINE_GCC_H_

/*
 * The file must not be included directly
 * Include arch/cpu.h instead
 */

#ifndef _ASMLANGUAGE

#include <zephyr/toolchain.h>
#include <zephyr/types.h>
#include <zephyr/arch/exception.h>
#include <cmsis_core.h>

#if defined(CONFIG_CPU_AARCH32_CORTEX_R) || defined(CONFIG_CPU_AARCH32_CORTEX_A)
#include <zephyr/arch/arm/cortex_a_r/cpu.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* On ARMv7-M and ARMv8-M Mainline CPUs, this function prevents regular
 * exceptions (i.e. with interrupt priority lower than or equal to
 * _EXC_IRQ_DEFAULT_PRIO) from interrupting the CPU. NMI, Faults, SVC,
 * and Zero Latency IRQs (if supported) may still interrupt the CPU.
 *
 * On ARMv6-M with CONFIG_ZERO_LATENCY_IRQS_ARMV6_M, this function masks
 * regular IRQs in the NVIC and defers SysTick while leaving zero-latency IRQs
 * enabled. NMI, Faults, SVC, PendSV and Zero Latency IRQs may still interrupt
 * the CPU.
 *
 * On ARMv6-M and ARMv8-M Baseline CPUs, this function reads the value of
 * PRIMASK which shows if interrupts are enabled, then disables all interrupts
 * except NMI.
 */

static ALWAYS_INLINE unsigned int arch_irq_lock(void)
{
	unsigned int key;

#if defined(CONFIG_ZERO_LATENCY_IRQS_ARMV6_M)
	unsigned int primask = __get_PRIMASK();
	bool zli_locked;

	__disable_irq();
	zli_locked = z_armv6m_zli_locked() || (z_armv6m_zli_get_shadow_reg() != 0U);
	key = ((primask != 0U) || zli_locked) ? 1U : 0U;

	/*
	 * If PRIMASK was already set, this is nested inside a global
	 * interrupt lock such as arch_zli_lock(). Preserve normal ARMv6-M
	 * key semantics and do not mutate the NVIC software mask.
	 */
	if (primask == 0U) {
		if (!zli_locked) {
			z_armv6m_zli_save_systick_state();
		}
		z_armv6m_zli_set_shadow_reg(z_armv6m_zli_get_shadow_reg() |
						 z_armv6m_zli_get_irq_status());
		z_armv6m_zli_set_irq_status(z_armv6m_zli_get_shadow_reg() &
					 z_armv6m_zli_get_mask());
		z_armv6m_zli_set_lock_flag(true);
	}
	__DSB();
	__ISB();
	__set_PRIMASK(primask);
	__ISB();
#elif defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
#if CONFIG_MP_MAX_NUM_CPUS == 1 || defined(CONFIG_ARMV8_M_BASELINE)
	key = __get_PRIMASK();
	__disable_irq();
#else
#error "Cortex-M0 and Cortex-M0+ require SoC specific support for cross core synchronisation."
#endif
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	key = __get_BASEPRI();
	__set_BASEPRI_MAX(_EXC_IRQ_DEFAULT_PRIO);
	__ISB();
#elif defined(CONFIG_ARMV7_R) || defined(CONFIG_AARCH32_ARMV8_R) \
	|| defined(CONFIG_ARMV7_A)
	__asm__ volatile(
		"mrs %0, cpsr;"
		"and %0, #" STRINGIFY(I_BIT) ";"
		"cpsid i;"
		: "=r" (key)
		:
		: "memory", "cc");
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */

	return key;
}


/* On Cortex-M0/M0+, this enables all interrupts if they were not
 * previously disabled.
 */

static ALWAYS_INLINE void arch_irq_unlock(unsigned int key)
{
#if defined(CONFIG_ZERO_LATENCY_IRQS_ARMV6_M)
	if (key == 0U) {
		__disable_irq();
		if (z_armv6m_zli_locked()) {
			z_armv6m_zli_set_irq_status(z_armv6m_zli_get_shadow_reg());
			z_armv6m_zli_set_shadow_reg(0U);
			z_armv6m_zli_set_lock_flag(false);
			z_armv6m_zli_restore_systick_state();
		}
		__DSB();
		__enable_irq();
		__ISB();
	}
#elif defined(CONFIG_ARMV6_M_ARMV8_M_BASELINE)
	if (key != 0U) {
		return;
	}
	__enable_irq();
	__ISB();
#elif defined(CONFIG_ARMV7_M_ARMV8_M_MAINLINE)
	__set_BASEPRI(key);
	__ISB();
#elif defined(CONFIG_ARMV7_R) || defined(CONFIG_AARCH32_ARMV8_R) \
	|| defined(CONFIG_ARMV7_A)
	if (key != 0U) {
		return;
	}
	__enable_irq();
#else
#error Unknown ARM architecture
#endif /* CONFIG_ARMV6_M_ARMV8_M_BASELINE */
}

static ALWAYS_INLINE bool arch_irq_unlocked(unsigned int key)
{
	/* This convention works for both PRIMASK and BASEPRI */
	return key == 0U;
}

#if defined(CONFIG_ZERO_LATENCY_IRQS) || defined(CONFIG_ZERO_LATENCY_IRQS_ARMV6_M)

static ALWAYS_INLINE unsigned int arch_zli_lock(void)
{
	unsigned int key;

	key = __get_PRIMASK();

	/*
	 * The cpsid instruction is self synchronizing within the instruction stream, no need for
	 * an explicit __ISB().
	 */
	__disable_irq();

	return key;
}

static ALWAYS_INLINE void arch_zli_unlock(unsigned int key)
{
	__set_PRIMASK(key);
	__ISB();
}

#endif /* CONFIG_ZERO_LATENCY_IRQS || CONFIG_ZERO_LATENCY_IRQS_ARMV6_M */

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_ARCH_ARM_ASM_INLINE_GCC_H_ */
