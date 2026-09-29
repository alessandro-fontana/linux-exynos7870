// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos7870 PMU (Power Management Unit) support
 *
 * The EL3 firmware of this SoC implements the PSCI CPU_SUSPEND power-down
 * state as a WFI that waits for the PMU to cut the core's power, but it
 * leaves the PMU alone: unless the kernel has cleared the core's local power
 * configuration beforehand, the core waits in that WFI forever. The vendor
 * kernel does it in its own cpuidle driver; here a CPU PM notifier does it
 * for the generic PSCI cpuidle driver. When the core wakes up, the PMU sets
 * the configuration back by itself; it is restored here too in case the
 * firmware returned without powering the core down (an interrupt already
 * pending), because a core left with a cleared configuration would lose its
 * power on the next plain WFI.
 *
 * CPU_OFF (hotplug) needs none of this: the firmware programs the PMU itself.
 *
 * Copyright (C) 2026 The LineageOS Project
 */

#include <linux/bits.h>
#include <linux/cpu_pm.h>
#include <linux/notifier.h>
#include <linux/soc/samsung/exynos-pmu.h>

#include <asm/cputype.h>

#include "exynos-pmu.h"

#define EXYNOS7870_CPU_CONFIGURATION(mpidr)				\
	(0x2000 + ((MPIDR_AFFINITY_LEVEL(mpidr, 1) << 2 |		\
		    MPIDR_AFFINITY_LEVEL(mpidr, 0)) * 0x80))
#define EXYNOS7870_CPU_LOCAL_PWR_CFG	GENMASK(3, 0)

static int exynos7870_cpu_pm_notify(struct notifier_block *nb,
				    unsigned long action, void *data)
{
	u32 offset = EXYNOS7870_CPU_CONFIGURATION(read_cpuid_mpidr());
	u32 val = pmu_raw_readl(offset);

	switch (action) {
	case CPU_PM_ENTER:
		pmu_raw_writel(val & ~EXYNOS7870_CPU_LOCAL_PWR_CFG, offset);
		break;
	case CPU_PM_ENTER_FAILED:
	case CPU_PM_EXIT:
		if ((val & EXYNOS7870_CPU_LOCAL_PWR_CFG) !=
		    EXYNOS7870_CPU_LOCAL_PWR_CFG)
			pmu_raw_writel(val | EXYNOS7870_CPU_LOCAL_PWR_CFG,
				       offset);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos7870_cpu_pm_nb = {
	.notifier_call = exynos7870_cpu_pm_notify,
};

static void exynos7870_pmu_init(void)
{
	cpu_pm_register_notifier(&exynos7870_cpu_pm_nb);
}

const struct exynos_pmu_data exynos7870_pmu_data = {
	.pmu_init = exynos7870_pmu_init,
};
