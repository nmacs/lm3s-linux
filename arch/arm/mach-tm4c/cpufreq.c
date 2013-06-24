/* linux/arch/arm/mach-tm4c/cpufreq.c
 *
 * Copyright 2013 Max Nekludov <Max.Nekludov@us.elster.com>
 *
 * TM4C CPUfreq Support
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>

static struct clk *sysclk;

static struct cpufreq_frequency_table tm4c_freq_table[] = {
#if 0
	{ 0, 1000 },
 	{ 1, 2000 },
 	{ 2, 4000 },
 	{ 3, 8000 },
#endif
#if 1
	{ 4, 16000 },
	{ 5, 20000 },
	{ 6, 24000 },
#endif
	{ 7, 30000 },
	{ 8, 32000 },
	{ 9, 40000 },
	{ 10, 48000 },
	{ 11, 60000 },
	{ 0, CPUFREQ_TABLE_END },
};

static int tm4c_cpufreq_verify_speed(struct cpufreq_policy *policy)
{
	if (policy->cpu != 0)
		return -EINVAL;

	return cpufreq_frequency_table_verify(policy, tm4c_freq_table);
}

static unsigned int tm4c_cpufreq_get_speed(unsigned int cpu)
{
	if (cpu != 0)
		return 0;

	return clk_get_rate(sysclk) / 1000;
}

static int tm4c_cpufreq_set_target(struct cpufreq_policy *policy,
				      unsigned int target_freq,
				      unsigned int relation)
{
	int ret;
	unsigned int i;
	struct cpufreq_freqs freqs;

	ret = cpufreq_frequency_table_target(policy, tm4c_freq_table,
					     target_freq, relation, &i);
	if (ret != 0)
		return ret;

	freqs.cpu = 0;
	freqs.old = clk_get_rate(sysclk) / 1000;
	freqs.new = tm4c_freq_table[i].frequency;
	freqs.flags = 0;

	if (freqs.old == freqs.new)
		return 0;

	cpufreq_notify_transition(&freqs, CPUFREQ_PRECHANGE);

	ret = clk_set_rate(sysclk, freqs.new * 1000);
	if (ret < 0) {
		pr_err("cpufreq: Failed to set rate %dkHz: %d\n",
		       freqs.new, ret);
		goto err;
	}

	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	pr_debug("cpufreq: Set actual frequency %lukHz\n",
		 clk_get_rate(sysclk) / 1000);

	return 0;

err:
	cpufreq_notify_transition(&freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

static int __init tm4c_cpufreq_driver_init(struct cpufreq_policy *policy)
{
	int ret;
	struct cpufreq_frequency_table *freq;

	if (policy->cpu != 0)
		return -EINVAL;

	sysclk = clk_get(NULL, "sys");
	if (IS_ERR(sysclk)) {
		pr_err("cpufreq: Unable to obtain SYSCLK: %ld\n",
		       PTR_ERR(sysclk));
		return PTR_ERR(sysclk);
	}

	freq = tm4c_freq_table;
	while (freq->frequency != CPUFREQ_TABLE_END) {
		unsigned long r;

		/* Check for frequencies we can generate */
		r = clk_round_rate(sysclk, freq->frequency * 1000);
		r /= 1000;
		if (r != freq->frequency) {
			pr_debug("cpufreq: %dkHz unsupported by clock\n",
				 freq->frequency);
			freq->frequency = CPUFREQ_ENTRY_INVALID;
		}
		freq++;
	}

	policy->cur = clk_get_rate(sysclk) / 1000;

	policy->cpuinfo.transition_latency = (500 * 1000);

	ret = cpufreq_frequency_table_cpuinfo(policy, tm4c_freq_table);
	if (ret != 0) {
		pr_err("cpufreq: Failed to configure frequency table: %d\n",
		       ret);
		clk_put(sysclk);
	}

	return ret;
}

static struct cpufreq_driver tm4c_cpufreq_driver = {
	.owner		= THIS_MODULE,
	.flags          = 0,
	.verify		= tm4c_cpufreq_verify_speed,
	.target		= tm4c_cpufreq_set_target,
	.get		= tm4c_cpufreq_get_speed,
	.init		= tm4c_cpufreq_driver_init,
	.name		= "tm4c",
};

static int __init tm4c_cpufreq_init(void)
{
	return cpufreq_register_driver(&tm4c_cpufreq_driver);
}
module_init(tm4c_cpufreq_init);
