/****************************************************************************
 *
 *   Copyright (C) 2013 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <Max.Nekludov@us.elster.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <mach/hardware.h>

#ifdef CONFIG_ARCH_TM4C
static const uint32_t g_rcgcgpiomask[STLR_NPORTS] =
{
	1 << 0, 1 << 1,  1 << 2,  1 << 3,
	1 << 4, 1 << 5,  1 << 6,  1 << 7,
	1 << 9, 1 << 10, 1 << 11, 1 << 13
};

static inline uint32_t rcgcgpiomask(unsigned int port)
{
	uint32_t mask = 0;
	if (port < STLR_NPORTS)
	{
		mask = g_rcgcgpiomask[port];
	}
	return mask;
}
#endif

void gpio_clock_ctrl(int port, int ctrl)
{
#if defined(CONFIG_ARCH_TM4C)
	uint32_t mask = rcgcgpiomask(port);
	uint32_t regval;
	
	regval = getreg32(STLR_SYSCON_RCGCGPIO);
	if (ctrl)
		regval |= mask;
	else
		regval &= ~mask;
  putreg32(regval, STLR_SYSCON_RCGCGPIO);
	
	if (ctrl)
		while( (getreg32(STLR_SYSCON_PRGPIO) & mask) == 0 ) {}
#elif defined(CONFIG_ARCH_LM3S)
	uint32_t regval;
	
	regval = getreg32(STLR_SYSCON_RCGC2);
	
	regval |= SYSCON_RCGC2_GPIO(port);
	
	/* Set RCGC2 twice(STLR errata) */
	putreg32(regval, STLR_SYSCON_RCGC2);
	putreg32(regval, STLR_SYSCON_RCGC2);
#endif
}

void uart_clock_ctrl(int port, int ctrl)
{
#if defined(CONFIG_ARCH_TM4C)
	uint32_t mask = 1 << port;
	uint32_t regval;
	
	regval = getreg32(STLR_SYSCON_RCGCUART);
	if (ctrl)
		regval |= mask;
	else
		regval &= ~mask;
	putreg32(regval, STLR_SYSCON_RCGCUART);
	
	if (ctrl)
		while( (getreg32(STLR_SYSCON_PRUART) & mask) == 0 ) {}
#endif
}

void dma_clock_ctrl(int ctrl)
{
#ifdef CONFIG_ARCH_TM4C
	if (ctrl)
	{
		putreg32(1, STLR_SYSCON_RCGCDMA);
		while( (getreg32(STLR_SYSCON_PRDMA) & 1) == 0 ) {}
	}
	else
		putreg32(0, STLR_SYSCON_RCGCDMA);
#endif
}

void ssi_clock_ctrl(int module, int ctrl)
{
#ifdef CONFIG_ARCH_TM4C
	uint32_t mask = 1 << module;
	uint32_t regval;
	
	regval = getreg32(STLR_SYSCON_RCGCSSI);
	if (ctrl)
		regval |= mask;
	else
		regval &= ~mask;
	putreg32(regval, STLR_SYSCON_RCGCSSI);
	
	if (ctrl)
		while( (getreg32(STLR_SYSCON_PRSSI) & mask) == 0 ) {}
#endif
}

void epi_clock_ctrl(int ctrl)
{
#ifdef CONFIG_ARCH_TM4C
	if (ctrl)
	{
		putreg32(1, STLR_SYSCON_RCGCEPI);
		while( (getreg32(STLR_SYSCON_PREPI) & 1) == 0 ) {}
	}
	else
		putreg32(0, STLR_SYSCON_RCGCEPI);
#endif
}

void timer_clock_ctrl(int module, int ctrl)
{
#ifdef CONFIG_ARCH_TM4C
	uint32_t mask = 1 << module;
	uint32_t regval;
	
	regval = getreg32(STLR_SYSCON_RCGCTIMER);
	if (ctrl)
		regval |= mask;
	else
		regval &= ~mask;
	putreg32(regval, STLR_SYSCON_RCGCTIMER);
	
	if (ctrl)
		while( (getreg32(STLR_SYSCON_PRTIMER) & mask) == 0 ) {}
#endif
}

void watchdog_clock_ctrl(int module, int ctrl)
{
#ifdef CONFIG_ARCH_TM4C
	uint32_t mask = 1 << module;
	uint32_t regval;
	
	regval = getreg32(STLR_SYSCON_RCGCWD);
	if (ctrl)
		regval |= mask;
	else
		regval &= ~mask;
	putreg32(regval, STLR_SYSCON_RCGCWD);
	
	if (ctrl)
		while( (getreg32(STLR_SYSCON_PRWD) & mask) == 0 ) {}
#endif
}

void adc_clock_ctrl(int module, int ctrl)
{
#ifdef CONFIG_ARCH_TM4C
	uint32_t mask = 1 << module;
	uint32_t regval;
	
	regval = getreg32(STLR_SYSCON_RCGCADC);
	if (ctrl)
		regval |= mask;
	else
		regval &= ~mask;
	putreg32(regval, STLR_SYSCON_RCGCADC);
	
	if (ctrl)
		while( (getreg32(STLR_SYSCON_PRADC) & mask) == 0 ) {}
#endif
}