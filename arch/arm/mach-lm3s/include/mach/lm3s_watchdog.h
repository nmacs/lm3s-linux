/************************************************************************************
 *
 *   Copyright (C) 2012 Max Nekludov. All rights reserved.
 *   Author: Max Nekludov <macscomp@gmail.com>
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
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 ************************************************************************************/

/* Watchdog register offsets ********************************************************/

#define WATCHDOG_WDTLOAD_OFFSET     0x000
#define WATCHDOG_WDTVALUE_OFFSET    0x004
#define WATCHDOG_WDTCTL_OFFSET      0x008
#define WATCHDOG_WDTICR_OFFSET      0x00C
#define WATCHDOG_WDTRIS_OFFSET      0x010
#define WATCHDOG_WDTMIS_OFFSET      0x014
#define WATCHDOG_WDTTEST_OFFSET     0x418
#define WATCHDOG_WDTLOCK_OFFSET     0xC00

/* Watchdog register addresses ******************************************************/

#define LM3S_WATCHDOG_WDTLOAD(n)  (LM3S_WDOG_BASE(n) + WATCHDOG_WDTLOAD_OFFSET)
#define LM3S_WATCHDOG_WDTVALUE(n) (LM3S_WDOG_BASE(n) + WATCHDOG_WDTVALUE_OFFSET)
#define LM3S_WATCHDOG_WDTCTL(n)   (LM3S_WDOG_BASE(n) + WATCHDOG_WDTCTL_OFFSET)
#define LM3S_WATCHDOG_WDTICR(n)   (LM3S_WDOG_BASE(n) + WATCHDOG_WDTICR_OFFSET)
#define LM3S_WATCHDOG_WDTRIS(n)   (LM3S_WDOG_BASE(n) + WATCHDOG_WDTRIS_OFFSET)
#define LM3S_WATCHDOG_WDTMIS(n)   (LM3S_WDOG_BASE(n) + WATCHDOG_WDTMIS_OFFSET)
#define LM3S_WATCHDOG_WDTTEST(n)  (LM3S_WDOG_BASE(n) + WATCHDOG_WDTTEST_OFFSET)
#define LM3S_WATCHDOG_WDTLOCK(n)  (LM3S_WDOG_BASE(n) + WATCHDOG_WDTLOCK_OFFSET)

/* Watchdog register bit defitiions *************************************************/

/* Watchdog Control (WDTCTL), offset 0x008 */

#define WATCHDOG_WDTCTL_INTEN_SHIFT      0    /* Bit 0: Watchdog Interrupt Enable */
#define WATCHDOG_WDTCTL_INTEN_MASK       (1 << WATCHDOG_WDTCTL_INTEN_SHIFT)
#define WATCHDOG_WDTCTL_RESEN_SHIFT      1    /* Bit 1: Watchdog Reset Enable */
#define WATCHDOG_WDTCTL_RESEN_MASK       (1 << WATCHDOG_WDTCTL_RESEN_SHIFT)
#define WATCHDOG_WDTCTL_WRC_SHIFT        31   /* Bit 31: Write Complete */
#define WATCHDOG_WDTCTL_WRC_MASK         (1 << WATCHDOG_WDTCTL_WRC_SHIFT)

/* Watchdog Raw Interrupt Status (WDTRIS), offset 0x010 */

#define WATCHDOG_WDTRIS_WDTRIS_SHIFT     0    /* Bit 0: Watchdog Raw Interrupt Status */
#define WATCHDOG_WDTRIS_WDTRIS_MASK      (1 << WATCHDOG_WDTRIS_WDTRIS_SHIFT)

/* Watchdog Test (WDTTEST), offset 0x418 */

#define WATCHDOG_WDTTEST_STALL_SHIFT     8    /* Bit 8: Watchdog Stall Enable */
#define WATCHDOG_WDTTEST_STALL_MASK      (1 << WATCHDOG_WDTTEST_STALL_SHIFT)

/* Watchdog Lock (WDTLOCK), offset 0xC00 */

/*A write of the value 0x1ACC.E551 unlocks the watchdog registers for
 *write access. A write of any other value reapplies the lock, preventing
 *any register updates.
 */

#define WATCHDOG_WDTLOCK_MAGIC           0x1ACCE551