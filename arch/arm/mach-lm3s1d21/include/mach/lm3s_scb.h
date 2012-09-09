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

/* SCB register offsets *************************************************************/

#define SCB_INTCTRL_OFFSET      0xD04
#define SCB_SYSCTRL_OFFSET      0xD10
#define SCP_FAULTSTAT_OFFSET    0xD28
#define SCP_HFAULTSTAT_OFFSET   0xD2C
#define SCP_MMADDR_OFFSET       0xD34

/* SCB register addresses ***********************************************************/

#define LM3S_SCB_INTCTRL          (LM3S_CM3P_BASE + SCB_INTCTRL_OFFSET)
#define LM3S_SCB_SYSCTRL          (LM3S_CM3P_BASE + SCB_SYSCTRL_OFFSET)
#define LM3S_SCB_FAULTSTAT        (LM3S_CM3P_BASE + SCP_FAULTSTAT_OFFSET)
#define LM3S_SCB_HFAULTSTAT       (LM3S_CM3P_BASE + SCP_HFAULTSTAT_OFFSET)
#define LM3S_SCB_MMADDR           (LM3S_CM3P_BASE + SCP_MMADDR_OFFSET)

/* SCB register bit defitiions ******************************************************/

/* Interrupt Control and State (INTCTRL), offset 0xD04 */

#define SCB_INTCTRL_VECPEND_SHIFT        12    /* Bits 18-12: Interrupt Pending Vector Number */
#define SCB_INTCTRL_VECPEND_MASK         (0x7F << SCB_INTCTRL_VECPEND_SHIFT)
#define SCB_INTCTRL_VECACT_SHIFT         0     /* Bits 6-0: Interrupt Pending Vector Number */
#define SCB_INTCTRL_VECACT_MASK          (0x7F << SCB_INTCTRL_VECACT_SHIFT)

/* System Control (SYSCTRL), offset 0xD10 */

#define SCB_SYSCTRL_SLEEPDEEP_SHIFT      2    /* Bits      2: Deep Sleep Enable */
#define SCB_SYSCTRL_SLEEPDEEP_MASK       (0x01 << SCB_SYSCTRL_SLEEPDEEP_SHIFT)
