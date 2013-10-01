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
#define SCP_FAULTADDR_OFFSET    0xD38

/* SCB register addresses ***********************************************************/

#define STLR_SCB_INTCTRL          (STLR_CM3P_BASE + SCB_INTCTRL_OFFSET)
#define STLR_SCB_SYSCTRL          (STLR_CM3P_BASE + SCB_SYSCTRL_OFFSET)
#define STLR_SCB_FAULTSTAT        (STLR_CM3P_BASE + SCP_FAULTSTAT_OFFSET)
#define STLR_SCB_HFAULTSTAT       (STLR_CM3P_BASE + SCP_HFAULTSTAT_OFFSET)
#define STLR_SCB_MMADDR           (STLR_CM3P_BASE + SCP_MMADDR_OFFSET)
#define STLR_SCB_FAULTADDR        (STLR_CM3P_BASE + SCP_FAULTADDR_OFFSET)

/* SCB register bit defitiions ******************************************************/

/* Interrupt Control and State (INTCTRL), offset 0xD04 */

#define SCB_INTCTRL_VECPEND_SHIFT        12    /* Bits 18-12: Interrupt Pending Vector Number */
#define SCB_INTCTRL_VECPEND_MASK         (0x7F << SCB_INTCTRL_VECPEND_SHIFT)
#define SCB_INTCTRL_VECACT_SHIFT         0     /* Bits 6-0: Interrupt Pending Vector Number */
#define SCB_INTCTRL_VECACT_MASK          (0x7F << SCB_INTCTRL_VECACT_SHIFT)

/* System Control (SYSCTRL), offset 0xD10 */

#define SCB_SYSCTRL_SLEEPDEEP_SHIFT      2    /* Bits      2: Deep Sleep Enable */
#define SCB_SYSCTRL_SLEEPDEEP_MASK       (0x01 << SCB_SYSCTRL_SLEEPDEEP_SHIFT)

/* Configurable Fault Status (FAULTSTAT), offset 0xD28 */

#define SCB_FAULTSTAT_IERR               1 << 0
#define SCB_FAULTSTAT_IERR_TEXT          "Instruction Access Violation"
#define SCB_FAULTSTAT_DERR               1 << 1
#define SCB_FAULTSTAT_DERR_TEXT          "Data Access Violation"
#define SCB_FAULTSTAT_MUSTKE             1 << 3
#define SCB_FAULTSTAT_MUSTKE_TEXT        "Unstack Access Violation"
#define SCB_FAULTSTAT_MSTKE              1 << 4
#define SCB_FAULTSTAT_MSTKE_TEXT         "Stack Access Violation"
#define SCB_FAULTSTAT_MLSPERR            1 << 5
#define SCB_FAULTSTAT_MLSPERR_TEXT       "Memory Management Fault on Floating-Point Lazy State Preservation"
#define SCB_FAULTSTAT_MMARV              1 << 7
#define SCB_FAULTSTAT_MMARV_TEXT         "Memory Management Fault Address Register Valid"
#define SCB_FAULTSTAT_IBUS               1 << 8
#define SCB_FAULTSTAT_IBUS_TEXT          "Instruction Bus Error"
#define SCB_FAULTSTAT_PRECISE            1 << 9
#define SCB_FAULTSTAT_PRECISE_TEXT       "Precise Data Bus Error"
#define SCB_FAULTSTAT_IMPRE              1 << 10
#define SCB_FAULTSTAT_IMPRE_TEXT         "Imprecise Data Bus Error"
#define SCB_FAULTSTAT_BUSTKE             1 << 11
#define SCB_FAULTSTAT_BUSTKE_TEXT        "Unstack Bus Fault"
#define SCB_FAULTSTAT_BSTKE              1 << 12
#define SCB_FAULTSTAT_BSTKE_TEXT         "Stack Bus Fault"
#define SCB_FAULTSTAT_BLSPERR            1 << 13
#define SCB_FAULTSTAT_BLSPERR_TEXT       "Bus Fault on Floating-Point Lazy State Preservation"
#define SCB_FAULTSTAT_BFARV              1 << 15
#define SCB_FAULTSTAT_BFARV_TEXT         "Bus Fault Address Register Valid"
#define SCB_FAULTSTAT_UNDEF              1 << 16
#define SCB_FAULTSTAT_UNDEF_TEXT         "Undefined Instruction Usage Fault"
#define SCB_FAULTSTAT_INVSTAT            1 << 17
#define SCB_FAULTSTAT_INVSTAT_TEXT       "Invalid State Usage Fault"
#define SCB_FAULTSTAT_INVPC              1 << 18
#define SCB_FAULTSTAT_INVPC_TEXT         "Invalid PC Load Usage Fault"
#define SCB_FAULTSTAT_NOCP               1 << 19
#define SCB_FAULTSTAT_NOCP_TEXT          "No Coprocessor Usage Fault"
#define SCB_FAULTSTAT_UNALIGN            1 << 24
#define SCB_FAULTSTAT_UNALIGN_TEXT       "Unaligned Access Usage Fault"
#define SCB_FAULTSTAT_DIV0               1 << 25
#define SCB_FAULTSTAT_DIV0_TEXT          "Divide-by-Zero Usage Fault"

#define SCB_FAULTSTAT_MASK               (SCB_FAULTSTAT_DIV0   | SCB_FAULTSTAT_UNALIGN | SCB_FAULTSTAT_NOCP    | \
                                          SCB_FAULTSTAT_INVPC  | SCB_FAULTSTAT_INVSTAT | SCB_FAULTSTAT_UNDEF   | \
                                          SCB_FAULTSTAT_BFARV  | SCB_FAULTSTAT_BLSPERR | SCB_FAULTSTAT_BSTKE   | \
                                          SCB_FAULTSTAT_BUSTKE | SCB_FAULTSTAT_IMPRE   | SCB_FAULTSTAT_PRECISE | \
                                          SCB_FAULTSTAT_IBUS   | SCB_FAULTSTAT_MMARV   | SCB_FAULTSTAT_MLSPERR | \
                                          SCB_FAULTSTAT_MSTKE  | SCB_FAULTSTAT_MUSTKE  | SCB_FAULTSTAT_DERR    | \
                                          SCB_FAULTSTAT_IERR)

/* Hard Fault Status (HFAULTSTAT), offset 0xD2C */

#define SCB_HFAULTSTAT_VECT              1 << 1
#define SCB_HFAULTSTAT_VECT_TEXT         "Vector Table Read Fault"
#define SCB_HFAULTSTAT_FORCED            1 << 30
#define SCB_HFAULTSTAT_FORCED_TEXT       "Forced Hard Fault"
#define SCB_HFAULTSTAT_DBG               1 << 31
#define SCB_HFAULTSTAT_DBG_TEXT          "Debug Event"

#define SCB_HFAULTSTAT_MASK              (SCB_HFAULTSTAT_VECT | SCB_HFAULTSTAT_FORCED | SCB_HFAULTSTAT_DBG)
