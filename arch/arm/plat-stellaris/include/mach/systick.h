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

#define STLR_SYSTICK_CTRL_OFFSET          0x010
#define STLR_SYSTICK_RELOAD_OFFSET        0x014
#define STLR_SYSTICK_CURRENT_OFFSET       0x018

#define STLR_SYSTICK_CTRL                 (STLR_SYSTICK_BASE + STLR_SYSTICK_CTRL_OFFSET)
#define STLR_SYSTICK_RELOAD               (STLR_SYSTICK_BASE + STLR_SYSTICK_RELOAD_OFFSET)
#define STLR_SYSTICK_CURRENT              (STLR_SYSTICK_BASE + STLR_SYSTICK_CURRENT_OFFSET)

#define STLR_SYSTICK_CTRL_ENABLE                (1 << 0)
#define STLR_SYSTICK_CTRL_INTEN                 (1 << 1)
#define STLR_SYSTICK_CTRL_CLK_SRC_SHIFT         2
#define STLR_SYSTICK_CTRL_CLK_SRC_MASK          (1 << STLR_SYSTICK_CTRL_CLK_SRC_SHIFT)
#define STLR_SYSTICK_CTRL_CLK_SRC_PIOSC_DIV_4   (0 << STLR_SYSTICK_CTRL_CLK_SRC_SHIFT)
#define STLR_SYSTICK_CTRL_CLK_SRC_SYS_CLOCK     (1 << STLR_SYSTICK_CTRL_CLK_SRC_SHIFT)
#define STLR_SYSTICK_CTRL_COUNT_MASK            (1 << 3)