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

/* MPU register offsets *************************************************************/

#define MPU_TYPE_OFFSET      0xD90
#define MPU_CTRL_OFFSET      0xD94
#define MPU_NUMBER_OFFSET    0xD98
#define MPU_BASE_OFFSET      0xD9C
#define MPU_ATTR_OFFSET      0xDA0

/* MPU register addresses ***********************************************************/

#define LM3S_MPU_TYPE             (LM3S_CM3P_BASE + MPU_TYPE_OFFSET)
#define LM3S_MPU_CTRL             (LM3S_CM3P_BASE + MPU_CTRL_OFFSET)
#define LM3S_MPU_NUMBER           (LM3S_CM3P_BASE + MPU_NUMBER_OFFSET)
#define LM3S_MPU_BASE             (LM3S_CM3P_BASE + MPU_BASE_OFFSET)
#define LM3S_MPU_ATTR             (LM3S_CM3P_BASE + MPU_ATTR_OFFSET)

/* MPU register bit defitiions ******************************************************/

/* MPU Type (MPUTYPE), offset 0xD90 */

#define MPU_MPUTYPE_DREGION_SHIFT        8    /* Bits 15-8: Number of D Regions */
#define MPU_MPUTYPE_DREGION_MASK         (0xFF << MPU_MPUTYPE_DREGION_SHIFT)

/* MPU Control (MPUCTRL), offset 0xD94 */

#define MPU_CTRL_ENABLE_SHIFT            0    /* Bits    0: MPU Enable */
#define MPU_CTRL_ENABLE_MASK             (0x01 << MPU_CTRL_ENABLE_SHIFT)
#define MPU_CTRL_PRIVDEFEN_SHIFT         2    /* Bits    2: MPU Default Region */
#define MPU_CTRL_PRIVDEFEN_MASK          (0x01 << MPU_CTRL_PRIVDEFEN_SHIFT)

/* MPU Region Number (MPUNUMBER), offset 0xD98 */

#define MPU_NUMBER_NUMBER_SHIFT          0    /* Bits  2-0: MPU Region to Access */
#define MPU_NUMBER_NUMBER_MASK           (0x07 << MPU_NUMBER_NUMBER_SHIFT)

/* MPU Region Base Address (MPUBASE), offset 0xD9C */

#define MPU_BASE_REGION_SHIFT            0    /* Bits  2-0: Region Number */
#define MPU_BASE_REGION_MASK             (0x07 << MPU_BASE_REGION_SHIFT)
#define MPU_BASE_VALID_SHIFT             4    /* Bits    4: Region Number Valid */
#define MPU_BASE_VALID_MASK              (0x01 << MPU_BASE_VALID_SHIFT)
#define MPU_BASE_ADDRESS_SHIFT           0    /* Bits 31-0(4-0 reserved): Base Address Mask */
#define MPU_BASE_ADDRESS_MASK            (0xFFFFFFE0)

/* MPU Region Attribute and Size (MPUATTR), offset 0xDA0 */

#define MPU_ATTR_ENABLE_SHIFT            0    /* Bits    0: Region Enable */
#define MPU_ATTR_ENABLE_MASK             (0x01 << MPU_ATTR_ENABLE_SHIFT)
#define MPU_ATTR_SIZE_SHIFT              1    /* Bits  5-1: Region Size Mask */
#define MPU_ATTR_SIZE_MASK               (0x1F << MPU_ATTR_SIZE_SHIFT)
#define MPU_ATTR_SRD_SHIFT               8    /* Bits 15-8: Subregion Disable Bits */
#define MPU_ATTR_SRD_MASK                (0xFF << MPU_ATTR_SRD_SHIFT)
#define MPU_ATTR_B_SHIFT                 16   /* Bits   16: Bufferable */
#define MPU_ATTR_B_MASK                  (0x01 << MPU_ATTR_B_SHIFT)
#define MPU_ATTR_C_SHIFT                 17   /* Bits   17: Cacheable */
#define MPU_ATTR_C_MASK                  (0x01 << MPU_ATTR_C_SHIFT)
#define MPU_ATTR_S_SHIFT                 18   /* Bits   18: Shareable */
#define MPU_ATTR_S_MASK                  (0x01 << MPU_ATTR_S_SHIFT)
#define MPU_ATTR_TEX_SHIFT               19   /* Bits 21-19: Type Extension Mask */
#define MPU_ATTR_TEX_MASK                (0x07 << MPU_ATTR_TEX_SHIFT)
#define MPU_ATTR_AP_SHIFT                24   /* Bits 26-24: Access Privilege */
#define MPU_ATTR_AP_MASK                 (0x07 << MPU_ATTR_AP_SHIFT)
#define MPU_ATTR_XN_SHIFT                28   /* Bits    28: Instruction Access Disabled */
#define MPU_ATTR_XN_MASK                 (0x01 << MPU_ATTR_XN_SHIFT)
