/*
 * Copyright 2015 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Authors: AMD
 *
 */

#ifndef __ACP_2X_H__
#define __ACP_2X_H__

#define ST_JADEITE 1
#define ACP_TILE_ON_MASK			0x03
#define ACP_TILE_OFF_MASK			0x02
#define ACP_TILE_ON_RETAIN_REG_MASK		0x1f
#define ACP_TILE_OFF_RETAIN_REG_MASK		0x20

#define ACP_TILE_P1_MASK			0x3e
#define ACP_TILE_P2_MASK			0x3d
#define ACP_TILE_DSP0_MASK			0x3b
#define ACP_TILE_DSP1_MASK			0x37

#define ACP_TILE_DSP2_MASK			0x2f

#define ACP_DMA_REGS_END			0x146c0
#define ACP_I2S_PLAY_REGS_START			0x14840
#define ACP_I2S_PLAY_REGS_END			0x148b4
#define ACP_I2S_CAP_REGS_START			0x148b8
#define ACP_I2S_CAP_REGS_END			0x1496c

#define ACP_I2S_COMP1_CAP_REG_OFFSET		0xac
#define ACP_I2S_COMP2_CAP_REG_OFFSET		0xa8
#define ACP_I2S_COMP1_PLAY_REG_OFFSET		0x6c
#define ACP_I2S_COMP2_PLAY_REG_OFFSET		0x68
#define ACP_BT_PLAY_REGS_START			0x14970
#define ACP_BT_PLAY_REGS_END			0x14a24
#define ACP_BT_COMP1_REG_OFFSET			0xac
#define ACP_BT_COMP2_REG_OFFSET			0xa8

#define mmACP_PGFSM_RETAIN_REG			0x51c9
#define mmACP_PGFSM_CONFIG_REG			0x51ca
#define mmACP_PGFSM_READ_REG_0			0x51cc

#define mmACP_MEM_SHUT_DOWN_REQ_LO		0x51f8
#define mmACP_MEM_SHUT_DOWN_REQ_HI		0x51f9
#define mmACP_MEM_SHUT_DOWN_STS_LO		0x51fa
#define mmACP_MEM_SHUT_DOWN_STS_HI		0x51fb

#define mmACP_CONTROL				0x5131
#define mmACP_STATUS				0x5133
#define mmACP_SOFT_RESET			0x5134
#define ACP_CONTROL__ClkEn_MASK			0x1
#define ACP_SOFT_RESET__SoftResetAud_MASK	0x100
#define ACP_SOFT_RESET__SoftResetAudDone_MASK	0x1000000
#define ACP_CLOCK_EN_TIME_OUT_VALUE		0x000000FF
#define ACP_SOFT_RESET_DONE_TIME_OUT_VALUE	0x000000FF

#define ACP_TIMEOUT_LOOP			0x000000FF
#define ACP_DEVS				4
#define ACP_SRC_ID				162

#define PLAYBACK_MIN_NUM_PERIODS    2
#define PLAYBACK_MAX_NUM_PERIODS    2
#define PLAYBACK_MAX_PERIOD_SIZE    16384
#define PLAYBACK_MIN_PERIOD_SIZE    1024
#define CAPTURE_MIN_NUM_PERIODS     2
#define CAPTURE_MAX_NUM_PERIODS     2
#define CAPTURE_MAX_PERIOD_SIZE     16384
#define CAPTURE_MIN_PERIOD_SIZE     1024

#define MAX_BUFFER (PLAYBACK_MAX_PERIOD_SIZE * PLAYBACK_MAX_NUM_PERIODS)
#define MIN_BUFFER MAX_BUFFER

#define ST_PLAYBACK_MAX_PERIOD_SIZE 4096
#define ST_CAPTURE_MAX_PERIOD_SIZE  ST_PLAYBACK_MAX_PERIOD_SIZE
#define ST_MAX_BUFFER (ST_PLAYBACK_MAX_PERIOD_SIZE * PLAYBACK_MAX_NUM_PERIODS)
#define ST_MIN_BUFFER ST_MAX_BUFFER

#define ACPIMMIO_MISC_BASE		0xFED80E00
 /* Clock Control 1 register */
#define MISCCLKCNTL1	0x40
/* Auxiliary clock1 enable bit */
#define OSCCLKENB	2

#endif