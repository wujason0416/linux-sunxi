/*
 * Copyright (C) 2012 Alejandro Mery <amery@geeks.cl>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef _SUNXI_PINCTRL_H
#define _SUNXI_PINCTRL_H

/* pin id = bank<<5 + port */
#define SUNXI_PIN_ID(B, N)	(((B)<<5) + (N)&0x1f)

#define SUNXI_PIN_BANK(V)	(((V)>>5)&0xf)
#define SUNXI_PIN_PORT(V)	((V)&0x1f)

/* pin value = func<<24 + dlevel<<16 + pull<<8 + val
 *
 * 0xff means "don't touch", negative counts as 0xff
 */
#define _SUNXI_PIN_CFG_SAFE(V)	(((V) < 0) ? 0xff : (V)&0xff)

#define SUNXI_PIN_CFG(F, L, P, V)	(_SUNXI_PIN_CFG_SAFE(F)<<24 | \
					 _SUNXI_PIN_CFG_SAFE(L)<<16 | \
					 _SUNXI_PIN_CFG_SAFE(P)<<8 | \
					 _SUNXI_PIN_CFG_SAFE(V))

#define SUNXI_PIN_CFG_FUNC(V)	((V)>>24)
#define SUNXI_PIN_CFG_DLVL(V)	(((V)>>16)&0xff)
#define SUNXI_PIN_CFG_PULL(V)	(((V)>>8)&0xff)
#define SUNXI_PIN_CFG_VAL(V)	((V)&0xff)

/* traditional port names */
#define PA(N)	SUNXI_PIN_ID(0, N)
#define PB(N)	SUNXI_PIN_ID(1, N)
#define PC(N)	SUNXI_PIN_ID(2, N)
#define PD(N)	SUNXI_PIN_ID(3, N)
#define PE(N)	SUNXI_PIN_ID(4, N)
#define PF(N)	SUNXI_PIN_ID(5, N)
#define PG(N)	SUNXI_PIN_ID(6, N)
#define PH(N)	SUNXI_PIN_ID(7, N)
#define PI(N)	SUNXI_PIN_ID(8, N)

#endif
