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

#define DRIVER_NAME	"pinctrl-sunxi"
#define pr_fmt(fmt)	DRIVER_NAME ": " fmt

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <linux/platform_data/pinctrl-sunxi.h>

/*
 * PIO
 */
struct sunxi_pio_bank {
	u32 mux[4];
	u32 dat;
	u32 drv[2];
	u32 pull[2];
};

struct sunxi_pio_intr {
	u32 cfg[3];
	u32 ctl;
	u32 sta;
	u32 deb;	/* interrupt debounce */
};

struct sunxi_pio_reg {
	struct sunxi_pio_bank bank[9];
	u8 reserved[0xbc];
	struct sunxi_pio_intr intr;
};

static inline unsigned sunxi_pio_get_mux(struct sunxi_pio_reg *reg,
					 unsigned bank, unsigned num)
{
	struct sunxi_pio_bank *pio = &reg->bank[bank];
	unsigned index = num>>3;
	unsigned offset = (num&0x7)<<2;

	return (pio->mux[index]>>offset)&0x7;
}

static inline unsigned sunxi_pio_get_drv(struct sunxi_pio_reg *reg,
					 unsigned bank, unsigned num)
{
	struct sunxi_pio_bank *pio = &reg->bank[bank];
	unsigned index = num>>4;
	unsigned offset = (num&0xf)<<1;

	return (pio->drv[index]>>offset)&0x3;
}

static inline unsigned sunxi_pio_get_pull(struct sunxi_pio_reg *reg,
					 unsigned bank, unsigned num)
{
	struct sunxi_pio_bank *pio = &reg->bank[bank];
	unsigned index = num>>4;
	unsigned offset = (num&0xf)<<1;

	return (pio->pull[index]>>offset)&0x3;
}

static inline unsigned sunxi_pio_get_val(struct sunxi_pio_reg *reg,
					 unsigned bank, unsigned num)
{
	struct sunxi_pio_bank *pio = &reg->bank[bank];
	return (pio->dat>>num)&0x01;
}

/*
 */
int __devinit sunxi_pinctrl_probe(struct platform_device *pdev,
				  struct sunxi_pinctrl_soc_data *soc_data)
{
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_pinctrl_probe);

int __devexit sunxi_pinctrl_remove(struct platform_device *pdev)
{
	return 0;
};
EXPORT_SYMBOL_GPL(sunxi_pinctrl_remove);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alejandro Mery <amery@geeks.cl>");
MODULE_DESCRIPTION("sunxi pin control driver");
