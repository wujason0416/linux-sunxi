/*
 * Copyright (C) 2012  Alejandro Mery <amery@geeks.cl>
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

#define pr_fmt(fmt)	"pinctrl-sunxi: " fmt

#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/pinctrl/pinctrl.h>

#include <plat/core.h>

/*
 * pinctrl_pin_desc generation based on on our knowledge
 * about what pins are valid on each chip
 *
 * for any unknown chip-id we blindly assume the 32x9 are
 * available
 */

#define MAX_BANK 9

static struct pin_counts {
	u32 chip_id;
	u8 pin_counts[MAX_BANK];
} pin_counts[] = {
#ifdef CONFIG_ARCH_SUN4I
	{ SUNXI_CHIP_ID_A10, { 18, 24, 25, 28, 12,  6, 12, 28, 22 }},
#endif
#ifdef CONFIG_ARCH_SUN5I
	{ SUNXI_CHIP_ID_A10s,{ 18, 21, 20, 28, 12,  6, 14 }},
	{ SUNXI_CHIP_ID_A13, {  0, 10, 17, 22, 12,  6,  9 }},
#endif
	{ 0, { 32, 32, 32, 32, 32, 32, 32, 32, 32 }},
};

static const struct pinctrl_pin_desc *pindesc_gen(u32 *npins) {
	u32 chip_id = sunxi_chip_id();
	u32 i, j, l;

	struct pin_counts *pc = pin_counts;
	struct pinctrl_pin_desc *pins;
	char *buf;

	for (pc = pin_counts; pc->chip_id; pc++) {
		if (pc->chip_id == chip_id)
			break;
	}

	for (i=0, l=0; i<MAX_BANK; i++)
		l += pc->pin_counts[i];

	pins = kmalloc(l*(5+sizeof(struct pinctrl_pin_desc)), GFP_KERNEL);
	if (!pins) {
		l = 0;
		goto done;
	}
	buf = (char*)(pins+l);

	for (i=0, l=0; i<MAX_BANK; i++) {
		u32 count = pc->pin_counts[i];
		for (j=0; j<count; j++) {
			sprintf(buf, "P%c%d", 'A'+i, j);
			pins[l++] = (struct pinctrl_pin_desc)PINCTRL_PIN((i<<5)+j, buf);
			buf += 5;
		}
	}

done:
	*npins = l;
	return pins;
}

static int pinctrl_sunxi_init(void)
{
	u32 npins = 0;
	const struct pinctrl_pin_desc *pins = pindesc_gen(&npins);
	if (pins) {
		u32 i;
		for (i=0; i<npins; i++)
			pr_debug("i:%u pin:%u name:%s\n",
				 i, pins[i].number, pins[i].name);

		kfree(pins);
	}
	return 0;
}
arch_initcall(pinctrl_sunxi_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alejandro Mery <amery@geeks.cl>");
MODULE_DESCRIPTION("sunxi pin control driver");
