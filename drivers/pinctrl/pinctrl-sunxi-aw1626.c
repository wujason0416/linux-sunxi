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

#define DRIVER_NAME	"pinctrl-sunxi-aw1626"
#define pr_fmt(fmt)	DRIVER_NAME ": " fmt

#include <linux/device.h>
#include <linux/module.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/platform_device.h>

#include <linux/platform_data/pinctrl-sunxi.h>

static struct pinctrl_pin_desc aw1626_pins[] = {
	PINCTRL_PIN(SUNXI_PB0, "PB0"),
	PINCTRL_PIN(SUNXI_PB1, "PB1"),
	PINCTRL_PIN(SUNXI_PB2, "PB2"),
	PINCTRL_PIN(SUNXI_PB3, "PB3"),
	PINCTRL_PIN(SUNXI_PB4, "PB4"),
	PINCTRL_PIN(SUNXI_PB5, "PB5"),
	PINCTRL_PIN(SUNXI_PB6, "PB6"),
	PINCTRL_PIN(SUNXI_PB7, "PB7"),
	PINCTRL_PIN(SUNXI_PB8, "PB8"),
	PINCTRL_PIN(SUNXI_PB9, "PB9"),
	PINCTRL_PIN(SUNXI_PC0, "PC0"),
	PINCTRL_PIN(SUNXI_PC1, "PC1"),
	PINCTRL_PIN(SUNXI_PC2, "PC2"),
	PINCTRL_PIN(SUNXI_PC3, "PC3"),
	PINCTRL_PIN(SUNXI_PC4, "PC4"),
	PINCTRL_PIN(SUNXI_PC5, "PC5"),
	PINCTRL_PIN(SUNXI_PC6, "PC6"),
	PINCTRL_PIN(SUNXI_PC7, "PC7"),
	PINCTRL_PIN(SUNXI_PC8, "PC8"),
	PINCTRL_PIN(SUNXI_PC9, "PC9"),
	PINCTRL_PIN(SUNXI_PC10, "PC10"),
	PINCTRL_PIN(SUNXI_PC11, "PC11"),
	PINCTRL_PIN(SUNXI_PC12, "PC12"),
	PINCTRL_PIN(SUNXI_PC13, "PC13"),
	PINCTRL_PIN(SUNXI_PC14, "PC14"),
	PINCTRL_PIN(SUNXI_PC15, "PC15"),
	PINCTRL_PIN(SUNXI_PC16, "PC16"),
	PINCTRL_PIN(SUNXI_PD0, "PD0"),
	PINCTRL_PIN(SUNXI_PD1, "PD1"),
	PINCTRL_PIN(SUNXI_PD2, "PD2"),
	PINCTRL_PIN(SUNXI_PD3, "PD3"),
	PINCTRL_PIN(SUNXI_PD4, "PD4"),
	PINCTRL_PIN(SUNXI_PD5, "PD5"),
	PINCTRL_PIN(SUNXI_PD6, "PD6"),
	PINCTRL_PIN(SUNXI_PD7, "PD7"),
	PINCTRL_PIN(SUNXI_PD8, "PD8"),
	PINCTRL_PIN(SUNXI_PD9, "PD9"),
	PINCTRL_PIN(SUNXI_PD10, "PD10"),
	PINCTRL_PIN(SUNXI_PD11, "PD11"),
	PINCTRL_PIN(SUNXI_PD12, "PD12"),
	PINCTRL_PIN(SUNXI_PD13, "PD13"),
	PINCTRL_PIN(SUNXI_PD14, "PD14"),
	PINCTRL_PIN(SUNXI_PD15, "PD15"),
	PINCTRL_PIN(SUNXI_PD16, "PD16"),
	PINCTRL_PIN(SUNXI_PD17, "PD17"),
	PINCTRL_PIN(SUNXI_PD18, "PD18"),
	PINCTRL_PIN(SUNXI_PD19, "PD19"),
	PINCTRL_PIN(SUNXI_PD20, "PD20"),
	PINCTRL_PIN(SUNXI_PD21, "PD21"),
	PINCTRL_PIN(SUNXI_PE0, "PE0"),
	PINCTRL_PIN(SUNXI_PE1, "PE1"),
	PINCTRL_PIN(SUNXI_PE2, "PE2"),
	PINCTRL_PIN(SUNXI_PE3, "PE3"),
	PINCTRL_PIN(SUNXI_PE4, "PE4"),
	PINCTRL_PIN(SUNXI_PE5, "PE5"),
	PINCTRL_PIN(SUNXI_PE6, "PE6"),
	PINCTRL_PIN(SUNXI_PE7, "PE7"),
	PINCTRL_PIN(SUNXI_PE8, "PE8"),
	PINCTRL_PIN(SUNXI_PE9, "PE9"),
	PINCTRL_PIN(SUNXI_PE10, "PE10"),
	PINCTRL_PIN(SUNXI_PE11, "PE11"),
	PINCTRL_PIN(SUNXI_PF0, "PF0"),
	PINCTRL_PIN(SUNXI_PF1, "PF1"),
	PINCTRL_PIN(SUNXI_PF2, "PF2"),
	PINCTRL_PIN(SUNXI_PF3, "PF3"),
	PINCTRL_PIN(SUNXI_PF4, "PF4"),
	PINCTRL_PIN(SUNXI_PF5, "PF5"),
	PINCTRL_PIN(SUNXI_PG0, "PG0"),
	PINCTRL_PIN(SUNXI_PG1, "PG1"),
	PINCTRL_PIN(SUNXI_PG2, "PG2"),
	PINCTRL_PIN(SUNXI_PG3, "PG3"),
	PINCTRL_PIN(SUNXI_PG4, "PG4"),
	PINCTRL_PIN(SUNXI_PG5, "PG5"),
	PINCTRL_PIN(SUNXI_PG6, "PG6"),
	PINCTRL_PIN(SUNXI_PG7, "PG7"),
	PINCTRL_PIN(SUNXI_PG8, "PG8"),
};
static struct sunxi_pinctrl_soc_data aw1626_pinctrl = {
	.name = "aw1626",
	.pins = aw1626_pins,
	.npins = ARRAY_SIZE(aw1626_pins),
};

static int __devinit aw1626_pinctrl_probe(struct platform_device *pdev)
{
	return sunxi_pinctrl_probe(pdev, &aw1626_pinctrl);
}

static struct of_device_id aw1626_pinctrl_of_match[] __devinitdata = {
	{ .compatible = "allwinner,aw1626-pinctrl", },
	{ .compatible = "allwinner,sunxi-aw1626-pinctrl", },
	{ .compatible = "allwinner,sunxi-a13-pinctrl", },
	{ },
};

static struct platform_driver aw1626_pinctrl_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = aw1626_pinctrl_of_match,
	},
	.probe = aw1626_pinctrl_probe,
	.remove = __devexit_p(sunxi_pinctrl_remove),
};

static int __init aw1626_pinctrl_init(void)
{
	return platform_driver_register(&aw1626_pinctrl_driver);
}
arch_initcall(aw1626_pinctrl_init);

static void __exit aw1626_pinctrl_exit(void)
{
	platform_driver_unregister(&aw1626_pinctrl_driver);
}
module_exit(aw1626_pinctrl_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alejandro Mery <amery@geeks.cl>");
MODULE_DESCRIPTION("sunxi (A13/AW1626) pin control driver");
MODULE_DEVICE_TABLE(of, aw1626_pinctrl_of_match);
