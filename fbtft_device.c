/*
 *
 * Copyright (C) 2013, Noralf Tronnes
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>

#include "fbtft.h"

#define DRVNAME "fbtft_device"

#define MAX_GPIOS 32

struct spi_device *spi_device;
struct platform_device *p_device;

static char *name;
module_param(name, charp, 0);
MODULE_PARM_DESC(name, "Devicename (required). " \
"name=list => list all supported devices.");


static unsigned busnum;
module_param(busnum, uint, 0);
MODULE_PARM_DESC(busnum, "SPI bus number (default=0)");

static unsigned cs;
module_param(cs, uint, 0);
MODULE_PARM_DESC(cs, "SPI chip select (default=0)");

static unsigned speed;
module_param(speed, uint, 0);
MODULE_PARM_DESC(speed, "SPI speed (override device default)");

static int mode = -1;
module_param(mode, int, 0);
MODULE_PARM_DESC(mode, "SPI mode (override device default)");

static unsigned long debug;
module_param(debug, ulong , 0);
MODULE_PARM_DESC(debug,
"level: 0-7 (the remaining 29 bits is for advanced usage)");

static unsigned verbose = 3;
module_param(verbose, uint, 0);
MODULE_PARM_DESC(verbose,
"0 silent, >0 show gpios, >1 show devices, >2 show devices before (default=3)");


struct fbtft_device_display {
	char *name;
	struct spi_board_info *spi;
};

/* Supported displays in alphabetical order */
static struct fbtft_device_display displays[] = {
	{
		.name = "spidev",
		.spi = &(struct spi_board_info) {
			.modalias = "spidev",
//			.max_speed_hz = 32000000,
			.max_speed_hz = 10000000,
			.mode = SPI_MODE_3,
		}
    },{
		.name = "lepton",
		.spi = &(struct spi_board_info) {
			.modalias = "lepton",
//			.max_speed_hz = 32000000,
			.max_speed_hz = 10000000,
			.mode = SPI_MODE_3,
		}
	}, {
		/* This should be the last item.
		   Used with the custom argument */
		.name = "",
		.spi = &(struct spi_board_info) {
			.modalias = "",
			.max_speed_hz = 0,
			.mode = SPI_MODE_0,
			.platform_data = &(struct fbtft_platform_data) {
				.gpios = (const struct fbtft_gpio []) {
					{},
				},
			}
		},
	}
};

static int spi_device_found(struct device *dev, void *data)
{
	struct spi_device *spi = container_of(dev, struct spi_device, dev);

	pr_info(DRVNAME":      %s %s %dkHz %d bits mode=0x%02X\n",
		spi->modalias, dev_name(dev), spi->max_speed_hz/1000,
		spi->bits_per_word, spi->mode);

	return 0;
}

static void pr_spi_devices(void)
{
	pr_info(DRVNAME":  SPI devices registered:\n");
	bus_for_each_dev(&spi_bus_type, NULL, NULL, spi_device_found);
}

#ifdef MODULE
static void fbtft_device_spi_delete(struct spi_master *master, unsigned cs)
{
	struct device *dev;
	char str[32];

	snprintf(str, sizeof(str), "%s.%u", dev_name(&master->dev), cs);

	dev = bus_find_device_by_name(&spi_bus_type, NULL, str);
	if (dev) {
		if (verbose)
			pr_info(DRVNAME": Deleting %s\n", str);
		device_del(dev);
	}
}

static int fbtft_device_spi_device_register(struct spi_board_info *spi)
{
	struct spi_master *master;

	master = spi_busnum_to_master(spi->bus_num);
	if (!master) {
		pr_err(DRVNAME ":  spi_busnum_to_master(%d) returned NULL\n",
								spi->bus_num);
		return -EINVAL;
	}
	/* make sure it's available */
	fbtft_device_spi_delete(master, spi->chip_select);
	spi_device = spi_new_device(master, spi);
	put_device(&master->dev);
	if (!spi_device) {
		pr_err(DRVNAME ":    spi_new_device() returned NULL\n");
		return -EPERM;
	}
	return 0;
}
#else
static int fbtft_device_spi_device_register(struct spi_board_info *spi)
{
	return spi_register_board_info(spi, 1);
}
#endif

static int __init fbtft_device_init(void)
{
	struct spi_board_info *spi = NULL;
	struct fbtft_platform_data *pdata;
	bool found = false;
	int i = 0;
	int ret = 0;

	pr_debug("\n\n"DRVNAME": init\n");

	if (name == NULL) {
#ifdef MODULE
		pr_err(DRVNAME":  missing module parameter: 'name'\n");
		return -EINVAL;
#else
		return 0;
#endif
	}

	if (verbose > 2)
		pr_spi_devices(); /* print list of registered SPI devices */

	pr_debug(DRVNAME":  name='%s', busnum=%d, cs=%d\n", name, busnum, cs);

	/* name=list lists all supported displays */
	if (strncmp(name, "list", 32) == 0) {
		pr_info(DRVNAME":  Supported displays:\n");

		for (i = 0; i < ARRAY_SIZE(displays); i++)
			pr_info(DRVNAME":      %s\n", displays[i].name);
		return -ECANCELED;
	}

	for (i = 0; i < ARRAY_SIZE(displays); i++) {
		if (strncmp(name, displays[i].name, 32) == 0) {
			if (displays[i].spi) {
				spi = displays[i].spi;
				spi->chip_select = cs;
				spi->bus_num = busnum;
				if (speed)
					spi->max_speed_hz = speed;
				if (mode != -1)
					spi->mode = mode;
				pdata = (void *)spi->platform_data;
			} else {
				pr_err(DRVNAME": broken displays array\n");
				return -EINVAL;
			}

			if (displays[i].spi) {
				ret = fbtft_device_spi_device_register(spi);
				if (ret) {
					pr_err(DRVNAME \
						": failed to register SPI device\n");
					return ret;
				}
				found = true;
				break;
			}
		}
	}

	if (!found) {
		pr_err(DRVNAME":  display not supported: '%s'\n", name);
		return -EINVAL;
	}

	if (spi_device && (verbose > 1))
		pr_spi_devices();

	return 0;
}

static void __exit fbtft_device_exit(void)
{
	pr_debug(DRVNAME" - exit\n");

	if (spi_device) {
		device_del(&spi_device->dev);
		kfree(spi_device);
	}

}

arch_initcall(fbtft_device_init);
module_exit(fbtft_device_exit);

MODULE_DESCRIPTION("Add a FBTFT device.");
MODULE_AUTHOR("Noralf Tronnes");
MODULE_LICENSE("GPL");
