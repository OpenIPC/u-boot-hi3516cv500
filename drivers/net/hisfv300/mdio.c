/*
 * Copyright (c) Hisilicon Technologies Co., Ltd. 2018-2020. All rights reserved.
 * Description: Hieth mdio operation
 * Author: KTP_BSP
 * Create: 2018-10-08
 * Notes:
 */

#define HIETH_SFV300

#include "hieth.h"
#include "mdio.h"

/* write mdio registers reset value */
static void mdio_reg_reset(struct hieth_netdev_local *ld)
{
	hieth_writel(ld, 0x00008000, MDIO_RWCTRL);
	hieth_writel(ld, 0x00000001, U_MDIO_PHYADDR);
	hieth_writel(ld, 0x00000001, D_MDIO_PHYADDR);
	hieth_writel(ld, 0x04631EA9, U_MDIO_ANEG_CTRL);
	hieth_writel(ld, 0x04631EA9, D_MDIO_ANEG_CTRL);
	hieth_writel(ld, 0x00000000, U_MDIO_IRQENA);
	hieth_writel(ld, 0x00000000, D_MDIO_IRQENA);
}

static int wait_mdio_ready(struct hieth_netdev_local *ld)
{
	int timeout_us = 5000;

	while (--timeout_us && !test_mdio_ready(ld))
		udelay(50);  /* delay 50us */

	return timeout_us;
}

int hieth_mdio_read(struct hieth_netdev_local *ld, int phy_addr, unsigned int regnum)
{
	int val = 0;
	hieth_assert((!((unsigned int)phy_addr & (~0x1F))) && (!(regnum & (~0x1F))));

	local_lock(ld);

	if (!wait_mdio_ready(ld)) {
		hieth_error("mdio busy");
		goto error_exit;
	}

	mdio_start_phyread(ld, (unsigned int)phy_addr, regnum);

	if (wait_mdio_ready(ld) != 0) {
		val = mdio_get_phyread_val(ld);
	} else {
		hieth_error("read timeout");
	}

error_exit:

	local_unlock(ld);

	hieth_trace(HIETH_TRACE_MDIO, "phy_addr = %d, regnum = %d, val = 0x%04x", phy_addr,
				(int)regnum, val);

	return val;
}

int hieth_mdio_write(struct hieth_netdev_local *ld, int phy_addr, int regnum, int val)
{
	hieth_assert((!((unsigned int)phy_addr & (~0x1F))) && (!((unsigned int)regnum & (~0x1F))));

	hieth_trace(HIETH_TRACE_MDIO, "phy_addr = %d, regnum = %d", phy_addr, regnum);

	local_lock(ld);

	if (!wait_mdio_ready(ld)) {
		hieth_error("mdio busy");
		val = -1;
		goto error_exit;
	}

	mdio_phywrite(ld, (unsigned int)phy_addr, (unsigned int)regnum, (unsigned int)val);

error_exit:

	local_unlock(ld);

	return val;
}

int hieth_mdio_reset(struct hieth_netdev_local *ld)
{
	mdio_reg_reset(ld);

	return 0;
}

int hieth_mdio_init(struct hieth_netdev_local *ld)
{
	local_lock_init(ld);

	hieth_mdio_reset(ld);

	return 0;
}

void hieth_mdio_exit(struct hieth_netdev_local *ld)
{
	local_lock_exit(ld);
}
