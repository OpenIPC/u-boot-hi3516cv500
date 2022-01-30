/*
 * Copyright (c) Hisilicon Technologies Co., Ltd. 2018-2020. All rights reserved.
 * Description: Higmac util header file
 * Author: KTP_BSP
 * Create: 2018-10-08
 * Notes:
 */

#ifndef __HIGMAC_UTIL_H__
#define __HIGMAC_UTIL_H__

#include <common.h>

#define HIGMAC_TRACE_ETH		2
#define HIGMAC_TRACE_MDIO		4
#define HIGMAC_TRACE_DRV		7
#define HIGMAC_TRACE_LEVEL		8

#define mk_bits(shift, nbits) ((((shift) & 0x1F) << 16) | ((nbits) & 0x3F))

#endif

