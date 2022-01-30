/*
 * Copyright (c) Hisilicon Technologies Co., Ltd. 2018-2020. All rights reserved.
 * Description: Hieth system operation header file
 * Author: KTP_BSP
 * Create: 2018-10-08
 * Notes:
 */

#ifndef __HIETH_SYS_H__
#define __HIETH_SYS_H__

void hieth_sys_init(void);
void hieth_sys_exit(void);

void hieth_set_crg_phy_mode(unsigned char is_rmii_mode);
void set_inner_phy_addr(u32 phyaddr);
void set_efuse_unread(void);

void hieth_sys_startup(void);
void hieth_sys_allstop(void);

void set_phy_valtage(void);
#endif
