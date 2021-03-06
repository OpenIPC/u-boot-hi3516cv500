/*
 * Copyright (c) Hisilicon Technologies Co., Ltd. 2016-2019. All rights reserved.
 * Description: vo hal level file.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "vou_hal.h"
#include <asm/io.h>
#include "hi3516ev200_vo.h"

#define IO_ADDRESS(x) (x)

#define HI_INVALID_LAYER (-1)

#define CRG_REGS_ADDR 0x12010000
#define CRG_REGS_SIZE 0X10000

#define CRG_PERCTL2_ADDR (0x0008 + CRG_REGS_ADDR)
#define CRG_PERCTL3_ADDR (0x000C + CRG_REGS_ADDR)
#define CRG_PERCTL4_ADDR (0x0010 + CRG_REGS_ADDR)
#define CRG_PERCTL5_ADDR (0x0014 + CRG_REGS_ADDR)
#define CRG_PERCTL6_ADDR (0x0018 + CRG_REGS_ADDR)
#define CRG_PERCTL7_ADDR (0x001c + CRG_REGS_ADDR)

#define CRG_PERCTL10_ADDR (0x0028 + CRG_REGS_ADDR)
#define CRG_PERCTL11_ADDR (0x002c + CRG_REGS_ADDR)
#define CRG_PERCTL18_ADDR (0x0048 + CRG_REGS_ADDR)

#define CRG_PERCTL19_ADDR (0x004c + CRG_REGS_ADDR)
#define CRG_PERCTL59_ADDR (0x00ec + CRG_REGS_ADDR)

#define CRG_PERCTL65_ADDR (0x0104 + CRG_REGS_ADDR) /* LCD CRG */
#define CRG_PERCTL66_ADDR (0x0108 + CRG_REGS_ADDR) /* VDP low power CRG */
#define CRG_PERCTL72_ADDR (0x0120 + CRG_REGS_ADDR)
#define CRG_PERCTL73_ADDR (0x0124 + CRG_REGS_ADDR) /* VDP CRG */
#define CRG_PERCTL15_ADDR (0x003C + CRG_REGS_ADDR)

#define LCD_CRG_PERCTL_ADDR CRG_PERCTL65_ADDR
#define VOU_CRG_PERCTL_ADDR CRG_PERCTL66_ADDR

#define MISC_REGS_ADDR 0x12028000
#define MISC_REGS_SIZE 0x8000
#define FDR_VID_OFFSET (0x200 / 4)

#define MISC_CTL18_ADDR (0x18 + MISC_REGS_ADDR)

volatile S_VDP_REGS_TYPE *g_vo_reg = HI_NULL;

hi_void hal_vo_init(hi_void)
{
    g_vo_reg = (volatile S_VDP_REGS_TYPE *)IO_ADDRESS(VO_BASE_ADDR);
}

hi_void hal_vo_exit(hi_void)
{
}

static inline void hi_reg_set_bit(unsigned long value, unsigned long offset,
                                  unsigned long addr)
{
    unsigned long t, mask;

    mask = 1 << offset;
    t = readl(addr);
    t &= ~mask;
    t |= (value << offset) & mask;
    writel(t, addr);
}

static inline void hi_reg_write32(unsigned long value, unsigned long mask,
                                  unsigned long addr)
{
    unsigned long t;

    t = readl(addr);
    t &= ~mask;
    t |= value & mask;
    writel(t, addr);
}

hi_void hal_write_reg(hi_u32 *address, hi_u32 value)
{
    *(volatile hi_u32 *)address = value;
    return;
}

hi_u32 hal_read_reg(const hi_u32 *address)
{
    return *(volatile hi_u32 *)(address);
}

hi_bool hal_intf_bt_set_dfir_en(hi_u32 dfir_en)
{
    U_INTF_BT_CTRL INTF_BT_CTRL;

    INTF_BT_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)&(g_vo_reg->INTF_BT_CTRL.u32));
    INTF_BT_CTRL.bits.dfir_en = dfir_en;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)&(g_vo_reg->INTF_BT_CTRL.u32), INTF_BT_CTRL.u32);

    return HI_TRUE;
}

hi_ulong vo_get_abs_addr(hal_disp_layer layer, hi_ulong reg)
{
    hi_ulong reg_abs_addr;

    switch (layer) {
        case HAL_DISP_LAYER_VHD0:
        case HAL_DISP_LAYER_VHD1: {
            reg_abs_addr = (reg) + (layer - HAL_DISP_LAYER_VHD0) * VHD_REGS_LEN;
            break;
        }

        case HAL_DISP_LAYER_GFX0:
        case HAL_DISP_LAYER_GFX1: {
            reg_abs_addr = (reg) + (layer - HAL_DISP_LAYER_GFX0) * GFX_REGS_LEN;
            break;
        }

        default: {
            return 0;
        }
    }

    return reg_abs_addr;
}

hi_ulong vo_get_chn_abs_addr(hal_disp_outputchannel chan, hi_ulong reg)
{
    volatile hi_ulong reg_abs_addr;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            reg_abs_addr = reg + (chan - HAL_DISP_CHANNEL_DHD0) * DHD_REGS_LEN;
            break;
        }

        default: {
            printf("error channel id found in %s: L%d\n", __FUNCTION__, __LINE__);
            return 0;
        }
    }

    return reg_abs_addr;
}

hi_s32 sys_hal_vo_bus_reset_sel(hi_bool reset)
{
    hi_u32 tmp = (reset == HI_TRUE) ? 1 : 0;
    hi_reg_set_bit(tmp, 0, IO_ADDRESS(VOU_CRG_PERCTL_ADDR));

    return 0;
}

hi_s32 sys_hal_vo_apb_clk_en(hi_bool clk_en)
{
    hi_u32 tmp = (clk_en == HI_TRUE) ? 1 : 0;
    hi_reg_set_bit(tmp, 1, IO_ADDRESS(VOU_CRG_PERCTL_ADDR));

    return 0;
}

hi_s32 sys_hal_vo_bus_clk_en(hi_bool clk_en)
{
    hi_u32 tmp = (clk_en == HI_TRUE) ? 1 : 0;
    hi_reg_set_bit(tmp, 2, IO_ADDRESS(VOU_CRG_PERCTL_ADDR)); /* 2bit */

    return 0;
}

hi_s32 sys_hal_vo_cfg_clk_en(hi_bool clk_en)
{
    hi_u32 tmp = (clk_en == HI_TRUE) ? 1 : 0;
    hi_reg_set_bit(tmp, 3, IO_ADDRESS(VOU_CRG_PERCTL_ADDR)); /* 3bit */

    return 0;
}

/* VO HD PPC clock gating enable */
hi_s32 sys_hal_vo_core_clk_en(hi_s32 dev, hi_bool clk_en)
{
    hi_u32 tmp = (clk_en == HI_TRUE) ? 1 : 0;

    hi_reg_set_bit(tmp, 5, IO_ADDRESS(VOU_CRG_PERCTL_ADDR)); /* 5bit */

    return 0;
}

hi_s32 sys_hal_vo_dev_clk_en(hi_s32 vo_dev, hi_bool clk_en)
{
    hi_u32 tmp = (clk_en == HI_TRUE) ? 1 : 0;

    if (vo_dev == 0) {
        hi_reg_set_bit(tmp, 5, IO_ADDRESS(VOU_CRG_PERCTL_ADDR)); /* 5bit */
        hi_reg_set_bit(tmp, 6, IO_ADDRESS(VOU_CRG_PERCTL_ADDR)); /* 6bit */
    } else {
        return -1;
    }

    return 0;
}

hi_s32 sys_hal_vo_bt_clk_en(hi_bool bt_clk_en)
{
    hi_u32 tmp = (bt_clk_en == HI_TRUE) ? 1 : 0;

    hi_reg_set_bit(tmp, 8, IO_ADDRESS(VOU_CRG_PERCTL_ADDR)); /* 8bit */
    hi_reg_set_bit(tmp, 9, IO_ADDRESS(VOU_CRG_PERCTL_ADDR)); /* 9bit */

    return 0;
}

hi_s32 sys_hal_vo_hd0_div_mode(hi_u32 hd0_div_mod)
{
    hi_reg_write32(hd0_div_mod << 12, 0x3 << 12, IO_ADDRESS(VOU_CRG_PERCTL_ADDR)); /* 12bit to set 0x3 */

    return 0;
}

hi_s32 sys_hal_vo_hd_out_pctrl(hi_bool clk_reverse)
{
    hi_u32 tmp = (clk_reverse == HI_TRUE) ? 1 : 0;

    hi_reg_set_bit(tmp, 20, IO_ADDRESS(VOU_CRG_PERCTL_ADDR)); /* 20bit */

    return 0;
}

hi_s32 sys_hal_vo_out_clk_sel(hi_u32 clk_sel)
{
    hi_reg_write32(clk_sel << 21, 0xf << 21, IO_ADDRESS(VOU_CRG_PERCTL_ADDR)); /* 21bit */

    return 0;
}

hi_s32 sys_hal_vo_lcd_clk_en(hi_bool clk_en)
{
    hi_u32 tmp = (clk_en == HI_TRUE) ? 1 : 0;

    hi_reg_set_bit(tmp, 27, IO_ADDRESS(LCD_CRG_PERCTL_ADDR)); /* 27bit */

    return 0;
}

hi_s32 sys_hal_lcd_mclk_div(hi_u32 mclk_div)
{
    hi_reg_write32(mclk_div, 0x7ffffff, IO_ADDRESS(LCD_CRG_PERCTL_ADDR));

    return 0;
}

hi_s32 sys_hal_vo_out_clk_en(hi_s32 dev, hi_bool clk_en)
{
    hi_u32 tmp = (clk_en == HI_TRUE) ? 1 : 0;

    hi_reg_set_bit(tmp, 8, IO_ADDRESS(VOU_CRG_PERCTL_ADDR)); /* 8bit */

    return 0;
}

hi_s32 sys_hal_lcd_data_mode(hi_u32 data_mode)
{
    hi_reg_write32(data_mode << 8, 0x7 << 8, IO_ADDRESS(MISC_CTL18_ADDR)); /* 8bit to set 0x7 */

    return 0;
}

hi_void hal_sys_control(hi_void)
{
    volatile U_VOCTRL VOCTRL;

    /* outstand */
    VOCTRL.u32 = g_vo_reg->VOCTRL.u32;
    VOCTRL.u32 = 0x80000000; /* 0x80000000 vo contrl */
    g_vo_reg->VOCTRL.u32 = VOCTRL.u32;
}

hi_ulong vo_get_intf_abs_addr(hal_disp_intf intf, hi_ulong reg)
{
    volatile hi_ulong reg_abs_addr;

    switch (intf) {
        case HAL_DISP_INTF_BT656:
        case HAL_DISP_INTF_BT1120: {
            reg_abs_addr = reg + 1 * INTF_REGS_LEN;
            break;
        }

        case HAL_DISP_INTF_LCD:
        case HAL_DISP_INTF_LCD_6BIT:
        case HAL_DISP_INTF_LCD_8BIT:
        case HAL_DISP_INTF_LCD_16BIT:
        case HAL_DISP_INTF_LCD_24BIT: {
            reg_abs_addr = reg + 2 * INTF_REGS_LEN; /* 2 to get addr */
            break;
        }

        default: {
            printf("error intf id found in %s: L%d\n", __FUNCTION__, __LINE__);
            return 0;
        }
    }

    return reg_abs_addr;
}

hi_ulong vo_get_vid_abs_addr(hal_disp_layer layer, hi_ulong reg)
{
    volatile hi_ulong reg_abs_addr;

    switch (layer) {
        case HAL_DISP_LAYER_VHD0: {
            reg_abs_addr = reg + (layer - HAL_DISP_LAYER_VHD0) * VID_REGS_LEN;
            break;
        }

        default: {
            return 0;
        }
    }

    return reg_abs_addr;
}

hi_ulong vo_get_gfx_abs_addr(hal_disp_layer layer, hi_ulong reg)
{
    volatile hi_ulong reg_abs_addr;

    switch (layer) {
        case HAL_DISP_LAYER_GFX0: {
            reg_abs_addr = reg + (layer - HAL_DISP_LAYER_GFX0) * GRF_REGS_LEN;
            break;
        }

        default: {
            return 0;
        }
    }

    return reg_abs_addr;
}

hi_bool hal_disp_set_intf_enable(hal_disp_outputchannel chan, hi_bool intf)
{
    volatile U_DHD0_CTRL DHD0_CTRL;
    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_CTRL.u32));
            DHD0_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            DHD0_CTRL.bits.intf_en = intf;
            hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_CTRL.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_lcd_serial_perd(hi_u32 serial_perd)
{
    U_INTF_LCD_CTRL INTF_LCD_CTRL;
    volatile hi_ulong addr_reg;

    addr_reg = vo_get_intf_abs_addr(HAL_DISP_INTF_LCD_8BIT, (hi_uintptr_t)&(g_vo_reg->INTF_HDMI_CTRL.u32));
    INTF_LCD_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);

    INTF_LCD_CTRL.bits.lcd_serial_perd = serial_perd;

    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, INTF_LCD_CTRL.u32);
    return HI_TRUE;
}

hi_bool hal_disp_set_intf_ctrl(hal_disp_intf intf, const hi_u32 *ctrl_info)
{
    U_INTF_BT_CTRL INTF_BT_CTRL;
    U_INTF_LCD_CTRL INTF_LCD_CTRL;
    volatile hi_ulong addr_reg;
    U_INTF_BT_CTRL *bt_ctrl = HI_NULL;
    U_INTF_LCD_CTRL *lcd_ctrl = HI_NULL;

    if ((intf == VO_INTF_BT1120) || (intf == VO_INTF_BT656)) {
        bt_ctrl = (U_INTF_BT_CTRL *)ctrl_info;
        addr_reg = vo_get_intf_abs_addr(intf, (hi_uintptr_t)&(g_vo_reg->INTF_HDMI_CTRL.u32));
        INTF_BT_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        INTF_BT_CTRL.bits.hdmi_mode = bt_ctrl->bits.hdmi_mode;
        INTF_BT_CTRL.bits.lcd_serial_mode = bt_ctrl->bits.lcd_serial_mode;
        INTF_BT_CTRL.bits.lcd_parallel_order = bt_ctrl->bits.lcd_parallel_order;
        INTF_BT_CTRL.bits.lcd_data_inv = bt_ctrl->bits.lcd_data_inv;
        INTF_BT_CTRL.bits.lcd_parallel_mode = bt_ctrl->bits.lcd_parallel_mode;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, INTF_BT_CTRL.u32);
    } else if ((intf == VO_INTF_LCD) || (intf == VO_INTF_LCD_6BIT) ||
               (intf == VO_INTF_LCD_8BIT) || (intf == VO_INTF_LCD_16BIT)) {
        lcd_ctrl = (U_INTF_LCD_CTRL *)ctrl_info;
        addr_reg = vo_get_intf_abs_addr(intf, (hi_uintptr_t)&(g_vo_reg->INTF_HDMI_CTRL.u32));
        INTF_LCD_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        INTF_LCD_CTRL.bits.hdmi_mode = lcd_ctrl->bits.hdmi_mode;
        INTF_LCD_CTRL.bits.lcd_serial_mode = lcd_ctrl->bits.lcd_serial_mode;
        INTF_LCD_CTRL.bits.lcd_parallel_order = lcd_ctrl->bits.lcd_parallel_order;
        INTF_LCD_CTRL.bits.lcd_data_inv = lcd_ctrl->bits.lcd_data_inv;
        INTF_LCD_CTRL.bits.lcd_parallel_mode = lcd_ctrl->bits.lcd_parallel_mode;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, INTF_LCD_CTRL.u32);
    } else {
        return HI_FALSE;
    }

    return HI_TRUE;
}

static hi_void hal_disp_set_intf_vertical_sync(hal_disp_outputchannel chan,
                                               const hal_disp_syncinfo *sync_info)
{
    volatile U_DHD0_VSYNC1 DHD0_VSYNC1;
    volatile U_DHD0_VSYNC2 DHD0_VSYNC2;
    volatile U_DHD0_VPLUS1 DHD0_VPLUS1;
    volatile U_DHD0_VPLUS2 DHD0_VPLUS2;

    volatile hi_ulong addr_reg;

    /* register is the value -1 */
    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_VSYNC1.u32));
    DHD0_VSYNC1.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    DHD0_VSYNC1.bits.vact = sync_info->vact - 1;
    DHD0_VSYNC1.bits.vbb = sync_info->vbb - 1;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_VSYNC1.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_VSYNC2.u32));
    DHD0_VSYNC2.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    DHD0_VSYNC2.bits.vfb = sync_info->vfb - 1;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_VSYNC2.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_VPLUS1.u32));
    DHD0_VPLUS1.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    DHD0_VPLUS1.bits.bvact = sync_info->bvact - 1;
    DHD0_VPLUS1.bits.bvbb = sync_info->bvbb - 1;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_VPLUS1.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_VPLUS2.u32));
    DHD0_VPLUS2.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    DHD0_VPLUS2.bits.bvfb = sync_info->bvfb - 1;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_VPLUS2.u32);

    return;
}

hi_bool hal_disp_set_intf_sync(hal_disp_outputchannel chan,
                               const hal_disp_syncinfo *sync_info, const hal_disp_syncinv *inv)
{
    volatile U_DHD0_CTRL DHD0_CTRL;
    volatile U_DHD0_HSYNC1 DHD0_HSYNC1;
    volatile U_DHD0_HSYNC2 DHD0_HSYNC2;
    volatile U_DHD0_PWR DHD0_PWR;

    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_CTRL.u32));
            DHD0_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            DHD0_CTRL.bits.iop = sync_info->iop;
            DHD0_CTRL.bits.intf_ihs = inv->hs_inv;
            DHD0_CTRL.bits.intf_ivs = inv->vs_inv;
            DHD0_CTRL.bits.intf_idv = inv->dv_inv;
            hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_CTRL.u32);

            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_HSYNC1.u32));
            DHD0_HSYNC1.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            DHD0_HSYNC1.bits.hact = sync_info->hact - 1;
            DHD0_HSYNC1.bits.hbb = (sync_info->hbb) - 1;
            hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_HSYNC1.u32);

            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_HSYNC2.u32));
            DHD0_HSYNC2.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            DHD0_HSYNC2.bits.hmid = (sync_info->hmid == 0) ? 0 : sync_info->hmid - 1;
            DHD0_HSYNC2.bits.hfb = (sync_info->hfb) - 1;
            hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_HSYNC2.u32);

            hal_disp_set_intf_vertical_sync(chan, sync_info);

            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_PWR.u32));
            DHD0_PWR.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            DHD0_PWR.bits.hpw = sync_info->hpw - 1;
            DHD0_PWR.bits.vpw = sync_info->vpw - 1;
            hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_PWR.u32);

            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_dev_multi_chn_en(hal_disp_outputchannel chn, hal_multi_chn multi_chn_en)
{
    volatile U_DHD0_PWR DHD0_PWR;
    volatile hi_ulong addr_reg;
    addr_reg = vo_get_chn_abs_addr(chn, (hi_uintptr_t)&(g_vo_reg->DHD0_PWR.u32));
    DHD0_PWR.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    DHD0_PWR.bits.multichn_en = multi_chn_en;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_PWR.u32);

    return HI_TRUE;
}

hi_bool hal_disp_set_intf_mux_sel(hal_disp_outputchannel chan, hal_disp_intf intf)
{
    volatile U_VO_MUX VO_MUX;
    volatile hi_ulong addr_reg;

    addr_reg = (hi_uintptr_t)&(g_vo_reg->VO_MUX.u32);
    VO_MUX.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    /* the numbers below are register config, not magic number. */
    switch (intf) {
        case HAL_DISP_INTF_BT1120: {
            VO_MUX.bits.digital_sel = 0;
            VO_MUX.bits.bt_sel = 0;
            break;
        }

        case HAL_DISP_INTF_BT656: {
            VO_MUX.bits.digital_sel = 1;
            VO_MUX.bits.bt_sel = 0;
            break;
        }

        case HAL_DISP_INTF_LCD:
        case HAL_DISP_INTF_LCD_6BIT:
        case HAL_DISP_INTF_LCD_8BIT:
        case HAL_DISP_INTF_LCD_16BIT: {
            VO_MUX.bits.digital_sel = 2; /* 2 register config */
            VO_MUX.bits.lcd_sel = 0;
            break;
        }

        default: {
            VO_MUX.bits.digital_sel = 15; /* 15 register config */
        }
    }

    hal_write_reg((hi_u32*)(hi_uintptr_t)&(g_vo_reg->VO_MUX.u32), VO_MUX.u32);

    return HI_TRUE;
}

hi_bool hal_disp_set_intf_clip(hal_disp_intf intf,
                               hi_bool clip,
                               const hal_disp_clip *clip_data)
{
    volatile U_BT_CLIP0_L BT_CLIP0_L;
    volatile U_BT_CLIP0_H BT_CLIP0_H;
    volatile hi_ulong addr_reg;

    addr_reg = (hi_uintptr_t)&(g_vo_reg->BT_CLIP0_L.u32);
    BT_CLIP0_L.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    BT_CLIP0_L.bits.clip_en = clip;
    BT_CLIP0_L.bits.clip_cl2 = clip_data->clip_low_y;
    BT_CLIP0_L.bits.clip_cl1 = clip_data->clip_low_cb;
    BT_CLIP0_L.bits.clip_cl0 = clip_data->clip_low_cr;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, BT_CLIP0_L.u32);

    addr_reg = (hi_uintptr_t)&(g_vo_reg->BT_CLIP0_H.u32);
    BT_CLIP0_H.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    BT_CLIP0_H.bits.clip_ch2 = clip_data->clip_high_y;
    BT_CLIP0_H.bits.clip_ch1 = clip_data->clip_high_cb;
    BT_CLIP0_H.bits.clip_ch0 = clip_data->clip_high_cr;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, BT_CLIP0_H.u32);

    return HI_TRUE;
}

hi_bool hal_disp_set_vt_thd_mode(hal_disp_outputchannel chan, hi_u32 field_mode)
{
    volatile U_DHD0_VTTHD DHD0_VTTHD;
    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_VTTHD.u32));
            DHD0_VTTHD.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            DHD0_VTTHD.bits.thd1_mode = field_mode;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_VTTHD.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_vt_thd(hal_disp_outputchannel chan, hi_u32 vtthd)
{
    volatile U_DHD0_VTTHD DHD0_VTTHD;
    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_VTTHD.u32));
            DHD0_VTTHD.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            DHD0_VTTHD.bits.vtmgthd1 = vtthd;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_VTTHD.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_int_mask(hi_u32 mask_en)
{
    volatile U_VOINTMSK VOINTMSK;
    /* dispaly interrupt mask enable */
    VOINTMSK.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)&(g_vo_reg->VOINTMSK.u32));
    VOINTMSK.u32 = VOINTMSK.u32 | mask_en;
    hal_write_reg((hi_u32*)(hi_uintptr_t)&(g_vo_reg->VOINTMSK.u32), VOINTMSK.u32);

    return HI_TRUE;
}

hi_bool hal_disp_clr_int_mask(hi_u32 mask_en)
{
    volatile U_VOINTMSK VOINTMSK;

    /* dispaly interrupt mask enable */
    VOINTMSK.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)&(g_vo_reg->VOINTMSK.u32));
    VOINTMSK.u32 = VOINTMSK.u32 & (~mask_en);
    hal_write_reg((hi_u32*)(hi_uintptr_t)&(g_vo_reg->VOINTMSK.u32), VOINTMSK.u32);

    return HI_TRUE;
}

hi_void hal_disp_set_reg_up(hal_disp_outputchannel chan)
{
    volatile U_DHD0_CTRL DHD0_CTRL;
    volatile hi_ulong addr_reg;

    if (chan >= HAL_DISP_CHANNEL_DHD1) {
        printf("error,hal_disp_set_reg_up select wrong CHANNEL ID\n");
        return;
    }

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->DHD0_CTRL.u32));
    DHD0_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    DHD0_CTRL.bits.regup = 0x1; /* 0x1 reg up */
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, DHD0_CTRL.u32);
    return;
}

hi_bool hal_video_set_layer_up_mode(hal_disp_layer layer, hi_u32 up_mode)
{
    U_V0_CTRL V0_CTRL;
    volatile hi_ulong addr_reg;

    switch (layer) {
        case HAL_DISP_LAYER_VHD0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CTRL.u32));
            V0_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_CTRL.bits.rgup_mode = up_mode;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CTRL.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_void hal_video_hfir_set_ck_gt_en(hal_disp_layer layer, hi_u32 ck_gt_en)
{
    U_V0_HFIR_CTRL V0_HFIR_CTRL;
    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HFIR_CTRL.u32));
        V0_HFIR_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HFIR_CTRL.bits.ck_gt_en = ck_gt_en;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_HFIR_CTRL.u32);
    }

    return;
}

hi_void hal_video_cvfir_set_out_height(hal_disp_layer layer, hi_u32 out_height)
{
    U_V0_CVFIR_VINFO V0_CVFIR_VINFO;
    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CVFIR_VINFO.u32));
        V0_CVFIR_VINFO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_CVFIR_VINFO.bits.out_height = out_height - 1;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CVFIR_VINFO.u32);
    }

    return;
}

hi_void hal_video_cvfir_set_out_fmt(hal_disp_layer layer, hi_u32 out_fmt)
{
    U_V0_CVFIR_VINFO V0_CVFIR_VINFO;
    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CVFIR_VINFO.u32));
        V0_CVFIR_VINFO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_CVFIR_VINFO.bits.out_fmt = out_fmt;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CVFIR_VINFO.u32);
    }

    return;
}

hi_void hal_video_cvfir_set_out_pro(hal_disp_layer layer, hi_u32 out_pro)
{
    U_V0_CVFIR_VINFO V0_CVFIR_VINFO;
    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CVFIR_VINFO.u32));
        V0_CVFIR_VINFO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_CVFIR_VINFO.bits.out_pro = out_pro;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CVFIR_VINFO.u32);
    }

    return;
}

hi_void hal_video_cvfir_set_vzme_ck_gt_en(hal_disp_layer layer, hi_bool vzme_ck_gt_en)
{
    U_V0_CVFIR_VINFO V0_CVFIR_VINFO;
    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CVFIR_VINFO.u32));
        V0_CVFIR_VINFO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_CVFIR_VINFO.bits.vzme_ck_gt_en = vzme_ck_gt_en;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CVFIR_VINFO.u32);
    }

    return;
}

hi_void hal_video_cvfir_set_cvfir_en(hal_disp_layer layer, hi_u32 cvfir_en)
{
    U_V0_CVFIR_VSP V0_CVFIR_VSP;
    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CVFIR_VSP.u32));
        V0_CVFIR_VSP.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_CVFIR_VSP.bits.cvfir_en = cvfir_en;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CVFIR_VSP.u32);
    }

    return;
}

hi_void hal_video_cvfir_set_cvmid_en(hal_disp_layer layer, hi_u32 cvmid_en)
{
    U_V0_CVFIR_VSP V0_CVFIR_VSP;
    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CVFIR_VSP.u32));
        V0_CVFIR_VSP.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_CVFIR_VSP.bits.cvmid_en = cvmid_en;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CVFIR_VSP.u32);
    }

    return;
}

hi_void hal_video_cvfir_set_cvfir_mode(hal_disp_layer layer, hi_u32 cvfir_mode)
{
    U_V0_CVFIR_VSP V0_CVFIR_VSP;
    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CVFIR_VSP.u32));
        V0_CVFIR_VSP.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_CVFIR_VSP.bits.cvfir_mode = cvfir_mode;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CVFIR_VSP.u32);
    }

    return;
}

hi_void hal_video_cvfir_set_vratio(hal_disp_layer layer, hi_u32 vratio)
{
    U_V0_CVFIR_VSP V0_CVFIR_VSP;
    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CVFIR_VSP.u32));
        V0_CVFIR_VSP.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_CVFIR_VSP.bits.vratio = vratio;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CVFIR_VSP.u32);
    }

    return;
}

hi_void hal_video_cvfir_set_v_chroma_offset(hal_disp_layer layer, hi_u32 vchroma_offset)
{
    U_V0_CVFIR_VOFFSET V0_CVFIR_VOFFSET;
    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CVFIR_VOFFSET.u32));
        V0_CVFIR_VOFFSET.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_CVFIR_VOFFSET.bits.vchroma_offset = vchroma_offset;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CVFIR_VOFFSET.u32);
    }

    return;
}

hi_void hal_video_cvfir_set_vb_chroma_offset(hal_disp_layer layer, hi_u32 vbchroma_offset)
{
    U_V1_CVFIR_VBOFFSET V0_CVFIR_VBOFFSET;
    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CVFIR_VBOFFSET.u32));
        V0_CVFIR_VBOFFSET.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_CVFIR_VBOFFSET.bits.vbchroma_offset = vbchroma_offset;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CVFIR_VBOFFSET.u32);
    }

    return;
}

hi_void hal_video_hfir_set_mid_en(hal_disp_layer layer, hi_u32 mid_en)
{
    U_V0_HFIR_CTRL V0_HFIR_CTRL;
    volatile hi_ulong addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HFIR_CTRL.u32));
        V0_HFIR_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HFIR_CTRL.bits.mid_en = mid_en;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HFIR_CTRL.u32);
    }

    return;
}

hi_void vdp_fdr_vid_set_chm_copy_en(hi_u32 layer, hi_u32 chm_copy_en)
{
    U_VID_READ_CTRL VID_READ_CTRL;

    VID_READ_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)(&(g_vo_reg->VID_READ_CTRL.u32) + layer * FDR_VID_OFFSET));
    VID_READ_CTRL.bits.chm_copy_en = chm_copy_en;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)(&(g_vo_reg->VID_READ_CTRL.u32) + layer * FDR_VID_OFFSET),
                   VID_READ_CTRL.u32);
    return;
}

hi_bool hal_video_set_hfir_mode(hal_disp_layer layer, hal_hfirmode mode)
{
    volatile U_V0_HFIR_CTRL V0_HFIR_CTRL;
    volatile hi_ulong addr_reg;

    switch (layer) {
        case HAL_DISP_LAYER_VHD0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HFIR_CTRL.u32));
            V0_HFIR_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_HFIR_CTRL.bits.hfir_mode = mode;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_HFIR_CTRL.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_video_hfir_set_coef(hal_disp_layer layer, const hfir_coef *coef)
{
    volatile U_V0_HFIRCOEF01 V0_HFIRCOEF01;
    volatile U_V0_HFIRCOEF23 V0_HFIRCOEF23;
    volatile U_V0_HFIRCOEF45 V0_HFIRCOEF45;
    volatile U_V0_HFIRCOEF67 V0_HFIRCOEF67;
    volatile hi_ulong addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HFIRCOEF01.u32));
        V0_HFIRCOEF01.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HFIRCOEF01.bits.coef0 = coef->coef0;
        V0_HFIRCOEF01.bits.coef1 = coef->coef1;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HFIRCOEF01.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HFIRCOEF23.u32));
        V0_HFIRCOEF23.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HFIRCOEF23.bits.coef2 = coef->coef2;
        V0_HFIRCOEF23.bits.coef3 = coef->coef3;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HFIRCOEF23.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HFIRCOEF45.u32));
        V0_HFIRCOEF45.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HFIRCOEF45.bits.coef4 = coef->coef4;
        V0_HFIRCOEF45.bits.coef5 = coef->coef5;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HFIRCOEF45.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HFIRCOEF67.u32));
        V0_HFIRCOEF67.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HFIRCOEF67.bits.coef6 = coef->coef6;
        V0_HFIRCOEF67.bits.coef7 = coef->coef7;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HFIRCOEF67.u32);
    }

    return HI_TRUE;
}

hi_bool hal_video_set_layer_disp_rect(hal_disp_layer layer, const hi_rect *rect)
{
    volatile U_V0_DFPOS V0_DFPOS;
    volatile U_V0_DLPOS V0_DLPOS;
    volatile U_G0_DFPOS G0_DFPOS;
    volatile U_G0_DLPOS G0_DLPOS;
    volatile hi_ulong addr_reg;

    switch (layer) {
        case HAL_DISP_LAYER_VHD0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_DFPOS.u32));
            V0_DFPOS.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_DFPOS.bits.disp_xfpos = rect->x;
            V0_DFPOS.bits.disp_yfpos = rect->y;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_DFPOS.u32);

            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_DLPOS.u32));
            V0_DLPOS.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_DLPOS.bits.disp_xlpos = rect->x + rect->width - 1;
            V0_DLPOS.bits.disp_ylpos = rect->y + rect->height - 1;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_DLPOS.u32);
            break;
        }

        case HAL_DISP_LAYER_GFX0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->G0_DFPOS));
            G0_DFPOS.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            G0_DFPOS.bits.disp_xfpos = rect->x;
            G0_DFPOS.bits.disp_yfpos = rect->y;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, G0_DFPOS.u32);

            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->G0_DLPOS));
            G0_DLPOS.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            G0_DLPOS.bits.disp_xlpos = rect->x + rect->width - 1;
            G0_DLPOS.bits.disp_ylpos = rect->y + rect->height - 1;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, G0_DLPOS.u32);
            break;
        }

        default: {
            printf("error layer id found in %s: L%d\n", __FUNCTION__, __LINE__);
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_video_set_layer_video_rect(hal_disp_layer layer, const hi_rect *rect)
{
    volatile U_V0_VFPOS V0_VFPOS;
    volatile U_V0_VLPOS V0_VLPOS;
    volatile U_G0_VFPOS G0_VFPOS;
    volatile U_G0_VLPOS G0_VLPOS;
    volatile hi_ulong addr_reg;

    switch (layer) {
        case HAL_DISP_LAYER_VHD0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_VFPOS.u32));
            V0_VFPOS.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_VFPOS.bits.video_xfpos = rect->x;
            V0_VFPOS.bits.video_yfpos = rect->y;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_VFPOS.u32);

            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_VLPOS.u32));
            V0_VLPOS.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_VLPOS.bits.video_xlpos = rect->x + rect->width - 1;
            V0_VLPOS.bits.video_ylpos = rect->y + rect->height - 1;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_VLPOS.u32);

            break;
        }

        case HAL_DISP_LAYER_GFX0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->G0_VFPOS));
            G0_VFPOS.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            G0_VFPOS.bits.video_xfpos = rect->x;
            G0_VFPOS.bits.video_yfpos = rect->y;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, G0_VFPOS.u32);

            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->G0_VLPOS));
            G0_VLPOS.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            G0_VLPOS.bits.video_xlpos = rect->x + rect->width - 1;
            G0_VLPOS.bits.video_ylpos = rect->y + rect->height - 1;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, G0_VLPOS.u32);
            break;
        }

        default: {
            printf("error layer id %d# found in %s,%s: L%d\n", layer, __FILE__, __FUNCTION__, __LINE__);
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_video_set_multi_area_l_addr(hal_disp_layer layer, hi_u32 area_num, hi_ulong l_addr, hi_u16 stride)
{
    hi_ulong pxaddr_addr;
    hi_ulong pxstride_addr;
    U_VID_STRIDE VID_STRIDE;

    if (layer == HAL_DISP_LAYER_VHD0) {
        pxaddr_addr = vo_get_vid_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->VID_ADDR_L));
        hal_write_reg((hi_u32*)(hi_uintptr_t)pxaddr_addr, get_low_addr(l_addr));

        pxaddr_addr = vo_get_vid_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->VID_ADDR_H));
        hal_write_reg((hi_u32*)(hi_uintptr_t)pxaddr_addr, get_high_addr(l_addr));

        pxstride_addr = vo_get_vid_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->VID_STRIDE.u32));
        VID_STRIDE.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)pxstride_addr);
        VID_STRIDE.bits.lm_stride = stride;
        hal_write_reg((hi_u32*)(hi_uintptr_t)pxstride_addr, VID_STRIDE.u32);
    } else {
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_video_set_multi_area_c_addr(hal_disp_layer layer, hi_u32 area_num, hi_ulong c_addr, hi_u16 stride)
{
    hi_ulong pxaddr_addr;
    hi_ulong pxstride_addr;
    U_VID_STRIDE VID_STRIDE;

    if (layer == HAL_DISP_LAYER_VHD0) {
        pxaddr_addr = vo_get_vid_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->VID_CADDR_L));
        hal_write_reg((hi_u32*)(hi_uintptr_t)pxaddr_addr, get_low_addr(c_addr));

        pxaddr_addr = vo_get_vid_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->VID_CADDR_H));
        hal_write_reg((hi_u32*)(hi_uintptr_t)pxaddr_addr, get_high_addr(c_addr));

        pxstride_addr = vo_get_vid_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->VID_STRIDE.u32));
        VID_STRIDE.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)pxstride_addr);
        VID_STRIDE.bits.chm_stride = stride;
        hal_write_reg((hi_u32*)(hi_uintptr_t)pxstride_addr, VID_STRIDE.u32);
    } else {
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_layer_enable_layer(hal_disp_layer layer, hi_u32 enable)
{
    volatile U_V0_CTRL V0_CTRL;
    volatile U_G0_CTRL G0_CTRL;
    volatile hi_ulong addr_reg;

    switch (layer) {
        case HAL_DISP_LAYER_VHD0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CTRL.u32));
            V0_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_CTRL.bits.surface_en = enable;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CTRL.u32);
            break;
        }

        case HAL_DISP_LAYER_GFX0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->G0_CTRL));
            G0_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            G0_CTRL.bits.surface_en = enable;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, G0_CTRL.u32);
            break;
        }

        default: {
            printf("error layer:%d id found in %s: L%d\n", layer, __FUNCTION__, __LINE__);
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_layer_set_layer_data_fmt(hal_disp_layer layer,
                                     hal_disp_pixel_format data_fmt)
{
    volatile U_VID_SRC_INFO VID_SRC_INFO;
    volatile U_GFX_SRC_INFO GFX_SRC_INFO;
    volatile hi_ulong addr_reg;

    if (layer == HAL_DISP_LAYER_VHD0) {
        addr_reg = vo_get_vid_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->VID_SRC_INFO.u32));
        VID_SRC_INFO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        VID_SRC_INFO.bits.data_type = data_fmt;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, VID_SRC_INFO.u32);
    } else if (layer == HAL_DISP_LAYER_GFX0) {
        addr_reg = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_SRC_INFO.u32));
        GFX_SRC_INFO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        GFX_SRC_INFO.bits.ifmt = data_fmt;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, GFX_SRC_INFO.u32);
    } else {
        printf("error layer id%d found in %s: L%d\n", layer, __FUNCTION__, __LINE__);
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_layer_set_layer_in_rect(hal_disp_layer layer, const hi_rect *rect)
{
    U_VID_IN_RESO VID_IN_RESO;
    U_GFX_IRESO GFX_IRESO;
    volatile hi_ulong addr_reg;

    if (layer == HAL_DISP_LAYER_VHD0) {
        addr_reg = vo_get_vid_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->VID_IN_RESO.u32));
        VID_IN_RESO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        VID_IN_RESO.bits.ireso_w = rect->width - 1;
        VID_IN_RESO.bits.ireso_h = rect->height - 1;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, VID_IN_RESO.u32);
    } else if (layer == HAL_DISP_LAYER_GFX0) {
        addr_reg = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_IRESO.u32));
        GFX_IRESO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        GFX_IRESO.bits.ireso_w = rect->width - 1;
        GFX_IRESO.bits.ireso_h = rect->height - 1;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, GFX_IRESO.u32);
    } else {
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_layer_set_layer_galpha(hal_disp_layer layer,
                                   hi_u8 alpha0)
{
    volatile U_V0_CTRL V0_CTRL;
    volatile hi_ulong addr_reg;

    switch (layer) {
        case HAL_DISP_LAYER_VHD0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_CTRL.u32));
            V0_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_CTRL.bits.galpha = alpha0;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_CTRL.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_layer_set_zme_info(hal_disp_layer layer, hi_u32 width, hi_u32 height,
                               hal_disp_zme_outfmt zme_out_fmt)
{
    volatile U_V0_ZME_HINFO V0_ZME_HINFO;
    volatile U_V0_ZME_VINFO V0_ZME_VINFO;
    volatile hi_ulong addr_reg;

    if (layer == HAL_DISP_LAYER_VHD0) {
        addr_reg = (hi_uintptr_t)&(g_vo_reg->V0_ZME_HINFO.u32);
        V0_ZME_HINFO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_ZME_HINFO.bits.out_width = width - 1;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_ZME_HINFO.u32);

        addr_reg = (hi_uintptr_t)&(g_vo_reg->V0_ZME_VINFO.u32);
        V0_ZME_VINFO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_ZME_VINFO.bits.out_pro = 1;
        V0_ZME_VINFO.bits.out_height = height - 1;
        V0_ZME_VINFO.bits.out_fmt = zme_out_fmt;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_ZME_VINFO.u32);
    } else {
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_layer_set_src_resolution(hal_disp_layer layer,
                                     const hi_rect *rect)
{
    U_VID_SRC_RESO VID_SRC_RESO;
    volatile hi_ulong addr_reg;

    if (layer == HAL_DISP_LAYER_VHD0) {
        addr_reg = vo_get_vid_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->VID_SRC_RESO.u32));
        VID_SRC_RESO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        VID_SRC_RESO.bits.src_w = rect->width - 1;
        VID_SRC_RESO.bits.src_h = rect->height - 1;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, VID_SRC_RESO.u32);
    }

    return HI_TRUE;
}

/* vou zoom enable */
hi_bool hal_layer_set_zme_enable(hal_disp_layer layer,
                                 hal_disp_zmemode mode,
                                 hi_u32 enable)
{
    volatile U_V0_ZME_HSP V0_ZME_HSP;
    volatile U_V0_ZME_VSP V0_ZME_VSP;

    volatile U_V1_CVFIR_VSP V1_CVFIR_VSP;

    volatile hi_ulong addr_reg;

    if (layer == HAL_DISP_LAYER_VHD0) {
        if ((mode == HAL_DISP_ZMEMODE_HORL) || (mode == HAL_DISP_ZMEMODE_HOR) || (mode == HAL_DISP_ZMEMODE_ALL)) {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_ZME_HSP.u32));
            V0_ZME_HSP.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_ZME_HSP.bits.lhfir_en = enable;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_ZME_HSP.u32);
        }

        if ((mode == HAL_DISP_ZMEMODE_HORC) || (mode == HAL_DISP_ZMEMODE_HOR) || (mode == HAL_DISP_ZMEMODE_ALL)) {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_ZME_HSP.u32));
            V0_ZME_HSP.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_ZME_HSP.bits.chfir_en = enable;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_ZME_HSP.u32);
        }

        if ((mode == HAL_DISP_ZMEMODE_VERL) || (mode == HAL_DISP_ZMEMODE_VER) || (mode == HAL_DISP_ZMEMODE_ALL)) {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_ZME_VSP.u32));
            V0_ZME_VSP.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_ZME_VSP.bits.lvfir_en = enable;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_ZME_VSP.u32);
        }

        if ((mode == HAL_DISP_ZMEMODE_VERC) || (mode == HAL_DISP_ZMEMODE_VER) || (mode == HAL_DISP_ZMEMODE_ALL)) {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_ZME_VSP.u32));
            V0_ZME_VSP.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_ZME_VSP.bits.cvfir_en = enable;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_ZME_VSP.u32);
        }
    } else if (layer == HAL_DISP_LAYER_VHD1) {
        if ((mode == HAL_DISP_ZMEMODE_VERL) || (mode == HAL_DISP_ZMEMODE_VER) || (mode == HAL_DISP_ZMEMODE_ALL)) {
            addr_reg = (hi_uintptr_t)&(g_vo_reg->V1_CVFIR_VSP.u32);
            V1_CVFIR_VSP.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V1_CVFIR_VSP.bits.cvfir_en = enable;
            V1_CVFIR_VSP.bits.cvmid_en = enable;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V1_CVFIR_VSP.u32);
        }
    } else {
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_dither_round_unlim(hal_disp_outputchannel chan, hi_u32 dither_round_unlim)
{
    volatile U_INTF0_DITHER_CTRL INTF0_DITHER_CTRL;
    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_CTRL.u32));
            INTF0_DITHER_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            INTF0_DITHER_CTRL.bits.dither_round_unlim = dither_round_unlim;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_CTRL.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_dither_data_in_out(hal_disp_outputchannel chan, hi_u32 i_data_width_dither,
                                        hi_u32 o_data_width_dither)
{
    volatile U_INTF0_DITHER_CTRL INTF0_DITHER_CTRL;
    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_CTRL.u32));
            INTF0_DITHER_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            INTF0_DITHER_CTRL.bits.i_data_width_dither = i_data_width_dither;
            INTF0_DITHER_CTRL.bits.o_data_width_dither = o_data_width_dither;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_CTRL.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_dither_en(hal_disp_outputchannel chan, hi_u32 dither_en)
{
    volatile U_INTF0_DITHER_CTRL INTF0_DITHER_CTRL;
    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_CTRL.u32));
            INTF0_DITHER_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            INTF0_DITHER_CTRL.bits.dither_en = dither_en;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_CTRL.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_dither_mode(hal_disp_outputchannel chan, hi_u32 dither_mode)
{
    volatile U_INTF0_DITHER_CTRL INTF0_DITHER_CTRL;
    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_CTRL.u32));
            INTF0_DITHER_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            INTF0_DITHER_CTRL.bits.dither_mode = dither_mode;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_CTRL.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_dither_round(hal_disp_outputchannel chan, hi_u32 dither_round)
{
    volatile U_INTF0_DITHER_CTRL INTF0_DITHER_CTRL;
    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_CTRL.u32));
            INTF0_DITHER_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            INTF0_DITHER_CTRL.bits.dither_round = dither_round;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_CTRL.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_dither_domain_mode(hal_disp_outputchannel chan, hi_u32 dither_domain_mode)
{
    volatile U_INTF0_DITHER_CTRL INTF0_DITHER_CTRL;
    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_CTRL.u32));
            INTF0_DITHER_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            INTF0_DITHER_CTRL.bits.dither_domain_mode = dither_domain_mode;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_CTRL.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_dither_tap_mode(hal_disp_outputchannel chan, hi_u32 dither_tap_mode)
{
    volatile U_INTF0_DITHER_CTRL INTF0_DITHER_CTRL;
    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_CTRL.u32));
            INTF0_DITHER_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            INTF0_DITHER_CTRL.bits.dither_tap_mode = dither_tap_mode;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_CTRL.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

static hi_void hal_disp_set_dither_sed0(hal_disp_outputchannel chan, const hal_disp_dihter_sed *dither_sed)
{
    volatile U_INTF0_DITHER_SED_Y0 INTF0_DITHER_SED_Y0;
    volatile U_INTF0_DITHER_SED_U0 INTF0_DITHER_SED_U0;
    volatile U_INTF0_DITHER_SED_V0 INTF0_DITHER_SED_V0;
    volatile U_INTF0_DITHER_SED_W0 INTF0_DITHER_SED_W0;

    volatile hi_ulong addr_reg;

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_Y0.u32));
    INTF0_DITHER_SED_Y0.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_Y0.bits.dither_sed_y0 = dither_sed->dither_sed_y0;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_Y0.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_U0.u32));
    INTF0_DITHER_SED_U0.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_U0.bits.dither_sed_u0 = dither_sed->dither_sed_u0;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_U0.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_V0.u32));
    INTF0_DITHER_SED_V0.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_V0.bits.dither_sed_v0 = dither_sed->dither_sed_v0;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_V0.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_W0.u32));
    INTF0_DITHER_SED_W0.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_W0.bits.dither_sed_w0 = dither_sed->dither_sed_w0;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_W0.u32);

    return;
}

static hi_void hal_disp_set_dither_sed1(hal_disp_outputchannel chan, const hal_disp_dihter_sed *dither_sed)
{
    volatile U_INTF0_DITHER_SED_Y1 INTF0_DITHER_SED_Y1;
    volatile U_INTF0_DITHER_SED_U1 INTF0_DITHER_SED_U1;
    volatile U_INTF0_DITHER_SED_V1 INTF0_DITHER_SED_V1;
    volatile U_INTF0_DITHER_SED_W1 INTF0_DITHER_SED_W1;

    volatile hi_ulong addr_reg;

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_Y1.u32));
    INTF0_DITHER_SED_Y1.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_Y1.bits.dither_sed_y1 = dither_sed->dither_sed_y1;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_Y1.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_U1.u32));
    INTF0_DITHER_SED_U1.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_U1.bits.dither_sed_u1 = dither_sed->dither_sed_u1;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_U1.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_V1.u32));
    INTF0_DITHER_SED_V1.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_V1.bits.dither_sed_v1 = dither_sed->dither_sed_v1;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_V1.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_W1.u32));
    INTF0_DITHER_SED_W1.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_W1.bits.dither_sed_w1 = dither_sed->dither_sed_w1;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_W1.u32);

    return ;
}

static hi_void hal_disp_set_dither_sed2(hal_disp_outputchannel chan, const hal_disp_dihter_sed *dither_sed)
{
    volatile U_INTF0_DITHER_SED_Y2 INTF0_DITHER_SED_Y2;
    volatile U_INTF0_DITHER_SED_U2 INTF0_DITHER_SED_U2;
    volatile U_INTF0_DITHER_SED_V2 INTF0_DITHER_SED_V2;
    volatile U_INTF0_DITHER_SED_W2 INTF0_DITHER_SED_W2;

    volatile hi_ulong addr_reg;

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_Y2.u32));
    INTF0_DITHER_SED_Y2.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_Y2.bits.dither_sed_y2 = dither_sed->dither_sed_y2;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_Y2.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_U2.u32));
    INTF0_DITHER_SED_U2.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_U2.bits.dither_sed_u2 = dither_sed->dither_sed_u2;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_U2.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_V2.u32));
    INTF0_DITHER_SED_V2.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_V2.bits.dither_sed_v2 = dither_sed->dither_sed_v2;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_V2.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_W2.u32));
    INTF0_DITHER_SED_W2.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_W2.bits.dither_sed_w2 = dither_sed->dither_sed_w2;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_W2.u32);

    return;
}

static hi_void hal_disp_set_dither_sed3(hal_disp_outputchannel chan, const hal_disp_dihter_sed *dither_sed)
{
    volatile U_INTF0_DITHER_SED_Y3 INTF0_DITHER_SED_Y3;
    volatile U_INTF0_DITHER_SED_U3 INTF0_DITHER_SED_U3;
    volatile U_INTF0_DITHER_SED_V3 INTF0_DITHER_SED_V3;
    volatile U_INTF0_DITHER_SED_W3 INTF0_DITHER_SED_W3;
    volatile hi_ulong addr_reg;

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_Y3.u32));
    INTF0_DITHER_SED_Y3.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_Y3.bits.dither_sed_y3 = dither_sed->dither_sed_y3;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_Y3.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_U3.u32));
    INTF0_DITHER_SED_U3.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_U3.bits.dither_sed_u3 = dither_sed->dither_sed_u3;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_U3.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_V3.u32));
    INTF0_DITHER_SED_V3.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_V3.bits.dither_sed_v3 = dither_sed->dither_sed_v3;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_V3.u32);

    addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_SED_W3.u32));
    INTF0_DITHER_SED_W3.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    INTF0_DITHER_SED_W3.bits.dither_sed_w3 = dither_sed->dither_sed_w3;
    hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_SED_W3.u32);

    return;
}

hi_bool hal_disp_set_dither_sed(hal_disp_outputchannel chan, const hal_disp_dihter_sed *dither_sed)
{
    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            /* set sed_y0,sed_u0,sed_v0,sed_w0 */
            hal_disp_set_dither_sed0(chan, dither_sed);

            /* set sed_y1,sed_u1,sed_v1,sed_w1 */
            hal_disp_set_dither_sed1(chan, dither_sed);

            /* set sed_y2,sed_u2,sed_v2,sed_w2 */
            hal_disp_set_dither_sed2(chan, dither_sed);

            /* set sed_y3,sed_u3,sed_v3,sed_w3 */
            hal_disp_set_dither_sed3(chan, dither_sed);

            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_disp_set_dither_thr_min_max(hal_disp_outputchannel chan, hi_u32 thr_min, hi_u32 thr_max)
{
    volatile U_INTF0_DITHER_THR INTF0_DITHER_THR;
    volatile hi_ulong addr_reg;

    switch (chan) {
        case HAL_DISP_CHANNEL_DHD0: {
            addr_reg = vo_get_chn_abs_addr(chan, (hi_uintptr_t)&(g_vo_reg->INTF0_DITHER_THR.u32));
            INTF0_DITHER_THR.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            INTF0_DITHER_THR.bits.dither_thr_min = thr_min;
            INTF0_DITHER_THR.bits.dither_thr_max = thr_max;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, INTF0_DITHER_THR.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_layer_set_reg_up(hal_disp_layer layer)
{
    U_V0_UPD V0_UPD;
    U_G0_UPD G0_UPD;
    volatile hi_ulong addr_reg;

    switch (layer) {
        case HAL_DISP_LAYER_VHD0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_UPD.u32));
            V0_UPD.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            /* video layer register update */
            V0_UPD.bits.regup = 0x1;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, V0_UPD.u32);
            break;
        }

        case HAL_DISP_LAYER_GFX0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->G0_UPD));
            G0_UPD.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            /* graphic layer register update */
            G0_UPD.bits.regup = 0x1;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, G0_UPD.u32);
            break;
        }

        default: {
            printf("error layer id found in %s: L%d\n", __FUNCTION__, __LINE__);
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_video_set_layer_alpha(hal_disp_layer layer, hi_u32 arange)
{
    volatile U_V0_ALPHA V0_ALPHA;
    volatile hi_ulong addr_reg;

    switch (layer) {
        case HAL_DISP_LAYER_VHD0: {
            addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_ALPHA.u32));
            V0_ALPHA.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            V0_ALPHA.bits.vbk_alpha = arange;
            hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_ALPHA.u32);
            break;
        }

        default: {
            return HI_FALSE;
        }
    }

    return HI_TRUE;
}

hi_bool hal_cbm_set_cbm_bkg(hi_u32 mixer_id, const hal_disp_bkcolor *bkg)
{
    U_CBM_BKG1 CBM_BKG1;

    if (mixer_id == HAL_CBMMIX1) {
        CBM_BKG1.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)&(g_vo_reg->CBM_BKG1.u32));
        CBM_BKG1.bits.cbm_bkgy1 = (bkg->bkg_y);
        CBM_BKG1.bits.cbm_bkgcb1 = (bkg->bkg_cb);
        CBM_BKG1.bits.cbm_bkgcr1 = (bkg->bkg_cr);
        hal_write_reg((hi_u32*)(hi_uintptr_t)&(g_vo_reg->CBM_BKG1.u32), CBM_BKG1.u32);
    } else {
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_cbm_set_cbm_mixer_prio(hal_disp_layer layer, hi_u8 prio, hi_u8 mixer_id)
{
    U_CBM_MIX1 CBM_MIX1;
    hi_u8 layer_id = 0;

    if (mixer_id == HAL_CBMMIX1) {
        switch (layer) {
            case HAL_DISP_LAYER_VHD0: {
                layer_id = 0x1; /* 0x1 register config */
                break;
            }

            case HAL_DISP_LAYER_GFX0: {
                layer_id = 0x2; /* 0x2 register config */
                break;
            }

            default: {
                return HI_FALSE;
            }
        }

        CBM_MIX1.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)&(g_vo_reg->CBM_MIX1.u32));

        switch (prio) {
            case 0: { /* prio 0 */
                CBM_MIX1.bits.mixer_prio0 = layer_id;
                break;
            }

            case 1: { /* prio 1 */
                CBM_MIX1.bits.mixer_prio1 = layer_id;
                break;
            }

            case 2: { /* prio 2 */
                CBM_MIX1.bits.mixer_prio2 = layer_id;
                break;
            }

            default: {
                return HI_FALSE;
            }
        }

        hal_write_reg((hi_u32*)(hi_uintptr_t)&(g_vo_reg->CBM_MIX1.u32), CBM_MIX1.u32);
    } else {
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_graphic_set_gfx_ext(hal_disp_layer layer,
                                hal_gfx_bitextend mode)
{
    U_GFX_OUT_CTRL GFX_OUT_CTRL;

    volatile hi_ulong addr_reg;

    if (layer == HAL_DISP_LAYER_GFX0) {
        addr_reg = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_OUT_CTRL.u32));
        GFX_OUT_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        GFX_OUT_CTRL.bits.bitext = mode;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, GFX_OUT_CTRL.u32);
    } else {
        printf("error layer id found in %s: L%d\n", __FUNCTION__, __LINE__);
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_graphic_set_gfx_palpha(hal_disp_layer layer,
                                   hi_u32 alpha_en, hi_u32 arange,
                                   hi_u8 alpha0, hi_u8 alpha1)
{
    U_GFX_OUT_CTRL GFX_OUT_CTRL;

    U_GFX_1555_ALPHA GFX_1555_ALPHA;

    volatile hi_ulong addr_reg;

    if (layer == HAL_DISP_LAYER_GFX0) {
        addr_reg = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_OUT_CTRL.u32));
        GFX_OUT_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        GFX_OUT_CTRL.bits.palpha_en = alpha_en;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, GFX_OUT_CTRL.u32);

        if (alpha_en == HI_TRUE) {
            addr_reg = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_1555_ALPHA.u32));
            GFX_1555_ALPHA.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            GFX_1555_ALPHA.bits.alpha_1 = alpha1;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, GFX_1555_ALPHA.u32);

            addr_reg = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_1555_ALPHA.u32));
            GFX_1555_ALPHA.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            GFX_1555_ALPHA.bits.alpha_0 = alpha0;
            hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, GFX_1555_ALPHA.u32);
        } else {
            addr_reg = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_1555_ALPHA.u32));
            GFX_1555_ALPHA.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            GFX_1555_ALPHA.bits.alpha_1 = 0xff; /* 0xff max alpha */
            hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, GFX_1555_ALPHA.u32);

            addr_reg = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_1555_ALPHA.u32));
            GFX_1555_ALPHA.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
            GFX_1555_ALPHA.bits.alpha_0 = 0xff; /* 0xff max alpha */
            hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, GFX_1555_ALPHA.u32);
        }
    } else {
        printf("error layer id found in %s: L%d\n", __FUNCTION__, __LINE__);
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_graphic_set_gfx_pre_mult(hal_disp_layer layer, hi_u32 enable)
{
    U_GFX_OUT_CTRL GFX_OUT_CTRL;

    volatile hi_ulong addr_reg;

    if (layer == HAL_DISP_LAYER_GFX0) {
        addr_reg = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_OUT_CTRL.u32));
        GFX_OUT_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        GFX_OUT_CTRL.bits.premulti_en = enable;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, GFX_OUT_CTRL.u32);
    } else {
        printf("error layer id found in %s: L%d\n", __FUNCTION__, __LINE__);
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_graphic_set_gfx_addr(hal_disp_layer layer, hi_u64 l_addr)
{
    volatile hi_ulong gfx_addr_h;
    volatile hi_ulong gfx_addr_l;

    if (layer == HAL_DISP_LAYER_GFX0) {
        gfx_addr_l = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_ADDR_L));
        hal_write_reg((hi_u32*)(hi_uintptr_t)gfx_addr_l, get_low_addr(l_addr));
        gfx_addr_h = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_ADDR_H));
        hal_write_reg((hi_u32*)(hi_uintptr_t)gfx_addr_h, get_high_addr(l_addr));
    } else {
        printf("error layer id found in %s: L%d\n", __FUNCTION__, __LINE__);
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_graphic_set_gfx_stride(hal_disp_layer layer, hi_u16 pitch)
{
    volatile U_GFX_STRIDE GFX_STRIDE;
    volatile hi_ulong addr_reg;

    if (layer == HAL_DISP_LAYER_GFX0) {
        addr_reg = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_STRIDE.u32));
        GFX_STRIDE.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        GFX_STRIDE.bits.surface_stride = pitch;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, GFX_STRIDE.u32);
    } else {
        printf("error layer id found in %s: L%d\n", __FUNCTION__, __LINE__);
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_bool hal_gfx_set_src_resolution(hal_disp_layer layer, const hi_rect *rect)
{
    U_GFX_SRC_RESO GFX_SRC_RESO;
    volatile hi_ulong addr_reg;

    if (layer == HAL_DISP_LAYER_GFX0) {
        addr_reg = vo_get_gfx_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->GFX_SRC_RESO.u32));
        GFX_SRC_RESO.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        GFX_SRC_RESO.bits.src_w = rect->width - 1;
        GFX_SRC_RESO.bits.src_h = rect->height - 1;
        hal_write_reg((hi_u32*)(hi_uintptr_t)addr_reg, GFX_SRC_RESO.u32);
    } else {
        printf("error:layer id not found in %s: L%d\n", __FUNCTION__, __LINE__);
        return HI_FALSE;
    }

    return HI_TRUE;
}

hi_void hal_layer_csc_set_dc_coef(hal_disp_layer layer, const vdp_csc_dc_coef *csc_dc_coef)
{
    U_V0_HIPP_CSC_IDC0 V0_HIPP_CSC_IDC0;
    U_V0_HIPP_CSC_IDC1 V0_HIPP_CSC_IDC1;
    U_V0_HIPP_CSC_IDC2 V0_HIPP_CSC_IDC2;
    U_V0_HIPP_CSC_ODC0 V0_HIPP_CSC_ODC0;
    U_V0_HIPP_CSC_ODC1 V0_HIPP_CSC_ODC1;
    U_V0_HIPP_CSC_ODC2 V0_HIPP_CSC_ODC2;

    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_IDC0.u32));
        V0_HIPP_CSC_IDC0.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_IDC0.bits.hipp_csc_idc0 = csc_dc_coef->csc_in_dc0;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_IDC0.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_IDC1.u32));
        V0_HIPP_CSC_IDC1.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_IDC1.bits.hipp_csc_idc1 = csc_dc_coef->csc_in_dc1;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_IDC1.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_IDC2.u32));
        V0_HIPP_CSC_IDC2.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_IDC2.bits.hipp_csc_idc2 = csc_dc_coef->csc_in_dc2;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_IDC2.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_ODC0.u32));
        V0_HIPP_CSC_ODC0.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_ODC0.bits.hipp_csc_odc0 = csc_dc_coef->csc_out_dc0;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_ODC0.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_ODC1.u32));
        V0_HIPP_CSC_ODC1.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_ODC1.bits.hipp_csc_odc1 = csc_dc_coef->csc_out_dc1;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_ODC1.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_ODC2.u32));
        V0_HIPP_CSC_ODC2.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_ODC2.bits.hipp_csc_odc2 = csc_dc_coef->csc_out_dc2;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_ODC2.u32);
    }
}

hi_void hal_layer_csc_set_csc_mode(hal_disp_layer layer, hi_s32 csc_mode)
{
    U_V0_HIPP_CSC_CTRL V0_HIPP_CSC_CTRL;
    volatile hi_ulong addr_reg;
    if (layer == HAL_DISP_LAYER_GFX0) {
        switch (csc_mode) {
            case HAL_CSC_MODE_RGB_TO_BT601_PC: {
                addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->G0_HIPP_CSC_CTRL.u32));
                V0_HIPP_CSC_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
                V0_HIPP_CSC_CTRL.bits.hipp_csc_mode = 0x0;
                hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_CTRL.u32);
                break;
            }
            case HAL_CSC_MODE_RGB_TO_BT709_PC: {
                addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->G0_HIPP_CSC_CTRL.u32));
                V0_HIPP_CSC_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
                V0_HIPP_CSC_CTRL.bits.hipp_csc_mode = 0x1;
                hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_CTRL.u32);
                break;
            }
            default: {
                break;
            }
        }
    }

    return;
}

hi_void hal_layer_csc_set_param(hal_disp_layer layer, const csc_coef_param *coef_param)
{
    U_V0_HIPP_CSC_SCALE V0_HIPP_CSC_SCALE;
    U_V0_HIPP_CSC_MIN_Y V0_HIPP_CSC_MIN_Y;
    U_V0_HIPP_CSC_MIN_C V0_HIPP_CSC_MIN_C;
    U_V0_HIPP_CSC_MAX_Y V0_HIPP_CSC_MAX_Y;
    U_V0_HIPP_CSC_MAX_C V0_HIPP_CSC_MAX_C;

    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_SCALE.u32));
        V0_HIPP_CSC_SCALE.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_SCALE.bits.hipp_csc_scale = coef_param->csc_scale2p;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_SCALE.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_MIN_Y.u32));
        V0_HIPP_CSC_MIN_Y.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_MIN_Y.bits.hipp_csc_min_y = coef_param->csc_clip_min;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_MIN_Y.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_MIN_C.u32));
        V0_HIPP_CSC_MIN_C.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_MIN_C.bits.hipp_csc_min_c = coef_param->csc_clip_min;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_MIN_C.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_MAX_Y.u32));
        V0_HIPP_CSC_MAX_Y.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_MAX_Y.bits.hipp_csc_max_y = coef_param->csc_clip_max;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_MAX_Y.u32);

        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_MAX_C.u32));
        V0_HIPP_CSC_MAX_C.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_MAX_C.bits.hipp_csc_max_c = coef_param->csc_clip_max;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_MAX_C.u32);
    }
}

static hi_void hal_layer_csc_set_video_coef0x(hal_disp_layer layer, const vdp_csc_coef *csc_coef)
{
    U_V0_HIPP_CSC_COEF00 V0_HIPP_CSC_COEF00;
    U_V0_HIPP_CSC_COEF01 V0_HIPP_CSC_COEF01;
    U_V0_HIPP_CSC_COEF02 V0_HIPP_CSC_COEF02;

    volatile hi_u32 addr_reg;

    addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_COEF00.u32));
    V0_HIPP_CSC_COEF00.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    V0_HIPP_CSC_COEF00.bits.hipp_csc_coef00 = csc_coef->csc_coef00;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_COEF00.u32);

    addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_COEF01.u32));
    V0_HIPP_CSC_COEF01.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    V0_HIPP_CSC_COEF01.bits.hipp_csc_coef01 = csc_coef->csc_coef01;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_COEF01.u32);

    addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_COEF02.u32));
    V0_HIPP_CSC_COEF02.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    V0_HIPP_CSC_COEF02.bits.hipp_csc_coef02 = csc_coef->csc_coef02;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_COEF02.u32);

    return;
}

static hi_void hal_layer_csc_set_video_coef1x(hal_disp_layer layer, const vdp_csc_coef *csc_coef)
{
    U_V0_HIPP_CSC_COEF10 V0_HIPP_CSC_COEF10;
    U_V0_HIPP_CSC_COEF11 V0_HIPP_CSC_COEF11;
    U_V0_HIPP_CSC_COEF12 V0_HIPP_CSC_COEF12;

    volatile hi_u32 addr_reg;

    addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_COEF10.u32));
    V0_HIPP_CSC_COEF10.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    V0_HIPP_CSC_COEF10.bits.hipp_csc_coef10 = csc_coef->csc_coef10;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_COEF10.u32);

    addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_COEF11.u32));
    V0_HIPP_CSC_COEF11.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    V0_HIPP_CSC_COEF11.bits.hipp_csc_coef11 = csc_coef->csc_coef11;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_COEF11.u32);

    addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_COEF12.u32));
    V0_HIPP_CSC_COEF12.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    V0_HIPP_CSC_COEF12.bits.hipp_csc_coef12 = csc_coef->csc_coef12;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_COEF12.u32);

    return;
}

static hi_void hal_layer_csc_set_video_coef2x(hal_disp_layer layer, const vdp_csc_coef *csc_coef)
{
    U_V0_HIPP_CSC_COEF20 V0_HIPP_CSC_COEF20;
    U_V0_HIPP_CSC_COEF21 V0_HIPP_CSC_COEF21;
    U_V0_HIPP_CSC_COEF22 V0_HIPP_CSC_COEF22;

    volatile hi_u32 addr_reg;

    addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_COEF20.u32));
    V0_HIPP_CSC_COEF20.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    V0_HIPP_CSC_COEF20.bits.hipp_csc_coef20 = csc_coef->csc_coef20;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_COEF20.u32);

    addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_COEF21.u32));
    V0_HIPP_CSC_COEF21.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    V0_HIPP_CSC_COEF21.bits.hipp_csc_coef21 = csc_coef->csc_coef21;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_COEF21.u32);

    addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_COEF22.u32));
    V0_HIPP_CSC_COEF22.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
    V0_HIPP_CSC_COEF22.bits.hipp_csc_coef22 = csc_coef->csc_coef22;
    hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_COEF22.u32);

    return;
}

hi_void hal_layer_csc_set_coef(hal_disp_layer layer, const vdp_csc_coef *csc_coef)
{
    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        hal_layer_csc_set_video_coef0x(layer, csc_coef);
        hal_layer_csc_set_video_coef1x(layer, csc_coef);
        hal_layer_csc_set_video_coef2x(layer, csc_coef);
    }

    return;
}

hi_bool hal_layer_set_csc_coef(hal_disp_layer layer, const vo_csc_coef *csc_coef)
{
    hal_layer_csc_set_dc_coef(layer, (vdp_csc_dc_coef *)(&csc_coef->csc_in_dc0));
    hal_layer_csc_set_coef(layer, (vdp_csc_coef *)(&csc_coef->csc_coef00));
    hal_layer_csc_set_param(layer, (csc_coef_param *)(&csc_coef->new_csc_scale2p));

    return HI_TRUE;
}

hi_void hal_layer_csc_set_ck_gt_en(hal_disp_layer layer, hi_bool ck_gt_en)
{
    U_V0_HIPP_CSC_CTRL V0_HIPP_CSC_CTRL;

    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_CTRL.u32));
        V0_HIPP_CSC_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_CTRL.bits.hipp_csc_ck_gt_en = ck_gt_en;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_CTRL.u32);
    } else if ((layer >= LAYER_GFX_START) && (layer <= LAYER_GFX_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->G0_HIPP_CSC_CTRL.u32));
        V0_HIPP_CSC_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_CTRL.bits.hipp_csc_ck_gt_en = ck_gt_en;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_CTRL.u32);
    }
}

hi_void hal_layer_csc_set_enable(hal_disp_layer layer, hi_bool csc_en)
{
    U_V0_HIPP_CSC_CTRL V0_HIPP_CSC_CTRL;

    volatile hi_u32 addr_reg;

    if ((layer >= LAYER_VHD_START) && (layer <= LAYER_VHD_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->V0_HIPP_CSC_CTRL.u32));
        V0_HIPP_CSC_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_CTRL.bits.hipp_csc_en = csc_en;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_CTRL.u32);
    } else if ((layer >= LAYER_GFX_START) && (layer <= LAYER_GFX_END)) {
        addr_reg = vo_get_abs_addr(layer, (hi_uintptr_t)&(g_vo_reg->G0_HIPP_CSC_CTRL.u32));
        V0_HIPP_CSC_CTRL.u32 = hal_read_reg((hi_u32*)(hi_uintptr_t)addr_reg);
        V0_HIPP_CSC_CTRL.bits.hipp_csc_en = csc_en;
        hal_write_reg ((hi_u32*)(hi_uintptr_t)addr_reg, V0_HIPP_CSC_CTRL.u32);
    }
}

hi_bool hal_layer_set_csc_en(hal_disp_layer layer, hi_bool csc_en)
{
    if ((layer != HAL_DISP_LAYER_VHD0) && (layer != HAL_DISP_LAYER_GFX0)) {
        printf("error, wrong layer ID!%d\n", __LINE__);
        return HI_FALSE;
    }

    hal_layer_csc_set_ck_gt_en(layer, csc_en);
    hal_layer_csc_set_enable(layer, csc_en);

    return HI_TRUE;
}

