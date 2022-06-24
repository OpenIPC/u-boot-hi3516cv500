/*
 * Copyright (c) 2018 HiSilicon Technologies Co., Ltd.
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

#include "hi_type.h"
#include "hdmi_reg_crg.h"
#include "hdmi_product_define.h"

volatile hdmi_reg_crg *g_crg_regs = NULL;

int hdmi_reg_crg_init(void)
{
    if (g_crg_regs != NULL) {
        return HI_SUCCESS;
    }

    g_crg_regs = (volatile hdmi_reg_crg *)(hi_uintptr_t)HDMI_IO_MAP(HDMI_CRG_ADDR, sizeof(hdmi_reg_crg));
    if (g_crg_regs == NULL) {
        hdmi_err("ioremap_nocache gophdmi2tx_dphy_regAllReg failed!!!!!!!\n");
        return HI_FAILURE;
    }

    return HI_SUCCESS;
}

int hdmi_reg_crg_deinit(void)
{
    if (g_crg_regs != HI_NULL) {
        HDMI_IO_UNMAP(g_crg_regs);
        g_crg_regs = HI_NULL;
    }

    return HI_SUCCESS;
}

int hdmi_reg_ssc_in_cken_set(unsigned int ssc_in_cken)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.ssc_in_cken = ssc_in_cken;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int hdmi_reg_ssc_bypass_cken_set(unsigned int ssc_bypass_cken)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.ssc_bypass_cken = ssc_bypass_cken;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int hdmi_reg_ctrl_osc_24m_cken_set(unsigned int hdmitx_ctrl_osc_24m_cken)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.hdmitx_ctrl_osc_24m_cken = hdmitx_ctrl_osc_24m_cken;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int hdmi_reg_ctrl_cec_cken_set(unsigned int hdmitx_ctrl_cec_cken)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.hdmitx_ctrl_cec_cken = hdmitx_ctrl_cec_cken;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int hdmi_reg_ctrl_os_cken_set(unsigned int hdmitx_ctrl_os_cken)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.hdmitx_ctrl_os_cken = hdmitx_ctrl_os_cken;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int hdmi_reg_ctrl_as_cken_set(unsigned int hdmitx_ctrl_as_cken)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.hdmitx_ctrl_as_cken = hdmitx_ctrl_as_cken;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}
int hdmi_reg_ctrl_bus_srst_req_set(unsigned int hdmitx_ctrl_bus_srst_req)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.hdmitx_ctrl_bus_srst_req = hdmitx_ctrl_bus_srst_req;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int hdmi_reg_ctrl_srst_req_set(unsigned int hdmitx_ctrl_srst_req)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.hdmitx_ctrl_srst_req = hdmitx_ctrl_srst_req;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int hdmi_reg_cec_srst_req_set(unsigned int hdmitx_ctrl_cec_srst_req)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.hdmitx_ctrl_cec_srst_req = hdmitx_ctrl_cec_srst_req;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int hdmi_reg_ssc_srst_req_set(unsigned int hdmitx_ssc_srst_req)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.hdmitx_ssc_srst_req = hdmitx_ssc_srst_req;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int hdmi_reg_ssc_clk_div_set(unsigned int ssc_clk_div)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.ssc_clk_div = ssc_clk_div;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int hdmi_reg_pxl_cken_set(unsigned int hdmitx_pxl_cken)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.hdmitx_pxl_cken = hdmitx_pxl_cken;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int reg_hdmi_crg_ssc_bypass_clk_sel_set(unsigned int ssc_bypass_clk_sel)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8144 peri_crg_8144;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8144.u32);
    peri_crg_8144.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8144.bits.ssc_bypass_clk_sel = ssc_bypass_clk_sel;
    hdmi_tx_reg_write(reg_addr, peri_crg_8144.u32);

    return HI_SUCCESS;
}

int hdmi_reg_hdmirx_phy_tmds_cken_set(unsigned int phy_tmds_cken)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8152 peri_crg_8152;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8152.u32);
    peri_crg_8152.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8152.bits.phy_tmds_cken = phy_tmds_cken;
    hdmi_tx_reg_write(reg_addr, peri_crg_8152.u32);

    return HI_SUCCESS;
}

int hdmi_reg_phy_srst_req_set(unsigned int hdmitx_phy_srst_req)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8152 peri_crg_8152;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8152.u32);
    peri_crg_8152.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8152.bits.hdmitx_phy_srst_req = hdmitx_phy_srst_req;
    hdmi_tx_reg_write(reg_addr, peri_crg_8152.u32);

    return HI_SUCCESS;
}

int hdmi_reg_phy_srst_req_get(hi_void)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8152 peri_crg_8152;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8152.u32);
    peri_crg_8152.u32 = hdmi_tx_reg_read(reg_addr);

    return peri_crg_8152.bits.hdmitx_phy_srst_req;
}

int hdmi_reg_phy_tmds_srst_req_set(unsigned int phy_tmds_srst_req)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8152 peri_crg_8152;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8152.u32);
    peri_crg_8152.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8152.bits.phy_tmds_srst_req = phy_tmds_srst_req;
    hdmi_tx_reg_write(reg_addr, peri_crg_8152.u32);

    return HI_SUCCESS;
}

int hdmi_reg_phy_tmds_srst_req_get(hi_void)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8152 peri_crg_8152;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8152.u32);
    peri_crg_8152.u32 = hdmi_tx_reg_read(reg_addr);

    return peri_crg_8152.bits.phy_tmds_srst_req;
}

int hdmi_reg_tmds_clk_div_set(unsigned int tmds_clk_div)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8152 peri_crg_8152;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8152.u32);
    peri_crg_8152.u32 = hdmi_tx_reg_read(reg_addr);
    peri_crg_8152.bits.tmds_clk_div = tmds_clk_div;
    hdmi_tx_reg_write(reg_addr, peri_crg_8152.u32);

    return HI_SUCCESS;
}

int reg_hdmi_crg_tmds_clk_div_get(hi_void)
{
    hi_u32 *reg_addr = NULL;
    peri_crg8152 peri_crg_8152;

    reg_addr = (hi_u32 *)&(g_crg_regs->crg8152.u32);
    peri_crg_8152.u32 = hdmi_tx_reg_read(reg_addr);

    return peri_crg_8152.bits.tmds_clk_div;
}

