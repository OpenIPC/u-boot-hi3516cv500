/*
 * hi3556v200.c
 *
 * The board init for hisilicon
 *
 * Copyright (c) 2015 HiSilicon Technologies Co., Ltd.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#include "../../../lib/hw_dec/hw_decompress_v1.h"
#include <config.h>
#include <common.h>
#include <asm/io.h>
#include <asm/arch/platform.h>
#include <spi_flash.h>
#include <linux/mtd/mtd.h>
#include <nand.h>
#include <netdev.h>
#include <mmc.h>
#include <asm/sections.h>
#include <hicpu_common.h>

#ifndef CONFIG_SYS_DCACHE_OFF
void enable_caches(void)
{
	/* Enable D-cache. I-cache is already enabled in start.S */
	dcache_enable();
}
#endif
static int boot_media = BOOT_MEDIA_UNKNOWN;

int get_boot_media(void)
{
	unsigned int reg_val, boot_mode, spi_device_mode;
	int boot_media = BOOT_MEDIA_UNKNOWN;

	reg_val = readl(SYS_CTRL_REG_BASE + REG_SYSSTAT);
	boot_mode = get_sys_boot_mode(reg_val);

	switch (boot_mode) {
	case BOOT_FROM_SPI:
		spi_device_mode = get_spi_device_type(reg_val);
		if (spi_device_mode)
			boot_media = BOOT_MEDIA_NAND;
		else
			boot_media = BOOT_MEDIA_SPIFLASH;
		break;
	case BOOT_FROM_EMMC:
		boot_media = BOOT_MEDIA_EMMC;
		break;
	default:
		boot_media = BOOT_MEDIA_UNKNOWN;
		break;
	}
	return boot_media;
}

#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
	printf("Boot reached stage %d\n", progress);
}
#endif

#define COMP_MODE_ENABLE ((unsigned int)0x0000EAEF)

static inline void delay(unsigned long loops)
{
	__asm__ volatile("1:\n"
			"subs %0, %1, #1\n"
			"bne 1b" : "=r"(loops) : "0"(loops));
}


int get_text_base(void)
{
	return CONFIG_SYS_TEXT_BASE;
}

static void boot_flag_init(void)
{
	unsigned int reg, boot_mode, spi_device_mode;

	/* get boot mode */
	reg = __raw_readl(SYS_CTRL_REG_BASE + REG_SYSSTAT);
	boot_mode = get_sys_boot_mode(reg);

	switch (boot_mode) {
	case BOOT_FROM_SPI:
		spi_device_mode = get_spi_device_type(reg);
		if (spi_device_mode)
			boot_media = BOOT_MEDIA_NAND;
		else
			boot_media = BOOT_MEDIA_SPIFLASH;
		break;
	case BOOT_FROM_EMMC:    /* emmc mode */
		boot_media = BOOT_MEDIA_EMMC;
		break;
	default:
		boot_media = BOOT_MEDIA_UNKNOWN;
		break;
	}
}

int board_early_init_f(void)
{
	return 0;
}

#if (CONFIG_AUTO_UPDATE == 1)

#define UBOOT_DATA_ADDR     (0x81000000)
#define UBOOT_DATA_SIZE     (0x80000)
int data_to_spiflash(void)
{
	static struct spi_flash *flash = NULL;
	void *buf = NULL;
	unsigned int val;

	/* 0:bus  0:cs  1000000:max_hz  0x3:spi_mode */
	flash = spi_flash_probe(0, 0, 1000000, 0x3);
	if (!flash) {
		printf("Failed to initialize SPI flash\n");
		return -1;  /* -1:failed */
	}

	/* erase the address range. */
	printf("Spi flash erase...\n");
	val = flash->erase(flash, NUM_0, UBOOT_DATA_SIZE);
	if (val) {
		printf("SPI flash sector erase failed\n");
		return 1; /* 1:failed */
	}

	buf = map_physmem((unsigned long)UBOOT_DATA_ADDR,
			UBOOT_DATA_SIZE, MAP_WRBACK);
	if (!buf) {
		puts("Failed to map physical memory\n");
		return 1; /* 1:failed */
	}

	/* copy the data from RAM to FLASH */
	printf("Spi flash write...\n");
	val = flash->write(flash, NUM_0, UBOOT_DATA_SIZE, buf);
	if (val) {
		printf("SPI flash write failed, return %u\n",
				val);
		unmap_physmem(buf, UBOOT_DATA_SIZE);
		return 1; /* 1:failed */
	}

	unmap_physmem(buf, UBOOT_DATA_SIZE);
	return 0; /* 0:success */
}

int data_to_nandflash(void)
{
	struct mtd_info *nand_flash = NULL;
	void *buf = NULL;
	size_t length = UBOOT_DATA_SIZE;
	unsigned int val;

	nand_flash = nand_info[0];

	printf("Nand flash erase...\n");
	val = nand_erase(nand_flash, 0, UBOOT_DATA_SIZE);
	if (val) {
		printf("Nand flash erase failed\n");
		return 1;
	}

	buf = map_physmem((unsigned long)UBOOT_DATA_ADDR,
			UBOOT_DATA_SIZE, MAP_WRBACK);
	if (!buf) {
		puts("Failed to map physical memory\n");
		return 1;
	}

	printf("Nand flash write...\n");
	val = nand_write(nand_flash, 0, &length, buf);
	if (val) {
		printf("Nand flash write failed, return %u\n",
				val);
		unmap_physmem(buf, UBOOT_DATA_SIZE);
		return 1;
	}

	unmap_physmem(buf, UBOOT_DATA_SIZE);

	return 0;
}

int data_to_emmc(void)
{
	struct mmc *mmc = find_mmc_device(0);
	void *buf = NULL;

	if (!mmc)
		return 1;

	(void)mmc_init(mmc);

	buf = map_physmem((unsigned long)UBOOT_DATA_ADDR,
			UBOOT_DATA_SIZE, MAP_WRBACK);
	if (!buf) {
		puts("Failed to map physical memory\n");
		return 1;
	}

	printf("MMC write...\n");
	blk_dwrite(mmc_get_blk_desc(mmc), 0, (UBOOT_DATA_SIZE >> NUM_9), buf);
	unmap_physmem(buf, UBOOT_DATA_SIZE);
	return 0;
}
int save_bootdata_to_flash(void)
{
	unsigned int sd_update_flag = 0;
	int ret = 0;
	sd_update_flag = readl(REG_BASE_SCTL + REG_SC_GEN4);
	if (sd_update_flag == START_MAGIC) {
#if defined(CONFIG_HIFMC)
		if (boot_media == BOOT_MEDIA_SPIFLASH) {
			ret = data_to_spiflash();
			if (ret != 0)
				return ret;
		}
		if (boot_media == BOOT_MEDIA_NAND) {
#ifdef CONFIG_CMD_NAND
			ret = data_to_nandflash();
			if (ret != 0)
				return ret;
#else
			printf("Nand flash not support!...\n");
			return 1;
#endif
		}
#endif
#if defined(CONFIG_SUPPORT_EMMC_BOOT)
		if (boot_media == BOOT_MEDIA_EMMC) {
			ret = data_to_emmc();
			if (ret != 0)
				return ret;
		}
#endif

		printf("update success!\n");
	}

	return 0;
}

int auto_update_flag = 0;
int bare_chip_program = 0;

#define REG_BASE_GPIO0          0x120d0000
#define GPIO0_0_DATA_OFST       0x4
#define GPIO_DIR_OFST           0x400

/* upgrade status register(SOFTINT) address: 8~15bit */
#define UPGRADE_STATUS_REG_ADDR (0x1202001C)

typedef enum tagUPGRADE_STATUS_E {
	UPGRADE_STATUS_IDLE = 0,
	UPGRADE_STATUS_PROCESSING,
	UPGRADE_STATUS_FINISH,
	UPGRADE_STATUS_BUTT
} upgrade_status_e;


int is_bare_program(void)
{
	return 1;
}

int is_auto_update(void)
{
#if (defined CONFIG_AUTO_SD_UPDATE) || (defined CONFIG_AUTO_USB_UPDATE)
	/* to add some judgement if neccessary */
	unsigned int  val[NUM_3];
	unsigned int *puregval = (unsigned int *)UPGRADE_STATUS_REG_ADDR;

	if (((*puregval) & (UPGRADE_STATUS_PROCESSING << NUM_8)) != 0)
		return 1; /* update enable */

	writel(0, REG_BASE_GPIO0 + GPIO_DIR_OFST);

	val[NUM_0] = readl(REG_BASE_GPIO0 + GPIO0_0_DATA_OFST);
	if (val[NUM_0])
		return 0;

	udelay(10000); /* delay 10000 us */
	val[NUM_1] = readl(REG_BASE_GPIO0 + GPIO0_0_DATA_OFST);
	udelay(10000); /* delay 10000 us */
	val[NUM_2] = readl(REG_BASE_GPIO0 + GPIO0_0_DATA_OFST);
	udelay(10000); /* delay 10000 us */

	if (val[NUM_0] == val[NUM_1] && val[NUM_1] == val[NUM_2] && val[NUM_0] == NUM_0)
		return 1;    /* update enable */
	else
		return 0;

#else
	return 0;
#endif
}
#endif /* CONFIG_AUTO_UPDATE */


struct boot_medium_interface {
	char name[0x10]; /* 0x10:name len */
	int (*init)(void);
	int (*read)(unsigned long offset, unsigned long len, void *buf);
};

#define NAND_MAX_SIZE  0xFFFFFFFF

static int nand_flash_read(unsigned long offset, unsigned long len, void *buf)
{
#ifdef  CONFIG_CMD_NAND
	struct mtd_info *mtd = nand_info[NUM_0];
	size_t rwsize = len;
	return nand_read_skip_bad(mtd, offset, &rwsize, NULL, NAND_MAX_SIZE,
			(unsigned char *)buf);
#else
	return -1;
#endif
}

static struct spi_flash *spinorflash;

#include "spi_flash.h"
static int spinor_flash_read(unsigned long offset, unsigned long len, void *buf)
{
	/* 0,0,0,0:bus,cs,max_hz,spi_mode */
	spinorflash = spi_flash_probe(0, 0, 0, 0);
	spi_flash_read(spinorflash, offset, len, buf);
	return 0;
}

static struct boot_medium_interface boot_intf[] = {
	{"nand",   NULL, nand_flash_read},
	{"spinor", NULL, spinor_flash_read}
};

static struct boot_medium_interface *boot_intf_p;
void boot_medium_init(void)
{
	/* register boot media interface */
	if (boot_media == BOOT_MEDIA_NAND)
		boot_intf_p = &boot_intf[NUM_0];
	else if(boot_media == BOOT_MEDIA_SPIFLASH)
		boot_intf_p = &boot_intf[NUM_1];
}

void select_upgrade_dev(void)
{
#ifdef CONFIG_EMMC
	unsigned int *puval = (unsigned int *)UPGRADE_STATUS_REG_ADDR;
	unsigned int stor_dev = 0;
	unsigned int stor_paration = 0;

	stor_dev = (*puval >> 16) & 0x03; /* 16 0x03: store dev */
	stor_paration = (*puval >> 18) & 0x1f; /* 18 0x1f: store paration */
	if (((*puval) & (UPGRADE_STATUS_PROCESSING << NUM_8)) != NUM_0) {
		/* the upgrade startup by linux application */
		if (stor_dev == NUM_0) {
			/* emmc */
			target_dev = NUM_0;
			target_paratition = stor_paration;
		} else {
			/* sd */
			target_dev = NUM_1;
		}
	} else {
		/* the upgrade startup by upgrade key on the board, os defaule upgrade style */
		target_dev = NUM_1;
	}

	printf("update dev is %d: paratition is %d\n",
			target_dev, target_paratition);
#endif
	return;
}

int misc_init_r(void)
{
	boot_medium_init();

#ifdef CONFIG_RANDOM_ETHADDR
	random_init_r();
#endif
	setenv("verify", "n");

#if (CONFIG_AUTO_UPDATE == 1)
	/* auto update flag */
	if (is_auto_update())
		auto_update_flag = 1;
	else
		auto_update_flag = 0;

	/* bare chip program flag */
	if (is_bare_program())
		bare_chip_program = 1;
	else
		bare_chip_program = 0;

#ifdef CFG_MMU_HANDLEOK
	dcache_stop();
#endif

#ifdef CFG_MMU_HANDLEOK
	dcache_start();
#endif

#endif

#if (CONFIG_AUTO_UPDATE == 1)
	int result = -1;
	if (auto_update_flag) {
		select_upgrade_dev();
		result = do_auto_update();
		unsigned int *puregval = (unsigned int *)UPGRADE_STATUS_REG_ADDR;

		if ((((*puregval) & (UPGRADE_STATUS_PROCESSING << NUM_8)) != NUM_0) &&
				(result == NUM_0)) {
			printf("update upgrade status: finish\n");
			*puregval = (UPGRADE_STATUS_FINISH << NUM_8);
		}
	}
	if (bare_chip_program && !auto_update_flag)
		save_bootdata_to_flash();
	if (result == 0)
		do_reset(NULL, 0, 0, NULL);
#endif
	return 0;
}

int board_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gd->bd->bi_arch_number = MACH_TYPE_HI3556V200;
	gd->bd->bi_boot_params = CFG_BOOT_PARAMS;

	boot_flag_init();

	return 0;
}

int dram_init(void)
{
	DECLARE_GLOBAL_DATA_PTR;

	gd->ram_size = PHYS_SDRAM_1_SIZE;
	return 0;
}

void reset_cpu(ulong addr)
{
	/* 0x12345678:writing any value will cause a reset. */
	writel(0x12345678, REG_BASE_SCTL + REG_SC_SYSRES);
	while (1);
}

int timer_init(void)
{
	/*
	 * Under uboot, 0xffffffff is set to load register,]
	 * timer_clk equals BUSCLK/2/256.
	 * e.g. BUSCLK equals 50M, it will roll back after 0xffffffff/timer_clk
	 * 43980s equals 12hours
	 */
	__raw_writel(0, CFG_TIMERBASE + REG_TIMER_CONTROL);
	__raw_writel(~0, CFG_TIMERBASE + REG_TIMER_RELOAD);

	/* 32 bit, periodic */
	__raw_writel(CFG_TIMER_CTRL, CFG_TIMERBASE + REG_TIMER_CONTROL);

	__raw_writel(0, TIMER6_REG_BASE + REG_TIMER_CONTROL);
	__raw_writel(~0, TIMER6_REG_BASE + REG_TIMER_RELOAD);
	/* 32 bit, periodic */
	__raw_writel(CFG_TIMER_CTRL, TIMER6_REG_BASE + REG_TIMER_CONTROL);
	return 0;
}

int board_eth_init(bd_t *bis)
{
	int rc = 0;

#ifdef CONFIG_NET_HISFV300
	rc = hieth_initialize(bis);
#endif
	return rc;
}
#ifdef CONFIG_GENERIC_MMC
int board_mmc_init(bd_t *bis)
{
	int ret = 0;
	int dev_num = 0;

#ifdef CONFIG_SUPPORT_EMMC_BOOT
	ret = himci_add_port(dev_num, EMMC_REG_BASE, CONFIG_HIMCI_MAX_FREQ);
	if (!ret) {
		ret = himci_probe(dev_num);
		if (ret)
			printf("No EMMC device found !\n");
	}
	dev_num++;
#endif

#ifdef CONFIG_AUTO_SD_UPDATE
	if (is_auto_update()) {
		ret = himci_add_port(dev_num, SDIO0_REG_BASE, CONFIG_SDIO0_FREQ);
		if (!ret) {
			ret = himci_probe(dev_num);
			if (ret)
				printf("No SD device found !\n");
		}
	}
#endif

	return ret;
}
#endif


#ifdef CONFIG_SECURE_SCHEME_ENABLE

/* macros to get secure register's offset address */
#define sec_rgn_map_ofst(region_idx) (0x1100 + 0x10 * (region_idx))
#define sec_rgn_map_ext_ofst(region_idx) (0x1200 + 0x10 * (region_idx))

#define sec_rgn_attrib_ofst(region_idx) (0x1104 + 0x10 * (region_idx))

#define sec_mid_wr_ofst(region_idx) (0x1108 + 0x10 * (region_idx))
#define sec_mid_wr_ext_ofst(region_idx) (0x1204 + 0x10 * (region_idx))

#define sec_mid_rd_ofst(region_idx) (0x110c + 0x10 * (region_idx))
#define sec_mid_rd_ext_ofst(region_idx) (0x1208 + 0x10 * (region_idx))

#define SEC_BYPASS_OFST         0x1004
#define SEC_INT_EN_OFST         0x1020

/* macros for register value */
#define SECURITY_ENABLE 0xff

#define SEC_INV_EN      (0x1 << 4)
#define SHARE_REGN_EN   (0xf)
#define SECUR_REGN_EN   (0xc)
#define NON_SEC_REGN_EN (0x3)

#define A7CPU0_MID     0xf
#define A7CPU1_MID     0x11
#define DSP_M0_MID      0x13
#define AVSP_M2_MID     0x18
#define DSP_M1_MID      0x1c

#define MID_HIGH_START_BIT  0x20
#define FMC_MID     0x21
#define EMMC_MID    0x28

#define IPCM_REGN_WR_MID_MASK   ((0x1 << A7CPU0_MID) | (0x1 << A7CPU1_MID))
#define IPCM_REGN_RD_MID_MASK   IPCM_REGN_WR_MID_MASK

#define DSP_REGN_WR_MID_MASK    ((0x1 << A7CPU0_MID) | (0x1 << DSP_M0_MID) |\
		(0x1 << AVSP_M2_MID) | (0x1 << DSP_M1_MID))
#define DSP_REGN_RD_MID_MASK    DSP_REGN_WR_MID_MASK

#define LITEOS_REGN_WR_MID_MASK (0x1 << A7CPU1_MID)
#define LITEOS_REGN_RD_MID_MASK LITEOS_REGN_WR_MID_MASK

#define LINUX_REGN_WR_MID_MASK  ((0x1 << A7CPU0_MID))
#define LINUX_REGN_RD_MID_MASK  LINUX_REGN_WR_MID_MASK

#define LINUX_REGN_WR_MID_HIGH_MASK ((0x1 << (FMC_MID - MID_HIGH_START_BIT)) |\
		(0x1 << (EMMC_MID - MID_HIGH_START_BIT)))
#define LINUX_REGN_RD_MID_HIGH_MASK LINUX_REGN_WR_MID_HIGH_MASK

#define REGN_CTRL_ENABLE        (0x1 << 31)

#define IPCM_REGN_HIGH_ADDR     0x2000  /* high 16 bit of address */
#define IPCM_REGN_SIZE          0x20    /* 0x20 * 64K Byte */

#define LITEOS_REGN_HIGH_ADDR   0x2200  /* high 16 bit of address */
#define LITEOS_REGN_SIZE        0x1000  /* 0x1000 * 64K Byte */

#define LINUX_REGN_HIGH_ADDR    0x3200  /* high 16 bit of address */
#define LINUX_REGN_SIZE         0x1000  /* 0x1000 * 64K Byte */

#define IPCM_DDR_REGN_IDX       1
#define DSP_DDR_REGN_IDX        2
#define LITEOS_DDR_REGN_IDX     3
#define LINUX_DDR_REGN_IDX      4

static void ipcm_tzasc_init(void)
{
	/* Set the share access authority */
	writel(SEC_INV_EN | SHARE_REGN_EN,
			DDRC0_REG_BASE + sec_rgn_attrib_ofst(IPCM_DDR_REGN_IDX));

	/* Set the master id mask of write command */
	writel(IPCM_REGN_WR_MID_MASK,
			DDRC0_REG_BASE + sec_mid_wr_ofst(IPCM_DDR_REGN_IDX));
	writel(0, DDRC0_REG_BASE + sec_mid_wr_ext_ofst(IPCM_DDR_REGN_IDX));

	/* Set the master id mask of read command */
	writel(IPCM_REGN_RD_MID_MASK,
			DDRC0_REG_BASE + sec_mid_rd_ofst(IPCM_DDR_REGN_IDX));
	writel(0, DDRC0_REG_BASE + sec_mid_rd_ext_ofst(IPCM_DDR_REGN_IDX));

	/* Set the start address and enable bit */
	writel(REGN_CTRL_ENABLE | IPCM_REGN_HIGH_ADDR,
			DDRC0_REG_BASE +  sec_rgn_map_ofst(IPCM_DDR_REGN_IDX));

	/* Set the size of ipcm ddr region */
	writel(IPCM_REGN_SIZE, DDRC0_REG_BASE + sec_rgn_map_ext_ofst(
				IPCM_DDR_REGN_IDX));
}

static void liteos_tzasc_init(void)
{
	/* Set the secure access authority */
	writel(SEC_INV_EN | SECUR_REGN_EN,
			DDRC0_REG_BASE + sec_rgn_attrib_ofst(LITEOS_DDR_REGN_IDX));

	/* Set the master id mask of write command */
	writel(LITEOS_REGN_WR_MID_MASK,
			DDRC0_REG_BASE + sec_mid_wr_ofst(LITEOS_DDR_REGN_IDX));
	writel(0, DDRC0_REG_BASE + sec_mid_wr_ext_ofst(LITEOS_DDR_REGN_IDX));

	/* Set the master id mask of read command */
	writel(LITEOS_REGN_RD_MID_MASK,
			DDRC0_REG_BASE + sec_mid_rd_ofst(LITEOS_DDR_REGN_IDX));
	writel(0, DDRC0_REG_BASE + sec_mid_rd_ext_ofst(LITEOS_DDR_REGN_IDX));

	/* Set the start address and enable bit */
	writel(REGN_CTRL_ENABLE | LITEOS_REGN_HIGH_ADDR,
			DDRC0_REG_BASE + sec_rgn_map_ofst(LITEOS_DDR_REGN_IDX));

	/* Set the size of dsp ddr region */
	writel(LITEOS_REGN_SIZE,
			DDRC0_REG_BASE + sec_rgn_map_ext_ofst(LITEOS_DDR_REGN_IDX));
}

static void linux_tzasc_init(void)
{
	/* Set the non-secure access authority */
	writel(SEC_INV_EN | NON_SEC_REGN_EN,
			DDRC0_REG_BASE + sec_rgn_attrib_ofst(LINUX_DDR_REGN_IDX));

	/* Set the master id mask of write command */
	writel(LINUX_REGN_WR_MID_MASK,
			DDRC0_REG_BASE + sec_mid_wr_ofst(LINUX_DDR_REGN_IDX));
	writel(LINUX_REGN_WR_MID_HIGH_MASK,
			DDRC0_REG_BASE + sec_mid_wr_ext_ofst(LINUX_DDR_REGN_IDX));

	/* Set the master id mask of read command */
	writel(LINUX_REGN_RD_MID_MASK,
			DDRC0_REG_BASE + sec_mid_rd_ofst(LINUX_DDR_REGN_IDX));
	writel(LINUX_REGN_RD_MID_HIGH_MASK,
			DDRC0_REG_BASE + sec_mid_rd_ext_ofst(LINUX_DDR_REGN_IDX));

	/* Set the start address and enable bit */
	writel(REGN_CTRL_ENABLE | LINUX_REGN_HIGH_ADDR,
			DDRC0_REG_BASE + sec_rgn_map_ofst(LINUX_DDR_REGN_IDX));

	/* Set the size of dsp ddr region */
	writel(LINUX_REGN_SIZE, DDRC0_REG_BASE + sec_rgn_map_ext_ofst(
				LINUX_DDR_REGN_IDX));
}

static int tzasc_init(void)
{
	/*
	 * Set IPCM ddr region attribution(0x40000000 ~ 0x40100000)*]
	 * */
	ipcm_tzasc_init();

	/*
	 * Set Liteos ddr region attribution(0x42000000 ~ 0x10000000)*]
	 * */
	liteos_tzasc_init();

	/*
	 * Set Linux ddr region attribution(0x42000000 ~ 0x10000000)*]
	 * */
	linux_tzasc_init();

	/* Disable interrupt */
	writel(0, DDRC0_REG_BASE + SEC_INT_EN_OFST);

	/* Enable security */
	writel(SECURITY_ENABLE, DDRC0_REG_BASE + SEC_BYPASS_OFST);

	return 0;
}
#endif

#define PAGE_SIZE       4096
#define page_size_align(x) (((x)+(PAGE_SIZE)-1) / (PAGE_SIZE)*(PAGE_SIZE))

static int __run_another_cpus(unsigned int addr)
{
	unsigned int start_phys = 0;
	unsigned int regval;

	printf("## Starting cpu1 A7 at 0x%08X ...\n", addr);
	/* 0xe51ff004 = "ldr  pc, [pc, #-4]" */
	writel(0xe51ff004, start_phys);
	/* 0x4:next addr value will be the "go_cpu1" addr */
	writel(addr, (start_phys + 0x4));

	asm("dsb");
	asm("isb");

	/* clear the slave cpu reset */
	/* 0x12010000:CRG base address; 0x0078:PERI_CRG30 */
	regval = readl(0x12010000 + 0x0078);
	regval &= ~(NUM_1 << NUM_2);
	/* 0x12010000:CRG base address; 0x0078:PERI_CRG30 */
	writel(regval, (0x12010000 + 0x0078));

	return 0;
}

static int start_other_cpus(cmd_tbl_t *cmdtp, int flag,
		int argc, char *const argv[])
{
	int ret = 0;
	unsigned int addr;
	unsigned int srcaddr;
	unsigned int offset;
	unsigned int maxsize = 0;
	int size_compressed;
	int size_uncompressed = 0;

	if (argc == NUM_2) {
		addr    = simple_strtoul(argv[NUM_1], NULL, 16); /* 16:base */
		flush_dcache_all();
		return __run_another_cpus(addr);
	}
	if (argc != NUM_5)
		return CMD_RET_USAGE;

#ifdef CONFIG_SECURE_SCHEME_ENABLE
	tzasc_init();
#endif
	addr    = simple_strtoul(argv[NUM_1], NULL, 16); /* 16:base */
	srcaddr = simple_strtoul(argv[NUM_2], NULL, 16); /* 16:base */
	offset  = simple_strtoul(argv[NUM_3], NULL, 16); /* 16:base */
	maxsize = simple_strtoul(argv[NUM_4], NULL, 16); /* 16:base */
	/* Read A7 bin size */
	boot_intf_p->read(offset, 0x80, (void *)srcaddr);
	flush_dcache_all();
	if ((*(int *)(srcaddr + HEAD_MAGIC_NUM0_OFFSET) != HEAD_MAGIC_NUM0) || \
			(*(int *)(srcaddr + HEAD_MAGIC_NUM1_OFFSET) != HEAD_MAGIC_NUM1)) {
		boot_intf_p->read(offset, page_size_align(maxsize), (void *)addr);
		flush_dcache_all();
		return __run_another_cpus(addr);
	}

	/* read A7 bin to DDR */
	size_compressed = *(int *)(srcaddr + COMPRESSED_SIZE_OFFSET);
	if (page_size_align(size_compressed) > maxsize) {
		printf("Image size info error !\n");
		return 0;
	}
	boot_intf_p->read(offset, page_size_align(size_compressed),
			(void *)srcaddr);
	flush_dcache_all();
	size_compressed = *(int *)(srcaddr + COMPRESSED_SIZE_OFFSET);
	size_uncompressed = *(int *)(srcaddr + UNCOMPRESSED_SIZE_OFFSET);
	hw_dec_type = 0;
	hw_dec_init();
	ret = hw_dec_decompress((unsigned char *)(uintptr_t)addr, &size_uncompressed, \
			((unsigned char *)(uintptr_t)srcaddr + HIGZIP_HEAD_SIZE),
			size_compressed, NULL);
	if (!ret) {
		printf("decompress ok!\n");
	} else {
		printf("ERR:\n\tdecompress fail!\n");
		return 0;
	}
	hw_dec_uinit();
#ifdef CONFIG_SYS_LEVEL2_CACHE_SHARE
	/* flushed dcache before start Cpu1 */
	flush_dcache_all();
#endif

	return __run_another_cpus(addr);
}

U_BOOT_CMD(
		go_cpu1, CONFIG_SYS_MAXARGS, 3, start_other_cpus,
		"start a7-cpu1 at address 'entry'",
		"entry [tmpaddr offset maxsize]\n"
		"Example:\n\tgo_cpu1 0x80200000 83000000 0xf00000 0x100000\n\tgo_cpu1 0x80200000"
	  );

#ifdef CONFIG_ARMV7_NONSEC
void smp_set_core_boot_addr(unsigned long addr, int corenr)
{
}

void smp_kick_all_cpus(void)
{
}

void smp_waitloop(unsigned previous_address)
{
}
#endif
