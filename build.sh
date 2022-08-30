#!/bin/sh -e

export ARCH=arm
export CROSS_COMPILE=${CROSS_COMPILE:-arm-himix100-linux-}

OUTPUTDIR="${HOME}/src/hisi/uboot"
SOCS="hi3516cv500 hi3516dv300 hi3516av300"

for soc in ${SOCS};do

make clean
make ${soc}_smp_defconfig
cp reginfo-${soc}.bin .reg
make  -j 8

[ ! -f tools/hi_gzip/bin/gzip ] && make -C tools/hi_gzip || cp tools/hi_gzip/bin/gzip arch/arm/cpu/armv7/${soc}/hw_compressed/ -rf

make u-boot-z.bin

cp u-boot-${soc}.bin /home/pk/src/hisi/uboot/u-boot-${soc}-universal.bin
#cp u-boot-${soc}.bin /srv/tftp/u-boot-${soc}-universal.bin

done
