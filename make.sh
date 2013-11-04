#!/bin/bash
 ################################################################
 # $ID: make.sh        Wed, 21 Aug 2013 15:27:22 +0800  mhfan $ #
 #                                                              #
 # Description:                                                 #
 #                                                              #
 # Maintainer:  ∑∂√¿ª‘(MeiHui FAN)  <mhfan@ustc.edu>            #
 #                                                              #
 # CopyLeft (c)  2013  M.H.Fan                                  #
 #   All rights reserved.                                       #
 #                                                              #
 # This file is free software;                                  #
 #   you are free to modify and/or redistribute it  	        #
 #   under the terms of the GNU General Public Licence (GPL).   #
 ################################################################

# http://elinux.org/Building_BBB_Kernel

CROSS_COMPILE=arm-linux-gnueabihf-

ROOTFS=../rootfs

false &&
#make ARCH=arm CROSS_COMPILE=$CROSS_COMPILE beaglebone_defconfig &&
make ARCH=arm CROSS_COMPILE=$CROSS_COMPILE LOADADDR=80200040 uImage dtbs &&
make ARCH=arm CROSS_COMPILE=$CROSS_COMPILE modules && \
#make ARCH=arm CROSS_COMPILE=$CROSS_COMPILE uImage-dtb.am335x-boneblack &&
make ARCH=arm CROSS_COMPILE=$CROSS_COMPILE \
	INSTALL_MOD_PATH=$ROOTFS modules_install

true && #make beaglebone_defconfig &&
make uImage dtbs -j4 && make modules -j4 &&
#make uImage-dtb.am335x-boneblack &&
	
sudo sh -c " \
    make INSTALL_MOD_PATH=$ROOTFS modules_install && \
    mkdir -p $ROOTFS/boot && \
    cp  arch/arm/boot/dts/*.dtb  $ROOTFS/boot/ && \
    cp  arch/arm/boot/zImage $ROOTFS/boot/ && \
    cp  arch/arm/boot/uImage $ROOTFS/boot/ \
"
#cp  arch/arm/boot/uImage-dtb.am335x-boneblack $ROOTFS/boot/ \

false && cd ../../graphics &&
make BUILD=release FBDEV=yes SUPPORT_XORG=0 PM_RUNTIME=1 OMAPES=8.x -j4 &&
sudo cp gfx_rel_es8.x-armhf/*.ko $ROOTFS/lib/modules/3.8.13/extra/

false && cd ../u-boot &&
make O=am335x am335x_boneblack_config \
	CROSS_COMPILE=arm-linux-gnueabihf- ARCH=arm &&
sudo cp am335x/{MLO,u-boot.img} $ROOTFS/boot/

# mmc rescan; run loadbootenv; run importbootenv
# load mmc ${mmcdev} ${loadaddr} uImage
# run findfdt; run loadfdt; run mmcargs
# spl export fdt ${loadaddr} - ${fdtaddr}

# fatwrite mmc 0:1 0x80f80000 args c42b

 # vim:sts=4:ts=8:
