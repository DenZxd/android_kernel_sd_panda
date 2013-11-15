#!/bin/bash
 ################################################################
 # $ID: strap-setup.sh Èý, 25  8ÔÂ 2010 15:00:38 +0800  mhfan $ #
 #                                                              #
 # Description:                                                 #
 #                                                              #
 # Maintainer:  ·¶ÃÀ»Ô(MeiHui FAN)  <mhfan@ustc.edu>            #
 #                                                              #
 # CopyLeft (c)  2010  M.H.Fan                                  #
 #   All rights reserved.                                       #
 #                                                              #
 # This file is free software;                                  #
 #   you are free to modify and/or redistribute it  	        #
 #   under the terms of the GNU General Public Licence (GPL).   #
 #                                                              #
 # Last modified: Thu, 14 Nov 2013 16:24:44 +0800       by root #
 ################################################################

#set -e

[ -z $1 ] && echo "Please specify a path of RootFS!" && exit 1

ROOTFS=$1
STRAPCONF=scripts/debian-sid.conf
[ "$ROOTFS" = "rootfs-ubt" ] && STRAPCONF=scripts/ubuntu.conf
#multistrap -f $STRAPCONF -d $ROOTFS #--tidy-up
cd $ROOTFS

for F in usr/lib/arm-linux-gnueabihf/*; do
    L=$(readlink $F) && [[ $L == /lib/arm-linux-gnueabihf/* ]] &&
	ln -sf ../../..$L $F;
done

sed -i -e 's,/usr/lib/arm-linux-gnueabihf/,,; s, /lib/, ../../../lib/,g' \
    usr/lib/arm-linux-gnueabihf/lib{c,pthread}.so

echo XCBS > etc/hostname

false && cat > etc/init.d/LXDUI << EOF

### BEGIN INIT INFO
# Provides:          mhfan
# Required-Start:    $remote_fs $syslog
# Required-Stop:     $remote_fs $syslog
# Should-Start:      udev
# Should-Stop:       
# X-Start-Before:    
# X-Stop-After:      
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# X-Interactive:     true
# Short-Description: Example initscript
# Description:       This file should be used to construct scripts to be
#                    placed in /etc/init.d.
### END INIT INFO

NAME=LXDUI
LXDP=/opt/LXDetector
PATH=$LXDP:/sbin:/bin:/usr/sbin:/usr/bin

set -e

/etc/init.d/rc.pvr $1
case "$1" in
  start|restart)
    echo -n "Starting LXDetector: "
    start-stop-daemon -S -b -p /var/run/$NAME.pid -m \
	-d $LXDP -x $LXDP/$NAME -n $NAME \
	-- -qws -display powervr #-mouse "..."
    echo $NAME.
    done
    ;;

  stop)
    ;;

  *)
    echo "Usage: /etc/init.d/$NAME {start|stop|restart|force-reload}" >&2
    exit 1
    ;;
esac

exit 0

EOF
#&& update-rc.d LXDUI defaults 01

F=etc/inittab; grep -q ttyO0 $F || cat >> $F <<  EOF

#PVR::sysinit:/etc/init.d/rc.pvr start
#LXD:2345:respawn:/opt/LXDetector/LXDUI -qws -display powervr #-mouse "..."

T0:2345:respawn:/sbin/getty -L ttyO0 115200 vt102

EOF

F=etc/rc.local; if ! grep -q mmcblk0 $F; then
sed -i -e 's/^exit 0/#\1/' $F && cat >> $F << EOF

/etc/init.d/rc.pvr start
while true; do
    #cd /opt/LXDetector && #stdsyslog \
    /opt/LXDetector/LXDUI -qws -display powervr #-mouse "..."
done &

[ -e /dev/mmcblk0p1 ] && modprobe g_mass_storage file=/dev/mmcblk0p1

exit 0

EOF
fi

cat > etc/environment << EOF

#TSLIB_CALIBFILE=/media/mmcblk0p1/pointercal
TSLIB_TSDEVICE=/dev/input/event1

QWS_DISPLAY=powervr
#POINTERCAL_FILE=\$TSLIB_CALIBFILE
#QWS_DISPLAY=LinuxFb:mmWidth=162:mmHeight=122:0	# XXX:
#QWS_MOUSE_PROTO=LinuxTP:/dev/input/event1:MouseMan:/dev/input/mice:Tslib:/dev/input/event1:LinuxInput:/dev/input/event1

# XXX: -qws -display "..." -mouse "..."

EOF

mkdir -p media/mmcblk0p1 #boot/u-boot
cat > etc/fstab << EOF
# <file system> <mount point> <type> <options> <dump> <pass>
/dev/mmcblk0p2 / ext4 noatime,errors=remount-ro 0 1

debugfs /sys/kernel/debug debugfs noauto 0 0

#/dev/mmcblk0p1 /boot/u-boot auto noauto 0 0

EOF

cat > etc/network/interfaces << EOF

auto  lo
iface lo inet loopback

auto  eth0
iface eth0 inet dhcp

#allow-hotplug usb0
#iface usb0 inet dhcp
iface usb0 inet static
    netmask 255.255.255.0
    network 192.168.7.0
    gateway 192.168.7.1
    address 192.168.7.2

EOF

cat > etc/e2fsck.conf << EOF
# fix rtc/fsck issue for beaglebone
[options]
broken_system_clock = true

EOF

cat > etc/udev/rules.d/local.rules << EOF

# Media automounting
SUBSYSTEM=="block", ACTION=="add"    RUN+="/etc/udev/scripts/mount.sh"
SUBSYSTEM=="block", ACTION=="remove" RUN+="/etc/udev/scripts/mount.sh"

# Handle network interface setup
SUBSYSTEM=="net",   ACTION=="add"    RUN+="/etc/udev/scripts/network.sh"
SUBSYSTEM=="net",   ACTION=="remove" RUN+="/etc/udev/scripts/network.sh"
# XXX: /etc/init.d/networking restart

# Create a symlink to any touchscreen input device
#SUBSYSTEM=="input", KERNEL=="event[0-9]*", ATTRS{modalias}=="input:*-e0*,3,*a0,1,*18,*", SYMLINK+="input/touchscreen0"

EOF

F=etc/default/ntpdate; NTP_CN=cn.pool.ntp.org;
grep -q $NTP_CN $F || cat >> $F <<  EOF

NTPSERVERS="$NTP_CN \$NTPSERVERS"

EOF

cp -f ~mhfan/.dir_colors etc/DIR_COLORS
F=etc/profile; grep -q dircolors $F || cat > $F << EOF

eval "\$(TERM=ansi dircolors -b /etc/DIR_COLORS)"
alias ls="ls --color=auto"
alias vi="busybox vi"

EOF

mkdir -p etc/python
cat > etc/python/debian_config << EOF
[DEFAULT]
# how to byte-compile (comma separated: standard, optimize)
byte-compile = optimize

EOF

cat > etc/locale.gen << EOF
#zh_CN.GB18030 GB18030
zh_CN.UTF-8 UTF-8
#zh_CN.GBK GBK
EOF

F=etc/locale.nopurge; grep -q zh_CN.UTF-8 $F || cat >> $F << EOF

en
en_US
en_US.UTF-8

zh
zh_CN
#zh_CN.GBK
zh_CN.UTF-8
#zh_TW.UTF-8
#zh_CN.GB18030

EOF

TIMEZ=Asia/Shanghai
echo $TIMEZ > etc/timezone
ln -sf ../usr/share/zoneinfo/$TIMEZ etc/localtime

F=etc/sudoers; grep -q mhfan $F ||
echo "mhfan	ALL=(ALL:ALL) NOPASSWD: ALL" >> $F
echo "nameserver 8.8.8.8" > etc/resolv.conf

QTE=opt/QtE-armhf
echo /$QTE/lib > etc/ld.so.conf.d/$(basename $QTE).conf
rm -rf $QTE/{bin,include,mkspecs,lib/{pkgconfig,*.{la,prl}}}
rm -rf $QTE/translation/{assistant,designer,linguist,qt_help}_*.qm
ln -sf ../../../../usr/share/fonts/truetype/wqy/* $QTE/lib/fonts/
find   $QTE -name \*.debug -exec rm -f '{}' ';'

sed -i -e 's/^\(\s\+.*restart.*\)/echo \1/' var/lib/dpkg/info/dropbear.postinst

STAGE2SH=tmp/strap-stage2.sh
cat > $STAGE2SH << EOF
export LC_ALL=C LANGUAGE=C LANG=C

for F in /bin/sh /usr/share/man/man1/sh.1.gz; do
    dpkg-divert --package dash --divert \$F.divert --add \$F
done

export DEBIAN_FRONTEND=noninteractive DEBCONF_NONINTERACTIVE_SEEN=true
dpkg --configure -a;	dpkg --configure -a	# XXX:

apt-get -y purge gcc-4.7-base linux-libc-dev libc-dev-bin perl libgdbm3
apt-get -y purge libjbig-dev liblcms1-dev liblzma-dev libts-dev libtiffxx5
rm -f /usr/lib/arm-linux-armhf/*.a

for S in networking mountnfs.sh mountnfs-bootclean.sh; do
    update-rc.d \$S disable S;
done
for S in 2 3 4 5; do update-rc.d dropbear disable \$S; done

#passwd -d root
echo 'root:!@#\$hjkl' | chpasswd
useradd -M -s /bin/bash -U -G root -p 'asdf&*()' \
	-c "MeiHui FAN <mhfan@ustc.edu>" mhfan
#mkdir -p /opt/xbin && busybox --install -s /opt/xbin

update-locale LC_ALL=zh_CN.UTF-8
localepurge

EOF

#mkdir -p /dev/pts
#mount --rbind /dev dev
#mount -t devpts none dev/pts

#mount -t proc  none proc
#mount -t sysfs none sys

cp -f /usr/bin/qemu-arm-static usr/bin/ && chroot . sh /$STAGE2SH
rm -rf selinux lib64 usr/share/{doc,man,info}/* tmp/*

exit 0	# XXX:
mkdir -p var/lib/x11

FILE=dev/fd; [ -L $FILE ] && ln -sfT ../proc/self/fd $FILE
FILE=lib/udev/devices/fd; [ -L $FILE ] && ln -sfT ../../../proc/self/fd $FILE

find {,usr/,usr/local/}lib -type f -exec \
    strip -d -R .comment -R .note '{}' ';' > /dev/null 2>&1
find {,usr/,usr/local/}{bin,sbin} -type f -exec \
    strip -s -R .comment -R .note '{}' ';' > /dev/null 2>&1

false &&
sudo tar czf ../rootfs.tgz --exclude dev/* --exclude proc/* --exclude sys/* .
#sudo tar zxf /path/to/rootfs.tgz --numeric-owner --preserve-permissions

 # vim:sts=4:ts=8:
