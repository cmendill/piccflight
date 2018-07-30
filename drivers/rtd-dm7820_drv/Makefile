#
#	FILE NAME: Makefile
#
#	FILE DESCRIPTION: Make description file for building driver
#
#	PROJECT NAME: Linux Software (DM7820)
#
#	PROJECT VERSION: (Defined in README.TXT)
#
#	This file and its contents are copyright (C) RTD Embedded Technologies,
#	Inc.  All Rights Reserved.
#
#	This software is licensed as described in the RTD End-User Software
#	License Agreement.  For a copy of this agreement, refer to the file
#	LICENSE.TXT (which should be included with this software) or contact RTD
#	Embedded Technologies, Inc.
#


MY_CFLAGS := -I$(shell pwd)/../../lib/librtd/include

#
# Kernel build environment directory.  Supposedly it is safer to use this
# method of referring to it than using /usr/src.
#

KERNEL_DIR := /lib/modules/$(shell uname -r)/build

obj-m := rtd-dm7820.o

DRIVER_MODULE=rtd-dm7820.ko

all: driver cleanup

driver:	rtd-dm7820.c
	make -C $(KERNEL_DIR) SUBDIRS=`pwd` EXTRA_CFLAGS="$(MY_CFLAGS)" modules

install:
	-/sbin/modprobe -r rtd-dm7820 > /dev/null 2>&1
	install -D -gwheel -oroot -m744 rtd-dm7820.ko /lib/modules/$(shell uname -r)/kernel/drivers/pci/rtd/rtd-dm7820.ko
	/sbin/depmod -a
	/sbin/modprobe rtd-dm7820

cleanup:
	rm -f *.o .*.cmd *.mod.c *.order *.symvers; rm -rf .tmp_versions

clean: cleanup
	rm -f *.ko test
