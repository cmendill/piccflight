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


#
# We are now supporting both the 2.4 and 2.6 kernels.  Unfortunately, the 2.6
# external module (external to the kernel source tree) build process differs
# radically from the 2.4 build process.  This means that the driver make file
# must now be split into three parts based upon the kernel version being
# compiled for: 1) a 2.4-specific section, 2) a 2.6-specific section, and 3) a
# section independent of the kernel version.  To determine the exact sections
# which must be enabled, the make process must be automated.
#
# In order to automate the make process, first thing that needs to be done is
# to figure out which kernel is running.  The "uname -r" command is used, which
# returns a string in the form "2.x.x????".  We then pipe this string through
# the cut command to extract the first three characters in the string.  This
# gives us the kernel version, which is used to build kernel-specific rules for
# 2.4 and 2.6 kernels.
#

KERNEL_VERSION=$(shell uname -r | cut -c1-3)


#===============================================================================
# Rules applicable only to the 2.4 kernel
#===============================================================================

ifeq ($(KERNEL_VERSION), 2.4)

OS_INCLUDE=/lib/modules/$(shell /bin/uname -r)/build/include

DRIVER_MODULE=rtd-dm7820.o
FILES_TO_CLEAN=rtd-dm7820.o

CC=gcc
DEBUG_INT=-DDM7820_DEBUG_INTERRUPTS
DEBUG_DMA=-DDM7820_DEBUG_DMA
DEBUG_FLAGS=$(DEBUG_INT) $(DEBUG_DMA)
INCLUDE_FLAGS=-I../include -I$(OS_INCLUDE)
KERNEL_FLAGS=-DMODULE -D__KERNEL__ -DLINUX -DRTD_2_4_KERNEL -URTD_2_6_KERNEL
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall
CFLAGS:= \
	$(INCLUDE_FLAGS) \
	$(KERNEL_FLAGS) \
	$(OPTIMIZE_FLAGS) \
	$(WARNING_FLAGS)

driver:	rtd-dm7820.o

rtd-dm7820.o:	rtd-dm7820.c
	$(CC) $(CFLAGS) -c rtd-dm7820.c

debug: rtd-dm7820.c
	$(CC) $(CFLAGS) $(DEBUG_FLAGS) -c rtd-dm7820.c
    
debug_dma: rtd-dm7820.c
	$(CC) $(CFLAGS) $(DEBUG_DMA) -c rtd-dm7820.c

endif


#===============================================================================
# Rules applicable only to the 2.6 kernel
#===============================================================================

ifeq ($(KERNEL_VERSION), 2.6)


MY_CFLAGS := -I$(shell pwd)/../include -DRTD_2_6_KERNEL -URTD_2_4_KERNEL

#
# Kernel build environment directory.  Supposedly it is safer to use this
# method of referring to it than using /usr/src.
#

KERNEL_DIR := /lib/modules/$(shell uname -r)/build

obj-m := rtd-dm7820.o

DRIVER_MODULE=rtd-dm7820.ko
FILES_TO_CLEAN= \
	*.o \
	rtd-dm7820.ko \
	.*.cmd \
	*.ko \
	.tmp_versions \
	rtd-dm7820.mod.c \
	.*.d \
	*.symvers \
	*.order \
    *.markers \
	*~

driver:		rtd-dm7820.c
	make -C $(KERNEL_DIR) SUBDIRS=`pwd` EXTRA_CFLAGS="$(MY_CFLAGS)" modules

.PHONY: debug
debug:
	make -C $(KERNEL_DIR) SUBDIRS=`pwd` EXTRA_CFLAGS="$(MY_CFLAGS) -DDM7820_DEBUG_INTERRUPTS" modules

.PHONY: debug_dma
debug_dma:
	make -C $(KERNEL_DIR) SUBDIRS=`pwd` EXTRA_CFLAGS="$(MY_CFLAGS) -DDM7820_DEBUG_DMA" modules



endif


#===============================================================================
# Rules applicable to either the 2.4 or 2.6 kernel
#===============================================================================

clean:
	rm -rf $(FILES_TO_CLEAN)

load:
	insmod ./$(DRIVER_MODULE)

devices:
	@echo "Removing old device files ..."; \
	rm -f /dev/rtd-dm7820-* \
	echo ""; \
	echo -n "Obtaining device major number ... "; \
	major=`awk "\\$$2==\"rtd-dm7820\" {print \\$$1}" /proc/devices`; \
	if [ "$${major}" == "" ]; \
	then \
	    echo ""; \
	    echo ""; \
	    echo "FAILED: The kernel driver doesn't seem to be loaded."; \
	    echo ""; \
	    exit 1; \
	fi; \
	echo "$${major}"; \
	echo ""; \
	echo -n "Getting number of devices ... "; \
	if [ ! -e /proc/rtd-dm7820 ]; \
	then \
	    echo ""; \
	    echo ""; \
	    echo "FAILED: /proc/rtd-dm7820 does not exist."; \
	    echo ""; \
	    exit 1; \
	fi; \
	if [ ! -r /proc/rtd-dm7820 ]; \
	then \
	    echo ""; \
	    echo ""; \
	    echo "FAILED: /proc/rtd-dm7820 is not readable."; \
	    echo ""; \
	    exit 1; \
	fi; \
	devices=`cat /proc/rtd-dm7820`; \
	echo "$${devices}"; \
	echo ""; \
	echo "Creating device files ..."; \
	minor=0; \
	while [ $${minor} -lt $${devices} ]; \
	do \
	    echo "   /dev/rtd-dm7820-$${minor}"; \
	    mknod --mode=0666 /dev/rtd-dm7820-$${minor} c $${major} $${minor}; \
	    minor=`expr $${minor} + 1`; \
	done;

unload:
	rmmod rtd-dm7820
