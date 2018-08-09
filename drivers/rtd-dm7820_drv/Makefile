MY_CFLAGS := -I$(shell pwd)/../../lib/librtd/include

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
