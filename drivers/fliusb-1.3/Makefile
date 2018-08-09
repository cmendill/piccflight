KERNEL_DIR := /lib/modules/$(shell uname -r)/build

obj-m := fliusb.o

DRIVER_MODULE=fliusb.ko

all: driver cleanup

driver:	fliusb.c
	make -C $(KERNEL_DIR) SUBDIRS=`pwd` EXTRA_CFLAGS="$(MY_CFLAGS)" modules

install:
	-/sbin/modprobe -r fliusb > /dev/null 2>&1
	install -D -gwheel -oroot -m744 fliusb.ko /lib/modules/$(shell uname -r)/kernel/drivers/pci/rtd/fliusb.ko
	/sbin/depmod -a
	/sbin/modprobe fliusb

cleanup:
	rm -f *.o .*.cmd *.mod.c *.order *.symvers; rm -rf .tmp_versions

clean: cleanup
	rm -f *.ko test
