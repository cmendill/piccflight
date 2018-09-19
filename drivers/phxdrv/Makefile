obj-m := phddrv.o
phddrv-objs := phd.o cdadrv.o cdapci.o

KDIR		?= /lib/modules/$(shell uname -r)/build
PWD		:= $(shell pwd)

EXTRA_CFLAGS += -I$(obj) -I$(obj)/include
EXTRA_CFLAGS += -D_PHX_LINUX -DNDEBUG -D_PHX_CDA -D_CDA_SG64

all: driver cleanup

driver:
	make -C $(KDIR) SUBDIRS=$(PWD) modules 

install: 
	-/sbin/modprobe -r phddrv > /dev/null 2>&1
	install -D -gwheel -oroot -m744 phddrv.ko /lib/modules/$(shell uname -r)/kernel/drivers/pci/phd/phddrv.ko
	$(PWD)/scripts/create_special_phx 243 > /dev/null 2>&1
	/sbin/depmod -a
	/sbin/modprobe phddrv

cleanup:
	rm -f *.o .*.cmd *.mod.c *.order *.symvers; rm -rf .tmp_versions

clean: cleanup
	rm -f *.ko test
