obj-m := phddrv.o
phddrv-objs := phd.o cdadrv.o cdapci.o

EXTRA_CFLAGS += -I$(obj) -I$(obj)/include
EXTRA_CFLAGS += -D_PHX_LINUX -DNDEBUG -D_PHX_CDA -D_CDA_SG64

ifneq ($(KERNELRELEASE),)
MODCONF_FILE := $(shell $(obj)/scripts/get_modprobe_file)
else
MODCONF_FILE := $(shell $(PWD)/scripts/get_modprobe_file)
endif

driver:
	echo $(PWD)
	make -C /lib/modules/$(shell uname -r)/build/ SUBDIRS=$(PWD) modules 

clean:
	rm -f cdadrv.o phd.o cdapci.o
	rm -f phddrv.o phddrv.mod.c phddrv.mod.o
	rm -f .cdadrv.o.cmd
	rm -f .cdapci.o.cmd
	rm -f .phd.o.cmd
	rm -f .phddrv.ko.cmd
	rm -f .phddrv.mod.o.cmd
	rm -f .phddrv.o.cmd
	rm -rf .tmp_versions

distclean: clean
	-/sbin/modprobe -r phddrv > /dev/null 2>&1
	rm -f /lib/modules/$(shell uname -r)/kernel/drivers/media/video/phddrv.ko
	/sbin/depmod -a
	$(PWD)/scripts/remove_special_phx 243
	-mv $(MODCONF_FILE) $(MODCONF_FILE).bak
	touch $(MODCONF_FILE).bak
	sed -f scripts/remove_modprobe_conf_local.sed $(MODCONF_FILE).bak > $(MODCONF_FILE)
	rm -f phddrv.ko

install: driver
	-/sbin/modprobe -r phddrv > /dev/null 2>&1
	install -D -gwheel -oroot -m744 phddrv.ko /lib/modules/$(shell uname -r)/kernel/drivers/media/video/phddrv.ko
	$(PWD)/scripts/create_special_phx 243 > /dev/null 2>&1
	/sbin/depmod -a
	/sbin/modprobe phddrv
	-mv $(MODCONF_FILE) $(MODCONF_FILE).bak
	touch $(MODCONF_FILE).bak
	sed -f scripts/create_modprobe_conf_local.sed $(MODCONF_FILE).bak > $(MODCONF_FILE)

uninstall: distclean

