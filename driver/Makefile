CONFIG_MODULE_SIG=n

MODULE_NAME := ch9433
KVER ?= $(shell uname -r)
KERNELDIR ?= /lib/modules/$(KVER)/build
INSTALL_MOD_DIR ?= extra
CONFIG_MODULE_SIG ?= n

ifneq ($(KERNELRELEASE),)
obj-m := $(MODULE_NAME).o
else

.PHONY: all default clean load unload install uninstall

all default:
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) CONFIG_MODULE_SIG=$(CONFIG_MODULE_SIG) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) clean

load: all
	modprobe can
	modprobe can-dev
	insmod ./$(MODULE_NAME).ko

unload:
	-rmmod $(MODULE_NAME)

install: all
	$(MAKE) -C $(KERNELDIR) M=$(CURDIR) CONFIG_MODULE_SIG=$(CONFIG_MODULE_SIG) INSTALL_MOD_DIR=$(INSTALL_MOD_DIR) modules_install
	depmod -a $(KVER)

uninstall:
	-rmmod $(MODULE_NAME)
	rm -f /lib/modules/$(KVER)/$(INSTALL_MOD_DIR)/$(MODULE_NAME).ko
	depmod -a $(KVER)

endif
