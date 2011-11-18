obj-m:=altera_drv.o


KDIR  := /lib/modules/$(shell uname -r)/build
PWD   := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

test: default
	sudo dmesg -c && sudo rmmod altera_drv && sudo insmod altera_drv.ko && sudo dmesg -c

.PHONY: test
