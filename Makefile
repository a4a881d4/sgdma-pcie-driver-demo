obj-m:=mini_block.o


KDIR  := /lib/modules/$(shell uname -r)/build
PWD   := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

test:
	sudo rmmod mini_block && sudo insmod mini_block.ko

.PHONY: test
