obj-m:=sgdma.o
#obj-m := dma_test.o


KDIR  := /lib/modules/$(shell uname -r)/build
PWD   := $(shell pwd)

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

test: default
	sudo dmesg -c && sudo rmmod sgdma && sudo insmod sgdma.ko && sudo dmesg -c



.PHONY: test dma_test
