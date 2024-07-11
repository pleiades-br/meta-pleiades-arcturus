ifneq ($(KERNELRELEASE),)
    obj-m       := eg91_ctrl.o

else
    PWD := $(shell pwd)
    KVER := $(shell uname -r)
    KDIR := /lib/modules/$(KVER)/build

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	rm -rf *.o *.mod.c *.mod.o *.ko *.symvers *.order *.a

endif