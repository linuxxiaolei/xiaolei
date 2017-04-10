KERNELDIR=/linux-3.6
PWD:=$(shell pwd)
INSTALLDIR=/opt/EmbedSky/lcd
CC=arm-linux-gcc
obj-m :=pt100dev.o
modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules
modules_install:
	cp pt100.ko $(INSTALLDIR)
clean:
	rm -rf *.o *.ko *.mod.c *.markers *.order *.symvers
.PHONY:modules modules_install clean
