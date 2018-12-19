

#KERNELDIR =
#INSTALLDIR =

obj-m := tm1638.o

modules:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

install:
	scp ./hello.ko $(INSTALLDIR)
clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions modules.order  Module.symvers

.PHONY: modules install clean
