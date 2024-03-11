
obj-m:=hdmdrv.o 
hdmdrv-objs:=binder_mod.o pipe.o 
KDIR:=/lib/modules/$(shell uname -r)/build  
MAKE:=make  
default:  
	$(MAKE) -C $(KDIR) M=$(PWD) modules  
clean:  
	$(MAKE) -C $(KDIR) M=$(PWD) clean
