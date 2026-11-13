#CONFIG_MODULE_SIG=n
ifneq (,$(filter $(CONFIG_VIDEO_ECAM_ISP),y m))
    ccflags-y += -I$(PWD)/../kernel/nvidia/include/
    obj-m       +=  ar1820.o
endif


#obj-m  += ar1820.o
#ar1820-objs := ar1820.o
#EXTRA_CFLAGS=-I$(PWD)/../kernel/nvidia/include/

ar1820:
	make -C $(TEGRA_KERNEL_OUT) M=$(PWD) modules

clean:
	make -C $(TEGRA_KERNEL_OUT) M=$(PWD) clean
