#ifneq (,$(filter $(CONFIG_VIDEO_ECAM),y m))
ccflags-y += -Wno-unused-variable -I$(srctree)/drivers/media/platform/tegra/ -I$(srctree)/../nvidia/include/ -Wno-unused-function
obj-y	+=  ar0234.o
ar0234-objs :=  ar0234_common.o serdes.o pca9685.o 
#endif

#ar0234:
#	make -C $(KERNEL_PATH) M=$(PWD) modules
#clean:
#	make -C $(KERNEL_PATH) M=$(PWD) clean
