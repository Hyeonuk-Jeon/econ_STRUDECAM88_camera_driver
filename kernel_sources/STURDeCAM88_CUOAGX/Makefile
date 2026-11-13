ccflags-y += -Wunused-variable -Wunused-function -I$(srctree)/drivers/media/platform/tegra/ -I$(srctree)/../nvidia/include/
obj-m	+= ecam_gmsl_yuv_common.o
ecam_gmsl_yuv_common-objs := ecam_gmsl_octa_yuv_common.o serdes.o sysfs_operations.o strm_mon_thrd.o


ecam_gmsl_yuv_common:
	make -C $(KERNEL_PATH) M=$(PWD) modules
clean:
	make -C $(KERNEL_PATH) M=$(PWD) clean

