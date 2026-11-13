ifneq (,$(filter $(CONFIG_VIDEO_ECAM_ISP),y m))
	ccflags-y += -Wno-unused-variable -I$(srctree)/drivers/media/platform/tegra/ -I$(srctree)/../nvidia/include/
	e-cam_imx485-objs := cam_common.o
endif

obj-m += e-cam_imx485.o
e-cam_imx485-objs := cam_common.o
EXTRA_CFLAGS=-I$(PWD)/../kernel/nvidia/include/
imx485:
	make -C $(KERNEL_PATH) M=$(PWD) modules
clean:
	make -C $(KERNEL_PATH) M=$(PWD) clean
