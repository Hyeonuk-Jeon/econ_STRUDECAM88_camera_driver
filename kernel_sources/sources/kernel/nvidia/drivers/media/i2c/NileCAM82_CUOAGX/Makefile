ifneq (,$(filter $(CONFIG_VIDEO_ECAM_ISP),y m))
	ccflags-y += -I$(srctree)/drivers/media/platform/tegra/ -I$(srctree)/../nvidia/include/
	e-con_cam-objs := serdes.o cam_common.o
endif

ifneq (,$(filter $(CONFIG_VIDEO_ECAM_ISP_MULTICAM),y m))
	ccflags-y += -DMASTER_SLAVE_MODE_ENABLED -DFRAME_SYNC_MODE_ENABLED -I$(srctree)/../nvidia/include/
	e-con_cam-objs  += pca9685.o
endif

obj-m += e-con_cam.o

e-con_cam:
	make -C $(KERNEL_PATH) M=$(PWD) modules
clean:
	make -C $(KERNEL_PATH) M=$(PWD) clean
