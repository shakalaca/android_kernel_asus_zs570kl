#
# Makefile for Broadcom shared uart transport solution
#
obj-$(CONFIG_SH_V4L2)		+= brcm_hci_ldisc.o
brcm_hci_ldisc-objs := brcm_hci.o brcm_sh_ldisc.o brcm_bluesleep.o

obj-$(CONFIG_BT_V4L2)		+= brcm_bt_drv.o

obj-$(CONFIG_FM_V4L2)		+= fm_drv.o
fm_drv-objs := fmdrv_main.o fmdrv_v4l2.o fmdrv_rx.o

obj-$(CONFIG_ANT_V4L2)		+= ant_drv.o
ant_drv-objs := antdrv_main.o antdrv_v4l2.o

ccflags-y := -DVERSION="\"$(BUILD_MODULE_VERSION)\"" -DLPM_BLUESLEEP

ccflags-y += -DDEF_V4L2_FM_AUDIO_PATH=FM_AUDIO_DAC

PWD := $(shell pwd)

KERNEL_DIR=$(PWD)/../../out

CCPATH := $(PWD)/../../../aarch64-linux-android-4.9/bin/aarch64-linux-android-
EXTRA_CFLAGS := -I$(PWD)/include/

default:
	$(MAKE) ARCH=arm64 CROSS_COMPILE=$(CCPATH) -C $(KERNEL_DIR) $(EXTRA_CFLAGS) M=$(PWD) modules
clean:
	$(MAKE) -C $(KERNEL_DIR) SUBDIRS=$(PWD) clean
