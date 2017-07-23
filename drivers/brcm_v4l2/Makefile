#
# Makefile for Broadcom shared uart transport solution
#
obj-$(CONFIG_SH_V4L2)		+= brcm_hci_ldisc.o
brcm_hci_ldisc-objs := brcm_hci.o brcm_sh_ldisc.o brcm_bluesleep.o

obj-$(CONFIG_BT_V4L2)		+= brcm_bt_drv.o

obj-$(CONFIG_ANT_V4L2)		+= ant_drv.o
ant_drv-objs := antdrv_main.o antdrv_v4l2.o

ccflags-y := -DVERSION="\"$(BUILD_MODULE_VERSION)\"" -DLPM_BLUESLEEP


