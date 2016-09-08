# Linux driver folder
include $(srctree)/../resource/LyFeatures.mk
ccflags-y += -I$(srctree)/drivers/misc/mediatek/mach/$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GT1X/
ccflags-y += -I$(srctree)/arch/arm/mach-$(MTK_PLATFORM)/$(ARCH_MTK_PROJECT)/touchpanel/GT1X/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/GT1X/
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include/

obj-y	+=  gt1x_extents.o
obj-y	+=  gt1x_generic.o
obj-y	+=  gt1x_tools.o
obj-y	+=  gt1x_tpd.o
obj-y	+=  gt1x_update.o

