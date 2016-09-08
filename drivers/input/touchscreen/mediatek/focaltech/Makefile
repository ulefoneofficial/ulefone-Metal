# Linux driver folder
MTK_PLATFORM := $(subst ",,$(CONFIG_MTK_PLATFORM))
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/

obj-y	+=  focaltech_core.o
obj-y	+=  focaltech_ctl.o
obj-y	+=  focaltech_ex_fun.o
obj-y	+=  focaltech_flash.o
obj-y	+=  focaltech_Gesture.o

obj-y += ft_gesture_lib.a

