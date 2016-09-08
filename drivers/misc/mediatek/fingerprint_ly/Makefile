#
# Makefile for the finger drivers. added by xiehaifei 150508
#
# Each configuration option enables a list of files.
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include

ifneq ($(LYCONFIG_COMB_FINGERPRINT_SUPPORT),null)
obj-y	+= fingerprint_pinctrl.o
endif

ifeq ($(LYCONFIG_COMB_FINGERPRINT_SUPPORT),AFS120)
obj-$(CONFIG_LY_FINGERPRINT_AFS120_SUPPORT) += afs120/
endif

ifeq ($(LYCONFIG_COMB_FINGERPRINT_SUPPORT),GFX1XM)
obj-$(CONFIG_LY_FINGERPRINT_GFX1XM_SUPPORT) += gfx1xm/
endif

ifeq ($(LYCONFIG_COMB_FINGERPRINT_SUPPORT),ET310)
obj-$(CONFIG_LY_FINGERPRINT_ET310_SUPPORT) += et310/
endif
