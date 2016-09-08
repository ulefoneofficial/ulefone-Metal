#
# Makefile for misc devices that really don't fit anywhere else.
#

LYCONFIG_GPIO_FLASHLIGHT_SUPPORT :=true

ifeq ($(strip $(LYCONFIG_GPIO_FLASHLIGHT_SUPPORT)),true)
    obj-y				+= leds_gpio_strobe.o
else
	ifeq ($(strip $(LYCONFIG_COMB_CHARGER_IC)),MTK_SM5701_SUPPORT)
		#obj-y				+= leds_strobe_sm5701.o
		obj-y				+= leds_strobe.o	
	else
		ifeq ($(LYCONFIG_MTK_LM3643_SUPPORT),true)
			#obj-y				+= leds_lm3643_strobe.o
 			obj-y				+= leds_strobe.o
		else
		        obj-y				+= leds_strobe.o
		    
		endif
	endif
endif
