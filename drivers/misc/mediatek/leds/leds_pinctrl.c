#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#if defined(LYCONFIG_RGB_LEDS_SUPPORT)
int Enable_leds_Pinctrl_LOG = 1;
#define leds_Pinctrl_xlog_printk(fmt, args...) \
  do { \
    if (Enable_leds_Pinctrl_LOG) { \
      pr_notice(fmt, ##args); \
    } \
  } while (0)
struct pinctrl *leds_pinctrl;
struct pinctrl_state *leds_pins_default;
struct pinctrl_state *leds_blue_gpio_output0,*leds_blue_gpio_output1,*leds_red_gpio_output0,*leds_red_gpio_output1,*leds_green_gpio_output0,*leds_green_gpio_output1;
static int leds_pinctrl_probe(struct platform_device *pdev)
{
	int retval = 0;

	leds_Pinctrl_xlog_printk("-----------leds_pinctrl_probe !-----------\n");
	
	leds_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(leds_pinctrl)) {
		leds_Pinctrl_xlog_printk("Cannot find leds pinctrl!\n");
	} else {
		leds_Pinctrl_xlog_printk("find leds pinctrl ok!\n");
	}
	/********************************************************/
	leds_pins_default = pinctrl_lookup_state(leds_pinctrl, "default");
	if (IS_ERR(leds_pins_default))
		leds_Pinctrl_xlog_printk( "Can *NOT* find default\n");
	else
		leds_Pinctrl_xlog_printk("Find default\n");
	

	leds_blue_gpio_output0 = pinctrl_lookup_state(leds_pinctrl, "state_leds_blue_gpio_output0");
	if (IS_ERR(leds_blue_gpio_output0))
		leds_Pinctrl_xlog_printk( "Can *NOT* find state_leds_blue_gpio_output0!\n");
	else
		leds_Pinctrl_xlog_printk("Find state_leds_blue_gpio_output0!\n");
	
	leds_blue_gpio_output1 = pinctrl_lookup_state(leds_pinctrl, "state_leds_blue_gpio_output1");
	if (IS_ERR(leds_blue_gpio_output1))
		leds_Pinctrl_xlog_printk( "Can *NOT* find state_leds_blue_gpio_output1!\n");
	else
		leds_Pinctrl_xlog_printk("Find state_leds_blue_gpio_output1!\n");

	leds_red_gpio_output0 = pinctrl_lookup_state(leds_pinctrl, "state_leds_red_gpio_output0");
	if (IS_ERR(leds_red_gpio_output0))
		leds_Pinctrl_xlog_printk( "Can *NOT* find state_leds_red_gpio_output0!\n");
	else
		leds_Pinctrl_xlog_printk("Find state_leds_red_gpio_output0!\n");
	
	leds_red_gpio_output1 = pinctrl_lookup_state(leds_pinctrl, "state_leds_red_gpio_output1");
	if (IS_ERR(leds_red_gpio_output1))
		leds_Pinctrl_xlog_printk( "Can *NOT* find state_leds_red_gpio_output1!\n");
	else
		leds_Pinctrl_xlog_printk("Find state_leds_red_gpio_output1!\n");

	leds_green_gpio_output0 = pinctrl_lookup_state(leds_pinctrl, "state_leds_green_gpio_output0");
	if (IS_ERR(leds_green_gpio_output0))
		leds_Pinctrl_xlog_printk( "Can *NOT* find state_leds_green_gpio_output0!\n");
	else
		leds_Pinctrl_xlog_printk("Find state_leds_green_gpio_output0!\n");
	
	leds_green_gpio_output1 = pinctrl_lookup_state(leds_pinctrl, "state_leds_green_gpio_output1");
	if (IS_ERR(leds_green_gpio_output1))
		leds_Pinctrl_xlog_printk( "Can *NOT* find state_leds_green_gpio_output1!\n");
	else
		leds_Pinctrl_xlog_printk("Find state_leds_green_gpio_output1!\n");
	
	return retval;
}

void leds_blue_en_gpio_output( int level)
{
	leds_Pinctrl_xlog_printk("[leds]leds_blue_en_gpio_output  pin , level = %d\n",  level);
	
       if (level){
	   	if (!IS_ERR(leds_blue_gpio_output1))
		pinctrl_select_state(leds_pinctrl, leds_blue_gpio_output1);
       }
	else{
		if (!IS_ERR(leds_blue_gpio_output0))
		pinctrl_select_state(leds_pinctrl, leds_blue_gpio_output0);
	}
}

void leds_red_en_gpio_output( int level)
{
	leds_Pinctrl_xlog_printk("[leds]leds_read_en_gpio_output  pin , level = %d\n",  level);
	
       if (level){
	   	if (!IS_ERR(leds_red_gpio_output1))
		pinctrl_select_state(leds_pinctrl, leds_red_gpio_output1);
       }
	else{
			if (!IS_ERR(leds_red_gpio_output0))
		pinctrl_select_state(leds_pinctrl, leds_red_gpio_output0);
	}
}
void leds_green_en_gpio_output( int level)
{
	leds_Pinctrl_xlog_printk("[leds]leds_green_en_gpio_output  pin , level = %d\n",  level);
	
       if (level){
	   	if (!IS_ERR(leds_green_gpio_output1))
		pinctrl_select_state(leds_pinctrl, leds_green_gpio_output1);
       }
	else{
		if (!IS_ERR(leds_green_gpio_output0))
		pinctrl_select_state(leds_pinctrl, leds_green_gpio_output0);
	}
}

static int leds_pinctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id leds_of_match[] = {
	{.compatible = "mediatek,ledspinctrl",},
	{},
};
static struct platform_driver leds_pinctrl_driver = {
	.probe = leds_pinctrl_probe,
	.remove = leds_pinctrl_remove,
	.driver = {
			.name = "leds_pinctrl",
#ifdef CONFIG_OF				
			.of_match_table = leds_of_match,
#endif			
		   },
};

struct platform_driver leds_pinctrl_driver_func(void)
{
	return leds_pinctrl_driver;
}

static int leds_pinctrl_init(void)
{
	int ret = 0;

	leds_Pinctrl_xlog_printk("[leds]leds_pinctrl_init begin!\n");
	ret = platform_driver_register(&leds_pinctrl_driver);
	if (ret)
		leds_Pinctrl_xlog_printk("[leds]platform_driver_register error:(%d)\n", ret);
	else
		leds_Pinctrl_xlog_printk("[leds]platform_driver_register done!\n");

	leds_Pinctrl_xlog_printk("[leds]leds_pinctrl_init done!\n");
	return ret;

}

static void leds_pinctrl_exit(void)
{
	leds_Pinctrl_xlog_printk("[leds]leds_pinctrl_exit\n");
	platform_driver_unregister(&leds_pinctrl_driver);

	leds_Pinctrl_xlog_printk("[leds]leds_pinctrl_exit Done!\n");
}

module_init(leds_pinctrl_init);
module_exit(leds_pinctrl_exit);
#endif
