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

#if !defined(LYCONFIG_COMB_FINGERPRINT_SUPPORT_null)
void fingerprint_power_en_gpio_output( int level);
void fingerprint_reset_gpio_output( int level);

int Enable_Fingerprint_Pinctrl_LOG = 1;
#define Fingerprint_Pinctrl_xlog_printk(fmt, args...) \
  do { \
    if (Enable_Fingerprint_Pinctrl_LOG) { \
      pr_notice(fmt, ##args); \
    } \
  } while (0)
struct pinctrl *fingerprint_pinctrl;
struct pinctrl_state *fingerprint_pins_default,*fingerprint_pins_irq;
struct pinctrl_state *fingerprint_reset_output0,*fingerprint_reset_output1,*fingerprint_power_en_output0,*fingerprint_power_en_output1;
static int fingerprint_pinctrl_probe(struct platform_device *pdev)
{
	int retval = 0;

	Fingerprint_Pinctrl_xlog_printk("-----------fingerprint_pinctrl_probe !-----------\n");
	
	fingerprint_pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(fingerprint_pinctrl)) {
		Fingerprint_Pinctrl_xlog_printk("Cannot find fingerprint pinctrl!\n");
	} else {
		Fingerprint_Pinctrl_xlog_printk("find fingerprint pinctrl ok!\n");
	}
	/********************************************************/
	fingerprint_pins_default = pinctrl_lookup_state(fingerprint_pinctrl, "default");
	if (IS_ERR(fingerprint_pins_default))
		Fingerprint_Pinctrl_xlog_printk( "Can *NOT* find default\n");
	else
		Fingerprint_Pinctrl_xlog_printk("Find default\n");
	
	fingerprint_pins_irq = pinctrl_lookup_state(fingerprint_pinctrl, "state_fingerprint_irq");
	if (IS_ERR(fingerprint_pins_irq))
		Fingerprint_Pinctrl_xlog_printk( "Can *NOT* find state_fingerprint_irq!\n");
	else
		Fingerprint_Pinctrl_xlog_printk("Find state_fingerprint_irq!\n");
	
	fingerprint_power_en_output0 = pinctrl_lookup_state(fingerprint_pinctrl, "state_fingerprint_power_en_output0");
	if (IS_ERR(fingerprint_power_en_output0))
		Fingerprint_Pinctrl_xlog_printk( "Can *NOT* find fingerprint_power_en_output0!\n");
	else{
		Fingerprint_Pinctrl_xlog_printk("Find fingerprint_power_en_output0!\n");
		fingerprint_power_en_gpio_output(0);
	}
	
	fingerprint_power_en_output1 = pinctrl_lookup_state(fingerprint_pinctrl, "state_fingerprint_power_en_output1");
	if (IS_ERR(fingerprint_power_en_output1))
		Fingerprint_Pinctrl_xlog_printk( "Can *NOT* find fingerprint_power_en_output1!\n");
	else
	{
		Fingerprint_Pinctrl_xlog_printk("Find fingerprint_power_en_output1!\n");
		fingerprint_power_en_gpio_output(1);
	}

	fingerprint_reset_output0 = pinctrl_lookup_state(fingerprint_pinctrl, "state_fingerprint_reset_output0");
	if (IS_ERR(fingerprint_reset_output0))
		Fingerprint_Pinctrl_xlog_printk( "Can *NOT* find fingerprint_reset_output0!\n");
	else
		Fingerprint_Pinctrl_xlog_printk("Find fingerprint_reset_output0!\n");
	
	fingerprint_reset_output1 = pinctrl_lookup_state(fingerprint_pinctrl, "state_fingerprint_reset_output1");
	if (IS_ERR(fingerprint_reset_output1))
		Fingerprint_Pinctrl_xlog_printk( "Can *NOT* find fingerprint_reset_output1!\n");
	else
		Fingerprint_Pinctrl_xlog_printk("Find fingerprint_reset_output1!\n");
	
	return retval;
}

void fingerprint_power_en_gpio_output( int level)
{
	Fingerprint_Pinctrl_xlog_printk("[Fingerprint]fingerprint_power_en_gpio_output  pin , level = %d\n",  level);
	
       if (level){
		pinctrl_select_state(fingerprint_pinctrl, fingerprint_power_en_output1);
       }
	else
		pinctrl_select_state(fingerprint_pinctrl, fingerprint_power_en_output0);
}

void fingerprint_reset_gpio_output( int level)
{
	Fingerprint_Pinctrl_xlog_printk("[Fingerprint]fingerprint_reset_gpio_output pin , level = %d\n",  level);
       if (level)
		pinctrl_select_state(fingerprint_pinctrl, fingerprint_reset_output1);
	else
		pinctrl_select_state(fingerprint_pinctrl, fingerprint_reset_output0);
}
static int fingerprint_pinctrl_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id fingerprint_of_match[] = {
	{.compatible = "mediatek,fingerprint",},
	{},
};
static struct platform_driver fingerprint_pinctrl_driver = {
	.probe = fingerprint_pinctrl_probe,
	.remove = fingerprint_pinctrl_remove,
	.driver = {
			.name = "fingerprint_pinctrl",
#ifdef CONFIG_OF				
			.of_match_table = fingerprint_of_match,
#endif			
		   },
};

struct platform_driver fingerprint_pinctrl_driver_func(void)
{
	return fingerprint_pinctrl_driver;
}

static int fingerprint_pinctrl_init(void)
{
	int ret = 0;

	Fingerprint_Pinctrl_xlog_printk("[Fingerprint]fingerprint_pinctrl_init begin!\n");
	ret = platform_driver_register(&fingerprint_pinctrl_driver);
	if (ret)
		Fingerprint_Pinctrl_xlog_printk("[Fingerprint]platform_driver_register error:(%d)\n", ret);
	else
		Fingerprint_Pinctrl_xlog_printk("[Fingerprint]platform_driver_register done!\n");

	Fingerprint_Pinctrl_xlog_printk("[Fingerprint]fingerprint_pinctrl_init done!\n");
	return ret;

}

static void fingerprint_pinctrl_exit(void)
{
	Fingerprint_Pinctrl_xlog_printk("[Fingerprint]fingerprint_pinctrl_exit\n");
	platform_driver_unregister(&fingerprint_pinctrl_driver);

	Fingerprint_Pinctrl_xlog_printk("[Fingerprint]fingerprint_pinctrl_exit Done!\n");
}

module_init(fingerprint_pinctrl_init);
module_exit(fingerprint_pinctrl_exit);
#endif
