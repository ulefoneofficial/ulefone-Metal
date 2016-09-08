/*
 *  drivers/switch/switch_gpio.c
 *
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/input.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#endif

#include <mach/gpio_const.h>
#include <mt-plat/mt_gpio.h>

struct gpio_switch_data {
	struct switch_dev sdev;
	unsigned gpio;
	const char *name_on;
	const char *name_off;
	const char *state_on;
	const char *state_off;
	struct device_node *irq_node;
	int irq;
	struct work_struct work;
};


#ifdef CONFIG_OF
static const struct of_device_id mhall_of_match[] = {
	{.compatible = "mediatek, MHALL-eint",},
	{},
};
#endif

extern struct input_dev *kpd_input_dev;
static unsigned int gpiopin, debounce;
//#define GPIO_HALL_1_PIN  (GPIO5)

static void gpio_switch_work(struct work_struct *work)
{
	int state;
	struct gpio_switch_data	*data =
		container_of(work, struct gpio_switch_data, work);
	
	state = gpio_get_value(gpiopin); //mt_get_gpio_in(GPIO_HALL_1_PIN);

	printk("[Hall]:  gpio_switch_work state = %d\n", state);	

#if 0
	if (kpd_input_dev != NULL)
	{
		input_report_key(kpd_input_dev, KEY_POWER, 1);

		input_report_key(kpd_input_dev, KEY_POWER, 0);
		input_sync(kpd_input_dev);
	}
#endif

	if (kpd_input_dev != NULL)
	{
		switch (state)
		{
			case 1:
				printk("[Hall]:  KEY_F10!\n");
				input_report_key(kpd_input_dev, KEY_F10, 1);
				input_report_key(kpd_input_dev, KEY_F10, 0);
				input_sync(kpd_input_dev);
				break;
			case 0:
				printk("[Hall]:  KEY_F9!\n");
				input_report_key(kpd_input_dev, KEY_F9, 1);
				input_report_key(kpd_input_dev, KEY_F9, 0);
				input_sync(kpd_input_dev);
				break;
			default:
				break;
		}		
	}

	if(state)
		irq_set_irq_type(data->irq, IRQ_TYPE_LEVEL_LOW);
	else
		irq_set_irq_type(data->irq, IRQ_TYPE_LEVEL_HIGH);
	
	enable_irq(data->irq);
	
	switch_set_state(&data->sdev, state);
}


static ssize_t switch_gpio_print_state(struct switch_dev *sdev, char *buf)
{
	struct gpio_switch_data	*switch_data = container_of(sdev, struct gpio_switch_data, sdev);
	const char *state;

	
	if (switch_get_state(sdev))
		state = switch_data->state_on;
	else
		state = switch_data->state_off;

	if (state)
		return sprintf(buf, "%s\n", state);
	return -1;
}


static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	struct gpio_switch_data *switch_data = (struct gpio_switch_data *)dev_id;
	
	disable_irq_nosync(switch_data->irq);
	
	schedule_work(&switch_data->work);
	
	return IRQ_HANDLED;
}

static int gpio_switch_probe(struct platform_device *pdev)
{
	struct gpio_switch_data *switch_data;
	int ret = -1;
	//truct pinctrl *pinctrl;
	//struct pinctrl_state *pins_default;
	//struct pinctrl_state *pins_eint_int;
	u32 ints[2] = {0, 0};

	switch_data = kzalloc(sizeof(struct gpio_switch_data), GFP_KERNEL);
	if (!switch_data){
		printk("[Hall]: kzalloc is failed\n");
		return -ENOMEM;
	}

	switch_data->sdev.name = "mhall";
	switch_data->sdev.print_state = switch_gpio_print_state;

	ret = switch_dev_register(&switch_data->sdev);
	if (ret < 0)
	{
		printk("[Hall]: switch_dev_register is failed\n");
		goto err_switch_dev_register;
	}

	INIT_WORK(&switch_data->work, gpio_switch_work);
	
	/* gpio setting */
/*	
	pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		ret = PTR_ERR(pinctrl);
		printk("[Hall]: Cannot find hall pinctrl!\n");
	}
	pins_default = pinctrl_lookup_state(pinctrl, "pin_default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		printk("[Hall]: Cannot find hall pinctrl default!\n");

	}

	pins_eint_int = pinctrl_lookup_state(pinctrl, "state_eint_as_int");
	if (IS_ERR(pins_eint_int)) {
		ret = PTR_ERR(pins_eint_int);
		printk("[Hall]: Cannot find hall pinctrl pins_eint_int!\n");

	}
	pinctrl_select_state(pinctrl, pins_eint_int);
*/
	switch_data->irq_node = of_find_compatible_node(NULL, NULL, "mediatek, MHALL-eint");

	if (switch_data->irq_node) {
		of_property_read_u32_array(switch_data->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		gpiopin = ints[0];
		debounce = ints[1];
		printk("[Hall]: gpiopin = %d,debounce=%d\n", gpiopin,debounce);
		
		switch_data->irq = irq_of_parse_and_map(switch_data->irq_node, 0);
		
		printk("[Hall]: switch_data->irq = %d\n", switch_data->irq);
		
		if (!switch_data->irq) {
			printk("[Hall]: irq_of_parse_and_map fail!!\n");
			goto err_detect_irq_num_failed;
		}

		ret = request_irq(switch_data->irq, gpio_irq_handler, IRQF_TRIGGER_NONE, "MHALL-eint", switch_data);
		if (ret < 0) 
		{
			printk("[Hall]: IRQ LINE NOT AVAILABLE!!\n");
			goto err_detect_irq_num_failed;
		}
	
	    } else {
			printk("[Hall]: null irq node!!\n");
			goto err_detect_irq_node_failed;
	}


	/* Perform initial detection */
	gpio_switch_work(&switch_data->work);

	printk("[Hall]: %s End\n", __func__);
	return 0;


err_detect_irq_num_failed:
err_detect_irq_node_failed:
	gpio_free(switch_data->gpio);
	switch_dev_unregister(&switch_data->sdev);
err_switch_dev_register:
	kfree(switch_data);	
	return ret;
}

static int gpio_switch_remove(struct platform_device *pdev)
{

	struct gpio_switch_data *switch_data = platform_get_drvdata(pdev);

	free_irq(switch_data->irq, pdev);
	
	gpio_free(switch_data->gpio);
	
	cancel_work_sync(&switch_data->work);
	
	switch_dev_unregister(&switch_data->sdev);

	kfree(switch_data);

	return 0;
}


static struct platform_driver gpio_switch_driver = {
	.probe		= gpio_switch_probe,
	.remove		= gpio_switch_remove,
	.driver		= {
		.name	= "mhall",
#ifdef CONFIG_OF
	 .of_match_table = mhall_of_match,
#endif		
	},
};

static int __init gpio_switch_init(void)
{
	int ret = 0;
	
	ret = platform_driver_register(&gpio_switch_driver);
	if (ret)
		printk("[Hall]: gpio_switch_driver register error:(%d)\n", ret);


	return ret;
}

static void __exit gpio_switch_exit(void)
{
	platform_driver_unregister(&gpio_switch_driver);
}

module_init(gpio_switch_init);
module_exit(gpio_switch_exit);

MODULE_AUTHOR("xinchao");
MODULE_DESCRIPTION("GPIO Switch driver");
MODULE_LICENSE("GPL");
