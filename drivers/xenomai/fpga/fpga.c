#include <linux/module.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/consumer.h>
#include <linux/interrupt.h>

#include <asm/mach/irq.h>

/*
 * FPGA驱动程序
 * 1. INT中断
 * 2. RESET GPIO
 * 地址映射GPMC已经处理
 */

#include "fpga.h"
#define FPGA_NAME "fpga"
#define FPGA_VERSION "rt v1.x"

static const struct of_device_id fpga_of_match[] = {
	{
		.compatible = "fpga",
	},
	{ },
};
MODULE_DEVICE_TABLE(of, fpga_of_match);

static void rt_fpga_reset(struct rt_fpga_info *info)
{
	if (info->reset_pin < 0) {
		return;
	}
	gpio_set_value(info->reset_pin, 0);
	msleep(1);
	gpio_set_value(info->reset_pin, 1);
}

static int rt_fpga_interrupt(rtdm_irq_t *irq_handle)
{
	struct rt_fpga_info *info;
	struct irq_desc *desc;
	struct irq_chip *chip;

	info = rtdm_irq_get_arg(irq_handle, struct rt_fpga_info);
	desc = irq_to_desc(info->irq);
	chip = desc->irq_data.chip;
	if (chip->irq_ack) {
		chip->irq_ack(&desc->irq_data);
	}
	rtdm_event_signal(&info->irq_event);
	return RTDM_IRQ_HANDLED;
}

// enable gpio irq
static int rt_fpga_irq_config(struct rt_fpga_info *info)
{
	struct platform_device *pdev = info->pdev;
	struct irq_desc *desc = irq_to_desc(info->irq);
	struct irq_chip *chip;
	if (desc == NULL) {
		dev_err(&pdev->dev, "rt_fpga_irq_config irq to desc failed\n");
		return -1;
	}
	// set up the irq type
	chip = desc->irq_data.chip;
	if (!chip || !chip->irq_set_type) {
		dev_err(&pdev->dev, "No set_type function for IRQ %d (%s)\n", info->irq,
			chip ? (chip->name ? : "unknown") : "unknown");
		return 0;
	}
	//chip->irq_set_type(&desc->irq_data, IRQF_TRIGGER_FALLING);
	// desc->handler = NULL??
	irqd_set_trigger_type(&desc->irq_data, IRQF_TRIGGER_RISING);
	if (chip->irq_enable) {
		chip->irq_enable(&desc->irq_data);
	} else {
		chip->irq_unmask(&desc->irq_data);
	}
	return 0;
}

static int rt_fpga_irq_stop(struct rt_fpga_info *info)
{
	struct platform_device *pdev = info->pdev;
	struct irq_desc *desc = irq_to_desc(info->irq);
	if (desc == NULL) {
		dev_err(&pdev->dev, "rt_fpga_irq_config irq to desc failed\n");
		return -1;
	}
	if (desc->irq_data.chip->irq_disable) {
		desc->irq_data.chip->irq_disable(&desc->irq_data);
	} else {
		desc->irq_data.chip->irq_mask(&desc->irq_data);
	}
	if (desc->irq_data.chip->irq_mask_ack) {
		desc->irq_data.chip->irq_mask_ack(&desc->irq_data);
	}
	return 0;
}

static int rt_fpga_open(struct rtdm_dev_context *context,
				rtdm_user_info_t * user_info, int oflags)
{
	struct rt_fpga_info *info = (struct rt_fpga_info *)context->dev_private;

	rt_fpga_reset(info);
	// rt irq event init
	rtdm_event_init(&info->irq_event, 0);

	dev_info(&info->pdev->dev, "fpga_open ok\n");
	
	return 0;
}

static int rt_fpga_close(struct rtdm_dev_context *context,
				rtdm_user_info_t * user_info)
{
	struct rt_fpga_info *info = (struct rt_fpga_info *)context->dev_private;
	
	rt_fpga_irq_stop(info);
	rtdm_irq_free(&info->int_irq_handle);

	// rt irq event destory
	rtdm_event_destroy(&info->irq_event);
	return 0;
}

static ssize_t rt_fpga_read_rt(struct rtdm_dev_context *context,
				    rtdm_user_info_t * user_info, void *buf,
				    size_t nbyte)
{
	return 0;
}

static ssize_t rt_fpga_write_rt(struct rtdm_dev_context *context,
				     rtdm_user_info_t * user_info,
				     const void *buf, size_t nbyte)
{
	return 0;
}



static int rt_fpga_ioctl_rt(struct rtdm_dev_context *context,
				rtdm_user_info_t *user_info,
				unsigned int request, void __user *arg)
{
	struct rt_fpga_info *info = (struct rt_fpga_info *)context->dev_private;
	int ret;
	rtdm_toseq_t timeout_seq;
	nanosecs_rel_t event_timeout = 8 * 1000 * 1000;
	
	switch (request) {
		case RT_FPGA_START:
			// 实时中断注册
			ret = rtdm_irq_request(&info->int_irq_handle, info->irq,
				rt_fpga_interrupt, /* RTDM_IRQTYPE_EDGE */0, "fpga_int", info);
			if (ret) {
				dev_err(&info->pdev->dev, "rt_jl098_open: rtdm_irq_request int irq %d failed %d\n", info->irq, ret);
				return ret;
			}
			rt_fpga_irq_config(info);
			dev_info(&info->pdev->dev, "fpga start %d ok\n", info->irq);
			break;
		case RT_FPGA_WAIT_IRQ:
			if (arg) {
				struct timespec ts;
				ret = rtdm_safe_copy_from_user(user_info, &ts, arg, sizeof(struct timespec));
				if (ret < 0)
					return ret;
				event_timeout = ts.tv_sec * 1000000000 + ts.tv_nsec;
				//dev_info(&info->pdev->dev, "fpga wait %llu ok\n", event_timeout);
			}
			rtdm_toseq_init(&timeout_seq, event_timeout);
			// 8 ms timeout
			ret = rtdm_event_timedwait(&info->irq_event,
				event_timeout, &timeout_seq);
			if (ret < 0) {
				//dev_info(&info->pdev->dev, "rtdm_event_timedwait return %d\n", ret);
				return ret;
			}
			break;
		default:
			return -EINVAL;
	}
	return ret;
}

static struct rtdm_device rt_fpga_device = {
	.struct_version    = RTDM_DEVICE_STRUCT_VER,

	.device_flags      = RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
	.context_size      = sizeof(struct rt_fpga_info),
	.device_name       = "rt_fpga_dev",

	.open_nrt = rt_fpga_open,
	//.open_rt = rt_jl098_open,

	.ops = {
		.close_nrt = rt_fpga_close,

		.read_rt   = rt_fpga_read_rt,
		.write_rt  = rt_fpga_write_rt,
		.ioctl_rt  = rt_fpga_ioctl_rt,
	},

	.device_class      = RTDM_CLASS_EXPERIMENTAL,
	.device_sub_class  = 0,
	.profile_version   = 1,
	.driver_name       = "rt_fpga_drv",
	.driver_version    = RTDM_DRIVER_VER(0, 1, 1),
	.peripheral_name   = "rt fpga for am335x",
	.provider_name     = "wjzhe",
	.proc_name         = rt_fpga_device.device_name,
};

static int fpga_probe(struct platform_device *pdev)
{
	struct rt_fpga_info *info;
	struct device_node *pnode = pdev->dev.of_node;
	struct pinctrl *pinctrl;
	int ret;

	/* we only support OF */
	if (pnode == NULL) {
		dev_err(&pdev->dev, "No platform of_node!\n");
		return -ENODEV;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		/* special handling for probe defer */
		if (PTR_ERR(pinctrl) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		dev_warn(&pdev->dev,
			"pins are not configured from the driver\n");
	}
	// rt device register
	// 实时驱动注册
	if ((ret = rtdm_dev_register(&rt_fpga_device)) < 0) {
		dev_err(&pdev->dev, "fpga init rtdm_dev_register failed %d\n", ret);
		return ret;
	}

	info = (struct rt_fpga_info *)rt_fpga_device.reserved.exclusive_context->dev_private;;
	if (info == NULL) {
		dev_err(&pdev->dev, "Failed to get info\n");
		ret = -ENOMEM;
		goto err_no_mem;
	}
	memset(info, 0, sizeof(*info));
	info->pdev = pdev;

	info->int_pin = of_get_named_gpio(pnode, "int-gpios", 0);
	info->reset_pin = of_get_named_gpio(pnode, "reset-gpios", 0);
	if (!gpio_is_valid(info->int_pin)) {
		dev_err(&pdev->dev, "No int pin!\n");
		goto err_no_gpio;
	}
	if (!gpio_is_valid(info->reset_pin)) {
		dev_err(&pdev->dev, "No reset pin!\n");
		info->reset_pin = -1;
		//goto err_no_gpio;
	} else {
		ret = gpio_request(info->reset_pin, "fpga-reset");
		if (ret) {
			dev_err(&pdev->dev, "gpio request reset-pin failed %d\n", ret);
			return ret;
		}
		ret = gpio_direction_output(info->reset_pin, 1);
		if (ret) {
			dev_err(&pdev->dev, "gpio direction input reset-pin failed %d\n", ret);
			return ret;
		}
	}

	/* irq, gpio init */
	info->irq = gpio_to_irq(info->int_pin);
	ret = gpio_request(info->int_pin, "fpga-int");
	if (ret) {
		dev_err(&pdev->dev, "gpio request int-pin failed %d\n", ret);
		return ret;
	}
	ret = gpio_direction_input(info->int_pin);
	if (ret) {
		dev_err(&pdev->dev, "gpio direction input int-pin failed %d\n", ret);
		return ret;
	}
	
	dev_info(&pdev->dev, "ready, int-pin %d, reset-pin %d, irq %d\n",
		info->int_pin, info->reset_pin, info->irq);
	return 0;
	
err_no_gpio:
err_no_mem:
	return ret;
}

static int fpga_remove(struct platform_device *pdev)
{
	struct rt_fpga_info *info;
	int ret;
	info = (struct rt_fpga_info *)rt_fpga_device.reserved.exclusive_context->dev_private;

	gpio_free(info->int_pin);
	if (info->reset_pin >= 0) {
		gpio_free(info->reset_pin);
	}
	
	dev_info(&pdev->dev, "removing\n");
	
	if ((ret = rtdm_dev_unregister(&rt_fpga_device, 100)) < 0) {
		printk("fpga exit rtdm_dev_unregister failed %d\n", ret);
	}

	return 0;
}

struct platform_driver fpga_driver = {
	.probe		= fpga_probe,
	.remove		= fpga_remove,
	.driver = {
		.name		= "rt_fpga",
		.owner		= THIS_MODULE,
		.of_match_table	= fpga_of_match,
	},
};

module_platform_driver(fpga_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("wjzhe");
MODULE_DESCRIPTION("FPGA DRIVER FOR AGV");
MODULE_ALIAS("platform:fpga");

