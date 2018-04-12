#ifndef FPGA_H_
#define FPGA_H_

#ifdef __KERNEL__
#include <rtdm/rtdm.h>
#include <rtdm/rtdm_driver.h>

#else
/* This one for user space */
#define PDEBUG(fmt, args...) do { \
	if (unlikely(fpga_debug)) { \
		fprintf(stderr, fmt, ## args); \
	} \
} while (0)
#endif

#ifdef __KERNEL__

struct rt_fpga_info {
	const char *name;
	struct platform_device *pdev;
	int int_pin;
	int irq;
	int reset_pin;
	// rtdm irq handle
	rtdm_irq_t int_irq_handle;
	// rt irq event
	rtdm_event_t irq_event;
};

#endif

// 定义ioctl命令
#define FPGA_IOBASE 'F'
#define RT_FPGA_START _IO(FPGA_IOBASE, 0)
#define RT_FPGA_WAIT_IRQ _IO(FPGA_IOBASE, 2)

#endif
