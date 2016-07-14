#ifndef JL098B_H_
#define JL098B_H_

#ifdef __KERNEL__
#include <rtdm/rtdm.h>
#include <rtdm/rtdm_driver.h>

#else
/* This one for user space */
#define PDEBUG(fmt, args...) do { \
	if (unlikely(jl098b_debug)) { \
		fprintf(stderr, fmt, ## args); \
	} \
} while (0)
#endif

#ifdef __KERNEL__

struct rt_jl098b_info {
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

// ∂®“Âioctl√¸¡Ó
#define JL098B_IOBASE 'J'
#define RT_JL098B_START _IO(JL098B_IOBASE, 0)
#define RT_JL098B_SETMODE _IOW(JL098B_IOBASE, 1, int)
#define RT_JL098B_WAIT_IRQ _IO(JL098B_IOBASE, 2)
#define RT_JL098B_SETGPG2 _IOW(JL098B_IOBASE, 3, int)
#define RT_JL098B_SETGPG5 _IOW(JL098B_IOBASE, 4, int)
#define RT_JL098B_GETBASE _IOR(JL098B_IOBASE, 5, int)

#define RT_JL098B_SETGCR1 _IOW(JL098B_IOBASE, 6, int)
#define RT_JL098B_SETGCR2 _IOW(JL098B_IOBASE, 7, int)
#define RT_JL098B_SETRCR1 _IOW(JL098B_IOBASE, 8, int)
#define RT_JL098B_SETRCR2 _IOW(JL098B_IOBASE, 9, int)
#define RT_JL098B_SETWCR _IOW(JL098B_IOBASE, 10, int)
#define RT_JL098B_SETWCR1 _IOW(JL098B_IOBASE, 11, int)
#define RT_JL098B_SETWCR2 _IOW(JL098B_IOBASE, 12, int)
#define RT_JL098B_BUSC _IO(JL098B_IOBASE, 13)

#endif
