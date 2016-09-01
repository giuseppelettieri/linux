#include <linux/types.h>
#include <linux/module.h>
#include <linux/aio.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/delay.h>


static int instances = 0; /* To be protected by a lock. */

#define EMULATE
#define PSPAT_QLEN           128

struct pspat {
	____cacheline_aligned_in_smp
	struct sk_buff      *inq[PSPAT_QLEN];

	____cacheline_aligned_in_smp
	struct sk_buff      *outq[PSPAT_QLEN];

	wait_queue_head_t wqh;
#ifdef EMULATE
	struct timer_list emu_tmr;
#endif
};

#ifdef EMULATE
/* These functions should be moved to the dev core module. */

static void
emu_tmr_cb(long unsigned arg)
{
	struct pspat *arb = (struct pspat *)arg;

	wake_up_interruptible(&arb->wqh);
	mod_timer(&arb->emu_tmr, jiffies + msecs_to_jiffies(1000));
}

int
pspat_arbiter_register(struct pspat *arb)
{
	arb->emu_tmr.function = emu_tmr_cb;
	arb->emu_tmr.data = (long unsigned)arb;
	mod_timer(&arb->emu_tmr, jiffies + msecs_to_jiffies(1000));

	return 0;
}

int
pspat_arbiter_unregister(struct pspat *arb)
{
	del_timer_sync(&arb->emu_tmr);

	return 0;
}
#endif

static int
pspat_open(struct inode *inode, struct file *f)
{
	struct pspat *arb;
	int ret;

	if (instances) {
		printk("PSPAT arbiter already exists\n");
		return -EBUSY;
	}

	arb = kzalloc(sizeof(*arb), GFP_KERNEL);
	if (!arb) {
		return -ENOMEM;
	}

	init_waitqueue_head(&arb->wqh);
	f->private_data = arb;

	ret = pspat_arbiter_register(arb);
	if (ret) {
		printk("Failed to register arbiter\n");
		kfree(arb);
		return ret;
	}

	instances ++;

	return 0;
}

static int
pspat_release(struct inode *inode, struct file *f)
{
	struct pspat *arb = (struct pspat *)f->private_data;

	pspat_arbiter_unregister(arb);

	kfree(arb);
	f->private_data = NULL;

	instances --;

	return 0;
}

static long
pspat_ioctl(struct file *f, unsigned int cmd, unsigned long flags)
{
	struct pspat *arb = (struct pspat *)f->private_data;
	DECLARE_WAITQUEUE(wait, current);

	(void) cmd;

	add_wait_queue(&arb->wqh, &wait);

	for (;;) {
		current->state = TASK_INTERRUPTIBLE;
		schedule();
		if (signal_pending(current)) {
			printk("Got a signal, returning to userspace\n");
			return 0;
		}
		current->state = TASK_RUNNING;
		printk("Woken up\n");
	}

	remove_wait_queue(&arb->wqh, &wait);

	return 0;
}

static const struct file_operations pspat_fops = {
	.owner          = THIS_MODULE,
	.release        = pspat_release,
	.open           = pspat_open,
	.unlocked_ioctl = pspat_ioctl,
	.llseek         = noop_llseek,
};

static struct miscdevice pspat_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "pspat",
	.fops = &pspat_fops,
};

static int __init
pspat_init(void)
{
	int ret;

	ret = misc_register(&pspat_misc);
	if (ret) {
		printk("Failed to register rlite misc device\n");
		return ret;
	}

	return 0;
}

static void __exit
pspat_fini(void)
{
	misc_deregister(&pspat_misc);
}

module_init(pspat_init);
module_exit(pspat_fini);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Vincenzo Maffione <v.maffione@gmail.com>");
