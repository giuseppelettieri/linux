#ifndef __PSPAT_H__
#define __PSPAT_H__

#include <linux/jiffies.h>
#define	time_second	(jiffies_to_msecs(jiffies) / 1000U )

#define START_NEW_CACHELINE	____cacheline_aligned_in_smp

//#define EMULATE
#define PSPAT_QLEN           8

struct pspat_queue {
	/* Input queue, written by clients, read by the arbiter. */
	START_NEW_CACHELINE
	struct sk_buff		*inq[PSPAT_QLEN];

	/* Data structures private to the clients. */
	START_NEW_CACHELINE
	uint32_t		cli_inq_tail; /* insertion point in inq  */
	uint32_t		cli_outq_head; /* extraction point from outq */

	/* Output queue, written by the arbiter, read by senders. */
	START_NEW_CACHELINE
	struct sk_buff		*outq[PSPAT_QLEN];

	/* Data structures private to the arbiter. */
	START_NEW_CACHELINE
	uint32_t		arb_outq_tail; /* insertion point in outq  */
	uint32_t		arb_inq_head; /* extraction point from inq */
	uint32_t		arb_inq_ntc;  /* next to clean */
	uint32_t		arb_cacheq_tail;
	uint32_t		arb_cacheq_head;
	uint32_t		arb_markq_tail;
	uint32_t		arb_markq_head;
	s64			arb_extract_next;
	uint32_t		arb_pending; /* packets in the Qdisc */
	int			arb_inq_full;

	struct sk_buff		*cacheq[PSPAT_QLEN];
	struct sk_buff		*markq[PSPAT_QLEN];

	/* Data structures private to the sender. */
	START_NEW_CACHELINE
	uint32_t		snd_outq_head; /* extraction point from outq  */
};

struct pspat {
	wait_queue_head_t wqh;
	struct Qdisc	       *qdiscs;
#ifdef EMULATE
	struct timer_list	emu_tmr;
#endif
	int			n_queues;
	struct pspat_queue	queues[0];
};

extern struct pspat *pspat_arb;

int pspat_do_arbiter(struct pspat *arb);

int pspat_client_handler(struct sk_buff *skb, struct Qdisc *q,
	              struct net_device *dev, struct netdev_queue *txq);
void pspat_shutdown(struct pspat *arb);

int pspat_do_sender(struct pspat *arb);

extern int pspat_enable;
extern int pspat_debug_xmit;
extern int pspat_direct_xmit;
extern u64 pspat_rate;
extern s64 pspat_arb_interval_ns;
extern uint32_t pspat_qdisc_batch_limit;
extern struct pspat_stats *pspat_stats;

struct pspat_stats {
	unsigned long dropped;
} __attribute__((aligned(32)));

#define ND(format, ...)
#define D(format, ...)						\
	do {							\
		struct timeval __xxts;				\
		do_gettimeofday(&__xxts);			\
		printk(KERN_ERR "%03d.%06d [%4d] %-25s " format "\n",	\
		(int)__xxts.tv_sec % 1000, (int)__xxts.tv_usec,	\
		__LINE__, __FUNCTION__, ##__VA_ARGS__);		\
	} while (0)

/* rate limited, lps indicates how many per second */
#define RD(lps, format, ...)					\
	do {							\
		static int t0, __cnt;				\
		if (t0 != time_second) {			\
			t0 = time_second;			\
			__cnt = 0;				\
		}						\
		if (__cnt++ < lps)				\
			D(format, ##__VA_ARGS__);		\
	} while (0)

#endif  /* __PSPAT_H__ */
