#ifndef __PSPAT_H__
#define __PSPAT_H__

/*
CODE ROADMAP

PSPAT runs schedulers in a private thread, using one
lockless, single-producer, single-consumer (LSPSC) queue
as a mailbox between each thread and the scheduler thread,
and one 'pspat_pcpu_queue' per cpu indicating mailboxes with
potential pending traffic on that CPU.

mailbox.h
mailbox.c
	LSPSC queues

pspat.h
	pspat_pcpu_queue
	main descriptor for the pspat object
	common prototypes.

pspat_main.c

 */

#include "mailbox.h"

/* per-cpu data structure */
// XXX explain how it works
struct pspat_pcpu_queue {
	/* Input queue, a mailbox of mailbox pointers.
	 * written by clients, read by the arbiter. */
	struct pspat_mailbox   *inq;

	/* client fields */
	START_NEW_CACHELINE
	struct pspat_mailbox   *cli_last_mb;

	/* arbiter fields */
	START_NEW_CACHELINE
	u64			arb_extract_next;
	struct pspat_mailbox   *arb_last_mb;
	struct list_head	mb_to_clear;
};

struct pspat {
	struct task_struct	*arb_task;
	struct task_struct	*snd_task; // XXX how many?

	/* list of all the qdiscs that we stole from the system */
	struct Qdisc	       *qdiscs;

	struct Qdisc		bypass_qdisc;
	struct list_head	active_txqs;
	int			n_queues;
	struct pspat_pcpu_queue	queues[0];
};

// XXX per-cpu stats, use a sysctl vector
struct pspat_stats {
	unsigned long inq_drop;
} __attribute__((aligned(32)));

extern struct pspat *pspat_arb;

int pspat_do_arbiter(struct pspat *arb);

int pspat_client_handler(struct sk_buff *skb, struct Qdisc *q,
	              struct net_device *dev, struct netdev_queue *txq);
void pspat_shutdown(struct pspat *arb);

int pspat_do_sender(struct pspat *arb);

int pspat_create_client_queue(void);

/* sysctl to control operation of PSPAT */

extern int pspat_enable;		/* toggle PSPAT on/off */
extern int pspat_debug_xmit;
#define PSPAT_XMIT_MODE_ARB		0 /* PSPAT also transmits */
#define PSPAT_XMIT_MODE_DISPATCH	1 /* use external dispatcher threads */
#define PSPAT_XMIT_MODE_MAX		2 /* PSPAT drops (test only) */
extern int pspat_xmit_mode;
extern int pspat_single_txq;		/* force a single txq to the device */
extern int pspat_tc_bypass;		/* performance testing. 0 for normal ops */
extern u64 pspat_rate;			/* link rate. XXX inherit from TC */
extern u64 pspat_arb_interval_ns;	/* scan interval for input queues */
	/* The interval should be in the 1000-5000 ns range */
extern u32 pspat_qdisc_batch_limit;	/* dequeue batch */

/* some statistics counters (readonly) */
extern u64 pspat_arb_tc_enq_drop;	/* dropped on enqueue */
extern u64 pspat_arb_tc_deq;		/* dequeued */
extern u64 pspat_arb_backpressure_drop;	/* ??? */
extern u64 pspat_xmit_ok;		/* successful transmits */
/* pspat_rounds[i] counts the number of times the arbiter has
 * seen i per-cpu queues not empty in a single round
 */
extern u64 *pspat_rounds;
extern struct pspat_stats *pspat_stats; /* one per cpu */

#endif  /* __PSPAT_H__ */
