/*******************************************************************************

  Intel PRO/1000 Linux driver
  Copyright(c) 1999 - 2006 Intel Corporation.

  This program is free software; you can redistribute it and/or modify it
  under the terms and conditions of the GNU General Public License,
  version 2, as published by the Free Software Foundation.

  This program is distributed in the hope it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
  more details.

  You should have received a copy of the GNU General Public License along with
  this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.

  The full GNU General Public License is included in this distribution in
  the file called "COPYING".

  Contact Information:
  Linux NICS <linux.nics@intel.com>
  m1000-devel Mailing List <m1000-devel@lists.sourceforge.net>
  Intel Corporation, 5200 N.E. Elam Young Parkway, Hillsboro, OR 97124-6497

*******************************************************************************/


/* Linux PRO/1000 Ethernet Driver main header file */

#ifndef _E1000_H_
#define _E1000_H_

#include <linux/stddef.h>
#include <linux/module.h>
#include <linux/types.h>
#include <asm/byteorder.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/pagemap.h>
#include <linux/dma-mapping.h>
#include <linux/bitops.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/capability.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <net/pkt_sched.h>
#include <linux/list.h>
#include <linux/reboot.h>
#include <net/checksum.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/if_vlan.h>


#define BAR_0		0
#define BAR_1		1
#define BAR_5		5

struct m1000_adapter;

//#include "m1000_hw.h"

/* this is the size past which hardware will drop packets when setting LPE=0 */
#define MAXIMUM_ETHERNET_VLAN_SIZE 1522

/* How many Tx Descriptors do we need to call netif_wake_queue ? */
#define E1000_TX_QUEUE_WAKE	16

/* How many Rx Buffers do we bundle into one write to the hardware ? */
#define E1000_RX_BUFFER_WRITE	16	/* Must be power of 2 */

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer */
struct m1000_buffer {
	struct sk_buff *skb;
	dma_addr_t dma;
	struct page *page;
	unsigned long time_stamp;
	u16 length;
	u16 next_to_watch;
	unsigned int segs;
	unsigned int bytecount;
	u16 mapped_as_page;
};

struct m1000_tx_ring {
	/* pointer to the descriptor ring memory */
	void *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* length of descriptor ring in bytes */
	unsigned int size;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct m1000_buffer *buffer_info;
};

struct m1000_rx_ring {
	/* pointer to the descriptor ring memory */
	void *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* length of descriptor ring in bytes */
	unsigned int size;
	/* number of descriptors in the ring */
	unsigned int count;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct m1000_buffer *buffer_info;
	struct sk_buff *rx_skb_top;

	/* cpu for rx queue */
	int cpu;

	u16 rdh;
	u16 rdt;
};

#define E1000_DESC_UNUSED(R)						\
	((((R)->next_to_clean > (R)->next_to_use)			\
	  ? 0 : (R)->count) + (R)->next_to_clean - (R)->next_to_use - 1)

#define E1000_RX_DESC_EXT(R, i)						\
	(&(((union m1000_rx_desc_extended *)((R).desc))[i]))
#define E1000_GET_DESC(R, i, type)	(&(((struct type *)((R).desc))[i]))
#define E1000_RX_DESC(R, i)		E1000_GET_DESC(R, i, m1000_rx_desc)
#define E1000_TX_DESC(R, i)		E1000_GET_DESC(R, i, m1000_tx_desc)
#define E1000_CONTEXT_DESC(R, i)	E1000_GET_DESC(R, i, m1000_context_desc)

/* board specific private data structure */

struct m1000_adapter {
    unsigned int total_tx_bytes;
    unsigned int total_tx_packets;
    unsigned int total_rx_bytes;
    unsigned int total_rx_packets;

    /* TX */
    struct m1000_tx_ring *tx_ring;      /* One per active queue */
    u32 tx_timeout_count;

    /* RX */
    struct m1000_rx_ring *rx_ring;      /* One per active queue */
    struct napi_struct napi;

    /* OS defined structs */
    struct net_device *netdev;
    struct pci_dev *pdev;

    /* structs defined in m1000_hw.h */
    struct m1000_hw hw;
    u8 __iomem *hw_addr;  // registers base address
    u8 mac_addr[6];

    unsigned long flags;

    /* for ioport free */
    int bars;

    struct mutex mutex;
};


#undef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#define e_err(msglvl, format, arg...) \
	netif_err(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_info(msglvl, format, arg...) \
	netif_info(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_warn(msglvl, format, arg...) \
	netif_warn(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_notice(msglvl, format, arg...) \
	netif_notice(adapter, msglvl, adapter->netdev, format, ## arg)
#define e_dev_info(format, arg...) \
	dev_info(&adapter->pdev->dev, format, ## arg)
#define e_dev_warn(format, arg...) \
	dev_warn(&adapter->pdev->dev, format, ## arg)
#define e_dev_err(format, arg...) \
	dev_err(&adapter->pdev->dev, format, ## arg)


extern int m1000_up(struct m1000_adapter *adapter);
extern void m1000_down(struct m1000_adapter *adapter);
extern void m1000_check_options(struct m1000_adapter *adapter);

#endif /* _E1000_H_ */
