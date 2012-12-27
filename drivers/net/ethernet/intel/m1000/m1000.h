#ifndef _M1000_H_
#define _M1000_H_

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

#include "m1000_emu.h"

#define BAR_0		0
#define BAR_1		1

/* How many Tx Descriptors do we need to call netif_wake_queue ? */
#define M1000_TX_QUEUE_WAKE	16

/* How many Rx Buffers do we bundle into one write to the hardware ? */
#define M1000_RX_BUFFER_WRITE	16	/* Must be power of 2 */

/* wrapper around a pointer to a socket buffer,
 * so a DMA handle can be stored along with the buffer */
struct m1000_buffer {
	struct sk_buff *skb;
	dma_addr_t dma;
	struct page *page;
	unsigned long time_stamp;
	uint32_t length;
	u16 mapped_as_page;
};

struct m1000_ring {
	/* pointer to the descriptor ring memory */
	struct m1000_descriptor *desc;
	/* physical address of the descriptor ring */
	dma_addr_t dma;
	/* length of descriptor ring in bytes */
	unsigned int size;
	/* number of descriptors in the ring */
	unsigned int length;
	/* next descriptor to associate a buffer with */
	unsigned int next_to_use;
	/* next descriptor to check for DD status bit */
	unsigned int next_to_clean;
	/* array of buffer information structs */
	struct m1000_buffer *buffer_info;
};

#define M1000_UNUSED_DESCRIPTORS(R)						\
	((((R)->next_to_clean > (R)->next_to_use)			\
	  ? 0 : (R)->length) + (R)->next_to_clean - (R)->next_to_use - 1)


#define M1000_CSB_SIZE 4096	// size of the communication status block

/* board specific private data structure */
struct m1000_adapter {
    unsigned int total_tx_bytes;
    unsigned int total_tx_packets;
    unsigned int total_rx_bytes;
    unsigned int total_rx_packets;

    /* TX */
    struct m1000_ring tx_ring;
    u32 tx_timeout_count;

    /* RX */
    struct m1000_ring rx_ring;
    struct napi_struct napi;

    uint32_t * csb;	    // communication status block
    dma_addr_t csb_dma;	    // dma handle for the CSB

    /* OS defined structs */
    struct net_device *netdev;
    struct pci_dev *pdev;

    u8 __iomem *hw_addr;  // register bank base address
    u8 mac_addr[6];	  // board hardware address

    unsigned long flags;

    /* for ioport free */
    int bars;

    struct mutex mutex;
};


static int m1000_unused_tx_descriptors(struct m1000_adapter * adapter)
{
    struct m1000_ring * tx_ring = &adapter->tx_ring;

    return ((tx_ring->next_to_clean > adapter->csb[TXSNTU]) ? 0 : tx_ring->length) + tx_ring->next_to_clean - adapter->csb[TXSNTU] - 1;
}
/*
static int m1000_unused_rx_descriptors(struct m1000_adapter * adapter)
{
    struct m1000_ring * rx_ring = &adapter->rx_ring;

    return ((rx_ring->next_to_clean > adapter->csb[RXSNTU]) ? 0 : rx_ring->length) + rx_ring->next_to_clean - adapter->csb[RXSNTU] - 1;
}
*/

extern void m1000_check_options(struct m1000_adapter *adapter);

#endif /* _M1000_H_ */
