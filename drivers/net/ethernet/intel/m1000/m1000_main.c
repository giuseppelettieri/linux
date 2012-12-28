/***************************************************************************
m1000 paravirtual driver

****************************************************************************/

#include <net/ip6_checksum.h>
#include <linux/io.h>
#include <linux/prefetch.h>
#include <linux/bitops.h>
#include <linux/if_vlan.h>

#include "m1000.h"

char m1000_driver_name[] = "m1000";
static char m1000_driver_string[] = "m1000 paravirtual driver";
#define DRV_VERSION "1.0-NAPI"
const char m1000_driver_version[] = DRV_VERSION;
static const char m1000_copyright[] = "copyright";

#define BANNER "[m1000] "
#define D(fmt, args...) printk(BANNER fmt, ##args)
//#define DD(fmt, args...) printk(BANNER fmt, ##args)
#define DD(fmt, args...) 

#define DTXS() DD("TXSNTC = %d, TXHNTS = %d, TXSNTS = %d\n", adapter->csb[TXSNTC], adapter->csb[TXHNTS], adapter->csb[TXSNTS]);
#define DRXS() DD("RXSNTR = %d, RXHNTR = %d, RXSNTP = %d\n", adapter->csb[RXSNTR], adapter->csb[RXHNTR], adapter->csb[RXSNTP]);


static void mmio_write32(struct m1000_adapter * adapter, int index, uint32_t value)
{
    writel(cpu_to_le32(value), adapter->hw_addr + index * 4);
}

static uint32_t mmio_read32(struct m1000_adapter * adapter, int index)
{
    return le32_to_cpu(readl(adapter->hw_addr + index * 4));
}

/* m1000_pci_tbl - PCI Device ID Table
*/
static DEFINE_PCI_DEVICE_TABLE(m1000_pci_tbl) = {
    { 0x9999, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    /* last entry must be all 0s */
    {0,}
};

MODULE_DEVICE_TABLE(pci, m1000_pci_tbl);

static int m1000_alloc_ring(struct m1000_adapter *adapter, struct m1000_ring * ring);
static void m1000_free_ring(struct m1000_adapter *adapter, struct m1000_ring * ring);
static int m1000_init_module(void);
static void m1000_exit_module(void);
static int m1000_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void __devexit m1000_remove(struct pci_dev *pdev);
static int m1000_open(struct net_device *netdev);
static int m1000_close(struct net_device *netdev);
static void m1000_reset_tx_ring(struct m1000_adapter *adapter);
static void m1000_reset_rx_ring(struct m1000_adapter *adapter);
static netdev_tx_t m1000_start_xmit(struct sk_buff *skb,
	struct net_device *netdev);
static struct net_device_stats * m1000_get_stats(struct net_device *netdev);
static irqreturn_t m1000_intr(int irq, void *data);
static bool m1000_clean_tx_ring(struct m1000_adapter *adapter);
static int m1000_clean(struct napi_struct *napi, int budget);
static bool m1000_receive_from_rx_ring(struct m1000_adapter *adapter,
	int *work_done, int work_to_do);
static void m1000_tx_timeout(struct net_device *dev);

static void m1000_prepare_rx_buffers(struct m1000_adapter *adapter);


static struct pci_driver m1000_driver = {
    .name     = m1000_driver_name,
    .id_table = m1000_pci_tbl,
    .probe    = m1000_probe,
    .remove   = __devexit_p(m1000_remove),
    //.shutdown = m1000_shutdown,
    //.err_handler = &m1000_err_handler
};

MODULE_AUTHOR("Vincenzo, <v.maffione@gmail.com>");
MODULE_DESCRIPTION("m1000 paravirtual driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

/**
 * m1000_init_module - Driver Registration Routine
 *
 * m1000_init_module is the first routine called when the driver is
 * loaded. All it does is register with the PCI subsystem.
 **/
static int __init m1000_init_module(void)
{
    int ret;

    D("%s - version %s\n", m1000_driver_string, m1000_driver_version);
    D("%s\n", m1000_copyright);

    ret = pci_register_driver(&m1000_driver);

    if (ret)	
	D("Failed to pci_register_driver()\n");

    return ret;
}

module_init(m1000_init_module);

/**
 * m1000_exit_module - Driver Exit Cleanup Routine
 *
 * m1000_exit_module is called just before the driver is removed
 * from memory.
 **/
static void __exit m1000_exit_module(void)
{
    pci_unregister_driver(&m1000_driver);
    printk(BANNER "Module unloaded");
}

module_exit(m1000_exit_module);

static int m1000_request_irq(struct m1000_adapter *adapter)
{
    struct net_device *netdev = adapter->netdev;
    irq_handler_t handler = m1000_intr;
    int irq_flags = IRQF_SHARED;
    int err;

    err = request_irq(adapter->pdev->irq, handler, irq_flags, netdev->name,
	    netdev);
    if (err) {
	D("Unable to allocate interrupt Error: %d\n", err);
    }

    return err;
}

static void m1000_free_irq(struct m1000_adapter *adapter)
{
    struct net_device *netdev = adapter->netdev;

    free_irq(adapter->pdev->irq, netdev);
}

/**
 * m1000_irq_disable - Mask off interrupt generation on the NIC
 * @adapter: board private structure
 **/
static void m1000_irq_disable(struct m1000_adapter *adapter)
{
    mmio_write32(adapter, IE, 0);
    synchronize_irq(adapter->pdev->irq);
}

/**
 * m1000_irq_enable - Enable default interrupt generation settings
 * @adapter: board private structure
 **/

static void m1000_irq_enable(struct m1000_adapter *adapter)
{
    mmio_write32(adapter, IE, 1);
}

static const struct net_device_ops m1000_netdev_ops = {
    .ndo_open		= m1000_open,
    .ndo_stop		= m1000_close,
    .ndo_start_xmit		= m1000_start_xmit,
    .ndo_get_stats		= m1000_get_stats,
    //.ndo_set_rx_mode	= m1000_set_rx_mode,
    //.ndo_set_mac_address	= m1000_set_mac,
    .ndo_tx_timeout		= m1000_tx_timeout,
    //.ndo_change_mtu		= m1000_change_mtu,
    //d.ndo_do_ioctl		= m1000_ioctl,
    .ndo_validate_addr	= eth_validate_addr,
    //.ndo_fix_features	= m1000_fix_features,
    //.ndo_set_features	= m1000_set_features,
};

/**
 * m1000_probe - Device Initialization Routine
 * @pdev: PCI device information struct
 * @ent: entry in m1000_pci_tbl
 *
 * Returns 0 on success, negative on failure
 *
 * m1000_probe initializes an adapter identified by a pci_dev structure.
 * The OS initialization, configuring of the adapter private structure,
 * and a hardware reset occur.
 **/
static int __devinit m1000_probe(struct pci_dev *pdev,
	const struct pci_device_id *ent)
{
    struct net_device *netdev;
    struct m1000_adapter *adapter;

    int err;
    int bars;
    uint32_t reg;

    bars = pci_select_bars(pdev, IORESOURCE_MEM);
    err = pci_enable_device_mem(pdev);
    if (err)
	return err;
    err = pci_request_selected_regions(pdev, bars, m1000_driver_name);
    if (err)
	goto err_pci_reg;
    pci_set_master(pdev);
    err = pci_save_state(pdev);
    if (err)
	goto err_alloc_etherdev;

    err = -ENOMEM;
    netdev = alloc_etherdev(sizeof(struct m1000_adapter));
    if (!netdev)
	goto err_alloc_etherdev;
    SET_NETDEV_DEV(netdev, &pdev->dev);
    pci_set_drvdata(pdev, netdev);
    adapter = netdev_priv(netdev);
    adapter->netdev = netdev;
    adapter->pdev = pdev;
    adapter->bars = bars;

    err = -EIO;
    adapter->hw_addr = pci_ioremap_bar(pdev, BAR_0); // INDIRIZZO FISICO!!
    if (!adapter->hw_addr)
	goto err_ioremap;

    /* set the dma mask */
    err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
    if (err) {
	pr_err("No usable DMA config, aborting\n");
	goto err_dma;
    }
    dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));  // for consistent allocation

    netdev->netdev_ops = &m1000_netdev_ops;
    //m1000_set_ethtool_ops(netdev);
    netdev->watchdog_timeo = 10 * HZ;
    netif_napi_add(netdev, &adapter->napi, m1000_clean, 512);

    strncpy(netdev->name, pci_name(pdev), sizeof(netdev->name) - 1);

    /* For the sake of safety, disable IRQ and tx/rx operation. */
    m1000_irq_disable(adapter);
    mmio_write32(adapter, CTRL, 0);

    mutex_init(&adapter->mutex);

    /* at this point netdev->features = netdev->hw_features = 0 */
    netdev->hw_features |= NETIF_F_RXALL;
    netdev->features |= netdev->hw_features;

    /* read the MAC address from the board */
    reg = mmio_read32(adapter, HWADDRH);
    adapter->mac_addr[0] = (reg & 0x0000ff00) >> 8;
    adapter->mac_addr[1] = (reg & 0x000000ff);
    reg = mmio_read32(adapter, HWADDRL);
    adapter->mac_addr[2] = (reg & 0xff000000) >> 24;
    adapter->mac_addr[3] = (reg & 0x00ff0000) >> 16;
    adapter->mac_addr[4] = (reg & 0x0000ff00) >> 8;
    adapter->mac_addr[5] = (reg & 0x000000ff);

    memcpy(netdev->dev_addr, adapter->mac_addr, netdev->addr_len);
    memcpy(netdev->perm_addr, adapter->mac_addr, netdev->addr_len);

    if (!is_valid_ether_addr(netdev->perm_addr))
	D("Invalid MAC Address\n");

    //INIT_DELAYED_WORK(&adapter->watchdog_task, m1000_watchdog);
    //m1000_check_options(adapter);

    strcpy(netdev->name, "eth%d");
    err = register_netdev(netdev);
    if (err)
	goto err_register;

    /* carrier off reporting is important to ethtool even BEFORE open */
    netif_carrier_off(netdev);

    D("probe finished\n");

    return 0;

err_register:
err_dma:
err_ioremap:
    free_netdev(netdev);
err_alloc_etherdev:
    pci_release_selected_regions(pdev, bars);
err_pci_reg:
    pci_disable_device(pdev);
    return err;
}

/**
 * m1000_remove - Device Removal Routine
 * @pdev: PCI device information struct
 *
 * m1000_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.  The could be caused by a
 * Hot-Plug event, or because the driver is going to be removed from
 * memory.
 **/
static void __devexit m1000_remove(struct pci_dev *pdev)
{
    struct net_device *netdev = pci_get_drvdata(pdev);
    struct m1000_adapter *adapter = netdev_priv(netdev);

    //cancel_delayed_work_sync(&adapter->watchdog_task);
    unregister_netdev(netdev);
    iounmap(adapter->hw_addr);
    pci_release_selected_regions(pdev, adapter->bars);
    free_netdev(netdev);
    pci_disable_device(pdev);
}

/**
 * m1000_open - Called when a network interface is made active
 * @netdev: network interface device structure
 *
 * Returns 0 on success, negative value on failure
 *
 * The open entry point is called when a network interface is made
 * active by the system (IFF_UP).  At this point all resources needed
 * for transmit and receive operations are allocated, the interrupt
 * handler is registered with the OS, the watchdog task is started,
 * and the stack is notified that the interface is ready.
 **/

static int m1000_open(struct net_device *netdev)
{
    struct m1000_adapter *adapter = netdev_priv(netdev);
    struct m1000_ring * tx_ring = &adapter->tx_ring;
    struct m1000_ring * rx_ring = &adapter->rx_ring;
    int err;

    netif_carrier_off(netdev);

    /* allocate transmit descriptors */
    tx_ring->length = 512;
    if ((err = m1000_alloc_ring(adapter, tx_ring)))
	goto err_alloc_tx_ring;

    /* allocate receive descriptors */
    rx_ring->length = 512;
    if ((err = m1000_alloc_ring(adapter, rx_ring)))
	goto err_alloc_rx_ring;

    /* allocate the Communication Status Block */
    adapter->csb = dma_alloc_coherent(&adapter->pdev->dev, M1000_CSB_SIZE,
			&adapter->csb_dma, GFP_KERNEL);
    if (!adapter->csb) {
	D("comm_status_block() allocation failed!");
	goto err_alloc_csb;
    }

    //mmio_write32(adapter, CTRL, 0);  // disable m1000 tx/rx
    //m1000_irq_disable(adapter);

    /* initialize the Communication Status Block */
    adapter->csb[RXSNTR] = adapter->csb[RXHNTR] = adapter->csb[RXSNTP] = 0;
    adapter->csb[TXSNTC] = adapter->csb[TXHNTS] = adapter->csb[TXSNTS] = 0;

    /* initialize all the registers containing addresses and lengths */
    mmio_write32(adapter, CSBBAH, (adapter->csb_dma >> 32));
    mmio_write32(adapter, CSBBAL, (adapter->csb_dma & 0x00000000ffffffffULL));
    mmio_write32(adapter, TXRLEN, tx_ring->length);
    mmio_write32(adapter, TXRBAH, (tx_ring->dma >> 32));
    mmio_write32(adapter, TXRBAL, (tx_ring->dma & 0x00000000ffffffffULL));
    mmio_write32(adapter, RXRLEN, rx_ring->length);
    mmio_write32(adapter, RXRBAH, (rx_ring->dma >> 32));
    mmio_write32(adapter, RXRBAL, (rx_ring->dma & 0x00000000ffffffffULL));

    m1000_prepare_rx_buffers(adapter);
    DRXS();

    /* before we allocate an interrupt, we must be ready to handle it.
     * Setting DEBUG_SHIRQ in the kernel makes it fire an interrupt
     * as soon as we call pci_request_irq. */
    if ((err = m1000_request_irq(adapter))) {
	D("irq allocation failed!\n");
	goto err_req_irq;
    }

    napi_enable(&adapter->napi);
    m1000_irq_enable(adapter);
    mmio_write32(adapter, CTRL, M1000_TX_ENABLED | M1000_RX_ENABLED);

    netif_carrier_on(netdev);
    netif_start_queue(netdev);
    DD("open finished\n");

    return 0;

err_req_irq:	
    dma_free_coherent(&adapter->pdev->dev, M1000_CSB_SIZE, adapter->csb, adapter->csb_dma);
err_alloc_csb:
    m1000_free_ring(adapter, tx_ring);
err_alloc_rx_ring:
    m1000_free_ring(adapter, rx_ring);
err_alloc_tx_ring:

    return err;
}

/**
 * m1000_close - Disables a network interface
 * @netdev: network interface device structure
 *
 * Returns 0, this is not allowed to fail
 *
 * The close entry point is called when an interface is de-activated
 * by the OS.  The hardware is still under the drivers control, but
 * needs to be disabled.  A global MAC reset is issued to stop the
 * hardware, and all transmit and receive resources are freed.
 **/

static int m1000_close(struct net_device *netdev)
{
    struct m1000_adapter *adapter = netdev_priv(netdev);

    netif_tx_disable(netdev);

    napi_disable(&adapter->napi);

    mmio_write32(adapter, CTRL, 0);
    m1000_irq_disable(adapter);

    netif_carrier_off(netdev);
    m1000_reset_tx_ring(adapter);
    m1000_reset_rx_ring(adapter);

    m1000_free_irq(adapter);

    dma_free_coherent(&adapter->pdev->dev, M1000_CSB_SIZE, adapter->csb, adapter->csb_dma);
    m1000_free_ring(adapter, &adapter->tx_ring);
    m1000_free_ring(adapter, &adapter->rx_ring);
    DD("close finished\n");

    return 0;
}

/**
 * m1000_alloc_tx_resources - allocate Tx resources (Descriptors)
 * @adapter: board private structure
 * @txdr:    tx descriptor ring (for a specific queue) to setup
 *
 * Return 0 on success, negative on failure
 **/
static int m1000_alloc_ring(struct m1000_adapter *adapter, struct m1000_ring * ring)
{
    struct pci_dev *pdev = adapter->pdev;
    int size;

    size = sizeof(struct m1000_sk_buff) * ring->length;
    ring->mskb_array = vzalloc(size);
    if (!ring->mskb_array) {
	D("Unable to allocate memory for the descriptor ring\n");
	return -ENOMEM;
    }

    /* round up to nearest 4K */
    ring->size = ring->length * sizeof(struct m1000_descriptor);
    ring->size = ALIGN(ring->size, 4096);

    ring->descriptor_array = dma_alloc_coherent(&pdev->dev, ring->size, &ring->dma, GFP_KERNEL); /* allocation and mapping of consistent memory */
    if (!ring->descriptor_array) {
	vfree(ring->mskb_array);
	D("Unable to allocate memory for the descriptor ring\n");
	return -ENOMEM;
    }

    memset(ring->descriptor_array, 0, ring->size);

    return 0;
}

/**
 * m1000_free_tx_resources - Free Tx Resources per Queue
 * @adapter: board private structure
 * @tx_ring: Tx descriptor ring for a specific queue
 *
 * Free all transmit software resources
 **/

static void m1000_free_ring(struct m1000_adapter *adapter, struct m1000_ring * ring)
{
    struct pci_dev *pdev = adapter->pdev;

    if (ring == &adapter->tx_ring)
	m1000_reset_tx_ring(adapter);
    else /* if (ring == &adapter->rx_ring) */
	m1000_reset_rx_ring(adapter);

    vfree(ring->mskb_array);
    ring->mskb_array = NULL;

    dma_free_coherent(&pdev->dev, ring->size, ring->descriptor_array, ring->dma);

    ring->descriptor_array = NULL;
}

static void m1000_unmap_and_free_tx_buffer(struct m1000_adapter *adapter,
	struct m1000_sk_buff *mskb)
{
    if (mskb->dma) {
	dma_unmap_single(&adapter->pdev->dev, mskb->dma,
		    mskb->length,
		    DMA_TO_DEVICE);
	mskb->dma = 0;
    }
    if (mskb->skb) {
	dev_kfree_skb_any(mskb->skb);
	mskb->skb = NULL;
    }
}

/**
 * m1000_reset_tx_ring - Free Tx Buffers
 * @adapter: board private structure
 * @tx_ring: ring to be cleaned
 **/

static void m1000_reset_tx_ring(struct m1000_adapter *adapter)
{
    struct m1000_ring * tx_ring = &adapter->tx_ring;
    struct m1000_sk_buff *mskb;
    unsigned long size;
    unsigned int i;

    /* Free all the Tx ring sk_buffs */

    for (i = 0; i < tx_ring->length; i++) {
	mskb = &tx_ring->mskb_array[i];
	m1000_unmap_and_free_tx_buffer(adapter, mskb);
    }

    netdev_reset_queue(adapter->netdev);
    size = sizeof(struct m1000_sk_buff) * tx_ring->length;
    memset(tx_ring->mskb_array, 0, size);

    /* Zero out the descriptor ring */

    memset(tx_ring->descriptor_array, 0, tx_ring->size);

    adapter->csb[TXSNTS] = 0;
    adapter->csb[TXSNTC] = 0;

    /*
       writel(0, hw->hw_addr + tx_ring->tdh);
       writel(0, hw->hw_addr + tx_ring->tdt); */
}

/**
 * m1000_reset_rx_ring - Free Rx Buffers per Queue
 * @adapter: board private structure
 * @rx_ring: ring to free buffers from
 **/

static void m1000_reset_rx_ring(struct m1000_adapter *adapter)
{
    struct m1000_sk_buff *mskb;
    struct pci_dev *pdev = adapter->pdev;
    struct m1000_ring * rx_ring = &adapter->rx_ring;
    unsigned long size;
    unsigned int i;

    /* Free all the Rx ring sk_buffs */
    for (i = 0; i < rx_ring->length; i++) {
	mskb = &rx_ring->mskb_array[i];
	if (mskb->dma) {
	    dma_unmap_single(&pdev->dev, mskb->dma,
		    mskb->length,
		    DMA_FROM_DEVICE);
	}

	mskb->dma = 0;
	if (mskb->skb) {
	    dev_kfree_skb(mskb->skb);
	    mskb->skb = NULL;
	}
    }

    size = sizeof(struct m1000_sk_buff) * rx_ring->length;
    memset(rx_ring->mskb_array, 0, size);

    /* Zero out the descriptor ring */
    memset(rx_ring->descriptor_array, 0, rx_ring->size);

    adapter->csb[RXSNTR] = 0;
    adapter->csb[RXSNTP] = 0;

    /*
       writel(0, hw->hw_addr + rx_ring->rdh);
       writel(0, hw->hw_addr + rx_ring->rdt); */
}

#define M1000_MAX_DATA_PER_TXD	4096

/* number of free tx descriptors (the very last is excluded) */
static inline int m1000_tx_free_desc(struct m1000_adapter * adapter)
{
    int free_desc;

    free_desc = adapter->csb[TXSNTC] - adapter->csb[TXSNTS] - 1;
    if (free_desc < 0)
	free_desc += adapter->tx_ring.length;
    return free_desc;
}

static netdev_tx_t m1000_start_xmit(struct sk_buff *skb,
	struct net_device *netdev)
{
    struct m1000_adapter *adapter = netdev_priv(netdev);
    struct m1000_ring *tx_ring;
    struct m1000_sk_buff *mskb;
    struct m1000_descriptor *tx_desc;
    int i;

    tx_ring = &adapter->tx_ring;

    if (unlikely(skb->len <= 0))
	goto drop;

    if (unlikely(skb_is_nonlinear(skb))) {
	D("non linear skb!\n");  // for now we don't manage nonlinear skb
	goto drop;
    }

    if (unlikely(skb->len > M1000_MAX_DATA_PER_TXD)) {
	D("skb->len=%d > %d\n", skb->len, M1000_MAX_DATA_PER_TXD);
	goto drop;
    }

    if (unlikely(skb_shinfo(skb)->gso_size)) {
	D("mss=%d\n", skb_shinfo(skb)->gso_size);
	goto drop;
    }

    switch (skb->ip_summed) {
	case CHECKSUM_PARTIAL:
	    DD("tx - Checksum is partial\n");
	    break;
	case CHECKSUM_UNNECESSARY:
	    DD("tx - Checksum is unnecessary\n");
	    break;
	case CHECKSUM_NONE:
	    DD("tx - Checksum is none\n");
	    break;
	case CHECKSUM_COMPLETE:
	    DD("tx - Checksum is partial\n");
	    break;
    }
    /*
       if (skb->ip_summed == CHECKSUM_PARTIAL)
       tx_flags |= M1000_TX_FLAGS_CSUM;

       if (unlikely(skb->no_fcs))
       tx_flags |= M1000_TX_FLAGS_NO_FCS;
       */

    DTXS();

    i = adapter->csb[TXSNTS];

    mskb = &tx_ring->mskb_array[i];
    mskb->skb = skb;
    mskb->length = skb->len;
    /* set time_stamp *before* dma to help avoid a possible race */
    mskb->dma = dma_map_single(&adapter->pdev->dev, skb->data, 
			    skb->len, DMA_TO_DEVICE);
    if (dma_mapping_error(&adapter->pdev->dev, mskb->dma)) {
	D("TX DMA map failed\n");
	mskb->dma = 0;
	mskb->skb = NULL;
	goto drop;
    }

    netdev_sent_queue(netdev, skb->len);
    skb_tx_timestamp(skb);

    tx_desc = &tx_ring->descriptor_array[i];
    tx_desc->buffer_address = cpu_to_le64(mskb->dma);
    tx_desc->buffer_length = cpu_to_le32(mskb->length);

    if (unlikely(++i == tx_ring->length)) i = 0;
    adapter->csb[TXSNTS] = i;

    DD("tx-len = %d, buffer address = %p\n", skb->len, (void *)mskb->dma);
    DTXS();
    mmio_write32(adapter, NTFY, M1000_NTFY_TXD);

    /* Make sure there is space in the ring for the next send. */
    if (unlikely(m1000_tx_free_desc(adapter) == 0)) {
	netif_stop_queue(netdev);
    }

    return NETDEV_TX_OK;

drop:
    dev_kfree_skb_any(skb);
    DD("drop!\n");
    return NETDEV_TX_OK;
}

/**
 * m1000_tx_timeout - Respond to a Tx Hang
 * @netdev: network interface device structure
 **/

static void m1000_tx_timeout(struct net_device *netdev)
{
    struct m1000_adapter *adapter = netdev_priv(netdev);

    /* Do the reset outside of interrupt context */
    adapter->tx_timeout_count++;
    D("TIMEOUT!\n");
}

/**
 * m1000_get_stats - Get System Network Statistics
 * @netdev: network interface device structure
 *
 * Returns the address of the device statistics structure.
 * The statistics are actually updated from the watchdog.
 **/

static struct net_device_stats *m1000_get_stats(struct net_device *netdev)
{
    /* only return the current stats */
    return &netdev->stats;
}


/**
 * m1000_intr - Interrupt Handler
 * @irq: interrupt number
 * @data: pointer to a network interface device structure
 **/

static irqreturn_t m1000_intr(int irq, void *data)
{
    struct net_device *netdev = data;
    struct m1000_adapter *adapter = netdev_priv(netdev);

    DD("interrupt received\n");
    mmio_write32(adapter, NTFY, M1000_NTFY_IC);

    /* disable interrupts, without the synchronize_irq bit */ 
    mmio_write32(adapter, IE, 0);

    if (likely(napi_schedule_prep(&adapter->napi))) {
	adapter->total_tx_bytes = 0;
	adapter->total_tx_packets = 0;
	adapter->total_rx_bytes = 0;
	adapter->total_rx_packets = 0;
	__napi_schedule(&adapter->napi);
    } else {
	/* this really should not happen! if it does it is basically a
	 * bug, but not a hard error, so enable ints and continue */
	m1000_irq_enable(adapter);
	D("NAPI BUG!\n");
    }

    return IRQ_HANDLED;
}

/**
 * m1000_clean - NAPI Rx polling callback
 * @adapter: board private structure
 **/
static int m1000_clean(struct napi_struct *napi, int budget)
{
    struct m1000_adapter *adapter = container_of(napi, 
					struct m1000_adapter, napi);
    int tx_clean_complete = 0, work_done = 0;
    
    DTXS(); DRXS();
    tx_clean_complete = m1000_clean_tx_ring(adapter);
    m1000_receive_from_rx_ring(adapter, &work_done, budget);
    DTXS(); DRXS();

    if (!tx_clean_complete)
	work_done = budget;
    /* If budget not fully consumed, exit the polling mode */
    if (work_done < budget) {
	napi_complete(napi);
	m1000_irq_enable(adapter);
    }

    return work_done;
}

/**
 * m1000_clean_tx_ring - Reclaim resources after transmit completes
 * @adapter: board private structure
 **/
static bool m1000_clean_tx_ring(struct m1000_adapter *adapter)
{
    struct net_device *netdev = adapter->netdev;
    struct m1000_ring *tx_ring = &adapter->tx_ring;
    struct m1000_descriptor *tx_desc;
    struct m1000_sk_buff *mskb;
    unsigned int i;
    unsigned int count = 0;
    unsigned int total_tx_bytes = 0, total_tx_packets = 0;
    unsigned int bytes_compl = 0, pkts_compl = 0;

    i = adapter->csb[TXSNTC];

    while (i != adapter->csb[TXHNTS] && count < tx_ring->length) {
	rmb();	/* read mskb after TXHNTS */
	tx_desc = &tx_ring->descriptor_array[i];
	mskb = &tx_ring->mskb_array[i];

	total_tx_packets++;
	total_tx_bytes += mskb->length;
	if (mskb->skb) {
	    bytes_compl += mskb->skb->len;
	    pkts_compl++;
	}

	m1000_unmap_and_free_tx_buffer(adapter, mskb);
	if (unlikely(++i == tx_ring->length)) i = 0;
	count++;
    }

    adapter->csb[TXSNTC] = i;

    netdev_completed_queue(netdev, pkts_compl, bytes_compl);

#define TX_WAKE_THRESHOLD 32
    if (unlikely(count && netif_carrier_ok(netdev) &&
		m1000_tx_free_desc(adapter) >= TX_WAKE_THRESHOLD)) {
	/* Make sure that anybody stopping the queue after this
	 * sees the new next_to_clean.
	 */
	smp_mb();

	if (netif_queue_stopped(netdev)) {
	    netif_wake_queue(netdev);
	    //++adapter->restart_queue;
	}
    }

    adapter->total_tx_bytes += total_tx_bytes;
    adapter->total_tx_packets += total_tx_packets;
    netdev->stats.tx_bytes += total_tx_bytes;
    netdev->stats.tx_packets += total_tx_packets;
    return count < tx_ring->length; /* haven't cleaned every descriptor in the ring */
}

/**
 * m1000_receive_from_rx_ring - Send received data up the network stack; legacy
 * @adapter: board private structure
 * @rx_ring: ring to clean
 * @work_done: amount of napi work completed this call
 * @work_to_do: max amount of work allowed for this call to do
 */
static bool m1000_receive_from_rx_ring(struct m1000_adapter *adapter,
	int *work_done, int work_to_do)
{
    struct net_device *netdev = adapter->netdev;
    struct pci_dev *pdev = adapter->pdev;
    struct m1000_ring *rx_ring = &adapter->rx_ring;
    struct m1000_descriptor *rx_desc, *next_rx_desc;
    struct m1000_sk_buff *mskb, *next_mskb;
    uint32_t length;
    unsigned int i;
    int count = 0;
    unsigned int total_rx_bytes = 0, total_rx_packets = 0;
    int err;

    //i = rx_ring->next_to_clean;
    i = adapter->csb[RXSNTR];
    rx_desc = &rx_ring->descriptor_array[i];
    mskb = &rx_ring->mskb_array[i];

    while (i != adapter->csb[RXHNTR]) {
	struct sk_buff *skb;

	if (*work_done >= work_to_do)
	    break;
	(*work_done)++;
	rmb(); /* read descriptor and rx_mskb after RXHNTR */

	skb = mskb->skb;
	mskb->skb = NULL;

	prefetch(skb->data - NET_IP_ALIGN);

	count++;
	if (++i == rx_ring->length) i = 0;
	next_rx_desc = &rx_ring->descriptor_array[i];
	prefetch(next_rx_desc);

	next_mskb = &rx_ring->mskb_array[i];

	dma_unmap_single(&pdev->dev, mskb->dma,
		mskb->length, DMA_FROM_DEVICE);
	mskb->dma = 0;

	length = le32_to_cpu(rx_desc->buffer_length);

	//process_skb:
	/* adjust length to remove Ethernet CRC 
	if (likely(!(netdev->features & NETIF_F_RXFCS)))
	    length -= 4; */

	total_rx_bytes += length; /* don't count FCS */
	total_rx_packets++;

	skb_put(skb, length);

	skb_checksum_none_assert(skb);
	/* TCP/UDP Checksum has not been calculated */

	/* It must be a TCP or UDP packet with a valid checksum */
	/* TCP checksum is good */
	//skb->ip_summed = CHECKSUM_UNNECESSARY;

	skb->protocol = eth_type_trans(skb, adapter->netdev);
	//napi_gro_receive(&adapter->napi, skb);
	err = netif_rx(skb); // netif_rx_ni(skb);
	if (err != NET_RX_SUCCESS)
	    D("rx packet dropped!!\n");

	/* next_skb: use prefetched values */
	rx_desc = next_rx_desc;
	mskb = next_mskb;
    }
    //rx_ring->next_to_clean = i;
    adapter->csb[RXSNTR] = i;

    if (count)
	m1000_prepare_rx_buffers(adapter);

    D("received %d packets\n", count);

    adapter->total_rx_packets += total_rx_packets;
    adapter->total_rx_bytes += total_rx_bytes;
    netdev->stats.rx_bytes += total_rx_bytes;
    netdev->stats.rx_packets += total_rx_packets;
    return true;
}

/**
 * m1000_prepare_rx_buffers - Replace used receive buffers; legacy & extended
 * @adapter: address of board private structure
 **/

static void m1000_prepare_rx_buffers(struct m1000_adapter *adapter)
{
    struct net_device *netdev = adapter->netdev;
    struct pci_dev *pdev = adapter->pdev;
    struct m1000_ring * rx_ring = &adapter->rx_ring;
    struct m1000_descriptor *rx_desc;
    struct m1000_sk_buff *mskb;
    struct sk_buff *skb;
    unsigned int i;
    unsigned int bufsz = 4096;
    int desc_to_prepare;

    /* prepare all the free descriptors (except the last, so that we
       can get the difference between "empty" and "full" */
    desc_to_prepare = adapter->csb[RXSNTR] - adapter->csb[RXSNTP] - 1;
    if (desc_to_prepare < 0)
	desc_to_prepare += rx_ring->length;
    if (desc_to_prepare == 0) // TODO should never happen
	return;

    //i = rx_ring->next_to_use;
    i = adapter->csb[RXSNTP];
    mskb = &rx_ring->mskb_array[i];

    while (desc_to_prepare--) {
	skb = mskb->skb;
	if (skb) {
	    skb_trim(skb, 0);
	    goto map_skb;
	}

	skb = netdev_alloc_skb_ip_align(netdev, bufsz);
	if (unlikely(!skb)) {
	    /* Better luck next round */
	    //adapter->alloc_rx_buff_failed++;
	    D("netdev_alloc_skb_ip_align() failed!\n");
	    break;
	}

	mskb->skb = skb;
	mskb->length = bufsz;
map_skb:
	mskb->dma = dma_map_single(&pdev->dev, skb->data, mskb->length,
					DMA_FROM_DEVICE);
	if (dma_mapping_error(&pdev->dev, mskb->dma)) {
	    dev_kfree_skb(skb);
	    mskb->skb = NULL;
	    mskb->dma = 0;
	    //adapter->alloc_rx_buff_failed++;
	    D("prepare: dma_mapping_error()\n");
	    break; /* while !mskb->skb */
	}

	rx_desc = &rx_ring->descriptor_array[i];
	rx_desc->buffer_address = cpu_to_le64(mskb->dma);
	rx_desc->buffer_length = 0;  // INUTILE

	if (unlikely(++i == rx_ring->length))
	    i = 0;
	mskb = &rx_ring->mskb_array[i];
    }

    if (likely(adapter->csb[RXSNTP] != i)) { //TODO should be always true
	//rx_ring->next_to_use = i;
	adapter->csb[RXSNTP] = i;
	/* Force memory writes to complete before letting h/w
	 * know there are new descriptors to fetch.  (Only
	 * applicable for weak-ordered memory model archs,
	 * such as IA-64). */
	wmb();
	mmio_write32(adapter, NTFY, M1000_NTFY_RXD);
    }
}

/* m1000_main.c */
