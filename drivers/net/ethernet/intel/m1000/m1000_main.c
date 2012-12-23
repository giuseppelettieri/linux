/***************************************************************************
m1000 paravirtual driver

*******************************************************************************/

#include "m1000.h"
#include <net/ip6_checksum.h>
#include <linux/io.h>
#include <linux/prefetch.h>
#include <linux/bitops.h>
#include <linux/if_vlan.h>


char m1000_driver_name[] = "m1000";
static char m1000_driver_string[] = "m1000 paravirtual driver";
#define DRV_VERSION "1.0-NAPI"
const char m1000_driver_version[] = DRV_VERSION;
static const char m1000_copyright[] = "copyright";

#define BANNER "[m1000] "
#define D(fmt, args...) printk(BANNER fmt, ##args)


/* m1000_hw.h */

struct m1000_descriptor {
    uint64_t buffer_address;
    uint32_t buffer_length;
    uint32_t padding;
};
#define M1000_TX_ENABLED    1
#define M1000_RX_ENABLED    2

/* registers index */
enum {
    CTRL = 0,	// control register (enable/disable rx/tx)
    IE,		// interrupt mask
    NBA,	// notifications base address
    CSBBA,      // communication status block base address
    RXRBAL,	// RX ring base address low
    RXRBAH,	// RX ring base address high
    RXRLEN,	// RX ring length (in descriptors)
    TXRBAL,
    TXRBAH,
    TXRLEN,
};

static void mmio_write32(struct m1000_adapter * adapter, int index, uint32_t value)
{
    writel(value, adapter->hw_addr + index * 4);
}

static uint32_t mmio_read32(struct m1000_adapter * adapter, int index)
{
    readl(adapter->hw_addr + index * 4);
}

/* m1000_pci_tbl - PCI Device ID Table
 */
static DEFINE_PCI_DEVICE_TABLE(m1000_pci_tbl) = {
	{ 0xABCD, PCI_ANY_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
	/* last entry must be all 0s */
	{0,}
};

MODULE_DEVICE_TABLE(pci, m1000_pci_tbl);

static int m1000_setup_tx_resources(struct m1000_adapter *adapter,
                             struct m1000_tx_ring *txdr);
static int m1000_setup_rx_resources(struct m1000_adapter *adapter,
                             struct m1000_rx_ring *rxdr);
static void m1000_free_tx_resources(struct m1000_adapter *adapter,
                             struct m1000_tx_ring *tx_ring);
static void m1000_free_rx_resources(struct m1000_adapter *adapter,
                             struct m1000_rx_ring *rx_ring);
static int m1000_init_module(void);
static void m1000_exit_module(void);
static int m1000_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void __devexit m1000_remove(struct pci_dev *pdev);
static int m1000_alloc_queues(struct m1000_adapter *adapter);
static int m1000_open(struct net_device *netdev);
static int m1000_close(struct net_device *netdev);
static void m1000_configure_tx(struct m1000_adapter *adapter);
static void m1000_configure_rx(struct m1000_adapter *adapter);
static void m1000_clean_tx_ring(struct m1000_adapter *adapter,
                                struct m1000_tx_ring *tx_ring);
static void m1000_clean_rx_ring(struct m1000_adapter *adapter,
                                struct m1000_rx_ring *rx_ring);
static netdev_tx_t m1000_xmit_frame(struct sk_buff *skb,
				    struct net_device *netdev);
static struct net_device_stats * m1000_get_stats(struct net_device *netdev);
static irqreturn_t m1000_intr(int irq, void *data);
static bool m1000_clean_tx_irq(struct m1000_adapter *adapter,
			       struct m1000_tx_ring *tx_ring);
static int m1000_clean(struct napi_struct *napi, int budget);
static bool m1000_clean_rx_irq(struct m1000_adapter *adapter,
			       struct m1000_rx_ring *rx_ring,
			       int *work_done, int work_to_do);
static int m1000_ioctl(struct net_device *netdev, struct ifreq *ifr, int cmd);
static void m1000_tx_timeout(struct net_device *dev);



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

    pr_info("%s - version %s\n", m1000_driver_string, m1000_driver_version);
    pr_info("%s\n", m1000_copyright);

    ret = pci_register_driver(&m1000_driver);

    if (ret)	
	printk(BANNER "Failed to pci_register_driver()\n");
    else
	printk(BANNER "PCI driver successfully registered\n");

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
	e_err(probe, "Unable to allocate interrupt Error: %d\n", err);
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
	.ndo_start_xmit		= m1000_xmit_frame,
	//.ndo_get_stats		= m1000_get_stats,
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

    err = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
    if (err) {
	pr_err("No usable DMA config, aborting\n");
	goto err_dma;
    }
    dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));

    netdev->netdev_ops = &m1000_netdev_ops;
    //m1000_set_ethtool_ops(netdev);
    netdev->watchdog_timeo = 10 * HZ;
    netif_napi_add(netdev, &adapter->napi, m1000_clean, 512);

    strncpy(netdev->name, pci_name(pdev), sizeof(netdev->name) - 1);

    /* setup the private structure */
    if (m1000_alloc_queues(adapter)) {
	e_err(probe, "Unable to allocate memory for queues\n");
	return -ENOMEM;
    }
    /* Explicitly disable IRQ since the NIC can be in any state. */
    m1000_irq_disable(adapter);
    mutex_init(&adapter->mutex);


    netdev->features |= netdev->hw_features;
    netdev->hw_features |= (NETIF_F_RXCSUM |
	    NETIF_F_RXALL |
	    NETIF_F_RXFCS);

    netdev->priv_flags |= IFF_UNICAST_FLT;

    adapter->mac_addr[0] = 0x00; 
    adapter->mac_addr[1] = 0xAA;
    adapter->mac_addr[2] = 0xBB;
    adapter->mac_addr[3] = 0xCC;
    adapter->mac_addr[4] = 0xDD;
    adapter->mac_addr[5] = 0x01;

    memcpy(netdev->dev_addr, adapter->mac_addr, netdev->addr_len);
    memcpy(netdev->perm_addr, adapter->mac_addr, netdev->addr_len);

    if (!is_valid_ether_addr(netdev->perm_addr))
	e_err(probe, "Invalid MAC Address\n");


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
    kfree(adapter->tx_ring);
    kfree(adapter->rx_ring);
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
	kfree(adapter->tx_ring);
	kfree(adapter->rx_ring);
	iounmap(adapter->hw_addr);
	pci_release_selected_regions(pdev, adapter->bars);
	free_netdev(netdev);
	pci_disable_device(pdev);
}


/**
 * m1000_alloc_queues - Allocate memory for all rings
 * @adapter: board private structure to initialize
 *
 * We allocate one ring per queue at run-time since we don't know the
 * number of queues at compile-time.
 **/

static int __devinit m1000_alloc_queues(struct m1000_adapter *adapter)
{
    // TODO swich to static allocation
    adapter->tx_ring = kcalloc(1, sizeof(struct m1000_tx_ring), GFP_KERNEL);
    if (!adapter->tx_ring)
	return -ENOMEM;

    adapter->rx_ring = kcalloc(1, sizeof(struct m1000_rx_ring), GFP_KERNEL);
    if (!adapter->rx_ring) {
	kfree(adapter->tx_ring);
	return -ENOMEM;
    }

    return 0;
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
	int err;

	netif_carrier_off(netdev);

	/* allocate transmit descriptors */
	adapter->tx_ring->count = 512;
	err = m1000_setup_tx_resources(adapter, adapter->tx_ring);
	if (err)
		goto err_setup_tx;

	/* allocate receive descriptors */
	adaèter->rx_ring->count = 512;
	err = m1000_setup_rx_resources(adapter, adapter->rx_ring);
	if (err)
		goto err_setup_rx;

	mmio_write32(adapter, CTRL, 0);  // tx/rx disabled
	mmio_write32(adapter, IE, 0);    // interrupt disabled
	/* before we allocate an interrupt, we must be ready to handle it.
	 * Setting DEBUG_SHIRQ in the kernel makes it fire an interrupt
	 * as soon as we call pci_request_irq, so we have to setup our
	 * clean_rx handler before we do so.  */
	m1000_configure_tx(adapter);
	m1000_configure_rx(adapter);
	m1000_alloc_rx_buffers(adapter, adapter->rx_ring, E1000_DESC_UNUSED(ring));

	err = m1000_request_irq(adapter);
	if (err)
		goto err_req_irq;

	/* From here on the code is the same as m1000_up() */
	napi_enable(&adapter->napi);

	m1000_irq_enable(adapter);

	netif_start_queue(netdev);

	return 0;

err_req_irq:
	m1000_free_rx_resources(adapter, adapter->rx_ring);
err_setup_rx:
	m1000_free_tx_resources(adapter, adapter->tx_ring);

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

    /* disable transmits in the hardware */
    mmio_write32(adapter, CTRL, 0);
    /* flush both disables and wait for them to finish */

    napi_disable(&adapter->napi);

    e1000_irq_disable(adapter);

    netif_carrier_off(netdev);
    m1000_clean_tx_ring(adapter, adapter->tx_ring);
    m1000_clean_rx_ring(adapter, adapter->rx_ring);

    m1000_free_irq(adapter);

    m1000_free_tx_resources(adapter, adapter->tx_ring);
    m1000_free_rx_resources(adapter, adapter->rx_ring);

    return 0;
}

/**
 * m1000_setup_tx_resources - allocate Tx resources (Descriptors)
 * @adapter: board private structure
 * @txdr:    tx descriptor ring (for a specific queue) to setup
 *
 * Return 0 on success, negative on failure
 **/
static int m1000_setup_tx_resources(struct m1000_adapter *adapter)
{
    struct pci_dev *pdev = adapter->pdev;
    int size;
    struct m1000_tx_ring * tx_ring = adapter->tx_ring;

    size = sizeof(struct m1000_buffer) * tx_ring->count;
    tx_ring->buffer_info = vzalloc(size);
    if (!tx_ring->buffer_info) {
	e_err(probe, "Unable to allocate memory for the Tx descriptor "
		"ring\n");
	return -ENOMEM;
    }

    /* round up to nearest 4K */
    tx_ring->size = tx_ring->count * sizeof(struct m1000_desc);
    tx_ring->size = ALIGN(tx_ring->size, 4096);

    tx_ring->desc = dma_alloc_coherent(&pdev->dev, tx_ring->size, &tx_ring->dma,
	    GFP_KERNEL);
    if (!tx_ring->desc) {
setup_tx_desc_die:
	vfree(tx_ring->buffer_info);
	e_err(probe, "Unable to allocate memory for the Tx descriptor "
		"ring\n");
	return -ENOMEM;
    }

    memset(tx_ring->desc, 0, tx_ring->size);

    tx_ring->next_to_use = 0;
    tx_ring->next_to_clean = 0;

    return 0;
}

/**
 * m1000_configure_tx - Configure 8254x Transmit Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Tx unit of the MAC after a reset.
 **/

static void m1000_configure_tx(struct m1000_adapter *adapter)
{
    u64 tdba;

    /* Setup the HW Tx Head and Tail descriptor pointers */
    tdba = adapter->tx_ring->dma;
    tdlen = adapter->tx_ring->count;
    mmio_write32(adapter, TXRLEN, tdlen);
    mmio_write32(adapter, TXRBAH, (tdba >> 32));
    mmio_write32(adapter, TXRBAL, (tdba & 0x00000000ffffffffULL));
}

/**
 * m1000_setup_rx_resources - allocate Rx resources (Descriptors)
 * @adapter: board private structure
 * @rxdr:    rx descriptor ring (for a specific queue) to setup
 *
 * Returns 0 on success, negative on failure
 **/

static int m1000_setup_rx_resources(struct m1000_adapter *adapter)
{
    struct pci_dev *pdev = adapter->pdev;
    int size, desc_len;
    struct m1000_rx_ring * rx_ring;

    size = sizeof(struct m1000_buffer) * rx_ring->count;
    rx_ring->buffer_info = vzalloc(size);
    if (!rx_ring->buffer_info) {
	e_err(probe, "Unable to allocate memory for the Rx descriptor "
		"ring\n");
	return -ENOMEM;
    }

    desc_len = sizeof(struct m1000_desc);

    /* Round up to nearest 4K */

    rx_ring->size = rx_ring->count * desc_len;
    rx_ring->size = ALIGN(rx_ring->size, 4096);

    rx_ring->desc = dma_alloc_coherent(&pdev->dev, rx_ring->size, &rx_ring->dma,
	    GFP_KERNEL);

    if (!rx_ring->desc) {
	e_err(probe, "Unable to allocate memory for the Rx descriptor "
		"ring\n");
setup_rx_desc_die:
	vfree(rx_ring->buffer_info);
	return -ENOMEM;
    }
    memset(rx_ring->desc, 0, rx_ring->size);

    rx_ring->next_to_clean = 0;
    rx_ring->next_to_use = 0;
    rx_ring->rx_skb_top = NULL;

    return 0;
}

/**
 * m1000_configure_rx - Configure 8254x Receive Unit after Reset
 * @adapter: board private structure
 *
 * Configure the Rx unit of the MAC after a reset.
 **/

static void m1000_configure_rx(struct m1000_adapter *adapter)
{
    u64 rdba;
    u32 rdlen, rctl, rxcsum;

    rdlen = adapter->rx_ring->count;

    /* Setup the HW Rx Head and Tail Descriptor Pointers and
     * the Base and Length of the Rx Descriptor Ring */
    rdba = adapter->rx_ring->dma;
    mmio_write32(adapter, RXRLEN, rdlen);
    mmio_write32(adapter, RXRBAH, (rdba >> 32));
    mmio_write32(adapter, RDBAL, (rdba & 0x00000000ffffffffULL));
}

/**
 * m1000_free_tx_resources - Free Tx Resources per Queue
 * @adapter: board private structure
 * @tx_ring: Tx descriptor ring for a specific queue
 *
 * Free all transmit software resources
 **/

static void m1000_free_tx_resources(struct m1000_adapter *adapter,
				    struct m1000_tx_ring *tx_ring)
{
	struct pci_dev *pdev = adapter->pdev;

	m1000_clean_tx_ring(adapter, tx_ring);

	vfree(tx_ring->buffer_info);
	tx_ring->buffer_info = NULL;

	dma_free_coherent(&pdev->dev, tx_ring->size, tx_ring->desc,
			  tx_ring->dma);

	tx_ring->desc = NULL;
}

static void m1000_unmap_and_free_tx_resource(struct m1000_adapter *adapter,
					     struct m1000_buffer *buffer_info)
{
	if (buffer_info->dma) {
		if (buffer_info->mapped_as_page)
			dma_unmap_page(&adapter->pdev->dev, buffer_info->dma,
				       buffer_info->length, DMA_TO_DEVICE);
		else
			dma_unmap_single(&adapter->pdev->dev, buffer_info->dma,
					 buffer_info->length,
					 DMA_TO_DEVICE);
		buffer_info->dma = 0;
	}
	if (buffer_info->skb) {
		dev_kfree_skb_any(buffer_info->skb);
		buffer_info->skb = NULL;
	}
	buffer_info->time_stamp = 0;
	/* buffer_info must be completely set up in the transmit path */
}

/**
 * m1000_clean_tx_ring - Free Tx Buffers
 * @adapter: board private structure
 * @tx_ring: ring to be cleaned
 **/

static void m1000_clean_tx_ring(struct m1000_adapter *adapter,
				struct m1000_tx_ring *tx_ring)
{
	struct m1000_buffer *buffer_info;
	unsigned long size;
	unsigned int i;

	/* Free all the Tx ring sk_buffs */

	for (i = 0; i < tx_ring->count; i++) {
		buffer_info = &tx_ring->buffer_info[i];
		m1000_unmap_and_free_tx_resource(adapter, buffer_info);
	}

	netdev_reset_queue(adapter->netdev);
	size = sizeof(struct m1000_buffer) * tx_ring->count;
	memset(tx_ring->buffer_info, 0, size);

	/* Zero out the descriptor ring */

	memset(tx_ring->desc, 0, tx_ring->size);

	tx_ring->next_to_use = 0;
	tx_ring->next_to_clean = 0;
	tx_ring->last_tx_tso = false;

	/*
	writel(0, hw->hw_addr + tx_ring->tdh);
	writel(0, hw->hw_addr + tx_ring->tdt); */
}

/**
 * m1000_free_rx_resources - Free Rx Resources
 * @adapter: board private structure
 * @rx_ring: ring to clean the resources from
 *
 * Free all receive software resources
 **/
static void m1000_free_rx_resources(struct m1000_adapter *adapter,
				    struct m1000_rx_ring *rx_ring)
{
	struct pci_dev *pdev = adapter->pdev;

	m1000_clean_rx_ring(adapter, rx_ring);

	vfree(rx_ring->buffer_info);
	rx_ring->buffer_info = NULL;

	dma_free_coherent(&pdev->dev, rx_ring->size, rx_ring->desc,
			  rx_ring->dma);

	rx_ring->desc = NULL;
}

/**
 * m1000_clean_rx_ring - Free Rx Buffers per Queue
 * @adapter: board private structure
 * @rx_ring: ring to free buffers from
 **/

static void m1000_clean_rx_ring(struct m1000_adapter *adapter,
				struct m1000_rx_ring *rx_ring)
{
	struct m1000_buffer *buffer_info;
	struct pci_dev *pdev = adapter->pdev;
	unsigned long size;
	unsigned int i;

	/* Free all the Rx ring sk_buffs */
	for (i = 0; i < rx_ring->count; i++) {
		buffer_info = &rx_ring->buffer_info[i];
		if (buffer_info->dma) {
			dma_unmap_single(&pdev->dev, buffer_info->dma,
			                 buffer_info->length,
					 DMA_FROM_DEVICE);
		}

		buffer_info->dma = 0;
		if (buffer_info->page) {
			put_page(buffer_info->page);
			buffer_info->page = NULL;
		}
		if (buffer_info->skb) {
			dev_kfree_skb(buffer_info->skb);
			buffer_info->skb = NULL;
		}
	}

	/* there also may be some cached data from a chained receive */
	if (rx_ring->rx_skb_top) {
		dev_kfree_skb(rx_ring->rx_skb_top);
		rx_ring->rx_skb_top = NULL;
	}

	size = sizeof(struct m1000_buffer) * rx_ring->count;
	memset(rx_ring->buffer_info, 0, size);

	/* Zero out the descriptor ring */
	memset(rx_ring->desc, 0, rx_ring->size);

	rx_ring->next_to_clean = 0;
	rx_ring->next_to_use = 0;

/*
	writel(0, hw->hw_addr + rx_ring->rdh);
	writel(0, hw->hw_addr + rx_ring->rdt); */
}


#define E1000_TX_FLAGS_CSUM		0x00000001
#define E1000_TX_FLAGS_VLAN		0x00000002
#define E1000_TX_FLAGS_TSO		0x00000004
#define E1000_TX_FLAGS_IPV4		0x00000008
#define E1000_TX_FLAGS_NO_FCS		0x00000010
#define E1000_TX_FLAGS_VLAN_MASK	0xffff0000
#define E1000_TX_FLAGS_VLAN_SHIFT	16

static int m1000_tso(struct m1000_adapter *adapter,
		     struct m1000_tx_ring *tx_ring, struct sk_buff *skb)
{
	struct m1000_context_desc *context_desc;
	struct m1000_buffer *buffer_info;
	unsigned int i;
	u32 cmd_length = 0;
	u16 ipcse = 0, tucse, mss;
	u8 ipcss, ipcso, tucss, tucso, hdr_len;
	int err;

	if (skb_is_gso(skb)) {
		if (skb_header_cloned(skb)) {
			err = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
			if (err)
				return err;
		}

		hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
		mss = skb_shinfo(skb)->gso_size;
		if (skb->protocol == htons(ETH_P_IP)) {
			struct iphdr *iph = ip_hdr(skb);
			iph->tot_len = 0;
			iph->check = 0;
			tcp_hdr(skb)->check = ~csum_tcpudp_magic(iph->saddr,
								 iph->daddr, 0,
								 IPPROTO_TCP,
								 0);
			cmd_length = E1000_TXD_CMD_IP;
			ipcse = skb_transport_offset(skb) - 1;
		} else if (skb->protocol == htons(ETH_P_IPV6)) {
			ipv6_hdr(skb)->payload_len = 0;
			tcp_hdr(skb)->check =
				~csum_ipv6_magic(&ipv6_hdr(skb)->saddr,
						 &ipv6_hdr(skb)->daddr,
						 0, IPPROTO_TCP, 0);
			ipcse = 0;
		}
		ipcss = skb_network_offset(skb);
		ipcso = (void *)&(ip_hdr(skb)->check) - (void *)skb->data;
		tucss = skb_transport_offset(skb);
		tucso = (void *)&(tcp_hdr(skb)->check) - (void *)skb->data;
		tucse = 0;

		cmd_length |= (E1000_TXD_CMD_DEXT | E1000_TXD_CMD_TSE |
			       E1000_TXD_CMD_TCP | (skb->len - (hdr_len)));

		i = tx_ring->next_to_use;
		context_desc = E1000_CONTEXT_DESC(*tx_ring, i);
		buffer_info = &tx_ring->buffer_info[i];

		context_desc->lower_setup.ip_fields.ipcss  = ipcss;
		context_desc->lower_setup.ip_fields.ipcso  = ipcso;
		context_desc->lower_setup.ip_fields.ipcse  = cpu_to_le16(ipcse);
		context_desc->upper_setup.tcp_fields.tucss = tucss;
		context_desc->upper_setup.tcp_fields.tucso = tucso;
		context_desc->upper_setup.tcp_fields.tucse = cpu_to_le16(tucse);
		context_desc->tcp_seg_setup.fields.mss     = cpu_to_le16(mss);
		context_desc->tcp_seg_setup.fields.hdr_len = hdr_len;
		context_desc->cmd_and_length = cpu_to_le32(cmd_length);

		buffer_info->time_stamp = jiffies;
		buffer_info->next_to_watch = i;

		if (++i == tx_ring->count) i = 0;
		tx_ring->next_to_use = i;

		return true;
	}
	return false;
}

static bool m1000_tx_csum(struct m1000_adapter *adapter,
			  struct m1000_tx_ring *tx_ring, struct sk_buff *skb)
{
	struct m1000_context_desc *context_desc;
	struct m1000_buffer *buffer_info;
	unsigned int i;
	u8 css;
	u32 cmd_len = E1000_TXD_CMD_DEXT;

	if (skb->ip_summed != CHECKSUM_PARTIAL)
		return false;

	switch (skb->protocol) {
	case cpu_to_be16(ETH_P_IP):
		if (ip_hdr(skb)->protocol == IPPROTO_TCP)
			cmd_len |= E1000_TXD_CMD_TCP;
		break;
	case cpu_to_be16(ETH_P_IPV6):
		/* XXX not handling all IPV6 headers */
		if (ipv6_hdr(skb)->nexthdr == IPPROTO_TCP)
			cmd_len |= E1000_TXD_CMD_TCP;
		break;
	default:
		if (unlikely(net_ratelimit()))
			e_warn(drv, "checksum_partial proto=%x!\n",
			       skb->protocol);
		break;
	}

	css = skb_checksum_start_offset(skb);

	i = tx_ring->next_to_use;
	buffer_info = &tx_ring->buffer_info[i];
	context_desc = E1000_CONTEXT_DESC(*tx_ring, i);

	context_desc->lower_setup.ip_config = 0;
	context_desc->upper_setup.tcp_fields.tucss = css;
	context_desc->upper_setup.tcp_fields.tucso =
		css + skb->csum_offset;
	context_desc->upper_setup.tcp_fields.tucse = 0;
	context_desc->tcp_seg_setup.data = 0;
	context_desc->cmd_and_length = cpu_to_le32(cmd_len);

	buffer_info->time_stamp = jiffies;
	buffer_info->next_to_watch = i;

	if (unlikely(++i == tx_ring->count)) i = 0;
	tx_ring->next_to_use = i;

	return true;
}

#define E1000_MAX_TXD_PWR	12
#define E1000_MAX_DATA_PER_TXD	(1<<E1000_MAX_TXD_PWR)

static int m1000_tx_map(struct m1000_adapter *adapter,
			struct m1000_tx_ring *tx_ring,
			struct sk_buff *skb, unsigned int first,
			unsigned int max_per_txd, unsigned int nr_frags,
			unsigned int mss)
{
	struct pci_dev *pdev = adapter->pdev;
	struct m1000_buffer *buffer_info;
	unsigned int len = skb_headlen(skb);
	unsigned int offset = 0, size, count = 0, i;
	unsigned int f, bytecount, segs;

	i = tx_ring->next_to_use;

	while (len) {
		buffer_info = &tx_ring->buffer_info[i];
		size = min(len, max_per_txd);

		buffer_info->length = size;
		/* set time_stamp *before* dma to help avoid a possible race */
		buffer_info->time_stamp = jiffies;
		buffer_info->mapped_as_page = false;
		buffer_info->dma = dma_map_single(&pdev->dev,
						  skb->data + offset,
						  size,	DMA_TO_DEVICE);
		if (dma_mapping_error(&pdev->dev, buffer_info->dma))
			goto dma_error;
		buffer_info->next_to_watch = i;

		len -= size;
		offset += size;
		count++;
		if (len) {
			i++;
			if (unlikely(i == tx_ring->count))
				i = 0;
		}
	}

	for (f = 0; f < nr_frags; f++) {
		const struct skb_frag_struct *frag;

		frag = &skb_shinfo(skb)->frags[f];
		len = skb_frag_size(frag);
		offset = 0;

		while (len) {
			unsigned long bufend;
			i++;
			if (unlikely(i == tx_ring->count))
				i = 0;

			buffer_info = &tx_ring->buffer_info[i];
			size = min(len, max_per_txd);
			/* Workaround for premature desc write-backs
			 * in TSO mode.  Append 4-byte sentinel desc */
			if (unlikely(mss && f == (nr_frags-1) && size == len && size > 8))
				size -= 4;
			/* Workaround for potential 82544 hang in PCI-X.
			 * Avoid terminating buffers within evenly-aligned
			 * dwords. */
			bufend = (unsigned long)
				page_to_phys(skb_frag_page(frag));
			bufend += offset + size - 1;
			if (unlikely(adapter->pcix_82544 &&
				     !(bufend & 4) &&
				     size > 4))
				size -= 4;

			buffer_info->length = size;
			buffer_info->time_stamp = jiffies;
			buffer_info->mapped_as_page = true;
			buffer_info->dma = skb_frag_dma_map(&pdev->dev, frag,
						offset, size, DMA_TO_DEVICE);
			if (dma_mapping_error(&pdev->dev, buffer_info->dma))
				goto dma_error;
			buffer_info->next_to_watch = i;

			len -= size;
			offset += size;
			count++;
		}
	}

	segs = skb_shinfo(skb)->gso_segs ?: 1;
	/* multiply data chunks by size of headers */
	bytecount = ((segs - 1) * skb_headlen(skb)) + skb->len;

	tx_ring->buffer_info[i].skb = skb;
	tx_ring->buffer_info[i].segs = segs;
	tx_ring->buffer_info[i].bytecount = bytecount;
	tx_ring->buffer_info[first].next_to_watch = i;

	return count;

dma_error:
	dev_err(&pdev->dev, "TX DMA map failed\n");
	buffer_info->dma = 0;
	if (count)
		count--;

	while (count--) {
		if (i==0)
			i += tx_ring->count;
		i--;
		buffer_info = &tx_ring->buffer_info[i];
		m1000_unmap_and_free_tx_resource(adapter, buffer_info);
	}

	return 0;
}

static void m1000_tx_queue(struct m1000_adapter *adapter,
			   struct m1000_tx_ring *tx_ring, int tx_flags,
			   int count)
{
	struct m1000_tx_desc *tx_desc = NULL;
	struct m1000_buffer *buffer_info;
	u32 txd_upper = 0, txd_lower = E1000_TXD_CMD_IFCS;
	unsigned int i;

	if (likely(tx_flags & E1000_TX_FLAGS_TSO)) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D |
		             E1000_TXD_CMD_TSE;
		txd_upper |= E1000_TXD_POPTS_TXSM << 8;

		if (likely(tx_flags & E1000_TX_FLAGS_IPV4))
			txd_upper |= E1000_TXD_POPTS_IXSM << 8;
	}

	if (likely(tx_flags & E1000_TX_FLAGS_CSUM)) {
		txd_lower |= E1000_TXD_CMD_DEXT | E1000_TXD_DTYP_D;
		txd_upper |= E1000_TXD_POPTS_TXSM << 8;
	}

	if (unlikely(tx_flags & E1000_TX_FLAGS_VLAN)) {
		txd_lower |= E1000_TXD_CMD_VLE;
		txd_upper |= (tx_flags & E1000_TX_FLAGS_VLAN_MASK);
	}

	if (unlikely(tx_flags & E1000_TX_FLAGS_NO_FCS))
		txd_lower &= ~(E1000_TXD_CMD_IFCS);

	i = tx_ring->next_to_use;

	while (count--) {
		buffer_info = &tx_ring->buffer_info[i];
		tx_desc = E1000_TX_DESC(*tx_ring, i);
		tx_desc->buffer_addr = cpu_to_le64(buffer_info->dma);
		tx_desc->lower.data =
			cpu_to_le32(txd_lower | buffer_info->length);
		tx_desc->upper.data = cpu_to_le32(txd_upper);
		if (unlikely(++i == tx_ring->count)) i = 0;
	}

	tx_desc->lower.data |= cpu_to_le32(adapter->txd_cmd);

	/* txd_cmd re-enables FCS, so we'll re-disable it here as desired. */
	if (unlikely(tx_flags & E1000_TX_FLAGS_NO_FCS))
		tx_desc->lower.data &= ~(cpu_to_le32(E1000_TXD_CMD_IFCS));

	/* Force memory writes to complete before letting h/w
	 * know there are new descriptors to fetch.  (Only
	 * applicable for weak-ordered memory model archs,
	 * such as IA-64). */
	wmb();

	tx_ring->next_to_use = i;
	//writel(i, hw->hw_addr + tx_ring->tdt);
	mmio_write32(adapter, NBA, 1);
	/* we need this if more than one processor can write to our tail
	 * at a time, it syncronizes IO on IA64/Altix systems */
	mmiowb();
}

/* 82547 workaround to avoid controller hang in half-duplex environment.
 * The workaround is to avoid queuing a large packet that would span
 * the internal Tx FIFO ring boundary by notifying the stack to resend
 * the packet at a later time.  This gives the Tx FIFO an opportunity to
 * flush all packets.  When that occurs, we reset the Tx FIFO pointers
 * to the beginning of the Tx FIFO.
 */

#define E1000_FIFO_HDR			0x10
#define E1000_82547_PAD_LEN		0x3E0

static int m1000_82547_fifo_workaround(struct m1000_adapter *adapter,
				       struct sk_buff *skb)
{
	u32 fifo_space = adapter->tx_fifo_size - adapter->tx_fifo_head;
	u32 skb_fifo_len = skb->len + E1000_FIFO_HDR;

	skb_fifo_len = ALIGN(skb_fifo_len, E1000_FIFO_HDR);

	if (adapter->link_duplex != HALF_DUPLEX)
		goto no_fifo_stall_required;

	if (atomic_read(&adapter->tx_fifo_stall))
		return 1;

	if (skb_fifo_len >= (E1000_82547_PAD_LEN + fifo_space)) {
		atomic_set(&adapter->tx_fifo_stall, 1);
		return 1;
	}

no_fifo_stall_required:
	adapter->tx_fifo_head += skb_fifo_len;
	if (adapter->tx_fifo_head >= adapter->tx_fifo_size)
		adapter->tx_fifo_head -= adapter->tx_fifo_size;
	return 0;
}

static int __m1000_maybe_stop_tx(struct net_device *netdev, int size)
{
	struct m1000_adapter *adapter = netdev_priv(netdev);
	struct m1000_tx_ring *tx_ring = adapter->tx_ring;

	netif_stop_queue(netdev);
	/* Herbert's original patch had:
	 *  smp_mb__after_netif_stop_queue();
	 * but since that doesn't exist yet, just open code it. */
	smp_mb();

	/* We need to check again in a case another CPU has just
	 * made room available. */
	if (likely(E1000_DESC_UNUSED(tx_ring) < size))
		return -EBUSY;

	/* A reprieve! */
	netif_start_queue(netdev);
	++adapter->restart_queue;
	return 0;
}

static int m1000_maybe_stop_tx(struct net_device *netdev,
                               struct m1000_tx_ring *tx_ring, int size)
{
	if (likely(E1000_DESC_UNUSED(tx_ring) >= size))
		return 0;
	return __m1000_maybe_stop_tx(netdev, size);
}

#define TXD_USE_COUNT(S, X) (((S) >> (X)) + 1 )
static netdev_tx_t m1000_xmit_frame(struct sk_buff *skb,
				    struct net_device *netdev)
{
	struct m1000_adapter *adapter = netdev_priv(netdev);
	struct m1000_tx_ring *tx_ring;
	unsigned int first, max_per_txd = E1000_MAX_DATA_PER_TXD;
	unsigned int max_txd_pwr = E1000_MAX_TXD_PWR;
	unsigned int tx_flags = 0;
	unsigned int len = skb_headlen(skb);
	unsigned int nr_frags;
	unsigned int mss;
	int count = 0;
	int tso;
	unsigned int f;

	/* This goes back to the question of how to logically map a tx queue
	 * to a flow.  Right now, performance is impacted slightly negatively
	 * if using multiple tx queues.  If the stack breaks away from a
	 * single qdisc implementation, we can look at this again. */
	tx_ring = adapter->tx_ring;

	if (unlikely(skb->len <= 0)) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	/* On PCI/PCI-X HW, if packet size is less than ETH_ZLEN,
	 * packets may get corrupted during padding by HW.
	 * To WA this issue, pad all small packets manually.
	 */
	if (skb->len < ETH_ZLEN) {
		if (skb_pad(skb, ETH_ZLEN - skb->len))
			return NETDEV_TX_OK;
		skb->len = ETH_ZLEN;
		skb_set_tail_pointer(skb, ETH_ZLEN);
	}

	mss = skb_shinfo(skb)->gso_size;
	/* The controller does a simple calculation to
	 * make sure there is enough room in the FIFO before
	 * initiating the DMA for each buffer.  The calc is:
	 * 4 = ceil(buffer len/mss).  To make sure we don't
	 * overrun the FIFO, adjust the max buffer len if mss
	 * drops. */
	if (mss) {
		u8 hdr_len;
		max_per_txd = min(mss << 2, max_per_txd);
		max_txd_pwr = fls(max_per_txd) - 1;

		hdr_len = skb_transport_offset(skb) + tcp_hdrlen(skb);
	}

	/* reserve a descriptor for the offload context */
	if ((mss) || (skb->ip_summed == CHECKSUM_PARTIAL))
		count++;
	count++;

	/* Controller Erratum workaround */
	if (!skb->data_len && tx_ring->last_tx_tso && !skb_is_gso(skb))
		count++;

	count += TXD_USE_COUNT(len, max_txd_pwr);

	if (adapter->pcix_82544)
		count++;

	nr_frags = skb_shinfo(skb)->nr_frags;
	for (f = 0; f < nr_frags; f++)
		count += TXD_USE_COUNT(skb_frag_size(&skb_shinfo(skb)->frags[f]),
				       max_txd_pwr);
	if (adapter->pcix_82544)
		count += nr_frags;

	/* need: count + 2 desc gap to keep tail from touching
	 * head, otherwise try next time */
	if (unlikely(m1000_maybe_stop_tx(netdev, tx_ring, count + 2)))
		return NETDEV_TX_BUSY;

	first = tx_ring->next_to_use;

	tso = m1000_tso(adapter, tx_ring, skb);
	if (tso < 0) {
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}

	if (likely(m1000_tx_csum(adapter, tx_ring, skb)))
		tx_flags |= E1000_TX_FLAGS_CSUM;

	if (likely(skb->protocol == htons(ETH_P_IP)))
		tx_flags |= E1000_TX_FLAGS_IPV4;

	if (unlikely(skb->no_fcs))
		tx_flags |= E1000_TX_FLAGS_NO_FCS;

	count = m1000_tx_map(adapter, tx_ring, skb, first, max_per_txd,
	                     nr_frags, mss);

	if (count) {
		netdev_sent_queue(netdev, skb->len);
		skb_tx_timestamp(skb);

		m1000_tx_queue(adapter, tx_ring, tx_flags, count);
		/* Make sure there is space in the ring for the next send. */
		m1000_maybe_stop_tx(netdev, tx_ring, MAX_SKB_FRAGS + 2);

	} else {
		dev_kfree_skb_any(skb);
		tx_ring->buffer_info[first].time_stamp = 0;
		tx_ring->next_to_use = first;
	}

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
	printk("TIMEOUT!\n");
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
    u32 icr = er32(ICR);

    if (unlikely((!icr)))
	return IRQ_NONE;  /* Not our interrupt */

    /*
     * we might have caused the interrupt, but the above
     * read cleared it, and just in case the driver is
     * down there is nothing to do so return handled
     */
    if (unlikely(test_bit(__E1000_DOWN, &adapter->flags)))
	return IRQ_HANDLED;


    /* disable interrupts, without the synchronize_irq bit */
    e1000_irq_disable(adapter);

    if (likely(napi_schedule_prep(&adapter->napi))) {
	adapter->total_tx_bytes = 0;
	adapter->total_tx_packets = 0;
	adapter->total_rx_bytes = 0;
	adapter->total_rx_packets = 0;
	__napi_schedule(&adapter->napi);
    } else {
	/* this really should not happen! if it does it is basically a
	 * bug, but not a hard error, so enable ints and continue */
	if (!test_bit(__E1000_DOWN, &adapter->flags))
	    m1000_irq_enable(adapter);
    }

    return IRQ_HANDLED;
}

/**
 * m1000_clean - NAPI Rx polling callback
 * @adapter: board private structure
 **/
static int m1000_clean(struct napi_struct *napi, int budget)
{
	struct m1000_adapter *adapter = container_of(napi, struct m1000_adapter, napi);
	int tx_clean_complete = 0, work_done = 0;

	tx_clean_complete = m1000_clean_tx_irq(adapter, &adapter->tx_ring[0]);

	m1000_clean_rx_irq(adapter, &adapter->rx_ring[0], &work_done, budget);

	if (!tx_clean_complete)
		work_done = budget;

	/* If budget not fully consumed, exit the polling mode */
	if (work_done < budget) {
		if (likely(adapter->itr_setting & 3))
			m1000_set_itr(adapter);
		napi_complete(napi);
		if (!test_bit(__E1000_DOWN, &adapter->flags))
			m1000_irq_enable(adapter);
	}

	return work_done;
}

/**
 * m1000_clean_tx_irq - Reclaim resources after transmit completes
 * @adapter: board private structure
 **/
static bool m1000_clean_tx_irq(struct m1000_adapter *adapter,
			       struct m1000_tx_ring *tx_ring)
{
	struct net_device *netdev = adapter->netdev;
	struct m1000_tx_desc *tx_desc, *eop_desc;
	struct m1000_buffer *buffer_info;
	unsigned int i, eop;
	unsigned int count = 0;
	unsigned int total_tx_bytes=0, total_tx_packets=0;
	unsigned int bytes_compl = 0, pkts_compl = 0;

	i = tx_ring->next_to_clean;
	eop = tx_ring->buffer_info[i].next_to_watch;
	eop_desc = E1000_TX_DESC(*tx_ring, eop);

	while ((eop_desc->upper.data & cpu_to_le32(E1000_TXD_STAT_DD)) &&
	       (count < tx_ring->count)) {
		bool cleaned = false;
		rmb();	/* read buffer_info after eop_desc */
		for ( ; !cleaned; count++) {
			tx_desc = E1000_TX_DESC(*tx_ring, i);
			buffer_info = &tx_ring->buffer_info[i];
			cleaned = (i == eop);

			if (cleaned) {
				total_tx_packets += buffer_info->segs;
				total_tx_bytes += buffer_info->bytecount;
				if (buffer_info->skb) {
					bytes_compl += buffer_info->skb->len;
					pkts_compl++;
				}

			}
			m1000_unmap_and_free_tx_resource(adapter, buffer_info);
			tx_desc->upper.data = 0;

			if (unlikely(++i == tx_ring->count)) i = 0;
		}

		eop = tx_ring->buffer_info[i].next_to_watch;
		eop_desc = E1000_TX_DESC(*tx_ring, eop);
	}

	tx_ring->next_to_clean = i;

	netdev_completed_queue(netdev, pkts_compl, bytes_compl);

#define TX_WAKE_THRESHOLD 32
	if (unlikely(count && netif_carrier_ok(netdev) &&
		     E1000_DESC_UNUSED(tx_ring) >= TX_WAKE_THRESHOLD)) {
		/* Make sure that anybody stopping the queue after this
		 * sees the new next_to_clean.
		 */
		smp_mb();

		if (netif_queue_stopped(netdev) &&
		    !(test_bit(__E1000_DOWN, &adapter->flags))) {
			netif_wake_queue(netdev);
			++adapter->restart_queue;
		}
	}

	adapter->total_tx_bytes += total_tx_bytes;
	adapter->total_tx_packets += total_tx_packets;
	netdev->stats.tx_bytes += total_tx_bytes;
	netdev->stats.tx_packets += total_tx_packets;
	return count < tx_ring->count;
}

/**
 * m1000_rx_checksum - Receive Checksum Offload for 82543
 * @adapter:     board private structure
 * @status_err:  receive descriptor status and error fields
 * @csum:        receive descriptor csum field
 * @sk_buff:     socket buffer with received data
 **/

static void m1000_rx_checksum(struct m1000_adapter *adapter, u32 status_err,
			      u32 csum, struct sk_buff *skb)
{
	u16 status = (u16)status_err;
	u8 errors = (u8)(status_err >> 24);

	skb_checksum_none_assert(skb);

	/* Ignore Checksum bit is set */
	if (unlikely(status & E1000_RXD_STAT_IXSM)) return;
	/* TCP/UDP checksum error bit is set */
	if (unlikely(errors & E1000_RXD_ERR_TCPE)) {
		/* let the stack verify checksum errors */
		adapter->hw_csum_err++;
		return;
	}
	/* TCP/UDP Checksum has not been calculated */
	if (!(status & E1000_RXD_STAT_TCPCS))
		return;

	/* It must be a TCP or UDP packet with a valid checksum */
	if (likely(status & E1000_RXD_STAT_TCPCS)) {
		/* TCP checksum is good */
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	}
	adapter->hw_csum_good++;
}

/**
 * m1000_receive_skb - helper function to handle rx indications
 * @adapter: board private structure
 * @status: descriptor status field as written by hardware
 * @vlan: descriptor vlan field as written by hardware (no le/be conversion)
 * @skb: pointer to sk_buff to be indicated to stack
 */
static void m1000_receive_skb(struct m1000_adapter *adapter, u8 status,
			      __le16 vlan, struct sk_buff *skb)
{
	skb->protocol = eth_type_trans(skb, adapter->netdev);

	if (status & E1000_RXD_STAT_VP) {
		u16 vid = le16_to_cpu(vlan) & E1000_RXD_SPC_VLAN_MASK;

		__vlan_hwaccel_put_tag(skb, vid);
	}
	napi_gro_receive(&adapter->napi, skb);
}


/*
 * this should improve performance for small packets with large amounts
 * of reassembly being done in the stack
 */
static void m1000_check_copybreak(struct net_device *netdev,
				 struct m1000_buffer *buffer_info,
				 u32 length, struct sk_buff **skb)
{
	struct sk_buff *new_skb;

	if (length > copybreak)
		return;

	new_skb = netdev_alloc_skb_ip_align(netdev, length);
	if (!new_skb)
		return;

	skb_copy_to_linear_data_offset(new_skb, -NET_IP_ALIGN,
				       (*skb)->data - NET_IP_ALIGN,
				       length + NET_IP_ALIGN);
	/* save the skb in buffer_info as good */
	buffer_info->skb = *skb;
	*skb = new_skb;
}

/**
 * m1000_clean_rx_irq - Send received data up the network stack; legacy
 * @adapter: board private structure
 * @rx_ring: ring to clean
 * @work_done: amount of napi work completed this call
 * @work_to_do: max amount of work allowed for this call to do
 */
static bool m1000_clean_rx_irq(struct m1000_adapter *adapter,
			       struct m1000_rx_ring *rx_ring,
			       int *work_done, int work_to_do)
{
	struct net_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
	struct m1000_rx_desc *rx_desc, *next_rxd;
	struct m1000_buffer *buffer_info, *next_buffer;
	unsigned long flags;
	u32 length;
	unsigned int i;
	int cleaned_count = 0;
	bool cleaned = false;
	unsigned int total_rx_bytes=0, total_rx_packets=0;

	i = rx_ring->next_to_clean;
	rx_desc = E1000_RX_DESC(*rx_ring, i);
	buffer_info = &rx_ring->buffer_info[i];

	while (rx_desc->status & E1000_RXD_STAT_DD) {
		struct sk_buff *skb;
		u8 status;

		if (*work_done >= work_to_do)
			break;
		(*work_done)++;
		rmb(); /* read descriptor and rx_buffer_info after status DD */

		status = rx_desc->status;
		skb = buffer_info->skb;
		buffer_info->skb = NULL;

		prefetch(skb->data - NET_IP_ALIGN);

		if (++i == rx_ring->count) i = 0;
		next_rxd = E1000_RX_DESC(*rx_ring, i);
		prefetch(next_rxd);

		next_buffer = &rx_ring->buffer_info[i];

		cleaned = true;
		cleaned_count++;
		dma_unmap_single(&pdev->dev, buffer_info->dma,
				 buffer_info->length, DMA_FROM_DEVICE);
		buffer_info->dma = 0;

		length = le16_to_cpu(rx_desc->length);
		/* !EOP means multiple descriptors were used to store a single
		 * packet, if thats the case we need to toss it.  In fact, we
		 * to toss every packet with the EOP bit clear and the next
		 * frame that _does_ have the EOP bit set, as it is by
		 * definition only a frame fragment
		 */
		if (unlikely(!(status & E1000_RXD_STAT_EOP)))
			adapter->discarding = true;

		if (adapter->discarding) {
			/* All receives must fit into a single buffer */
			e_dbg("Receive packet consumed multiple buffers\n");
			/* recycle */
			buffer_info->skb = skb;
			if (status & E1000_RXD_STAT_EOP)
				adapter->discarding = false;
			goto next_desc;
		}


process_skb:
		total_rx_bytes += (length - 4); /* don't count FCS */
		total_rx_packets++;

		if (likely(!(netdev->features & NETIF_F_RXFCS)))
			/* adjust length to remove Ethernet CRC, this must be
			 * done after the TBI_ACCEPT workaround above
			 */
			length -= 4;

		m1000_check_copybreak(netdev, buffer_info, length, &skb);

		skb_put(skb, length);

		/* Receive Checksum Offload */
		m1000_rx_checksum(adapter,
				  (u32)(status) |
				  ((u32)(rx_desc->errors) << 24),
				  le16_to_cpu(rx_desc->csum), skb);

		m1000_receive_skb(adapter, status, rx_desc->special, skb);

next_desc:
		rx_desc->status = 0;
p
		/* return some buffers to hardware, one at a time is too slow */
		if (unlikely(cleaned_count >= E1000_RX_BUFFER_WRITE)) {
			m1000_alloc_rx_buffers(adapter, rx_ring, cleaned_count);
			cleaned_count = 0;
		}

		/* use prefetched values */
		rx_desc = next_rxd;
		buffer_info = next_buffer;
	}
	rx_ring->next_to_clean = i;

	cleaned_count = E1000_DESC_UNUSED(rx_ring);
	if (cleaned_count)
		m1000_alloc_rx_buffers(adapter, rx_ring, cleaned_count);

	adapter->total_rx_packets += total_rx_packets;
	adapter->total_rx_bytes += total_rx_bytes;
	netdev->stats.rx_bytes += total_rx_bytes;
	netdev->stats.rx_packets += total_rx_packets;
	return cleaned;
}

/**
 * m1000_alloc_rx_buffers - Replace used receive buffers; legacy & extended
 * @adapter: address of board private structure
 **/

static void m1000_alloc_rx_buffers(struct m1000_adapter *adapter,
				   struct m1000_rx_ring *rx_ring,
				   int cleaned_count)
{
	struct net_device *netdev = adapter->netdev;
	struct pci_dev *pdev = adapter->pdev;
	struct m1000_rx_desc *rx_desc;
	struct m1000_buffer *buffer_info;
	struct sk_buff *skb;
	unsigned int i;
	unsigned int bufsz = adapter->rx_buffer_len;

	i = rx_ring->next_to_use;
	buffer_info = &rx_ring->buffer_info[i];

	while (cleaned_count--) {
		skb = buffer_info->skb;
		if (skb) {
			skb_trim(skb, 0);
			goto map_skb;
		}

		skb = netdev_alloc_skb_ip_align(netdev, bufsz);
		if (unlikely(!skb)) {
			/* Better luck next round */
			adapter->alloc_rx_buff_failed++;
			break;
		}

		/* Fix for errata 23, can't cross 64kB boundary */
		if (!m1000_check_64k_bound(adapter, skb->data, bufsz)) {
			struct sk_buff *oldskb = skb;
			e_err(rx_err, "skb align check failed: %u bytes at "
			      "%p\n", bufsz, skb->data);
			/* Try again, without freeing the previous */
			skb = netdev_alloc_skb_ip_align(netdev, bufsz);
			/* Failed allocation, critical failure */
			if (!skb) {
				dev_kfree_skb(oldskb);
				adapter->alloc_rx_buff_failed++;
				break;
			}

			if (!m1000_check_64k_bound(adapter, skb->data, bufsz)) {
				/* give up */
				dev_kfree_skb(skb);
				dev_kfree_skb(oldskb);
				adapter->alloc_rx_buff_failed++;
				break; /* while !buffer_info->skb */
			}

			/* Use new allocation */
			dev_kfree_skb(oldskb);
		}
		buffer_info->skb = skb;
		buffer_info->length = adapter->rx_buffer_len;
map_skb:
		buffer_info->dma = dma_map_single(&pdev->dev,
						  skb->data,
						  buffer_info->length,
						  DMA_FROM_DEVICE);
		if (dma_mapping_error(&pdev->dev, buffer_info->dma)) {
			dev_kfree_skb(skb);
			buffer_info->skb = NULL;
			buffer_info->dma = 0;
			adapter->alloc_rx_buff_failed++;
			break; /* while !buffer_info->skb */
		}

		/*
		 * XXX if it was allocated cleanly it will never map to a
		 * boundary crossing
		 */

		/* Fix for errata 23, can't cross 64kB boundary */
		if (!m1000_check_64k_bound(adapter,
					(void *)(unsigned long)buffer_info->dma,
					adapter->rx_buffer_len)) {
			e_err(rx_err, "dma align check failed: %u bytes at "
			      "%p\n", adapter->rx_buffer_len,
			      (void *)(unsigned long)buffer_info->dma);
			dev_kfree_skb(skb);
			buffer_info->skb = NULL;

			dma_unmap_single(&pdev->dev, buffer_info->dma,
					 adapter->rx_buffer_len,
					 DMA_FROM_DEVICE);
			buffer_info->dma = 0;

			adapter->alloc_rx_buff_failed++;
			break; /* while !buffer_info->skb */
		}
		rx_desc = E1000_RX_DESC(*rx_ring, i);
		rx_desc->buffer_addr = cpu_to_le64(buffer_info->dma);

		if (unlikely(++i == rx_ring->count))
			i = 0;
		buffer_info = &rx_ring->buffer_info[i];
	}

	if (likely(rx_ring->next_to_use != i)) {
		rx_ring->next_to_use = i;
		if (unlikely(i-- == 0))
			i = (rx_ring->count - 1);

		/* Force memory writes to complete before letting h/w
		 * know there are new descriptors to fetch.  (Only
		 * applicable for weak-ordered memory model archs,
		 * such as IA-64). */
		wmb();
		//writel(i, hw->hw_addr + rx_ring->rdt);
		mmio_write32(adapter, NBA, 2);
	}
}

/* m1000_main.c */
