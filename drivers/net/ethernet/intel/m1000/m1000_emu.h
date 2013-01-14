
/* registers index */
enum {
    CTRL = 0,	// control register (enable/disable rx/tx)
    IE,		// interrupt mask
    NTFY,	// notification register, used to notify the hypervisor
    CSBBAL,     // communication status block base address low
    CSBBAH,	// communication status block base address high

    RXRTH,	// size threshold that specify what rx ring the hardware should use when receiving a packet

    RX0RBAL,	// RX0 ring base address low
    RX0RBAH,	// RX0 ring base address high
    RX0RSZ,	// RX0 ring buffer size (in bytes)

    RX1RBAL,	// RX1 ring base address low
    RX1RBAH,	// RX1 ring base address high
    RX1RLEN,	// RX1 ring length (in descriptors)

    TXRBAL,
    TXRBAH,
    TXRLEN,

    HWADDRL,
    HWADDRH,

    UALTMR,	// hardware UAL timer interval (in ms)
    UALDTH,	// UAL drop threshold (in %)
};

/* communication status block index */
enum {

/* Hardware-driven words */
    RX0HNTR = 0,// RX0 hardware next to receive
    RX1HNTR,	// RX1 hardware next to receive
    TXHNTS,	// TX hardware next to send
    TXNTFY,	// the HW specifies if it wants to be notified by the driver

/* Software-driven words */
    RX0SNTR,	// RX0 software next to receive
    RX1SNTR,	// RX1 software next to receive
    RX1SNTP,	// RX1 software next to prepare
    TXSNTS,	// TX software next to send
    TXSNTC,	// TX software netx to clean
    UALDRP,	// percentage of UDPv4 dropped packets (guest feedback)
};

/* This define MUST use the name of the very first element in the software
   driven CSB. */
#define CSB_READ_OFFSET  (sizeof(uint32_t) * RX0SNTR)

#define M1000_NTFYMASK_RX   0x00000001  // there are new rx frames in the ring


struct m1000_descriptor {
    uint64_t buffer_address;
    uint32_t buffer_length;
    uint32_t padding;
};

#define M1000_TX_ENABLED    0x00000001
#define M1000_RX_ENABLED    0x00000002

#define M1000_NTFY_TXD	    0x00000001  /* there are tx descriptors ready to be processed by the board */
#define M1000_NTFY_RXD	    0x00000002  /* there are rx descriptors to be returned to the board */
#define M1000_NTFY_IC	    0X00000004  /* interrupt clear */

/* this is the size past which hardware will drop packets when setting LPE=0 */
#define MAXIMUM_ETHERNET_VLAN_SIZE 1522
