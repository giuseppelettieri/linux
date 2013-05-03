#ifndef NET_PARAVIRT_H
#define NET_PARAVIRT_H

/*
 Support for virtio-like communication.
 1. the guest allocates the shared Communication Status Block (csb) and
    write its physical address at CSBAL and CSBAH (offsets
    0x2830 and 0x2834, data is little endian).
    csb->csb_on enables the mode. If disabled, the device is a
    regular one.

 2. notifications for tx and rx are exchanged without vm exits
    if possible. In particular (only mentioning csb mode below):

 TX: host sets host_need_txkick=1 when the I/O thread bh is idle.
     Guest updates guest_tdt and returns if host_need_txkick == 0,
     otherwise dues a regular write to the TDT.
     If the txring runs dry, guest sets guest_need_txkick and retries
     to recover buffers.
     Host reacts to writes to the TDT by clearing host_need_txkick
     and scheduling a thread to do the reads.
     The thread is kept active until there are packets (with a
     configurable number of retries). Eventually it sets
     host_need_txkick=1, does a final check for packets and blocks.
     An interrupt is generated if guest_need_txkick == 1.

 */
struct paravirt_csb {
    /* XXX revise the layout to minimize cache bounces. Usage:
     * 	gw+	written frequently by the guest
     * 	gw-	written rarely by the guest
     * 	hr+	read frequently by the host
     *  ...
     */
    /* these are (mostly) written by the guest */
    uint32_t guest_tdt;            /* gw+ hr+ pkt to transmit */
    uint32_t guest_need_txkick;    /* gw- hr+ ran out of tx bufs, request kick */
    uint32_t guest_need_rxkick;    /* gw- hr+ ran out of rx pkts, request kick  */
    uint32_t guest_csb_on;         /* gw- hr+ enable paravirtual mode */
    uint32_t guest_rdt;            /* gw+ hr+ rx buffers available */
    uint32_t pad[11];

    /* these are (mostly) written by the host */
    uint32_t host_tdh;             /* hw+ gr0 shadow register, mostly unused */
    uint32_t host_need_txkick;     /* hw- gr+ start the iothread */
    uint32_t host_txcycles_lim;    /* gw- hr- how much to spin before  sleep.
				    * set by the guest */
    uint32_t host_txcycles;        /* gr0 hw- counter, but no need to be exported */
    uint32_t host_rdh;             /* hw+ gr0 shadow register, mostly unused */
    uint32_t host_need_rxkick;     /* hw- gr+ flush rx queued packets */
};

#endif
