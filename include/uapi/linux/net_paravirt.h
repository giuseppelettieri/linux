/*
 * Copyright (C) 2013 Luigi Rizzo. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *   1. Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef NET_PARAVIRT_H
#define NET_PARAVIRT_H

/*
 Support for virtio-like communication between host and guest NICs.

 1. the guest allocates the shared Communication Status Block (csb) and
    write its physical address at CSBAL and CSBAH (data is little endian).
    csb->csb_on enables the mode. If disabled, the device acts a regular one.

 2. notifications for tx and rx are exchanged without vm exits
    if possible. In particular (only mentioning csb mode below):

 TX: host sets host_need_txkick=1 when the I/O thread bh is idle.
     Guest updates guest_tdt and returns if host_need_txkick == 0,
     otherwise does a regular write to the TDT.
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
    /* XXX revise the layout to minimize cache bounces.
     * Usage is described as follows:
     * 	[GH][RW][+-0]	guest/host reads/writes frequently/rarely/almost never
     */
    /* these are (mostly) written by the guest */
    uint32_t guest_tdt;            /* GW+ HR+ pkt to transmit */
    uint32_t guest_need_txkick;    /* GW- HR+ ran out of tx bufs, request kick */
    uint32_t guest_need_rxkick;    /* GW- HR+ ran out of rx pkts, request kick  */
    uint32_t guest_csb_on;         /* GW- HR+ enable paravirtual mode */
    uint32_t guest_rdt;            /* GW+ HR+ rx buffers available */
    uint32_t guest_txkick_at; /* Ring index where guest expects a txkick. */
    uint32_t pad[10];

    /* these are (mostly) written by the host */
    uint32_t host_tdh;             /* HW+ GR0 shadow register, mostly unused */
    uint32_t host_need_txkick;     /* HW- GR+ start the iothread */
    uint32_t host_txcycles_lim;    /* GW- HR- how much to spin before  sleep.
				    * set by the guest */
    uint32_t host_txcycles;        /* GR0 HW- counter, but no need to be exported */
    uint32_t host_rdh;             /* HW+ GR0 shadow register, mostly unused */
    uint32_t host_need_rxkick;     /* HW- GR+ flush rx queued packets */
    uint32_t host_isr;
    uint32_t host_rxkick_at;  /* Ring index where host expects an rxkick. */
};

#define NET_PARAVIRT_CSB_SIZE   4096
#define NET_PARAVIRT_NONE   (~((uint32_t)0))

#ifdef	QEMU_PCI_H

/*
 * API functions only available within QEMU
 */

void paravirt_configure_csb(struct paravirt_csb** csb, uint32_t csbbal,
			uint32_t csbbah, QEMUBH* tx_bh, AddressSpace *as);

#endif /* QEMU_PCI_H */

#endif /* NET_PARAVIRT_H */
