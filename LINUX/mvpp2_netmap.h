/*
 * Copyright 2023, Allied Telesis Labs New Zealand, Ltd
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
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
#include <bsd_glue.h>
#include <net/netmap.h>
#include <netmap/netmap_kern.h>

/* Private adapter storage */
struct mvpp2_nm_adapter {
	struct netmap_hw_adapter up;
	bool irqs_enabled[32];
};

/* Used functions from below netmap header include */
static u32 mvpp2_thread_read(struct mvpp2 *priv, unsigned int thread, u32 offset);
static u32 mvpp2_thread_read_relaxed(struct mvpp2 *priv, unsigned int thread, u32 offset);
static void mvpp2_thread_write(struct mvpp2 *priv, unsigned int thread, u32 offset, u32 data);
static inline u32 mvpp2_cpu_to_thread(struct mvpp2 *priv, int cpu);
static inline void mvpp2_cause_error(struct net_device *dev, int cause);
static inline void mvpp2_qvec_interrupt_enable(struct mvpp2_queue_vector *qvec);
static inline void mvpp2_qvec_interrupt_disable(struct mvpp2_queue_vector *qvec);
static inline int mvpp2_rxq_received(struct mvpp2_port *port, int rxq_id);
static inline struct mvpp2_rx_desc *mvpp2_rxq_next_desc_get(struct mvpp2_rx_queue *rxq);
static u32 mvpp2_rxdesc_status_get(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc);
static unsigned long mvpp2_rxdesc_cookie_get(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc);
static size_t mvpp2_rxdesc_size_get(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc);
static dma_addr_t mvpp2_rxdesc_dma_addr_get(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc);
static void mvpp2_rx_error(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc);
static void mvpp2_buff_hdr_pool_put(struct mvpp2_port *port, struct mvpp2_rx_desc *rx_desc, int pool, u32 rx_status);
static inline void mvpp2_bm_pool_put(struct mvpp2_port *port, int pool, dma_addr_t buf_dma_addr, phys_addr_t buf_phys_addr);
static inline void mvpp2_rxq_status_update(struct mvpp2_port *port, int rxq_id, int used_count, int free_count);

/* Mask/unmask TX interupts - we will process tx completion before sending new packets */
static void mvpp2_netmap_mask_tx_interrupts(struct mvpp2_port *port, unsigned int thread)
{
	if (port->has_tx_irqs) {
		u32 val = mvpp2_thread_read(port->priv, thread, MVPP2_ISR_RX_TX_MASK_REG(port->id));
		val &= ~MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;
		mvpp2_thread_write(port->priv, thread, MVPP2_ISR_RX_TX_MASK_REG(port->id), val);
	}
}
static void mvpp2_netmap_unmask_tx_interrupts(struct mvpp2_port *port, unsigned int thread)
{
	if (port->has_tx_irqs) {
		u32 val = mvpp2_thread_read(port->priv, thread, MVPP2_ISR_RX_TX_MASK_REG(port->id));
		val |= MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;
		mvpp2_thread_write(port->priv, thread, MVPP2_ISR_RX_TX_MASK_REG(port->id), val);
	}
}

/* Enable queue interrupts */
static void mvpp2_netmap_qvec_interrupt_enable (struct netmap_adapter *na, int vec)
{
	struct mvpp2_nm_adapter *mna = (struct mvpp2_nm_adapter *)na;
	struct ifnet *ifp = na->ifp;
	struct mvpp2_port *port = netdev_priv(ifp);
	struct mvpp2_queue_vector *qv = port->qvecs + vec;

	if (!mna->irqs_enabled[vec]) {
		nm_prdis("NETMAP[%s:%d] Enabled IRQ\n", ifp->name, vec);
		mna->irqs_enabled[vec] = true;
		mvpp2_qvec_interrupt_enable(qv);
	}
}

/* Disable queue interrupts */
static void mvpp2_netmap_qvec_interrupt_disable (struct netmap_adapter *na, int vec)
{
	struct mvpp2_nm_adapter *mna = (struct mvpp2_nm_adapter *)na;
	struct ifnet *ifp = na->ifp;
	struct mvpp2_port *port = netdev_priv(ifp);
	struct mvpp2_queue_vector *qv = port->qvecs + vec;

	if (mna->irqs_enabled[vec]) {
		nm_prdis("NETMAP[%s] Disable IRQ\n", ifp->name);
		mvpp2_qvec_interrupt_disable(qv);
		mna->irqs_enabled[vec] = false;
	}
}

/* Handle driver intterupts
	MVPP2_ISR_ENABLE_REG 0xF2005420 + (4 * port=0-3) 0xF4005420 + (4 * port=0-3) 0xF6005420 + (4 * port=0-3)
		0:15 Enable port interrupt (write a 1 to enable, read shows status)
		16:31 DisablePortInterupt (write a 1 to disable)

	MVPP2_ISR_RX_TX_CAUSE_REG 0xF2005480 + (4 * port=0-3) 0xF4005480 + (4 * port=0-3) 0xF6005480 + (4 * port=0-3)
		0:7 Rx Occupied Descriptor (1 bit per queue of the Rx queues subgroup)
		8:15 Reserved
		16:23 Tx Occupied Descriptor (1 bit per Tx queue of the Ethernet port)
		24:31 Various Errors
*/
static int mvpp2_netmap_rx_irq(struct mvpp2_queue_vector *qv)
{
	struct netmap_adapter *na = NA(qv->port->dev);
	struct mvpp2_port *port = qv->port;
	unsigned int thread = mvpp2_cpu_to_thread(port->priv, smp_processor_id());
	uint32_t cause_rx_tx, cause_rx_exc, cause_rx, cause_tx, cause_misc;
	int dummy;

	if (!nm_netmap_on(na))
		return NM_IRQ_PASS;

	/* IRQ cause */
	cause_rx_tx = mvpp2_thread_read_relaxed(port->priv, thread, MVPP2_ISR_RX_TX_CAUSE_REG(port->id));

	/* ERROR */
	cause_misc = cause_rx_tx & MVPP2_CAUSE_MISC_SUM_MASK;
	if (cause_misc) {
		nm_prdis("NETMAP[%s:%d]CAUSE_MISC(0x%08x)\n", port->dev->name, thread, cause_misc);
		mvpp2_cause_error(port->dev, cause_misc);
		mvpp2_write(port->priv, MVPP2_ISR_MISC_CAUSE_REG, 0);
		mvpp2_thread_write(port->priv, thread, MVPP2_ISR_RX_TX_CAUSE_REG(port->id), cause_rx_tx & ~MVPP2_CAUSE_MISC_SUM_MASK);
	}
	cause_rx_exc = cause_rx_tx & MVPP2_CAUSE_RX_EXCEPTION_SUM_MASK;
	if (cause_rx_exc) {
		nm_prdis("NETMAP[%s:%d] CAUSE_RX_EXCEPTION(0x%08x)\n", port->dev->name, thread, cause_rx_exc);
		mvpp2_thread_write(port->priv, thread, MVPP2_ISR_RX_TX_CAUSE_REG(port->id),cause_rx_tx & ~MVPP2_CAUSE_RX_EXCEPTION_SUM_MASK);

		/* Allow napi process to finish */
		return NM_IRQ_PASS;
	}

	/* RX */
	cause_rx = cause_rx_tx & MVPP2_CAUSE_RXQ_OCCUP_DESC_ALL_MASK(port->priv->hw_version);
	if (cause_rx && netmap_rx_irq(port->dev, thread, &dummy) == NM_IRQ_COMPLETED) {
		nm_prdis("NETMAP[%s:%d] CAUSE_RXQ_OCCUP_DESC(0x%08x)\n", port->dev->name, thread, cause_rx);
		mvpp2_netmap_qvec_interrupt_disable (na, thread);
		// mvpp2_netmap_mask_rx_interrupts(qv);
	}

	/* TX */
	cause_tx = cause_rx_tx & MVPP2_CAUSE_TXQ_OCCUP_DESC_ALL_MASK;
	if (cause_tx) {
		nm_prdis("NETMAP[%s:%d] CAUSE_TXQ_OCCUP_DESC(0x%08x)\n", port->dev->name, thread, cause_tx);
	}

	wmb(); /* Force memory writes to complete */

	return IRQ_HANDLED;
}

static int mvpp2_netmap_txsync(struct netmap_kring *kring, int flags)
{
	struct netmap_adapter *na = (struct netmap_adapter *)kring->na;
	struct ifnet *ifp = na->ifp;

	nm_prerr("NETMAP[%s] txsync\n", ifp->name);
	BUG(); // TODO
	return 0;
}

static int mvpp2_netmap_rxsync(struct netmap_kring *kring, int flags)
{
	int force_update = (flags & NAF_FORCE_READ) || kring->nr_kflags & NKR_PENDINTR;
	struct netmap_adapter *na = (struct netmap_adapter *)kring->na;
	struct ifnet *ifp = na->ifp;
	struct mvpp2_port *port = netdev_priv(ifp);
	struct mvpp2_rx_queue *rxq = port->rxqs[kring->ring_id];
	struct netmap_ring *ring = kring->ring;
	u_int const lim = kring->nkr_num_slots - 1;
	u_int const head = kring->rhead;
	u_int nm_i;
	int i;

	if (!netif_carrier_ok(ifp))
	{
		return 0;
	}

	rmb(); /* Force memory reads to complete */

	if (head > lim)
	{
		nm_prerr("NETMAP[%s:%d] head(%u) > lim(%u)\n", ifp->name, rxq->id, head, lim);
		return netmap_ring_reinit(kring);
	}

	/* First part: import newly received packets */
	if (netmap_no_pendintr || force_update) {
		uint32_t hwtail_lim = nm_prev(kring->nr_hwcur, lim);
		int rx_received;
		int rx_complete = 0;
		int rx_packets = 0;
		uint64_t rx_bytes = 0;

		rx_received = mvpp2_rxq_received(port, rxq->id);
		nm_prdis("NETMAP[%s:%d] RX(%s) pkts=%d\n", ifp->name, rxq->id, force_update ? "force": "irq", rx_received);

		nm_i = kring->nr_hwtail;
		while (rx_received && nm_i != hwtail_lim) {
			struct mvpp2_rx_desc *rx_desc = mvpp2_rxq_next_desc_get(rxq);
			u32 status = mvpp2_rxdesc_status_get(port, rx_desc);
			int pool = (status & MVPP2_RXD_BM_POOL_ID_MASK) >> MVPP2_RXD_BM_POOL_ID_OFFS;
			phys_addr_t phys_addr = mvpp2_rxdesc_cookie_get(port, rx_desc);
			dma_addr_t dma_addr = mvpp2_rxdesc_dma_addr_get(port, rx_desc);
			void *data = (void *)phys_to_virt(phys_addr);
			struct page *page = virt_to_page(data);
			void *addr = NMB(na, &ring->slot[nm_i]);
			uint64_t offset = nm_get_offset(kring, &ring->slot[nm_i]);
			int length;

			if (status & (MVPP2_RXD_BUF_HDR|MVPP2_RXD_ERR_SUMMARY))
			{
				nm_prerr("NETMAP[%s:%d] rxsync error(0x%08x)\n", ifp->name, rxq->id, status);
				ifp->stats.rx_errors++;
				mvpp2_rx_error(port, rx_desc);
				if (status & MVPP2_RXD_BUF_HDR)
					mvpp2_buff_hdr_pool_put(port, rx_desc, pool, status);
				else
					mvpp2_bm_pool_put(port, pool, dma_addr, phys_addr);
				rx_received--;
				rx_complete++;
				continue;
			}

			prefetch(page);
			length = mvpp2_rxdesc_size_get(port, rx_desc);
			length -= MVPP2_MH_SIZE;
			rx_packets++;
			rx_bytes += length;

			dma_sync_single_for_cpu(((struct net_device *)ifp)->dev.parent, dma_addr, length + MVPP2_MH_SIZE, DMA_FROM_DEVICE);
			prefetch(data + MVPP2_MH_SIZE + MVPP2_SKB_HEADROOM);

			// TODO - populate pool with netmap buffer
			memcpy(addr + offset, data + MVPP2_MH_SIZE + MVPP2_SKB_HEADROOM, length);
			// print_hex_dump(KERN_INFO, "", DUMP_PREFIX_NONE, 16, 1, (u8*)(data + MVPP2_MH_SIZE + MVPP2_SKB_HEADROOM), length, true);

			mvpp2_bm_pool_put(port, pool, dma_addr, phys_addr);

			ring->slot[nm_i].len = length;
			ring->slot[nm_i].flags = 0;

			nm_i = kring->nr_hwtail = nm_next(nm_i, lim);
			rx_received--;
			rx_complete++;
		}

		if (rx_complete) {
			nm_prdis("NETMAP[%s:%d] RXQ Update used:%d free:%d\n", ifp->name, rxq->id, rx_complete, rx_complete);
			mvpp2_rxq_status_update(port, rxq->id, rx_complete, rx_complete);
			if (rx_packets) {
				struct mvpp2_pcpu_stats *stats = this_cpu_ptr(port->stats);
				u64_stats_update_begin(&stats->syncp);
				stats->rx_packets += rx_packets;
				stats->rx_bytes   += rx_bytes;
				u64_stats_update_end(&stats->syncp);
			}
		}

		/* Netmap ring is fully empty and no packets available to process */
		if (kring->nr_hwtail == kring->nr_hwcur && !rx_received) {
			kring->nr_kflags &= ~NKR_PENDINTR;
			mvpp2_netmap_qvec_interrupt_enable (na, kring->ring_id);
		}
	}

	/* Second part: skip past packets that userspace has released */
	nm_i = kring->nr_hwcur;
	for (i = 0; nm_i != head; i++) {
		struct netmap_slot *slot = &ring->slot[nm_i];
		void *addr = NMB(na, slot);

		/* We currently do not do anything here. But if we
		 * decide to use pool buffers for the Netmap ring,
		 * then this code would need to free the buffer
		 * back to the pool.
		 */
		if (addr == NETMAP_BUF_BASE(na))	/* bad buf */
		{
			printk("NETMAP[%s:%d] Bad buffer @ %p (index %d)\n", ifp->name, rxq->id, addr + nm_get_offset(kring, slot), nm_i);
			return netmap_ring_reinit(kring);
		}
		slot->flags &= ~NS_BUF_CHANGED;
		nm_i = nm_next(nm_i, lim);
	}
	kring->nr_hwcur = head;

	wmb(); /* Force memory writes to complete */

	return 0;
}

static int mvpp2_netmap_reg(struct netmap_adapter *na, int onoff)
{
	struct mvpp2_nm_adapter *mna = (struct mvpp2_nm_adapter *)na;
	struct mvpp2_port *port = netdev_priv(na->ifp);
	int r;

	nm_prdis("NETMAP[%s] %s (active=%d)", na->ifp->name, onoff ? "ON" : "OFF", na->active_fds);

	/* Enable or disable */
	if (onoff) {
		if (na->active_fds == 0) {
			/* We process TX completions before sending new packets */
			for (r = 0; r < port->priv->nthreads; r++) {
				mvpp2_netmap_mask_tx_interrupts(port, r);
				mna->irqs_enabled[r] = true;
			}
		}
		nm_set_native_flags(na);
	} else {
		nm_clear_native_flags(na);
		if (na->active_fds == 0) {
			for (r = 0; r < port->priv->nthreads; r++) {
				mvpp2_netmap_unmask_tx_interrupts(port, r);
				mvpp2_netmap_qvec_interrupt_enable (na, r);
			}
		}
	}

	for (r = 0; r < na->num_rx_rings; r++)
		(void)netmap_reset(na, NR_RX, r, 0);
	for (r = 0; r < na->num_tx_rings; r++)
		(void)netmap_reset(na, NR_TX, r, 0);

	return 0;
}

static int
mvpp2_netmap_config(struct netmap_adapter *na, struct nm_config_info *info)
{
	struct mvpp2_port *port = netdev_priv(na->ifp);

	info->num_tx_rings = port->ntxqs;
	info->num_rx_rings = port->nrxqs;
	info->num_tx_descs = 256;
	info->num_rx_descs = 256;
	info->rx_buf_maxsize = NETMAP_BUF_SIZE(na);

	return 0;
}

static void mvpp2_netmap_attach(struct mvpp2_port *port)
{
	struct netmap_adapter na;

	nm_prdis("NETMAP[%s] attach\n", port->dev->name);

	bzero(&na, sizeof(na));
	na.na_flags = NAF_OFFSETS;
	na.ifp = port->dev;
	na.num_tx_desc = 256;
	na.num_rx_desc = 256;
	na.nm_register = mvpp2_netmap_reg;
	na.nm_config = mvpp2_netmap_config;
	na.nm_txsync = mvpp2_netmap_txsync;
	na.nm_rxsync = mvpp2_netmap_rxsync;
	na.num_tx_rings = port->ntxqs;
	na.num_rx_rings = port->nrxqs;
	netmap_attach_ext(&na, sizeof(struct mvpp2_nm_adapter), 0);

	nm_prdis("NETMAP[%s] attached\n", port->dev->name);
}

static void mvpp2_netmap_detach(struct mvpp2_port *port)
{
	nm_prdis("NETMAP[%s] detach", port->dev->name);
	netmap_detach(port->dev);
}
