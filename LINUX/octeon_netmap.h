/*
 * Copyright 2017, Allied Telesis Labs New Zealand, Ltd
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

#define MAX_RINGS 4
#define WORK_GROUP(port,offset) ((get_interface_num(port) * MAX_RINGS) + offset)
#define OUTPUT_QUEUE	 0

static inline int get_interface_num(int ipd_port)
{
   if (ipd_port < 16)
       return 0;
   else if (ipd_port < 24)
       return 1;
   else if (ipd_port < 36)
       return 2;
   else
       panic("Illegal IPD port number (%d)\n", ipd_port);

   return -1;
}

static void
octeon_netmap_irq_enable(struct netmap_adapter *na, int queue, int onoff)
{
	struct netmap_kring *kring = NMR(na, NR_RX);
	union cvmx_pow_wq_int wq_int;
	union cvmx_pow_wq_int_thrx int_thr;
	struct ifnet *ifp = na->ifp;
	struct octeon_ethernet *priv = netdev_priv(ifp);
	int port = priv->port;
	int group = WORK_GROUP(port, queue);

	if (netmap_verbose & 0x8000)
		D("%s: IRQ %s", na->name, onoff ? "ON" : "OFF");

	mtx_lock(&kring->q_lock);
	if (onoff) {
		/* Enable */
		int_thr.u64 = 0;
		int_thr.s.iq_thr = 1;
		cvmx_write_csr(CVMX_POW_WQ_INT_THRX(group), int_thr.u64);
	}
	else {
		/* Disable */
		int_thr.u64 = 0;
		int_thr.s.iq_thr = 0;
		cvmx_write_csr (CVMX_POW_WQ_INT_THRX(group), int_thr.u64);
		/* Clear */
		wq_int.u64 = 0;
		wq_int.s.wq_int = 1 << group;
		cvmx_write_csr(CVMX_POW_WQ_INT, wq_int.u64);
	}
	mtx_unlock(&kring->q_lock);
}

static irqreturn_t
octeon_netmap_do_interrupt(int cpl, void *data)
{
	struct netmap_adapter *na = (struct netmap_adapter *) data;
	int queue = (cpl - OCTEON_IRQ_WORKQ0) % MAX_RINGS;
	int work_done = 0;

	octeon_netmap_irq_enable(na, queue, 0);
	return netmap_rx_irq(na->ifp, queue, &work_done) ==
			NM_IRQ_COMPLETED ? IRQ_HANDLED : IRQ_NONE;
}

static int
octeon_netmap_enable_port(struct netmap_adapter *na, int ringid)
{
	struct ifnet *ifp = na->ifp;
	struct octeon_ethernet *priv = netdev_priv(ifp);
	union cvmx_pip_prt_tagx tag_config;
	union cvmx_pip_prt_cfgx port_config;
	int port = priv->port;
	int group = WORK_GROUP(port, ringid);

	D("%s: Enable Port %d: group %d irq:%d",
			ifp->name, port, group, OCTEON_IRQ_WORKQ0 + group);

	/* Register an IRQ handler to receive POW interrupts */
	if (request_irq(OCTEON_IRQ_WORKQ0 + group,
			octeon_netmap_do_interrupt, 0, "Octeon-Netmap", na))
		panic("Could not acquire Ethernet IRQ %d\n",
				      OCTEON_IRQ_WORKQ0 + group);

	/* Enable POW interrupt when our port has at least one packet */
	octeon_netmap_irq_enable(na, ringid, 1);

	/* Configure port to use netmap receive group */
	tag_config.u64 = cvmx_read_csr(CVMX_PIP_PRT_TAGX(port));
	tag_config.s.grptagbase = (get_interface_num(port) * MAX_RINGS);

	switch (nr_cpu_ids)
	{
	case 4:
		tag_config.s.grptagmask = 0xC;
		break;
	case 2:
		tag_config.s.grptagmask = 0xE;
		break;
	default:
		D("Octeon Netmap needs a mask for other core counts\n");
		BUG();
		break;
	}
	tag_config.s.grptag = 1;
	tag_config.s.grp = (get_interface_num(port) * MAX_RINGS);
	cvmx_write_csr(CVMX_PIP_PRT_TAGX(port), tag_config.u64);

	/* Set the default QOS level for the port */
	port_config.u64 = cvmx_read_csr (CVMX_PIP_PRT_CFGX (port));
	port_config.s.qos = 1;
	cvmx_write_csr (CVMX_PIP_PRT_CFGX (port), port_config.u64);

	/* Free all skb that are currently queued for TX */
	cvm_oct_tx_shutdown_dev(priv->netdev);
	cvmx_fau_atomic_write32(priv->fau + OUTPUT_QUEUE * 4, 0);

	return 0;
}

static int
octeon_netmap_disable_port(struct netmap_adapter *na, int ringid)
{
	const int coreid = cvmx_get_core_num();
	struct ifnet *ifp = na->ifp;
	struct octeon_ethernet *priv = netdev_priv(ifp);
	union cvmx_pow_wq_int_thrx int_thr;
	union cvmx_pip_prt_tagx pip_prt_tagx;
	int port = priv->port;
	int group = WORK_GROUP(port, ringid);

	D("%s: Disable Port %d: group %d irq:%d",
			ifp->name, port, group, OCTEON_IRQ_WORKQ0 + group);

	/* Re-initialise the skb_to_free counter */
	cvmx_fau_atomic_write32(priv->fau + OUTPUT_QUEUE * 4, 0);

	/* Configure port to use default receive group */
	pip_prt_tagx.u64 = cvmx_read_csr(CVMX_PIP_PRT_TAGX(port));
	pip_prt_tagx.s.grp = pow_receive_group;
	pip_prt_tagx.s.grptag = 0;
	cvmx_write_csr(CVMX_PIP_PRT_TAGX(port), pip_prt_tagx.u64);

	/* Pass all queued packets to the default work queue */
	while (1) {
		u64 old_group_mask;
		cvmx_wqe_t *work;

		old_group_mask = cvmx_read_csr(CVMX_POW_PP_GRP_MSKX(coreid));
		cvmx_write_csr(CVMX_POW_PP_GRP_MSKX(coreid),
			(old_group_mask & ~0xFFFFull) | 1 << group);
		work = cvmx_pow_work_request_sync(CVMX_POW_NO_WAIT);
		cvmx_write_csr(CVMX_POW_PP_GRP_MSKX(coreid), old_group_mask);
		if (!work)
			break;
		cvmx_pow_work_submit(work, work->word1.tag, work->word1.tag_type,
						cvmx_wqe_get_qos(work), pow_receive_group);
	}

	/* Disable POW interrupt */
	int_thr.u64 = 0;
	int_thr.s.tc_en = 0;
	int_thr.s.tc_thr = 1;
	cvmx_write_csr(CVMX_POW_WQ_INT_THRX(group), int_thr.u64);

	/* Free the interrupt handler */
	free_irq(OCTEON_IRQ_WORKQ0 + group, na);

	return 0;
}

/*
 * Register/unregister. We are already under netmap lock.
 */
static int
octeon_netmap_reg(struct netmap_adapter *na, int onoff)
{
	int i, t;

	D("%s: %s", na->ifp->name, onoff ? "ON" : "OFF");


	/* Enable or disable */
	if (onoff) {
		if (na->active_fds == 0) {
			for (i = 0; i < na->num_rx_rings; i++)
				octeon_netmap_enable_port(na, i);
		}
		for_rx_tx(t) {
			for (i = 0; i <= nma_get_nrings(na, t); i++) {
				struct netmap_kring *kring = &NMR(na, t)[i];
				if (nm_kring_pending_on(kring)) {
					kring->nr_mode = NKR_NETMAP_ON;
				}
			}
		}
		nm_set_native_flags(na);
	} else {
		nm_clear_native_flags(na);
		for_rx_tx(t) {
			for (i = 0; i <= nma_get_nrings(na, t); i++) {
				struct netmap_kring *kring = &NMR(na, t)[i];
				if (nm_kring_pending_off(kring)) {
					kring->nr_mode = NKR_NETMAP_OFF;
				}
			}
		}
		if (na->active_fds == 0) {
			for (i = 0; i < na->num_rx_rings; i++)
				octeon_netmap_disable_port(na, i);
		}
	}

	return 0;
}

/*
 * Reconcile kernel and user view of the transmit ring.
 */
static int
octeon_netmap_txsync(struct netmap_kring *kring, int flags)
{
	struct netmap_adapter *na = kring->na;
	struct ifnet *ifp = na->ifp;
	struct octeon_ethernet *priv = netdev_priv(ifp);
	struct netmap_ring *ring = kring->ring;
	u_int const lim = kring->nkr_num_slots - 1;
	u_int const head = kring->rhead;
	u_int nm_i;
	u_int n;

	if (!netif_carrier_ok(ifp)) {
		return 0;
	}

	/* First part: process new packets to send */
	nm_i = kring->nr_hwcur;
	for (n = 0; nm_i != head; n++) {
		struct netmap_slot *slot = &ring->slot[nm_i];
		cvmx_pko_command_word0_t pko_command;
		union cvmx_buf_ptr hw_buffer;
		void *buffer = NMB(na, &ring->slot[nm_i]);
		u_int len = slot->len;

		NM_CHECK_ADDR_LEN(na, buffer, len);

		if (unlikely(netmap_verbose))
			D("%s: TX %d bytes @ %p (index %d)",
				kring->name, len, buffer, nm_i);

		/* The Netmap allocated buffers are fine
		 * for transmission via the PKO. We just
		 * have to make sure we do not let the PKO
		 * free the buffer as it would put it back
		 * in the FPA - not what we want.
		 */
		pko_command.u64 = 0;
#ifdef __LITTLE_ENDIAN
		pko_command.s.le = 1;
#endif
		pko_command.s.n2 = 1;
		pko_command.s.segs = 1;
		pko_command.s.total_bytes = len;
		pko_command.s.size0 = CVMX_FAU_OP_SIZE_32;
		pko_command.s.subone0 = 1;
		pko_command.s.dontfree = 1;
		pko_command.s.reg0 = priv->fau + OUTPUT_QUEUE * 4;

		hw_buffer.u64 = 0;
		hw_buffer.s.addr = XKPHYS_TO_PHYS((u64)buffer);
		hw_buffer.s.pool = 0;
		hw_buffer.s.size = len;

		cvmx_pko_send_packet_prepare(priv->port,
				priv->queue + OUTPUT_QUEUE, CVMX_PKO_LOCK_NONE);
		if (unlikely(cvmx_pko_send_packet_finish(priv->port,
								 priv->queue + OUTPUT_QUEUE,
								 pko_command, hw_buffer,
								 CVMX_PKO_LOCK_NONE))) {
			printk_ratelimited("%s: Failed to send the packet\n", kring->name);
		}

		slot->flags &= ~(NS_REPORT | NS_BUF_CHANGED);
		nm_i = nm_next(nm_i, lim);
	}
	kring->nr_hwcur = head;

	/* Second part: reclaim buffers for completed transmissions */
	if (flags & NAF_FORCE_RECLAIM || nm_kr_txempty(kring)) {
		int32_t transmitted;

		/* The PKO decrements the counter stored in the FAU
		 * for each buffer it transmits. This allows us to
		 * shift the tail and hence free up the TX ring.
		 */
		transmitted = -(cvmx_fau_fetch_and_add32(priv->fau + OUTPUT_QUEUE * 4, 0));
		cvmx_fau_atomic_add32(priv->fau + OUTPUT_QUEUE * 4, transmitted);

		if (unlikely(netmap_verbose))
			D("%s: Free %d TX buffers", kring->name, transmitted);

		nm_i = kring->nr_hwtail + transmitted;
		if (nm_i >= kring->nkr_num_slots) {
			nm_i -= kring->nkr_num_slots;
		}
		kring->nr_hwtail = nm_i;
	}

	return 0;
}

/*
 * Reconcile kernel and user view of the receive ring.
 */
static int
octeon_netmap_rxsync(struct netmap_kring *kring, int flags)
{
	const int coreid = cvmx_get_core_num();
	struct netmap_adapter *na = kring->na;
	struct ifnet *ifp = na->ifp;
	struct octeon_ethernet *priv = netdev_priv(ifp);
	int port = priv->port;
	int group = WORK_GROUP(port, kring->ring_id);
	struct netmap_ring *ring = kring->ring;
	u_int const lim = kring->nkr_num_slots - 1;
	u_int const head = kring->rhead;
	u_int nm_i;
	u_int n;

	if (!netif_carrier_ok(ifp))
		return 0;

	if (head > lim)
		return netmap_ring_reinit(kring);

	/* First part: import newly received packets */
	if (netmap_no_pendintr ||
		flags & NAF_FORCE_READ ||
		kring->nr_kflags & NKR_PENDINTR) {
		uint16_t slot_flags = kring->nkr_slot_flags;
		uint32_t hwtail_lim = nm_prev(kring->nr_hwcur, lim);
		void *addr = NULL;
		int length = 0;

		nm_i = kring->nr_hwtail;
		while (nm_i != hwtail_lim) {
			u64 old_group_mask;
			cvmx_wqe_t *work;

			old_group_mask = cvmx_read_csr(CVMX_POW_PP_GRP_MSKX(coreid));
			cvmx_write_csr(CVMX_POW_PP_GRP_MSKX(coreid),
				(old_group_mask & ~0xFFFFull) | 1 << group);
			work = cvmx_pow_work_request_sync(CVMX_POW_NO_WAIT);
			cvmx_write_csr(CVMX_POW_PP_GRP_MSKX(coreid), old_group_mask);
			if (!work)
				break;

			if (unlikely(work->word2.snoip.rcv_error ||
					     cvmx_wqe_get_grp(work) != group)) {
				cvmx_pow_work_submit(work, work->word1.tag, work->word1.tag_type,
							cvmx_wqe_get_qos(work), pow_receive_group);
				continue;
			}

			/* Currently we copy into the netmap allocated buffer
			 * as Octeon buffers are shared for all interfaces
			 * and hence we cannot force a single interface to
			 * use only Netmap buffers allocated for its ring.
			 * Potentially we could teach Netmap to use the FPA
			 * buffer pool for all Netmap rings and hence avoid
			 * this copy.
			 */
			addr = NMB(na, &ring->slot[nm_i]);
			length = work->word1.len;
			memcpy(addr, cvmx_phys_to_ptr (work->packet_ptr.s.addr), length);
			cvm_oct_free_work(work);

			if (unlikely(netmap_verbose))
				D("%s: RX %d bytes @ %p (index %d)",
					kring->name, length, addr, nm_i);

			ring->slot[nm_i].len = length;
			ring->slot[nm_i].flags = slot_flags;

			nm_i = kring->nr_hwtail = nm_next(nm_i, lim);
		}
		if (nm_i != hwtail_lim) {
			kring->nr_kflags &= ~NKR_PENDINTR;
			octeon_netmap_irq_enable(na, kring->ring_id, 1);
		}
	}

	/* Second part: skip past packets that userspace has released */
	nm_i = kring->nr_hwcur;
	for (n = 0; nm_i != head; n++) {
		struct netmap_slot *slot = &ring->slot[nm_i];
		void *addr = NMB(na, &ring->slot[nm_i]);

		/* We currently do not do anything here. But if we
		 * decide to use FPA buffers for the Netmap ring,
		 * then this code would need to free the buffer
		 * back to the FPA pool.
		 */
		if (unlikely(netmap_verbose))
			D("%s: Free RX buffer @ %p (index %d)",
				kring->name, addr, nm_i);

		if (addr == NETMAP_BUF_BASE(na)) /* bad buf */
			goto ring_reset;
		slot->flags &= ~NS_BUF_CHANGED;
		nm_i = nm_next(nm_i, lim);
	}
	kring->nr_hwcur = head;

	return 0;
ring_reset:
	return netmap_ring_reinit(kring);
}

static void
octeon_netmap_attach(struct octeon_ethernet *priv)
{
	struct netmap_adapter na;

	bzero(&na, sizeof(na));
	na.ifp = priv->netdev;
	na.num_tx_desc = 128;
	na.num_rx_desc = 128;
	na.nm_register = octeon_netmap_reg;
	na.nm_txsync = octeon_netmap_txsync;
	na.nm_rxsync = octeon_netmap_rxsync;
	na.num_tx_rings = 1;
	na.num_rx_rings = num_online_cpus();
	netmap_attach(&na);
}

static void
octeon_netmap_detach(struct octeon_ethernet *priv)
{
	netmap_detach(priv->netdev);
}
