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

#define CQE_ADDR(CQ, idx) ((CQ)->cqe_base + ((CQ)->cqe_size * (idx)))

static int otx2_netmap_reg(struct netmap_adapter *na, int onoff)
{
	bool if_up;
	int r;

	if_up = netif_running(na->ifp);

	if (onoff)
		nm_set_native_flags(na);
	else
		nm_clear_native_flags(na);

	for (r = 0; r < na->num_rx_rings; r++) {
		(void)netmap_reset(na, NR_RX, r, 0);
	}

	return 0;
}

static int otx2_netmap_config(struct netmap_adapter *na, struct nm_config_info *info)
{
	info->num_rx_rings = info->num_tx_rings = 24;
	info->num_rx_descs = info->num_tx_descs = 1024;
	info->rx_buf_maxsize = NETMAP_BUF_SIZE(na);

	return 0;
}

static int otx2_nix_cq_op_status(struct otx2_nic *pfvf,
				 struct otx2_cq_queue *cq)
{
	u64 incr = (u64)(cq->cq_idx) << 32;
	u64 status;

	if (!pfvf->cq_op_addr)
		return -1;

	status = otx2_atomic64_fetch_add(incr, pfvf->cq_op_addr);

	if (unlikely(status & BIT_ULL(CQ_OP_STAT_OP_ERR) ||
		     status & BIT_ULL(CQ_OP_STAT_CQ_ERR))) {
		dev_err(pfvf->dev, "CQ stopped due to error");
		return -EINVAL;
	}

	cq->cq_tail = status & 0xFFFFF;
	cq->cq_head = (status >> 20) & 0xFFFFF;
	if (cq->cq_tail < cq->cq_head)
		cq->pend_cqe = (cq->cqe_cnt - cq->cq_head) +
				cq->cq_tail;
	else
		cq->pend_cqe = cq->cq_tail - cq->cq_head;

	return 0;
}

static void otx2_intr_cleanup (struct otx2_nic *pf, int qidx)
{
	/* Clear the IRQ */
	otx2_write64(pf, NIX_LF_CINTX_INT(qidx), BIT_ULL(0));

	/* Re-enable interrupts */
	otx2_write64(pf, NIX_LF_CINTX_ENA_W1S(qidx),
					BIT_ULL(0));
}

static int otx2_netmap_rxsync(struct netmap_kring *kring, int flags)
{
	struct netmap_adapter *na = (struct netmap_adapter *)kring->na;
	struct ifnet *ifp = na->ifp;
	struct netmap_ring *ring = kring->ring;
	u_int const lim = kring->nkr_num_slots - 1;
	u_int const head = kring->rhead;
	u_int qidx = kring->ring_id;
	u_int nm_i;
	u_int n;

	/* Do not process packets if the link is down */
	if (!netif_running (ifp))
		return 0;

	rmb(); /* Force memory reads to complete */

	/* HW */
	struct otx2_nic *pf = netdev_priv(ifp);
	struct otx2_qset *qset = &pf->qset;
	struct otx2_cq_poll *cq_poll = &qset->napi[qidx];
	struct otx2_cq_queue *rx_cq = NULL;
	int processed_cqe = 0;

	if (head > lim)
		goto ring_reset;

	if (netmap_no_pendintr || flags & NAF_FORCE_READ
				|| kring->nr_kflags & NKR_PENDINTR) {
		uint32_t hwtail_lim = nm_prev(kring->nr_hwcur, lim);

		/* Find Rx queue */
		int cq_idx = cq_poll->cq_ids[CQ_RX];
		rx_cq = &qset->cq[cq_idx];

		/* Find pending packets to process */
		if (otx2_nix_cq_op_status(pf, rx_cq))
			return 0;

		nm_i = kring->nr_hwtail;
		while (nm_i != hwtail_lim && rx_cq->pend_cqe) {
			/* Retrieve completion queue entry and descriptor */
			struct nix_cqe_rx_s *cqe = (struct nix_cqe_rx_s *)CQE_ADDR(rx_cq, rx_cq->cq_head);
			struct nix_rx_sg_s *sg = &cqe->sg;

			/* Retrieve nic buffer addr */
			u64 iova = cqe->sg.seg_addr;
			u64 pa = otx2_iova_to_phys(pf->iommu_domain, iova);
			void *va = phys_to_virt(pa);

			/* Retrieve nmap buffer addr */
			void *addr = NMB(na, &ring->slot[nm_i]);
			int length = sg->seg_size;

			/* Copy packet into nmap buffer */
			memcpy(addr, va, length);
			ring->slot[nm_i].len = length;
			ring->slot[nm_i].flags = 0;

			/* Advance and clear queue entries */
			rx_cq->cq_head++;
			rx_cq->cq_head &= (rx_cq->cqe_cnt - 1);
			cqe->hdr.cqe_type = NIX_XQE_TYPE_INVALID;
			cqe->sg.seg_addr = 0x00;
			rx_cq->pend_cqe--;
			processed_cqe++;

			/* Free up buffers */
			otx2_dma_unmap_page(pf, iova - OTX2_HEAD_ROOM,
				pf->rbsize, DMA_FROM_DEVICE);
			otx2_aura_freeptr(pf, rx_cq->cq_idx, iova);

			/* Advance nmap ring */
			nm_i = nm_next(nm_i, lim);
		}
		kring->nr_hwtail = nm_i;

		/* If both rings are empty */
		if (kring->nr_hwtail == kring->nr_hwcur && !rx_cq->pend_cqe) {
			kring->nr_kflags &= ~NKR_PENDINTR;
			otx2_intr_cleanup (pf, qidx);
		}

		/* Free CQEs to HW */
		otx2_write64(pf, NIX_LF_CQ_OP_DOOR,
		     ((u64)rx_cq->cq_idx << 32) | processed_cqe);
	}

	/* Second part: skip past packets that userspace has released */
	nm_i = kring->nr_hwcur;
	for (n = 0; nm_i != head; n++) {
		struct netmap_slot *slot = &ring->slot[nm_i];
		void *addr = NMB(na, slot);

		if (addr == NETMAP_BUF_BASE(na))	/* bad buf */
			goto ring_reset;
		slot->flags &= ~NS_BUF_CHANGED;
		nm_i = nm_next(nm_i, lim);
	}
	kring->nr_hwcur = head;

	return 0;

ring_reset:
	otx2_intr_cleanup (pf, qidx);
	return netmap_ring_reinit(kring);
}


static int otx2_netmap_txsync (struct netmap_kring *kring, int flags)
{
	/* Nothing to be done, not supported */
	return 0;
}

static void otx2_netmap_attach(struct otx2_nic *pf)
{
	struct netmap_adapter na;

	bzero(&na, sizeof(na));

	na.ifp = pf->netdev;

	na.nm_register = otx2_netmap_reg;
	na.nm_config = otx2_netmap_config;
	na.nm_rxsync = otx2_netmap_rxsync;
	na.nm_txsync = otx2_netmap_txsync;

	/* 24 Rx and 24 Tx queues */
	na.num_rx_rings = na.num_tx_rings = pf->hw.rx_queues;

	/* 1024 Rx ring size and 4096 Tx ring size
	 * Netmap does not allow for >1024 ring param.
	 * The otx2 driver requires at least 4K queues.
	 * Lie to netmap, as TX for this driver is not utilised.
	 */
	na.num_rx_desc = na.num_tx_desc = 1024;

	netmap_attach_ext(&na, sizeof(struct netmap_hw_adapter), 0);
}

static void otx2_netmap_detach(struct otx2_nic *pf)
{
	netmap_detach(pf->netdev);
}
