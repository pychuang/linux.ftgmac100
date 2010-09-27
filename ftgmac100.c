/*
 * Faraday FTGMAC100 Ethernet
 *
 * (C) Copyright 2009 Faraday Technology
 * Po-Yu Chuang <ratbert@faraday-tech.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/version.h>
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/mii.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <asm/io.h>

#include "ftgmac100.h"

#define USE_NAPI

#define DRV_NAME	"ftgmac100"
#define DRV_VERSION	"0.2"

#define RX_QUEUE_ENTRIES	256	/* must be power of 2 */
#define TX_QUEUE_ENTRIES	512	/* must be power of 2 */

#define MAX_PKT_SIZE		1518
#define RX_BUF_SIZE		2044	/* must be smaller than 0x3fff */

/******************************************************************************
 * priveate data
 *****************************************************************************/
struct ftgmac100_descs {
	struct ftgmac100_rxdes rxdes[RX_QUEUE_ENTRIES];
	struct ftgmac100_txdes txdes[TX_QUEUE_ENTRIES];
};

struct ftgmac100_priv
{
	struct resource *res;
	void __iomem *base;
	int irq;

	struct ftgmac100_descs *descs;
	dma_addr_t descs_dma_addr;

	unsigned int rx_pointer;
	unsigned int tx_clean_pointer;
	unsigned int tx_pointer;
	unsigned int tx_pending;

	spinlock_t hw_lock;
	spinlock_t rx_lock;
	spinlock_t tx_lock;

	struct net_device *dev;
#ifdef USE_NAPI
	struct napi_struct napi;
#endif

	struct net_device_stats stats;
	struct mii_bus *mii_bus;
	int phy_irq[PHY_MAX_ADDR];
	struct phy_device *phydev;
	int old_speed;
};

/******************************************************************************
 * internal functions (hardware register access)
 *****************************************************************************/
#define INT_MASK_RX_DISABLED	(FTGMAC100_INT_RPKT_LOST	|	\
				 FTGMAC100_INT_XPKT_ETH		|	\
				 FTGMAC100_INT_XPKT_LOST	|	\
				 FTGMAC100_INT_AHB_ERR		|	\
				 FTGMAC100_INT_PHYSTS_CHG)

#define INT_MASK_ALL_ENABLED	(INT_MASK_RX_DISABLED		|	\
				 FTGMAC100_INT_RPKT_BUF		|	\
				 FTGMAC100_INT_NO_RXBUF)

#define	INT_MASK_ALL_DISABLED	0

static inline void ftgmac100_set_rx_ring_base(struct ftgmac100_priv *priv, 
		dma_addr_t addr)
{
	iowrite32(addr, priv->base + FTGMAC100_OFFSET_RXR_BADR);
}

static inline void ftgmac100_set_rx_buffer_size(struct ftgmac100_priv *priv,
		unsigned int size)
{
	size = FTGMAC100_RBSR_SIZE(size);
	iowrite32(size, priv->base + FTGMAC100_OFFSET_RBSR);
}

static inline void ftgmac100_set_normal_prio_tx_ring_base(
		struct ftgmac100_priv *priv, dma_addr_t addr)
{
	iowrite32(addr, priv->base + FTGMAC100_OFFSET_NPTXR_BADR);
}

static inline void ftgmac100_txdma_normal_prio_start_polling(
		struct ftgmac100_priv *priv)
{
	iowrite32(1, priv->base + FTGMAC100_OFFSET_NPTXPD);
}

static int ftgmac100_reset_hw(struct ftgmac100_priv *priv)
{
	unsigned long flags;
	int i;

	/* NOTE: reset clears all registers */

	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(FTGMAC100_MACCR_SW_RST, priv->base + FTGMAC100_OFFSET_MACCR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	for (i = 0; i < 5; i++) {
		int maccr;

		maccr = ioread32(priv->base + FTGMAC100_OFFSET_MACCR);
		if (!(maccr & FTGMAC100_MACCR_SW_RST)) {
			return 0;
		}
		msleep_interruptible(10);
	}

	dev_err(&priv->dev->dev, "software reset failed\n");
	return -EIO;
}

static void ftgmac100_set_mac(struct ftgmac100_priv *priv, const unsigned char *mac)
{
	unsigned int maddr = mac[0] << 8 | mac[1];
	unsigned int laddr = mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5];

	iowrite32(maddr, priv->base + FTGMAC100_OFFSET_MAC_MADR);
	iowrite32(laddr, priv->base + FTGMAC100_OFFSET_MAC_LADR);
}

static void ftgmac100_init_hw(struct ftgmac100_priv *priv)
{
	unsigned long flags;

	/* setup ring buffer base registers */

	spin_lock_irqsave(&priv->hw_lock, flags);
	ftgmac100_set_rx_ring_base(priv,
		priv->descs_dma_addr + offsetof(struct ftgmac100_descs, rxdes));
	ftgmac100_set_normal_prio_tx_ring_base(priv,
		priv->descs_dma_addr + offsetof(struct ftgmac100_descs, txdes));

	ftgmac100_set_rx_buffer_size(priv, RX_BUF_SIZE);

	iowrite32(FTGMAC100_APTC_RXPOLL_CNT(1), priv->base + FTGMAC100_OFFSET_APTC);

	ftgmac100_set_mac(priv, priv->dev->dev_addr);
	spin_unlock_irqrestore(&priv->hw_lock, flags);
}

static void ftgmac100_start_hw(struct ftgmac100_priv *priv, int speed)
{
	unsigned long flags;
	int maccr = FTGMAC100_MACCR_TXDMA_EN
		  | FTGMAC100_MACCR_RXDMA_EN
		  | FTGMAC100_MACCR_TXMAC_EN
		  | FTGMAC100_MACCR_RXMAC_EN
		  | FTGMAC100_MACCR_FULLDUP
		  | FTGMAC100_MACCR_CRC_APD
		  | FTGMAC100_MACCR_RX_RUNT
		  | FTGMAC100_MACCR_RX_BROADPKT;

	switch (speed) {
	default:
	case 10:
		break;

	case 100:
		maccr |= FTGMAC100_MACCR_FAST_MODE;
		break;

	case 1000:
		maccr |= FTGMAC100_MACCR_GIGA_MODE;
		break;
	}

	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(maccr, priv->base + FTGMAC100_OFFSET_MACCR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);
}

static void ftgmac100_stop_hw(struct ftgmac100_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(0, priv->base + FTGMAC100_OFFSET_MACCR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);
}

/******************************************************************************
 * internal functions (receive descriptor)
 *****************************************************************************/
static inline int ftgmac100_rxdes_first_segment(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_FRS;
}

static inline int ftgmac100_rxdes_last_segment(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_LRS;
}

static inline int ftgmac100_rxdes_packet_ready(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_RXPKT_RDY;
}

static inline void ftgmac100_rxdes_set_dma_own(struct ftgmac100_rxdes *rxdes)
{
	/* clear status bits */
	rxdes->rxdes0 &= FTGMAC100_RXDES0_EDORR;
}

static inline int ftgmac100_rxdes_rx_error(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_RX_ERR;
}

static inline int ftgmac100_rxdes_crc_error(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_CRC_ERR;
}

static inline int ftgmac100_rxdes_frame_too_long(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_FTL;
}

static inline int ftgmac100_rxdes_runt(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_RUNT;
}

static inline int ftgmac100_rxdes_odd_nibble(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_RX_ODD_NB;
}

static inline unsigned int ftgmac100_rxdes_frame_length(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_VDBC;
}

static inline int ftgmac100_rxdes_multicast(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_MULTICAST;
}

static inline void ftgmac100_rxdes_set_end_of_ring(struct ftgmac100_rxdes *rxdes)
{
	rxdes->rxdes0 |= FTGMAC100_RXDES0_EDORR;
}

static inline void ftgmac100_rxdes_set_dma_addr(struct ftgmac100_rxdes *rxdes, dma_addr_t addr)
{
	rxdes->rxdes3 = addr;
}

static inline dma_addr_t ftgmac100_rxdes_get_dma_addr(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes3;
}

/* rxdes2 is not used by hardware, we use it to keep track of buffer */
static inline void ftgmac100_rxdes_set_va(struct ftgmac100_rxdes *rxdes, void *addr)
{
	rxdes->rxdes2 = (unsigned int)addr;
}

static inline void *ftgmac100_rxdes_get_va(struct ftgmac100_rxdes *rxdes)
{
	return (void *)rxdes->rxdes2;
}

/******************************************************************************
 * internal functions (receive)
 *****************************************************************************/
static inline int ftgmac100_next_rx_pointer(int pointer)
{
	return (pointer + 1) & (RX_QUEUE_ENTRIES - 1);
}

static inline void ftgmac100_rx_pointer_advance(struct ftgmac100_priv *priv)
{
	priv->rx_pointer = ftgmac100_next_rx_pointer(priv->rx_pointer);
}

static inline struct ftgmac100_rxdes *ftgmac100_current_rxdes(struct ftgmac100_priv *priv)
{
	return &priv->descs->rxdes[priv->rx_pointer];
}

static struct ftgmac100_rxdes *ftgmac100_rx_locate_first_segment(
		struct ftgmac100_priv *priv)
{
	struct ftgmac100_rxdes *rxdes = ftgmac100_current_rxdes(priv);

	while (ftgmac100_rxdes_packet_ready(rxdes)) {
		if (ftgmac100_rxdes_first_segment(rxdes))
			return rxdes;

		ftgmac100_rxdes_set_dma_own(rxdes);
		ftgmac100_rx_pointer_advance(priv);
		rxdes = ftgmac100_current_rxdes(priv);
	}

	return NULL;
}

static int ftgmac100_rx_packet_error(struct ftgmac100_priv *priv, struct ftgmac100_rxdes *rxdes)
{
	int error = 0;

	if (unlikely(ftgmac100_rxdes_rx_error(rxdes))) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "rx err\n");

		priv->stats.rx_errors++;
		error = 1;
	}

	if (unlikely(ftgmac100_rxdes_crc_error(rxdes))) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "rx crc err\n");

		priv->stats.rx_crc_errors++;
		error = 1;
	}

	if (unlikely(ftgmac100_rxdes_frame_too_long(rxdes))) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "rx frame too long\n");

		priv->stats.rx_length_errors++;
		error = 1;
	}

	if (unlikely(ftgmac100_rxdes_runt(rxdes))) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "rx runt\n");

		priv->stats.rx_length_errors++;
		error = 1;
	}

	if (unlikely(ftgmac100_rxdes_odd_nibble(rxdes))) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "rx odd nibble\n");

		priv->stats.rx_length_errors++;
		error = 1;
	}

	return error;
}

static void ftgmac100_rx_drop_packet(struct ftgmac100_priv *priv)
{
	struct ftgmac100_rxdes *rxdes = ftgmac100_current_rxdes(priv);
	int done = 0;

	if (printk_ratelimit())
		dev_dbg(&priv->dev->dev, "drop packet %p\n", rxdes);

	do {
		if (ftgmac100_rxdes_last_segment(rxdes))
			done = 1;

		ftgmac100_rxdes_set_dma_own(rxdes);
		ftgmac100_rx_pointer_advance(priv);
		rxdes = ftgmac100_current_rxdes(priv);
	} while (!done && ftgmac100_rxdes_packet_ready(rxdes));

	priv->stats.rx_dropped++;
}

static int ftgmac100_rx_packet(struct ftgmac100_priv *priv, int *processed)
{
	unsigned long flags;
	struct ftgmac100_rxdes *rxdes;
	struct sk_buff *skb;
	int length;
	int copied = 0;
	int done = 0;

	spin_lock_irqsave(&priv->rx_lock, flags);
	rxdes = ftgmac100_rx_locate_first_segment(priv);
	spin_unlock_irqrestore(&priv->rx_lock, flags);
	if (!rxdes)
		return 0;

	if (unlikely(ftgmac100_rx_packet_error(priv, rxdes))) {
		spin_lock_irqsave(&priv->rx_lock, flags);
		ftgmac100_rx_drop_packet(priv);
		spin_unlock_irqrestore(&priv->rx_lock, flags);
		return 1;
	}

	/* start processing */

	length = ftgmac100_rxdes_frame_length(rxdes);
	skb = dev_alloc_skb(length + NET_IP_ALIGN);
	if (unlikely(!skb)) {
		if (printk_ratelimit())
			dev_err(&priv->dev->dev, "rx skb alloc failed\n");

		spin_lock_irqsave(&priv->rx_lock, flags);
		ftgmac100_rx_drop_packet(priv);
		spin_unlock_irqrestore(&priv->rx_lock, flags);
		return 1;
	}

	if (unlikely(ftgmac100_rxdes_multicast(rxdes)))
		priv->stats.multicast++;

	skb_reserve(skb, NET_IP_ALIGN);

	do {
		dma_addr_t d = ftgmac100_rxdes_get_dma_addr(rxdes);
		void *buf = ftgmac100_rxdes_get_va(rxdes);
		int size;

		size = min(length - copied, RX_BUF_SIZE);

		dma_sync_single_for_cpu(NULL, d, RX_BUF_SIZE, DMA_FROM_DEVICE);
		memcpy(skb_put(skb, size), buf, size);

		copied += size;

		if (ftgmac100_rxdes_last_segment(rxdes))
			done = 1;

		dma_sync_single_for_device(NULL, d, RX_BUF_SIZE, DMA_FROM_DEVICE);

		spin_lock_irqsave(&priv->rx_lock, flags);

		ftgmac100_rxdes_set_dma_own(rxdes);

		ftgmac100_rx_pointer_advance(priv);
		rxdes = ftgmac100_current_rxdes(priv);

		spin_unlock_irqrestore(&priv->rx_lock, flags);
	} while (!done && copied < length);

	skb->protocol = eth_type_trans(skb, priv->dev);

	/* push packet to protocol stack */

#ifdef USE_NAPI
	netif_receive_skb(skb);
#else
	netif_rx(skb);
#endif

	priv->dev->last_rx = jiffies;

	priv->stats.rx_packets++;
	priv->stats.rx_bytes += skb->len;

	(*processed)++;

	return 1;
}

/******************************************************************************
 * internal functions (transmit descriptor)
 *****************************************************************************/
static inline void ftgmac100_txdes_reset(struct ftgmac100_txdes *txdes)
{
	/* clear all except end of ring bit */
	txdes->txdes0 &= FTGMAC100_TXDES0_EDOTR;
	txdes->txdes1 = 0;
	txdes->txdes2 = 0;
	txdes->txdes3 = 0;
}

static inline int ftgmac100_txdes_owned_by_dma(struct ftgmac100_txdes *txdes)
{
	return txdes->txdes0 & FTGMAC100_TXDES0_TXDMA_OWN;
}

static inline void ftgmac100_txdes_set_dma_own(struct ftgmac100_txdes *txdes)
{
	txdes->txdes0 |= FTGMAC100_TXDES0_TXDMA_OWN;
}

static inline void ftgmac100_txdes_set_end_of_ring(struct ftgmac100_txdes *txdes)
{
	txdes->txdes0 |= FTGMAC100_TXDES0_EDOTR;
}

static inline void ftgmac100_txdes_set_first_segment(struct ftgmac100_txdes *txdes)
{
	txdes->txdes0 |= FTGMAC100_TXDES0_FTS;
}

static inline void ftgmac100_txdes_set_last_segment(struct ftgmac100_txdes *txdes)
{
	txdes->txdes0 |= FTGMAC100_TXDES0_LTS;
}

static inline void ftgmac100_txdes_set_txint(struct ftgmac100_txdes *txdes)
{
	txdes->txdes1 |= FTGMAC100_TXDES1_TXIC;
}

static inline void ftgmac100_txdes_set_buffer_size(struct ftgmac100_txdes *txdes, unsigned int len)
{
	txdes->txdes0 |= FTGMAC100_TXDES0_TXBUF_SIZE(len);
}

static inline void ftgmac100_txdes_set_dma_addr(struct ftgmac100_txdes *txdes, dma_addr_t addr)
{
	txdes->txdes3 = addr;
}

/* txdes2 is not used by hardware, we use it to keep track of socket buffer */
static inline void ftgmac100_txdes_set_skb(struct ftgmac100_txdes *txdes, struct sk_buff *skb)
{
	txdes->txdes2 = (unsigned int)skb;
}

static inline struct sk_buff *ftgmac100_txdes_get_skb(struct ftgmac100_txdes *txdes)
{
	return (struct sk_buff *)txdes->txdes2;
}

/******************************************************************************
 * internal functions (transmit)
 *****************************************************************************/
static inline int ftgmac100_next_tx_pointer(int pointer)
{
	return (pointer + 1) & (TX_QUEUE_ENTRIES - 1);
}

static inline void ftgmac100_tx_pointer_advance(struct ftgmac100_priv *priv)
{
	priv->tx_pointer = ftgmac100_next_tx_pointer(priv->tx_pointer);
}

static inline void ftgmac100_tx_clean_pointer_advance(struct ftgmac100_priv *priv)
{
	priv->tx_clean_pointer = ftgmac100_next_tx_pointer(priv->tx_clean_pointer);
}

static inline struct ftgmac100_txdes *ftgmac100_current_txdes(struct ftgmac100_priv *priv)
{
	return &priv->descs->txdes[priv->tx_pointer];
}

static inline struct ftgmac100_txdes *ftgmac100_current_clean_txdes(struct ftgmac100_priv *priv)
{
	return &priv->descs->txdes[priv->tx_clean_pointer];
}

static int ftgmac100_tx_complete_packet(struct ftgmac100_priv *priv)
{
	struct ftgmac100_txdes *txdes;
	struct sk_buff *skb;
	struct skb_shared_info *sp;

	if (priv->tx_pending == 0)
		return 0;

	txdes = ftgmac100_current_clean_txdes(priv);

	if (ftgmac100_txdes_owned_by_dma(txdes))
		return 0;

	skb = ftgmac100_txdes_get_skb(txdes);
	sp = skb_shinfo(skb);

	priv->stats.tx_packets++;
	priv->stats.tx_bytes += skb->len;

	dma_unmap_single(NULL, sp->dma_head, skb_headlen(skb), DMA_TO_DEVICE);

	dev_kfree_skb_irq(skb);

	ftgmac100_txdes_reset(txdes);

	ftgmac100_tx_clean_pointer_advance(priv);

	priv->tx_pending--;
	netif_wake_queue(priv->dev);

	return 1;
}

static void ftgmac100_tx_complete(struct ftgmac100_priv *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->tx_lock, flags);
	while (ftgmac100_tx_complete_packet(priv));
	spin_unlock_irqrestore(&priv->tx_lock, flags);
}

static int ftgmac100_xmit(struct sk_buff *skb, struct ftgmac100_priv *priv)
{
	struct ftgmac100_txdes *txdes;
	unsigned int len = (skb->len < ETH_ZLEN) ? ETH_ZLEN : skb->len;
	unsigned long flags;

	txdes = ftgmac100_current_txdes(priv);
	ftgmac100_tx_pointer_advance(priv);

	/* setup TX descriptor */

	spin_lock_irqsave(&priv->tx_lock, flags);
	ftgmac100_txdes_set_skb(txdes, skb);
	ftgmac100_txdes_set_dma_addr(txdes, skb_shinfo(skb)->dma_head);

	ftgmac100_txdes_set_first_segment(txdes);
	ftgmac100_txdes_set_last_segment(txdes);
	ftgmac100_txdes_set_txint(txdes);
	ftgmac100_txdes_set_buffer_size(txdes, len);

	priv->tx_pending++;
	if (priv->tx_pending == TX_QUEUE_ENTRIES) {
		if (printk_ratelimit())
			dev_info(&priv->dev->dev, "tx queue full\n");

		netif_stop_queue(priv->dev);
	}

	/* start transmit */

	wmb();
	ftgmac100_txdes_set_dma_own(txdes);
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	spin_lock_irqsave(&priv->hw_lock, flags);
	ftgmac100_txdma_normal_prio_start_polling(priv);
	spin_unlock_irqrestore(&priv->hw_lock, flags);
	priv->dev->trans_start = jiffies;

	return NETDEV_TX_OK;
}

/******************************************************************************
 * internal functions (buffer)
 *****************************************************************************/
static void ftgmac100_free_buffers(struct ftgmac100_priv *priv)
{
	int i;

	for (i = 0; i < RX_QUEUE_ENTRIES; i += 2) {
		struct ftgmac100_rxdes *rxdes = &priv->descs->rxdes[i];
		dma_addr_t d = ftgmac100_rxdes_get_dma_addr(rxdes);
		void *page = ftgmac100_rxdes_get_va(rxdes);

		if (d)
			dma_unmap_single(NULL, d, PAGE_SIZE, DMA_FROM_DEVICE);

		if (page != NULL)
			free_page((unsigned long)page);
	}

	for (i = 0; i < TX_QUEUE_ENTRIES; i++) {
		struct ftgmac100_txdes *txdes = &priv->descs->txdes[i];
		struct sk_buff *skb = ftgmac100_txdes_get_skb(txdes);

		if (skb) {
			struct skb_shared_info *sp = skb_shinfo(skb);

			dma_unmap_single(NULL, sp->dma_head, skb_headlen(skb),
				DMA_TO_DEVICE);
			dev_kfree_skb(skb);
		}
	}

	dma_free_coherent(NULL, sizeof(struct ftgmac100_descs), priv->descs,
							priv->descs_dma_addr);
}

static int ftgmac100_alloc_buffers(struct ftgmac100_priv *priv)
{
	int i;

	priv->descs = dma_alloc_coherent(NULL, sizeof(struct ftgmac100_descs),
				&priv->descs_dma_addr, GFP_KERNEL | GFP_DMA);
	if (priv->descs == NULL)
		return -ENOMEM;

	memset(priv->descs, 0, sizeof(struct ftgmac100_descs));

	/* initialize RX ring */

	ftgmac100_rxdes_set_end_of_ring(&priv->descs->rxdes[RX_QUEUE_ENTRIES - 1]);

	for (i = 0; i < RX_QUEUE_ENTRIES; i += 2) {
		struct ftgmac100_rxdes *rxdes = &priv->descs->rxdes[i];
		void *page;
		dma_addr_t d;

		page = (void *)__get_free_page(GFP_KERNEL | GFP_DMA);
		if (page == NULL)
			goto err;

		d = dma_map_single(NULL, page, PAGE_SIZE, DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(NULL, d))) {
			free_page((unsigned long)page);
			goto err;
		}

		/*
		 * The hardware enforces a sub-2K maximum packet size, so we
		 * put two buffers on every hardware page.
		 */
		ftgmac100_rxdes_set_va(rxdes, page);
		ftgmac100_rxdes_set_va(rxdes + 1, page + PAGE_SIZE / 2);

		ftgmac100_rxdes_set_dma_addr(rxdes, d);
		ftgmac100_rxdes_set_dma_addr(rxdes + 1, d + PAGE_SIZE / 2);

		ftgmac100_rxdes_set_dma_own(rxdes);
		ftgmac100_rxdes_set_dma_own(rxdes + 1);
	}

	/* initialize TX ring */

	ftgmac100_txdes_set_end_of_ring(&priv->descs->txdes[TX_QUEUE_ENTRIES - 1]);
	return 0;

err:
	ftgmac100_free_buffers(priv);
	return -ENOMEM;
}

/******************************************************************************
 * internal functions (mdio)
 *****************************************************************************/
static void ftgmac100_adjust_link(struct net_device *dev)
{
	struct ftgmac100_priv *priv = netdev_priv(dev);
	struct phy_device *phydev = priv->phydev;
	unsigned long flags;
	int ier;

	if (phydev->speed == priv->old_speed)
		return;

	priv->old_speed = phydev->speed;

	spin_lock_irqsave(&priv->hw_lock, flags);
	ier = ioread32(priv->base + FTGMAC100_OFFSET_IER);

	/* disable all interrupts */
	iowrite32(INT_MASK_ALL_DISABLED, priv->base + FTGMAC100_OFFSET_IER);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	netif_stop_queue(dev);
	ftgmac100_stop_hw(priv);

	netif_start_queue(dev);
	ftgmac100_init_hw(priv);
	ftgmac100_start_hw(priv, phydev->speed);

	/* re-enable interrupts */
	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(ier, priv->base + FTGMAC100_OFFSET_IER);
	spin_unlock_irqrestore(&priv->hw_lock, flags);
}

static int ftgmac100_mii_probe(struct ftgmac100_priv *priv)
{
	struct net_device *dev = priv->dev;
	struct phy_device *phydev = NULL;
	int i;

	/* search for connect PHY device */
	for (i = 0; i < PHY_MAX_ADDR; i++) {
		struct phy_device *tmp = priv->mii_bus->phy_map[i];

		if (tmp) {
			phydev = tmp;
			break;
		}
	}

	/* now we are supposed to have a proper phydev, to attach to... */
	if (!phydev) {
		dev_info(&dev->dev, "%s: no PHY found\n",
			dev->name);
		return -ENODEV;
	}

	phydev = phy_connect(dev, dev_name(&phydev->dev), &ftgmac100_adjust_link, 0,
			PHY_INTERFACE_MODE_GMII);

	if (IS_ERR(phydev)) {
		dev_err(&dev->dev, "%s: Could not attach to PHY\n", dev->name);
		return PTR_ERR(phydev);
	}

	priv->phydev = phydev;

	return 0;
}

/******************************************************************************
 * struct mii_bus functions
 *****************************************************************************/
static int ftgmac100_mdiobus_read(struct mii_bus *bus, int phy_addr, int regnum)
{
	struct net_device *dev = bus->priv;
	struct ftgmac100_priv *priv = netdev_priv(dev);
	unsigned long flags;
	int phycr;
	int i;

	spin_lock_irqsave(&priv->hw_lock, flags);
	phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

	/* preserve MDC cycle threshold */
	phycr &= FTGMAC100_PHYCR_MDC_CYCTHR_MASK;

	phycr |= FTGMAC100_PHYCR_PHYAD(phy_addr)
	      |  FTGMAC100_PHYCR_REGAD(regnum)
	      |  FTGMAC100_PHYCR_MIIRD;

	iowrite32(phycr, priv->base + FTGMAC100_OFFSET_PHYCR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	for (i = 0; i < 10; i++) {
		phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

		if ((phycr & FTGMAC100_PHYCR_MIIRD) == 0) {
			int data;

			data = ioread32(priv->base + FTGMAC100_OFFSET_PHYDATA);
			return FTGMAC100_PHYDATA_MIIRDATA(data);
		}

		msleep(1);
	}

	dev_err(&dev->dev, "mdio read timed out\n");
	return -EIO;
}

static int ftgmac100_mdiobus_write(struct mii_bus *bus, int phy_addr,
	int regnum, u16 value)
{
	struct net_device *dev = bus->priv;
	struct ftgmac100_priv *priv = netdev_priv(dev);
	unsigned long flags;
	int phycr;
	int data;
	int i;

	spin_lock_irqsave(&priv->hw_lock, flags);
	phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

	/* preserve MDC cycle threshold */
	phycr &= FTGMAC100_PHYCR_MDC_CYCTHR_MASK;

	phycr |= FTGMAC100_PHYCR_PHYAD(phy_addr)
	      |  FTGMAC100_PHYCR_REGAD(regnum)
	      |  FTGMAC100_PHYCR_MIIWR;

	data = FTGMAC100_PHYDATA_MIIWDATA(value);

	iowrite32(data, priv->base + FTGMAC100_OFFSET_PHYDATA);
	iowrite32(phycr, priv->base + FTGMAC100_OFFSET_PHYCR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	for (i = 0; i < 10; i++) {
		phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

		if ((phycr & FTGMAC100_PHYCR_MIIWR) == 0)
			return 0;

		msleep(1);
	}

	dev_err(&dev->dev, "mdio write timed out\n");
	return -EIO;
}

static int ftgmac100_mdiobus_reset(struct mii_bus *bus)
{
	return 0;
}

/******************************************************************************
 * struct ethtool_ops functions
 *****************************************************************************/
static void ftgmac100_get_drvinfo(struct net_device *dev, struct ethtool_drvinfo *info)
{
	strcpy(info->driver, DRV_NAME);
	strcpy(info->version, DRV_VERSION);
	strcpy(info->bus_info, dev_name(&dev->dev));
}

static int ftgmac100_get_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct ftgmac100_priv *priv = netdev_priv(dev);

	return phy_ethtool_gset(priv->phydev, cmd);
}

static int ftgmac100_set_settings(struct net_device *dev, struct ethtool_cmd *cmd)
{
	struct ftgmac100_priv *priv = netdev_priv(dev);

	return phy_ethtool_sset(priv->phydev, cmd);
}

static struct ethtool_ops ftgmac100_ethtool_ops = {
	.set_settings		= ftgmac100_set_settings,
	.get_settings		= ftgmac100_get_settings,
	.get_drvinfo		= ftgmac100_get_drvinfo,
	.get_link		= ethtool_op_get_link,
};

/******************************************************************************
 * interrupt handler
 *****************************************************************************/
static irqreturn_t ftgmac100_interrupt(int irq, void *dev_id)
{
	struct net_device *dev = dev_id;
	struct ftgmac100_priv *priv = netdev_priv(dev);
	unsigned long flags;
	unsigned int status;

	spin_lock_irqsave(&priv->hw_lock, flags);
	status = ioread32(priv->base + FTGMAC100_OFFSET_ISR);
	iowrite32(status, priv->base + FTGMAC100_OFFSET_ISR);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	if (status & (FTGMAC100_INT_RPKT_BUF | FTGMAC100_INT_NO_RXBUF)) {
		/*
		 * FTGMAC100_INT_RPKT_BUF:
		 *	RX DMA has received packets into RX buffer successfully
		 *
		 * FTGMAC100_INT_NO_RXBUF:
		 *	RX buffer unavailable
		 */
#ifdef USE_NAPI
		/* Disable interrupts for polling */
		spin_lock_irqsave(&priv->hw_lock, flags);
		iowrite32(INT_MASK_RX_DISABLED, priv->base + FTGMAC100_OFFSET_IER);
		spin_unlock_irqrestore(&priv->hw_lock, flags);

		napi_schedule(&priv->napi);
#else
		int rx = 0;

		while (ftgmac100_rx_packet(priv, &rx));
#endif
	}

	if (status & FTGMAC100_INT_NO_RXBUF) {
		/* RX buffer unavailable */
		if (printk_ratelimit())
			dev_info(&dev->dev, "INT_NO_RXBUF\n");

		priv->stats.rx_over_errors++;
	}

	if (status & (FTGMAC100_INT_XPKT_ETH | FTGMAC100_INT_XPKT_LOST)) {
		/*
		 * FTGMAC100_INT_XPKT_ETH:
		 * 	 packet transmitted to ethernet successfully
		 *
		 * FTGMAC100_INT_XPKT_LOST:
		 *	packet transmitted to ethernet lost due to late
		 *	collision or excessive collision
		 */
		ftgmac100_tx_complete(priv);
	}

	if (status & FTGMAC100_INT_RPKT_LOST) {
		/* received packet lost due to RX FIFO full */
		if (printk_ratelimit())
			dev_info(&dev->dev, "INT_RPKT_LOST\n");

		priv->stats.rx_fifo_errors++;
	}

	if (status & FTGMAC100_INT_AHB_ERR) {
		/* AHB error */
		if (printk_ratelimit())
			dev_info(&dev->dev, "INT_AHB_ERR\n");

		/* do nothing */
	}

	if (status & FTGMAC100_INT_PHYSTS_CHG) {
		/* PHY link status change */
		if (printk_ratelimit())
			dev_info(&dev->dev, "INT_PHYSTS_CHG\n");
	}

	return IRQ_HANDLED;
}

/******************************************************************************
 * struct napi_struct functions
 *****************************************************************************/
#ifdef USE_NAPI
static int ftgmac100_poll(struct napi_struct *napi, int budget)
{
	struct ftgmac100_priv *priv = container_of(napi, struct ftgmac100_priv, napi);
	int retry;
	int rx = 0;

	do {
		retry = ftgmac100_rx_packet(priv, &rx);
	} while (retry && rx < budget);

	if (!retry || rx < budget) {
		unsigned long flags;

		napi_complete(napi);

		/* enable all interrupts */
		spin_lock_irqsave(&priv->hw_lock, flags);
		iowrite32(INT_MASK_ALL_ENABLED, priv->base + FTGMAC100_OFFSET_IER);
		spin_unlock_irqrestore(&priv->hw_lock, flags);
	}

	return rx;
}
#endif

/******************************************************************************
 * struct net_device_ops functions
 *****************************************************************************/
static int ftgmac100_open(struct net_device *dev)
{
	struct ftgmac100_priv *priv = netdev_priv(dev);
	unsigned long flags;
	int err;

	err = ftgmac100_alloc_buffers(priv);
	if (err) {
		dev_err(&dev->dev, "failed to allocate buffers\n");
		goto err_alloc;
	}

	err = request_irq(priv->irq, ftgmac100_interrupt, IRQF_SHARED, dev->name, dev);
	if (err) {
		dev_err(&dev->dev, "failed to request irq %d\n", priv->irq);
		goto err_irq;
	}

	priv->rx_pointer = 0;
	priv->tx_clean_pointer = 0;
	priv->tx_pointer = 0;
	priv->tx_pending = 0;

	spin_lock_init(&priv->hw_lock);
	spin_lock_init(&priv->rx_lock);
	spin_lock_init(&priv->tx_lock);

	err = ftgmac100_reset_hw(priv);
	if (err)
		goto err_hw;

	ftgmac100_init_hw(priv);
	ftgmac100_start_hw(priv, 10);

	phy_start(priv->phydev);

#ifdef USE_NAPI
	napi_enable(&priv->napi);
#endif
	netif_start_queue(dev);

	/* enable all interrupts */
	spin_lock_irqsave(&priv->hw_lock, flags);
	iowrite32(INT_MASK_ALL_ENABLED, priv->base + FTGMAC100_OFFSET_IER);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	return 0;

err_hw:
	free_irq(priv->irq, dev);
err_irq:
	ftgmac100_free_buffers(priv);
err_alloc:
	return err;
}

static int ftgmac100_stop(struct net_device *dev)
{
	struct ftgmac100_priv *priv = netdev_priv(dev);
	unsigned long flags;

	spin_lock_irqsave(&priv->hw_lock, flags);
	/* disable all interrupts */
	iowrite32(INT_MASK_ALL_DISABLED, priv->base + FTGMAC100_OFFSET_IER);
	spin_unlock_irqrestore(&priv->hw_lock, flags);

	netif_stop_queue(dev);
#ifdef USE_NAPI
	napi_disable(&priv->napi);
#endif
	phy_stop(priv->phydev);

	ftgmac100_stop_hw(priv);
	free_irq(priv->irq, dev);
	ftgmac100_free_buffers(priv);

	return 0;
}

static int ftgmac100_hard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ftgmac100_priv *priv = netdev_priv(dev);
	struct skb_shared_info *sp = skb_shinfo(skb);
	dma_addr_t map;

	if (unlikely(skb->len > MAX_PKT_SIZE)) {
		if (printk_ratelimit())
			dev_dbg(&dev->dev, "tx packet too big\n");

		priv->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	map = dma_map_single(NULL, skb->data, skb_headlen(skb), DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(NULL, map))) {
		/* drop packet */
		if (printk_ratelimit())
			dev_err(&dev->dev, "map socket buffer failed\n");

		priv->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	sp->dma_head = map;
	return ftgmac100_xmit(skb, priv);
}

static struct net_device_stats *ftgmac100_get_stats(struct net_device *dev)
{
	struct ftgmac100_priv *priv = netdev_priv(dev);

	return &priv->stats;
}

/* optional */
static int ftgmac100_do_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct ftgmac100_priv *priv = netdev_priv(dev);
	struct mii_ioctl_data *data = if_mii(ifr);

	return phy_mii_ioctl(priv->phydev, data, cmd);
}

static struct net_device_ops ftgmac100_netdev_ops = {
	.ndo_open		= ftgmac100_open,
	.ndo_stop		= ftgmac100_stop,
	.ndo_start_xmit		= ftgmac100_hard_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_get_stats		= ftgmac100_get_stats,
	.ndo_do_ioctl		= ftgmac100_do_ioctl,
};

/******************************************************************************
 * struct platform_driver functions
 *****************************************************************************/
static int ftgmac100_probe(struct platform_device *pdev)
{
	struct resource *res;
	int irq;
	struct net_device *dev;
	struct ftgmac100_priv *priv;
	int err;
	int i;

	if (pdev == NULL)
		return -ENODEV;

	if ((res = platform_get_resource(pdev, IORESOURCE_MEM, 0)) == 0) {
		return -ENXIO;
	}

	if ((irq = platform_get_irq(pdev, 0)) < 0) {
		return irq;
	}

	/* setup net_device */

	dev = alloc_etherdev(sizeof(struct ftgmac100_priv));
	if (dev == NULL) {
		err = -ENOMEM;
		goto err_alloc_etherdev;
	}

	SET_NETDEV_DEV(dev, &pdev->dev);

	SET_ETHTOOL_OPS(dev, &ftgmac100_ethtool_ops);
	dev->netdev_ops	= &ftgmac100_netdev_ops;

	platform_set_drvdata(pdev, dev);

	/* setup private data */

	priv = netdev_priv(dev);
	priv->dev = dev;

#ifdef USE_NAPI
	/* initialize NAPI */
	netif_napi_add(dev, &priv->napi, ftgmac100_poll, 64);
#endif

	/* map io memory */

	priv->res = request_mem_region(res->start, res->end - res->start,
			dev_name(&pdev->dev));
	if (priv->res == NULL) {
		dev_err(&pdev->dev, "Could not reserve memory region\n");
		err = -ENOMEM;
		goto err_req_mem;
	}

	priv->base = ioremap(res->start, res->end - res->start);
	if (priv->base == NULL) {
		dev_err(&pdev->dev, "Failed to ioremap ethernet registers\n");
		err = -EIO;
		goto err_ioremap;
	}

	priv->irq = irq;

	/* initialize mdio bus */

	priv->mii_bus = mdiobus_alloc();
	if (priv->mii_bus == NULL) {
		err = -EIO;
		goto err_alloc_mdiobus;
	}

	priv->mii_bus->name = "ftgmac100_mdio";
	snprintf(priv->mii_bus->id, MII_BUS_ID_SIZE, "ftgmac100_mii");

	priv->mii_bus->priv = dev;
	priv->mii_bus->read = ftgmac100_mdiobus_read;
	priv->mii_bus->write = ftgmac100_mdiobus_write;
	priv->mii_bus->reset = ftgmac100_mdiobus_reset;
	priv->mii_bus->irq = priv->phy_irq;

	for (i = 0; i < PHY_MAX_ADDR; i++)
		priv->mii_bus->irq[i] = PHY_POLL;

	err = mdiobus_register(priv->mii_bus);
	if (err) {
		dev_err(&pdev->dev, "Cannot register MDIO bus!\n");
		goto err_register_mdiobus;
	}

	err = ftgmac100_mii_probe(priv);
	if (err) {
		dev_err(&pdev->dev, "MII Probe failed!\n");
		goto err_mii_probe;
	}

	/* register network device */

	err = register_netdev(dev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register netdev\n");
		goto err_register_netdev;
	}

	dev_info(&dev->dev, "irq %d, mapped at %p\n", priv->irq, priv->base);

	if (is_zero_ether_addr(dev->dev_addr)) {
		random_ether_addr(dev->dev_addr);
		dev_info(&dev->dev, "generated random MAC address "
			"%.2x:%.2x:%.2x:%.2x:%.2x:%.2x.\n",
			dev->dev_addr[0], dev->dev_addr[1],
			dev->dev_addr[2], dev->dev_addr[3],
			dev->dev_addr[4], dev->dev_addr[5]);
	}

	return 0;

err_register_netdev:
	phy_disconnect(priv->phydev);
err_mii_probe:
	mdiobus_unregister(priv->mii_bus);
err_register_mdiobus:
	mdiobus_free(priv->mii_bus);
err_alloc_mdiobus:
	iounmap(priv->base);
err_ioremap:
	release_resource(priv->res);
err_req_mem:
	platform_set_drvdata(pdev, NULL);
	free_netdev(dev);
err_alloc_etherdev:
	return err;
}

static int ftgmac100_remove(struct platform_device *pdev)
{
	struct net_device *dev;
	struct ftgmac100_priv *priv;

	dev = platform_get_drvdata(pdev);
	priv = netdev_priv(dev);

	unregister_netdev(dev);

	phy_disconnect(priv->phydev);
	mdiobus_unregister(priv->mii_bus);
	mdiobus_free(priv->mii_bus);

	iounmap(priv->base);
	release_resource(priv->res);

	platform_set_drvdata(pdev, NULL);

	free_netdev(dev);

	return 0;
}

static struct platform_driver ftgmac100_driver = {
	.probe		= ftgmac100_probe,
	.remove		= ftgmac100_remove,
	.driver		= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

/******************************************************************************
 * initialization / finalization
 *****************************************************************************/
static int __init ftgmac100_init(void)
{
	printk(KERN_INFO "Loading " DRV_NAME ": version " DRV_VERSION " ...\n");
	return platform_driver_register(&ftgmac100_driver);
}

static void __exit ftgmac100_exit(void)
{
	platform_driver_unregister(&ftgmac100_driver);
}

module_init(ftgmac100_init);
module_exit(ftgmac100_exit);

MODULE_AUTHOR("Po-Yu Chuang <ratbert@faraday-tech.com>");
MODULE_DESCRIPTION("FTGMAC100 driver");
MODULE_LICENSE("GPL");
