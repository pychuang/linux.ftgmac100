/*
 * Faraday FTGMAC100 Gigabit Ethernet
 *
 * (C) Copyright 2009-2011 Faraday Technology
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

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/etherdevice.h>
#include <linux/ethtool.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/netdevice.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/version.h>

#include "ftgmac100.h"

#define DRV_NAME	"ftgmac100"
#define DRV_VERSION	"0.2"

#define RX_QUEUE_ENTRIES	256	/* must be power of 2 */
#define TX_QUEUE_ENTRIES	512	/* must be power of 2 */

#define MAX_PKT_SIZE		1518
#define RX_BUF_SIZE		2044	/* must be smaller than 0x3fff */

/******************************************************************************
 * private data
 *****************************************************************************/
struct ftgmac100_descs {
	struct ftgmac100_rxdes rxdes[RX_QUEUE_ENTRIES];
	struct ftgmac100_txdes txdes[TX_QUEUE_ENTRIES];
};

struct ftgmac100 {
	struct resource *res;
	void __iomem *base;
	int irq;

	struct ftgmac100_descs *descs;
	dma_addr_t descs_dma_addr;

	unsigned int rx_pointer;
	unsigned int tx_clean_pointer;
	unsigned int tx_pointer;
	unsigned int tx_pending;

	spinlock_t tx_lock;

	struct net_device *netdev;
	struct device *dev;
	struct napi_struct napi;

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

#define INT_MASK_ALL_DISABLED	0

static inline void ftgmac100_set_rx_ring_base(struct ftgmac100 *priv,
		dma_addr_t addr)
{
	iowrite32(addr, priv->base + FTGMAC100_OFFSET_RXR_BADR);
}

static inline void ftgmac100_set_rx_buffer_size(struct ftgmac100 *priv,
		unsigned int size)
{
	size = FTGMAC100_RBSR_SIZE(size);
	iowrite32(size, priv->base + FTGMAC100_OFFSET_RBSR);
}

static inline void ftgmac100_set_normal_prio_tx_ring_base(
		struct ftgmac100 *priv, dma_addr_t addr)
{
	iowrite32(addr, priv->base + FTGMAC100_OFFSET_NPTXR_BADR);
}

static inline void ftgmac100_txdma_normal_prio_start_polling(
		struct ftgmac100 *priv)
{
	iowrite32(1, priv->base + FTGMAC100_OFFSET_NPTXPD);
}

static int ftgmac100_reset_hw(struct ftgmac100 *priv)
{
	struct net_device *netdev = priv->netdev;
	int i;

	/* NOTE: reset clears all registers */
	iowrite32(FTGMAC100_MACCR_SW_RST, priv->base + FTGMAC100_OFFSET_MACCR);
	for (i = 0; i < 5; i++) {
		int maccr;

		maccr = ioread32(priv->base + FTGMAC100_OFFSET_MACCR);
		if (!(maccr & FTGMAC100_MACCR_SW_RST))
			return 0;

		msleep(10);
	}

	netdev_err(netdev, "software reset failed\n");
	return -EIO;
}

static void ftgmac100_set_mac(struct ftgmac100 *priv, const unsigned char *mac)
{
	unsigned int maddr = mac[0] << 8 | mac[1];
	unsigned int laddr = mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5];

	iowrite32(maddr, priv->base + FTGMAC100_OFFSET_MAC_MADR);
	iowrite32(laddr, priv->base + FTGMAC100_OFFSET_MAC_LADR);
}

static void ftgmac100_init_hw(struct ftgmac100 *priv)
{
	/* setup ring buffer base registers */
	ftgmac100_set_rx_ring_base(priv,
		priv->descs_dma_addr + offsetof(struct ftgmac100_descs, rxdes));
	ftgmac100_set_normal_prio_tx_ring_base(priv,
		priv->descs_dma_addr + offsetof(struct ftgmac100_descs, txdes));

	ftgmac100_set_rx_buffer_size(priv, RX_BUF_SIZE);

	iowrite32(FTGMAC100_APTC_RXPOLL_CNT(1),
		priv->base + FTGMAC100_OFFSET_APTC);

	ftgmac100_set_mac(priv, priv->netdev->dev_addr);
}

static void ftgmac100_start_hw(struct ftgmac100 *priv, int speed)
{
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

	iowrite32(maccr, priv->base + FTGMAC100_OFFSET_MACCR);
}

static void ftgmac100_stop_hw(struct ftgmac100 *priv)
{
	iowrite32(0, priv->base + FTGMAC100_OFFSET_MACCR);
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

static inline unsigned int ftgmac100_rxdes_frame_length(
		struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_VDBC;
}

static inline int ftgmac100_rxdes_multicast(struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes0 & FTGMAC100_RXDES0_MULTICAST;
}

static inline void ftgmac100_rxdes_set_end_of_ring(
		struct ftgmac100_rxdes *rxdes)
{
	rxdes->rxdes0 |= FTGMAC100_RXDES0_EDORR;
}

static inline void ftgmac100_rxdes_set_dma_addr(struct ftgmac100_rxdes *rxdes,
		dma_addr_t addr)
{
	rxdes->rxdes3 = addr;
}

static inline dma_addr_t ftgmac100_rxdes_get_dma_addr(
		struct ftgmac100_rxdes *rxdes)
{
	return rxdes->rxdes3;
}

/* rxdes2 is not used by hardware, we use it to keep track of buffer */
static inline void ftgmac100_rxdes_set_va(struct ftgmac100_rxdes *rxdes,
		void *addr)
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

static inline void ftgmac100_rx_pointer_advance(struct ftgmac100 *priv)
{
	priv->rx_pointer = ftgmac100_next_rx_pointer(priv->rx_pointer);
}

static inline struct ftgmac100_rxdes *ftgmac100_current_rxdes(
		struct ftgmac100 *priv)
{
	return &priv->descs->rxdes[priv->rx_pointer];
}

static struct ftgmac100_rxdes *ftgmac100_rx_locate_first_segment(
		struct ftgmac100 *priv)
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

static int ftgmac100_rx_packet_error(struct ftgmac100 *priv,
		struct ftgmac100_rxdes *rxdes)
{
	struct net_device *netdev = priv->netdev;
	int error = 0;

	if (unlikely(ftgmac100_rxdes_rx_error(rxdes))) {
		if (printk_ratelimit())
			netdev_info(netdev, "rx err\n");

		netdev->stats.rx_errors++;
		error = 1;
	}

	if (unlikely(ftgmac100_rxdes_crc_error(rxdes))) {
		if (printk_ratelimit())
			netdev_info(netdev, "rx crc err\n");

		netdev->stats.rx_crc_errors++;
		error = 1;
	}

	if (unlikely(ftgmac100_rxdes_frame_too_long(rxdes))) {
		if (printk_ratelimit())
			netdev_info(netdev, "rx frame too long\n");

		netdev->stats.rx_length_errors++;
		error = 1;
	}

	if (unlikely(ftgmac100_rxdes_runt(rxdes))) {
		if (printk_ratelimit())
			netdev_info(netdev, "rx runt\n");

		netdev->stats.rx_length_errors++;
		error = 1;
	}

	if (unlikely(ftgmac100_rxdes_odd_nibble(rxdes))) {
		if (printk_ratelimit())
			netdev_info(netdev, "rx odd nibble\n");

		netdev->stats.rx_length_errors++;
		error = 1;
	}

	return error;
}

static void ftgmac100_rx_drop_packet(struct ftgmac100 *priv)
{
	struct net_device *netdev = priv->netdev;
	struct ftgmac100_rxdes *rxdes = ftgmac100_current_rxdes(priv);
	int done = 0;

	if (printk_ratelimit())
		netdev_dbg(netdev, "drop packet %p\n", rxdes);

	do {
		if (ftgmac100_rxdes_last_segment(rxdes))
			done = 1;

		ftgmac100_rxdes_set_dma_own(rxdes);
		ftgmac100_rx_pointer_advance(priv);
		rxdes = ftgmac100_current_rxdes(priv);
	} while (!done && ftgmac100_rxdes_packet_ready(rxdes));

	netdev->stats.rx_dropped++;
}

static int ftgmac100_rx_packet(struct ftgmac100 *priv, int *processed)
{
	struct net_device *netdev = priv->netdev;
	struct ftgmac100_rxdes *rxdes;
	struct sk_buff *skb;
	int length;
	int copied = 0;
	int done = 0;

	rxdes = ftgmac100_rx_locate_first_segment(priv);
	if (!rxdes)
		return 0;

	if (unlikely(ftgmac100_rx_packet_error(priv, rxdes))) {
		ftgmac100_rx_drop_packet(priv);
		return 1;
	}

	/* start processing */

	length = ftgmac100_rxdes_frame_length(rxdes);
	skb = netdev_alloc_skb_ip_align(netdev, length);
	if (unlikely(!skb)) {
		if (printk_ratelimit())
			netdev_err(netdev, "rx skb alloc failed\n");

		ftgmac100_rx_drop_packet(priv);
		return 1;
	}

	if (unlikely(ftgmac100_rxdes_multicast(rxdes)))
		netdev->stats.multicast++;

	do {
		dma_addr_t map = ftgmac100_rxdes_get_dma_addr(rxdes);
		void *buf = ftgmac100_rxdes_get_va(rxdes);
		int size;

		size = min(length - copied, RX_BUF_SIZE);

		dma_sync_single_for_cpu(priv->dev, map, RX_BUF_SIZE,
			DMA_FROM_DEVICE);
		memcpy(skb_put(skb, size), buf, size);

		copied += size;

		if (ftgmac100_rxdes_last_segment(rxdes))
			done = 1;

		dma_sync_single_for_device(priv->dev, map, RX_BUF_SIZE,
			DMA_FROM_DEVICE);

		ftgmac100_rxdes_set_dma_own(rxdes);

		ftgmac100_rx_pointer_advance(priv);
		rxdes = ftgmac100_current_rxdes(priv);
	} while (!done && copied < length);

	skb->protocol = eth_type_trans(skb, netdev);

	/* push packet to protocol stack */

	netif_receive_skb(skb);

	netdev->stats.rx_packets++;
	netdev->stats.rx_bytes += skb->len;

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

static inline void ftgmac100_txdes_set_end_of_ring(
		struct ftgmac100_txdes *txdes)
{
	txdes->txdes0 |= FTGMAC100_TXDES0_EDOTR;
}

static inline void ftgmac100_txdes_set_first_segment(
		struct ftgmac100_txdes *txdes)
{
	txdes->txdes0 |= FTGMAC100_TXDES0_FTS;
}

static inline void ftgmac100_txdes_set_last_segment(
		struct ftgmac100_txdes *txdes)
{
	txdes->txdes0 |= FTGMAC100_TXDES0_LTS;
}

static inline void ftgmac100_txdes_set_txint(struct ftgmac100_txdes *txdes)
{
	txdes->txdes1 |= FTGMAC100_TXDES1_TXIC;
}

static inline void ftgmac100_txdes_set_buffer_size(
		struct ftgmac100_txdes *txdes, unsigned int len)
{
	txdes->txdes0 |= FTGMAC100_TXDES0_TXBUF_SIZE(len);
}

static inline void ftgmac100_txdes_set_dma_addr(struct ftgmac100_txdes *txdes,
		dma_addr_t addr)
{
	txdes->txdes3 = addr;
}

static inline dma_addr_t ftgmac100_txdes_get_dma_addr(
		struct ftgmac100_txdes *txdes)
{
	return txdes->txdes3;
}

/* txdes2 is not used by hardware, we use it to keep track of socket buffer */
static inline void ftgmac100_txdes_set_skb(struct ftgmac100_txdes *txdes,
		struct sk_buff *skb)
{
	txdes->txdes2 = (unsigned int)skb;
}

static inline struct sk_buff *ftgmac100_txdes_get_skb(
		struct ftgmac100_txdes *txdes)
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

static inline void ftgmac100_tx_pointer_advance(struct ftgmac100 *priv)
{
	priv->tx_pointer = ftgmac100_next_tx_pointer(priv->tx_pointer);
}

static inline void ftgmac100_tx_clean_pointer_advance(struct ftgmac100 *priv)
{
	priv->tx_clean_pointer =
		ftgmac100_next_tx_pointer(priv->tx_clean_pointer);
}

static inline struct ftgmac100_txdes *ftgmac100_current_txdes(
		struct ftgmac100 *priv)
{
	return &priv->descs->txdes[priv->tx_pointer];
}

static inline struct ftgmac100_txdes *ftgmac100_current_clean_txdes(
		struct ftgmac100 *priv)
{
	return &priv->descs->txdes[priv->tx_clean_pointer];
}

static int ftgmac100_tx_complete_packet(struct ftgmac100 *priv)
{
	struct net_device *netdev = priv->netdev;
	struct ftgmac100_txdes *txdes;
	struct sk_buff *skb;
	dma_addr_t map;

	if (priv->tx_pending == 0)
		return 0;

	txdes = ftgmac100_current_clean_txdes(priv);

	if (ftgmac100_txdes_owned_by_dma(txdes))
		return 0;

	skb = ftgmac100_txdes_get_skb(txdes);
	map = ftgmac100_txdes_get_dma_addr(txdes);

	netdev->stats.tx_packets++;
	netdev->stats.tx_bytes += skb->len;

	dma_unmap_single(priv->dev, map, skb_headlen(skb), DMA_TO_DEVICE);

	dev_kfree_skb_irq(skb);

	ftgmac100_txdes_reset(txdes);

	ftgmac100_tx_clean_pointer_advance(priv);

	priv->tx_pending--;
	netif_wake_queue(netdev);

	return 1;
}

static void ftgmac100_tx_complete(struct ftgmac100 *priv)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->tx_lock, flags);
	while (ftgmac100_tx_complete_packet(priv))
		;
	spin_unlock_irqrestore(&priv->tx_lock, flags);
}

static int ftgmac100_xmit(struct ftgmac100 *priv, struct sk_buff *skb,
		dma_addr_t map)
{
	struct net_device *netdev = priv->netdev;
	struct ftgmac100_txdes *txdes;
	unsigned int len = (skb->len < ETH_ZLEN) ? ETH_ZLEN : skb->len;
	unsigned long flags;

	txdes = ftgmac100_current_txdes(priv);
	ftgmac100_tx_pointer_advance(priv);

	/* setup TX descriptor */

	spin_lock_irqsave(&priv->tx_lock, flags);
	ftgmac100_txdes_set_skb(txdes, skb);
	ftgmac100_txdes_set_dma_addr(txdes, map);

	ftgmac100_txdes_set_first_segment(txdes);
	ftgmac100_txdes_set_last_segment(txdes);
	ftgmac100_txdes_set_txint(txdes);
	ftgmac100_txdes_set_buffer_size(txdes, len);

	priv->tx_pending++;
	if (priv->tx_pending == TX_QUEUE_ENTRIES) {
		if (printk_ratelimit())
			netdev_info(netdev, "tx queue full\n");

		netif_stop_queue(netdev);
	}

	/* start transmit */

	wmb();
	ftgmac100_txdes_set_dma_own(txdes);
	spin_unlock_irqrestore(&priv->tx_lock, flags);

	ftgmac100_txdma_normal_prio_start_polling(priv);

	return NETDEV_TX_OK;
}

/******************************************************************************
 * internal functions (buffer)
 *****************************************************************************/
static void ftgmac100_free_buffers(struct ftgmac100 *priv)
{
	int i;

	for (i = 0; i < RX_QUEUE_ENTRIES; i += 2) {
		struct ftgmac100_rxdes *rxdes = &priv->descs->rxdes[i];
		dma_addr_t map = ftgmac100_rxdes_get_dma_addr(rxdes);
		void *page = ftgmac100_rxdes_get_va(rxdes);

		if (map)
			dma_unmap_single(priv->dev, map, PAGE_SIZE,
				DMA_FROM_DEVICE);

		if (page)
			free_page((unsigned long)page);
	}

	for (i = 0; i < TX_QUEUE_ENTRIES; i++) {
		struct ftgmac100_txdes *txdes = &priv->descs->txdes[i];
		struct sk_buff *skb = ftgmac100_txdes_get_skb(txdes);

		if (skb) {
			dma_addr_t map;

			map = ftgmac100_txdes_get_dma_addr(txdes);
			dma_unmap_single(priv->dev, map, skb_headlen(skb),
				DMA_TO_DEVICE);
			dev_kfree_skb(skb);
		}
	}

	dma_free_coherent(priv->dev, sizeof(struct ftgmac100_descs),
		priv->descs, priv->descs_dma_addr);
}

static int ftgmac100_alloc_buffers(struct ftgmac100 *priv)
{
	int i;

	priv->descs = dma_alloc_coherent(priv->dev,
		sizeof(struct ftgmac100_descs), &priv->descs_dma_addr,
		GFP_KERNEL | GFP_DMA);
	if (!priv->descs)
		return -ENOMEM;

	memset(priv->descs, 0, sizeof(struct ftgmac100_descs));

	/* initialize RX ring */

	ftgmac100_rxdes_set_end_of_ring(
		&priv->descs->rxdes[RX_QUEUE_ENTRIES - 1]);

	for (i = 0; i < RX_QUEUE_ENTRIES; i += 2) {
		struct ftgmac100_rxdes *rxdes = &priv->descs->rxdes[i];
		void *page;
		dma_addr_t map;

		page = (void *)__get_free_page(GFP_KERNEL | GFP_DMA);
		if (!page)
			goto err;

		map = dma_map_single(priv->dev, page, PAGE_SIZE, DMA_FROM_DEVICE);
		if (unlikely(dma_mapping_error(priv->dev, map))) {
			free_page((unsigned long)page);
			goto err;
		}

		/*
		 * The hardware enforces a sub-2K maximum packet size, so we
		 * put two buffers on every hardware page.
		 */
		ftgmac100_rxdes_set_va(rxdes, page);
		ftgmac100_rxdes_set_va(rxdes + 1, page + PAGE_SIZE / 2);

		ftgmac100_rxdes_set_dma_addr(rxdes, map);
		ftgmac100_rxdes_set_dma_addr(rxdes + 1, map + PAGE_SIZE / 2);

		ftgmac100_rxdes_set_dma_own(rxdes);
		ftgmac100_rxdes_set_dma_own(rxdes + 1);
	}

	/* initialize TX ring */

	ftgmac100_txdes_set_end_of_ring(
		&priv->descs->txdes[TX_QUEUE_ENTRIES - 1]);
	return 0;

err:
	ftgmac100_free_buffers(priv);
	return -ENOMEM;
}

/******************************************************************************
 * internal functions (mdio)
 *****************************************************************************/
static void ftgmac100_adjust_link(struct net_device *netdev)
{
	struct ftgmac100 *priv = netdev_priv(netdev);
	struct phy_device *phydev = priv->phydev;
	int ier;

	if (phydev->speed == priv->old_speed)
		return;

	priv->old_speed = phydev->speed;

	ier = ioread32(priv->base + FTGMAC100_OFFSET_IER);

	/* disable all interrupts */
	iowrite32(INT_MASK_ALL_DISABLED, priv->base + FTGMAC100_OFFSET_IER);

	netif_stop_queue(netdev);
	ftgmac100_stop_hw(priv);

	netif_start_queue(netdev);
	ftgmac100_init_hw(priv);
	ftgmac100_start_hw(priv, phydev->speed);

	/* re-enable interrupts */
	iowrite32(ier, priv->base + FTGMAC100_OFFSET_IER);
}

static int ftgmac100_mii_probe(struct ftgmac100 *priv)
{
	struct net_device *netdev = priv->netdev;
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
		netdev_info(netdev, "%s: no PHY found\n",
			netdev->name);
		return -ENODEV;
	}

	phydev = phy_connect(netdev, dev_name(&phydev->dev),
		&ftgmac100_adjust_link, 0, PHY_INTERFACE_MODE_GMII);

	if (IS_ERR(phydev)) {
		netdev_err(netdev, "%s: Could not attach to PHY\n", netdev->name);
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
	struct net_device *netdev = bus->priv;
	struct ftgmac100 *priv = netdev_priv(netdev);
	int phycr;
	int i;

	phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

	/* preserve MDC cycle threshold */
	phycr &= FTGMAC100_PHYCR_MDC_CYCTHR_MASK;

	phycr |= FTGMAC100_PHYCR_PHYAD(phy_addr)
	      |  FTGMAC100_PHYCR_REGAD(regnum)
	      |  FTGMAC100_PHYCR_MIIRD;

	iowrite32(phycr, priv->base + FTGMAC100_OFFSET_PHYCR);

	for (i = 0; i < 10; i++) {
		phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

		if ((phycr & FTGMAC100_PHYCR_MIIRD) == 0) {
			int data;

			data = ioread32(priv->base + FTGMAC100_OFFSET_PHYDATA);
			return FTGMAC100_PHYDATA_MIIRDATA(data);
		}

		msleep(1);
	}

	netdev_err(netdev, "mdio read timed out\n");
	return -EIO;
}

static int ftgmac100_mdiobus_write(struct mii_bus *bus, int phy_addr,
	int regnum, u16 value)
{
	struct net_device *netdev = bus->priv;
	struct ftgmac100 *priv = netdev_priv(netdev);
	int phycr;
	int data;
	int i;

	phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

	/* preserve MDC cycle threshold */
	phycr &= FTGMAC100_PHYCR_MDC_CYCTHR_MASK;

	phycr |= FTGMAC100_PHYCR_PHYAD(phy_addr)
	      |  FTGMAC100_PHYCR_REGAD(regnum)
	      |  FTGMAC100_PHYCR_MIIWR;

	data = FTGMAC100_PHYDATA_MIIWDATA(value);

	iowrite32(data, priv->base + FTGMAC100_OFFSET_PHYDATA);
	iowrite32(phycr, priv->base + FTGMAC100_OFFSET_PHYCR);

	for (i = 0; i < 10; i++) {
		phycr = ioread32(priv->base + FTGMAC100_OFFSET_PHYCR);

		if ((phycr & FTGMAC100_PHYCR_MIIWR) == 0)
			return 0;

		msleep(1);
	}

	netdev_err(netdev, "mdio write timed out\n");
	return -EIO;
}

static int ftgmac100_mdiobus_reset(struct mii_bus *bus)
{
	return 0;
}

/******************************************************************************
 * struct ethtool_ops functions
 *****************************************************************************/
static void ftgmac100_get_drvinfo(struct net_device *netdev,
		struct ethtool_drvinfo *info)
{
	strcpy(info->driver, DRV_NAME);
	strcpy(info->version, DRV_VERSION);
	strcpy(info->bus_info, dev_name(&netdev->dev));
}

static int ftgmac100_get_settings(struct net_device *netdev,
		struct ethtool_cmd *cmd)
{
	struct ftgmac100 *priv = netdev_priv(netdev);

	return phy_ethtool_gset(priv->phydev, cmd);
}

static int ftgmac100_set_settings(struct net_device *netdev,
		struct ethtool_cmd *cmd)
{
	struct ftgmac100 *priv = netdev_priv(netdev);

	return phy_ethtool_sset(priv->phydev, cmd);
}

static const struct ethtool_ops ftgmac100_ethtool_ops = {
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
	struct net_device *netdev = dev_id;
	struct ftgmac100 *priv = netdev_priv(netdev);
	unsigned int status;

	status = ioread32(priv->base + FTGMAC100_OFFSET_ISR);
	iowrite32(status, priv->base + FTGMAC100_OFFSET_ISR);

	if (status & (FTGMAC100_INT_RPKT_BUF | FTGMAC100_INT_NO_RXBUF)) {
		/*
		 * FTGMAC100_INT_RPKT_BUF:
		 *	RX DMA has received packets into RX buffer successfully
		 *
		 * FTGMAC100_INT_NO_RXBUF:
		 *	RX buffer unavailable
		 */

		/* Disable interrupts for polling */
		iowrite32(INT_MASK_RX_DISABLED,
			priv->base + FTGMAC100_OFFSET_IER);

		napi_schedule(&priv->napi);
	}

	if (status & FTGMAC100_INT_NO_RXBUF) {
		/* RX buffer unavailable */
		if (printk_ratelimit())
			netdev_info(netdev, "INT_NO_RXBUF\n");

		netdev->stats.rx_over_errors++;
	}

	if (status & (FTGMAC100_INT_XPKT_ETH | FTGMAC100_INT_XPKT_LOST)) {
		/*
		 * FTGMAC100_INT_XPKT_ETH:
		 *	packet transmitted to ethernet successfully
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
			netdev_info(netdev, "INT_RPKT_LOST\n");

		netdev->stats.rx_fifo_errors++;
	}

	if (status & FTGMAC100_INT_AHB_ERR) {
		/* AHB error */
		if (printk_ratelimit())
			netdev_info(netdev, "INT_AHB_ERR\n");

		/* do nothing */
	}

	if (status & FTGMAC100_INT_PHYSTS_CHG) {
		/* PHY link status change */
		if (printk_ratelimit())
			netdev_info(netdev, "INT_PHYSTS_CHG\n");
	}

	return IRQ_HANDLED;
}

/******************************************************************************
 * struct napi_struct functions
 *****************************************************************************/
static int ftgmac100_poll(struct napi_struct *napi, int budget)
{
	struct ftgmac100 *priv = container_of(napi, struct ftgmac100, napi);
	int retry;
	int rx = 0;

	do {
		retry = ftgmac100_rx_packet(priv, &rx);
	} while (retry && rx < budget);

	if (!retry || rx < budget) {
		napi_complete(napi);

		/* enable all interrupts */
		iowrite32(INT_MASK_ALL_ENABLED,
			priv->base + FTGMAC100_OFFSET_IER);
	}

	return rx;
}

/******************************************************************************
 * struct net_device_ops functions
 *****************************************************************************/
static int ftgmac100_open(struct net_device *netdev)
{
	struct ftgmac100 *priv = netdev_priv(netdev);
	int err;

	err = ftgmac100_alloc_buffers(priv);
	if (err) {
		netdev_err(netdev, "failed to allocate buffers\n");
		goto err_alloc;
	}

	err = request_irq(priv->irq, ftgmac100_interrupt, IRQF_SHARED,
		netdev->name, netdev);
	if (err) {
		netdev_err(netdev, "failed to request irq %d\n", priv->irq);
		goto err_irq;
	}

	priv->rx_pointer = 0;
	priv->tx_clean_pointer = 0;
	priv->tx_pointer = 0;
	priv->tx_pending = 0;

	err = ftgmac100_reset_hw(priv);
	if (err)
		goto err_hw;

	ftgmac100_init_hw(priv);
	ftgmac100_start_hw(priv, 10);

	phy_start(priv->phydev);

	napi_enable(&priv->napi);
	netif_start_queue(netdev);

	/* enable all interrupts */
	iowrite32(INT_MASK_ALL_ENABLED, priv->base + FTGMAC100_OFFSET_IER);
	return 0;

err_hw:
	free_irq(priv->irq, netdev);
err_irq:
	ftgmac100_free_buffers(priv);
err_alloc:
	return err;
}

static int ftgmac100_stop(struct net_device *netdev)
{
	struct ftgmac100 *priv = netdev_priv(netdev);

	/* disable all interrupts */
	iowrite32(INT_MASK_ALL_DISABLED, priv->base + FTGMAC100_OFFSET_IER);

	netif_stop_queue(netdev);
	napi_disable(&priv->napi);
	phy_stop(priv->phydev);

	ftgmac100_stop_hw(priv);
	free_irq(priv->irq, netdev);
	ftgmac100_free_buffers(priv);

	return 0;
}

static int ftgmac100_hard_start_xmit(struct sk_buff *skb,
		struct net_device *netdev)
{
	struct ftgmac100 *priv = netdev_priv(netdev);
	dma_addr_t map;

	if (unlikely(skb->len > MAX_PKT_SIZE)) {
		if (printk_ratelimit())
			netdev_dbg(netdev, "tx packet too big\n");

		netdev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	map = dma_map_single(priv->dev, skb->data, skb_headlen(skb),
		DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(priv->dev, map))) {
		/* drop packet */
		if (printk_ratelimit())
			netdev_err(netdev, "map socket buffer failed\n");

		netdev->stats.tx_dropped++;
		dev_kfree_skb(skb);
		return NETDEV_TX_OK;
	}

	return ftgmac100_xmit(priv, skb, map);
}

/* optional */
static int ftgmac100_do_ioctl(struct net_device *netdev, struct ifreq *ifr,
		int cmd)
{
	struct ftgmac100 *priv = netdev_priv(netdev);
	struct mii_ioctl_data *data = if_mii(ifr);

	return phy_mii_ioctl(priv->phydev, data, cmd);
}

static const struct net_device_ops ftgmac100_netdev_ops = {
	.ndo_open		= ftgmac100_open,
	.ndo_stop		= ftgmac100_stop,
	.ndo_start_xmit		= ftgmac100_hard_start_xmit,
	.ndo_set_mac_address	= eth_mac_addr,
	.ndo_validate_addr	= eth_validate_addr,
	.ndo_do_ioctl		= ftgmac100_do_ioctl,
};

/******************************************************************************
 * struct platform_driver functions
 *****************************************************************************/
static int ftgmac100_probe(struct platform_device *pdev)
{
	struct resource *res;
	int irq;
	struct net_device *netdev;
	struct ftgmac100 *priv;
	int err;
	int i;

	if (!pdev)
		return -ENODEV;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENXIO;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	/* setup net_device */
	netdev = alloc_etherdev(sizeof(*priv));
	if (!netdev) {
		err = -ENOMEM;
		goto err_alloc_etherdev;
	}

	SET_NETDEV_DEV(netdev, &pdev->dev);

	SET_ETHTOOL_OPS(netdev, &ftgmac100_ethtool_ops);
	netdev->netdev_ops = &ftgmac100_netdev_ops;

	platform_set_drvdata(pdev, netdev);

	/* setup private data */
	priv = netdev_priv(netdev);
	priv->netdev = netdev;
	priv->dev = &pdev->dev;

	spin_lock_init(&priv->tx_lock);

	/* initialize NAPI */
	netif_napi_add(netdev, &priv->napi, ftgmac100_poll, 64);

	/* map io memory */
	priv->res = request_mem_region(res->start, res->end - res->start,
			dev_name(&pdev->dev));
	if (!priv->res) {
		dev_err(&pdev->dev, "Could not reserve memory region\n");
		err = -ENOMEM;
		goto err_req_mem;
	}

	priv->base = ioremap(res->start, res->end - res->start);
	if (!priv->base) {
		dev_err(&pdev->dev, "Failed to ioremap ethernet registers\n");
		err = -EIO;
		goto err_ioremap;
	}

	priv->irq = irq;

	/* initialize mdio bus */
	priv->mii_bus = mdiobus_alloc();
	if (!priv->mii_bus) {
		err = -EIO;
		goto err_alloc_mdiobus;
	}

	priv->mii_bus->name = "ftgmac100_mdio";
	snprintf(priv->mii_bus->id, MII_BUS_ID_SIZE, "ftgmac100_mii");

	priv->mii_bus->priv = netdev;
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
	err = register_netdev(netdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to register netdev\n");
		goto err_register_netdev;
	}

	netdev_info(netdev, "irq %d, mapped at %p\n", priv->irq, priv->base);

	if (!is_valid_ether_addr(netdev->dev_addr)) {
		random_ether_addr(netdev->dev_addr);
		netdev_info(netdev, "generated random MAC address "
			"%.2x:%.2x:%.2x:%.2x:%.2x:%.2x.\n",
			netdev->dev_addr[0], netdev->dev_addr[1],
			netdev->dev_addr[2], netdev->dev_addr[3],
			netdev->dev_addr[4], netdev->dev_addr[5]);
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
	netif_napi_del(&priv->napi);
	platform_set_drvdata(pdev, NULL);
	free_netdev(netdev);
err_alloc_etherdev:
	return err;
}

static int ftgmac100_remove(struct platform_device *pdev)
{
	struct net_device *netdev;
	struct ftgmac100 *priv;

	netdev = platform_get_drvdata(pdev);
	priv = netdev_priv(netdev);

	unregister_netdev(netdev);

	phy_disconnect(priv->phydev);
	mdiobus_unregister(priv->mii_bus);
	mdiobus_free(priv->mii_bus);

	iounmap(priv->base);
	release_resource(priv->res);

	netif_napi_del(&priv->napi);
	platform_set_drvdata(pdev, NULL);
	free_netdev(netdev);
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
