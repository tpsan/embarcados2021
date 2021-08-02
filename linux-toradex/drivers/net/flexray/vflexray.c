/* Copyright 2012 Eberspächer Electronics GmbH & Co. KG. All Rights Reserved */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/flexray.h>
#include <linux/flexray/dev.h>
#include <linux/slab.h>
#include <net/rtnetlink.h>

/*
 * FLEXRAY test feature:
 * Enable the echo on driver level for testing the FLEXRAY core echo modes.
 * See Documentation/networking/flexray.txt for details.
 */

static int echo; /* echo testing. Default: 0 (Off) */
module_param(echo, bool, S_IRUGO);
MODULE_PARM_DESC(echo, "Echo sent frames (for testing). Default: 0 (Off)");

static void vflexray_rx(struct sk_buff *skb, struct net_device *dev)
{
	struct flexray_frame *frf = (struct flexray_frame *)skb->data;
	struct net_device_stats *stats = &dev->stats;

	stats->rx_packets++;
	stats->rx_bytes += frf->frhead.plr;

	skb->protocol  = htons(ETH_P_FLEXRAY);
	skb->pkt_type  = PACKET_BROADCAST;
	skb->dev       = dev;
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	netif_rx_ni(skb);
}

static netdev_tx_t vflexray_tx(struct sk_buff *skb, struct net_device *dev)
{
	struct flexray_frame *frf = (struct flexray_frame *)skb->data;
	struct net_device_stats *stats = &dev->stats;
	int loop;

	if (flexray_dropped_invalid_skb(dev, skb))
		return NETDEV_TX_OK;

	stats->tx_packets++;
	stats->tx_bytes += frf->frhead.plr;
	/* set flag whether this packet has to be looped back */
	loop = skb->pkt_type == PACKET_LOOPBACK;

	if (!echo) {
		/* no echo handling available inside this driver */

		if (loop) {
			/*
			 * only count the packets here, because the
			 * FLEXRAY core already did the echo for us
			 */
			stats->rx_packets++;
			stats->rx_bytes += frf->frhead.plr;
		}
		kfree_skb(skb);
		return NETDEV_TX_OK;
	}

   /* perform standard echo handling for FLEXRAY network interfaces */

   if (loop) {
		struct sock *srcsk = skb->sk;

		skb = skb_share_check(skb, GFP_ATOMIC);
		if (!skb)
			return NETDEV_TX_OK;

		/* receive with packet counting */
		skb->sk = srcsk;
		vflexray_rx(skb, dev);
	} else {
		kfree_skb(skb);
	}

	return NETDEV_TX_OK;
}

static const struct net_device_ops vflexray_netdev_ops = {
	.ndo_start_xmit = vflexray_tx,
};

static void vflexray_setup(struct net_device *dev)
{
	dev->type		= ARPHRD_FLEXRAY;
	dev->mtu		= sizeof(struct flexray_frame);
	dev->hard_header_len	= 0;
	dev->addr_len		= 0;
	dev->tx_queue_len	= 0;
	dev->flags		= IFF_NOARP;

   /* set flags according to driver capabilities */
   if (echo) {
      dev->flags |= IFF_ECHO;
   }
   
   dev->netdev_ops		= &vflexray_netdev_ops;
	dev->destructor		= free_netdev;
}

static struct rtnl_link_ops vflexray_link_ops __read_mostly = {
	.kind   = "vflexray",
	.setup  = vflexray_setup,
};

static __init int vflexray_init_module(void)
{
	pr_info("vflexray: Virtual FlexRay interface driver\n");

   if (echo)
      printk(KERN_INFO "vflexray: enabled echo on driver level.\n");

   return rtnl_link_register(&vflexray_link_ops);
}

static __exit void vflexray_cleanup_module(void)
{
	rtnl_link_unregister(&vflexray_link_ops);
}

module_init(vflexray_init_module);
module_exit(vflexray_cleanup_module);

MODULE_DESCRIPTION("virtual FlexRay interface");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Benedikt Spranger <b.spranger@linutronix.de>");
