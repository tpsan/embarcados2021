/* Copyright 2012 Eberspächer Electronics GmbH & Co. KG. All Rights Reserved */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/uaccess.h>
#include <linux/net.h>
#include <linux/netdevice.h>
#include <linux/socket.h>
#include <linux/if_ether.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <linux/flexray.h>
#include <linux/flexray/core.h>
#include <linux/flexray/dev.h>
#include <linux/ratelimit.h>
#include <net/net_namespace.h>
#include <net/sock.h>

#include "af_flexray.h"

/* receive filters subscribed for 'all' FlexRay devices */
struct dev_rcv_lists flexray_rx_alldev_list;
static DEFINE_SPINLOCK(flexray_rcvlists_lock);

static struct kmem_cache *rcv_cache __read_mostly;

/* table of registered FlexRay protocols */
static const struct flexray_proto *proto_tab[FLEXRAY_NPROTO] __read_mostly;
static DEFINE_MUTEX(proto_tab_lock);

struct s_stats	flexray_stats;	/* packet statistics */
struct s_pstats	flexray_pstats;	/* receive list statistics */

/* af_flexray socket functions */

int flexray_ioctl(struct socket *sock, unsigned int cmd, unsigned long arg)
{
	struct sock *sk = sock->sk;

	switch (cmd) {

	case SIOCGSTAMP:
		return sock_get_timestamp(sk, (struct timeval __user *)arg);

	default:
		return -ENOIOCTLCMD;
	}
}
EXPORT_SYMBOL(flexray_ioctl);

static void flexray_sock_destruct(struct sock *sk)
{
	skb_queue_purge(&sk->sk_receive_queue);
}

static const struct flexray_proto *flexray_get_proto(int protocol)
{
	const struct flexray_proto *frp;

	rcu_read_lock();
	frp = rcu_dereference(proto_tab[protocol]);
	if (frp && !try_module_get(frp->prot->owner))
		frp = NULL;
	rcu_read_unlock();

	return frp;
}

static inline void flexray_put_proto(const struct flexray_proto *frp)
{
	module_put(frp->prot->owner);
}

static int flexray_create(struct net *net, struct socket *sock, int protocol,
			  int kern)
{
	struct sock *sk;
	const struct flexray_proto *frp;
	int err = 0;

	sock->state = SS_UNCONNECTED;

	if (protocol < 0 || protocol >= FLEXRAY_NPROTO)
		return -EINVAL;

	if (!net_eq(net, &init_net))
		return -EAFNOSUPPORT;

	frp = flexray_get_proto(protocol);

#ifdef CONFIG_MODULES
	if (!frp) {
		request_module("flexray-proto-%d", protocol);
		frp = flexray_get_proto(protocol);
	}
#endif
	/* check for available protocol and correct usage */
	if (!frp)
		return -EPROTONOSUPPORT;

	if (frp->type != sock->type) {
		err = -EPROTOTYPE;
		goto errout;
	}

	sock->ops = frp->ops;

	sk = sk_alloc(net, PF_FLEXRAY, GFP_KERNEL, frp->prot);
	if (!sk) {
		err = -ENOMEM;
		goto errout;
	}

	sock_init_data(sock, sk);
	sk->sk_destruct = flexray_sock_destruct;

	if (sk->sk_prot->init)
		err = sk->sk_prot->init(sk);

	if (err) {
		/* release sk on errors */
		sock_orphan(sk);
		sock_put(sk);
	}

errout:
	flexray_put_proto(frp);
	return err;
}

/**
 * flexray_send - transmit a FlexRay frame
 * @skb: pointer to socket buffer with FlexRay frame in data section
 * @loop: loopback for listeners on local CAN sockets (recommended default!)
 *
 * Due to the loopback this routine must not be called from hardirq context.
 *
 * Return:
 *  0 on success
 *  -ENETDOWN when the selected interface is down
 *  -ENOBUFS on full driver queue (see net_xmit_errno())
 *  -EPERM when trying to send on a non-FlexRay interface
 *  -EINVAL when the skb->data does not contain a valid FlexRay frame
 *  The skb is always consumed.
 */
int flexray_send(struct sk_buff *skb, int loop)
{
   struct sk_buff *newskb = NULL;
	struct flexray_priv *priv;
	int err;

	priv = netdev_priv(skb->dev);

	if (skb->dev->type != ARPHRD_FLEXRAY) {
		err = -EPERM;
      kfree_skb(skb);
      return err;
	}

	if (!(skb->dev->flags & IFF_UP)) {
		err = -ENETDOWN;
      kfree_skb(skb);
      return err;
	}

#if 0
   // do_validate is used for hardware flexcard_fr.c , but skip this for vflexray-usage.
	if (!priv->do_validate) {
		err = -EINVAL;
      kfree_skb(skb);
      return err;
	}

	err = priv->do_validate(skb);
	if (err) {
      kfree_skb(skb);
      return err;
   }
#endif
   
	skb->protocol = htons(ETH_P_FLEXRAY);
	skb_reset_network_header(skb);
	skb_reset_transport_header(skb);

	if (loop) {
		/* local loopback of sent CAN frames */

		/* indication for the CAN driver: do loopback */
		skb->pkt_type = PACKET_LOOPBACK;

		/*
		 * The reference to the originating sock may be required
		 * by the receiving socket to check whether the frame is
		 * its own. Example: can_raw sockopt CAN_RAW_RECV_OWN_MSGS
		 * Therefore we have to ensure that skb->sk remains the
		 * reference to the originating sock by restoring skb->sk
		 * after each skb_clone() or skb_orphan() usage.
		 */

		if (!(skb->dev->flags & IFF_ECHO)) {
			/*
			 * If the interface is not capable to do loopback
			 * itself, we do it here.
			 */
			newskb = skb_clone(skb, GFP_ATOMIC);
			if (!newskb) {
				kfree_skb(skb);
				return -ENOMEM;
			}

			newskb->sk = skb->sk;
			newskb->ip_summed = CHECKSUM_UNNECESSARY;
			newskb->pkt_type = PACKET_BROADCAST;
		}
	} else {
		/* indication for the CAN driver: no loopback required */
		skb->pkt_type = PACKET_HOST;
	}

   /* send to netdevice */
   err = dev_queue_xmit(skb);
   if (err > 0)
      err = net_xmit_errno(err);

   if (err) {
      kfree_skb(newskb);
      return err;
   }

   if (newskb)
      netif_rx_ni(newskb);

   /* update statistics */
	flexray_stats.tx_frames++;

   return 0;
}
EXPORT_SYMBOL(flexray_send);

/* af_flexray rx path */

static struct dev_rcv_lists *find_dev_rcv_lists(struct net_device *dev)
{
	if (!dev)
		return &flexray_rx_alldev_list;
	else
		return (struct dev_rcv_lists *)dev->ml_priv;
}

int flexray_rx_register(struct net_device *dev, flexray_frame_filter_t frame_id,
			flexray_frame_filter_t mask,
			void (*func)(struct sk_buff *, void *),
			void *data, char *ident)
{
	struct receiver *r;
	struct dev_rcv_lists *d;
	int err = 0;

	if (dev && dev->type != ARPHRD_FLEXRAY)
		return -ENODEV;

	r = kmem_cache_alloc(rcv_cache, GFP_KERNEL);
	if (!r)
		return -ENOMEM;

	spin_lock(&flexray_rcvlists_lock);

	d = find_dev_rcv_lists(dev);
	if (d) {
		r->func		= func;
		r->data		= data;
		r->ident	= ident;


		hlist_add_head_rcu(&r->list, &d->rx[RX_ALL]);
		d->entries++;

		flexray_pstats.rcv_entries++;
		if (flexray_pstats.rcv_entries_max < flexray_pstats.rcv_entries)
			flexray_pstats.rcv_entries_max =
				flexray_pstats.rcv_entries;
	} else {
		kmem_cache_free(rcv_cache, r);
		err = -ENODEV;
	}

	spin_unlock(&flexray_rcvlists_lock);

	return err;
}
EXPORT_SYMBOL(flexray_rx_register);

static void flexray_rx_delete_receiver(struct rcu_head *rp)
{
	struct receiver *r = container_of(rp, struct receiver, rcu);

	kmem_cache_free(rcv_cache, r);
}

void flexray_rx_unregister(struct net_device *dev, flexray_frame_filter_t frame_id,
			flexray_frame_filter_t mask,
			void (*func)(struct sk_buff *, void *), void *data)
{
	struct receiver *r = NULL;
	struct hlist_head *rl;
	struct hlist_node *next;
	struct dev_rcv_lists *d;

	if (dev && dev->type != ARPHRD_FLEXRAY)
		return;

	spin_lock(&flexray_rcvlists_lock);

	d = find_dev_rcv_lists(dev);
	if (!d) {
		netdev_err(dev, "BUG: receive list not found\n");
		goto out;
	}

	rl = &d->rx[RX_ALL];

	/* Search the receiver list for the item to delete. This should
	 * exist, since no receiver may be unregistered that hasn't
	 * been registered before.
	 */

	hlist_for_each_entry_rcu(r, next, rl, list) {
		if (r->func == func && r->data == data) {
			break;
		}
	}

	/*
	 * Check for bugs in FLEXRAY protocol implementations:
	 * If no matching list item was found, the list cursor variable next
	 * will be NULL, while r will point to the last item of the list.
	 */

	if (!next) {
		netdev_err(dev, "BUG: receive list entry not found\n");
		r = NULL;
		goto out;
	}

	hlist_del_rcu(&r->list);
	d->entries--;

	if (flexray_pstats.rcv_entries > 0)
		flexray_pstats.rcv_entries--;

	/* remove device structure requested by NETDEV_UNREGISTER */
	if (d->remove_on_zero_entries && !d->entries) {
		kfree(d);
		dev->ml_priv = NULL;
	}

out:
	spin_unlock(&flexray_rcvlists_lock);

	/* schedule the receiver item for deletion */
	if (r)
		call_rcu(&r->rcu, flexray_rx_delete_receiver);
}
EXPORT_SYMBOL(flexray_rx_unregister);

static inline void deliver(struct sk_buff *skb, struct receiver *r)
{
	r->func(skb, r->data);
}

static int flexray_rcv_filter(struct dev_rcv_lists *d, struct sk_buff *skb)
{
	struct receiver *r;
	struct hlist_node *next;
	int matches = 0;

	if (d->entries == 0)
		return 0;

	/* check for unfiltered entries */
	hlist_for_each_entry_rcu(r, next, &d->rx[RX_ALL], list) {
		deliver(skb, r);
		matches++;
	}

	return matches;
}

static int flexray_rcv(struct sk_buff *skb, struct net_device *dev,
		       struct packet_type *pt, struct net_device *orig_dev)
{
	struct dev_rcv_lists *d;
	struct flexray_frame *frf = (struct flexray_frame *)skb->data;
	int matches;

	if (!net_eq(dev_net(dev), &init_net)) {
		goto drop;
   }
   
	if (WARN_ONCE(dev->type != ARPHRD_FLEXRAY,
		      "PF_FLEXRAY: dropped non conform skbuf: "
		      "dev type %d, len %d, payload %d\n", dev->type, skb->len, 
		      frf->frhead.plr))
		goto drop;

	/* update statistics */
	flexray_stats.rx_frames++;
   
	rcu_read_lock();

	/* deliver the packet to sockets listening on all devices */
	matches = flexray_rcv_filter(&flexray_rx_alldev_list, skb);

	/* find receive list for this device */
	d = find_dev_rcv_lists(dev);
	if (d) {
		matches += flexray_rcv_filter(d, skb);
   }

	rcu_read_unlock();

	/* consume the skbuff allocated by the netdevice driver */
	consume_skb(skb);

	if (matches > 0) {
		flexray_stats.matches++;
   }
	return NET_RX_SUCCESS;

drop:
	kfree_skb(skb);
	return NET_RX_DROP;
}

/* af_flexray protocol functions */
int flexray_proto_register(const struct flexray_proto *frp)
{
	int proto = frp->protocol;
	int err = 0;

	if (proto < 0 || proto >= FLEXRAY_NPROTO) {
		pr_err("flexray: protocol number %d out of range\n", proto);
		return -EINVAL;
	}

	err = proto_register(frp->prot, 0);
	if (err < 0)
		return err;

	mutex_lock(&proto_tab_lock);

	if (proto_tab[proto]) {
		pr_err("flexray: protocol %d already registered\n",
		       proto);
		err = -EBUSY;
	} else
		rcu_assign_pointer(proto_tab[proto], frp);

	mutex_unlock(&proto_tab_lock);

	if (err < 0)
		proto_unregister(frp->prot);

	return err;
}
EXPORT_SYMBOL(flexray_proto_register);

void flexray_proto_unregister(const struct flexray_proto *frp)
{
	int proto = frp->protocol;

	mutex_lock(&proto_tab_lock);
	BUG_ON(proto_tab[proto] != frp);
	rcu_assign_pointer(proto_tab[proto], NULL);
	mutex_unlock(&proto_tab_lock);

	synchronize_rcu();

	proto_unregister(frp->prot);
}
EXPORT_SYMBOL(flexray_proto_unregister);

/* af_flexray notifier to create/remove FlexRay netdevice specific structs */
static int flexray_notifier(struct notifier_block *nb, unsigned long msg,
			    void *data)
{
	struct net_device *dev = (struct net_device *)data;
	struct dev_rcv_lists *d;

	if (!net_eq(dev_net(dev), &init_net))
		return NOTIFY_DONE;

	if (dev->type != ARPHRD_FLEXRAY)
		return NOTIFY_DONE;

	switch (msg) {

	case NETDEV_REGISTER:
		/* create new dev_rcv_lists for this device */
		d = kzalloc(sizeof(*d), GFP_KERNEL);
		if (!d) {
			netdev_err(dev, "allocation of receive list failed\n");
			return NOTIFY_DONE;
		}
		BUG_ON(dev->ml_priv);
		dev->ml_priv = d;

		break;

	case NETDEV_UNREGISTER:
		spin_lock(&flexray_rcvlists_lock);

		d = dev->ml_priv;
		if (d) {
			if (d->entries)
				d->remove_on_zero_entries = 1;
			else {
				kfree(d);
				dev->ml_priv = NULL;
			}
		} else
			netdev_err(dev, "notifier: receive list not found\n");

		spin_unlock(&flexray_rcvlists_lock);
		break;
	}

	return NOTIFY_DONE;
}

/* af_flexray module init/exit functions */

static struct packet_type flexray_packet __read_mostly = {
	.type = cpu_to_be16(ETH_P_FLEXRAY),
   .dev  = NULL,
	.func = flexray_rcv,
};

static const struct net_proto_family flexray_family_ops = {
	.family = PF_FLEXRAY,
	.create = flexray_create,
	.owner  = THIS_MODULE,
};

/* notifier block for netdevice event */
static struct notifier_block flexray_netdev_notifier __read_mostly = {
	.notifier_call = flexray_notifier,
};

static __init int flexray_init(void)
{
	memset(&flexray_rx_alldev_list, 0, sizeof(flexray_rx_alldev_list));

	rcv_cache = kmem_cache_create("flexray_receiver",
				      sizeof(struct receiver), 0, 0, NULL);
	if (!rcv_cache)
		return -ENOMEM;

	flexray_init_proc();

	/* protocol register */
	sock_register(&flexray_family_ops);
	register_netdevice_notifier(&flexray_netdev_notifier);
	dev_add_pack(&flexray_packet);
	return 0;
}
module_init(flexray_init);

static __exit void flexray_exit(void)
{
	struct net_device *dev;

	flexray_remove_proc();

	/* protocol unregister */
	dev_remove_pack(&flexray_packet);
	unregister_netdevice_notifier(&flexray_netdev_notifier);
	sock_unregister(PF_FLEXRAY);

	/* remove created dev_rcv_lists from still registered FlexRay devices */
	rcu_read_lock();
	for_each_netdev_rcu(&init_net, dev) {
		if (dev->type == ARPHRD_FLEXRAY && dev->ml_priv) {
			struct dev_rcv_lists *d = dev->ml_priv;

			BUG_ON(d->entries);
			kfree(d);
			dev->ml_priv = NULL;
		}
	}
	rcu_read_unlock();

	rcu_barrier(); /* Wait for completion of call_rcu()'s */

	kmem_cache_destroy(rcv_cache);
}
module_exit(flexray_exit);

MODULE_DESCRIPTION("FlexRay PF_FLEXRAY core");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Benedikt Spranger <b.spranger@linutronix.de>");

MODULE_ALIAS_NETPROTO(PF_FLEXRAY);
