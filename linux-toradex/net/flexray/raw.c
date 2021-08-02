/* Copyright 2012 Eberspächer Electronics GmbH & Co. KG. All Rights Reserved */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/uio.h>
#include <linux/net.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/socket.h>
#include <linux/if_arp.h>
#include <linux/skbuff.h>
#include <linux/flexray.h>
#include <linux/flexray/core.h>
#include <linux/flexray/raw.h>
#include <net/sock.h>
#include <net/net_namespace.h>

#define MASK_ALL 0

struct flexray_raw_sock {
	struct sock sk;
	int bound;
	int ifindex;
	struct notifier_block notifier;
	int loopback;
   int recv_own_msgs;
   int count;                 /* number of active filters */
	struct flexray_filter dfilter;	/* default/single filter */
   struct flexray_filter *filter; /* pointer to filter(s) */
   
};

static inline unsigned int *raw_flags(struct sk_buff *skb)
{
	BUILD_BUG_ON(sizeof(skb->cb) <= (sizeof(struct sockaddr_flexray) +
					 sizeof(unsigned int)));

	/* return pointer after struct sockaddr_flexray */
	return (unsigned int *)(&((struct sockaddr_flexray *)skb->cb)[1]);
}

static inline struct flexray_raw_sock *raw_sk(const struct sock *sk)
{
	return (struct flexray_raw_sock *)sk;
}

static void raw_rcv(struct sk_buff *oskb, void *data)
{
	struct sock *sk = (struct sock *)data;
	struct flexray_raw_sock *ro = raw_sk(sk);
	struct sockaddr_flexray *addr;
	struct sk_buff *skb;
	unsigned int *pflags;

	/* check the received tx sock reference */
	if (!ro->recv_own_msgs && oskb->sk == sk)
		return;

	/* clone the given skb to be able to enqueue it into the rcv queue */
	skb = skb_clone(oskb, GFP_ATOMIC);
	if (!skb)
		return;

	/*  Put the datagram to the queue so that raw_recvmsg() can
	 *  get it from there.  We need to pass the interface index to
	 *  raw_recvmsg().  We pass a whole struct sockaddr_flexray in skb->cb
	 *  containing the interface index.
	 */

	BUILD_BUG_ON(sizeof(skb->cb) < sizeof(struct sockaddr_flexray));
	addr = (struct sockaddr_flexray *)skb->cb;
	memset(addr, 0, sizeof(*addr));
	addr->flexray_family  = AF_FLEXRAY;
	addr->flexray_ifindex = skb->dev->ifindex;

	/* add FlexRay specific message flags for raw_recvmsg() */
	pflags = raw_flags(skb);
	*pflags = 0;
	if (oskb->sk)
		*pflags |= MSG_DONTROUTE;
	if (oskb->sk == sk)
		*pflags |= MSG_CONFIRM;

	if (sock_queue_rcv_skb(sk, skb) < 0)
		kfree_skb(skb);
}

static int raw_enable_filters(struct net_device *dev, struct sock *sk,
			      struct flexray_filter *filter, int count)
{
	int err = 0;
	int i;

	for (i = 0; i < count; i++) {
      //printk(KERN_INFO "raw_enable_filters: flexray_rx_register %d 0x%X 0x%X\n", i,  filter[i].flexray_id, filter[i].flexray_mask);
		err = flexray_rx_register(dev, filter[i].flexray_id,
				      filter[i].flexray_mask,
				      raw_rcv, sk, "raw");
		if (err) {
         //printk(KERN_INFO "raw_enable_filters: failed, remove again\n");
			/* clean up successfully registered filters */
			while (--i >= 0)
				flexray_rx_unregister(dev, filter[i].flexray_id,
						  filter[i].flexray_mask,
						  raw_rcv, sk);
			break;
		}
	}

	return err;
}

static void raw_disable_filters(struct net_device *dev, struct sock *sk,
			      struct flexray_filter *filter, int count)
{
	int i;

	for (i = 0; i < count; i++) {
		flexray_rx_unregister(dev, filter[i].flexray_id, filter[i].flexray_mask,
				  raw_rcv, sk);
   }
}

static int raw_enable_allfilters(struct net_device *dev, struct sock *sk)
{
	//return flexray_rx_register(dev, raw_rcv, sk, "raw");
   struct flexray_raw_sock *ro = raw_sk(sk);
   int err;

   err = raw_enable_filters(dev, sk, ro->filter, ro->count);
   if (err) {
      raw_disable_filters(dev, sk, ro->filter, ro->count);
   }
   //printk(KERN_INFO "raw_enable_allfilters: err=%d count=%d\n", err, ro->count);
   return err;
}

static inline void raw_disable_allfilters(struct net_device *dev,
					  struct sock *sk)
{
	//flexray_rx_unregister(dev, raw_rcv, sk);
   struct flexray_raw_sock *ro = raw_sk(sk);

   raw_disable_filters(dev, sk, ro->filter, ro->count);
}

static struct flexray_raw_sock *nb_to_raw_sock(struct notifier_block *nb)
{
	return container_of(nb, struct flexray_raw_sock, notifier);
}

static int raw_notifier(struct notifier_block *nb,
			unsigned long msg, void *data)
{
	struct net_device *dev = (struct net_device *)data;
	struct flexray_raw_sock *ro = nb_to_raw_sock(nb);
	struct sock *sk = &ro->sk;

	if (!net_eq(dev_net(dev), &init_net))
		return NOTIFY_DONE;

	if (dev->type != ARPHRD_FLEXRAY)
		return NOTIFY_DONE;

	if (ro->ifindex != dev->ifindex)
		return NOTIFY_DONE;

	switch (msg) {

	case NETDEV_UNREGISTER:
		lock_sock(sk);
		/* remove current filters & unregister */
		if (ro->bound)
			raw_disable_allfilters(dev, sk);

		ro->ifindex = 0;
		ro->bound   = 0;
		release_sock(sk);

		sk->sk_err = ENODEV;
		if (!sock_flag(sk, SOCK_DEAD))
			sk->sk_error_report(sk);
		break;

	case NETDEV_DOWN:
		sk->sk_err = ENETDOWN;
		if (!sock_flag(sk, SOCK_DEAD))
			sk->sk_error_report(sk);
		break;
	}

	return NOTIFY_DONE;
}

static int raw_init(struct sock *sk)
{
	struct flexray_raw_sock *ro = raw_sk(sk);

	ro->bound			= 0;
	ro->ifindex			= 0;

   /* set default filter to single entry dfilter */
   ro->dfilter.flexray_id   = 0;
   ro->dfilter.flexray_mask = MASK_ALL;
   ro->filter           = &ro->dfilter;
   ro->count            = 1;
   
	/* set default loopback behaviour */
	ro->loopback         = 1;
	ro->recv_own_msgs    = 0;

	/* set notifier */
	ro->notifier.notifier_call	= raw_notifier;

	register_netdevice_notifier(&ro->notifier);

	return 0;
}

static int raw_release(struct socket *sock)
{
	struct sock *sk = sock->sk;
	struct flexray_raw_sock *ro;

	if (!sk)
		return 0;

	ro = raw_sk(sk);

	unregister_netdevice_notifier(&ro->notifier);

	lock_sock(sk);

	/* remove current filters & unregister */
	if (ro->bound) {
		if (ro->ifindex) {
			struct net_device *dev;

			dev = dev_get_by_index(&init_net, ro->ifindex);
			if (dev) {
				raw_disable_allfilters(dev, sk);
				dev_put(dev);
			}
		} else
			raw_disable_allfilters(NULL, sk);
	}

	ro->ifindex = 0;
	ro->bound   = 0;

	sock_orphan(sk);
	sock->sk = NULL;

	release_sock(sk);
	sock_put(sk);

	return 0;
}

static int raw_bind(struct socket *sock, struct sockaddr *uaddr, int len)
{
	struct sockaddr_flexray *addr = (struct sockaddr_flexray *)uaddr;
	struct sock *sk = sock->sk;
	struct flexray_raw_sock *ro = raw_sk(sk);
	int ifindex;
	int err = 0;
	int notify_enetdown = 0;

	if (len < sizeof(*addr))
		return -EINVAL;

	lock_sock(sk);

	if (ro->bound && addr->flexray_ifindex == ro->ifindex) {
		goto out;
   }
   
	if (addr->flexray_ifindex) {
		struct net_device *dev;

		dev = dev_get_by_index(&init_net, addr->flexray_ifindex);
		if (!dev) {
			err = -ENODEV;
			goto out;
		}

		if (dev->type != ARPHRD_FLEXRAY) {
			dev_put(dev);
			err = -ENODEV;
			goto out;
		}
		if (!(dev->flags & IFF_UP))
			notify_enetdown = 1;

		ifindex = dev->ifindex;

		/* filters set by default/setsockopt */
		err = raw_enable_allfilters(dev, sk);
		dev_put(dev);
	} else {
		ifindex = 0;

		/* filters set by default/setsockopt */
		err = raw_enable_allfilters(NULL, sk);
	}

	if (!err) {
		if (ro->bound) {
			/* unregister old filters */
			if (ro->ifindex) {
				struct net_device *dev;

				dev = dev_get_by_index(&init_net, ro->ifindex);
				if (dev) {
					raw_disable_allfilters(dev, sk);
					dev_put(dev);
				}
			} else
				raw_disable_allfilters(NULL, sk);
		}
		ro->ifindex = ifindex;
		ro->bound = 1;
	}

out:
	release_sock(sk);

	if (notify_enetdown) {
		sk->sk_err = ENETDOWN;
		if (!sock_flag(sk, SOCK_DEAD))
			sk->sk_error_report(sk);
	}

	return err;
}

static int raw_getname(struct socket *sock, struct sockaddr *uaddr,
		       int *len, int peer)
{
	struct sockaddr_flexray *addr = (struct sockaddr_flexray *)uaddr;
	struct sock *sk = sock->sk;
	struct flexray_raw_sock *ro = raw_sk(sk);

	if (peer)
		return -EOPNOTSUPP;

	memset(addr, 0, sizeof(*addr));
	addr->flexray_family  = AF_FLEXRAY;
	addr->flexray_ifindex = ro->ifindex;

	*len = sizeof(*addr);

	return 0;
}

static int raw_setsockopt(struct socket *sock, int level, int optname,
			  char __user *optval, unsigned int optlen)
{
	struct sock *sk = sock->sk;
	struct flexray_raw_sock *ro = raw_sk(sk);
	struct flexray_filter *filter = NULL;  /* dyn. alloc'ed filters */
	struct flexray_filter sfilter;         /* single filter */
	struct net_device *dev = NULL;
	//flexray_err_mask_t err_mask = 0;
	int count = 0;
	int err = 0;

	if (level != SOL_FLEXRAY_RAW)
		return -EINVAL;

	switch (optname) {

	case FLEXRAY_RAW_FILTER:
		if (optlen % sizeof(struct flexray_filter) != 0)
			return -EINVAL;

		count = optlen / sizeof(struct flexray_filter);
		if (count > 1) {
			/* filter does not fit into dfilter => alloc space */
			filter = memdup_user(optval, optlen);
			if (IS_ERR(filter))
				return PTR_ERR(filter);
		} else if (count == 1) {
			if (copy_from_user(&sfilter, optval, sizeof(sfilter)))
				return -EFAULT;
		}

		lock_sock(sk);

		if (ro->bound && ro->ifindex)
			dev = dev_get_by_index(&init_net, ro->ifindex);

		if (ro->bound) {
			/* (try to) register the new filters */
			if (count == 1)
				err = raw_enable_filters(dev, sk, &sfilter, 1);
			else
				err = raw_enable_filters(dev, sk, filter,
							 count);
			if (err) {
				if (count > 1)
					kfree(filter);
				goto out_fil;
			}

			/* remove old filter registrations */
			raw_disable_filters(dev, sk, ro->filter, ro->count);
		}

		/* remove old filter space */
		if (ro->count > 1)
			kfree(ro->filter);

		/* link new filters to the socket */
		if (count == 1) {
			/* copy filter data for single filter */
			ro->dfilter = sfilter;
			filter = &ro->dfilter;
		}
		ro->filter = filter;
		ro->count  = count;

 out_fil:
		if (dev)
			dev_put(dev);

		release_sock(sk);

		break;

   case FLEXRAY_RAW_LOOPBACK:
		if (optlen != sizeof(ro->loopback))
			return -EINVAL;

		if (copy_from_user(&ro->loopback, optval, optlen))
			return -EFAULT;

		break;

   case FLEXRAY_RAW_RECV_OWN_MSGS:
      if (optlen != sizeof(ro->recv_own_msgs))
         return -EINVAL;
      
      if (copy_from_user(&ro->recv_own_msgs, optval, optlen))
         return -EFAULT;
      
      break;
      
   default:
		return -ENOPROTOOPT;
	}
	return err;
}


static int raw_getsockopt(struct socket *sock, int level, int optname,
			  char __user *optval, int __user *optlen)
{
	struct sock *sk = sock->sk;
	struct flexray_raw_sock *ro = raw_sk(sk);
	int len;
	void *val;
   int err;
   
	if (level != SOL_FLEXRAY_RAW)
		return -EINVAL;
	if (get_user(len, optlen))
		return -EFAULT;
	if (len < 0)
		return -EINVAL;

	switch (optname) {

	case FLEXRAY_RAW_FILTER:
		lock_sock(sk);
		if (ro->count > 0) {
			int fsize = ro->count * sizeof(struct flexray_filter);
			if (len > fsize)
				len = fsize;
			if (copy_to_user(optval, ro->filter, len))
				err = -EFAULT;
		} else
			len = 0;
		release_sock(sk);

		if (!err)
			err = put_user(len, optlen);
		return err;

	case FLEXRAY_RAW_LOOPBACK:
		if (len > sizeof(int))
			len = sizeof(int);
		val = &ro->loopback;
		break;

	case FLEXRAY_RAW_RECV_OWN_MSGS:
		if (len > sizeof(int))
			len = sizeof(int);
		val = &ro->recv_own_msgs;
		break;

	default:
		return -ENOPROTOOPT;
	}

	if (put_user(len, optlen))
		return -EFAULT;
	if (copy_to_user(optval, val, len))
		return -EFAULT;
	return 0;
}


static int raw_sendmsg(struct kiocb *iocb, struct socket *sock,
		       struct msghdr *msg, size_t size)
{
	struct sock *sk = sock->sk;
	struct flexray_raw_sock *ro = raw_sk(sk);
	struct sk_buff *skb;
	struct net_device *dev;
	int ifindex;
	int err;

	if (msg->msg_name) {
		struct sockaddr_flexray *addr =
			(struct sockaddr_flexray *)msg->msg_name;

		if (msg->msg_namelen < sizeof(*addr))
			return -EINVAL;

		if (addr->flexray_family != AF_FLEXRAY)
			return -EINVAL;

		ifindex = addr->flexray_ifindex;
	} else {
		ifindex = ro->ifindex;
   }
   
	dev = dev_get_by_index(&init_net, ifindex);
	if (!dev)
		return -ENXIO;

	skb = sock_alloc_send_skb(sk, size, msg->msg_flags & MSG_DONTWAIT,
				  &err);
	if (!skb)
		goto put_dev;

	err = memcpy_fromiovec(skb_put(skb, size), msg->msg_iov, size);
	if (err < 0)
		goto free_skb;
   err = sock_tx_timestamp(sk, &skb_shinfo(skb)->tx_flags);
   if (err < 0)
      goto free_skb;
   
   /* to be able to check the received tx sock reference in raw_rcv() */
   skb_shinfo(skb)->tx_flags |= SKBTX_DRV_NEEDS_SK_REF;

   skb->dev = dev;
	skb->sk  = sk;

   err = flexray_send(skb, ro->loopback);

   dev_put(dev);

   if (err)
      goto send_failed;

   return size;

free_skb:
   kfree_skb(skb);
put_dev:
   dev_put(dev);
send_failed:
   return err;
}

static int raw_recvmsg(struct kiocb *iocb, struct socket *sock,
		       struct msghdr *msg, size_t size, int flags)
{
	struct sock *sk = sock->sk;
	struct sk_buff *skb;
	int err = 0;
	int noblock;

	noblock =  flags & MSG_DONTWAIT;
	flags   &= ~MSG_DONTWAIT;

	skb = skb_recv_datagram(sk, flags, noblock, &err);

	if (!skb)
		return err;

	if (size < skb->len)
		msg->msg_flags |= MSG_TRUNC;
	else
		size = skb->len;

	err = memcpy_toiovec(msg->msg_iov, skb->data, size);
	if (err < 0) {
		skb_free_datagram(sk, skb);
		return err;
	}

	sock_recv_ts_and_drops(msg, sk, skb);

	if (msg->msg_name) {
		msg->msg_namelen = sizeof(struct sockaddr_flexray);
		memcpy(msg->msg_name, skb->cb, msg->msg_namelen);
	}

	/* assign the flags that have been recorded in raw_rcv() */
	msg->msg_flags |= *(raw_flags(skb));

	skb_free_datagram(sk, skb);

	return size;
}

static const struct proto_ops raw_ops = {
	.family		= PF_FLEXRAY,
	.release	= raw_release,
	.bind		= raw_bind,
	.connect	= sock_no_connect,
	.socketpair	= sock_no_socketpair,
	.accept		= sock_no_accept,
	.getname	= raw_getname,
	.poll		= datagram_poll,
	.ioctl		= flexray_ioctl,
	.listen		= sock_no_listen,
	.shutdown	= sock_no_shutdown,
	.setsockopt    = raw_setsockopt,
	.getsockopt    = raw_getsockopt,
	.sendmsg	= raw_sendmsg,
	.recvmsg	= raw_recvmsg,
	.mmap		= sock_no_mmap,
	.sendpage	= sock_no_sendpage,
};

static struct proto raw_proto __read_mostly = {
	.name		= "FLEXRAY_RAW",
	.owner		= THIS_MODULE,
	.obj_size	= sizeof(struct flexray_raw_sock),
	.init		= raw_init,
};

static const struct flexray_proto raw_flexray_proto = {
	.type		= SOCK_RAW,
	.protocol	= FLEXRAY_RAW,
	.ops		= &raw_ops,
	.prot		= &raw_proto,
};

static __init int raw_module_init(void)
{
	int err;

	err = flexray_proto_register(&raw_flexray_proto);
	if (err < 0)
		pr_err("flexray: registration of raw protocol failed\n");

	return err;
}
module_init(raw_module_init);

static __exit void raw_module_exit(void)
{
	flexray_proto_unregister(&raw_flexray_proto);
}
module_exit(raw_module_exit);

MODULE_DESCRIPTION("PF_FLEXRAY raw protocol");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Benedikt Spranger <b.spranger@linutronix.de>");
MODULE_ALIAS("flexray-proto-1");
