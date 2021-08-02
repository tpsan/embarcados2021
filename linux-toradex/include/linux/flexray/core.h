/*
 * Copyright 2012 Eberspächer Electronics GmbH & Co. KG. All Rights Reserved.
 */

#ifndef FLEXRAY_CORE_H
#define FLEXRAY_CORE_H

#include <linux/flexray.h>
#include <linux/skbuff.h>
#include <linux/netdevice.h>

#define DNAME(dev) ((dev) ? (dev)->name : "any")

/**
 * struct flexray_proto - FlexRay protocol structure
 * @type:       type argument in socket() syscall, e.g. SOCK_DGRAM.
 * @protocol:   protocol number in socket() syscall.
 * @ops:        pointer to struct proto_ops for sock->ops.
 * @prot:       pointer to struct proto structure.
 */
struct flexray_proto {
	int type;
	int protocol;
	const struct proto_ops *ops;
	struct proto *prot;
};

/* function prototypes for the FlexRay networklayer core (af_flexray.c) */

extern int  flexray_proto_register(const struct flexray_proto *frp);
extern void flexray_proto_unregister(const struct flexray_proto *frp);

extern int  flexray_rx_register(struct net_device *dev, flexray_frame_filter_t frame_id,
				flexray_frame_filter_t mask,
				void (*func)(struct sk_buff *, void *),
				void *data, char *ident);

extern void flexray_rx_unregister(struct net_device *dev, flexray_frame_filter_t frame_id,
				flexray_frame_filter_t mask,
				void (*func)(struct sk_buff *, void *),
				void *data);

extern int flexray_send(struct sk_buff *skb, int loop);
extern int flexray_ioctl(struct socket *sock, unsigned int cmd, unsigned long arg);

#endif /* FLEXRAY_CORE_H */
