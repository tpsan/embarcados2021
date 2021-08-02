/*
 * Copyright 2012 Eberspächer Electronics GmbH & Co. KG. All Rights Reserved.
 */

#ifndef FLEXRAY_DEV_H
#define FLEXRAY_DEV_H

#include <linux/flexray.h>
#include <linux/flexray/netlink.h>

/* FlexRay common private data */

struct flexray_priv {
	struct flexray_device_stats flexray_stats;

	enum flexray_state state;

	int (*do_get_state)(const struct net_device *dev,
			    enum flexray_state *state);
	int (*do_set_state)(struct net_device *dev,
			    enum flexray_state state);
	int (*do_validate)(struct sk_buff *skb);

	struct flexray_cluster_param cluster;
	struct flexray_node_param node;
	struct flexray_symbol_param symbol;
	__u32 sw_filter[FLEXRAY_MAX_SW_FILTER];
	u8 version;
};

/* Drop a given socketbuffer if it does not contain a valid FlexRay frame. */
static inline int flexray_dropped_invalid_skb(struct net_device *dev,
					      struct sk_buff *skb)
{
	const struct flexray_frame *frf = (struct flexray_frame *)skb->data;

	if (unlikely(skb->len != sizeof(*frf))) {
		kfree_skb(skb);
		dev->stats.tx_dropped++;
		return 1;
	}

	return 0;
}

struct net_device *alloc_flexraydev(int sizeof_priv, u8 version);
void free_flexraydev(struct net_device *dev);

int open_flexraydev(struct net_device *dev);
void close_flexraydev(struct net_device *dev);

int register_flexraydev(struct net_device *dev);
void unregister_flexraydev(struct net_device *dev);

int flexray_restart_now(struct net_device *dev);

struct sk_buff *alloc_flexray_skb(struct net_device *dev,
				  struct flexray_frame **cf);

void print_cluster_config(struct flexray_cluster_param *cp, int is_v3);
void print_node_config(struct flexray_node_param *np, int is_v3);
void print_symbol_config(struct flexray_symbol_param *sp);

#endif /* FLEXRAY_DEV_H */
