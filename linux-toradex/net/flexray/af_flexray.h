/* Copyright 2012 Eberspächer Electronics GmbH & Co. KG. All Rights Reserved */

#ifndef AF_FLEXRAY_H
#define AF_FLEXRAY_H

#include <linux/skbuff.h>
#include <linux/netdevice.h>
#include <linux/list.h>
#include <linux/rcupdate.h>
#include <linux/flexray.h>

/* af_flexray rx dispatcher structures */

struct receiver {
	struct hlist_node list;
	struct rcu_head rcu;
	void (*func)(struct sk_buff *, void *);
	void *data;
	char *ident;
};

enum {RX_ALL, RX_MAX};

/* per device receive filters linked at dev->ml_priv */
struct dev_rcv_lists {
	struct hlist_head rx[RX_MAX];
	struct hlist_head rx_sff[0x800];
	int remove_on_zero_entries;
	int entries;
};

/* can be reset e.g. by flexray_init_stats() */
struct s_stats {
	unsigned long rx_frames;
	unsigned long tx_frames;
	unsigned long matches;
};

/* persistent statistics */
struct s_pstats {
	unsigned long stats_reset;
	unsigned long user_reset;
	unsigned long rcv_entries;
	unsigned long rcv_entries_max;
};

/* function prototypes for the FlexRay networklayer procfs (proc.c) */
extern void flexray_init_proc(void);
extern void flexray_remove_proc(void);
extern void flexray_stat_update(unsigned long data);

/* structures and variables from af_flexray.c needed in proc.c for reading */
extern struct s_stats	flexray_stats;		/* packet statistics */
extern struct s_pstats	flexray_pstats;		/* receive list statistics */
extern struct hlist_head flexray_rx_dev_list;	/* rx dispatcher structures */
/* receive filters subscribed for 'all' FlexRay devices */
extern struct dev_rcv_lists flexray_rx_alldev_list;

#endif /* AF_FLEXRAY_H */
