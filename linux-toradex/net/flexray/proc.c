/* Copyright 2012 Eberspächer Electronics GmbH & Co. KG. All Rights Reserved */

#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/list.h>
#include <linux/rcupdate.h>
#include <linux/if_arp.h>
#include <linux/flexray/core.h>

#include "af_flexray.h"

/* proc filenames for the PF_FLEXRAY core */

#define FLEXRAY_PROC_STATS       "stats"
#define FLEXRAY_PROC_RESET_STATS "reset_stats"
#define FLEXRAY_PROC_RCVLIST_ALL "rcvlist_all"
#define FLEXRAY_PROC_RCVLIST_FIL "rcvlist_fil"

static struct proc_dir_entry *flexray_dir;
static struct proc_dir_entry *pde_stats;
static struct proc_dir_entry *pde_reset_stats;
static struct proc_dir_entry *pde_rcvlist_all;

static const char rx_list_name[][8] = {
	[RX_ALL] = "rx_all",
};

static void flexray_print_rcvlist(struct seq_file *m,
				  struct hlist_head *rx_list,
				  struct net_device *dev)
{
	struct receiver *r;
	struct hlist_node *n;

   hlist_for_each_entry_rcu(r, n, rx_list, list)
		seq_printf(m, "   %-5s     %pK  %pK  %s\n",
				DNAME(dev), r->func, r->data, r->ident);
}

static void flexray_print_recv_banner(struct seq_file *m)
{
	seq_puts(m, "  device   flexray_id   flexray_mask  function"
		 "  userdata   matches  ident\n");
}

static int flexray_stats_proc_show(struct seq_file *m, void *v)
{
	seq_putc(m, '\n');
	seq_printf(m, " %8ld transmitted frames (TXF)\n",
		   flexray_stats.tx_frames);
	seq_printf(m, " %8ld received frames (RXF)\n",
		   flexray_stats.rx_frames);
	seq_printf(m, " %8ld matched frames (RXMF)\n",
		   flexray_stats.matches);

	seq_putc(m, '\n');

	seq_printf(m, " %8ld current receive list entries (CRCV)\n",
		   flexray_pstats.rcv_entries);
	seq_printf(m, " %8ld maximum receive list entries (MRCV)\n",
		   flexray_pstats.rcv_entries_max);

	if (flexray_pstats.stats_reset)
		seq_printf(m, "\n %8ld statistic resets (STR)\n",
			   flexray_pstats.stats_reset);

	if (flexray_pstats.user_reset)
		seq_printf(m, " %8ld user statistic resets (USTR)\n",
			   flexray_pstats.user_reset);

	seq_putc(m, '\n');
	return 0;
}

static int flexray_stats_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, flexray_stats_proc_show, NULL);
}

static const struct file_operations flexray_stats_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= flexray_stats_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int flexray_reset_stats_proc_show(struct seq_file *m, void *v)
{
	seq_printf(m, "Performed statistic reset #%ld.\n",
		   flexray_pstats.stats_reset);

	return 0;
}

static int flexray_reset_stats_proc_open(struct inode *inode,
					 struct file *file)
{
	return single_open(file, flexray_reset_stats_proc_show, NULL);
}

static const struct file_operations flexray_reset_stats_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= flexray_reset_stats_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void flexray_rcvlist_proc_show_one(struct seq_file *m, int idx,
						 struct net_device *dev,
						 struct dev_rcv_lists *d)
{
	if (!hlist_empty(&d->rx[idx])) {
		flexray_print_recv_banner(m);
		flexray_print_rcvlist(m, &d->rx[idx], dev);
	} else
		seq_printf(m, "  (%s: no entry)\n", DNAME(dev));
}

static int flexray_rcvlist_proc_show(struct seq_file *m, void *v)
{
	int idx = (int)(long)m->private;
	struct net_device *dev;
	struct dev_rcv_lists *d;

	seq_printf(m, "\nreceive list '%s':\n", rx_list_name[idx]);

	rcu_read_lock();

	/* receive list for 'all' FlexRay devices (dev == NULL) */
	d = &flexray_rx_alldev_list;
	flexray_rcvlist_proc_show_one(m, idx, NULL, d);

	/* receive list for registered FlexRay devices */
	for_each_netdev_rcu(&init_net, dev) {
		if (dev->type == ARPHRD_FLEXRAY && dev->ml_priv)
			flexray_rcvlist_proc_show_one(m, idx, dev,
						      dev->ml_priv);
	}

	rcu_read_unlock();

	seq_putc(m, '\n');
	return 0;
}

static int flexray_rcvlist_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, flexray_rcvlist_proc_show, NULL);
}

static const struct file_operations flexray_rcvlist_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= flexray_rcvlist_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void flexray_remove_proc_readentry(const char *name)
{
	remove_proc_entry(name, flexray_dir);
}

void flexray_init_proc(void)
{
	int perm = S_IRUGO | S_IWUSR;

	flexray_dir = proc_mkdir("flexray", init_net.proc_net);
	if (!flexray_dir)
		return;

	pde_stats = proc_create(FLEXRAY_PROC_STATS, perm, flexray_dir,
			&flexray_stats_proc_fops);
	pde_reset_stats = proc_create(FLEXRAY_PROC_RESET_STATS, perm,
			flexray_dir, &flexray_reset_stats_proc_fops);
	pde_rcvlist_all = proc_create_data(FLEXRAY_PROC_RCVLIST_ALL, perm,
			flexray_dir, &flexray_rcvlist_proc_fops,
			(void *)RX_ALL);
}

void flexray_remove_proc(void)
{
	if (!flexray_dir)
		return;
	if (pde_stats)
		flexray_remove_proc_readentry(FLEXRAY_PROC_STATS);

	if (pde_reset_stats)
		flexray_remove_proc_readentry(FLEXRAY_PROC_RESET_STATS);

	if (pde_rcvlist_all)
		flexray_remove_proc_readentry(FLEXRAY_PROC_RCVLIST_ALL);

	remove_proc_entry("flexray", init_net.proc_net);
}
