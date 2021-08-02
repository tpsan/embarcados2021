/* Copyright 2012 Eberspächer Electronics GmbH & Co. KG. All Rights Reserved */

#include <linux/netdevice.h>
#include <linux/flexray.h>
#include <linux/flexray/dev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/eray.h>
#include <linux/flexcard.h>
#include <linux/flexray/flexcard_netlink.h>
#include <net/netlink.h>
#include <net/genetlink.h>

#define FC_MSGBUF_DUMP_CFG

static int cc_change_state(struct eray_cc *cc, enum eray_cc_state state,
			   int retry);
static void cc_get_conf(struct net_device *dev);

struct fc_enum2name {
	int type;
	char *name;
};
#define FC_ENUM2NAME(x)		{x, #x}

static struct fc_enum2name flexray_state[] = {
	FC_ENUM2NAME(FLEXRAY_STATE_UNSPEC),
	FC_ENUM2NAME(FLEXRAY_STATE_DEFAULT_CONFIG),
	FC_ENUM2NAME(FLEXRAY_STATE_CONFIG),
	FC_ENUM2NAME(FLEXRAY_STATE_READY),
	FC_ENUM2NAME(FLEXRAY_STATE_WAKEUP),
	FC_ENUM2NAME(FLEXRAY_STATE_STARTUP),
	FC_ENUM2NAME(FLEXRAY_STATE_NORMAL_ACTIVE),
	FC_ENUM2NAME(FLEXRAY_STATE_NORMAL_PASSIVE),
	FC_ENUM2NAME(FLEXRAY_STATE_HALT),
	FC_ENUM2NAME(FLEXRAY_STATE_MONITOR_MODE),
	FC_ENUM2NAME(FLEXRAY_STATE_COLDSTART),
	FC_ENUM2NAME(FLEXRAY_STATE_MAX),
};

static inline char *fc_flexray_state_name(enum flexray_state cmd)
{
	if (cmd < 0)
		cmd = FLEXRAY_STATE_UNSPEC;
	if (cmd > FLEXRAY_STATE_MAX)
		cmd = FLEXRAY_STATE_UNSPEC;

	return flexray_state[cmd].name;
}

static struct fc_enum2name eray_state[] = {
	FC_ENUM2NAME(ERAY_CMD_INVALID),
	FC_ENUM2NAME(ERAY_CMD_CONFIG),
	FC_ENUM2NAME(ERAY_CMD_READY),
	FC_ENUM2NAME(ERAY_CMD_WAKEUP),
	FC_ENUM2NAME(ERAY_CMD_RUN),
	FC_ENUM2NAME(ERAY_CMD_ALL_SLOTS),
	FC_ENUM2NAME(ERAY_CMD_HALT),
	FC_ENUM2NAME(ERAY_CMD_FREEZE),
	FC_ENUM2NAME(ERAY_CMD_SEND_MTS),
	FC_ENUM2NAME(ERAY_CMD_ALLOW_COLDSTART),
	FC_ENUM2NAME(ERAY_CMD_RESET_STATUS_INDICATORS),
	FC_ENUM2NAME(ERAY_CMD_MONITOR_MODE),
	FC_ENUM2NAME(ERAY_CMD_CLEAR_RAMS),
};

static inline char *fc_eray_cmd_name(enum eray_cc_state cmd)
{
	if (cmd < 0)
		cmd = ERAY_CMD_INVALID;
	if (cmd > ERAY_CMD_CLEAR_RAMS)
		cmd = ERAY_CMD_INVALID;

	return eray_state[cmd].name;
}

struct fc_msg {
	u32 buf_id;
	u8 data[254];
} __packed;

struct flexcard_priv {
	struct flexray_priv flexray;
	struct net_device *dev;
	struct eray_cc *cc;
	int id;
	void __iomem *conf;
};

static int _fc_write_data(struct flexcard_priv *priv, unsigned int msgbuf_id,
			  unsigned char *payload, size_t byte_len);

static struct nla_policy fc_msgbuf_genl_policy[__FC_MSGBUF_ATTR_MAX] = {
	[FC_MSGBUF_ATTR_BUF_ID] = {
		.type = NLA_U8,
	},
	[FC_MSGBUF_ATTR_DEV_ID] = {
		.type = NLA_U32,
	},
	[FC_MSGBUF_ATTR_DEV_NAME] = {
		.type = NLA_NUL_STRING,
	},
	[FC_MSGBUF_ATTR_CFG] = {
		.type = NLA_BINARY,
		.len = sizeof(struct fc_msgbuf_cfg),
	},
};

static struct genl_family fc_msgbuf_genl_family = {
	.id = GENL_ID_GENERATE,
	.hdrsize = 0,
	.name = "FC_MSGBUF",
	.version = FC_MSGBUF_VERSION,
	.maxattr = FC_MSGBUF_ATTR_MAX,
};

#ifdef FC_MSGBUF_DUMP_CFG

#define ERAY_MSGBUF_PRINT_FLAGS(x)			\
	do {						\
		if (cfg->flags & x)			\
			pr_cont(#x " ");		\
	} while (0)

static void eray_dump_msg_cfg(struct eray_msgbuf_cfg *cfg, int buf,
		const char *func)
{
	pr_debug("%s: msg. buffer dump %03d\n", func, buf);
	pr_debug("eray_msgbuf_cfg: cfg = %p\n", cfg);
	if (!cfg)
		return;
	pr_debug("flags   : ");
	ERAY_MSGBUF_PRINT_FLAGS(ERAY_MSGBUF_USED);
	ERAY_MSGBUF_PRINT_FLAGS(ERAY_MSGBUF_STARTUP);
	ERAY_MSGBUF_PRINT_FLAGS(ERAY_MSGBUF_SYNC);
	ERAY_MSGBUF_PRINT_FLAGS(ERAY_MSGBUF_PPIT);
	ERAY_MSGBUF_PRINT_FLAGS(ERAY_MSGBUF_TXCONT);
	ERAY_MSGBUF_PRINT_FLAGS(ERAY_MSGBUF_FIFOREJ_NULL);
	ERAY_MSGBUF_PRINT_FLAGS(ERAY_MSGBUF_FIFOREJ_INSEG);
	pr_cont("\n");
	pr_debug("id      : %d\n", cfg->id);
	pr_debug("cyc     : %d\n", cfg->cyc);
	pr_debug("len     : %d\n", cfg->len);
	pr_debug("max     : %d\n", cfg->max);
	pr_debug("frame_id: %d\n", cfg->frame_id);
	pr_debug("wrhs1   : 0x%08x\n", cfg->wrhs1);
	pr_debug("wrhs2   : 0x%08x\n", cfg->wrhs2);
	pr_debug("wrhs3   : 0x%08x\n", cfg->wrhs3);
	pr_debug("type    : ");
	switch (cfg->type) {
	case eray_msgbuf_type_none:
		pr_cont("NONE\n");
		break;
	case eray_msgbuf_type_fifo:
		pr_cont("FIFO\n");
		break;
	case eray_msgbuf_type_rx:
		pr_cont("RX\n");
		break;
	case eray_msgbuf_type_tx:
		pr_cont("TX\n");
		break;
	default:
		pr_cont("UNKNOWN (%d)\n", cfg->type);
		break;
	}
	pr_debug("channel : ");
	switch (cfg->channel) {
	case eray_msgbuf_ch_none:
		pr_cont("NONE\n");
		break;
	case eray_msgbuf_ch_a:
		pr_cont("CH A\n");
		break;
	case eray_msgbuf_ch_b:
		pr_cont("CH B\n");
		break;
	case eray_msgbuf_ch_both:
		pr_cont("BOTH\n");
		break;
	default:
		pr_cont("UNKNOWN (%d)\n", cfg->channel);
		break;
	}
}
#else
static inline void eray_dump_msg_cfg(struct eray_msgbuf_cfg *cfg, int buf,
		const char *func)
{ }
#endif

static struct net_device *get_dev(struct genl_info *info)
{
	struct net_device *dev = NULL;
	struct nlattr *nla;

	nla = info->attrs[FC_MSGBUF_ATTR_DEV_NAME];
	if (nla)
		dev = dev_get_by_name(&init_net, nla_data(nla));
	if (dev)
		return dev;
	nla = info->attrs[FC_MSGBUF_ATTR_DEV_ID];
	if (nla)
		dev = dev_get_by_index(&init_net, nla_get_u32(nla));

	return dev;
}

static void fc2eray(struct fc_msgbuf_cfg *src, struct eray_msgbuf_cfg *dest)
{
	dest->flags = src->flags;
	dest->cyc = src->cyc;
	dest->len = src->len;
	dest->max = src->max;
	dest->frame_id = src->frame_id;
	dest->reject_mask = src->reject_mask;
	dest->type = src->type;
	dest->channel = src->channel;
}

static void eray2fc(struct eray_msgbuf_cfg *src, struct fc_msgbuf_cfg *dest)
{
	dest->flags = src->flags;
	dest->cyc = src->cyc;
	dest->len = src->len;
	dest->max = src->max;
	dest->frame_id = src->frame_id;
	dest->reject_mask = src->reject_mask;
	dest->type = src->type;
	dest->channel = src->channel;
}

static int remain_buffer_entries(struct eray_cc *cc)
{
	int i, fill;

	/* calculate buffer fill value */
	fill = 0;
	for (i = 0; i < cc->act_cfg; i++) {
		fill += ERAY_MSGBUF_CFG_LEN;
		if (cc->cfg[i].type == eray_msgbuf_type_rx ||
		    cc->cfg[i].type == eray_msgbuf_type_tx)
			fill += DIV_ROUND_UP(cc->cfg[i].max, 2);
		else
			fill += DIV_ROUND_UP(cc->fifo_len, 2);
	}

	return ERAY_MAX_MEM - fill;
}

static int fc_msgbuf_get_cfg(struct sk_buff *_skb, struct genl_info *info)
{
	struct fc_msgbuf_cfg cfg;
	struct fc_msgbuf_cfg *nlcfg;
	struct flexcard_priv *priv;
	struct net_device *dev;
	struct eray_cc *cc;
	struct sk_buff *skb;
	struct nlattr *nla;
	unsigned long flags;
	void *msg_head;
	int ret;
	u8 buf_id;

	dev = get_dev(info);
	if (!dev)
		return -ENODEV;

	priv = netdev_priv(dev);
	cc = priv->cc;

	if (dev->type != ARPHRD_FLEXRAY) {
		ret = -EINVAL;
		goto out;
	}

	nla = info->attrs[FC_MSGBUF_ATTR_BUF_ID];
	if (!nla) {
		ret = -ENOMSG;
		goto out;
	}
	buf_id = nla_get_u8(nla);

	nla = info->attrs[FC_MSGBUF_ATTR_CFG];
	if (!nla) {
		netdev_warn(dev, "no config\n");
		ret = -ENOMSG;
		goto out;
	}
	nlcfg = nla_data(nla);

	spin_lock_irqsave(&cc->lock, flags);

	if (nlcfg->flags & FC_MSGBUF_SELFSYNC) {
		if (buf_id >= cc->ssync_num) {
			netdev_warn(dev, "invalid self sync buffer id %d\n",
					buf_id);
			ret = -ENOMSG;
			spin_unlock_irqrestore(&cc->lock, flags);
			goto out;
		}

		eray2fc(&cc->ssync_cfg[buf_id], &cfg);
		cfg.buf_id = buf_id;
	} else {
		if (buf_id >= cc->act_cfg) {
			netdev_warn(dev, "invalid buffer id %d\n", buf_id);
			ret = -ENOMSG;
			spin_unlock_irqrestore(&cc->lock, flags);
			goto out;
		}

		eray2fc(&cc->cfg[buf_id], &cfg);
		cfg.buf_id = buf_id;
		/* re-adjust length of fifo buffers */
		if (cfg.type == eray_msgbuf_type_fifo)
			cfg.len = cc->fifo_len;
	}
	spin_unlock_irqrestore(&cc->lock, flags);

	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (!skb) {
		ret = -ENOMEM;
		goto out;
	}

	msg_head = genlmsg_put_reply(skb, info, &fc_msgbuf_genl_family,
				      0, FC_MSGBUF_CMD_GET_CFG);
	if (!msg_head) {
		ret = -ENOMEM;
		goto free_skb_out;
	}

	ret = nla_put_u8(skb, FC_MSGBUF_ATTR_BUF_ID, buf_id);
	if (ret)
		goto free_skb_out;

	ret = nla_put(skb, FC_MSGBUF_ATTR_CFG, sizeof(cfg), &cfg);
	if (ret)
		goto free_skb_out;

	genlmsg_end(skb, msg_head);

	ret = genlmsg_reply(skb, info);
out:
	dev_put(dev);

	return ret;

free_skb_out:
	kfree_skb(skb);
	goto out;
}

static int fc_msgbuf_reset_cfg(struct sk_buff *_skb, struct genl_info *info)
{
	struct eray_msgbuf_cfg *cfg;
	struct fc_msgbuf_cfg *nlcfg;
	struct flexcard_priv *priv;
	struct net_device *dev;
	struct nlattr *nla;
	struct eray_cc *cc;
	unsigned long flags;
	int i, ret = -EINVAL;

	dev = get_dev(info);
	if (!dev)
		return -ENODEV;

	if (dev->type != ARPHRD_FLEXRAY)
		goto out;

	priv = netdev_priv(dev);
	cc = priv->cc;

	nla = info->attrs[FC_MSGBUF_ATTR_CFG];
	if (!nla) {
		netdev_warn(dev, "no config\n");
		ret = -ENOMSG;
		goto out;
	}
	nlcfg = nla_data(nla);

	spin_lock_irqsave(&cc->lock, flags);

	if (nlcfg->flags & FC_MSGBUF_SELFSYNC) {
		cc->ssync_start = 0;
		cc->ssync_num = 0;
		memset(&cc->ssync_cfg, 0x0, sizeof(cc->ssync_cfg));

		ret = 0;
		goto out_unlock;
	}

	cc->ready = 0;

	for (i = 0; i < ERAY_MAX_BUFS; i++) {
		cc->cfg[i].flags = 0;
		cc->cfg[i].len = 0;
		cc->cfg[i].max = 0;
		cc->cfg[i].cyc = 0;
		cc->cfg[i].channel = eray_msgbuf_ch_none;
	}
	cc->act_cfg = 1;
	cc->fifo_len = 0;
	cc->sync_start = 0;
	cc->sync_num = 0;
	memset(&cc->sync_cfg, 0x0, sizeof(cc->sync_cfg));

	cc->fifo_threshold = ERAY_FIFO_THRESHOLD;

	cfg = &cc->cfg[0];
	cfg->flags |= ERAY_MSGBUF_USED;
	cfg->type = eray_msgbuf_type_fifo;
	cfg->max = 127;

	/* CLEAR_RAMS can only be called in DEFAULT_CONFIG or CONFIG mode */
	ret = cc_change_state(cc, ERAY_CMD_CONFIG, 5);
	if (ret < 0) {
		netdev_err(dev, "CC DEFAULT_CONFIG failed\n");
		goto out_unlock;
	}

	ret = cc_change_state(cc, ERAY_CMD_CLEAR_RAMS, 15);
	if (ret < 0)
		netdev_err(dev, "%s: CC CLEAR_RAMS failed\n", __func__);
out_unlock:
	spin_unlock_irqrestore(&cc->lock, flags);
out:
	dev_put(dev);
	return ret;
}

static int fc_msgbuf_read_cfg(struct sk_buff *_skb, struct genl_info *info)
{
	struct eray_msgbuf_cfg *cfg;
	struct flexcard_priv *priv;
	struct net_device *dev;
	struct eray_cc *cc;
	unsigned long flags;
	u32 reg;
	u8 fdb, ffb, lcb, splm, act = 1;
	int i, ret = -EINVAL;

	dev = get_dev(info);
	if (!dev)
		return -ENODEV;

	if (dev->type != ARPHRD_FLEXRAY)
		goto out;

	priv = netdev_priv(dev);
	cc = priv->cc;

	spin_lock_irqsave(&cc->lock, flags);

	for (i = 0; i < ERAY_MAX_BUFS; i++) {
		cc->cfg[i].flags = 0;
		cc->cfg[i].len = 0;
		cc->cfg[i].max = 0;
		cc->cfg[i].cyc = 0;
		cc->cfg[i].channel = eray_msgbuf_ch_both;
	}

	cc->fifo_threshold = ERAY_FIFO_THRESHOLD;

	/* The FlexCard firmware needs one or more configured FIFO buffer
	 * to receive FlexRay frames.
	 */

	cfg = &cc->cfg[0];
	cfg->flags |= ERAY_MSGBUF_USED;
	cfg->type = eray_msgbuf_type_fifo;
	cfg->max = 127;

	reg = eray_readl(cc, ERAY_FRF);
	cfg->channel = (reg & ERAY_FRF_CH_MASK) >> ERAY_FRF_CH_SHIFT;
	cfg->frame_id = (reg & ERAY_FRF_FID_MASK) >> ERAY_FRF_FID_SHIFT;
	cfg->cyc = (reg & ERAY_FRF_CYC_MASK) >> ERAY_FRF_CYC_SHIFT;
	if (reg & ERAY_FRF_RNF_MASK)
		cfg->flags |= ERAY_MSGBUF_FIFOREJ_NULL;
	if (reg & ERAY_FRF_RSS_MASK)
		cfg->flags |= ERAY_MSGBUF_FIFOREJ_INSEG;

	reg = eray_readl(cc, ERAY_FRFM);
	cfg->reject_mask = (reg & ERAY_FRFM_MFID_MASK) >> ERAY_FRFM_MFID_SHIFT;

	cc->act_cfg = 1;

	reg = eray_readl(cc, ERAY_MRC);
	lcb = (reg & ERAY_MRC_LCB_MASK) >> ERAY_MRC_LCB_SHIFT;
	ffb = (reg & ERAY_MRC_FFB_MASK) >> ERAY_MRC_FFB_SHIFT;
	fdb = (reg & ERAY_MRC_FDB_MASK) >> ERAY_MRC_FDB_SHIFT;
	splm = (reg & ERAY_MRC_SPLM_MASK) >> ERAY_MRC_SPLM_SHIFT;

	for (i = 0; i < lcb + 1; i++) {
		eray_writel(ERAY_OBCM_RHSS_MASK, cc, ERAY_OBCM);
		eray_writel(i | ERAY_OBCR_REQ_MASK, cc, ERAY_OBCR);
		ret = eray_wait_clear(cc, ERAY_OBCR, ERAY_OBCR_OBSYS_MASK, 10);
		if (ret)
			goto out_unlock;

		eray_writel(ERAY_OBCR_VIEW_MASK, cc, ERAY_OBCR);
		eray_readl(cc, ERAY_OBCR);

		cfg = &cc->cfg[act];

		cfg->wrhs1 = eray_readl(cc, ERAY_RDHS1);
		cfg->wrhs2 = eray_readl(cc, ERAY_RDHS2);
		cfg->wrhs3 = eray_readl(cc, ERAY_RDHS3);

		cfg->id = i;
		cc->rev_id[i] = cfg->id;
		cfg->flags |= ERAY_MSGBUF_USED;

		if (i > ffb) {
			cfg->type = eray_msgbuf_type_fifo;
			cfg->max = (cfg->wrhs2 >> ERAY_WRHS2_PLC_SHIFT) &
				ERAY_WRHS2_PLC_MASK;
			if (!cc->fifo_len)
				cc->fifo_len = cfg->max;

			/* copy fifo reject configuration from message buffer 0
			 */
			cfg->channel = cc->cfg[0].channel;
			cfg->frame_id = cc->cfg[0].frame_id;
			cfg->cyc = cc->cfg[0].cyc;
			cfg->reject_mask = cc->cfg[0].reject_mask;

			if (cc->fifo_threshold)
				cc->fifo_threshold--;
		} else {
			if (cfg->wrhs1 & ERAY_WRHS1_CFG_MASK)
				cfg->type = eray_msgbuf_type_tx;
			else
				cfg->type = eray_msgbuf_type_rx;

			/* WRHS1:
			 * 31-16: MBI:TXM:PPIT:CFG:CHB:CHA:0:CYC[6..0]
			 * 15- 0: 00000:FID[10..0]
			 */
			cfg->frame_id = (cfg->wrhs1 & ERAY_WRHS1_FID_MASK) >>
				ERAY_WRHS1_FID_SHIFT;

			if (!cfg->frame_id)
				continue;

			cfg->cyc = (cfg->wrhs1 & ERAY_WRHS1_CYC_MASK) >>
				ERAY_WRHS1_CYC_SHIFT;
			cfg->channel = (cfg->wrhs1 & ERAY_WRHS1_CH_MASK) >>
				ERAY_WRHS1_CH_SHIFT;
			if (cfg->wrhs1 & ERAY_WRHS1_PPIT_MASK)
				cfg->flags |= ERAY_MSGBUF_PPIT;

			/* if ERAY_WRHS1_TXM is *not* set the ERAY core
			 * works in continous mode.
			 */
			if ((cfg->type == eray_msgbuf_type_tx) &&
			    !(cfg->wrhs1 & ERAY_WRHS1_TXM_MASK))
				cfg->flags |= ERAY_MSGBUF_TXCONT;

			/* WRHS2:
			 * 31- 0: 000000000:PLC[6-0]:00000:CRC[10..0]
			 */
			cfg->max = (cfg->wrhs2 & ERAY_WRHS2_PLC_MASK) >>
				ERAY_WRHS2_PLC_SHIFT;
		}
		cfg->len = cfg->max;
		act++;

		eray_dump_msg_cfg(cfg, i, __func__);
	}

	reg = eray_read_succ1(cc, 10);
	if (reg < 0) {
		ret = -EBUSY;
		goto out_unlock;
	}

	if (reg & ERAY_SUCC1_TXST_MASK) {
		cc->sync_start |= ERAY_MSGBUF_STARTUP;
		cc->cfg[1].flags |= ERAY_MSGBUF_STARTUP;
		if (splm)
			cc->cfg[2].flags |= ERAY_MSGBUF_STARTUP;
	}

	if (reg & ERAY_SUCC1_TXSY_MASK) {
		cc->sync_start |= ERAY_MSGBUF_SYNC;
		cc->cfg[1].flags |= ERAY_MSGBUF_SYNC;
		if (splm)
			cc->cfg[2].flags |= ERAY_MSGBUF_SYNC;
	}

	cc_get_conf(dev);
	ret = 0;
	cc->ready = 1;
	cc->act_cfg = act;

out_unlock:
	spin_unlock_irqrestore(&cc->lock, flags);
out:
	dev_put(dev);
	return ret;
}

static int is_double_sync(struct fc_msgbuf_cfg *cfg,
		struct eray_msgbuf_cfg *sync_cfg, int sync_num)
{
	if (!sync_num)
		return 0;

	if (sync_num > 1)
		return 1;

	if ((cfg->cyc != sync_cfg->cyc) ||
	    (cfg->len != sync_cfg->len) ||
	    (cfg->max != sync_cfg->max) ||
	    (cfg->frame_id != sync_cfg->frame_id))
		return 1;

	if (cfg->channel == eray_msgbuf_ch_both)
		return 1;

	if ((sync_cfg->channel == eray_msgbuf_ch_a) &&
	    (cfg->channel == eray_msgbuf_ch_a))
		return 1;

	if ((sync_cfg->channel == eray_msgbuf_ch_b) &&
	    (cfg->channel == eray_msgbuf_ch_b))
		return 1;

	return 0;
}

static int fc_msgbuf_set_cfg_ssync(struct net_device *dev,
		struct fc_msgbuf_cfg *cfg, struct eray_cc *cc, int aquire)
{
	u8 buf_id;

	if (aquire) {
		buf_id = cc->ssync_num;
	} else {
		buf_id = cfg->buf_id;

		if (!(cc->ssync_start & ERAY_MSGBUF_SYNC)) {
			netdev_warn(dev, "self sync not yet configured\n");
			return -ENOMSG;
		}

		if (buf_id > cc->ssync_num) {
			netdev_warn(dev, "unused buffer id %d\n", buf_id);
			return -ENOMSG;
		}
	}

	if (buf_id > ERAY_MAX_BUFS_SSYNC) {
		netdev_warn(dev, "invalid buffer id %d\n", buf_id);
		return -EINVAL;
	}

	if (!(cfg->flags & ERAY_MSGBUF_SYNC) ||
	    !(cfg->flags & ERAY_MSGBUF_STARTUP)) {
		netdev_warn(dev, "no sync/start frame\n");
		return -EINVAL;
	}

	if (cfg->type != eray_msgbuf_type_tx) {
		netdev_warn(dev, "wrong type (!tx)\n");
		return -EINVAL;
	}

	if (cfg->cyc > 1) {
		netdev_warn(dev, "cycle counter filter not valid\n");
		return -EINVAL;
	}

	if (cfg->channel == eray_msgbuf_ch_none) {
		netdev_warn(dev, "channel not valid for sync (none)\n");
		return -EINVAL;
	}

	/* cc->ssync_num == 0 is coverd by is_double_sync() */
	if (is_double_sync(cfg, &cc->ssync_cfg[0], cc->ssync_num)) {
		netdev_warn(dev, "double sync frame\n");
		return -EINVAL;
	}

	if (cfg->frame_id > cc->static_id) {
		netdev_warn(dev, "sync frame not in static segment\n");
		return -EINVAL;
	}

	cfg->buf_id = buf_id;

	cc->ssync_start |= ERAY_MSGBUF_SYNC;
	cc->ssync_start |= ERAY_MSGBUF_STARTUP;

	fc2eray(cfg, &cc->ssync_cfg[buf_id]);
	cc->ssync_cfg[buf_id].flags |= ERAY_MSGBUF_USED;

	if (aquire)
		cc->ssync_num++;

	return 0;
}

static int fc_msgbuf_set_cfg(struct sk_buff *_skb, struct genl_info *info)
{
	struct fc_msgbuf_cfg *cfg;
	struct flexcard_priv *priv;
	struct net_device *dev;
	struct eray_cc *cc;
	struct sk_buff *skb;
	struct nlattr *nla;
	unsigned long flags;
	void *msg_head;
	int aquire, ret;
	int i;
	u8 buf_id;

	dev = get_dev(info);
	if (!dev)
		return -ENODEV;

	if (dev->type != ARPHRD_FLEXRAY) {
		netdev_warn(dev, "device is not a FlexRay device\n");
		ret = -EINVAL;
		goto out;
	}
	priv = netdev_priv(dev);
	cc = priv->cc;

	spin_lock_irqsave(&cc->lock, flags);
	cc->ready = 0;
	nla = info->attrs[FC_MSGBUF_ATTR_BUF_ID];
	if (nla) {
		buf_id = nla_get_u8(nla);
		aquire = 0;
	} else {
		buf_id = cc->act_cfg;
		aquire = 1;
	}

	nla = info->attrs[FC_MSGBUF_ATTR_CFG];
	if (!nla) {
		netdev_warn(dev, "no config\n");
		ret = -ENOMSG;
		goto out_unlock;
	}
	cfg = nla_data(nla);

	if (cfg->flags & FC_MSGBUF_SELFSYNC) {
		ret = fc_msgbuf_set_cfg_ssync(dev, cfg, cc, aquire);
		if (ret)
			goto out_unlock;

		buf_id = cfg->buf_id;
		goto nlreply_unlock;
	}

	if (aquire) {
		/* The ERAY core can only allocate memory in 32-bit chunks,
		 * while FlexRay is based on 16-bit words. To allocate a
		 * proper amount of chunks use M = (N + 1)/2
		 * M = number of chunks (32bit)
		 * N = number of FlexRay words (16bit)
		 */

		if (remain_buffer_entries(cc) < 0) {
			netdev_warn(dev, "no room for header\n");
			ret = -ENOMEM;
			goto out_unlock;
		}
	}

	if (buf_id >= ERAY_MAX_BUFS) {
		netdev_warn(dev, "invalid buffer id %d\n", buf_id);
		ret = -ERANGE;
		goto out_unlock;
	}

	if (!aquire && !(cc->cfg[buf_id].flags & ERAY_MSGBUF_USED)) {
		netdev_warn(dev, "buffer %d not in use\n", buf_id);
		ret = -ENOMSG;
		goto out_unlock;
	}

	if (cfg->max > 254) {
		netdev_warn(dev,
			  "payload len %d no valid\n", cfg->max);
		ret = -EINVAL;
		goto out_unlock;
	}

	if (cfg->type == eray_msgbuf_type_fifo) {
		cfg->flags &= ~ERAY_MSGBUF_STARTUP;
		cfg->flags &= ~ERAY_MSGBUF_SYNC;

		if (cc->fifo_threshold)
			cc->fifo_threshold--;
	}

	if ((cc->act_cfg + cc->fifo_threshold) >= ERAY_MAX_BUFS) {
		netdev_warn(dev, "min fifo limit reached\n");
		ret = -E2BIG;
		goto out_unlock;
	}

	if (cfg->flags & ERAY_MSGBUF_SYNC) {
		if (cfg->channel == eray_msgbuf_ch_none) {
			netdev_warn(dev,
				    "channel not valid for sync (none)\n");
			ret = -EINVAL;
			goto out_unlock;
		}
		if (is_double_sync(cfg, &cc->sync_cfg, cc->sync_num)) {
			netdev_warn(dev, "double sync frame\n");
			ret = -EINVAL;
			goto out_unlock;
		}
		cc->sync_start |= ERAY_MSGBUF_SYNC;
		cc->sync_num++;
		fc2eray(cfg, &cc->sync_cfg);
	}

	if (cfg->flags & ERAY_MSGBUF_STARTUP) {
		if (!(cfg->flags & ERAY_MSGBUF_SYNC)) {
			netdev_warn(dev,
				    "startup frame is not sync frame\n");
			ret = -EINVAL;
			goto out_unlock;
		}
		cc->sync_start |= ERAY_MSGBUF_STARTUP;
	}

	if ((cfg->flags & ERAY_MSGBUF_SYNC) &&
	    (cfg->frame_id > cc->static_id)) {
		netdev_warn(dev, "sync frame not in static segment\n");
		ret = -EINVAL;
		goto out_unlock;
	}


	/* avoid duplicate configuration */
	if (aquire && cfg->type != eray_msgbuf_type_fifo) {
		for (i = 0; i < cc->act_cfg; i++) {
			if ((cc->cfg[i].frame_id == cfg->frame_id) &&
			    (cc->cfg[i].channel & cfg->channel) &&
			    (cc->cfg[i].cyc ==  cfg->cyc)) {
				netdev_warn(dev, "duplicate configuration\n");
				ret = -EINVAL;
				goto out_unlock;
			}
		}
	}

	if (cfg->type == eray_msgbuf_type_fifo) {
		if (!cc->fifo_len)
			cc->fifo_len = cfg->max;
		else if (cfg->max != cc->fifo_len) {
			netdev_warn(dev, "payload len %d != %d\n",
				  cfg->max, cc->fifo_len);
			ret = -EINVAL;
			goto out_unlock;
		}
		/* copy fifo reject configuration to message buffer 0 */
		cc->cfg[0].channel = cfg->channel;
		cc->cfg[0].frame_id = cfg->frame_id;
		cc->cfg[0].cyc = cfg->cyc;
		cc->cfg[0].reject_mask = cfg->reject_mask;
	}

	if (cfg->type == eray_msgbuf_type_tx) {
		if (cfg->len > cfg->max) {
			netdev_warn(dev, "payload len %d > payload len %d\n",
					cfg->len, cfg->max);
			ret = -EINVAL;
			goto out_unlock;
		}
		/* reject messagebuffer if it's static and the length is bigger
		 * than the cluster wide static messagebuffer length
		 */
		if (cfg->frame_id <= cc->static_id &&
				cfg->len > cc->static_len) {
			netdev_warn(dev, "payload len %d > static_len %d\n",
					cfg->len, cc->static_len);
			ret = -EINVAL;
			goto out_unlock;
		}
		if (cfg->channel == eray_msgbuf_ch_both &&
				cfg->frame_id > cc->static_id) {
			netdev_warn(dev, "both channel in dynamic frame\n");
			ret = -EINVAL;
			goto out_unlock;
		}
	}

	if ((remain_buffer_entries(cc) - ERAY_MSGBUF_CFG_LEN -
				DIV_ROUND_UP(cfg->max, 2)) < 0) {
		netdev_warn(dev, "no room for buffer\n");
		ret = -EINVAL;
		goto out_unlock;
	}

	fc2eray(cfg, &cc->cfg[buf_id]);

	cc->cfg[buf_id].flags |= ERAY_MSGBUF_USED;
	if (aquire)
		cc->act_cfg++;
nlreply_unlock:
	spin_unlock_irqrestore(&cc->lock, flags);

	skb = genlmsg_new(NLMSG_GOODSIZE, GFP_KERNEL);
	if (!skb) {
		netdev_warn(dev, "could not allocate skb\n");
		ret = -ENOMEM;
		goto out;
	}

	msg_head = genlmsg_put_reply(skb, info, &fc_msgbuf_genl_family, 0,
			FC_MSGBUF_CMD_SET_CFG);
	if (!msg_head) {
		netdev_warn(dev, "generating NL reply failed\n");
		ret = -ENOMEM;
		goto out_free_skb;
	}
	ret = nla_put_u8(skb, FC_MSGBUF_ATTR_BUF_ID, buf_id);
	if (ret) {
		netdev_warn(dev, "could not add buffer id\n");
		ret = -EINVAL;
		goto out_free_skb;
	}
	genlmsg_end(skb, msg_head);

	ret = genlmsg_reply(skb, info);
out:
	dev_put(dev);
	return ret;

out_free_skb:
	kfree(skb);
	goto out;

out_unlock:
	spin_unlock_irqrestore(&cc->lock, flags);
	dev_put(dev);
	return ret;
}

static struct genl_ops fc_msgbuf_genl_ops[] = {
	{
		.cmd = FC_MSGBUF_CMD_GET_CFG,
		.policy = fc_msgbuf_genl_policy,
		.doit = fc_msgbuf_get_cfg,
	},
	{
		.cmd = FC_MSGBUF_CMD_SET_CFG,
		.policy = fc_msgbuf_genl_policy,
		.doit = fc_msgbuf_set_cfg,
	},
	{
		.cmd = FC_MSGBUF_CMD_RESET_CFG,
		.policy = fc_msgbuf_genl_policy,
		.doit = fc_msgbuf_reset_cfg,
	},
	{
		.cmd = FC_MSGBUF_CMD_READ_CFG,
		.policy = fc_msgbuf_genl_policy,
		.doit = fc_msgbuf_read_cfg,
	},
};

/* calculate header CRC as specified in FlexRay Protocol Specification V2.1
 * Rev.A chapter 4.2.8
 */
static u32 crc_header(struct eray_msgbuf_cfg *cfg)
{
	u32 val = 0x1a;
	u32 crc_next;
	u32 next_bit;
	u32 data;
	int i;

	data = cfg->len & (ERAY_WRHS2_PLC_MASK >> ERAY_WRHS2_PLC_SHIFT);
	data |= (cfg->frame_id & ERAY_WRHS1_FID_MASK) << 7;
	if (cfg->flags & ERAY_MSGBUF_STARTUP)
		data |= 1 << 18;
	if (cfg->flags & ERAY_MSGBUF_SYNC)
		data |= 1 << 19;

	for (i = 19; i >= 0; i--) {
		next_bit = (data >> i) & 0x1;
		crc_next = next_bit ^ ((val >> 10) & 0x1);

		val <<= 1;
		val &= 0xfffffffe;

		if (crc_next)
			val ^= 0x385;

		val &= 0x7ff;
	}

	return val & ERAY_WRHS2_CRC_MASK;
}

static int fc_wait_for_ram_offset(struct eray_cc *cc, uint32_t offset)
{
	int ret;

	/* wait until host and shadow RAM actions are finished */
	ret = eray_wait_clear(cc, ERAY_IBCR + offset, ERAY_IBCR_IBSYH_MASK, 10);
	if (ret)
		goto out;

	ret = eray_wait_clear(cc, ERAY_IBCR + offset, ERAY_IBCR_IBSYS_MASK, 10);
out:
	return ret;
}

static int fc_wait_for_ram(struct eray_cc *cc)
{
	return fc_wait_for_ram_offset(cc, 0);
}

static int fc_prepare_msgbuf_data_ssync(struct net_device *dev)
{
	struct flexcard_priv *priv = netdev_priv(dev);
	struct eray_cc *cc = priv->cc;
	struct eray_msgbuf_cfg *cfg;
	u8 fdb, ffb, lcb, splm;
	u32 reg, dp = ERAY_MAX_MEM;
	int i, ret;

	if (!cc->ssync_num)
		return 0;

	fdb = 0x80;	/* only static msgbuf in self sync */
	ffb = 0x80;	/* non FIFO in self sync */
	lcb = cc->ssync_num ? cc->ssync_num - 1 : 0x80;
	splm = (cc->ssync_num > 1) ? 1 : 0;

	eray_chg_reg(fdb, cc, ERAY_MRC + FC_SSYNC_OFFSET,
		     ERAY_MRC_FDB_MASK, ERAY_MRC_FDB_SHIFT);
	eray_chg_reg(ffb, cc, ERAY_MRC + FC_SSYNC_OFFSET,
		     ERAY_MRC_FFB_MASK, ERAY_MRC_FFB_SHIFT);
	eray_chg_reg(lcb, cc, ERAY_MRC + FC_SSYNC_OFFSET,
		     ERAY_MRC_LCB_MASK, ERAY_MRC_LCB_SHIFT);
	eray_chg_reg(splm, cc, ERAY_MRC + FC_SSYNC_OFFSET,
		     ERAY_MRC_SPLM_MASK, ERAY_MRC_SPLM_SHIFT);

	for (i = 0; i < cc->ssync_num; i++) {
		cfg = &cc->ssync_cfg[i];

		/* self sync need no mapping */
		cfg->id = i;

		cfg->len = cc->static_len;

		/* WRHS1 */
		reg = (cfg->frame_id << ERAY_WRHS1_FID_SHIFT) &
			ERAY_WRHS1_FID_MASK;
		reg |= (cfg->cyc << ERAY_WRHS1_CYC_SHIFT) &
			ERAY_WRHS1_CYC_MASK;
		reg |= (cfg->channel << ERAY_WRHS1_CH_SHIFT) &
			ERAY_WRHS1_CH_MASK;
		reg |= ERAY_WRHS1_CFG_MASK;
		if (cfg->flags & ERAY_MSGBUF_PPIT)
			reg |= ERAY_WRHS1_PPIT_MASK;
		if (cfg->flags & FC_MSGBUF_ACK)
			reg |= ERAY_WRHS1_MBI_MASK;

		/* if ERAY_WRHS1_TXM is *not* set the ERAY core
		 * works in continous mode.
		 */
		if (!(cfg->flags & ERAY_MSGBUF_TXCONT))
			reg |= ERAY_WRHS1_TXM_MASK;
		cfg->wrhs1 = reg;

		/* WRHS2 */
		reg = crc_header(cfg) & ERAY_WRHS2_CRC_MASK;
		reg |= (cfg->max << ERAY_WRHS2_PLC_SHIFT) &
			ERAY_WRHS2_PLC_MASK;
		cfg->wrhs2 = reg;

		/* WRHS3 */
		dp -= DIV_ROUND_UP(cfg->max, 2);
		reg = dp & ERAY_WRHS3_DP_MASK;
		cfg->wrhs3 = reg;

		eray_dump_msg_cfg(cfg, i, __func__);

		ret = fc_wait_for_ram_offset(cc, FC_SSYNC_OFFSET);
		if (ret)
			goto out;

		eray_writel(cfg->wrhs1, cc, ERAY_WRHS1 + FC_SSYNC_OFFSET);
		eray_writel(cfg->wrhs2, cc, ERAY_WRHS2 + FC_SSYNC_OFFSET);
		eray_writel(cfg->wrhs3, cc, ERAY_WRHS3 + FC_SSYNC_OFFSET);
		eray_writel(ERAY_IBCM_LHSH_MASK, cc,
				ERAY_IBCM + FC_SSYNC_OFFSET);

		/* set the new configuration */
		eray_writel(cfg->id, cc, ERAY_IBCR + FC_SSYNC_OFFSET);

		ret = fc_wait_for_ram_offset(cc, FC_SSYNC_OFFSET);
		if (ret)
			goto out;

		reg = 0x0;
		if (cfg->flags & FC_MSGBUF_ACK) {
			reg |= cfg->cyc << FC_BUF_INFO_CYC_SHIFT;
			reg |= cfg->channel << FC_BUF_INFO_CHANNEL_SHIFT;
			reg |= cfg->frame_id;

			if (cfg->flags & FC_MSGBUF_ACK_PAYLOAD)
				reg |= FC_BUF_INFO_ENABLE_PAYLOAD;
			if (cfg->flags & FC_MSGBUF_ACK_NULL)
				reg |= FC_BUF_INFO_ENABLE_NULLFRAMES;
			if (cfg->type == eray_msgbuf_type_tx)
				reg |= FC_BUF_INFO_IS_TX;
		}

		eray_writel(reg, cc, (FC_BUFFER_INFO_TABLE + i * 4) +
				FC_SSYNC_TXACK_OFFSET);
	}

	for (i = 0; i < cc->ssync_num; i++) {
		int tx_buf_id;

		cfg = &cc->ssync_cfg[i];
		if (cfg->flags & ERAY_MSGBUF_TXCONT) {
			tx_buf_id = i | FC_FLEX_ID_SSYNC_FLAG;
			ret = _fc_write_data(priv, tx_buf_id,
					     cfg->tx_cont_data,
					     cfg->tx_cont_len);
			if (ret)
				goto out;
		}
	}

out:
	return ret;
}

static int fc_prepare_msgbuf_data(struct net_device *dev)
{
	struct flexcard_priv *priv = netdev_priv(dev);
	struct eray_cc *cc = priv->cc;
	struct eray_msgbuf_cfg *cfg;
	unsigned long flags;
	u32 reg, sum_flags = 0, dp = ERAY_MAX_MEM;
	u8 fdb, ffb, lcb, splm, ndyn = 0, nfifo = 0;
	int i, remain, ret = -EINVAL, map_i = 0;
	unsigned int remain_nr;

	spin_lock_irqsave(&cc->lock, flags);

	if (cc->act_cfg > ERAY_MAX_BUFS) {
		netdev_err(dev, "too many msg buffers (%d)\n", cc->act_cfg);
		goto out;
	}

	for (i = 0; i < cc->act_cfg; i++) {
		cc->cfg[i].id = 0xff;
		cc->cfg[i].len = 0;
	}

	if (!cc->fifo_len)
		cc->fifo_len = 127;

	remain = remain_buffer_entries(cc);
	if (remain < 0) {
		netdev_err(dev, "buffer memory exhausted (%d)\n",
			   ERAY_MAX_MEM + remain);
		goto out;
	}

	/* map all remaining message buffers as fifo buffer */
	remain_nr = DIV_ROUND_UP(cc->fifo_len, 2) + ERAY_MSGBUF_CFG_LEN;
	remain_nr = remain / remain_nr;
	if (remain_nr)
		remain_nr--;

	if (cc->act_cfg + remain_nr > ERAY_MAX_BUFS)
		remain_nr = ERAY_MAX_BUFS - cc->act_cfg;

	for (i = cc->act_cfg; i < cc->act_cfg + remain_nr; i++) {
		cc->cfg[i].flags |= ERAY_MSGBUF_USED;
		cc->cfg[i].id = 0xff;
		cc->cfg[i].type = eray_msgbuf_type_fifo;

		/* copy fifo reject configuration from message buffer 0 */
		cc->cfg[i].channel = cc->cfg[0].channel;
		cc->cfg[i].frame_id = cc->cfg[0].frame_id;
		cc->cfg[i].cyc = cc->cfg[0].cyc;
		cc->cfg[i].reject_mask = cc->cfg[0].reject_mask;
	}
	cc->act_cfg += remain_nr;

	eray_get_val16(&cc->static_id, cc,
		      ERAY_GTUC7, ERAY_GTUC7_NSS_MASK, ERAY_GTUC7_NSS_SHIFT);

	eray_get_val8(&cc->static_len, cc,
		      ERAY_MHDC, ERAY_MHDC_SFDL_MASK, ERAY_MHDC_SFDL_SHIFT);

	/* Map sync buffers first */
	for (i = 0; i < cc->act_cfg; i++) {
		cfg = &cc->cfg[i];

		if (cfg->type != eray_msgbuf_type_tx)
			continue;
		if (cfg->frame_id == 0)
			continue;
		if (!(cfg->flags & ERAY_MSGBUF_SYNC))
			continue;

		cfg->id = map_i;
		cc->rev_id[map_i] = cfg->id;
		map_i++;
		ndyn++;
	}

	/* then map tx buffers */
	for (i = 0; i < cc->act_cfg; i++) {
		cfg = &cc->cfg[i];

		if (cfg->type != eray_msgbuf_type_tx)
			continue;
		if (cfg->frame_id == 0)
			continue;
		if (cfg->flags & ERAY_MSGBUF_SYNC)
			continue;

		cfg->id = map_i;
		cc->rev_id[map_i] = cfg->id;
		map_i++;
		ndyn++;
	}

	/* then map RX buffers */
	for (i = 0; i < cc->act_cfg; i++) {
		cfg = &cc->cfg[i];

		if (cfg->type != eray_msgbuf_type_rx)
			continue;

		cfg->id = map_i;
		cc->rev_id[map_i] = cfg->id;
		map_i++;
		ndyn++;
	}

	/* then map fifo buffers */
	for (i = 0; i < cc->act_cfg; i++) {

		cfg = &cc->cfg[i];

		if (cfg->type != eray_msgbuf_type_fifo)
			continue;

		/* assign mapping */
		cfg->len = cfg->max = cc->fifo_len;
		cfg->id = map_i;

		/* summarize all flags */
		cc->rev_id[map_i] = cfg->id;
		sum_flags |= cfg->flags;

		map_i++;
		nfifo++;
	}

	/* CLEAR_RAMS can only be called in DEFAULT_CONFIG or CONFIG mode */
	ret = cc_change_state(cc, ERAY_CMD_CONFIG, 5);
	if (ret < 0) {
		netdev_err(dev, "CC DEFAULT_CONFIG failed\n");
		goto out;
	}

	ret = cc_change_state(cc, ERAY_CMD_CLEAR_RAMS, 15);
	if (ret < 0)
		netdev_err(dev, "%s: CC CLEAR_RAMS failed\n", __func__);

	fdb = ndyn ? 0 : 0x80;
	ffb = nfifo ? ndyn : 0x80;
	lcb = map_i ? map_i - 1 : 0x80;
	splm = (cc->sync_num > 1) ? 1 : 0;

	if (ndyn + nfifo != map_i)
		netdev_warn(dev, "invalid msg buffer configuration\n");

	eray_chg_reg(fdb, cc, ERAY_MRC, ERAY_MRC_FDB_MASK, ERAY_MRC_FDB_SHIFT);
	eray_chg_reg(ffb, cc, ERAY_MRC, ERAY_MRC_FFB_MASK, ERAY_MRC_FFB_SHIFT);
	eray_chg_reg(lcb, cc, ERAY_MRC, ERAY_MRC_LCB_MASK, ERAY_MRC_LCB_SHIFT);
	eray_chg_reg(splm, cc, ERAY_MRC, ERAY_MRC_SPLM_MASK,
			ERAY_MRC_SPLM_SHIFT);
	/* setup data for registers */
	for (i = 0; i < cc->act_cfg; i++) {
		cfg = &cc->cfg[i];

		switch (cfg->type) {
		case eray_msgbuf_type_none:
			continue;

		case eray_msgbuf_type_fifo:
			cfg->wrhs1 = 0;
			cfg->wrhs2 = (cc->fifo_len << ERAY_WRHS2_PLC_SHIFT) &
				ERAY_WRHS2_PLC_MASK;

			reg = (cfg->frame_id << ERAY_FRF_FID_SHIFT) &
				ERAY_FRF_FID_MASK;
			reg |= (cfg->cyc << ERAY_FRF_CYC_SHIFT) &
				ERAY_FRF_CYC_MASK;
			reg |= (cfg->channel << ERAY_FRF_CH_SHIFT) &
				ERAY_FRF_CH_MASK;
			if (sum_flags & ERAY_MSGBUF_FIFOREJ_NULL)
				reg |= ERAY_FRF_RNF_MASK;
			if (sum_flags & ERAY_MSGBUF_FIFOREJ_INSEG)
				reg |= ERAY_FRF_RSS_MASK;

			eray_writel(reg, cc, ERAY_FRF);

			reg = (cfg->reject_mask << ERAY_FRFM_MFID_SHIFT) &
				ERAY_FRFM_MFID_MASK;

			eray_writel(reg, cc, ERAY_FRFM);

			break;

		case eray_msgbuf_type_rx:
			reg = (cfg->frame_id << ERAY_WRHS1_FID_SHIFT) &
				ERAY_WRHS1_FID_MASK;
			reg |= (cfg->cyc << ERAY_WRHS1_CYC_SHIFT) &
				ERAY_WRHS1_CYC_MASK;
			reg |= (cfg->channel << ERAY_WRHS1_CH_SHIFT) &
				ERAY_WRHS1_CH_MASK;
			cfg->wrhs1 = reg;

			reg = (cfg->max << ERAY_WRHS2_PLC_SHIFT) &
				ERAY_WRHS2_PLC_MASK;
			cfg->wrhs2 = reg;
			break;

		case eray_msgbuf_type_tx:
			reg = (cfg->frame_id << ERAY_WRHS1_FID_SHIFT) &
				ERAY_WRHS1_FID_MASK;
			reg |= (cfg->cyc << ERAY_WRHS1_CYC_SHIFT) &
				ERAY_WRHS1_CYC_MASK;
			reg |= (cfg->channel << ERAY_WRHS1_CH_SHIFT) &
				ERAY_WRHS1_CH_MASK;
			reg |= ERAY_WRHS1_CFG_MASK;
			if (cfg->flags & ERAY_MSGBUF_PPIT)
				reg |= ERAY_WRHS1_PPIT_MASK;
			if (cfg->flags & FC_MSGBUF_ACK)
				reg |= ERAY_WRHS1_MBI_MASK;

			/* if ERAY_WRHS1_TXM is *not* set the ERAY core
			 * works in continous mode.
			 */
			if (!(cfg->flags & ERAY_MSGBUF_TXCONT))
				reg |= ERAY_WRHS1_TXM_MASK;
			cfg->wrhs1 = reg;

			if (cfg->frame_id <= cc->static_id)
				cfg->len = cc->static_len;
			else
				cfg->len = cfg->max;

			reg = crc_header(cfg) & ERAY_WRHS2_CRC_MASK;
			reg |= (cfg->max << ERAY_WRHS2_PLC_SHIFT) &
				ERAY_WRHS2_PLC_MASK;
			cfg->wrhs2 = reg;
			break;

		default:
			netdev_warn(dev, "unknown msgbuf type %d ignored\n",
				    cfg->type);
			continue;
		}

		/* WRHS3 */
		dp -= DIV_ROUND_UP(cfg->max, 2);
		reg = dp & ERAY_WRHS3_DP_MASK;
		cfg->wrhs3 = reg;

		eray_dump_msg_cfg(cfg, i, __func__);

		ret = fc_wait_for_ram(cc);
		if (ret)
			goto out;

		eray_writel(cfg->wrhs1, cc, ERAY_WRHS1);
		eray_writel(cfg->wrhs2, cc, ERAY_WRHS2);
		eray_writel(cfg->wrhs3, cc, ERAY_WRHS3);
		eray_writel(ERAY_IBCM_LHSH_MASK, cc, ERAY_IBCM);

		/* set the new configuration */
		eray_writel(cfg->id, cc, ERAY_IBCR);

		ret = fc_wait_for_ram(cc);
		if (ret)
			goto out;

		reg = 0x0;
		if (cfg->flags & FC_MSGBUF_ACK) {
			reg |= cfg->cyc << FC_BUF_INFO_CYC_SHIFT;
			reg |= cfg->channel << FC_BUF_INFO_CHANNEL_SHIFT;
			reg |= cfg->frame_id;

			if (cfg->flags & FC_MSGBUF_ACK_PAYLOAD)
				reg |= FC_BUF_INFO_ENABLE_PAYLOAD;
			if (cfg->flags & FC_MSGBUF_ACK_NULL)
				reg |= FC_BUF_INFO_ENABLE_NULLFRAMES;
			if (cfg->type == eray_msgbuf_type_tx)
				reg |= FC_BUF_INFO_IS_TX;
		}

		eray_writel(reg, cc, FC_BUFFER_INFO_TABLE + i*4);
	}

	/* To setup E-Ray to send startup/sync frames the appropriate
	 * bits in the SUCC1 must be set. To carry out the configuration
	 * a ALLOW_COLDSTART command must be executed.
	 */
	if ((cc->sync_start & ERAY_MSGBUF_STARTUP) ||
	    (cc->sync_start & ERAY_MSGBUF_SYNC)) {
		reg = eray_read_succ1(cc, 10);
		if (reg < 0) {
			ret = -EBUSY;
			goto out;
		}

		if (cc->sync_start & ERAY_MSGBUF_STARTUP)
			reg |= ERAY_SUCC1_TXST_MASK;

		if (cc->sync_start & ERAY_MSGBUF_SYNC)
			reg |= ERAY_SUCC1_TXSY_MASK;

		reg &= ~ERAY_SUCC1_CMD_MASK;
		reg |= ERAY_CMD_ALLOW_COLDSTART;

		eray_writel(reg, cc, ERAY_SUCC1);
	}

	cc->ready = 1;

	for (i = 0; i < cc->act_cfg; i++) {
		cfg = &cc->cfg[i];

		if ((cfg->type == eray_msgbuf_type_tx) &&
		    cfg->flags & ERAY_MSGBUF_TXCONT) {
			ret = _fc_write_data(priv, i, cfg->tx_cont_data,
					     cfg->tx_cont_len);
			if (ret)
				goto out;
		}
	}

	ret = fc_prepare_msgbuf_data_ssync(dev);
	if (ret)
		goto out;

	ret = 0;
out:
	spin_unlock_irqrestore(&cc->lock, flags);

	return ret;
}

static int fc_write_data(struct flexcard_priv *priv, unsigned int msgbuf_id,
			 unsigned char *payload, size_t byte_len)
{
	struct eray_cc *cc = priv->cc;
	int ret;

	spin_lock(&cc->lock);
	ret = _fc_write_data(priv, msgbuf_id, payload, byte_len);
	spin_unlock(&cc->lock);

	return ret;
}

static int _fc_write_data(struct flexcard_priv *priv, unsigned int msgbuf_id,
			  unsigned char *payload, size_t byte_len)
{
	struct eray_cc *cc = priv->cc;
	struct eray_msgbuf_cfg *cfg;
	u32 txrq, *data = (u32 *) payload;
	int i, count, rem, err;
	u32 offset = 0;
	int ssync;

	ssync = (msgbuf_id & FC_FLEX_ID_SSYNC_FLAG) ? 1 : 0;
	msgbuf_id &= ~(FC_FLEX_ID_SSYNC_FLAG);
	if (ssync) {
		offset = FC_SSYNC_OFFSET;
		cfg = &cc->ssync_cfg[msgbuf_id];
	} else
		cfg = &cc->cfg[msgbuf_id];

	if (cfg->flags & ERAY_MSGBUF_TXCONT) {
		if (byte_len > 256)
			byte_len = 256;
		cfg->tx_cont_len = byte_len;

		memcpy(cfg->tx_cont_data, payload, cfg->tx_cont_len);
	}

	if (!cc->ready) {
		/* Discard packets if CC is not ready. If we return an error
		 * here we never make progress and recover from this state.
		 */
		err = 0;
		goto out;
	}

	/* ignore TXRQ bit in continous mode since it is allways set */
	if (!(cfg->flags & ERAY_MSGBUF_TXCONT)) {
		txrq = eray_readl(cc, (ERAY_TXRQ1 + cfg->id/32) + offset);
		if (txrq & 1 << (cfg->id%32)) {
			err = -EBUSY;
			goto out;
		}
	}

	err = fc_wait_for_ram_offset(cc, offset);
	if (err)
		goto out;

	/* check packet length to decide if message buffer configuration
	 * needs to be reprogrammed.
	 */
	if (((byte_len + 1) / 2) != cfg->len) {
		u32 reg;

		/* setup wrhs2 again */
		cfg->len  = (byte_len + 1) / 2;

		if (cfg->frame_id < cc->static_id)
			reg = cfg->wrhs2 & ERAY_WRHS2_CRC_MASK;
		else
			reg = crc_header(cfg) & ERAY_WRHS2_CRC_MASK;
		reg |= (cfg->len << ERAY_WRHS2_PLC_SHIFT) &
			ERAY_WRHS2_PLC_MASK;
		cfg->wrhs2 = reg;

		/* write msgbuf config */
		eray_writel(cfg->wrhs1, cc, ERAY_WRHS1 + offset);
		eray_writel(cfg->wrhs2, cc, ERAY_WRHS2 + offset);
		eray_writel(cfg->wrhs3, cc, ERAY_WRHS3 + offset);

		eray_writel(ERAY_IBCM_LHSH_MASK, cc, ERAY_IBCM + offset);
		eray_writel(cfg->id & ERAY_IBCR_IBRH_MASK, cc,
				ERAY_IBCR + offset);
		err = fc_wait_for_ram_offset(cc, offset);
		if (err)
			goto out;

	}

	count = byte_len >> 2;
	rem = byte_len & 0x3;

	/* write 32-bit data words */
	for (i = 0; i < count; i++)
		eray_writel(cpu_to_le32(data[i]), cc, ERAY_WRDS(i) + offset);

	/* write remaining data bytes */
	if (rem) {
		u32 wrd = 0;
		memcpy(&wrd, &payload[byte_len - rem], rem);
		eray_writel(cpu_to_le32(wrd), cc, ERAY_WRDS(i) + offset);
	}

	eray_writel(ERAY_IBCM_LDSH_MASK | ERAY_IBCM_STXRH_MASK, cc,
			ERAY_IBCM + offset);
	eray_writel(cfg->id & ERAY_IBCR_IBRH_MASK, cc, ERAY_IBCR + offset);

	err = fc_wait_for_ram_offset(cc, offset);
	if (!err) {
		spin_lock(&cfg->lock);
		cfg->queued = 0;
		spin_unlock(&cfg->lock);
	}
out:
	return err;
}

static enum eray_cc_state e_state(enum flexray_state state)
{
	switch (state) {
	case FLEXRAY_STATE_DEFAULT_CONFIG:
		return ERAY_CMD_FREEZE;
	case FLEXRAY_STATE_CONFIG:
		return ERAY_CMD_CONFIG;
	case FLEXRAY_STATE_READY:
		return ERAY_CMD_READY;
	case FLEXRAY_STATE_WAKEUP:
		return ERAY_CMD_WAKEUP;
	case FLEXRAY_STATE_STARTUP:
		return ERAY_CMD_RUN;
	case FLEXRAY_STATE_HALT:
		return ERAY_CMD_HALT;
	case FLEXRAY_STATE_MONITOR_MODE:
		return ERAY_CMD_MONITOR_MODE;
	case FLEXRAY_STATE_COLDSTART:
		return ERAY_CMD_ALLOW_COLDSTART;
	case FLEXRAY_STATE_NORMAL_ACTIVE:
	case FLEXRAY_STATE_NORMAL_PASSIVE:
	case FLEXRAY_STATE_MAX:
	case FLEXRAY_STATE_UNSPEC:
	default:
		return ERAY_CMD_INVALID;
	}
}

static int cc_change_state(struct eray_cc *cc, enum eray_cc_state state,
			   int retry)
{
	u32 stat;

	stat = eray_read_succ1(cc, 10);
	if (stat < 0)
		return -EBUSY;

	stat &= ~ERAY_SUCC1_CMD_MASK;
	stat |= state;

	if (state == ERAY_CMD_READY || state == ERAY_CMD_MONITOR_MODE) {
		eray_writel(0xCE, cc, ERAY_LCK);
		eray_writel(0x31, cc, ERAY_LCK);
	}

	eray_writel(stat, cc, ERAY_SUCC1);

	stat = eray_read_succ1(cc, retry);
	if (!(stat & ERAY_SUCC1_CMD_MASK))
		return -EINVAL;

	if (state == ERAY_CMD_FREEZE) {
		stat &= ~ERAY_SUCC1_CMD_MASK;
		stat |= ERAY_CMD_CONFIG;
		eray_writel(stat, cc, ERAY_SUCC1);

		stat = eray_read_succ1(cc, retry);
		if (!(stat & ERAY_SUCC1_CMD_MASK))
			return -EINVAL;
	}

	return 0;
}

static int cc_reset(struct net_device *dev)
{
	struct flexcard_priv *priv = netdev_priv(dev);
	struct eray_cc *cc = priv->cc;
	int ret;

	/* Set FR CCs to a well-defined state,
	 * because the cc reset doesn't work always
	 * 1. send freeze command
	 * 2. wait for halt state
	 * 3. send config command
	 * 4. wait for default config state
	 * 5. send clear_rams command
	 * 6. wait max. 150 us for end of ram initialization
	 * (calculated time 62 us)
	 * 7. Flexcard reset (50us settle time)
	 * 8 Flexcard Filter reset (70us settle time)
	 */

	ret = cc_change_state(cc, ERAY_CMD_FREEZE, 5);
	if (ret < 0) {
		dev_err(&dev->dev, "CC FREEZE failed\n");
		goto out;
	}

	ret = cc_change_state(cc, ERAY_CMD_CONFIG, 5);
	if (ret < 0) {
		dev_err(&dev->dev, "CC DEFAULT_CONFIG failed\n");
		goto out;
	}

	ret = cc_change_state(cc, ERAY_CMD_CLEAR_RAMS, 15);
	if (ret < 0) {
		netdev_err(dev, "%s: CC CLEAR_RAMS failed\n", __func__);
		goto out;
	}

	writel(1<<priv->id, priv->conf + FC_FC_RESET);
	udelay(50);

	eray_writel(0, cc, FC_RXFILTID);
	eray_writel(3, cc, FC_RXFILTCH);
	eray_writel(0, cc, FC_TXFILTID);
	eray_writel(0, cc, FC_TXFILTCH);
	udelay(70);

	ret = cc_change_state(cc, ERAY_CMD_CONFIG, 5);
	if (ret < 0) {
		dev_err(&dev->dev, "CC CONFIG failed\n");
		goto out;
	}

	eray_writel(0, cc, ERAY_MHDC);
	eray_writel(0, cc, ERAY_GTUC7);

out:
	return ret;
}

static void cc_get_conf(struct net_device *dev)
{
	struct flexcard_priv *priv = netdev_priv(dev);
	struct eray_cc *cc = priv->cc;
	struct flexray_priv *flexray = &priv->flexray;
	u8 BRP, EOCC, ERCC;

	eray_get_val16(&cc->static_id, cc,
		      ERAY_GTUC7, ERAY_GTUC7_NSS_MASK, ERAY_GTUC7_NSS_SHIFT);

	eray_get_val8(&cc->static_len, cc,
		      ERAY_MHDC, ERAY_MHDC_SFDL_MASK, ERAY_MHDC_SFDL_SHIFT);

	/* cluster */
	eray_get_val8(&flexray->cluster.gColdstartAttempts, cc,
		      ERAY_SUCC1, ERAY_SUCC1_CSA_MASK, ERAY_SUCC1_CSA_SHIFT);
	eray_get_val8(&flexray->cluster.gdCASRxLowMax, cc,
		      ERAY_PRTC1, ERAY_PRTC1_CASM_MASK, ERAY_PRTC1_CASM_SHIFT);
	eray_get_val8(&flexray->cluster.gdTSSTransmitter, cc,
		      ERAY_PRTC1, ERAY_PRTC1_TSST_MASK, ERAY_PRTC1_TSST_SHIFT);
	eray_get_val8(&flexray->cluster.gListenNoise, cc,
		      ERAY_SUCC2, ERAY_SUCC2_LTN_MASK, ERAY_SUCC2_LTN_SHIFT);
	flexray->cluster.gListenNoise += 1;
	eray_get_val8(&flexray->cluster.gMaxWithoutClockCorrectionFatal, cc,
		      ERAY_SUCC3, ERAY_SUCC3_WCF_MASK, ERAY_SUCC3_WCF_SHIFT);
	eray_get_val8(&flexray->cluster.gMaxWithoutClockCorrectionPassive, cc,
		      ERAY_SUCC3, ERAY_SUCC3_WCP_MASK, ERAY_SUCC3_WCP_SHIFT);
	eray_get_val8(&BRP, cc,
		      ERAY_PRTC1, ERAY_PRTC1_BRP_MASK, ERAY_PRTC1_BRP_SHIFT);
	switch (BRP) {
	case 0:
		flexray->cluster.gdSampleClockPeriod = 1;
		flexray->node.pSamplesPerMicrotick = 2;
		break;
	case 1:
		flexray->cluster.gdSampleClockPeriod = 2;
		flexray->node.pSamplesPerMicrotick = 1;
		break;
	case 2:
	case 3:
		flexray->cluster.gdSampleClockPeriod = 4;
		flexray->node.pSamplesPerMicrotick = 1;
		break;
	}
	eray_get_val8(&flexray->cluster.gNetworkManagementVectorLength, cc,
		      ERAY_NEMC, ERAY_NEMC_NML_MASK, ERAY_NEMC_NML_SHIFT);
	eray_get_val16(&flexray->cluster.v2.gdWakeupSymbolRxWindow, cc,
		       ERAY_PRTC1, ERAY_PRTC1_RXW_MASK, ERAY_PRTC1_RXW_SHIFT);
	eray_get_val8(&flexray->cluster.v2.gdWakeupSymbolRxIdle, cc,
		      ERAY_PRTC2, ERAY_PRTC2_RXI_MASK, ERAY_PRTC2_RXI_SHIFT);
	eray_get_val8(&flexray->cluster.v2.gdWakeupSymbolRxLow, cc,
		      ERAY_PRTC2, ERAY_PRTC2_RXL_MASK, ERAY_PRTC2_RXL_SHIFT);
	eray_get_val8(&flexray->cluster.v2.gdWakeupSymbolTxIdle, cc,
		      ERAY_PRTC2, ERAY_PRTC2_TXI_MASK, ERAY_PRTC2_TXI_SHIFT);
	eray_get_val8(&flexray->cluster.v2.gdWakeupSymbolTxLow, cc,
		      ERAY_PRTC2, ERAY_PRTC2_TXL_MASK, ERAY_PRTC2_TXL_SHIFT);
	eray_get_val8(&flexray->cluster.gPayloadLengthStatic, cc,
		      ERAY_MHDC, ERAY_MHDC_SFDL_MASK, ERAY_MHDC_SFDL_SHIFT);
	eray_get_val16(&flexray->cluster.gMacroPerCycle, cc,
		       ERAY_GTUC2, ERAY_GTUC2_MPC_MASK, ERAY_GTUC2_MPC_SHIFT);
	eray_get_val8(&flexray->cluster.v2.gSyncNodeMax, cc,
		      ERAY_GTUC2, ERAY_GTUC2_SNM_MASK, ERAY_GTUC2_SNM_SHIFT);
	eray_get_val16(&flexray->cluster.gdNIT, cc,
		      ERAY_GTUC4, ERAY_GTUC4_NIT_MASK, ERAY_GTUC4_NIT_SHIFT);
	flexray->cluster.gdNIT = flexray->cluster.gMacroPerCycle - 1 -
		flexray->cluster.gdNIT;
	eray_get_val16(&flexray->cluster.v2.gOffsetCorrectionStart, cc,
		       ERAY_GTUC4, ERAY_GTUC4_OCS_MASK, ERAY_GTUC4_OCS_SHIFT);
	flexray->cluster.v2.gOffsetCorrectionStart += 1;
	eray_get_val16(&flexray->cluster.gdStaticSlot, cc,
		       ERAY_GTUC7, ERAY_GTUC7_SSL_MASK, ERAY_GTUC7_SSL_SHIFT);
	eray_get_val16(&flexray->cluster.gNumberOfStaticSlots, cc,
		       ERAY_GTUC7, ERAY_GTUC7_NSS_MASK, ERAY_GTUC7_NSS_SHIFT);
	eray_get_val8(&flexray->cluster.gdMinislot, cc,
		      ERAY_GTUC8, ERAY_GTUC8_MSL_MASK, ERAY_GTUC8_MSL_SHIFT);
	eray_get_val16(&flexray->cluster.gNumberOfMinislots, cc,
		       ERAY_GTUC8, ERAY_GTUC8_NMS_MASK, ERAY_GTUC8_NMS_SHIFT);
	eray_get_val8(&flexray->cluster.gdActionPointOffset, cc,
		      ERAY_GTUC9, ERAY_GTUC9_APO_MASK, ERAY_GTUC9_APO_SHIFT);
	eray_get_val8(&flexray->cluster.gdMinislotActionPointOffset, cc,
		      ERAY_GTUC9, ERAY_GTUC9_MAPO_MASK, ERAY_GTUC9_MAPO_SHIFT);
	eray_get_val8(&flexray->cluster.gdDynamicSlotIdlePhase, cc,
		     ERAY_GTUC9, ERAY_GTUC9_DSI_MASK, ERAY_GTUC9_DSI_SHIFT);

	/* node */
	eray_get_val8(&flexray->node.pAllowHaltDueToClock, cc,
		      ERAY_SUCC1, ERAY_SUCC1_HCSE_MASK, ERAY_SUCC1_HCSE_SHIFT);
	eray_get_val8(&flexray->node.pAllowPassiveToActive, cc,
		      ERAY_SUCC1, ERAY_SUCC1_PTA_MASK, ERAY_SUCC1_PTA_SHIFT);
	eray_get_val8(&flexray->node.pChannels, cc,
		      ERAY_SUCC1, ERAY_SUCC1_CCH_MASK, ERAY_SUCC1_CCH_SHIFT);
	eray_get_val32(&flexray->node.pdListenTimeout, cc,
		       ERAY_SUCC2, ERAY_SUCC2_LT_MASK, ERAY_SUCC2_LT_SHIFT);
	eray_get_val8(&flexray->node.v2.pSingleSlotEnabled, cc,
		      ERAY_SUCC1, ERAY_SUCC1_TSM_MASK, ERAY_SUCC1_TSM_SHIFT);
	eray_get_val8(&flexray->node.pKeySlotUsedForStartup, cc,
		      ERAY_SUCC1, ERAY_SUCC1_TXST_MASK, ERAY_SUCC1_TXST_SHIFT);
	eray_get_val8(&flexray->node.pKeySlotUsedForSync, cc,
		      ERAY_SUCC1, ERAY_SUCC1_TXSY_MASK, ERAY_SUCC1_TXSY_SHIFT);
	eray_get_val8(&flexray->node.pWakeupChannel, cc,
		      ERAY_SUCC1, ERAY_SUCC1_WUCS_MASK, ERAY_SUCC1_WUCS_SHIFT);
	eray_get_val8(&flexray->node.pWakeupPattern, cc,
		      ERAY_PRTC1, ERAY_PRTC1_RWP_MASK, ERAY_PRTC1_RWP_SHIFT);
	eray_get_val16(&flexray->node.pLatestTx, cc,
		       ERAY_MHDC, ERAY_MHDC_SLT_MASK, ERAY_MHDC_SLT_SHIFT);
	eray_get_val32(&flexray->node.pMicroPerCycle, cc,
		       ERAY_GTUC1, ERAY_GTUC1_UT_MASK, ERAY_GTUC1_UT_SHIFT);
	eray_get_val16(&flexray->node.pMicroInitialOffsetA, cc,
		       ERAY_GTUC3, ERAY_GTUC3_UIOA_MASK, ERAY_GTUC3_UIOA_SHIFT);
	eray_get_val16(&flexray->node.pMicroInitialOffsetB, cc,
		       ERAY_GTUC3, ERAY_GTUC3_UIOB_MASK, ERAY_GTUC3_UIOB_SHIFT);
	eray_get_val8(&flexray->node.pMacroInitialOffsetA, cc,
		      ERAY_GTUC3, ERAY_GTUC3_MIOA_MASK, ERAY_GTUC3_MIOA_SHIFT);
	eray_get_val8(&flexray->node.pMacroInitialOffsetB, cc,
		      ERAY_GTUC3, ERAY_GTUC3_MIOB_MASK, ERAY_GTUC3_MIOB_SHIFT);
	eray_get_val8(&flexray->node.pDelayCompensationA, cc,
		      ERAY_GTUC5, ERAY_GTUC5_DCA_MASK, ERAY_GTUC5_DCA_SHIFT);
	eray_get_val8(&flexray->node.pDelayCompensationB, cc,
		      ERAY_GTUC5, ERAY_GTUC5_DCB_MASK, ERAY_GTUC5_DCB_SHIFT);
	eray_get_val8(&flexray->node.pClusterDriftDamping, cc,
		      ERAY_GTUC5, ERAY_GTUC5_CDD_MASK, ERAY_GTUC5_CDD_SHIFT);
	eray_get_val8(&flexray->node.pDecodingCorrection, cc,
		      ERAY_GTUC5, ERAY_GTUC5_DEC_MASK, ERAY_GTUC5_DEC_SHIFT);
	eray_get_val16(&flexray->node.pdAcceptedStartupRange, cc,
		       ERAY_GTUC6, ERAY_GTUC6_ASR_MASK, ERAY_GTUC6_ASR_SHIFT);
	eray_get_val16(&flexray->node.v2.pdMaxDrift, cc,
		       ERAY_GTUC6, ERAY_GTUC6_MOD_MASK, ERAY_GTUC6_MOD_SHIFT);
	eray_get_val16(&flexray->node.pOffsetCorrectionOut, cc, ERAY_GTUC10,
			ERAY_GTUC10_MOC_MASK, ERAY_GTUC10_MOC_SHIFT);
	eray_get_val16(&flexray->node.pRateCorrectionOut, cc, ERAY_GTUC10,
			ERAY_GTUC10_MRC_MASK, ERAY_GTUC10_MRC_SHIFT);
	eray_get_val8(&flexray->node.pExternOffsetCorrection, cc,
		      ERAY_GTUC11, ERAY_GTUC11_EOC_MASK, ERAY_GTUC11_EOC_SHIFT);
	eray_get_val8(&flexray->node.pExternRateCorrection, cc,
		      ERAY_GTUC11, ERAY_GTUC11_ERC_MASK, ERAY_GTUC11_ERC_SHIFT);
	eray_get_val8(&EOCC, cc, ERAY_GTUC11, ERAY_GTUC11_EOCC_MASK,
			ERAY_GTUC11_EOCC_SHIFT);

	/* symbol */
	eray_get_val8(&flexray->symbol.pChannelsMTS, cc,
		      ERAY_SUCC1, ERAY_SUCC1_MTS_MASK, ERAY_SUCC1_MTS_SHIFT);

	switch (EOCC) {
	case 3:
		flexray->node.vExternOffsetControl = 1;
		break;
	case 2:
		flexray->node.vExternOffsetControl = -1;
		break;
	default:
		flexray->node.vExternOffsetControl = 0;
	}
	eray_get_val8(&ERCC, cc, ERAY_GTUC11, ERAY_GTUC11_ERCC_MASK,
			ERAY_GTUC11_ERCC_SHIFT);
	switch (ERCC) {
	case 3:
		flexray->node.vExternRateControl = 1;
		break;
	case 2:
		flexray->node.vExternRateControl = -1;
		break;
	default:
		flexray->node.vExternRateControl = 0;
	}
}

static void cc_irq_setup(struct net_device *dev)
{
	struct flexcard_priv *priv = netdev_priv(dev);
	struct eray_cc *cc = priv->cc;

	/* error eray_int0 */
	eray_writel(0, cc, ERAY_EILS);

	/* status eray_int1 */
	eray_writel(ERAY_SIR_MASK, cc, ERAY_SILS);

	/* disable error interrupts */
	eray_writel(0, cc, ERAY_EIES);

	/* enable receive interrupts */
	eray_writel(ERAY_SIR_RXI, cc, ERAY_SIES);

	/* enable eray_int0 and eray_int1 line */
	eray_writel(ERAY_ILE_MASK, cc, ERAY_ILE);
}

static void cc_stopwatch_setup(struct net_device *dev, u32 reg)
{
	struct flexcard_priv *priv = netdev_priv(dev);
	struct eray_cc *cc = priv->cc;

	eray_writel(reg & 0x3f, cc, ERAY_STPW1);
}

static struct sk_buff *alloc_flexcard_skb(struct net_device *dev,
					  struct fc_packet_buf **cf,
					  size_t len)
{
	struct sk_buff *skb;

	skb = netdev_alloc_skb(dev, len);
	if (unlikely(!skb))
		return NULL;

	skb->protocol = htons(ETH_P_FLEXRAY);
	skb->pkt_type = PACKET_BROADCAST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	*cf = (struct fc_packet_buf *)skb_put(skb, len);

	return skb;
}

static int fc_rx_pkt(void *p, void *data, size_t len)
{
	struct net_device *dev = p;
	struct net_device_stats *stats = &dev->stats;
	struct flexcard_priv *priv = netdev_priv(dev);
	struct eray_cc *cc = priv->cc;
	struct fc_packet_buf *pb = data;
	union fc_packet_types *pt = &pb->packet;
	struct fc_packet_buf *frf;
	struct sk_buff *skb;
	u32 l;

	switch (le32_to_cpu(pb->header.type)) {
	case fc_packet_type_flexray_frame:
		l = fc_get_packet_len(pt->flexray_frame.header);
		break;
	case fc_packet_type_tx_ack:
		/* The buffer id is visible to userspace. Msg. buffer in
		 * kernel starts with 0, in oposite to userspace starting
		 * with 1. Add 1 to buffer id make it even.
		 */
		pt->tx_ack_packet.bufferid =
			cc->rev_id[pt->tx_ack_packet.bufferid] + 1;
		l = fc_get_packet_len(pt->tx_ack_packet.header);
		break;
	default:
		l = 0;
	}

	if (len > sizeof(struct fc_packet_buf) + l) {
		len = sizeof(struct fc_packet_buf) + l;
		WARN(1, "FlexRay payload to large: truncate.");
	}

	skb = alloc_flexcard_skb(dev, &frf, len);
	if (!skb)
		return -ENOMEM;

	memcpy(frf, data, len);
	netif_receive_skb(skb);

	stats->rx_packets++;
	stats->rx_bytes += len;

	return 0;
}

static int flexcard_open(struct net_device *dev)
{
	int ret = -ENOMEM;

	ret = cc_reset(dev);
	if (ret < 0) {
		netdev_err(dev, "CC reset failed\n");
		goto out;
	}

	cc_stopwatch_setup(dev, 0);

	ret = open_flexraydev(dev);
	if (ret) {
		netdev_err(dev, "open flexray device failed\n");
		goto out;
	}

	cc_irq_setup(dev);

	return 0;

out:
	return ret;
}

static int flexcard_close(struct net_device *dev)
{
	struct flexcard_priv *priv = netdev_priv(dev);
	struct eray_cc *cc = priv->cc;

	cc_change_state(cc, ERAY_CMD_FREEZE, 10);
	netif_stop_queue(dev);

	close_flexraydev(dev);

	return 0;
}

static int flexcard_get_state(const struct net_device *dev,
			       enum flexray_state *state)
{
	struct flexcard_priv *priv = netdev_priv(dev);
	struct eray_cc *cc = priv->cc;
	int pocs;

	pocs = eray_readl(cc, ERAY_CCSV) & 0x3f;
	switch (pocs) {
	case 0x00:
		*state = FLEXRAY_STATE_DEFAULT_CONFIG;
		break;
	case 0x01:
		*state = FLEXRAY_STATE_READY;
		break;
	case 0x02:
		*state = FLEXRAY_STATE_NORMAL_ACTIVE;
		break;
	case 0x03:
		*state = FLEXRAY_STATE_NORMAL_PASSIVE;
		break;
	case 0x04:
		*state = FLEXRAY_STATE_HALT;
		break;
	case 0x05:
		*state = FLEXRAY_STATE_MONITOR_MODE;
		break;
	case 0x0f:
		*state = FLEXRAY_STATE_CONFIG;
		break;
	case 0x10:
	case 0x11:
	case 0x12:
	case 0x13:
		*state = FLEXRAY_STATE_WAKEUP;
		break;
	case 0x20:
	case 0x21:
	case 0x22:
	case 0x24:
	case 0x25:
	case 0x26:
	case 0x27:
	case 0x28:
	case 0x29:
	case 0x2a:
	case 0x2b:
		*state = FLEXRAY_STATE_STARTUP;
		break;
	default:
		*state = FLEXRAY_STATE_UNSPEC;
	}

	return 0;
}

static int flexcard_set_state(struct net_device *dev,
			      enum flexray_state state)
{
	struct flexcard_priv *priv = netdev_priv(dev);
	struct eray_cc *cc = priv->cc;
	int ret;

	switch (state) {
	case FLEXRAY_STATE_READY:
		ret = fc_prepare_msgbuf_data(dev);
		if (ret)
			return ret;
	case FLEXRAY_STATE_WAKEUP:
	case FLEXRAY_STATE_STARTUP:
	case FLEXRAY_STATE_NORMAL_ACTIVE:
	case FLEXRAY_STATE_NORMAL_PASSIVE:
	case FLEXRAY_STATE_MONITOR_MODE:
	case FLEXRAY_STATE_COLDSTART:
		netif_start_queue(dev);
		break;
	default:
		cc->ready = 0;
	}

	return cc_change_state(cc, e_state(state), 10);
}

static int flexcard_validate(struct sk_buff *skb)
{
	struct fc_msg *fcm = (struct fc_msg *)skb->data;
	struct eray_msgbuf_cfg *cfg;
	struct flexcard_priv *priv = netdev_priv(skb->dev);
	struct eray_cc *cc = priv->cc;
	unsigned long flags;
	u32 txrq;
	int ret = 0;
	u32 buf_id;
	int ssync;

	ssync = (fcm->buf_id & FC_FLEX_ID_SSYNC_FLAG) ? 1 : 0;
	buf_id = fcm->buf_id & ~(FC_FLEX_ID_SSYNC_FLAG);

	if (ssync) {
		if (buf_id >= cc->ssync_num)
			return -EFAULT;
		cfg = &cc->ssync_cfg[buf_id];
	} else {
		if (buf_id >= cc->act_cfg)
			return -EFAULT;
		cfg = &cc->cfg[buf_id];
	}

	if (!(cfg->flags & ERAY_MSGBUF_USED))
		return -EFAULT;

	if (cfg->type != eray_msgbuf_type_tx)
		return -EINVAL;

	if (cfg->channel == eray_msgbuf_ch_none)
		return -EINVAL;

	if (skb->len > (2 * cfg->max + 4))
		return -EINVAL;

	/* queue frame, if message buffer is in continuos mode */
	if (cfg->flags & ERAY_MSGBUF_TXCONT)
		return 0;

	if ((cfg->frame_id <= cc->static_id) &&
	    ((skb->len - 4) / 2 != cc->static_len))
		return -EINVAL;

	if (!cc->ready)
		return -EBUSY;

	spin_lock_irqsave(&cfg->lock, flags);
	if (!ssync) {
		txrq = eray_readl(cc, ERAY_TXRQ1 + cfg->id/32);

		if (txrq & (1 << (cfg->id%32)))
			ret = -ENOBUFS;
	}

	if (cfg->queued)
		ret = -ENOBUFS;

	if (!ret)
		cfg->queued = 1;

	spin_unlock_irqrestore(&cfg->lock, flags);

	return ret;
}

static int flexcard_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct net_device_stats *stats = &dev->stats;
	struct flexcard_priv *priv = netdev_priv(dev);
	struct fc_msg *fcm = (struct fc_msg *)skb->data;

	if (fc_write_data(priv, fcm->buf_id, fcm->data,
			  skb->len - sizeof(fcm->buf_id))) {
		net_info_ratelimited("%s() fc_write_data() failed.\n",
				dev->name);
		return NETDEV_TX_BUSY;
	}

	stats->tx_bytes += skb->len - sizeof(fcm->buf_id);

	kfree_skb(skb);

	return NETDEV_TX_OK;
}

static const struct net_device_ops flexcard_netdev_ops = {
	.ndo_open       = flexcard_open,
	.ndo_stop       = flexcard_close,
	.ndo_start_xmit = flexcard_start_xmit,
};

static ssize_t fw_ver_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct flexcard_priv *priv = netdev_priv(ndev);
	struct eray_cc *cc = priv->cc;
	u32 fw_ver;

	fw_ver = eray_readl(cc, ERAY_CREL);

	return sprintf(buf, "%x.%02x %02x.%02x.200%x\n",
		       fw_ver >> 28 & 0xf, fw_ver >> 20 & 0xff,
		       fw_ver & 0xff, fw_ver >> 8 & 0xff, fw_ver >> 16 & 0xf);
}

static ssize_t succ1_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct flexcard_priv *priv = netdev_priv(ndev);
	struct eray_cc *cc = priv->cc;
	u32 reg;

	reg = eray_read_succ1(cc, 20);

	return sprintf(buf, "0x%08x\n", reg);
}

#define FC_SHOW_REG(x, y)						\
	static ssize_t x##_show(struct device *dev,			\
				struct device_attribute *attr,		\
				char *buf)				\
	{								\
		struct platform_device *pdev = to_platform_device(dev);	\
		struct net_device *ndev = platform_get_drvdata(pdev);	\
		struct flexcard_priv *priv = netdev_priv(ndev);		\
		struct eray_cc *cc = priv->cc;				\
		u32 reg;						\
		reg = eray_readl(cc, y);				\
									\
		return sprintf(buf, "0x%08x\n", reg);			\
	}

#define FC_SHOW_REGS(x, y) \
	static ssize_t x##_show(struct device *dev,			\
				struct device_attribute *attr,		\
				char *buf)				\
	{								\
		struct platform_device *pdev = to_platform_device(dev);	\
		struct net_device *ndev = platform_get_drvdata(pdev);	\
		struct flexcard_priv *priv = netdev_priv(ndev);		\
		struct eray_cc *cc = priv->cc;				\
		u32 reg[4];						\
									\
		reg[0] = eray_readl(cc, y);				\
		reg[1] = eray_readl(cc, y + 4);				\
		reg[2] = eray_readl(cc, y + 8);				\
		reg[3] = eray_readl(cc, y + 12);			\
									\
		return sprintf(buf, "0x%08x%08x%08x%08x\n",		\
			       reg[0], reg[1], reg[2], reg[3]);		\
	}

FC_SHOW_REG(ccsv, ERAY_CCSV);
FC_SHOW_REG(eir, ERAY_EIR);
FC_SHOW_REG(sir, ERAY_SIR);
FC_SHOW_REGS(txrq, ERAY_TXRQ1);
FC_SHOW_REGS(ndat, ERAY_NDAT1);
FC_SHOW_REGS(mbsc, ERAY_MBSC1);

static struct device_attribute flex_dev_attrs[] = {
	__ATTR_RO(fw_ver),
	__ATTR_RO(succ1),
	__ATTR_RO(ccsv),
	__ATTR_RO(eir),
	__ATTR_RO(sir),
	__ATTR_RO(txrq),
	__ATTR_RO(ndat),
	__ATTR_RO(mbsc),
	__ATTR_NULL,
};

static int device_add_attributes(struct device *dev,
				 struct device_attribute *attrs)
{
	int error = 0;
	int i;

	for (i = 0; attr_name(attrs[i]); i++) {
		error = device_create_file(dev, &attrs[i]);
		if (error)
			break;
	}
	if (error)
		while (--i >= 0)
			device_remove_file(dev, &attrs[i]);
	return error;
}

static void device_remove_attributes(struct device *dev,
				     struct device_attribute *attrs)
{
	int i;

	for (i = 0; attr_name(attrs[i]); i++)
		device_remove_file(dev, &attrs[i]);
}

static int flexcard_probe(struct platform_device *pdev)
{
	struct eray_msgbuf_cfg *cfg;
	struct flexcard_priv *priv;
	struct eray_cc *cc;
	struct net_device *dev;
	struct resource *res, *res_conf;
	int i, irq, ret = -ENXIO;
	u32 fw_ver;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "failed to get IRQ number\n");
		goto out;
	}

	res_conf = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res_conf) {
		dev_err(&pdev->dev, "failed to get conf I/O memory\n");
		goto out;
	}
	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (!res) {
		dev_err(&pdev->dev, "failed to get mmio I/O memory\n");
		goto out;
	}

	cc = kzalloc(sizeof(*cc), GFP_KERNEL);
	if (!cc) {
		dev_err(&pdev->dev, "failed to alloc memory\n");
		goto out_release;
	}

	spin_lock_init(&cc->lock);
	for (i = 0; i < ERAY_MAX_BUFS; i++)
		spin_lock_init(&cc->cfg[i].lock);

	cc->fifo_threshold = ERAY_FIFO_THRESHOLD;

	dev = alloc_flexraydev(sizeof(struct flexcard_priv), 2);
	if (!dev) {
		dev_err(&pdev->dev, "failed to alloc netdevice\n");
		goto out_free;
	}

	dev->netdev_ops = &flexcard_netdev_ops;
	dev->irq = irq;
	dev->flags |= IFF_ECHO;

	priv = netdev_priv(dev);
	priv->cc = cc;
	priv->dev = dev;
	priv->id = pdev->id;
	priv->flexray.do_get_state = flexcard_get_state;
	priv->flexray.do_set_state = flexcard_set_state;
	priv->flexray.do_validate = flexcard_validate;

	cfg = &cc->cfg[0];
	cfg->flags |= ERAY_MSGBUF_USED;
	cfg->type = eray_msgbuf_type_fifo;
	cfg->max = 127;

	dev_set_drvdata(&pdev->dev, dev);
	SET_NETDEV_DEV(dev, &pdev->dev);

	priv->conf = ioremap_nocache(res_conf->start, resource_size(res_conf));
	if (!priv->conf) {
		dev_err(&pdev->dev, "failed to remap conf I/O memory\n");
		goto out_free_dev;
	}

	cc->base = ioremap_nocache(res->start, resource_size(res));
	if (!cc->base) {
		dev_err(&pdev->dev, "failed to remap mmio I/O memory\n");
		goto out_unmap_conf;
	}

	ret = register_flexraydev(dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register netdevice\n");
		goto out_unmap_mmio;
	}

	ret = device_add_attributes(&pdev->dev, flex_dev_attrs);
	if (ret)
		goto out_unregister;

	ret = fc_register_rx_pkt(priv->id, dev, fc_rx_pkt);
	if (ret) {
		netdev_err(dev, "register RX DMA callback failed: %d\n", ret);
		goto out_add_attr;
	}

	fw_ver = eray_readl(cc, ERAY_CREL);
	dev_info(&pdev->dev, "E-Ray FW ver.%x.%02x %02x.%02x.200%x\n",
		 fw_ver >> 28 & 0xf, fw_ver >> 20 & 0xff,
		 fw_ver & 0xff, fw_ver >> 8 & 0xff, fw_ver >> 16 & 0xf);

	return 0;
out_add_attr:
	device_remove_attributes(&pdev->dev, flex_dev_attrs);
out_unregister:
	unregister_flexraydev(dev);
out_unmap_mmio:
	iounmap(cc->base);
out_unmap_conf:
	iounmap(priv->conf);
out_free_dev:
	free_flexraydev(dev);
out_free:
	kfree(cc);
out_release:
	release_mem_region(res->start, resource_size(res));
out:
	return ret;
}

static int flexcard_remove(struct platform_device *pdev)
{
	struct net_device *dev = platform_get_drvdata(pdev);
	struct flexcard_priv *priv = netdev_priv(dev);
	struct eray_cc *cc = priv->cc;

	fc_unregister_rx_pkt(priv->id);
	device_remove_attributes(&pdev->dev, flex_dev_attrs);

	unregister_flexraydev(dev);
	free_flexraydev(dev);
	iounmap(cc->base);
	iounmap(priv->conf);
	kfree(cc);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static struct platform_driver flexcard_driver = {
	.probe = flexcard_probe,
	.remove = flexcard_remove,
	.driver = {
		.name = "flexcard-eray",
		.owner = THIS_MODULE,
	},
};
MODULE_ALIAS("platform:flexcard-eray");

static int __init flexcard_init(void)
{
	int ret;

	ret = genl_register_family_with_ops(&fc_msgbuf_genl_family,
					    fc_msgbuf_genl_ops,
					    ARRAY_SIZE(fc_msgbuf_genl_ops));
	if (ret) {
		pr_err("flexcard: register genl failed %d\n", ret);
		goto out;
	}

	ret = platform_driver_register(&flexcard_driver);
out:
	return ret;
}
module_init(flexcard_init);

static void __exit flexcard_exit(void)
{
	genl_unregister_family(&fc_msgbuf_genl_family);
	platform_driver_unregister(&flexcard_driver);
}
module_exit(flexcard_exit);

MODULE_AUTHOR("Benedikt Spranger <b.spranger@linutronix.de>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Eberspächer Flexcard FlexRay driver");
