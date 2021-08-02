/* Copyright 2012 Eberspächer Electronics GmbH & Co. KG. All Rights Reserved.
 *
 * valid flexray paramter ranges are from
 * FlexRay - Protocol Specification - V2.1rev A
 * FlexRay - Protocol Specification - V3.0.1
 * eray-specific: Bosch E-Ray FlexRay IP-Module, User's Manual,
 *		  Revision 1.2.7 (06.02.2009)
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/flexray.h>
#include <linux/flexray/dev.h>
#include <linux/flexray/netlink.h>
#include <net/rtnetlink.h>

static int flexray_check_cluster_params(struct flexray_cluster_param *cp,
					struct net_device *dev);
static int flexray_check_node_params(struct flexray_node_param *np,
				     struct net_device *dev);

static void flexray_setup(struct net_device *dev)
{
	dev->type = ARPHRD_FLEXRAY;
	dev->mtu = sizeof(struct flexray_frame);
	dev->hard_header_len = 0;
	dev->addr_len = 0;
	dev->tx_queue_len = 10;
	dev->flags = IFF_NOARP;
	dev->features = NETIF_F_NO_CSUM;
}

struct sk_buff *alloc_flexray_skb(struct net_device *dev,
				  struct flexray_frame **cf)
{
	struct sk_buff *skb;

	skb = netdev_alloc_skb(dev, sizeof(struct flexray_frame));
	if (unlikely(!skb))
		return NULL;

	skb->protocol = htons(ETH_P_FLEXRAY);
	skb->pkt_type = PACKET_BROADCAST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	*cf = (struct flexray_frame *)skb_put(skb,
					      sizeof(struct flexray_frame));
	memset(*cf, 0, sizeof(struct flexray_frame));

	return skb;
}
EXPORT_SYMBOL_GPL(alloc_flexray_skb);

/* Allocate and setup space for the FLEXRAY network device */
struct net_device *alloc_flexraydev(int sizeof_priv, u8 version)
{
	struct net_device *dev;
	struct flexray_priv *priv;

	if (version != 2 && version != 3)
		return NULL;

	/* 3 TXs queues: Key, Fixed, Dynamic */
	dev = alloc_netdev_mqs(sizeof_priv, "flexray%d", flexray_setup, 3, 1);
	if (!dev)
		return NULL;

	priv = netdev_priv(dev);
	priv->state = FLEXRAY_STATE_UNSPEC;
	priv->version = version;

	return dev;
}
EXPORT_SYMBOL_GPL(alloc_flexraydev);

/* Free space of the FLEXRAY network device */
void free_flexraydev(struct net_device *dev)
{
	free_netdev(dev);
}
EXPORT_SYMBOL_GPL(free_flexraydev);

int open_flexraydev(struct net_device *dev)
{
	/* Switch carrier on if device was stopped while in bus-off state */
	if (!netif_carrier_ok(dev))
		netif_carrier_on(dev);

	return 0;
}
EXPORT_SYMBOL_GPL(open_flexraydev);

void close_flexraydev(struct net_device *dev)
{
}
EXPORT_SYMBOL_GPL(close_flexraydev);

/* debug functionality */

#define PRINT_PARAM(name, p)	pr_debug(#name" = %d\n", p->name);

void print_cluster_config(struct flexray_cluster_param *cp, int is_v3)
{
	pr_debug("Cluster configuration:\n");

	PRINT_PARAM(gColdstartAttempts, cp);
	PRINT_PARAM(gdActionPointOffset, cp);
	PRINT_PARAM(gdCASRxLowMax, cp);
	PRINT_PARAM(gdDynamicSlotIdlePhase, cp);
	PRINT_PARAM(gdMinislot, cp);
	PRINT_PARAM(gdMinislotActionPointOffset, cp);
	PRINT_PARAM(gdStaticSlot, cp);
	PRINT_PARAM(gdSymbolWindow, cp);
	PRINT_PARAM(gdTSSTransmitter, cp);
	PRINT_PARAM(gListenNoise, cp);
	PRINT_PARAM(gMacroPerCycle, cp);
	PRINT_PARAM(gMaxWithoutClockCorrectionFatal, cp);
	PRINT_PARAM(gMaxWithoutClockCorrectionPassive, cp);
	PRINT_PARAM(gNumberOfMinislots, cp);
	PRINT_PARAM(gNumberOfStaticSlots, cp);
	PRINT_PARAM(gPayloadLengthStatic, cp);
	PRINT_PARAM(gChannels, cp);
	PRINT_PARAM(gClusterDriftDamping, cp);
	PRINT_PARAM(gdBit, cp);
	PRINT_PARAM(gdCycle, cp);
	PRINT_PARAM(gdMacrotick, cp);
	PRINT_PARAM(gdNIT, cp);
	PRINT_PARAM(gdSampleClockPeriod, cp);
	PRINT_PARAM(gNetworkManagementVectorLength, cp);
	if (!is_v3) {
		PRINT_PARAM(v2.gAssumedPrecision, cp);
		PRINT_PARAM(v2.gdMaxInitializationError, cp);
		PRINT_PARAM(v2.gdMaxMicrotick, cp);
		PRINT_PARAM(v2.gdMaxPropagationDelay, cp);
		PRINT_PARAM(v2.gdMinPropagationDelay, cp);
		PRINT_PARAM(v2.gdWakeupSymbolRxIdle, cp);
		PRINT_PARAM(v2.gdWakeupSymbolRxLow, cp);
		PRINT_PARAM(v2.gdWakeupSymbolRxWindow, cp);
		PRINT_PARAM(v2.gdWakeupSymbolTxIdle, cp);
		PRINT_PARAM(v2.gdWakeupSymbolTxLow, cp);
		PRINT_PARAM(v2.gOffsetCorrectionMax, cp);
		PRINT_PARAM(v2.gOffsetCorrectionStart, cp);
		PRINT_PARAM(v2.gSyncNodeMax, cp);
	} else {
		PRINT_PARAM(v3.gClockDeviationMax, cp);
		PRINT_PARAM(v3.gCycleCountMax, cp);
		PRINT_PARAM(v3.gdIgnoreAfterTx, cp);
		PRINT_PARAM(v3.gdSymbolWindowActionPointOffset, cp);
		PRINT_PARAM(v3.gdWakeupRxIdle, cp);
		PRINT_PARAM(v3.gdWakeupRxLow, cp);
		PRINT_PARAM(v3.gdWakeupRxWindow, cp);
		PRINT_PARAM(v3.gdWakeupTxActive, cp);
		PRINT_PARAM(v3.gdWakeupTxIdle, cp);
		PRINT_PARAM(v3.gExternOffsetCorrection, cp);
		PRINT_PARAM(v3.gExternRateCorrection, cp);
		PRINT_PARAM(v3.gSyncFrameIDCountMax, cp);
	}
}
EXPORT_SYMBOL_GPL(print_cluster_config);

void print_node_config(struct flexray_node_param *np, int is_v3)
{
	pr_debug("Node configuration:\n");

	PRINT_PARAM(pAllowHaltDueToClock, np);
	PRINT_PARAM(pAllowPassiveToActive, np);
	PRINT_PARAM(pChannels, np);
	PRINT_PARAM(pClusterDriftDamping, np);
	PRINT_PARAM(pdAcceptedStartupRange, np);
	PRINT_PARAM(pDecodingCorrection, np);
	PRINT_PARAM(pDelayCompensationA, np);
	PRINT_PARAM(pDelayCompensationB, np);
	PRINT_PARAM(pdListenTimeout, np);
	PRINT_PARAM(pExternOffsetCorrection, np);
	PRINT_PARAM(pExternRateCorrection, np);
	PRINT_PARAM(pKeySlotID, np);
	PRINT_PARAM(pKeySlotUsedForStartup, np);
	PRINT_PARAM(pKeySlotUsedForSync, np);
	PRINT_PARAM(pLatestTx, np);
	PRINT_PARAM(pMacroInitialOffsetA, np);
	PRINT_PARAM(pMacroInitialOffsetB, np);
	PRINT_PARAM(pMicroInitialOffsetA, np);
	PRINT_PARAM(pMicroInitialOffsetB, np);
	PRINT_PARAM(pMicroPerCycle, np);
	PRINT_PARAM(vExternOffsetControl, np);
	PRINT_PARAM(pOffsetCorrectionOut, np);
	PRINT_PARAM(vExternRateControl, np);
	PRINT_PARAM(pRateCorrectionOut, np);
	PRINT_PARAM(pWakeupChannel, np);
	PRINT_PARAM(pWakeupPattern, np);
	PRINT_PARAM(pdMicrotick, np);
	PRINT_PARAM(pPayloadLengthDynMax, np);
	PRINT_PARAM(pSamplesPerMicrotick, np);

	if (is_v3) {
		PRINT_PARAM(v3.pExternalSync, np);
		PRINT_PARAM(v3.pFallBackInternal, np);
		PRINT_PARAM(v3.pKeySlotOnlyEnabled, np);
		PRINT_PARAM(v3.pNMVectorEarlyUpdate, np);
		PRINT_PARAM(v3.pOffsetCorrectionStart, np);
		PRINT_PARAM(v3.pSecondKeySlotID, np);
		PRINT_PARAM(v3.pTwoKeySlotMode, np);
	} else {
		PRINT_PARAM(v2.pdMaxDrift, np);
		PRINT_PARAM(v2.pMicroPerMacroNom, np);
		PRINT_PARAM(v2.pSingleSlotEnabled, np);
	}
}
EXPORT_SYMBOL_GPL(print_node_config);

void print_symbol_config(struct flexray_symbol_param *sp)
{
	pr_debug("Symbol configuration:\n");

	PRINT_PARAM(pChannelsMTS, sp);
}
EXPORT_SYMBOL_GPL(print_symbol_config);

/* Parameter check and set functions */

#define FR_CHECK_PARAM(p, name, min, max)				\
	do {								\
		if (p->name < min || p->name > max) {			\
			netdev_info(dev, #name" (0x%x) out of range\n",	\
					p->name);			\
			return -EINVAL;					\
		}							\
	} while (0)

static int flexray_check_cluster_params(struct flexray_cluster_param *cp,
					struct net_device *dev)
{
	struct flexray_priv *priv = netdev_priv(dev);

	FR_CHECK_PARAM(cp, gColdstartAttempts, 2, 31);
	FR_CHECK_PARAM(cp, gdActionPointOffset, 1, 63);
	FR_CHECK_PARAM(cp, gdMinislot, 2, 63);
	FR_CHECK_PARAM(cp, gdMinislotActionPointOffset, 1, 31);
	FR_CHECK_PARAM(cp, gListenNoise, 2, 16);
	FR_CHECK_PARAM(cp, gMacroPerCycle, 10, 16000);
	FR_CHECK_PARAM(cp, gMaxWithoutClockCorrectionFatal, 1, 15);
	FR_CHECK_PARAM(cp, gMaxWithoutClockCorrectionPassive, 1, 15);
	FR_CHECK_PARAM(cp, gNumberOfMinislots, 0, 7986);
	FR_CHECK_PARAM(cp, gNumberOfStaticSlots, 2, 1023);
	FR_CHECK_PARAM(cp, gPayloadLengthStatic, 0, 127);
	FR_CHECK_PARAM(cp, gClusterDriftDamping, 0, 5);
	FR_CHECK_PARAM(cp, gdNIT, 2, 805);
	FR_CHECK_PARAM(cp, gNetworkManagementVectorLength, 0, 12);
	FR_CHECK_PARAM(cp, gdDynamicSlotIdlePhase, 0, 2);

	if (priv->version == 2) {
		FR_CHECK_PARAM(cp, v2.gdMaxInitializationError, 0, 117);
		FR_CHECK_PARAM(cp, v2.gdMinPropagationDelay, 0,
			       cp->v2.gdMaxPropagationDelay);
		FR_CHECK_PARAM(cp, v2.gdWakeupSymbolRxIdle, 14, 59);
		FR_CHECK_PARAM(cp, v2.gdWakeupSymbolRxWindow, 76, 301);
		FR_CHECK_PARAM(cp, v2.gdWakeupSymbolTxIdle, 45, 180);
		FR_CHECK_PARAM(cp, v2.gdWakeupSymbolTxLow, 15, 60);
		FR_CHECK_PARAM(cp, v2.gOffsetCorrectionStart, 9, 15999);
		FR_CHECK_PARAM(cp, v2.gSyncNodeMax, 2, 15);
		FR_CHECK_PARAM(cp, gdCASRxLowMax, 67, 99);
		FR_CHECK_PARAM(cp, gdTSSTransmitter, 3, 15);
#ifndef CONFIG_MFD_EBEL_FLEXCARD_PROTPARAM
		FR_CHECK_PARAM(cp, v2.gdWakeupSymbolRxLow, 11, 59);
		FR_CHECK_PARAM(cp, gdStaticSlot, 4, 661);
#else
		FR_CHECK_PARAM(cp, v2.gdWakeupSymbolRxLow, 10, 55);
		FR_CHECK_PARAM(cp, gdStaticSlot, 4, 659);
#endif /* CONFIG_MFD_EBEL_FLEXCARD_PROTPARAM */
	}

	if (priv->version == 3) {
		FR_CHECK_PARAM(cp, v3.gCycleCountMax, 7, 63);
		FR_CHECK_PARAM(cp, v3.gdSymbolWindowActionPointOffset, 1, 63);
		FR_CHECK_PARAM(cp, v3.gdWakeupRxIdle, 8, 59);
		FR_CHECK_PARAM(cp, v3.gdWakeupRxLow, 8, 59);
		FR_CHECK_PARAM(cp, v3.gdWakeupRxWindow, 76, 485);
		FR_CHECK_PARAM(cp, v3.gdWakeupTxActive, 15, 60);
		FR_CHECK_PARAM(cp, v3.gdWakeupTxIdle, 45, 180);
		FR_CHECK_PARAM(cp, v3.gSyncFrameIDCountMax, 2, 15);
		FR_CHECK_PARAM(cp, v3.gClockDeviationMax, 1, 1500);
		FR_CHECK_PARAM(cp, v3.gExternOffsetCorrection, 0, 35);
		FR_CHECK_PARAM(cp, v3.gdIgnoreAfterTx, 0, 15);
		FR_CHECK_PARAM(cp, gdCASRxLowMax, 28, 254);
		FR_CHECK_PARAM(cp, gdStaticSlot, 3, 664);
		FR_CHECK_PARAM(cp, gdTSSTransmitter, 1, 15);
		if (!(cp->v3.gCycleCountMax % 2)) {
			netdev_info(dev, "gCycleCountMax is even\n");
			return -EINVAL;
		}
	}

	return 0;
}

static int flexray_set_cluster_params(struct flexray_cluster_param *cp,
				      struct net_device *dev)
{
	struct flexray_priv *priv = netdev_priv(dev);

	memcpy(&priv->cluster, cp, sizeof(*cp));

	return 0;
}

static int
flexray_check_and_set_cluster_params(struct flexray_cluster_param *cp,
				     struct net_device *dev)
{
	int ret;

	ret = flexray_check_cluster_params(cp, dev);
	if (ret)
		return ret;

	return flexray_set_cluster_params(cp, dev);
}

static int flexray_check_node_params(struct flexray_node_param *np,
				     struct net_device *dev)
{
	struct flexray_priv *priv = netdev_priv(dev);

	FR_CHECK_PARAM(np, pAllowPassiveToActive, 0, 31);
	FR_CHECK_PARAM(np, pClusterDriftDamping, 0, 20);
	FR_CHECK_PARAM(np, pExternRateCorrection, 0, 28);
	FR_CHECK_PARAM(np, vExternOffsetControl, -1, 1);
	FR_CHECK_PARAM(np, vExternRateControl, -1, 1);
	FR_CHECK_PARAM(np, pWakeupPattern, 2, 63);
	FR_CHECK_PARAM(np, pPayloadLengthDynMax, 0, 254);
	FR_CHECK_PARAM(np, pAllowPassiveToActive, 0, 31);
	FR_CHECK_PARAM(np, pChannels, 0, 3);
	FR_CHECK_PARAM(np, pWakeupChannel, 0, 3);
	FR_CHECK_PARAM(np, pKeySlotUsedForStartup, 0, 1);
	FR_CHECK_PARAM(np, pKeySlotUsedForSync, 0, 1);
#ifndef CONFIG_MFD_EBEL_FLEXCARD_PROTPARAM
	FR_CHECK_PARAM(np, pMacroInitialOffsetA, 2, 68);
	FR_CHECK_PARAM(np, pMacroInitialOffsetB, 2, 68);
	FR_CHECK_PARAM(np, pMicroInitialOffsetA, 0, 239);
	FR_CHECK_PARAM(np, pMicroInitialOffsetB, 0, 239);
#else
	FR_CHECK_PARAM(np, pMacroInitialOffsetA, 2, 72);
	FR_CHECK_PARAM(np, pMacroInitialOffsetB, 2, 72);
	FR_CHECK_PARAM(np, pMicroInitialOffsetA, 0, 240);
	FR_CHECK_PARAM(np, pMicroInitialOffsetB, 0, 240);
#endif /* CONFIG_MFD_EBEL_FLEXCARD_PROTPARAM */

	if (priv->version == 2) {
		FR_CHECK_PARAM(np, v2.pdMaxDrift, 2, 1923);
		FR_CHECK_PARAM(np, pdAcceptedStartupRange, 0, 1875);
		FR_CHECK_PARAM(np, pDecodingCorrection, 14, 143);
		FR_CHECK_PARAM(np, pdListenTimeout, 1284, 1283846);
		FR_CHECK_PARAM(np, pExternOffsetCorrection, 0, 7);
		FR_CHECK_PARAM(np, pKeySlotID, 0, 1023);
		FR_CHECK_PARAM(np, pMicroPerCycle, 640, 640000);
		FR_CHECK_PARAM(np, pRateCorrectionOut, 2, 1923);
#ifndef CONFIG_MFD_EBEL_FLEXCARD_PROTPARAM
		FR_CHECK_PARAM(np, pLatestTx, 0, 7980);
		FR_CHECK_PARAM(np, pOffsetCorrectionOut, 13, 15567);
#else
		FR_CHECK_PARAM(np, pLatestTx, 0, 7981);
		FR_CHECK_PARAM(np, pOffsetCorrectionOut, 5, 15266);
#endif /* CONFIG_MFD_EBEL_FLEXCARD_PROTPARAM */
	}
	if (priv->version == 3) {
		FR_CHECK_PARAM(np, v3.pOffsetCorrectionStart, 7, 15999);
		FR_CHECK_PARAM(np, v3.pSecondKeySlotID, 0, 1023);
		FR_CHECK_PARAM(np, pdAcceptedStartupRange, 29, 2743);
		FR_CHECK_PARAM(np, pDecodingCorrection, 12, 136);
		FR_CHECK_PARAM(np, pdListenTimeout, 1926, 2567692);
		FR_CHECK_PARAM(np, pExternOffsetCorrection, 0, 28);
		FR_CHECK_PARAM(np, pKeySlotID, 0, 1023);
		FR_CHECK_PARAM(np, pMicroPerCycle, 960, 1280000);
		FR_CHECK_PARAM(np, pRateCorrectionOut, 3, 3846);
		FR_CHECK_PARAM(np, pLatestTx, 0, 7988);
		FR_CHECK_PARAM(np, pOffsetCorrectionOut, 15, 16082);
	}

	return 0;
}

static int flexray_set_node_params(struct flexray_node_param *np,
			    struct net_device *dev)
{
	struct flexray_priv *priv = netdev_priv(dev);

	memcpy(&priv->node, np, sizeof(*np));

	return 0;
}

static int flexray_check_and_set_node_params(struct flexray_node_param *np,
					     struct net_device *dev)
{
	int ret;

	ret = flexray_check_node_params(np, dev);
	if (ret)
		return ret;

	return flexray_set_node_params(np, dev);
}

static int flexray_check_symbol_params(struct flexray_symbol_param *sp,
				       struct net_device *dev)
{
	FR_CHECK_PARAM(sp, pChannelsMTS, 0, 3);

	return 0;
}

static int flexray_set_symbol_params(struct flexray_symbol_param *sp,
				     struct net_device *dev)
{
	struct flexray_priv *priv = netdev_priv(dev);

	memcpy(&priv->symbol, sp, sizeof(*sp));

	return 0;
}

static int flexray_check_and_set_symbol_params(struct flexray_symbol_param *sp,
					       struct net_device *dev)
{
	int ret;

	ret = flexray_check_symbol_params(sp, dev);
	if (ret)
		return ret;

	return flexray_set_symbol_params(sp, dev);
}

/* FLEXRAY netlink interface */
static const struct nla_policy flexray_policy[IFLA_FLEXRAY_MAX + 1] = {
	[IFLA_FLEXRAY_STATE] = {.type = NLA_U32},
	[IFLA_FLEXRAY_VERSION] = {.type = NLA_U8},
	[IFLA_FLEXRAY_CLUSTER] = {.len = sizeof(struct flexray_cluster_param)},
	[IFLA_FLEXRAY_NODE] = {.len = sizeof(struct flexray_node_param)},
	[IFLA_FLEXRAY_SYMBOL] = {.len = sizeof(struct flexray_symbol_param)},
	[IFLA_FLEXRAY_SW_FILTER] = {.type = NLA_NESTED},
};

static const struct nla_policy flexray_filt_pol[IFLA_FLEXRAY_FILTER_MAX + 1] = {
	[IFLA_FLEXRAY_FILTER_ENTRY] = { .len = sizeof(struct flexray_filter) },
};

static int validate_and_set_sw_filter(struct flexray_sw_filter *sw, u32 *id)
{
	int i;

	if (sw->pos >= FLEXRAY_MAX_SW_FILTER)
		return -EINVAL;

	if (sw->id != 0)
		for (i = 0; i < sw->pos; i++)
			if (id[i] > sw->id)
				return -EINVAL;

	id[sw->pos] = sw->id;

	return 0;
}

static int flexray_changelink(struct net_device *dev,
			      struct nlattr *tb[], struct nlattr *data[])
{
	struct flexray_priv *priv = netdev_priv(dev);
	struct nlattr *attr;
	int rem, ret = 0;

	/* We need synchronization with dev->stop() */
	ASSERT_RTNL();

	if (data[IFLA_FLEXRAY_STATE]) {
		enum flexray_state state;

		state = nla_get_u32(data[IFLA_FLEXRAY_STATE]);
		if (priv->do_set_state)
			ret = priv->do_set_state(dev, state);
		if (ret)
			return ret;
		priv->state = state;
	}

	if (data[IFLA_FLEXRAY_CLUSTER]) {
		struct flexray_cluster_param cp;

		memcpy(&cp, nla_data(data[IFLA_FLEXRAY_CLUSTER]), sizeof(cp));
		ret = flexray_check_and_set_cluster_params(&cp, dev);
		if (ret)
			return ret;
	}

	if (data[IFLA_FLEXRAY_NODE]) {
		struct flexray_node_param np;

		memcpy(&np, nla_data(data[IFLA_FLEXRAY_NODE]), sizeof(np));
		ret = flexray_check_and_set_node_params(&np, dev);
		if (ret)
			return ret;
	}

	if (data[IFLA_FLEXRAY_SYMBOL]) {
		struct flexray_symbol_param sp;

		memcpy(&sp, nla_data(data[IFLA_FLEXRAY_SYMBOL]), sizeof(sp));
		ret = flexray_check_and_set_symbol_params(&sp, dev);
		if (ret)
			return ret;
	}

	if (data[IFLA_FLEXRAY_SW_FILTER]) {
		struct flexray_sw_filter *sw;

		nla_for_each_nested(attr, data[IFLA_FLEXRAY_SW_FILTER], rem) {
			sw = nla_data(attr);
			ret = validate_and_set_sw_filter(sw, priv->sw_filter);
			if (ret)
				return ret;
		}
	}

	return ret;
}

static inline int flexray_validate_sw_filter(struct nlattr *nest)
{
	struct flexray_sw_filter *sw;
	struct nlattr *attr;
	int rem, ret;
	u32 id = 0;

	if (!nest)
		return 0;

	ret = nla_validate_nested(nest, FLEXRAY_MAX_SW_FILTER,
				  flexray_filt_pol);
	if (ret)
		return ret;

	nla_for_each_nested(attr, nest, rem) {
		sw = nla_data(attr);
		if (sw->id < id)
			return -EINVAL;

		id = sw->id;
	}

	return 0;
}

static int flexray_validate(struct nlattr *tb[], struct nlattr *data[])
{
	int ret;

	ret = flexray_validate_sw_filter(data[IFLA_FLEXRAY_SW_FILTER]);
	if (ret)
		return ret;

	return 0;
}

static size_t flexray_get_size(const struct net_device *dev)
{
	size_t size;

	size = nla_total_size(sizeof(u32));
	size += nla_total_size(sizeof(struct flexray_cluster_param));
	size += nla_total_size(sizeof(struct flexray_node_param));

	return size;
}

static int flexray_fill_info(struct sk_buff *skb, const struct net_device *dev)
{
	struct flexray_priv *priv = netdev_priv(dev);
	enum flexray_state state = priv->state;
	struct flexray_cluster_param *cp = &priv->cluster;
	struct flexray_node_param *np = &priv->node;
	struct flexray_symbol_param *sp = &priv->symbol;
	struct flexray_sw_filter sw;
	struct nlattr *nest;
	int i, ret = 0;

	if (priv->do_get_state)
		ret = priv->do_get_state(dev, &state);
	if (nla_put_u32(skb, IFLA_FLEXRAY_STATE, state))
		goto nla_put_failure;
	if (nla_put_u8(skb, IFLA_FLEXRAY_VERSION, priv->version))
		goto nla_put_failure;
	if (nla_put(skb, IFLA_FLEXRAY_CLUSTER, sizeof(*cp), cp))
		goto nla_put_failure;
	if (nla_put(skb, IFLA_FLEXRAY_NODE, sizeof(*np), np))
		goto nla_put_failure;
	if (nla_put(skb, IFLA_FLEXRAY_SYMBOL, sizeof(*sp), sp))
		goto nla_put_failure;

	nest = nla_nest_start(skb, IFLA_FLEXRAY_SW_FILTER);
	if (nest == NULL)
		goto nla_put_failure;

	for (i = 0; i < FLEXRAY_MAX_SW_FILTER; i++) {
		sw.pos = i;
		sw.id = priv->sw_filter[i];
		if (nla_put(skb, IFLA_FLEXRAY_FILTER_ENTRY, sizeof(sw), &sw))
			goto nla_put_failure;

		if (sw.id == 0)
			break;
	}
	nla_nest_end(skb, nest);

	return ret;

nla_put_failure:

	return -EMSGSIZE;
}

static size_t flexray_get_xstats_size(const struct net_device *dev)
{
	return sizeof(struct flexray_device_stats);
}

static int flexray_fill_xstats(struct sk_buff *skb,
			       const struct net_device *dev)
{
	struct flexray_priv *priv = netdev_priv(dev);

	if (nla_put(skb, IFLA_INFO_XSTATS,
		    sizeof(priv->flexray_stats), &priv->flexray_stats))
		goto nla_put_failure;

	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

static int flexray_newlink(struct net *src_net, struct net_device *dev,
		       struct nlattr *tb[], struct nlattr *data[])
{
	return -EOPNOTSUPP;
}

static struct rtnl_link_ops flexray_link_ops __read_mostly = {
	.kind		= "flexray",
	.maxtype	= IFLA_FLEXRAY_MAX,
	.policy		= flexray_policy,
	.setup		= flexray_setup,
	.validate	= flexray_validate,
	.newlink	= flexray_newlink,
	.changelink	= flexray_changelink,
	.get_size	= flexray_get_size,
	.fill_info	= flexray_fill_info,
	.get_xstats_size = flexray_get_xstats_size,
	.fill_xstats	= flexray_fill_xstats,
};

/* Register the FLEXRAY network device */
int register_flexraydev(struct net_device *dev)
{
	dev->rtnl_link_ops = &flexray_link_ops;
	return register_netdev(dev);
}
EXPORT_SYMBOL_GPL(register_flexraydev);

/* Unregister the FLEXRAY network device */
void unregister_flexraydev(struct net_device *dev)
{
	unregister_netdev(dev);
}
EXPORT_SYMBOL_GPL(unregister_flexraydev);

static __init int flexray_dev_init(void)
{
	int ret;

	ret = rtnl_link_register(&flexray_link_ops);
	if (!ret)
		pr_info("FlexRay netlink interface\n");

	return ret;
}
module_init(flexray_dev_init);

static __exit void flexray_dev_exit(void)
{
	rtnl_link_unregister(&flexray_link_ops);
}
module_exit(flexray_dev_exit);

MODULE_ALIAS_RTNL_LINK("flexray");

MODULE_DESCRIPTION("FlexRay device driver interface");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Benedikt Spranger <b.spranger@linutronix.de>");
