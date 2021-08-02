/*
 * Copyright 2012 Eberspächer Electronics GmbH & Co. KG. All Rights Reserved.
 */

#ifndef FLEXRAY_NETLINK_H
#define FLEXRAY_NETLINK_H

#include <linux/types.h>

#define FLEXRAY_MAX_SW_FILTER	2048

/*
 * FlexRay operational and error states
 */
enum flexray_state {
	FLEXRAY_STATE_UNSPEC,
	FLEXRAY_STATE_DEFAULT_CONFIG,
	FLEXRAY_STATE_CONFIG,
	FLEXRAY_STATE_READY,
	FLEXRAY_STATE_WAKEUP,
	FLEXRAY_STATE_STARTUP,
	FLEXRAY_STATE_NORMAL_ACTIVE,
	FLEXRAY_STATE_NORMAL_PASSIVE,
	FLEXRAY_STATE_HALT,
	FLEXRAY_STATE_MONITOR_MODE,
	FLEXRAY_STATE_COLDSTART,
	FLEXRAY_STATE_MAX
};

/*
 * FLEXRAY controller mode
 */
struct flexray_ctrlmode {
   __u32 mask;
   __u32 flags;
};

#define FLEXRAY_CTRLMODE_LOOPBACK    0x01  /* Loopback mode */

/*
 * FlexRay device statistics
 */
struct flexray_device_stats {
	__u32 txack;
	__u32 manage;
	__u32 info;
	__u32 status;
	__u32 notify;
};

/*
 * FlexRay netlink interface
 */
enum {
	IFLA_FLEXRAY_UNSPEC,
	IFLA_FLEXRAY_STATE,
	IFLA_FLEXRAY_VERSION,
	IFLA_FLEXRAY_CLUSTER,
	IFLA_FLEXRAY_NODE,
	IFLA_FLEXRAY_SYMBOL,
	IFLA_FLEXRAY_SW_FILTER,
	__IFLA_FLEXRAY_MAX
};

#define IFLA_FLEXRAY_MAX    (__IFLA_FLEXRAY_MAX - 1)

/*
 * FlexRay filter netlink interface
 */
enum {
	IFLA_FLEXRAY_FILTER_UNSPEC,
	IFLA_FLEXRAY_FILTER_ENTRY,
	__IFLA_FLEXRAY_FILTER_MAX
};

#define IFLA_FLEXRAY_FILTER_MAX    (__IFLA_FLEXRAY_FILTER_MAX - 1)

struct cluster_v2 {
	__u16 gAssumedPrecision;
	__u8 gdMaxInitializationError;
	__u8 gdMaxMicrotick;
	__u8 gdMaxPropagationDelay;
	__u8 gdMinPropagationDelay;
	__u8 gdWakeupSymbolRxIdle;
	__u8 gdWakeupSymbolRxLow;
	__u16 gdWakeupSymbolRxWindow;
	__u8 gdWakeupSymbolTxIdle;
	__u8 gdWakeupSymbolTxLow;
	__u32 gOffsetCorrectionMax;
	__u16 gOffsetCorrectionStart;
	__u8 gSyncNodeMax;
};

struct cluster_v3 {
	__u16 gClockDeviationMax;
	__u8 gCycleCountMax;
	__u8 gdIgnoreAfterTx;
	__u8 gdSymbolWindowActionPointOffset;
	__u8 gdWakeupRxIdle;
	__u8 gdWakeupRxLow;
	__u16 gdWakeupRxWindow;
	__u8 gdWakeupTxActive;
	__u8 gdWakeupTxIdle;
	__u8 gExternOffsetCorrection;
	__u8 gExternRateCorrection;
	__u8 gSyncFrameIDCountMax;
};

struct flexray_cluster_param {
	/* Protocol relevant */
	__u8 gColdstartAttempts;
	__u8 gdActionPointOffset;
	__u8 gdCASRxLowMax;
	__u8 gdDynamicSlotIdlePhase;
	__u8 gdMinislot;
	__u8 gdMinislotActionPointOffset;
	__u16 gdStaticSlot;
	__u8 gdSymbolWindow;
	__u8 gdTSSTransmitter;
	__u8 gListenNoise;
	__u16 gMacroPerCycle;
	__u8 gMaxWithoutClockCorrectionFatal;
	__u8 gMaxWithoutClockCorrectionPassive;
	__u16 gNumberOfMinislots;
	__u16 gNumberOfStaticSlots;
	__u8 gPayloadLengthStatic;

	/* Protocol related */
	__u8 gChannels;
	__u8 gClusterDriftDamping;
	__u8 gdBit;
	__u16 gdCycle;
	__u8 gdMacrotick;
	__u16 gdNIT;
	__u8 gdSampleClockPeriod;
	__u8 gNetworkManagementVectorLength;
	union {
		struct cluster_v2 v2;
		struct cluster_v3 v3;
	};
};

struct node_v2 {
	__u16 pdMaxDrift;
	__u8 pMicroPerMacroNom;
	__u8 pSingleSlotEnabled;
};

struct node_v3 {
	__u8 pExternalSync;
	__u8 pFallBackInternal;
	__u8 pKeySlotOnlyEnabled;
	__u8 pNMVectorEarlyUpdate;
	__u8 pOffsetCorrectionStart;
	__u8 pSecondKeySlotID;
	__u8 pTwoKeySlotMode;
};

struct flexray_node_param {
	/* Protocol relevant */
	__u8 pAllowHaltDueToClock;
	__u8 pAllowPassiveToActive;
	__u8 pChannels;
	__u8 pClusterDriftDamping;
	__u16 pdAcceptedStartupRange;
	__u8 pDecodingCorrection;
	__u8 pDelayCompensationA;
	__u8 pDelayCompensationB;
	__u32 pdListenTimeout;
	__s8 vExternOffsetControl;
	__s8 vExternRateControl;
	__u8 pExternOffsetCorrection;
	__u8 pExternRateCorrection;
	__u16 pKeySlotID;
	__u8 pKeySlotUsedForStartup;
	__u8 pKeySlotUsedForSync;
	__u16 pLatestTx;
	__u8 pMacroInitialOffsetA;
	__u8 pMacroInitialOffsetB;
	__u16 pMicroInitialOffsetA;
	__u16 pMicroInitialOffsetB;
	__u32 pMicroPerCycle;
	__u16 pOffsetCorrectionOut;
	__u16 pRateCorrectionOut;
	__u8 pWakeupChannel;
	__u8 pWakeupPattern;

	/* Protocol related */
	__u8 pdMicrotick;
	__u8 pPayloadLengthDynMax;
	__u8 pSamplesPerMicrotick;

	union {
		struct node_v2 v2;
		struct node_v3 v3;
	};
};

struct flexray_symbol_param {
	/* Protocol related */
	__u8 pChannelsMTS;
};

/*
 * FlexRay software ID filter
 */

struct flexray_sw_filter {
	__u32 pos;
	__u32 id;
};

#endif /* FLEXRAY_NETLINK_H */
