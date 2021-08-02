/*
 * Copyright 2012 Eberspächer Electronics GmbH & Co. KG. All Rights Reserved.
 */

#ifndef FLEXRAY_RAW_H
#define FLEXRAY_RAW_H

#include <linux/flexray.h>

#define SOL_FLEXRAY_RAW (SOL_FLEXRAY_BASE + FLEXRAY_RAW)

/* for socket options affecting the socket (not the global system) */

enum {
	FLEXRAY_RAW_FILTER = 1,     /* set 0 .. n can_filter(s) */
	FLEXRAY_RAW_LOOPBACK,       /* local loopback (default:on)       */
	FLEXRAY_RAW_RECV_OWN_MSGS   /* receive my own msgs (default:off) */
};

#endif
