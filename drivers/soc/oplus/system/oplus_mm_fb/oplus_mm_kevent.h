/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#ifndef _OPLUS_MM_KEVENT_
#define _OPLUS_MM_KEVENT_

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/types.h>
#include <linux/netlink.h>
#include <net/net_namespace.h>
#include <net/sock.h>
#include <linux/version.h>

#define MAX_PAYLOAD_TAG 			 (128)
#define MAX_PAYLOAD_EVENTID 		 (128)
#define MAX_PAYLOAD_DATASIZE		 (1024)

#ifndef OPLUS_NETLINK_MM_KEVENT
#define OPLUS_NETLINK_MM_KEVENT      (41)
#endif

#define MM_KEVENT_MODULE_SIZE_MAX	 (16)
#define MM_KEVENT_MODULE_LEN_MAX	 (64)

#define MM_KEVENT_BAD_VALUE 		 (-1)
#define MM_KEVENT_NO_ERROR			 (0)

//#define OPLUS_NETLINK_MM_KEVENT_TEST
#define OPLUS_NETLINK_MM_DBG_LV1     0x1
#define OPLUS_NETLINK_MM_DBG_LV2     0x2

#define DP_FB_EVENT 	"mm_kevent_dp"
#define ATLAS_FB_EVENT 	"mm_kevent_atlas"

enum mm_kevent_type {
	MM_KEVENT_NOME = 0x0,
	MM_KEVENT_CONNECT,
};

struct mm_kevent_module {
	u32 pid;
	char modl[MM_KEVENT_MODULE_LEN_MAX];
};

typedef void (*mm_kevent_recv_user_func)(int type, int flags, char* data);

struct mm_kevent_packet {
	int  type;							/* 0:warrning,1:error,2:hw error*/
	char tag[MAX_PAYLOAD_TAG];			/* logTag */
	char event_id[MAX_PAYLOAD_EVENTID]; /* eventID */
	size_t len; 						/* Length of packet data */
	unsigned char data[0];				/* Optional packet data */
}__attribute__((packed));

#ifdef CONFIG_OPLUS_FEATURE_MM_FEEDBACK
int mm_fb_kevent_send_to_user(struct mm_kevent_packet *userinfo);
void mm_fb_kevent_set_recv_user(mm_kevent_recv_user_func recv_func);
void mm_fb_kevent_receive(struct sk_buff *__skbbr);
#else //CONFIG_OPLUS_FEATURE_MM_FEEDBACK
int mm_fb_kevent_send_to_user(struct mm_kevent_packet *userinfo) {return 0;}
void mm_fb_kevent_set_recv_user(mm_kevent_recv_user_func recv_func) {return;}
void mm_fb_kevent_receive(struct sk_buff *__skbbr) {return;}
#endif //CONFIG_OPLUS_FEATURE_MM_FEEDBACK
#endif  //_OPLUS_MM_KEVENT_

