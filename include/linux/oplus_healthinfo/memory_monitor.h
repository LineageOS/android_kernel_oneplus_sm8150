/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */
#ifndef _MEMORY_MONITOR_H_
#define _MEMORY_MONITOR_H_

extern void memory_alloc_monitor(gfp_t gfp_mask, unsigned int order, u64 wait_ms);
extern struct alloc_wait_para allocwait_para;

extern void oplus_ionwait_monitor(u64 wait_ms);
extern struct ion_wait_para ionwait_para;

#endif /*_MEMORY_MONITOR_H_*/
