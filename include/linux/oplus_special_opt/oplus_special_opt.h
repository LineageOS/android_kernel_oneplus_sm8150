/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Oplus. All rights reserved.
 */


#ifndef _OPLUS_SPECIAL_OPT
#define _OPLUS_SPECIAL_OPT
#define HEAVY_LOAD_RUNTIME (64000000)
#define HEAVY_LOAD_SCALE (80)
extern bool is_heavy_load_task(struct task_struct *p);
#endif
