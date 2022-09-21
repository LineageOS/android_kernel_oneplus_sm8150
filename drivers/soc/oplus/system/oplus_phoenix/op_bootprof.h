/**************************************************************
SPDX-License-Identifier: GPL-2.0-only
Copyright (C) 2018-2020 Oplus. All rights reserved.

* File : Op_bootprof.c
* Description: Source file for bootlog.
* Storage the boot log on proc.
* Version : 1.0
* Date : 2018-10-18
* Author : bright.zhang@oplus.com
* ---------------- Revision History: --------------------------
* <author>       <date>           <desc>
* Bright Zhang   2018/12/24       create this file
****************************************************************/

/*=============================================================================

                            INCLUDE FILES FOR MODULE

=============================================================================*/
#ifndef OP_BOOTPROF_H
#define  OP_BOOTPROF_H
#include <linux/seq_file.h>
#define SEQ_printf(m, x...)	    \
	do {			    \
		if (m)		    \
			seq_printf(m, x);	\
		else		    \
			pr_info(x);	    \
	} while (0)


void op_log_boot(const char *str);

#endif
