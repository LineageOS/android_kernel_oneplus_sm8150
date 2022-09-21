/***********************************************************
** Copyright (C), 2008-2020, OPLUS Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - fs/ext4/e4defrag.h
** Description:  code to support ext4 defrag
**
** Version: 1.0
** Date : 2020/02/29
**
** ------------------ Revision History:------------------------
** <author> <data> <version > <desc>
** yanwu 2020/02/29 1.0  create the file
****************************************************************/

#ifndef _EXT4_DEFRAG_H
#define _EXT4_DEFRAG_H

#include <linux/rbtree.h>

/* customized mount flag */
#define EXT4_MOUNT2_BG_DEFRAG 0x10000000	/* background online defrag */

#define DEFRAG_MIN_SLEEP_TIME (2*1000)
#define DEFRAG_SLEEP_TIME (15*60*1000)
#define DEFRAG_MAX_SLEEP_TIME (60*60*1000)
#define DEFRAG_INTERVAL (60*60*1000)
#define DEFRAG_MAX_LEN (31)
#define DEFRAG_MIN_FREE(sb) (10 * EXT4_BLOCKS_PER_GROUP(sb) / 100)
#define DEFRAG_NR_TO_SCAN (16)

/* sometimes we need to lookup the inode in other group to found
 * the extent of the current group, so we cache the upto 8 other
 * groups which contains max extent of current group.
 */
struct group_entry {
	unsigned short group;	/* group number */
	unsigned short len;	/* extents length we found */
	unsigned short first_bit;	/* first inode to scan */
	unsigned short last_bit;	/* last inode to scan */
};

struct group_table {
	short nr_entries;
	struct group_entry entries[8];
};

struct defrag_group_state {
	unsigned short first_free[2];	/* first free block in group */
	unsigned short free[2];	/* free blocks in group */
	unsigned short fragments[2];	/* fragments in the give block */
	unsigned short scanned:1;	/* inode in the group scanned ? */
	unsigned short loaded:1;	/* is extent load done ? */
	unsigned short started:1;	/* defrag started? */
	unsigned short paused:1;	/* defrag paused? */
	unsigned short done:1;	/* defrag done? */
	unsigned long start;	/* last defrag start time */
	unsigned long end;	/* last defrag end time */
	unsigned long update;	/* last state update time */
	unsigned long count;	/* number of defrag run */
	unsigned long moved;	/* blocks moved by defrag */
	unsigned long scan;	/* time cost to load extents */
	unsigned long cost;	/* time cost to defrag in ms */
	unsigned long duration;	/* defrag average duration */
	struct group_table table;	/* groups cantain inodes to lookup */
};

struct group_extent_tree {
	struct rb_root root;	/* root of all extents */
	unsigned short len;	/* length of all extents */
};

/* defrag state */
enum {
	DS_INIT,
	DS_RUNNING,
	DS_DONE,
};

/* defrag thread wake up reason */
enum {
	WAKE_TIMEOUT = 0,
	WAKE_USER,
	WAKE_SYS,
};

/* infomation for ext4 defrag */
struct ext4_defrag_info {
	struct task_struct *task;	/* task struct of defrag thread */
	wait_queue_head_t init_wq;	/* wait for defrag thread start */
	wait_queue_head_t wq;	/* defrag thread wait queue */
	unsigned int min_sleep_time;	/* sleep time if defrag paused */
	unsigned int sleep_time;	/* sleep time after 1 group defragged */
	unsigned int max_sleep_time;	/* sleep time if no group need defrag */
	unsigned int interval;	/* interval for 1 group defrag again */
	unsigned int wake:2;	/* wake up reason of defrag thread */
	unsigned int force_mode:1;	/* in force_mode, we ignore user io */
	unsigned int state:2;	/* current defrag state */
	unsigned int policy:2;	/* group select policy */
	unsigned int min_score;	/* frag_score threshold for group select */
	unsigned int min_free;	/* free space threshold for group select */
	unsigned int nr_to_scan;	/* nr_groups to query for group select */
	unsigned int max_len;	/* max extent len we can move */
	unsigned int group;	/* current defrag group */
	unsigned long last_ino;	/* ino when defrag interruptted */
	struct group_extent_tree tree;	/* extents tree of defrag group */
	struct inode *donor_inode;	/* donor inode for move extent */
	unsigned int ngroups;	/* number of group in fs */
	struct defrag_group_state *groups;
	struct kobject kobj;
};

/* e4defrag.c */
int e4defrag_init(struct super_block *sb);
void e4defrag_exit(struct super_block *sb);
int __init e4defrag_init_fs(void);
void e4defrag_exit_fs(void);
bool need_defrag(struct super_block *sb, unsigned free, unsigned fragments);
void e4defrag_wake_up_thread(struct super_block *sb);

#define FRAG_SCORE_FACTOR  (1<<15)
#define FRAG_SCORE(s) ((s) * FRAG_SCORE_FACTOR / 100)
#define NORM_SCORE(s) ((s) * 100 / FRAG_SCORE_FACTOR)
#define DEFRAG_MIN_SCORE (30)

#ifndef SB_RDONLY
static inline bool sb_rdonly(const struct super_block *sb)
{
	return sb->s_flags & MS_RDONLY;
}
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
static inline void *kvmalloc(size_t size, gfp_t flags)
{
	void *ret;

	ret = kmalloc(size, flags | __GFP_NOWARN);
	if (!ret) {
		ret = __vmalloc(size, flags, PAGE_KERNEL);
	}
	return ret;
}

/**
 * Reads are always treated as synchronous, as are requests with the FUA or
 * PREFLUSH flag.  Other operations may be marked as synchronous using the
 * REQ_SYNC flag.
 */
static inline bool bio_is_sync(struct bio *bio)
{
	return (bio_op(bio) == REQ_OP_READ ||
		bio_flags(bio) & (REQ_SYNC | REQ_FUA | REQ_PREFLUSH));
}

static inline bool ext4_is_quota_file(struct inode *inode)
{
	return IS_NOQUOTA(inode);
}
#else
static inline bool bio_is_sync(struct bio *bio)
{
	return op_is_sync(bio->bi_opf);
}
#endif

#ifndef IS_ENCRYPTED
static inline bool ext4_encrypted_inode(struct inode *inode);
#define IS_ENCRYPTED(inode) ext4_encrypted_inode(inode)
static inline int fscrypt_using_hardware_encryption(struct inode *inode)
{
	return fs_using_hardware_encryption(inode);
}

static inline int fscrypt_require_key(struct inode *inode)
{
	if (IS_ENCRYPTED(inode)) {
		int err = fscrypt_get_encryption_info(inode);
		if (err) {
			return err;
		}
		if (!fscrypt_has_encryption_key(inode)) {
			return -ENOKEY;
		}
	}
	return 0;
}
#endif
#endif				/* _EXT4_DEFRAG_H */
