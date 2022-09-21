/***********************************************************
** Copyright (C), 2008-2020, OPLUS Mobile Comm Corp., Ltd.
** VENDOR_EDIT
** File: - fs/ext4/e4defrag.c
** Description:  code to support ext4 defrag
**
** Version: 1.0
** Date : 2020/02/29
**
** ------------------ Revision History:------------------------
** <author> <data> <version > <desc>
** yanwu 2020/02/29 1.0  create the file
****************************************************************/

#include <linux/fs.h>
#include <linux/version.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/freezer.h>
#include <linux/rbtree.h>
#include <linux/jiffies.h>
#include <linux/seq_file.h>
#include <linux/random.h>
#include <trace/events/block.h>
#include <asm/local.h>
#include <linux/sched.h>
#include <linux/cred.h>
#include <linux/key.h>
#include <linux/task_io_accounting_ops.h>
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
#include <linux/sched/mm.h>
#endif

#include "ext4.h"
#include "mballoc.h"
#include "extents_status.h"
#include "ext4_extents.h"
#include "ext4_jbd2.h"

#define E4DEFRAG_SBI(_dfi) (container_of(_dfi, struct ext4_sb_info, dfi))
#define E4DEFRAG_SB(_dfi) (E4DEFRAG_SBI(_dfi)->s_sb)
#define E4DEFRAG_I(sb)  (&EXT4_SB(sb)->dfi)
#define E4DEFRAG_NGROUPS(sb)  (E4DEFRAG_I(sb)->ngroups)
#define E4DEFRAG_GS(sb, group)  (&E4DEFRAG_I(sb)->groups[(group)])
#define E4DEFRAG_ET(sb)  (&E4DEFRAG_I(sb)->tree)
#define E4DEFRAG_INO(sb, group, bit)  (EXT4_INODES_PER_GROUP(sb) * (group) + (bit) + 1)
#define E4DEFRAG_FIRST_INO(sb, group)  ((group) ? E4DEFRAG_INO(sb, group, 0) : EXT4_FIRST_INO(sb))
#define E4DEFRAG_LAST_INO(sb, group)  (E4DEFRAG_INO(sb, (group) + 1, -1))
#define E4DEFRAG_GT(sb, group)  (&E4DEFRAG_GS(sb, group)->table)
#define E4DEFRAG_GE(tbl, i)  (&(tbl)->entries[i])
#define group_table_for_each(tbl, entry, i)  \
	for (i = 0; entry = E4DEFRAG_GE(tbl, i), i < (tbl)->nr_entries; i++)

/* #define DEBUG */
#ifdef DEBUG
#define e4defrag_dbg(f, a...)						\
	do {								\
		printk(KERN_DEBUG "e4defrag debug (%s, %d): %s:",	\
			__FILE__, __LINE__, __func__);			\
		printk(KERN_DEBUG f, ## a);				\
	} while (0)
#else
#define e4defrag_dbg(fmt, ...)	no_printk(fmt, ##__VA_ARGS__)
#endif
#define e4defrag_msg(f, a...) 						\
	do { 								\
		printk_ratelimited(KERN_INFO "e4defrag: " f, ## a);	\
	} while (0)

#define e4defrag_err(f, a...)						\
	do { 								\
		printk_ratelimited(KERN_ERR "e4defrag: " f, ## a);	\
	} while (0)

#define DEFRAG_PROTECT_NORMAL (0x0)
#define DEFRAG_PROTECT_LOWPOWER (0x1)
#define DEFRAG_PROTECT_EOL (0x2)
int ext4_defrag_protect = DEFRAG_PROTECT_NORMAL;

struct defrag_extent {
	struct rb_node node;
	unsigned int ino;
	unsigned short len;
	ext4_lblk_t lblk;
	ext4_fsblk_t pblk;
};

static struct kmem_cache *defrag_extent_cachep;

static void free_e4defrag_thread(struct ext4_defrag_info *dfi);

static int __init e4defrag_create_cache(void)
{
	defrag_extent_cachep = kmem_cache_create("defrag_extent",
						 sizeof(struct defrag_extent),
						 0, (SLAB_RECLAIM_ACCOUNT),
						 NULL);
	if (defrag_extent_cachep == NULL) {
		return -ENOMEM;
	}
	return 0;
}

static void __exit e4defrag_destroy_cache(void)
{
	if (defrag_extent_cachep) {
		kmem_cache_destroy(defrag_extent_cachep);
	}
}

static struct defrag_extent *e4defrag_alloc_extent(unsigned long ino,
						   ext4_lblk_t lblk,
						   ext4_lblk_t len,
						   ext4_grpblk_t pblk)
{
	struct defrag_extent *ex;
	ex = kmem_cache_alloc(defrag_extent_cachep, GFP_NOFS);
	if (ex == NULL) {
		return NULL;
	}
	ex->ino = ino;
	ex->lblk = lblk;
	ex->len = len;
	ex->pblk = pblk;
	return ex;
}

static void e4defrag_free_extent(struct defrag_extent *ex)
{
	kmem_cache_free(defrag_extent_cachep, ex);
}

/* group table */
static inline void group_entry_init(struct group_entry *entry,
				    ext4_group_t group, int len, int bit)
{
	entry->group = group;
	entry->len = len;
	entry->first_bit = bit;
	entry->last_bit = bit;
}

static inline void group_entry_merge(struct group_entry *entry1,
				     struct group_entry *entry2)
{
	entry1->len += entry2->len;
	if (entry1->first_bit > entry2->first_bit) {
		entry1->first_bit = entry2->first_bit;
	}
	if (entry1->last_bit < entry2->last_bit) {
		entry1->last_bit = entry2->last_bit;
	}
}

static inline unsigned long group_entry_first_ino(struct super_block *sb,
						  struct group_entry *entry)
{
	return E4DEFRAG_INO(sb, entry->group, entry->first_bit);
}

static inline unsigned long group_entry_last_ino(struct super_block *sb,
						 struct group_entry *entry)
{
	return E4DEFRAG_INO(sb, entry->group, entry->last_bit);
}

static inline void group_table_init(struct super_block *sb, ext4_group_t group)
{
	E4DEFRAG_GT(sb, group)->nr_entries = 0;
}

/* check if we can load extents using group table */
static inline ext4_grpblk_t defrag_free(struct super_block *sb,
					ext4_group_t group, bool idx);
static inline ext4_grpblk_t defrag_first_free(struct super_block *sb,
					      ext4_group_t group, bool idx);
static inline bool is_group_table_valid(struct super_block *sb,
					ext4_group_t group)
{
	unsigned int expect, found, first, free;
	struct group_table *tbl = E4DEFRAG_GT(sb, group);
	struct group_entry *entry;
	int i;

	if (!tbl->nr_entries) {
		return false;
	}
	first = defrag_first_free(sb, group, 0);
	free = defrag_free(sb, group, 0);
	expect = EXT4_CLUSTERS_PER_GROUP(sb) - first - free;
	found = 0;
	group_table_for_each(tbl, entry, i) {
		e4defrag_dbg("group %u entry (%u,%u,%u,%u)", group,
			     entry->group, entry->len, entry->first_bit,
			     entry->last_bit);
		found += entry->len;
	}
	found = found > first ? found - first : 0;
	e4defrag_dbg("group %u table (%u,%u)", group, expect, found);
	/* table valid if half of extents found */
	return found >= (expect >> 2);
}

/* get the ino to start scan for @group */
static inline unsigned long get_first_ino(struct super_block *sb,
					  ext4_group_t group)
{
	struct group_table *tbl = E4DEFRAG_GT(sb, group);
	return tbl->nr_entries ?
	    group_entry_first_ino(sb, E4DEFRAG_GE(tbl, 0)) :
	    E4DEFRAG_FIRST_INO(sb, group);
}

static inline bool defrag_scanned(struct super_block *sb, ext4_group_t group);
static inline void group_table_add_entry(struct super_block *sb,
					 ext4_group_t group,
					 unsigned int len, unsigned long ino)
{
	int i;
	unsigned int ino_bit;
	ext4_group_t ino_group;
	struct group_entry *entry, new_entry, *min_entry;
	struct group_table *tbl = E4DEFRAG_GT(sb, group);

	ino_bit = (ino - 1) % EXT4_INODES_PER_GROUP(sb);
	ino_group = (ino - 1) / EXT4_INODES_PER_GROUP(sb);
	if (defrag_scanned(sb, ino_group)) {
		return;
	}
	min_entry = E4DEFRAG_GE(tbl, 0);
	group_table_for_each(tbl, entry, i) {
		if (ino_group == entry->group) {
			group_entry_init(&new_entry, ino_group, len, ino_bit);
			group_entry_merge(entry, &new_entry);
			return;
		}
		if (entry->len < min_entry->len) {
			min_entry = entry;
		}
	}

	if (tbl->nr_entries < ARRAY_SIZE(tbl->entries)) {
		group_entry_init(E4DEFRAG_GE(tbl, i), ino_group, len, ino_bit);
		tbl->nr_entries++;
	} else {
		e4defrag_msg("group %u table full, evict (%d,%d)", group,
			     min_entry->group, min_entry->len);
		group_entry_init(min_entry, ino_group, len, ino_bit);
	}
}

static inline unsigned long
defrag_time_diff(unsigned long start, unsigned long end)
{
	if (end >= start) {
		return jiffies_to_msecs(end - start);
	}
	return jiffies_to_msecs(end + MAX_JIFFY_OFFSET - start);
}

/* defrag state machine */
enum defrag_phase {
	DEFRAG_INIT,
	DEFRAG_START,
	DEFRAG_SCAN_PAUSE,
	DEFRAG_GROUP_SCANNED,
	DEFRAG_SCAN_DONE,
	DEFRAG_SCAN_RESET,
	DEFRAG_PAUSE,
	DEFRAG_CONTINUE,
	DEFRAG_DONE,
};

/* the last argument for defrag_update_state */
union defrag_state_data {
	/* parameters for @DEFRAG_START and @DEFRAG_DONE */
	struct {
		ext4_grpblk_t first_free;
		ext4_grpblk_t free;
		ext4_grpblk_t fragments;
		ext4_lblk_t moved_;
	};
	/* parameters for @DEFRAG_PAUSE */
	ext4_lblk_t moved;
	unsigned long ino;
};

static void defrag_update_state(struct super_block *sb, ext4_group_t group,
				enum defrag_phase phase,
				union defrag_state_data *dsd)
{
	struct ext4_defrag_info *dfi = E4DEFRAG_I(sb);
	struct defrag_group_state *dgs = E4DEFRAG_GS(sb, group);
	unsigned long cost;

	switch (phase) {
	case DEFRAG_INIT:
		dgs->scanned = 0;
		dgs->started = 0;
		dgs->paused = 0;
		dgs->done = 0;
		dgs->cost = 0;
		dgs->count = 0;
		dgs->duration = 0;
		dgs->moved = 0;
		dgs->scan = 0;
		group_table_init(sb, group);
		break;
	case DEFRAG_START:
		dgs->start = jiffies;
		dgs->update = jiffies;
		dgs->loaded = 0;
		dgs->started = 1;
		dgs->paused = 0;
		dgs->done = 0;
		dgs->count++;
		dgs->first_free[0] = dsd->first_free;
		dgs->free[0] = dsd->free;
		dgs->fragments[0] = dsd->fragments;
		dfi->group = group;
		dfi->last_ino = get_first_ino(sb, group);
		break;
	case DEFRAG_SCAN_PAUSE:
		dfi->last_ino = dsd->ino;
		cost = defrag_time_diff(dgs->update, jiffies);
		dgs->update = jiffies;
		dgs->scan += cost;
		dgs->cost += cost;
		dgs->paused = 1;
		break;
	case DEFRAG_GROUP_SCANNED:
		dgs->scanned = 1;
		break;
	case DEFRAG_SCAN_DONE:
		cost = defrag_time_diff(dgs->update, jiffies);
		dgs->update = jiffies;
		dgs->scan += cost;
		dgs->cost += cost;
		dgs->loaded = 1;
		break;
	case DEFRAG_SCAN_RESET:
		dgs->scanned = 0;
		group_table_init(sb, group);
		break;
	case DEFRAG_PAUSE:
		dgs->paused = 1;
		dgs->moved += dsd->moved;
		dgs->cost += defrag_time_diff(dgs->update, jiffies);
		break;
	case DEFRAG_CONTINUE:
		dgs->paused = 0;
		dgs->update = jiffies;
		break;
	case DEFRAG_DONE:
		dgs->loaded = 0;
		dgs->started = 0;
		dgs->paused = 0;
		dgs->done = 1;
		dgs->first_free[1] = dsd->first_free;
		dgs->free[1] = dsd->free;
		dgs->fragments[1] = dsd->fragments;
		dgs->moved += dsd->moved_;
		dgs->end = jiffies;
		dgs->cost += defrag_time_diff(dgs->update, dgs->end);
		dgs->duration = (dgs->duration +
				 7 * defrag_time_diff(dgs->start,
						      dgs->end)) >> 3;
		break;
	default:
		BUG_ON(1);
	}
}

static inline bool defrag_force_mode(struct super_block *sb)
{
	return E4DEFRAG_I(sb)->force_mode;
}

/* get the current group to defrag */
static inline ext4_group_t defrag_group(struct super_block *sb)
{
	return E4DEFRAG_I(sb)->group;
}

/* check if inodes in @group have been scanned */
static inline bool defrag_scanned(struct super_block *sb, ext4_group_t group)
{
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return E4DEFRAG_GS(sb, group)->scanned;
}

/* check if defrag started */
static inline bool defrag_started(struct super_block *sb, ext4_group_t group)
{
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return E4DEFRAG_GS(sb, group)->started;
}

/* check if the extents in @group loaded */
static inline bool defrag_extents_loaded(struct super_block *sb,
					 ext4_group_t group)
{
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return E4DEFRAG_GS(sb, group)->loaded;
}

/* check if defrag paused */
static inline bool defrag_paused(struct super_block *sb, ext4_group_t group)
{
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return E4DEFRAG_GS(sb, group)->paused;
}

/* check if @group have defraged */
static inline bool defrag_done(struct super_block *sb, ext4_group_t group)
{
	struct defrag_group_state *dgs = E4DEFRAG_GS(sb, group);
	unsigned int interval = E4DEFRAG_I(sb)->interval;
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	if (!dgs->done) {
		return false;
	}
	return defrag_time_diff(dgs->end, jiffies) < interval;
}

/* check if defrag @group cost too much time, 10s */
static inline bool defrag_too_long(struct super_block *sb, ext4_group_t group)
{
	struct defrag_group_state *dgs = E4DEFRAG_GS(sb, group);
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return dgs->count == 0 || (dgs->cost / dgs->count >= 10000);
}

/* get the first free block in @group */
static inline ext4_grpblk_t defrag_first_free(struct super_block *sb,
					      ext4_group_t group, bool idx)
{
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return E4DEFRAG_GS(sb, group)->first_free[idx];
}

/* get the free blocks in @group */
static inline ext4_grpblk_t defrag_free(struct super_block *sb,
					ext4_group_t group, bool idx)
{
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return E4DEFRAG_GS(sb, group)->free[idx];
}

/* get the fragments in @group */
static inline ext4_grpblk_t defrag_fragments(struct super_block *sb,
					     ext4_group_t group, bool idx)
{
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return E4DEFRAG_GS(sb, group)->fragments[idx];
}

/* get the time cost to load extents of @group */
static inline unsigned long defrag_scan_cost(struct super_block *sb,
					     ext4_group_t group)
{
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return E4DEFRAG_GS(sb, group)->scan;
}

/* get the total time cost to defrag @group */
static inline unsigned long defrag_cost(struct super_block *sb,
					ext4_group_t group)
{
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return E4DEFRAG_GS(sb, group)->cost;
}

/* get number of defrag in @group */
static inline unsigned long defrag_count(struct super_block *sb,
					 ext4_group_t group)
{
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return E4DEFRAG_GS(sb, group)->count;
}

/* get the average duration to defrag @group */
static inline unsigned long defrag_duration(struct super_block *sb,
					    ext4_group_t group)
{
	struct defrag_group_state *dgs = E4DEFRAG_GS(sb, group);
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return dgs->done ? dgs->duration : defrag_time_diff(dgs->start,
							    jiffies);
}

/* get blocks moved to defrag @group */
static inline unsigned long defrag_moved(struct super_block *sb,
					 ext4_group_t group)
{
	BUG_ON(group >= E4DEFRAG_NGROUPS(sb));
	return E4DEFRAG_GS(sb, group)->moved;
}

/* extent routines */
struct free_extent_info {
	ext4_grpblk_t start;
	ext4_grpblk_t len;
};

static int get_free_extent(struct super_block *sb, ext4_group_t group,
			   ext4_grpblk_t start, ext4_grpblk_t len, void *priv)
{
	struct free_extent_info *fei = priv;
	fei->start = start;
	fei->len = len;
	return 1;
}

/* find the first free extent after @start in @group */
static ext4_grpblk_t e4defrag_next_free_extent(struct super_block *sb,
					       ext4_group_t group,
					       ext4_grpblk_t start,
					       ext4_grpblk_t * len)
{
	struct free_extent_info fei;
	fei.start = start;
	fei.len = 0;
	ext4_mballoc_query_range(sb, group, start, -1, get_free_extent, &fei);
	if (len) {
		*len = fei.len;
	}
	return fei.start;
}

/* sort by reverse order in pblk */
static inline int ex_cmp(struct defrag_extent *ex1, struct defrag_extent *ex2)
{
	if (ex1->pblk > ex2->pblk) {
		return -1;
	} else if (ex1->pblk < ex2->pblk) {
		return 1;
	} else {
		return 0;
	}
}

/* add defrag extent to extents tree */
static int e4defrag_add_extent(struct inode *inode, ext4_lblk_t lblk, int len,
			       ext4_fsblk_t pblk)
{
	struct defrag_extent *ex, *new_ex;
	ext4_group_t group;
	ext4_grpblk_t offset;
	struct group_extent_tree *tree = E4DEFRAG_ET(inode->i_sb);
	struct rb_node **n = &tree->root.rb_node;
	struct rb_node *parent = NULL;

	/* update group table */
	ext4_get_group_no_and_offset(inode->i_sb, pblk, &group, &offset);
	group_table_add_entry(inode->i_sb, group, len, inode->i_ino);

	/* only mantain the extents in defrag group */
	if (group != defrag_group(inode->i_sb)
	    || offset < defrag_first_free(inode->i_sb, group, 0)) {
		return 0;
	}

	new_ex = e4defrag_alloc_extent(inode->i_ino, lblk, len, pblk);
	if (!new_ex) {
		return -1;
	}
	while (*n) {
		int res;
		parent = *n;
		ex = rb_entry(parent, struct defrag_extent, node);
		res = ex_cmp(new_ex, ex);
		if (res < 0) {
			n = &(*n)->rb_left;
		} else if (res > 0) {
			n = &(*n)->rb_right;
		} else {
			e4defrag_err("extent overlap found\n");
			e4defrag_free_extent(new_ex);
			tree->len += len - ex->len;
			ex->lblk = lblk;
			ex->len = len;
			ex->pblk = pblk;
			return 0;
		}
	}
	rb_link_node(&new_ex->node, parent, n);
	rb_insert_color(&new_ex->node, &tree->root);
	tree->len += len;
	return len;
}

/* remove defrag extent from extents tree */
static inline void e4defrag_remove_extent(struct super_block *sb,
					  struct defrag_extent *ex)
{
	rb_erase(&ex->node, &E4DEFRAG_ET(sb)->root);
	e4defrag_free_extent(ex);
}

/* check if the block (@group, @offset) is free*/
static inline bool is_block_free(struct super_block *sb, ext4_group_t group,
				 ext4_grpblk_t offset)
{
	ext4_grpblk_t start, len;
	start = e4defrag_next_free_extent(sb, group, offset, &len);
	return start == offset && len > 0;
}

/**
 * @MC_NONE: all extent can be moved
 * @MC_LENGTH: extent which length matched can be moved
 * @MC_FREE_ONE: extent which length matched and one side free can be moved
 * @MC_FREE_BOTH: extent which length matched and both side free can be moved
 */
enum move_check {
	MC_NONE,
	MC_LENGTH,
	MC_FREE_ONE,
	MC_FREE_BOTH,
};

/* check if the extent can be moved */
static inline bool is_move_ok(struct super_block *sb, struct defrag_extent *ex,
			      ext4_grpblk_t len, enum move_check mc)
{
	bool left_is_free, right_is_free;
	ext4_group_t group;
	ext4_grpblk_t offset;
	if (mc == MC_NONE) {
		return true;
	}
	/* extent is too large */
	if (len < ex->len) {
		return false;
	}
	if (mc == MC_LENGTH) {
		return true;
	}
	ext4_get_group_no_and_offset(sb, ex->pblk, &group, &offset);
	/* check if the left of extent is free */
	left_is_free = (offset > 0) && is_block_free(sb, group, offset - 1);

	if (left_is_free && (mc == MC_FREE_ONE || len == ex->len)) {
		return true;
	}
	if (!left_is_free && mc == MC_FREE_BOTH && len != ex->len) {
		return false;
	}

	/* check if the right of extent is free */
	offset = offset + ex->len;
	right_is_free = (offset < EXT4_CLUSTERS_PER_GROUP(sb)) &&
	    is_block_free(sb, group, offset);

	if (right_is_free
	    && (left_is_free || mc == MC_FREE_ONE || len == ex->len)) {
		return true;
	}
	return false;
}

/* check and add 1 extent of inode to extent tree */
static bool add_extent_fn(struct extent_status *es, void *arg)
{
	struct inode *inode = (struct inode *)arg;
	ext4_fsblk_t es_pblk;
	int ret;
	if (ext4_es_is_delayed(es) || ext4_es_is_hole(es)) {
		es_pblk = 0;
	} else {
		es_pblk = ext4_es_pblock(es);
	}
	/* only load extent with pblk mapped */
	if (!es_pblk) {
		return false;
	}
	ret = e4defrag_add_extent(inode, es->es_lblk, es->es_len, es_pblk);
	return ret < 0 ? true : false;
}

static bool is_idle(struct super_block *sb);
static bool load_extents(struct inode *inode, void *priv)
{
	int depth, bits;
	ext4_lblk_t blocks;
	struct super_block *sb = inode->i_sb;
	if (!is_idle(sb)) {
		union defrag_state_data dsd;
		dsd.ino = inode->i_ino;
		defrag_update_state(sb, defrag_group(sb), DEFRAG_SCAN_PAUSE,
				    &dsd);
		return true;
	}
	e4defrag_dbg("load extents from ino: %lu\n", inode->i_ino);
	/* only load the extent of regualar file not inlined */
	if (!S_ISREG(inode->i_mode) || ext4_should_journal_data(inode) ||
	    IS_SWAPFILE(inode) || ext4_is_quota_file(inode) ||
	    ext4_has_inline_data(inode) || !inode->i_size ||
	    !ext4_test_inode_flag(inode, EXT4_INODE_EXTENTS)) {
		return false;
	}

	/* skip encrypted file without ICE */
	if (IS_ENCRYPTED(inode)) {
		int err = fscrypt_require_key(inode);
		if (err) {
			e4defrag_dbg("ino:%lu required key failed, err:%d\n",
				     inode->i_ino, err);
			return false;
		}
		/* TODO: */
		/*
		if (!fscrypt_using_hardware_encryption(inode)) {
			e4defrag_dbg("ino:%lu not ICE\n", inode->i_ino);
			return false;
		}
		*/
	}

	down_read(&EXT4_I(inode)->i_data_sem);
	depth = ext_depth(inode);
	/* special case: all extents in i_data */
	if (depth == 0) {
		struct ext4_extent_header *eh;
		struct ext4_extent *ex;
		eh = ext_inode_hdr(inode);
		for (ex = EXT_FIRST_EXTENT(eh); ex <= EXT_LAST_EXTENT(eh); ex++) {
			e4defrag_add_extent(inode,
					    le32_to_cpu(ex->ee_block),
					    ext4_ext_get_actual_len(ex),
					    ext4_ext_pblock(ex));
		}
		up_read(&EXT4_I(inode)->i_data_sem);
		return false;
	}
	up_read(&EXT4_I(inode)->i_data_sem);
	/* load the extent of the inode */
	bits = sb->s_blocksize_bits;
	blocks = (inode->i_size + (1 << bits) - 1) >> bits;
	ext4_query_extents_range(inode, 0, blocks, add_extent_fn,
				 (void *)inode);
	return false;
}

static inline bool defrag_extents_enough(struct super_block *sb,
					 ext4_group_t group)
{
	unsigned int found, expect;
	unsigned int first, free;

	first = defrag_first_free(sb, group, 0);
	free = defrag_free(sb, group, 0);
	expect = EXT4_CLUSTERS_PER_GROUP(sb) - first - free;
	found = E4DEFRAG_ET(sb)->len;
	e4defrag_dbg("group %u extents (%u,%u,%u,%u)", group, expect, found,
		     first, free);
	/* 1/4 of extents found */
	return found >= (expect >> 2);
}

/* build the extents tree of @group */
static bool e4defrag_load_extents(struct super_block *sb, ext4_group_t group)
{
	unsigned long start, end;
	unsigned long last_ino;
	int i;
	bool is_first = true;
	struct group_table *tbl = E4DEFRAG_GT(sb, group);
	struct group_entry *entry;

	last_ino = E4DEFRAG_I(sb)->last_ino;
	e4defrag_dbg("group %u last_ino: %lu\n", group, last_ino);
	/* load from group table first */
	group_table_for_each(tbl, entry, i) {
		if (defrag_scanned(sb, entry->group)) {
			start = group_entry_first_ino(sb, entry);
			end = group_entry_last_ino(sb, entry);
		} else {
			start = E4DEFRAG_FIRST_INO(sb, group);
			end = E4DEFRAG_LAST_INO(sb, group);
		}
		if (is_first) {
			/* we continue from last paused point */
			if ((last_ino < start) || (last_ino > end)) {
				continue;
			}
			start = last_ino;
			is_first = false;
		}
		ext4_query_inode_range(sb, start, end, load_extents, NULL);
		if (defrag_paused(sb, group)) {
			return false;
		}
		if (!defrag_scanned(sb, group)) {
			defrag_update_state(sb, entry->group,
					    DEFRAG_GROUP_SCANNED, NULL);
		}
	}
	if (!is_group_table_valid(sb, group)) {
		ext4_group_t ngroups, i, grp;
		grp = (last_ino - 1) / EXT4_INODES_PER_GROUP(sb);
		ngroups = E4DEFRAG_NGROUPS(sb);
		/* search group in reverse order from local group */
		for (i = 0; i < ngroups; i++, grp--) {
			if (i == 0) {
				start = last_ino;
			} else {
				if (grp == -1) {
					grp = ngroups - 1;
				}
				start = E4DEFRAG_FIRST_INO(sb, grp);
			}
			end = E4DEFRAG_LAST_INO(sb, grp);
			if (defrag_scanned(sb, grp)) {
				continue;
			}
			ext4_query_inode_range(sb, start, end,
					       load_extents, NULL);
			if (defrag_paused(sb, group)) {
				return false;
			}
			defrag_update_state(sb, grp, DEFRAG_GROUP_SCANNED,
					    NULL);
			/* check if enough extents found */
			if (is_group_table_valid(sb, group)) {
				break;
			}
		}
	}
	defrag_update_state(sb, group, DEFRAG_SCAN_DONE, NULL);
	return true;
}

/* destory the extent tree of group */
static void e4defrag_release_extents(struct super_block *sb, ext4_group_t group)
{
	struct group_extent_tree *tree = E4DEFRAG_ET(sb);
	struct rb_node *n = rb_first(&tree->root);
	struct defrag_extent *ex;
	while (n) {
		ex = rb_entry(n, struct defrag_extent, node);
		n = rb_next(n);
		e4defrag_remove_extent(sb, ex);
	}
	tree->len = 0;
}

/* move file range [@start, @start + len) of @ino to [@goal, @goal+@len) in disk */
static int do_move(struct super_block *sb, unsigned long ino,
		   ext4_lblk_t lblk, ext4_lblk_t len, ext4_fsblk_t goal,
		   ext4_lblk_t * moved_len)
{
	struct ext4_extent newext;
	struct ext4_ext_path *path;
	struct inode *orig_inode, *donor_inode;
	struct file orig_file, donor_file;
	handle_t *handle;
	__u64 moved;
	int credits, err;
	struct ext4_allocation_request ar;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
	orig_inode = ext4_iget_normal(sb, ino);
#else
	orig_inode = ext4_iget(sb, ino, EXT4_IGET_NORMAL);
#endif
	if (IS_ERR(orig_inode)) {
		err = PTR_ERR(orig_inode);
		e4defrag_err("fail to get original ino: %lu, err: %d\n", ino,
			     err);
		goto out;
	}
	err = fscrypt_require_key(orig_inode);
	if (err) {
		e4defrag_err("fail to require key for ino: %lu, err: %d\n", ino,
			     err);
		goto out_iput;
	}

	donor_inode = EXT4_SB(sb)->dfi.donor_inode;
	down_write(&EXT4_I(donor_inode)->i_data_sem);
	path = ext4_find_extent(donor_inode, 0, NULL, EXT4_EX_NOCACHE);
	up_write(&EXT4_I(donor_inode)->i_data_sem);
	if (IS_ERR(path)) {
		err = PTR_ERR(path);
		e4defrag_err("fail to get extpath of donor inode, err: %d\n",
			     err);
		goto out_put;
	}

	credits =
	    3 + ext4_ext_calc_credits_for_single_extent(donor_inode, len, path);
	handle = ext4_journal_start(donor_inode, EXT4_HT_MOVE_EXTENTS, credits);
	if (IS_ERR(handle)) {
		err = PTR_ERR(handle);
		e4defrag_err("fail to start journal, err: %d\n", err);
		goto out_put;
	}
	/* allocate [@goal, @goal + @len) for donor inode */
	ar.inode = donor_inode;
	ar.goal = goal;
	ar.len = len;
	ar.flags = EXT4_MB_HINT_DATA | EXT4_MB_HINT_NOPREALLOC |
	    EXT4_MB_HINT_GOAL_ONLY | EXT4_MB_HINT_TRY_GOAL;
	ext4_mb_new_blocks(handle, &ar, &err);
	if (err) {
		e4defrag_err("fail to reserve [%llu,%d), err: %d\n",
			     ar.goal, len, err);
		ext4_journal_stop(handle);
		goto out_put;
	}
	/* prepare extent for donor inode */
	newext.ee_block = cpu_to_le32(0);
	newext.ee_len = cpu_to_le16(ar.len);
	ext4_ext_mark_unwritten(&newext);
	ext4_ext_store_pblock(&newext, ar.goal);
	down_write(&EXT4_I(donor_inode)->i_data_sem);
	err = ext4_ext_insert_extent(handle, donor_inode, &path, &newext, 0);
	up_write(&EXT4_I(donor_inode)->i_data_sem);
	if (err) {
		e4defrag_err("fail to insert extent, err: %d\n", err);
		ext4_free_blocks(handle, donor_inode, NULL, goal, len,
				 EXT4_FREE_BLOCKS_FORGET);
		ext4_journal_stop(handle);
		goto out_put;
	}
	ext4_journal_stop(handle);

	err = ext4_inode_attach_jinode(orig_inode);
	if (err) {
		e4defrag_err("fail to attach jinode, err: %d\n", err);
		goto out_put;
	}
	/* swap the extent of inode and donor inode */
	orig_file.f_inode = orig_inode;
	donor_file.f_inode = donor_inode;
	err = ext4_move_extents(&orig_file, &donor_file, lblk, 0, len, &moved);
	e4defrag_dbg("moved: %llu/%u\n", moved, err);
	if (!err && moved_len) {
		*moved_len = moved;
	}
	down_write(&EXT4_I(donor_inode)->i_data_sem);
	ext4_es_remove_extent(donor_inode, 0, len);
	ext4_ext_remove_space(donor_inode, 0, len - 1);
	up_write(&EXT4_I(donor_inode)->i_data_sem);

 out_put:
	ext4_ext_drop_refs(path);
	kfree(path);
 out_iput:
	iput(orig_inode);
 out:
	return err;
}

static long read_bio_inflight(void);
static bool is_idle(struct super_block *sb)
{
	long inflight;
	if (defrag_force_mode(sb)) {
		return true;
	}
	inflight = read_bio_inflight();
	if (unlikely(inflight != 0)) {
		e4defrag_dbg("bio inflight: %ld\n", inflight);
		return false;
	}
	return true;
}

static int find_free_extent(struct super_block *sb, ext4_group_t group,
			    ext4_grpblk_t start, ext4_grpblk_t len, void *priv)
{
	struct free_extent_info *fei = priv;
	if (len >= fei->len) {
		fei->start = start;
		fei->len = len;
		return 1;
	}
	return 0;
}

/* find the first free extent can move to */
static ext4_grpblk_t e4defrag_find_free_extent(struct super_block *sb,
					       ext4_group_t group,
					       ext4_grpblk_t ex_blk,
					       ext4_grpblk_t ex_len,
					       ext4_grpblk_t * len)
{
	struct free_extent_info fei;
	fei.start = ex_blk;
	fei.len = ex_len;
	ext4_mballoc_query_range(sb, group, 0, ex_blk, find_free_extent, &fei);
	if (fei.start < ex_blk) {
		if (len) {
			*len = fei.len;
		}
		return fei.start;
	}
	return -1;
}

/* defrag one group and return blocks moved */
static ext4_lblk_t do_defrag(struct super_block *sb, ext4_group_t group)
{
	struct ext4_defrag_info *dfi = E4DEFRAG_I(sb);
	struct rb_node *n;
	struct defrag_extent *ex;
	ext4_grpblk_t len = 0;
	unsigned long moved = 0;
	ext4_fsblk_t base, goal;
	/* do not defrag the group if too many extents unknown */
	if (!defrag_extents_enough(sb, group)) {
		e4defrag_msg("group %u not enough extents found\n", group);
		/* this means group table is obsolete, we need to reset table */
		if (is_group_table_valid(sb, group)) {
			ext4_group_t i;
			for (i = 0; i < E4DEFRAG_NGROUPS(sb); i++) {
				defrag_update_state(sb, i, DEFRAG_SCAN_RESET,
						    NULL);
			}
		}
		return 0;
	}

	base = ext4_group_first_block_no(sb, group);
	n = rb_first(&dfi->tree.root);
	while (n) {
		if (!is_idle(sb)) {
			union defrag_state_data dsd;
			dsd.moved = moved;
			defrag_update_state(sb, group, DEFRAG_PAUSE, &dsd);
			break;
		}

		ex = rb_entry(n, struct defrag_extent, node);
		if (!is_move_ok(sb, ex, dfi->max_len, MC_FREE_ONE)) {
			n = rb_next(n);
			e4defrag_remove_extent(sb, ex);
			continue;
		}
		e4defrag_dbg("ex[%u,%u/%llu/%u)\n", ex->lblk, ex->len, ex->pblk,
			     ex->ino);
		/* find the free extent move to */
		goal =
		    e4defrag_find_free_extent(sb, group, ex->pblk - base,
					      ex->len, &len);
		if (goal == -1) {
			n = rb_next(n);
			e4defrag_remove_extent(sb, ex);
			continue;
		}
		goal += base;
		e4defrag_dbg("fex[%llu,%u)\n", goal, len);
		len = min_t(ext4_lblk_t, len, ex->len);
		if (!do_move(sb, ex->ino, ex->lblk, len, goal, &len)) {
			moved += len;
		}
		/* partial moved */
		if (ex->len > len) {
			ex->lblk += len;
			ex->pblk += len;
			ex->len -= len;
		} else {
			n = rb_next(n);
			e4defrag_remove_extent(sb, ex);
		}
	}
	e4defrag_dbg("total moved: %lu\n", moved);
	return moved;
}

#ifdef DEBUG
struct freefrag_info {
	ext4_grpblk_t first_free;
	ext4_grpblk_t free;
	ext4_grpblk_t fragments;
	ext4_grpblk_t counters[8];
};

static int acct_freefrag(struct super_block *sb, ext4_group_t group,
			 ext4_grpblk_t start, ext4_grpblk_t len, void *priv)
{
	struct freefrag_info *ffi = priv;
	int order;
	if (ffi->first_free == -1) {
		ffi->first_free = start;
	}
	ffi->free += len;
	ffi->fragments++;
	order = fls(len) - 1;
	if (order > 7) {
		order = 7;
	}
	ffi->counters[order]++;

	return 0;
}

static void show_freefrag(struct super_block *sb, ext4_group_t group)
{
	struct freefrag_info ffi;
	memset(&ffi, 0, sizeof(ffi));
	ffi.first_free = -1;
	ext4_mballoc_query_range(sb, group, 0, -1, acct_freefrag, &ffi);
	e4defrag_msg("group %u free stats: %u %u %u\n", group,
		     ffi.first_free, ffi.free, ffi.fragments);
	e4defrag_msg("fragments: %u %u %u %u %u %u %u\n", ffi.counters[0],
		     ffi.counters[1], ffi.counters[2], ffi.counters[3],
		     ffi.counters[4], ffi.counters[5], ffi.counters[6]);
	e4defrag_msg("defrag stats: %lu %lu %lu %lu %lu",
		     defrag_count(sb, group), defrag_moved(sb, group),
		     defrag_scan_cost(sb, group), defrag_cost(sb, group),
		     defrag_duration(sb, group));
}
#else
static inline void show_freefrag(struct super_block *sb, ext4_group_t group)
{
};
#endif

/* get the free block stats of @group */
bool get_free_stats(struct ext4_group_info *grp, ext4_group_t group, void *priv)
{
	union defrag_state_data *dsd = priv;
	dsd->first_free = grp->bb_first_free;
	dsd->free = grp->bb_free;
	dsd->fragments = grp->bb_fragments;
	return true;
}

/* defrag one block group: return true if done, false if interrupted */
static bool defrag_one_group(struct super_block *sb, ext4_group_t group)
{
	unsigned long moved;
	/* continue defrag if paused */
	if (defrag_paused(sb, group)) {
		defrag_update_state(sb, group, DEFRAG_CONTINUE, NULL);
	}

	/* load group extents if needed */
	if (!defrag_extents_loaded(sb, group)) {
		e4defrag_load_extents(sb, group);
		if (defrag_paused(sb, group)) {
			return false;
		}
	}
	show_freefrag(sb, group);
	/* move data to do defrag */
	moved = do_defrag(sb, group);
	show_freefrag(sb, group);
	/* release group extents if done */
	if (!defrag_paused(sb, group) || defrag_too_long(sb, group)) {
		union defrag_state_data dsd;
		e4defrag_release_extents(sb, group);
		/* query the group free block state after defrag */
		memset(&dsd, 0, sizeof(dsd));
		ext4_mb_query_group_info(sb, group, 1, get_free_stats, &dsd,
					 false);
		if (!defrag_paused(sb, group)) {
			dsd.moved_ = moved;
		}
		defrag_update_state(sb, group, DEFRAG_DONE, &dsd);
		return true;
	}

	return false;
}

/**
 * @GSP_FIRST_MATCH: select the first group satisfy free/score threshold
 * @GSP_MAX_FRAGMENTS: select the group with max fragments
 * @GSP_MAX_SCORE: select the group with max fragmentation score
 */
enum bg_select_policy {
	GSP_FIRST_MATCH,
	GSP_MAX_FRAGMENTS,
	GSP_MAX_SCORE,
};

struct bg_select_data {
	struct super_block *sb;
	enum bg_select_policy policy;
	bool reverse;
	ext4_group_t group, next;
	unsigned int min_score;
	unsigned int min_free;
	/* stats of selected group */
	unsigned int score;
	unsigned int first_free;
	unsigned int free;
	unsigned int fragments;
};

/* init the group select data */
static inline void bg_select_data_init(struct super_block *sb,
				       struct bg_select_data *gsd,
				       enum bg_select_policy policy)
{
	struct ext4_defrag_info *dfi = E4DEFRAG_I(sb);
	gsd->sb = sb;
	gsd->policy = policy;
	gsd->reverse = true;
	gsd->min_free = dfi->min_free;
	gsd->min_score = FRAG_SCORE(dfi->min_score);
	gsd->group = -1;
	gsd->next = 0;
	gsd->score = 0;
	gsd->free = 0;
	gsd->fragments = 0;
}

bool need_defrag(struct super_block *sb, unsigned free, unsigned fragments)
{
	struct ext4_defrag_info *dfi = E4DEFRAG_I(sb);
	unsigned score;
	/* skip group if free space too small */
	if (free <= dfi->min_free) {
		return false;
	}
	score = (fragments - 1) * FRAG_SCORE_FACTOR / free;
	/* skip group not fragmented */
	if (score < dfi->min_score) {
		return false;
	}
	return true;
}

/* select group to do defrag */
bool defrag_select_group(struct ext4_group_info * grp,
			 ext4_group_t group, void *priv)
{
	struct bg_select_data *gsd = priv;
	struct ext4_defrag_info *dfi = E4DEFRAG_I(gsd->sb);
	unsigned int score;

	if (unlikely(group >= dfi->ngroups)) {
		struct defrag_group_state *groups;
		ext4_group_t grp, ngroups;
		e4defrag_msg("group number exceed ngroups (%u, %u)",
			     group, dfi->ngroups);
		ngroups = ext4_get_groups_count(gsd->sb);
		if (ngroups > dfi->ngroups) {
			groups =
			    kvmalloc(ngroups *
				     sizeof(struct defrag_group_state),
				     GFP_KERNEL);
			if (ZERO_OR_NULL_PTR(groups)) {
				return false;
			}
			if (dfi->groups) {
				memcpy(groups, dfi->groups,
					dfi->ngroups * sizeof(*groups));
				kvfree(dfi->groups);
			}
			dfi->groups = groups;
			for (grp = dfi->ngroups; grp < ngroups; grp++) {
				defrag_update_state(gsd->sb, grp, DEFRAG_INIT,
						    NULL);
			}
			dfi->ngroups = ngroups;
		}
	}

	gsd->next = gsd->reverse ? group - 1 : group + 1;
	/* skip group if free space too small */
	if (grp->bb_free <= gsd->min_free) {
		return false;
	}
	/* skip group if the group defragged recently */
	if (defrag_done(gsd->sb, group)) {
		return false;
	}
	score = (grp->bb_fragments - 1) * FRAG_SCORE_FACTOR / grp->bb_free;
	/* skip group not fragmented */
	if (score < gsd->min_score) {
		return false;
	}
	switch (gsd->policy) {
	case GSP_FIRST_MATCH:
		gsd->group = group;
		gsd->score = score;
		gsd->first_free = grp->bb_first_free;
		gsd->free = grp->bb_free;
		gsd->fragments = grp->bb_fragments;
		return true;
	case GSP_MAX_FRAGMENTS:
		if (grp->bb_fragments > gsd->fragments) {
			if (gsd->group != -1) {
				gsd->next = gsd->group;
			}
			gsd->group = group;
			gsd->score = score;
			gsd->first_free = grp->bb_first_free;
			gsd->free = grp->bb_free;
			gsd->fragments = grp->bb_fragments;
		}
		return false;
	case GSP_MAX_SCORE:
		if (score > gsd->score) {
			if (gsd->group != -1) {
				gsd->next = gsd->group;
			}
			gsd->group = group;
			gsd->score = score;
			gsd->first_free = grp->bb_first_free;
			gsd->free = grp->bb_free;
			gsd->fragments = grp->bb_fragments;
		}
		return false;
	default:
		e4defrag_err("unknown defrag group select policy: %u\n",
			     gsd->policy);
		BUG_ON(1);
	}
	return false;
}

/* defrag thread flag */
#define PT_E4DEFRAG_BIT (29)
#define PT_E4DEFRAG_THREAD (1 << PT_E4DEFRAG_BIT)
static inline void mark_e4defrag_thread(void)
{
	current->ptrace |= PT_E4DEFRAG_THREAD;
}

static inline void clear_e4defrag_thread(void)
{
	current->ptrace &= ~PT_E4DEFRAG_THREAD;
}

static inline bool is_e4defrag_thread(void)
{
	return current->ptrace & PT_E4DEFRAG_THREAD;
}

/* we need session keyring to encrypt/decrypt file */
static struct key *e4defrag_install_session_keyring(pid_t pid)
{
	struct task_struct *task;
	struct cred *new;
	struct key *key = NULL;
	task = get_pid_task(find_get_pid(pid), PIDTYPE_PID);
	if (!task) {
		e4defrag_err("fail to find task, pid=%d\n", pid);
		goto out;
	}
	new = prepare_kernel_cred(current);
	if (!new) {
		e4defrag_err("fail to prepare new cred\n");
		goto out_put;
	}
	key = key_get(task_cred_xxx(task, session_keyring));
	key_put(current_cred_xxx(session_keyring));
	rcu_assign_pointer(new->session_keyring, key);
	commit_creds(new);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0))
	e4defrag_dbg("key usage: %d\n", refcount_read(&key->usage));
#else
	e4defrag_dbg("key usage: %d\n", atomic_read(&key->usage));
#endif
 out_put:
	put_task_struct(task);
 out:
	return key;
}

static inline void set_sched_policy(void)
{
	cpumask_t allowed_mask = CPU_MASK_NONE;
	unsigned int cpu;
	/* set nice as THREAD_PRIORITY_BACKGROUND */
	set_user_nice(current, 10);
	/* bound to little cluster */
	for (cpu = 0; cpu < 4; cpu++) {
		cpumask_set_cpu(cpu, &allowed_mask);
	}
	set_cpus_allowed_ptr(current, &allowed_mask);
}

/* main function of defrag thread */
static int e4defrag_func(void *data)
{
	struct super_block *sb = (struct super_block *)data;
	struct ext4_defrag_info *dfi = E4DEFRAG_I(sb);
	ext4_group_t first_group = dfi->ngroups - 1, nr_skip = 0;
	unsigned int wait_ms;
	struct bg_select_data gsd;
	bool done = true;

	wait_ms = dfi->sleep_time;
	set_sched_policy();
	set_freezable();
	mark_e4defrag_thread();
	dfi->task = current;
	wake_up(&dfi->init_wq);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(4, 19, 0))
	/* not call into fs layer in mem allocation path */
	memalloc_nofs_save();
#endif
	/* we get the session keyring from init process and install it */
	e4defrag_install_session_keyring(1);
 loop:
	wait_event_interruptible_timeout(dfi->wq, kthread_should_stop()
					 || freezing(current)
					 || (dfi->wake && !ext4_defrag_protect),
					 msecs_to_jiffies(wait_ms));
	if (freezing(current)) {
		e4defrag_dbg("suspending online defrag thread\n");
		try_to_freeze();
	} else if (kthread_should_stop() ||
		((ext4_defrag_protect == DEFRAG_PROTECT_EOL))) {
		e4defrag_dbg("exiting online defrag thread\n");
		clear_e4defrag_thread();
		free_e4defrag_thread(dfi);
		e4defrag_msg("online defrag thread exited\n");
		return 0;
	} else if (ext4_defrag_protect == DEFRAG_PROTECT_LOWPOWER) {
		e4defrag_dbg("protecting online defrag thread as low power\n");
	} else {
		ext4_group_t nr_to_scan =
		    min_t(ext4_group_t, dfi->ngroups, dfi->nr_to_scan);
		e4defrag_dbg("defrag wake up: %u\n", dfi->wake);
		if (dfi->wake) {
			dfi->wake = WAKE_TIMEOUT;
		}
		/* find a group to defrag */
		if (done) {
			bg_select_data_init(sb, &gsd, dfi->policy);
			ext4_mb_query_group_info(sb, first_group,
						 nr_to_scan,
						 defrag_select_group, &gsd,
						 gsd.reverse);
			first_group = gsd.next;
			if (gsd.group != -1) {
				union defrag_state_data dsd;
				dfi->state = DS_RUNNING;
				dsd.first_free = gsd.first_free;
				dsd.free = gsd.free;
				dsd.fragments = gsd.fragments;
				defrag_update_state(sb, gsd.group,
						    DEFRAG_START, &dsd);
			}
		}
		if (gsd.group != -1) {
			nr_skip = 0;
			e4defrag_msg("defrag group %u (%u, %u, %u)\n",
				     gsd.group, gsd.free, gsd.fragments,
				     NORM_SCORE(gsd.score));
			done = defrag_one_group(sb, gsd.group);
			if (defrag_force_mode(sb)) {
				wait_ms = 0;
			} else if (done) {
				wait_ms = dfi->sleep_time;
			} else {
				wait_ms = dfi->min_sleep_time;
			}
		} else {
			nr_skip += nr_to_scan;
			if (nr_skip >= dfi->ngroups) {
				/* all groups scanned, wait for max interval */
				nr_skip = 0;
				dfi->state = DS_DONE;
				e4defrag_dbg("no group need to defrag now\n");
				wait_ms = dfi->max_sleep_time;
			} else {
				/* scan next batch of groups */
				e4defrag_dbg("check groups [%u, %u)\n",
					     first_group, nr_to_scan);
				if (defrag_force_mode(sb)) {
					wait_ms = 0;
				} else {
					wait_ms = dfi->min_sleep_time;
				}
			}
		}
	}

	if (!kthread_should_stop()) {
		goto loop;
	}

	clear_e4defrag_thread();
	free_e4defrag_thread(dfi);
	e4defrag_msg("online defrag thread exit\n");
	return 0;
}

/* create donor inode for defrag */
static struct inode *e4defrag_create_donor_inode(struct super_block *sb)
{
	handle_t *handle;
	struct inode *inode, *dir = d_inode(sb->s_root);
	int credits = (EXT4_DATA_TRANS_BLOCKS(sb) +
		       EXT4_INDEX_EXTRA_TRANS_BLOCKS + 3);
	handle = ext4_journal_start(dir, EXT4_HT_MOVE_EXTENTS, credits);
	if (IS_ERR(handle)) {
		e4defrag_err("fail to start journal, err: %ld\n",
			     PTR_ERR(handle));
		return ERR_PTR(PTR_ERR(handle));
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0))
	inode = ext4_new_inode(handle, dir, S_IFREG, NULL, 0, NULL);
#else
	inode = ext4_new_inode(handle, dir, S_IFREG, NULL, 0, NULL, 0);
#endif
	if (IS_ERR(inode)) {
		e4defrag_err("fail to create donor inode, err: %ld\n",
			     PTR_ERR(inode));
		return ERR_PTR(PTR_ERR(handle));
		ext4_journal_stop(handle);
		return inode;
	}
	clear_nlink(inode);
	i_size_write(inode, DEFRAG_MAX_LEN * EXT4_BLOCK_SIZE(sb));
	ext4_ext_tree_init(handle, inode);
	/* ext4_set_aops(inode); */
	ext4_orphan_add(handle, inode);
	ext4_journal_stop(handle);
	unlock_new_inode(inode);
	e4defrag_dbg("create donor inode: %lu\n",
		     IS_ERR(inode) ? -1 : inode->i_ino);
	return inode;
}

/* unlink the donor inode */
static void e4defrag_delete_donor_inode(struct ext4_defrag_info *dfi)
{
	struct inode *inode = dfi->donor_inode;
	dfi->donor_inode = NULL;
	if (inode) {
		iput(inode);
	}
}

static int e4defrag_start_thread(struct super_block *sb)
{
	struct ext4_defrag_info *dfi = E4DEFRAG_I(sb);
	struct task_struct *t;
	ext4_group_t ngroups, group;
	e4defrag_msg("start online defrag thread");
	init_waitqueue_head(&dfi->init_wq);
	init_waitqueue_head(&dfi->wq);
	dfi->force_mode = 0;
	dfi->state = DS_INIT;
	dfi->min_sleep_time = DEFRAG_MIN_SLEEP_TIME;
	dfi->sleep_time = DEFRAG_SLEEP_TIME;
	dfi->max_sleep_time = DEFRAG_MAX_SLEEP_TIME;
	dfi->interval = DEFRAG_INTERVAL;
	dfi->max_len = DEFRAG_MAX_LEN;
	dfi->min_score = DEFRAG_MIN_SCORE;
	dfi->min_free = DEFRAG_MIN_FREE(sb);
	dfi->nr_to_scan = DEFRAG_NR_TO_SCAN;
	dfi->policy = GSP_FIRST_MATCH;
	dfi->group = -1;
	dfi->tree.len = 0;
	dfi->tree.root = RB_ROOT;
	ngroups = ext4_get_groups_count(sb);
	dfi->groups =
	    kvmalloc(ngroups * sizeof(struct defrag_group_state), GFP_KERNEL);
	if (ZERO_OR_NULL_PTR(dfi->groups)) {
		return -1;
	}
	for (group = 0; group < ngroups; group++) {
		defrag_update_state(sb, group, DEFRAG_INIT, NULL);
	}
	dfi->ngroups = ngroups;
	dfi->donor_inode = e4defrag_create_donor_inode(sb);
	if (IS_ERR(dfi->donor_inode)) {
		return -1;
	}

	t = kthread_run(e4defrag_func, sb, "e4defrag/%s", sb->s_id);
	if (IS_ERR(t)) {
		e4defrag_err("fail to start online defrag thread\n");
		return PTR_ERR(t);
	}

	wait_event(dfi->init_wq, dfi->task != NULL);
	return 0;
}


static void free_e4defrag_thread(struct ext4_defrag_info *dfi)
{
	e4defrag_msg("free online defrag groups resource\n");
	dfi->ngroups = 0;
	kvfree(dfi->groups);
	e4defrag_delete_donor_inode(dfi);
	dfi->task = NULL;
}

static void e4defrag_stop_thread(struct super_block *sb)
{
	struct ext4_defrag_info *dfi = E4DEFRAG_I(sb);
	e4defrag_msg("stop online defrag thread\n");
	kthread_stop(dfi->task);
}

void e4defrag_wake_up_thread(struct super_block *sb)
{
	struct ext4_defrag_info *dfi = E4DEFRAG_I(sb);
	static unsigned long next_time = 0;
	bool wakeup = dfi->task && !is_e4defrag_thread()
	    && time_is_before_eq_jiffies(next_time)
	    && is_idle(sb) && wq_has_sleeper(&dfi->wq);
	if (wakeup) {
		next_time = jiffies + msecs_to_jiffies(dfi->sleep_time);
		dfi->wake = WAKE_SYS;
		wake_up_interruptible(&dfi->wq);
	}
}

/* parameters control defrag, in /sys/fs/ext4/<disk>/defrag/ */
typedef enum {
	attr_pointer_ui,
	attr_group_select_policy,
	attr_wake,
	attr_force_mode,
	attr_state,
	attr_stats,
} attr_id_t;

typedef enum {
	ptr_explicit,
	ptr_ext4_defrag_info_offset,
} attr_ptr_t;

struct e4defrag_attr {
	struct attribute attr;
	short attr_id;
	short attr_ptr;
	union {
		int offset;
		void *explicit_ptr;
	} u;
};

#define E4DEFRAG_ATTR(_name, _mode, _id)					\
static struct e4defrag_attr e4defrag_attr_##_name = {			\
	.attr = {.name = __stringify(_name), .mode = _mode },		\
	.attr_id = attr_##_id,						\
}
#define E4DEFRAG_ATTR_FUNC(_name, _mode)  E4DEFRAG_ATTR(_name, _mode, _name)
#define E4DEFRAG_ATTR_OFFSET(_name, _mode, _id, _struct, _elname)	\
static struct e4defrag_attr e4defrag_attr_##_name = {		\
	.attr = {.name = __stringify(_name), .mode = _mode },	\
	.attr_id = attr_##_id,					\
	.attr_ptr = ptr_##_struct##_offset,			\
	.u = {							\
		.offset = offsetof(struct _struct, _elname),	\
	},							\
}
#define ATTR_LIST(name) &e4defrag_attr_##name.attr
#define E4DEFRAG_RO_ATTR(_name, _elname)  \
	E4DEFRAG_ATTR_OFFSET(_name, 0444, pointer_ui, ext4_defrag_info, _elname)
#define E4DEFRAG_RW_ATTR(_name, _elname)  \
	E4DEFRAG_ATTR_OFFSET(_name, 0644, pointer_ui, ext4_defrag_info, _elname)

E4DEFRAG_RW_ATTR(min_sleep_time, min_sleep_time);
E4DEFRAG_RW_ATTR(sleep_time, sleep_time);
E4DEFRAG_RW_ATTR(max_sleep_time, max_sleep_time);
E4DEFRAG_RW_ATTR(interval, interval);
E4DEFRAG_RW_ATTR(min_score, min_score);
E4DEFRAG_RW_ATTR(min_free_blocks, min_free);
E4DEFRAG_RW_ATTR(nr_to_scan, nr_to_scan);
E4DEFRAG_RW_ATTR(max_extent_size, max_len);
E4DEFRAG_RO_ATTR(current_group, group);
E4DEFRAG_ATTR_FUNC(group_select_policy, 0644);
E4DEFRAG_ATTR_FUNC(wake, 0200);
E4DEFRAG_ATTR_FUNC(force_mode, 0644);
E4DEFRAG_ATTR_FUNC(state, 0444);
E4DEFRAG_ATTR_FUNC(stats, 0444);

static struct attribute *e4defrag_attrs[] = {
	ATTR_LIST(min_sleep_time),
	ATTR_LIST(sleep_time),
	ATTR_LIST(max_sleep_time),
	ATTR_LIST(interval),
	ATTR_LIST(min_score),
	ATTR_LIST(min_free_blocks),
	ATTR_LIST(nr_to_scan),
	ATTR_LIST(max_extent_size),
	ATTR_LIST(current_group),
	ATTR_LIST(group_select_policy),
	ATTR_LIST(wake),
	ATTR_LIST(force_mode),
	ATTR_LIST(state),
	ATTR_LIST(stats),
	NULL,
};

static ssize_t group_select_policy_show(struct ext4_defrag_info *dfi, char *buf)
{
	const char *policy;
	switch (dfi->policy) {
	case GSP_FIRST_MATCH:
		policy = "first match";
		break;
	case GSP_MAX_FRAGMENTS:
		policy = "max fragments";
		break;
	case GSP_MAX_SCORE:
		policy = "max score";
		break;
	default:
		policy = "unknown";
		break;
	}
	return snprintf(buf, PAGE_SIZE, "%s\n", policy);
}

static ssize_t group_select_policy_store(struct ext4_defrag_info *dfi,
					 const char *buf, size_t len)
{
	unsigned long t;
	int ret;
	ret = kstrtoul(skip_spaces(buf), 0, &t);
	if (ret) {
		return ret;
	}
	switch (t) {
	case GSP_FIRST_MATCH:
	case GSP_MAX_FRAGMENTS:
	case GSP_MAX_SCORE:
		dfi->policy = t;
		return len;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static ssize_t state_show(struct ext4_defrag_info *dfi, char *buf)
{
	const char *state;
	switch (dfi->state) {
	case DS_INIT:
		state = "init";
		break;
	case DS_RUNNING:
		state = "running";
		break;
	case DS_DONE:
		state = "done";
		break;
	default:
		state = "unknown";
		break;
	}
	return snprintf(buf, PAGE_SIZE, "%s\n", state);
}

static ssize_t stats_show(struct ext4_defrag_info *dfi, char *buf)
{
	struct super_block *sb = E4DEFRAG_SB(dfi);
	unsigned long moved, cost, count, read, write;
	ext4_group_t group;
	moved = 0;
	cost = 0;
	count = 0;
	for (group = 0; group < dfi->ngroups; group++) {
		if (defrag_count(sb, group)) {
			cost += defrag_cost(sb, group);
			moved += defrag_moved(sb, group);
			count += defrag_count(sb, group);
		}
	}
	moved <<= (EXT4_BLOCK_SIZE_BITS(E4DEFRAG_SB(dfi)) - 10);
	get_task_struct(dfi->task);
	read = task_io_get_inblock(dfi->task) >> 1;
	write = task_io_get_oublock(dfi->task) >> 1;
	put_task_struct(dfi->task);
	return snprintf(buf, PAGE_SIZE, "moved: %lu\nread: %lu\nwrite: %lu\n"
			"cost: %lu\ncount: %lu\n", moved, read, write, cost,
			count);
}

static void *calc_ptr(struct e4defrag_attr *a, struct ext4_defrag_info *dfi)
{
	switch (a->attr_ptr) {
	case ptr_explicit:
		return a->u.explicit_ptr;
	case ptr_ext4_defrag_info_offset:
		return (void *)(((char *)dfi) + a->u.offset);
	}
	return NULL;
}

static ssize_t e4defrag_attr_show(struct kobject *kobj,
				  struct attribute *attr, char *buf)
{
	struct ext4_defrag_info *dfi =
	    container_of(kobj, struct ext4_defrag_info,
			 kobj);
	struct e4defrag_attr *a =
	    container_of(attr, struct e4defrag_attr, attr);
	void *ptr = calc_ptr(a, dfi);
	switch (a->attr_id) {
	case attr_pointer_ui:
		if (!ptr) {
			return 0;
		}
		return snprintf(buf, PAGE_SIZE, "%u\n", *((unsigned int *)ptr));
	case attr_group_select_policy:
		return group_select_policy_show(dfi, buf);
	case attr_wake:
		return -EINVAL;
	case attr_force_mode:
		return snprintf(buf, PAGE_SIZE, "%u\n", dfi->force_mode);
	case attr_state:
		return state_show(dfi, buf);
	case attr_stats:
		return stats_show(dfi, buf);
	}

	return 0;
}

static ssize_t e4defrag_attr_store(struct kobject *kobj,
				   struct attribute *attr,
				   const char *buf, size_t len)
{
	struct ext4_defrag_info *dfi =
	    container_of(kobj, struct ext4_defrag_info,
			 kobj);
	struct e4defrag_attr *a =
	    container_of(attr, struct e4defrag_attr, attr);
	void *ptr = calc_ptr(a, dfi);
	unsigned long t;
	int ret;
	switch (a->attr_id) {
	case attr_pointer_ui:
		if (!ptr) {
			return 0;
		}
		ret = kstrtoul(skip_spaces(buf), 0, &t);
		if (ret) {
			return ret;
		}
		*((unsigned int *)ptr) = t;
		return len;
	case attr_group_select_policy:
		return group_select_policy_store(dfi, buf, len);
	case attr_wake:
		ret = kstrtoul(skip_spaces(buf), 0, &t);
		if (ret) {
			return ret;
		}
		if (t != 0) {
			dfi->wake = WAKE_USER;
			wake_up_interruptible(&dfi->wq);
		}
		return len;
	case attr_force_mode:
		ret = kstrtoul(skip_spaces(buf), 0, &t);
		if (ret) {
			return ret;
		}
		dfi->force_mode = t ? 1 : 0;
		return len;
	}
	return 0;
}

static void e4defrag_kobject_release(struct kobject *kobj)
{
	e4defrag_dbg("release defrag kobject\n");
	memset(kobj, 0, sizeof(*kobj));
}

static const struct sysfs_ops e4defrag_attr_ops = {
	.show = e4defrag_attr_show,
	.store = e4defrag_attr_store,
};

static struct kobj_type e4defrag_ktype = {
	.default_attrs = e4defrag_attrs,
	.sysfs_ops = &e4defrag_attr_ops,
	.release = e4defrag_kobject_release,
};

/* /proc/fs/ext4/<disk>/defrag_groups_state */
static void *e4defrag_seq_groups_start(struct seq_file *seq, loff_t * pos)
{
	struct ext4_defrag_info *dfi = PDE_DATA(file_inode(seq->file));
	ext4_group_t group;

	if (*pos < 0 || *pos >= dfi->ngroups) {
		return NULL;
	}
	group = *pos + 1;
	return (void *)((unsigned long)group);
}

static void *e4defrag_seq_groups_next(struct seq_file *seq, void *v,
				      loff_t * pos)
{
	struct ext4_defrag_info *dfi = PDE_DATA(file_inode(seq->file));
	ext4_group_t group;

	++*pos;
	if (*pos < 0 || *pos >= dfi->ngroups) {
		return NULL;
	}
	group = *pos + 1;
	return (void *)((unsigned long)group);
}

static int e4defrag_seq_groups_show(struct seq_file *seq, void *v)
{
	struct ext4_defrag_info *dfi = PDE_DATA(file_inode(seq->file));
	struct super_block *sb = E4DEFRAG_SB(dfi);
	ext4_group_t group = (ext4_group_t) ((unsigned long)v);

	group--;
	if (group == 0) {
		seq_puts(seq, "group,count,moved,scan-cost,total-cost,duration,"
			 "first0,free0,frags0,first1,free1,frags1\n");
	}

	/* skip group not been defragged */
	if (!defrag_count(sb, group)) {
		return 0;
	}
	seq_printf(seq, "%u,%lu,%lu,%lu,%lu,%lu,%u,%u,%u,%u,%u,%u\n", group,
		   defrag_count(sb, group), defrag_moved(sb, group),
		   defrag_scan_cost(sb, group), defrag_cost(sb, group),
		   defrag_duration(sb, group), defrag_first_free(sb, group, 0),
		   defrag_free(sb, group, 0), defrag_fragments(sb, group, 0),
		   defrag_first_free(sb, group, 1), defrag_free(sb, group, 1),
		   defrag_fragments(sb, group, 1));
	return 0;
}

static void e4defrag_seq_groups_stop(struct seq_file *seq, void *v)
{
}

const struct seq_operations e4defrag_seq_groups_ops = {
	.start = e4defrag_seq_groups_start,
	.next = e4defrag_seq_groups_next,
	.stop = e4defrag_seq_groups_stop,
	.show = e4defrag_seq_groups_show,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
static int e4defrag_groups_open(struct inode *inode, struct file *file)
{
	struct ext4_defrag_info *dfi = PDE_DATA(inode);
	int rc;

	rc = seq_open(file, &e4defrag_seq_groups_ops);
	if (!rc) {
		struct seq_file *m = file->private_data;
		m->private = dfi;
	}
	return rc;
}

static const struct file_operations e4defrag_seq_groups_fops = {
	.open = e4defrag_groups_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};
#endif

int e4defrag_init(struct super_block *sb)
{
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	struct ext4_defrag_info *dfi = E4DEFRAG_I(sb);
	int ret;
	if (!ext4_has_feature_extents(sb)
	    || ext4_has_feature_bigalloc(sb)) {
		e4defrag_err("online defrag not supported\n");
		return -ENOTSUPP;
	}

	if (dfi->task) {
		e4defrag_dbg("online defrag thread already started\n");
		return 0;
	}
	ret = e4defrag_start_thread(sb);
	if (!ret) {
		ret = kobject_init_and_add(&dfi->kobj, &e4defrag_ktype,
					   &sbi->s_kobj, "defrag");
	}
	if (!ret && sbi->s_proc) {
#if (LINUX_VERSION_CODE < KERNEL_VERSION(4, 19, 0))
		proc_create_data("defrag_groups_state", S_IRUGO, sbi->s_proc,
				 &e4defrag_seq_groups_fops, dfi);
#else
		proc_create_seq_data("defrag_groups_state", S_IRUGO,
				     sbi->s_proc, &e4defrag_seq_groups_ops,
				     dfi);
#endif
	}
	return ret;
}

void e4defrag_exit(struct super_block *sb)
{
	struct ext4_sb_info *sbi = EXT4_SB(sb);
	struct ext4_defrag_info *dfi = E4DEFRAG_I(sb);
	if (!dfi->task) {
		return;
	}
	if (sbi->s_proc) {
		remove_proc_entry("defrag_groups_state", sbi->s_proc);
	}
	kobject_put(&dfi->kobj);
	e4defrag_stop_thread(sb);
}

/* count the bio queued and completed, for device idle check */
static DEFINE_PER_CPU(local_t, bio_inflight) = LOCAL_INIT(0);

static long read_bio_inflight(void)
{
	int cpu;
	long cnt = 0;
	for_each_present_cpu(cpu) {
		cnt += local_read(&per_cpu(bio_inflight, cpu));
	}
	return cnt;
}

static inline bool need_trace(struct bio *bio)
{
	return !is_e4defrag_thread() && bio_is_sync(bio);
}

#define BIO_TRACED (12)
static void monit_bio_queue(void *priv, struct request_queue *q,
			    struct bio *bio)
{
	if (need_trace(bio)) {
		bio_set_flag(bio, BIO_TRACED);
		local_inc(this_cpu_ptr(&bio_inflight));
	}
}

static void monit_bio_complete(void *priv, struct request_queue *q,
			       struct bio *bio, int err)
{
	if (bio_flagged(bio, BIO_TRACED)) {
		bio_clear_flag(bio, BIO_TRACED);
		local_dec(this_cpu_ptr(&bio_inflight));
	}
}

static int __init e4defrag_bio_monitor_init(void)
{
	int ret;
	ret = register_trace_block_bio_complete(monit_bio_complete, NULL);
	WARN_ON(ret);
	ret = register_trace_block_bio_queue(monit_bio_queue, NULL);
	WARN_ON(ret);
	return ret;
}

static void e4defrag_bio_monitor_exit(void)
{
	unregister_trace_block_bio_queue(monit_bio_queue, NULL);
	unregister_trace_block_bio_complete(monit_bio_complete, NULL);
}

int __init e4defrag_init_fs()
{
	int ret;
	ret = e4defrag_create_cache();
	if (ret) {
		return ret;
	}
	return e4defrag_bio_monitor_init();
}

void e4defrag_exit_fs()
{
	e4defrag_bio_monitor_exit();
	e4defrag_destroy_cache();
}
