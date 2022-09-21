/*
 * DSPG DBMDX codec driver character device interface
 *
 * Copyright (C) 2014 DSP Group
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/module.h>
#include <linux/kfifo.h>
#include <linux/delay.h>

#include "dbmdx-interface.h"

static struct dbmdx_private *dbmdx_p;

static atomic_t cdev_opened[2] = { ATOMIC_INIT(0), ATOMIC_INIT(0) };

static int dbmdx_record_open_dev0(struct inode *inode, struct file *file)
{
	struct dbmdx_private *p = dbmdx_p;

	if (!p || !p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EIO;
	}

	if ((p->chdev_options & DBMDX_CHDEV_OPT_DEV0_MP) &&
		 !(p->multipurpose_kfifo_in_use)) {
		dev_err(p->dev, "%s: MP FIFO is not in use\n",	__func__);
		return -EIO;
	}

	if (!atomic_add_unless(&(cdev_opened[0]), 1, 1))
		return -EBUSY;

	file->private_data = dbmdx_p;

	return 0;
}

static int dbmdx_record_release_dev0(struct inode *inode, struct file *file)
{
	struct dbmdx_private *p = (struct dbmdx_private *)file->private_data;

	if (!p || !p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		goto out;
	}

	if (!(p->chdev_options & DBMDX_CHDEV_OPT_DEV0_MP) &&
		!(p->chdev_options &
			DBMDX_CHDEV_OPT_CONT_BUFFERING_ON_DEV0_RELEASE)) {
		dbmdx_p->lock(dbmdx_p);
		dbmdx_p->va_flags.buffering = 0;
		dbmdx_p->unlock(dbmdx_p);

		flush_work(&dbmdx_p->sv_work);
	}
out:
	atomic_dec(&(cdev_opened[0]));

	return 0;
}

static int dbmdx_record_open_dev1(struct inode *inode, struct file *file)
{
	struct dbmdx_private *p = dbmdx_p;

	if (!p || !p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EIO;
	}

	if ((p->chdev_options & DBMDX_CHDEV_OPT_DEV1_MP) &&
		 !(p->multipurpose_kfifo_in_use)) {
		dev_err(p->dev, "%s: MP FIFO is not in use\n",	__func__);
		return -EIO;
	}

	if (!atomic_add_unless(&(cdev_opened[1]), 1, 1))
		return -EBUSY;

	file->private_data = dbmdx_p;

	return 0;
}

static int dbmdx_record_release_dev1(struct inode *inode, struct file *file)
{
	struct dbmdx_private *p = (struct dbmdx_private *)file->private_data;

	if (!p || !p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		goto out;
	}

	if (!(p->chdev_options & DBMDX_CHDEV_OPT_DEV1_MP) &&
		!(p->chdev_options &
			DBMDX_CHDEV_OPT_CONT_BUFFERING_ON_DEV1_RELEASE)) {
		dbmdx_p->lock(dbmdx_p);
		dbmdx_p->va_flags.buffering = 0;
		dbmdx_p->unlock(dbmdx_p);

		flush_work(&dbmdx_p->sv_work);
	}
out:
	atomic_dec(&(cdev_opened[1]));

	return 0;
}

/*
 * read out of the kfifo as much as was requested or if requested more
 * as much as is in the FIFO
 */
static ssize_t read_from_kfifo(struct dbmdx_private *p, char __user *buf,
				 size_t count_want, loff_t *f_pos,
				 struct kfifo *samples_kfifo)
{
	size_t not_copied;
	ssize_t to_copy = count_want;
	int avail;
	unsigned int copied;
	int ret;

	dev_dbg(p->dbmdx_dev, "%s: count_want:%zu f_pos:%lld\n",
			__func__, count_want, *f_pos);

	avail = kfifo_len(samples_kfifo);

	if (avail == 0)
		return 0;

	if (count_want > avail)
		to_copy = avail;

	ret = kfifo_to_user(samples_kfifo, buf, to_copy, &copied);
	if (ret)
		return -EIO;

	not_copied = count_want - copied;
	*f_pos = *f_pos + (count_want - not_copied);

	return count_want - not_copied;
}

/*
 * read out of the kfifo as much as was requested and block until all
 * data is available or a timeout occurs
 */
static ssize_t read_from_kfifo_blocking(struct dbmdx_private *p,
		char __user *buf, size_t count_want, loff_t *f_pos,
		struct kfifo *samples_kfifo)
{
	size_t not_copied;
	ssize_t to_copy = count_want;
	int avail = 0;
	unsigned int copied, total_copied = 0;
	int ret;
	unsigned long timeout = jiffies + msecs_to_jiffies(50);

	dev_dbg(p->dbmdx_dev, "%s: count_want:%zu f_pos:%lld\n",
			__func__, count_want, *f_pos);

	while ((total_copied < count_want) &&
			time_before(jiffies, timeout)) {

		avail = kfifo_len(samples_kfifo);

		if (avail == 0) {
			if(p->va_flags.buffering)
				msleep(20);
			else
				break;
		}

		if (avail > 0) {
			to_copy = avail;
			if (count_want - total_copied < avail)
				to_copy = count_want - total_copied;

			ret = kfifo_to_user(samples_kfifo,
				buf + total_copied, to_copy, &copied);
			if (ret)
				return -EIO;

			total_copied += copied;
		}
	}

	if (avail && (total_copied < count_want))
		dev_err(p->dev, "dbmdx: timeout during reading\n");

	not_copied = count_want - total_copied;
	*f_pos = *f_pos + (count_want - not_copied);

	dev_dbg(p->dbmdx_dev,
		"%s: count_want:%zu total copied:%u, not copied %zu\n",
			__func__, count_want, total_copied, not_copied);

	return count_want - not_copied;
}

/*
 * read out of the kfifo as much as was requested or if requested more
 * as much as is in the FIFO
 */
static ssize_t dbmdx_record_read_dev0(struct file *file, char __user *buf,
				 size_t count_want, loff_t *f_pos)
{
	struct dbmdx_private *p = (struct dbmdx_private *)file->private_data;
	struct kfifo *cur_kfifo = &(p->detection_samples_kfifo);

	if (!p || !p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EIO;
	}

	if ((p->chdev_options & DBMDX_CHDEV_OPT_DEV0_MP)) {
		if (!(p->multipurpose_kfifo_in_use)) {
			dev_err(p->dev, "%s: MP FIFO is not in use\n",
				__func__);
			return -EIO;
		}
		cur_kfifo = &(p->multipurpose_samples_kfifo);
	}

	if (p->chdev_options & DBMDX_CHDEV_OPT_DEV0_BLOCKING)
		return read_from_kfifo_blocking(p, buf, count_want, f_pos,
						cur_kfifo);
	else
		return read_from_kfifo(p, buf, count_want, f_pos, cur_kfifo);
}

static ssize_t dbmdx_record_read_dev1(struct file *file, char __user *buf,
				 size_t count_want, loff_t *f_pos)
{
	struct dbmdx_private *p = (struct dbmdx_private *)file->private_data;
	struct kfifo *cur_kfifo = &(p->detection_samples_kfifo);

	if (!p || !p->device_ready) {
		dev_err(p->dev, "%s: device not ready\n", __func__);
		return -EIO;
	}

	if ((p->chdev_options & DBMDX_CHDEV_OPT_DEV1_MP)) {
		if (!(p->multipurpose_kfifo_in_use)) {
			dev_err(p->dev, "%s: MP FIFO is not in use\n",
				__func__);
			return -EIO;
		}
		cur_kfifo = &(p->multipurpose_samples_kfifo);
	}

	if (p->chdev_options & DBMDX_CHDEV_OPT_DEV1_NON_BLOCKING)
		return read_from_kfifo(p, buf, count_want, f_pos, cur_kfifo);
	else
		return read_from_kfifo_blocking(p, buf, count_want, f_pos,
						cur_kfifo);
}


static const struct file_operations dbmdx_cdev_fops_dev0 = {
	.owner   = THIS_MODULE,
	.open    = dbmdx_record_open_dev0,
	.release = dbmdx_record_release_dev0,
	.read    = dbmdx_record_read_dev0,
};

static const struct file_operations dbmdx_cdev_fops_dev1 = {
	.owner   = THIS_MODULE,
	.open    = dbmdx_record_open_dev1,
	.release = dbmdx_record_release_dev1,
	.read    = dbmdx_record_read_dev1,
};

int dbmdx_register_cdev(struct dbmdx_private *p)
{
	int ret;

	dbmdx_p = p;

	ret = alloc_chrdev_region(&p->record_chrdev, 0, 2, "dbmdx");
	if (ret) {
		dev_err(p->dbmdx_dev, "failed to allocate character device\n");
		return -EINVAL;
	}

	cdev_init(&p->record_cdev0, &dbmdx_cdev_fops_dev0);
	cdev_init(&p->record_cdev1, &dbmdx_cdev_fops_dev1);

	p->record_cdev0.owner = THIS_MODULE;
	p->record_cdev1.owner = THIS_MODULE;

	ret = cdev_add(&p->record_cdev0, p->record_chrdev, 1);
	if (ret) {
		dev_err(p->dbmdx_dev, "failed to add character device\n");
		unregister_chrdev_region(p->record_chrdev, 1);
		return -EINVAL;
	}

	ret = cdev_add(&p->record_cdev1, p->record_chrdev + 1, 1);
	if (ret) {
		dev_err(p->dbmdx_dev,
			"failed to add blocking character device\n");
		unregister_chrdev_region(p->record_chrdev, 1);
		return -EINVAL;
	}

	p->record_dev0 = device_create(p->ns_class, &platform_bus,
				      MKDEV(MAJOR(p->record_chrdev), 0),
				      p, "dbmdx-%d", 0);
	if (IS_ERR(p->record_dev0)) {
		dev_err(p->dev, "could not create device\n");
		unregister_chrdev_region(p->record_chrdev, 1);
		cdev_del(&p->record_cdev0);
		return -EINVAL;
	}

	p->record_dev1 = device_create(p->ns_class, &platform_bus,
					    MKDEV(MAJOR(p->record_chrdev), 1),
					    p, "dbmdx-%d", 1);
	if (IS_ERR(p->record_dev1)) {
		dev_err(p->dev, "could not create device\n");
		unregister_chrdev_region(p->record_chrdev, 1);
		cdev_del(&p->record_cdev1);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(dbmdx_register_cdev);

void dbmdx_deregister_cdev(struct dbmdx_private *p)
{
	if (p->record_dev0) {
		device_unregister(p->record_dev0);
		cdev_del(&p->record_cdev0);
	}
	if (p->record_dev1) {
		device_unregister(p->record_dev1);
		cdev_del(&p->record_cdev1);
	}
	unregister_chrdev_region(p->record_chrdev, 2);

	dbmdx_p = NULL;
}
EXPORT_SYMBOL(dbmdx_deregister_cdev);
