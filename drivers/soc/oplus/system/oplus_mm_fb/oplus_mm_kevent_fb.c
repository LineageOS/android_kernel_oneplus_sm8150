// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */

#include <linux/err.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <soc/oplus/system/oplus_mm_kevent_fb.h>
#include "oplus_mm_kevent.h"

static void mm_fb_kevent_upload_jobs(struct work_struct *work);

static LIST_HEAD(mm_kevent_list);
static DEFINE_MUTEX(mm_kevent_lock);
static DECLARE_DELAYED_WORK(mm_kevent_upload_work_thread, mm_fb_kevent_upload_jobs);
static struct workqueue_struct *mm_kevent_wq;
static int mm_kevent_len = 0;

#define LIMIT_UPLOAD_TIME_MS    10000//ms
struct limit_upload_frq {
    unsigned int last_id;
    ktime_t last_time;
};
static struct limit_upload_frq g_limit;

struct mm_kevent {
	struct list_head head;
	enum OPLUS_MM_DIRVER_FB_EVENT_MODULE module;
	unsigned int event_id;
	struct mutex lock;
	int count;
	int count_total;
	u32 count_limit;
	int rate_limit_ms;
	ktime_t first;
	ktime_t last;
	ktime_t last_upload;
	struct delayed_work dwork;
	char *payload;
	char name[0];
};

int upload_mm_fb_kevent(enum OPLUS_MM_DIRVER_FB_EVENT_MODULE module, unsigned int event_id, unsigned char *payload) {
	struct mm_kevent_packet *user_msg_info;
	char event_id_str[MAX_PAYLOAD_EVENTID] = {0};
	void* buffer = NULL;
	int len, size;

	mutex_lock(&mm_kevent_lock);

	len = strlen(payload);

	size = sizeof(struct mm_kevent_packet) + len + 1;
	printk(KERN_INFO "mm_kevent fb:size=%d, module:%d, event_id:%d\n", size, module, event_id);

	buffer = kmalloc(size, GFP_ATOMIC);
	memset(buffer, 0, size);
	user_msg_info = (struct mm_kevent_packet *)buffer;
	user_msg_info->type = 1;

	memcpy(user_msg_info->tag, ATLAS_FB_EVENT, sizeof(ATLAS_FB_EVENT));

	snprintf(event_id_str, sizeof(event_id_str), "%d", event_id);
	memcpy(user_msg_info->event_id, event_id_str, strlen(event_id_str) + 1);

	user_msg_info->len = len + 1;
	memcpy(user_msg_info->data, payload, len + 1);

	mm_fb_kevent_send_to_user(user_msg_info);

	kfree(buffer);
	mutex_unlock(&mm_kevent_lock);
	return 0;
}


static void mm_fb_kevent_upload_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct mm_kevent *new_kevent = container_of(dwork, struct mm_kevent, dwork);
	struct mm_kevent *kevent = NULL, *n = NULL;
	bool found = false;

	list_for_each_entry_safe(kevent, n, &mm_kevent_list, head) {
		if (!strcmp(kevent->name, new_kevent->name)) {
			found = true;
			break;
		}
	}

	if (!found) {
		if (mm_kevent_len > 200) {
			unsigned char payload[150] = "";
			pr_err("mm_kevent large than 200");
			#ifdef NEED_FEEDBACK_TO_DISPLAY
			scnprintf(payload, sizeof(payload), "OutOfEvent$$EventID@@420$$MSG@@%s", new_kevent->name);
			#else //NEED_FEEDBACK_TO_DISPLAY
			scnprintf(payload, sizeof(payload), "MSG@@%s", new_kevent->name);
			#endif //NEED_FEEDBACK_TO_DISPLAY
			upload_mm_fb_kevent(kevent->module, kevent->event_id, payload);
			goto done;
		}

		kevent = new_kevent;
		kevent->count = 1;
		kevent->count_total = 1;
		new_kevent = NULL;
		mm_kevent_len++;
		list_add_tail(&kevent->head, &mm_kevent_list);
		goto done;
	}

	if (WARN_ON(!kevent))
		goto done;

	mutex_lock(&kevent->lock);
	kevent->count++;
	kevent->count_total++;
	kevent->last = new_kevent->first;
	kfree(kevent->payload);
	kevent->payload = new_kevent->payload;
	new_kevent->payload = NULL;
	mutex_unlock(&kevent->lock);
done:
	mm_fb_kevent_upload_jobs(NULL);
	if (new_kevent)
		kfree(new_kevent->payload);
	kfree(new_kevent);
}

static void mm_fb_kevent_upload_jobs(struct work_struct *work)
{
	struct mm_kevent *kevent = NULL, *n = NULL;
	unsigned char payload[150] = "";
	int cnt;

	list_for_each_entry_safe(kevent, n, &mm_kevent_list, head) {
		if (ktime_before(kevent->last, kevent->last_upload))
			continue;

		if (kevent->count_limit && (kevent->count_total % kevent->count_limit == 0)) {
			kevent->count_limit <<= 1;
			if (kevent->count_limit > 4096)
				kevent->count_limit = 4096;
		} else if (!kevent->rate_limit_ms || (kevent->rate_limit_ms &&
			   ktime_before(ktime_get(), ktime_add_ms(kevent->last_upload, kevent->rate_limit_ms)))) {
				continue;
		}

		mutex_lock(&kevent->lock);
		cnt = 0;
		#define PAYLOAD(fmt, ...) \
			if (sizeof(payload) > cnt) \
				cnt += scnprintf(payload + cnt, sizeof(payload) - cnt, fmt, ##__VA_ARGS__);
		#ifdef NEED_FEEDBACK_TO_DISPLAY
		PAYLOAD("EventName@@%s$$EventID@@%d$$", kevent->name, OPLUS_MM_DIRVER_FB_EVENT_ID_ERROR);
		#else //NEED_FEEDBACK_TO_DISPLAY
		PAYLOAD("EventName@@%s$$", kevent->name);
		#endif //NEED_FEEDBACK_TO_DISPLAY
		PAYLOAD("CT@@%d$$", kevent->count);
		PAYLOAD("FT@@%lu$$", ktime_to_ms(kevent->first) / 1000);
		PAYLOAD("ET@@%lu$$", ktime_to_ms(kevent->last) / 1000);
		PAYLOAD("MSG@@%s", kevent->payload ? kevent->payload : "NULL");

		if (kevent->payload) {
			kfree(kevent->payload);
			kevent->payload = NULL;
		}

		kevent->count = 0;
		kevent->last_upload = ktime_get();
		mutex_unlock(&kevent->lock);
		upload_mm_fb_kevent(kevent->module, kevent->event_id, payload);
	}

	mod_delayed_work(mm_kevent_wq, &mm_kevent_upload_work_thread, 60 * 60 * HZ);
}

static void mm_fb_kevent_upload_recv_user(int type, int flags, char* data) {
	printk(KERN_INFO "mm_kevent fb recv user type=0x%x, flags=0x%x, data=%s\n",
		type, flags, data);
    #ifdef OPLUS_NETLINK_MM_KEVENT_TEST
    if (flags & OPLUS_NETLINK_MM_DBG_LV2) {
        printk(KERN_INFO "mm_kevent: send to user, to display and atlas\n");
        upload_mm_fb_kevent(OPLUS_MM_DIRVER_FB_EVENT_TO_DISPLAY, OPLUS_MM_EVENTID_TEST_OR_DEBUG, data);
        upload_mm_fb_kevent(OPLUS_MM_DIRVER_FB_EVENT_TO_ATLAS, OPLUS_MM_EVENTID_TEST_OR_DEBUG, data);
    }
    #endif
}

int upload_mm_fb_kevent_limit(enum OPLUS_MM_DIRVER_FB_EVENT_MODULE module, unsigned int event_id,
		     const char *name, int rate_limit_ms, char *payload)
{
	struct mm_kevent *kevent = NULL;
	int size;
	
	if (!mm_kevent_wq)
		return -EINVAL;

	size = strlen(name) + sizeof(*kevent) + 1;
	kevent = kzalloc(size, GFP_ATOMIC);
	if (!kevent)
		return -ENOMEM;

	kevent->module = module;
	kevent->event_id = event_id;
	kevent->count_limit = 1;
	kevent->first = ktime_get();
	kevent->last = ktime_get();
	kevent->last_upload = ktime_get();
	kevent->rate_limit_ms = rate_limit_ms;
	memcpy(kevent->name, name, strlen(name) + 1);
	kevent->payload = kmemdup(payload, strlen(payload) + 1, GFP_ATOMIC);
	mutex_init(&kevent->lock);
	INIT_DELAYED_WORK(&kevent->dwork, mm_fb_kevent_upload_work);
	queue_delayed_work(mm_kevent_wq, &kevent->dwork, 0);

	return 0;
}

int upload_mm_fb_kevent_to_atlas_limit(unsigned int event_id, unsigned char *payload, int limit_ms)
{
	struct mm_kevent_packet *user_msg_info;
	char event_id_str[MAX_PAYLOAD_EVENTID] = {0};
	void* buffer = NULL;
	int len, size;
	int ret = 0;

	mutex_lock(&mm_kevent_lock);

	if ((limit_ms > 0) && (g_limit.last_id == event_id)) {
		if (ktime_before(ktime_get(), ktime_add_ms(g_limit.last_time, limit_ms))) {
			printk(KERN_INFO "upload event_id=%d failed, report too often, limit_ms=%d\n",
					event_id, limit_ms);
			ret = -1;
			goto _exit;
		}
	}

	len = strlen(payload);

	size = sizeof(struct mm_kevent_packet) + len + 1;
	printk(KERN_INFO "kevent_send_to_user:event_id=%d,size=%d,buflen=%d,payload:%s\n",
			event_id, size, len, payload);

	buffer = kmalloc(size, GFP_ATOMIC);
	memset(buffer, 0, size);
	user_msg_info = (struct mm_kevent_packet *)buffer;
	user_msg_info->type = 1;

	memcpy(user_msg_info->tag, ATLAS_FB_EVENT, strlen(ATLAS_FB_EVENT) + 1);

	snprintf(event_id_str, sizeof(event_id_str), "%d", event_id);
	memcpy(user_msg_info->event_id, event_id_str, strlen(event_id_str) + 1);

	user_msg_info->len = len + 1;
	memcpy(user_msg_info->data, payload, len + 1);

	mm_fb_kevent_send_to_user(user_msg_info);
	//msleep(20);
	kfree(buffer);
	g_limit.last_id = event_id;
	g_limit.last_time = ktime_get();

_exit:
	mutex_unlock(&mm_kevent_lock);
	return 0;
}
EXPORT_SYMBOL(upload_mm_fb_kevent_to_atlas_limit);

#define MM_FB_EVENTID_LEN   5
#define MM_FB_HAL_LIMIT    (60*1000)
#define IS_DIGITAL(x) (((x) >= '0') && ((x) <= '9'))
static ssize_t mm_fb_write(struct file *file,
				const char __user *buf,
				size_t count,
				loff_t *lo)
{
	char *r_buf;
	unsigned int event_id = 0;
	int len, i;

	r_buf = (char *)kzalloc(MAX_PAYLOAD_DATASIZE, GFP_KERNEL);
	if (!r_buf) {
		return count;
    }

	if (copy_from_user(r_buf, buf, MAX_PAYLOAD_DATASIZE > count ? count : MAX_PAYLOAD_DATASIZE)) {
		goto exit;
	}

	r_buf[MAX_PAYLOAD_DATASIZE - 1] ='\0'; /*make sure last bype is eof*/
	printk(KERN_INFO "mm_fb_write: feedback data=%s\n", r_buf);
	len = strlen(r_buf);
	if (len < (MM_FB_EVENTID_LEN + 2)) {
		printk(KERN_INFO "mm_fb_write: data len=%d is error\n", len);
		goto exit;
	}

	for (i = 0; i < MM_FB_EVENTID_LEN; i++) {
		if (IS_DIGITAL(r_buf[i])) {
			event_id = event_id*10 + r_buf[i] - '0';
		} else {
			printk(KERN_INFO "mm_fb_write: eventid is error, data=%s\n", r_buf);
			goto exit;
		}
	}

	upload_mm_fb_kevent_to_atlas_limit(event_id, r_buf + MM_FB_EVENTID_LEN + 1, MM_FB_HAL_LIMIT);

exit:

	kfree(r_buf);
	return count;
}

static ssize_t mm_fb_read(struct file *file,
				char __user *buf,
				size_t count,
				loff_t *ppos)
{
	return count;
}

static const struct file_operations mm_fb_fops = {
	.write = mm_fb_write,
	.read  = mm_fb_read,
	.open  = simple_open,
	.owner = THIS_MODULE,
};

int mm_fb_kevent_init(void)
{
	struct proc_dir_entry *d_entry = NULL;
	
	mm_kevent_wq = create_workqueue("mm_kevent");
	if (!mm_kevent_wq)
		return -ENOMEM;
	queue_delayed_work(mm_kevent_wq, &mm_kevent_upload_work_thread, 0);

    mm_fb_kevent_set_recv_user(mm_fb_kevent_upload_recv_user);

    g_limit.last_id = 0xFFFFFFFF;
    g_limit.last_time = 0;

	d_entry = proc_create_data("mm_fb", 0664, NULL, &mm_fb_fops, NULL);
	if (!d_entry) {
		pr_err("failed to create node\n");
		return -ENODEV;
	}

	return 0;
}

void mm_fb_kevent_deinit(void)
{
	if (mm_kevent_wq)
		destroy_workqueue(mm_kevent_wq);

	mm_fb_kevent_set_recv_user(NULL);
}


module_init(mm_fb_kevent_init);
module_exit(mm_fb_kevent_deinit);

