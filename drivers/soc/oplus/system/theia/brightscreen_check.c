// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/notifier.h>
#include <linux/timer.h>
#include <linux/kthread.h>
#include <soc/oplus/system/oplus_bscheck.h>
#include <soc/oplus/system/oplus_brightscreen_check.h>

#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#ifdef CONFIG_ARM
#include <linux/sched.h>
#else
#include <linux/wait.h>
#endif
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/sched/debug.h>
#include <linux/nmi.h>
#include <soc/oplus/system/boot_mode.h>
#if IS_MODULE(CONFIG_OPLUS_FEATURE_THEIA)
#include <linux/sysrq.h>
#endif

#ifdef CONFIG_DRM_MSM
#include <linux/msm_drm_notify.h>
#endif

#define BRIGHTSCREEN_COUNT_FILE	"/data/oplus/log/bsp/brightscreen_count.txt"
#define BRIGHTSCREEN_HAPPENED_FILE	"/data/oplus/log/bsp/brightscreen_happened.txt"

#define BRIGHT_DEBUG_PRINTK(a, arg...)\
    do{\
         printk("[bright_check]: " a, ##arg);\
    }while(0)

#define BRIGHT_STATUS_INIT				1
#define BRIGHT_STATUS_INIT_FAIL 		2
#define BRIGHT_STATUS_INIT_SUCCEES		3
#define BRIGHT_STATUS_CHECK_ENABLE 		4
#define BRIGHT_STATUS_CHECK_DISABLE 	5
#define BRIGHT_STATUS_CHECK_DEBUG 		6

#define BRIGHT_CLEAR_ERROR_HAPPENED		0
#define BRIGHT_MARK_ERROR_HAPPENED		1

#define BRIGHT_MAX_WRITE_NUMBER			3000
#define BRIGHT_DEFAULT_TIMEOUT_MS		120000
#define BRIGHT_INIT_STATUS_TIMEOUT_MS	150000
#define BRIGHT_SLOW_TIMEOUT_MS		    20000

#define SIG_THEIA (SIGRTMIN + 0x13)
#define TASK_INIT_COMM                                     "init"
static int def_swich=1;
struct bright_data{
	int is_panic;
	int status;
	int blank;
	int get_log;
	unsigned int timeout_ms;
	int error_detected;
	unsigned int error_count_new;
	unsigned int error_count_old;
	struct notifier_block fb_notif;
	struct timer_list timer;
	wait_queue_head_t keylog_thread_wq;
	struct work_struct error_happen_work;
	struct work_struct clear_error_happened_flag_work;
	struct delayed_work determine_init_status_work;
};

static void bright_timer_func(struct timer_list *t);
static int get_status(void);

static struct bright_data g_bright_data = {
	.is_panic = 0,
	.status = BRIGHT_STATUS_INIT,
#ifdef CONFIG_DRM_MSM
	.blank = MSM_DRM_BLANK_UNBLANK,
#else
	.blank = FB_BLANK_UNBLANK,
#endif
	.timeout_ms = BRIGHT_SLOW_TIMEOUT_MS,
	.error_detected = 0,
	.error_count_new = 0,
	.error_count_old = 0,
	//.timer.function = bright_timer_func,
};

static wait_queue_head_t bright_check_wq;

int bright_screen_timer_restart(void)
{
	static bool start_check = false;

	//BRIGHT_DEBUG_PRINTK("bright_screen_timer_restart:blank = %d,status = %d\n",g_bright_data.blank,g_bright_data.status);
	if(g_bright_data.status != BRIGHT_STATUS_CHECK_ENABLE && g_bright_data.status != BRIGHT_STATUS_CHECK_DEBUG){
		BRIGHT_DEBUG_PRINTK("bright_screen_timer_restart:g_bright_data.status = %d return\n",g_bright_data.status);
		return g_bright_data.status;
	}
#ifdef CONFIG_DRM_MSM
		if(g_bright_data.blank == MSM_DRM_BLANK_POWERDOWN)	//MSM_DRM_BLANK_POWERDOWN
#else
		if(g_bright_data.blank == FB_BLANK_POWERDOWN)	//FB_BLANK_POWERDOWN
#endif
        {
			start_check = true;
		}

		if (start_check == false) {
			BRIGHT_DEBUG_PRINTK("bright_screen_timer_restart: blank:%d status:%d never bright, return\n",
				g_bright_data.blank, g_bright_data.status);
			return g_bright_data.status;
		}

#ifdef CONFIG_DRM_MSM
	if(g_bright_data.blank == MSM_DRM_BLANK_UNBLANK)	//MSM_DRM_BLANK_POWERDOWN
#else
	if(g_bright_data.blank == FB_BLANK_UNBLANK)		//FB_BLANK_POWERDOWN
#endif
    {
		mod_timer(&g_bright_data.timer,jiffies + msecs_to_jiffies(g_bright_data.timeout_ms));
		BRIGHT_DEBUG_PRINTK("bright_screen_timer_restart: bright screen check start %u\n",g_bright_data.timeout_ms);
        theia_pwk_stage_start();
		return 0;
	}
	return g_bright_data.blank;
}
EXPORT_SYMBOL(bright_screen_timer_restart);

static ssize_t bright_screen_check_proc_read(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;

	len = sprintf(&page[len],"%d %u %d %d\n",
		g_bright_data.status,g_bright_data.timeout_ms,g_bright_data.is_panic,g_bright_data.get_log);

	if(len > *off)
	   len -= *off;
	else
	   len = 0;

	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	return (len < count ? len : count);

}

static ssize_t bright_screen_check_proc_write(struct file *file, const char __user *buf,
		size_t count,loff_t *off)
{
	int tmp_status = 0;
	int tmp_timeout = 0;
	int tmp_isPanic = 0;
	int tmp_log = 0;
	int ret = 0;
    char buffer[40] = {0};

	if(g_bright_data.status == BRIGHT_STATUS_INIT || g_bright_data.status == BRIGHT_STATUS_INIT_FAIL){
		BRIGHT_DEBUG_PRINTK("%s init not finish: status = %d\n", __func__, g_bright_data.status);
		return count;
	}

    if (count > 40) {
       count = 40;
    }

    if (copy_from_user(buffer, buf, count)) {
		BRIGHT_DEBUG_PRINTK("%s: read proc input error.\n", __func__);
		return count;
    }
	ret = sscanf(buffer, "%d %u %d %d", &tmp_status,&tmp_timeout,&tmp_isPanic,&tmp_log);

    if (ret == 1) {
    	g_bright_data.status = tmp_status;
	} else if(ret == 4){
		g_bright_data.status = tmp_status;
		if(tmp_timeout)
			g_bright_data.timeout_ms = tmp_timeout;
		g_bright_data.is_panic = tmp_isPanic;
		g_bright_data.get_log = tmp_log;
	}else{
	    BRIGHT_DEBUG_PRINTK("%s invalid content: '%s', length = %zd\n", __func__, buffer, count);
	}

	def_swich = tmp_status == BRIGHT_STATUS_CHECK_ENABLE || BRIGHT_STATUS_CHECK_DEBUG == tmp_status;
	BRIGHT_DEBUG_PRINTK("%s: status = %d timeout_ms = %u is_panic = %d\n",
		__func__, g_bright_data.status,g_bright_data.timeout_ms,g_bright_data.is_panic);

	return count;

}

struct file_operations bright_screen_check_proc_fops = {
	.read = bright_screen_check_proc_read,
	.write = bright_screen_check_proc_write,
};


static ssize_t bright_screen_check_status_proc_read(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
	char page[256] = {0};
	int len = 0;
	int error_detected = 0;

	BRIGHT_DEBUG_PRINTK("read bright_err_detected:%d count:%d begin\n",
		g_bright_data.error_detected, g_bright_data.error_count_new);
	wait_event_interruptible(bright_check_wq,g_bright_data.error_detected>0);
	BRIGHT_DEBUG_PRINTK("read bright_err_detected:%d count:%d end\n",
		g_bright_data.error_detected, g_bright_data.error_count_new);

	if (g_bright_data.error_detected) {
		len = sprintf(&page[len],"%d", g_bright_data.error_count_new);
	} else {
		len = sprintf(&page[len],"%d", 0);
	}
	error_detected = g_bright_data.error_detected;
	g_bright_data.error_detected = 0;
	if(len > *off)
	   len -= *off;
	else
	   len = 0;
	if(copy_to_user(buf,page,(len < count ? len : count))){
	   return -EFAULT;
	}
	*off += len < count ? len : count;
	if (error_detected) {
		schedule_work(&g_bright_data.clear_error_happened_flag_work);
	}

	return (len < count ? len : count);

}


struct file_operations bright_screen_check_status_proc_fops = {
	.read = bright_screen_check_status_proc_read,
};


static bool is_zygote_process(struct task_struct *t)
{
	const struct cred *tcred = __task_cred(t);
	if(!strcmp(t->comm, "main") && (tcred->uid.val == 0) &&
		(t->parent != 0 && !strcmp(t->parent->comm,"init")))
		return true;
	else
		return false;
	return false;
}
extern void touch_all_softlockup_watchdogs(void);

static void bright_show_coretask_state(void)
{
	struct task_struct *g, *p;

	rcu_read_lock();
	for_each_process_thread(g, p) {
		if (is_zygote_process(p) || !strncmp(p->comm,"system_server", TASK_COMM_LEN)
			|| !strncmp(p->comm,"surfaceflinger", TASK_COMM_LEN)) {
#if IS_MODULE(CONFIG_OPLUS_FEATURE_THEIA)
			touch_nmi_watchdog();
#endif
			sched_show_task(p);
		}
	}

#if IS_BUILTIN(CONFIG_OPLUS_FEATURE_THEIA)
	touch_all_softlockup_watchdogs();
#endif
	rcu_read_unlock();
}

static bool bright_check_error_happened_before(struct bright_data *bri_data)
{
	struct file *fp;
	loff_t pos;
	ssize_t len = 0;
	char buf[256] = {'\0'};
	int error_happened = 0;

	fp = filp_open(BRIGHTSCREEN_HAPPENED_FILE, O_RDWR | O_CREAT, 0664);
	if (IS_ERR(fp)) {
		BRIGHT_DEBUG_PRINTK("create %s file error fp:%p\n", BRIGHTSCREEN_COUNT_FILE, fp);
		return false;
	}

	pos = 0;
	len = kernel_read(fp, buf, sizeof(buf), &pos);
	if (len < 0) {
		BRIGHT_DEBUG_PRINTK("read %s file error\n", BRIGHTSCREEN_HAPPENED_FILE);
		goto out;
	}
	sscanf(buf, "%d", &error_happened);
out:
	BRIGHT_DEBUG_PRINTK("error_happened before:%d\n", error_happened);
	filp_close(fp, NULL);

	return (error_happened > 0 ? true : false);
}

static int bright_mark_or_clear_error_happened_flag(struct bright_data *bri_data, int mark_or_clear)
{
	struct file *fp;
	loff_t pos;
	ssize_t len = 0;
	char buf[256] = {'\0'};
	int old_value = 0;

	fp = filp_open(BRIGHTSCREEN_HAPPENED_FILE, O_RDWR | O_CREAT, 0664);
	if (IS_ERR(fp)) {
		BRIGHT_DEBUG_PRINTK("create %s file error fp:%p\n", BRIGHTSCREEN_HAPPENED_FILE, fp);
		return -1;
	}

	//read to check whether write or not
	pos = 0;
	len = kernel_read(fp, buf, sizeof(buf), &pos);
	if (len < 0) {
		BRIGHT_DEBUG_PRINTK("read %s file error\n", BRIGHTSCREEN_HAPPENED_FILE);
		goto out;
	}
	sscanf(buf, "%d", &old_value);
	if (old_value == mark_or_clear) {
		BRIGHT_DEBUG_PRINTK("buf:%s, old_value:%d, mark_or_clear:%d\n",
			buf, old_value, mark_or_clear);
		goto out;
	}
	//read end

	//write begin
	sprintf(buf, "%d\n", mark_or_clear);
	pos = 0;
	len = kernel_write(fp, buf, strlen(buf), &pos);
	if (len < 0)
		BRIGHT_DEBUG_PRINTK("write %s file error\n", BRIGHTSCREEN_HAPPENED_FILE);
	//write end

	pos = 0;
	kernel_read(fp, buf, sizeof(buf), &pos);
	BRIGHT_DEBUG_PRINTK("bright_mark_or_clear_error_happened_flag %s, mark:%d\n",
		buf, mark_or_clear);
out:
	filp_close(fp, NULL);

	return len;
}

static int bright_write_error_count(struct bright_data *bri_data)
{
	struct file *fp;
	loff_t pos;
	ssize_t len = 0;
	char buf[256] = {'\0'};
	static bool have_read_old = false;

	fp = filp_open(BRIGHTSCREEN_COUNT_FILE, O_RDWR | O_CREAT, 0664);
	if (IS_ERR(fp)) {
		BRIGHT_DEBUG_PRINTK("create %s file error fp:%p\n", BRIGHTSCREEN_COUNT_FILE, fp);
		return -1;
	}

	//read old error_count begin
	if (have_read_old == false) {
		pos = 0;
		len = kernel_read(fp, buf, sizeof(buf), &pos);
		if (len < 0) {
			BRIGHT_DEBUG_PRINTK("read %s file error\n", BRIGHTSCREEN_COUNT_FILE);
			goto out;
		}
		sscanf(buf, "%d", &bri_data->error_count_old);
		bri_data->error_count_new = bri_data->error_count_new + bri_data->error_count_old;
		have_read_old = true;
		BRIGHT_DEBUG_PRINTK("read_buf:%s count_old:%d count_new:%d\n",
			buf, bri_data->error_count_old, bri_data->error_count_new);

	}
	//read old error_count_new end

	sprintf(buf, "%d\n", bri_data->error_count_new);

	pos = 0;
	len = kernel_write(fp, buf, strlen(buf), &pos);
	if (len < 0)
		BRIGHT_DEBUG_PRINTK("write %s file error\n", BRIGHTSCREEN_COUNT_FILE);

	pos = 0;
	kernel_read(fp, buf, sizeof(buf), &pos);
	BRIGHT_DEBUG_PRINTK("bright_write_error_count %s\n", buf);

out:
	filp_close(fp, NULL);

	return len;
}

static void bright_determine_init_status_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct bright_data *bri_data =
			container_of(dwork, struct bright_data, determine_init_status_work);

	if(bri_data->status != BRIGHT_STATUS_CHECK_ENABLE && bri_data->status != BRIGHT_STATUS_CHECK_DEBUG){
		BRIGHT_DEBUG_PRINTK("init_status status: %d, return\n", bri_data->status);
		return;
	}
	if (bright_check_error_happened_before(bri_data)) {
		bri_data->error_detected = 1;
		wake_up_interruptible(&bright_check_wq);
	}
}
static void find_task_by_comm(const char * pcomm, struct task_struct ** t_result)
{
    struct task_struct *g, *t;
    rcu_read_lock();
    for_each_process_thread(g, t)
    {
        if(!strcmp(t->comm, pcomm))
        {
            *t_result = t;
            rcu_read_unlock();
            return;
        }
    }
    t_result = NULL;
    rcu_read_unlock();
}

//copy mtk_boot_common.h
#define NORMAL_BOOT 0
#define ALARM_BOOT 7

static int get_status(void)
{
#ifdef CONFIG_DRM_MSM
    if(MSM_BOOT_MODE__NORMAL == get_boot_mode())
    {
		if(def_swich)
			return BRIGHT_STATUS_CHECK_ENABLE;
    }
    return BRIGHT_STATUS_INIT_SUCCEES;
#else
    if((get_boot_mode() == NORMAL_BOOT) || (get_boot_mode() == ALARM_BOOT)) 
    {
       if(def_swich)
			return BRIGHT_STATUS_CHECK_ENABLE;
    }
    return BRIGHT_STATUS_INIT_SUCCEES;

#endif
}

static int get_log_swich()
{
    return  (BRIGHT_STATUS_CHECK_ENABLE == get_status()||BRIGHT_STATUS_CHECK_DEBUG == get_status())&& g_bright_data.get_log;
}
static void dump_freeze_log(void)
{
    struct task_struct *t_init;
	if (!get_log_swich())
	{
	    BRIGHT_DEBUG_PRINTK("log swich is disable ,return !!!");
	    return;
	}
    t_init = NULL;
    find_task_by_comm(TASK_INIT_COMM, &t_init);
    if(NULL != t_init)
    {
        BRIGHT_DEBUG_PRINTK("send signal %d at ", SIG_THEIA);
        send_sig(SIG_THEIA, t_init, 0);
    }
}

static void bright_clear_error_happened_flag_work(struct work_struct *work)
{
	struct bright_data *bri_data
		= container_of(work, struct bright_data, clear_error_happened_flag_work);

	bright_mark_or_clear_error_happened_flag(bri_data, BRIGHT_CLEAR_ERROR_HAPPENED);
}

static void bright_error_happen_work(struct work_struct *work)
{
	struct bright_data *bri_data
							= container_of(work, struct bright_data, error_happen_work);

	if (bri_data->error_count_new < BRIGHT_MAX_WRITE_NUMBER) {
		bri_data->error_count_new++;
		dump_freeze_log();
		bright_write_error_count(bri_data);
		bright_mark_or_clear_error_happened_flag(bri_data, BRIGHT_MARK_ERROR_HAPPENED);
	}
	BRIGHT_DEBUG_PRINTK("bright_error_happen_work error_count_new = %d old = %d\n",
		bri_data->error_count_new, bri_data->error_count_old);
    theia_pwk_stage_end();
	if(bri_data->is_panic) {
		//1.meminfo
		//2. show all D task
#if IS_MODULE(CONFIG_OPLUS_FEATURE_THEIA)
		handle_sysrq('w');
#else
		show_state_filter(TASK_UNINTERRUPTIBLE);
#endif
		//3. show system_server zoygot surfacefliger state
		bright_show_coretask_state();
		//4.current cpu registers :skip for minidump
		panic("bright screen detected, force panic");
	}
	bri_data->error_detected = 1;
	wake_up_interruptible(&bright_check_wq);
}
static void bright_timer_func(struct timer_list *t)
{
//	struct bright_data * p = (struct bright_data *)data;
	struct bright_data * p = from_timer(p, t, timer);


	BRIGHT_DEBUG_PRINTK("bright_timer_func is called\n");

	schedule_work(&p->error_happen_work);
}

#ifdef CONFIG_DRM_MSM
static int bright_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank;

 	if (event == MSM_DRM_EVENT_BLANK && evdata && evdata->data)
	{
		blank = evdata->data;
		g_bright_data.blank = *blank;
		if(g_bright_data.status != BRIGHT_STATUS_CHECK_DEBUG){
			del_timer(&g_bright_data.timer);
			BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback: del timer,event:%lu status:%d blank:%d\n",
				event, g_bright_data.status, g_bright_data.blank);
		} else {
			BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback:event = %lu status:%d blank:%d\n",
				event,g_bright_data.status,g_bright_data.blank);
		}
	}
	else if (event == MSM_DRM_EARLY_EVENT_BLANK && evdata && evdata->data)
	{
		blank = evdata->data;

		blank = evdata->data;
		g_bright_data.blank = *blank;
		if(g_bright_data.status != BRIGHT_STATUS_CHECK_DEBUG){
			del_timer(&g_bright_data.timer);
			BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback: del timer event:%lu status:%d blank:%d\n",
				event, g_bright_data.status, g_bright_data.blank);
		} else {
			BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback:event = %lu status:%d blank:%d\n",
				event,g_bright_data.status,g_bright_data.blank);
		}
	} else {
		BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback:event = %lu status:%d\n",event,g_bright_data.status);
	}

	return 0;
}
#else
#ifndef FB_EARLY_EVENT_BLANK
#define FB_EARLY_EVENT_BLANK    0x10
#endif

static int bright_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK)
	{
		blank = evdata->data;
		g_bright_data.blank = *blank;
		if(g_bright_data.status != BRIGHT_STATUS_CHECK_DEBUG){
			del_timer(&g_bright_data.timer);
			BRIGHT_DEBUG_PRINTK("bright fb_notifier_callback: del timer,event:%d,status:%d,blank:%d\n",
				event, g_bright_data.status, g_bright_data.blank);
		} else {
			BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback:event = %lu,status:%d,blank:%d\n",
				event,g_bright_data.status,g_bright_data.blank = *blank);
		}
	}
    else if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK)
	{
		blank = evdata->data;
		g_bright_data.blank = *blank;
		if(g_bright_data.status != BRIGHT_STATUS_CHECK_DEBUG){
			del_timer(&g_bright_data.timer);
			BRIGHT_DEBUG_PRINTK("bright fb_notifier_callback: del timer,event:%d,status:%d,blank:%d\n",
				event, g_bright_data.status, g_bright_data.blank);
		} else {
			BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback:event = %lu,status:%d,blank:%d\n",
				event,g_bright_data.status,g_bright_data.blank = *blank);
		}
    } else {
		BRIGHT_DEBUG_PRINTK("bright_fb_notifier_callback:event = %lu,status:%d\n",event,g_bright_data.status);
	}
	return 0;
}
#endif /* CONFIG_DRM_MSM */
static int passok(int stage)
{
    return stage==0;
}
static ssize_t bright_screen_ur_process(int stage)
{
    if(passok(stage))
	{

	    del_timer(&g_bright_data.timer);
			BRIGHT_DEBUG_PRINTK("del timer,stage: %d  \n",
				stage);
	}
	return 0;
}

static ssize_t bright_screen_switch_proc_write(struct file *file, const char __user *buf,
		size_t count,loff_t *off)
{
	int tmp_stage = 0;
	int ret = 0;
    char buffer[40] = {0};

	if(g_bright_data.status == BRIGHT_STATUS_INIT || g_bright_data.status == BRIGHT_STATUS_INIT_FAIL){
		BRIGHT_DEBUG_PRINTK("%s init not finish: status = %d\n", __func__, g_bright_data.status);
		return count;
	}

    if (count > 40) {
       count = 40;
    }

    if (copy_from_user(buffer, buf, count)) {
		BRIGHT_DEBUG_PRINTK("%s: read proc input error.\n", __func__);
		return count;
    }

	ret = sscanf(buffer, "%d", &tmp_stage);
    if(1 == tmp_stage)
    {
        del_timer(&g_bright_data.timer);
    }

    if (ret == 1) {
		BRIGHT_DEBUG_PRINTK("%s recv us report  stage: %d\n", __func__, tmp_stage);
		bright_screen_ur_process(tmp_stage);
    	//g_black_data.status = tmp_status;
	}else{
	    BRIGHT_DEBUG_PRINTK("%s invalid content: '%s', length = %zd\n", __func__, buf, count);
	}

	return count;
}
static ssize_t bright_screen_switch_proc_read(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
    return 0;
}

struct file_operations bright_screen_switch_proc_fops = {
	.read = bright_screen_switch_proc_read,
	.write = bright_screen_switch_proc_write,
};

static int __init bright_screen_check_init(void)
{
	int ret = 0;


	g_bright_data.fb_notif.notifier_call = bright_fb_notifier_callback;
#ifdef CONFIG_DRM_MSM
	msm_drm_register_client(&g_bright_data.fb_notif);
#else
	fb_register_client(&g_bright_data.fb_notif);
#endif
    if (ret) {
		g_bright_data.status = ret;
        printk("bright block_screen_init register fb client fail\n");
		return ret;
    }
	proc_create("brightCheck", S_IRWXUGO, NULL, &bright_screen_check_proc_fops);
	proc_create("brightCheckStatus", S_IRWXUGO, NULL, &bright_screen_check_status_proc_fops);

	proc_create("brightSwitch", S_IRWXUGO, NULL, &bright_screen_switch_proc_fops);
	init_waitqueue_head(&bright_check_wq);
	INIT_WORK(&g_bright_data.error_happen_work, bright_error_happen_work);
	INIT_WORK(&g_bright_data.clear_error_happened_flag_work, bright_clear_error_happened_flag_work);
	INIT_DELAYED_WORK(&g_bright_data.determine_init_status_work, bright_determine_init_status_work);

	timer_setup((&g_bright_data.timer), (bright_timer_func),TIMER_DEFERRABLE);
	schedule_delayed_work(&g_bright_data.determine_init_status_work,
		msecs_to_jiffies(BRIGHT_INIT_STATUS_TIMEOUT_MS));
	g_bright_data.status = get_status();
	return 0;
}

static void __exit bright_screen_exit(void)
{

	del_timer(&g_bright_data.timer);
    fb_unregister_client(&g_bright_data.fb_notif);
}

late_initcall(bright_screen_check_init);
module_exit(bright_screen_exit);

#if IS_MODULE(CONFIG_OPLUS_FEATURE_THEIA)
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
#endif
MODULE_LICENSE("GPL v2");
