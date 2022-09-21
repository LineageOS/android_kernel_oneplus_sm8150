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

#define BLACKSCREEN_COUNT_FILE	"/data/oplus/log/bsp/blackscreen_count.txt"
#define BLACKSCREEN_HAPPENED_FILE	"/data/oplus/log/bsp/blackscreen_happened.txt"


#define BLACK_DEBUG_PRINTK(a, arg...)\
    do{\
         printk("[black_check]: " a, ##arg);\
    }while(0)

#define BLACK_STATUS_INIT 				1
#define BLACK_STATUS_INIT_FAIL 			2
#define BLACK_STATUS_INIT_SUCCEES		3
#define BLACK_STATUS_CHECK_ENABLE 		4
#define BLACK_STATUS_CHECK_DISABLE 		5
#define BLACK_STATUS_CHECK_DEBUG 		6

#define BLACK_CLEAR_ERROR_HAPPENED		0
#define BLACK_MARK_ERROR_HAPPENED		1

#define BLACK_MAX_WRITE_NUMBER			3000

#define BLACK_DEFAULT_TIMEOUT_MS		120000
#define BLACK_INIT_STATUS_TIMEOUT_MS	150000
#define BLACK_SLOW_STATUS_TIMEOUT_MS	20000

#define BLACK_STAGE_CONSUMED		    80000
#define BLACK_STAGE_END 		        90000

#define SIG_THEIA (SIGRTMIN + 0x13)
#define TASK_INIT_COMM                                     "init"
static int def_swich=1;

static char *flow_buf=NULL;
static char *flow_buf_curr=NULL;
static const int flow_size=16;
static const int stage_brief_size=64;
static const int stage_total_size=stage_brief_size*flow_size;

static int flow_index=0;
static int stage_start=0;
static int stage_end=0;

struct black_data{
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

static void black_timer_func(struct timer_list *t);

static struct black_data g_black_data = {
	.is_panic = 0,
	.status = BLACK_STATUS_INIT,
#ifdef CONFIG_DRM_MSM
	.blank = MSM_DRM_BLANK_UNBLANK,
#else
	.blank = FB_BLANK_UNBLANK,
#endif
	.timeout_ms = BLACK_SLOW_STATUS_TIMEOUT_MS,
	.error_detected = 0,
	.error_count_new = 0,
	.error_count_old = 0,
};

static wait_queue_head_t black_check_wq;
static int ur_stage;

void theia_pwk_stage_start(void)
{
    BLACK_DEBUG_PRINTK("set start %x:  %x   %x   flow_buf\n", flow_buf,flow_buf_curr,flow_index);

    stage_start = flow_index;
}
EXPORT_SYMBOL(theia_pwk_stage_start);

void theia_pwk_stage_end(void)
{
    BLACK_DEBUG_PRINTK("set end %x:  %x   %x   flow_buf\n", flow_buf,flow_buf_curr,flow_index);

    stage_end = flow_index;
}
EXPORT_SYMBOL(theia_pwk_stage_end);

int black_screen_timer_restart(void)
{
	//BLACK_DEBUG_PRINTK("black_screen_timer_restart:blank = %d,status = %d\n",g_black_data.blank,g_black_data.status);
	if(g_black_data.status != BLACK_STATUS_CHECK_ENABLE && g_black_data.status != BLACK_STATUS_CHECK_DEBUG){
		BLACK_DEBUG_PRINTK("black_screen_timer_restart:g_black_data.status = %d return\n",g_black_data.status);
		return g_black_data.status;
	}
#ifdef CONFIG_DRM_MSM
	if(g_black_data.blank == MSM_DRM_BLANK_POWERDOWN)
#else
	if(g_black_data.blank == FB_BLANK_POWERDOWN)
#endif
    {
		mod_timer(&g_black_data.timer,jiffies + msecs_to_jiffies(g_black_data.timeout_ms));
		BLACK_DEBUG_PRINTK("black_screen_timer_restart: black screen check start %u\n",g_black_data.timeout_ms);
        theia_pwk_stage_start();
		return 0;
	}
	return g_black_data.blank;
}
EXPORT_SYMBOL(black_screen_timer_restart);

static ssize_t black_screen_check_proc_read(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
    char page[256] = {0};
    int len = 0;

    len = sprintf(&page[len],"%d %u %d %d\n",
        g_black_data.status,g_black_data.timeout_ms,g_black_data.is_panic,g_black_data.get_log);

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

static ssize_t black_screen_check_proc_write(struct file *file, const char __user *buf,
		size_t count,loff_t *off)
{
    int tmp_status = 0;
    int tmp_timeout = 0;
    int tmp_isPanic = 0;
    int tmp_log = 0;
    int ret = 0;
    char buffer[40] = {0};

    if(g_black_data.status == BLACK_STATUS_INIT || g_black_data.status == BLACK_STATUS_INIT_FAIL){
    	BLACK_DEBUG_PRINTK("%s init not finish: status = %d\n", __func__, g_black_data.status);
    	return count;
    }

    if (count > 40) {
       count = 40;
    }

    if (copy_from_user(buffer, buf, count)) {
    	BLACK_DEBUG_PRINTK("%s: read proc input error.\n", __func__);
    	return count;
    }
    ret = sscanf(buffer, "%d %u %d %d", &tmp_status,&tmp_timeout,&tmp_isPanic,&tmp_log);

    if (ret == 1) {
    	g_black_data.status = tmp_status;
    } else if(ret == 4){
    	g_black_data.status = tmp_status;
    	if(tmp_timeout)
    		g_black_data.timeout_ms = tmp_timeout;
    	g_black_data.is_panic = tmp_isPanic;
    	g_black_data.get_log = tmp_log;
    }else{
        BLACK_DEBUG_PRINTK("%s invalid content: '%s', length = %zd\n", __func__, buffer, count);
    }
    def_swich = tmp_status == BLACK_STATUS_CHECK_ENABLE|| BLACK_STATUS_CHECK_DEBUG == tmp_status;

    BLACK_DEBUG_PRINTK("%s: status = %d timeout_ms = %u is_panic = %d\n",
    	__func__, g_black_data.status,g_black_data.timeout_ms,g_black_data.is_panic);

	return count;

}
static void record_stage(const char *buf)
{

    memset(flow_buf_curr, 0, stage_brief_size);
    memcpy(flow_buf_curr, buf, stage_brief_size - 1);
    *(flow_buf_curr + stage_brief_size - 1)='\n';
    flow_buf_curr+=stage_brief_size;

    //w lock index
    flow_index++;
    if(flow_index == flow_size){
        flow_index = 0;
        flow_buf_curr = flow_buf;
    }
    //w lock index

    //flow_buf_curr+=stage_brief_size;

}

static ssize_t black_screen_report_proc_write(struct file *file, const char __user *buf,
		size_t count,loff_t *off)
{

    char buffer[stage_brief_size] = {0};

	if(g_black_data.status == BLACK_STATUS_INIT || g_black_data.status == BLACK_STATUS_INIT_FAIL){
		BLACK_DEBUG_PRINTK("%s init not finish: status = %d\n", __func__, g_black_data.status);
		return count;
	}

    if (count > stage_brief_size) {
       count = stage_brief_size;
    }

    if (copy_from_user(buffer, buf, count)) {
		BLACK_DEBUG_PRINTK("%s: read proc input error.\n", __func__);
		return count;
    }

    record_stage(buffer);
	return count;
}


static ssize_t rick_pick_fail_stage(struct file *file, char __user *buf,
        size_t count,loff_t *off)
{

    int start,end;
    char *t_sbuf= NULL;
    char *buf_curr = NULL;
    char sbuf[stage_total_size] = {0};

    if(stage_start == stage_end)
    {
        return 0;
    }

    buf_curr = stage_start*stage_brief_size + flow_buf;
    start = stage_start;
    end = stage_end+1;
    if(end == flow_size)
    {
        end = 0;
    }
    BLACK_DEBUG_PRINTK("rick_pick_fail_stage %d:  %d   \n", stage_start,stage_end);
    BLACK_DEBUG_PRINTK("rick_pick_fail_stage %x:  %x   %x   \n", flow_buf,flow_buf_curr,flow_index);

    t_sbuf= sbuf;
    for(;start!= end;)
    {
        memset(t_sbuf,0,stage_brief_size);
        memcpy(t_sbuf,buf_curr,stage_brief_size);

        buf_curr+=stage_brief_size;
        t_sbuf+=stage_brief_size;

        //w lock index
        start++;
        if(start == flow_size)
        {
            start = 0;
            buf_curr = flow_buf;
        }
    }

    return simple_read_from_buffer(buf, count, off, sbuf,
               stage_brief_size*flow_size);

}
static void print_all_flow(void)
{
    char *t_sbuf= NULL;

    int start = 0;
    t_sbuf = flow_buf;

    for(;start < flow_size;start++)
    {
         BLACK_DEBUG_PRINTK("%s  \n", t_sbuf);
        t_sbuf+=stage_brief_size;
    }
    BLACK_DEBUG_PRINTK("exit print_all \n");
}

static ssize_t black_screen_report_proc_read(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
    BLACK_DEBUG_PRINTK("enter black_screen_report_proc_read %d  %d",count,*off);

    print_all_flow();

    return rick_pick_fail_stage( file, buf, count, off);
}

static int passok(int stage)
{
    return stage==0 || stage == BLACK_STAGE_END || stage == BLACK_STAGE_CONSUMED;
}
static ssize_t black_screen_ur_process(int stage)
{
    if(passok(stage))
	{
	    ur_stage = stage;
	    del_timer(&g_black_data.timer);
			BLACK_DEBUG_PRINTK("del timer,stage: %d  blank:%d\n",
				stage, g_black_data.blank);
	}
	return 0;
}

static ssize_t black_screen_switch_proc_write(struct file *file, const char __user *buf,
		size_t count,loff_t *off)
{
	int tmp_stage = 0;
	int ret = 0;
    char buffer[40] = {0};

	if(g_black_data.status == BLACK_STATUS_INIT || g_black_data.status == BLACK_STATUS_INIT_FAIL){
		BLACK_DEBUG_PRINTK("%s init not finish: status = %d\n", __func__, g_black_data.status);
		return count;
	}

    if (count > 40) {
       count = 40;
    }

    if (copy_from_user(buffer, buf, count)) {
		BLACK_DEBUG_PRINTK("%s: read proc input error.\n", __func__);
		return count;
    }

	ret = sscanf(buffer, "%d", &tmp_stage);
    if(1 == tmp_stage)
    {
        del_timer(&g_black_data.timer);
    }

    if (ret == 1) {
		BLACK_DEBUG_PRINTK("%s recv us report  stage: %d\n", __func__, tmp_stage);
		black_screen_ur_process(tmp_stage);
	}else{
	    BLACK_DEBUG_PRINTK("%s invalid content: '%s', length = %zd\n", __func__, buf, count);
	}

	return count;
}
static ssize_t black_screen_switch_proc_read(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
    return 0;
}

struct file_operations black_screen_check_proc_fops = {
	.read = black_screen_check_proc_read,
	.write = black_screen_check_proc_write,
};

struct file_operations black_screen_report_proc_fops = {
	.read = black_screen_report_proc_read,
	.write = black_screen_report_proc_write,
};
struct file_operations black_screen_switch_proc_fops = {
	.read = black_screen_switch_proc_read,
	.write = black_screen_switch_proc_write,
};


static ssize_t black_screen_check_status_proc_read(struct file *file, char __user *buf,
		size_t count,loff_t *off)
{
    char page[256] = {0};
    int len = 0;
    int error_detected = 0;

    BLACK_DEBUG_PRINTK("read black_err_detected:%d count:%d begin\n",
    	g_black_data.error_detected, g_black_data.error_count_new);
    wait_event_interruptible(black_check_wq, g_black_data.error_detected>0);
    BLACK_DEBUG_PRINTK("read black_err_detected:%d count:%d end\n",
    	g_black_data.error_detected, g_black_data.error_count_new);

    if (g_black_data.error_detected) {
    	len = sprintf(&page[len],"%d", g_black_data.error_count_new);
    } else {
    	len = sprintf(&page[len],"%d", 0);
    }
    error_detected = g_black_data.error_detected;
    g_black_data.error_detected = 0;
    if(len > *off)
       len -= *off;
    else
       len = 0;
    if(copy_to_user(buf,page,(len < count ? len : count))){
       return -EFAULT;
    }
    *off += len < count ? len : count;
    if (error_detected) {
    	schedule_work(&g_black_data.clear_error_happened_flag_work);
    }
    return (len < count ? len : count);
}

struct file_operations black_screen_check_status_proc_fops = {
	.read = black_screen_check_status_proc_read,
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


static void black_show_coretask_state(void)
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

static bool black_check_error_happened_before(struct black_data *bla_data)
{
	struct file *fp;
	loff_t pos;
	ssize_t len = 0;
	char buf[256] = {'\0'};
	int error_happened = 0;

	fp = filp_open(BLACKSCREEN_HAPPENED_FILE, O_RDWR | O_CREAT, 0664);
	if (IS_ERR(fp)) {
		BLACK_DEBUG_PRINTK("create %s file error fp:%p\n", BLACKSCREEN_COUNT_FILE, fp);
		return false;
	}

	pos = 0;
	len = kernel_read(fp, buf, sizeof(buf), &pos);
	if (len < 0) {
		BLACK_DEBUG_PRINTK("read %s file error\n", BLACKSCREEN_HAPPENED_FILE);
		goto out;
	}
	sscanf(buf, "%d", &error_happened);
out:
	BLACK_DEBUG_PRINTK("error_happened before:%d\n", error_happened);
	filp_close(fp, NULL);

	return (error_happened > 0 ? true : false);
}

static int black_mark_or_clear_error_happened_flag(struct black_data *bla_data, int mark_or_clear)
{
	struct file *fp;
	loff_t pos;
	ssize_t len = 0;
	char buf[256] = {'\0'};
	int old_value = 0;

	fp = filp_open(BLACKSCREEN_HAPPENED_FILE, O_RDWR | O_CREAT, 0664);
	if (IS_ERR(fp)) {
		BLACK_DEBUG_PRINTK("create %s file error fp:%p\n", BLACKSCREEN_HAPPENED_FILE, fp);
		return -1;
	}

	//read to check whether write or not
	pos = 0;
	len = kernel_read(fp, buf, sizeof(buf), &pos);
	if (len < 0) {
		BLACK_DEBUG_PRINTK("read %s file error\n", BLACKSCREEN_HAPPENED_FILE);
		goto out;
	}
	sscanf(buf, "%d", &old_value);
	if (old_value == mark_or_clear) {
		BLACK_DEBUG_PRINTK("buf:%s, old_value:%d, mark_or_clear:%d\n",
			buf, old_value, mark_or_clear);
		goto out;
	}
	//read end

	//write begin
	sprintf(buf, "%d\n", mark_or_clear);
	pos = 0;
	len = kernel_write(fp, buf, strlen(buf), &pos);
	if (len < 0)
		BLACK_DEBUG_PRINTK("write %s file error\n", BLACKSCREEN_HAPPENED_FILE);
	//write end

	pos = 0;
	kernel_read(fp, buf, sizeof(buf), &pos);
	BLACK_DEBUG_PRINTK("black_mark_or_clear_error_happened_flag %s, mark:%d\n",
		buf, mark_or_clear);
out:
	filp_close(fp, NULL);

	return len;
}

static int black_write_error_count(struct black_data *bla_data)
{
	struct file *fp;
	loff_t pos;
	ssize_t len = 0;
	char buf[256] = {'\0'};
	static bool have_read_old = false;

	fp = filp_open(BLACKSCREEN_COUNT_FILE, O_RDWR | O_CREAT, 0664);
	if (IS_ERR(fp)) {
		BLACK_DEBUG_PRINTK("create %s file error fp:%p %d \n", BLACKSCREEN_COUNT_FILE, fp,PTR_ERR(fp));
		return -1;
	}

	//read old error_count begin
	if (have_read_old == false) {
		pos = 0;
		len = kernel_read(fp, buf, sizeof(buf), &pos);
		if (len < 0) {
			BLACK_DEBUG_PRINTK("read %s file error\n", BLACKSCREEN_COUNT_FILE);
			goto out;
		}
		sscanf(buf, "%d", &bla_data->error_count_old);
		bla_data->error_count_new = bla_data->error_count_new + bla_data->error_count_old;
		have_read_old = true;
		BLACK_DEBUG_PRINTK("read_buf:%s count_old:%d count_new:%d\n",
			buf, bla_data->error_count_old, bla_data->error_count_new);
	}
	//read old error_count_new end

	sprintf(buf, "%d\n", bla_data->error_count_new);

	pos = 0;
	len = kernel_write(fp, buf, strlen(buf), &pos);
	if (len < 0)
		BLACK_DEBUG_PRINTK("write %s file error\n", BLACKSCREEN_COUNT_FILE);

	pos = 0;
	kernel_read(fp, buf, sizeof(buf), &pos);
	BLACK_DEBUG_PRINTK("black_write_error_count %s\n", buf);

out:
	filp_close(fp, NULL);

	return len;
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
			return BLACK_STATUS_CHECK_ENABLE;
    }
    return BLACK_STATUS_INIT_SUCCEES;
#else
    if((get_boot_mode() == NORMAL_BOOT) || (get_boot_mode() == ALARM_BOOT)) 
    {
       if(def_swich)
			return BLACK_STATUS_CHECK_ENABLE;
    }
    return BLACK_STATUS_INIT_SUCCEES;

#endif
}

static int get_log_swich()
{
    return  (BLACK_STATUS_CHECK_ENABLE == get_status()||BLACK_STATUS_CHECK_DEBUG == get_status())&& g_black_data.get_log;
}

static void dump_freeze_log(void)
{
    struct task_struct *t_init;
    t_init = NULL;

	if (!get_log_swich())
	{
	    BLACK_DEBUG_PRINTK("log swich is disable ,return !!!");
	    return;
	}

    find_task_by_comm(TASK_INIT_COMM, &t_init);
    if(NULL != t_init)
    {
        BLACK_DEBUG_PRINTK("send signal %d ", SIG_THEIA);
        send_sig(SIG_THEIA, t_init, 0);
    }
}

static void black_determine_init_status_work(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct black_data *bla_data =
			container_of(dwork, struct black_data, determine_init_status_work);

	if(bla_data->status != BLACK_STATUS_CHECK_ENABLE && bla_data->status != BLACK_STATUS_CHECK_DEBUG){
		BLACK_DEBUG_PRINTK("init_status status: %d, return\n", bla_data->status);
		return;
	}
	if (black_check_error_happened_before(bla_data)) {
		bla_data->error_detected = 1;
		wake_up_interruptible(&black_check_wq);
	}
}

static void black_clear_error_happened_flag_work(struct work_struct *work)
{
	struct black_data *bla_data
		= container_of(work, struct black_data, clear_error_happened_flag_work);

	black_mark_or_clear_error_happened_flag(bla_data, BLACK_CLEAR_ERROR_HAPPENED);
}

static void black_error_happen_work(struct work_struct *work)
{
	struct black_data *bla_data
							= container_of(work, struct black_data, error_happen_work);

	if (bla_data->error_count_new < BLACK_MAX_WRITE_NUMBER) {
		bla_data->error_count_new++;
		dump_freeze_log();
		black_write_error_count(bla_data);
		black_mark_or_clear_error_happened_flag(bla_data, BLACK_MARK_ERROR_HAPPENED);
	}
	BLACK_DEBUG_PRINTK("black_error_happen_work error_count_new = %d old = %d stage = %d\n",
		bla_data->error_count_new, bla_data->error_count_old,ur_stage);
	theia_pwk_stage_end();
	if(bla_data->is_panic) {
		//1.meminfo
		//2. show all D task
#if IS_MODULE(CONFIG_OPLUS_FEATURE_THEIA)
		handle_sysrq('w');
#else
		show_state_filter(TASK_UNINTERRUPTIBLE);
#endif
		//3. show system_server zoygot surfacefliger state
		black_show_coretask_state();
		//4.current cpu registers :skip for minidump
		//panic("black screen detected, force panic");
	}
	bla_data->error_detected = 1;
	wake_up_interruptible(&black_check_wq);
}

static void black_timer_func(struct timer_list *t)
{
	struct black_data *p = from_timer(p, t, timer);
	BLACK_DEBUG_PRINTK("black_timer_func is called\n");
	schedule_work(&p->error_happen_work);
}

#ifdef CONFIG_DRM_MSM
static int black_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct msm_drm_notifier *evdata = data;
	int *blank;

 	if (event == MSM_DRM_EVENT_BLANK && evdata && evdata->data)
	{
		blank = evdata->data;
		g_black_data.blank = *blank;
		if(g_black_data.status != BLACK_STATUS_CHECK_DEBUG){
			del_timer(&g_black_data.timer);
			BLACK_DEBUG_PRINTK("black_fb_notifier_callback: del timer,event:%lu status:%d blank:%d\n",
				event, g_black_data.status, g_black_data.blank);
		} else {
			BLACK_DEBUG_PRINTK("black_fb_notifier_callback:event = %lu status:%d blank:%d\n",
				event,g_black_data.status,g_black_data.blank);
		}
	}
	else if (event == MSM_DRM_EARLY_EVENT_BLANK && evdata && evdata->data)
	{
		blank = evdata->data;

		blank = evdata->data;
		g_black_data.blank = *blank;
		if(g_black_data.status != BLACK_STATUS_CHECK_DEBUG){
			del_timer(&g_black_data.timer);
			BLACK_DEBUG_PRINTK("black_fb_notifier_callback: del timer,event:%lu status:%d blank:%d\n",
				event, g_black_data.status, g_black_data.blank);
		} else {
			BLACK_DEBUG_PRINTK("black_fb_notifier_callback:event = %lu status:%d blank:%d\n",
				event,g_black_data.status,g_black_data.blank);
		}
	} else {
		BLACK_DEBUG_PRINTK("black_fb_notifier_callback:event = %lu status:%d\n",event,g_black_data.status);
	}
	return 0;
}
#else
#ifndef FB_EARLY_EVENT_BLANK
#define FB_EARLY_EVENT_BLANK    0x10
#endif

static int black_fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;

	if (evdata && evdata->data && event == FB_EVENT_BLANK)
	{
		blank = evdata->data;
		g_black_data.blank = *blank;
		if(g_black_data.status != BLACK_STATUS_CHECK_DEBUG){
			del_timer(&g_black_data.timer);
			BLACK_DEBUG_PRINTK("black_fb_notifier_callback: del timer,event:%lu status:%d blank:%d\n",
				event, g_black_data.status, g_black_data.blank);
		} else {
			BLACK_DEBUG_PRINTK("black_fb_notifier_callback:event = %lu status:%d blank:%d\n",
				event,g_black_data.status,g_black_data.blank);
		}
	}
    else if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK)
	{
		blank = evdata->data;
		g_black_data.blank = *blank;
		if(g_black_data.status != BLACK_STATUS_CHECK_DEBUG){
			del_timer(&g_black_data.timer);
			BLACK_DEBUG_PRINTK("black_fb_notifier_callback: del timer,event:%lu status:%d blank:%d\n",
				event, g_black_data.status, g_black_data.blank);
		} else {
			BLACK_DEBUG_PRINTK("black_fb_notifier_callback:event = %lu status:%d blank:%d\n",
				event,g_black_data.status,g_black_data.blank);
		}
    } else {
		BLACK_DEBUG_PRINTK("black_fb_notifier_callback:event = %lu status:%d\n",event,g_black_data.status);
	}
	return 0;
}
#endif /* CONFIG_DRM_MSM */

static int __init black_screen_check_init(void)
{
	int ret = 0;

	g_black_data.fb_notif.notifier_call = black_fb_notifier_callback;
#ifdef CONFIG_DRM_MSM
	msm_drm_register_client(&g_black_data.fb_notif);
#else
	fb_register_client(&g_black_data.fb_notif);
#endif
    if (ret) {
		g_black_data.status = ret;
        printk("block_screen_init register fb client fail\n");
		return ret;
    }
	proc_create("blackCheck", S_IRWXUGO, NULL, &black_screen_check_proc_fops);
	proc_create("blackCheckStatus", S_IRWXUGO, NULL, &black_screen_check_status_proc_fops);
	proc_create("theiaPwkReport", S_IRWXUGO, NULL, &black_screen_report_proc_fops);
	proc_create("blackSwitch", S_IRWXUGO, NULL, &black_screen_switch_proc_fops);
	init_waitqueue_head(&black_check_wq);
	INIT_WORK(&g_black_data.error_happen_work, black_error_happen_work);
	INIT_WORK(&g_black_data.clear_error_happened_flag_work, black_clear_error_happened_flag_work);
	INIT_DELAYED_WORK(&g_black_data.determine_init_status_work, black_determine_init_status_work);

	timer_setup((&g_black_data.timer), (black_timer_func),TIMER_DEFERRABLE);
	schedule_delayed_work(&g_black_data.determine_init_status_work,
			msecs_to_jiffies(BLACK_INIT_STATUS_TIMEOUT_MS));
	g_black_data.status = get_status();

    flow_buf= vmalloc(stage_brief_size*flow_size);
    if (!flow_buf)
        return -ENOMEM;
    memset(flow_buf,0,stage_brief_size*flow_size);
    flow_buf_curr = flow_buf;

	return 0;
}

static void __exit black_screen_exit(void)
{
	del_timer(&g_black_data.timer);
    fb_unregister_client(&g_black_data.fb_notif);
    if (!flow_buf)
    {
        vfree(flow_buf);
        flow_buf = NULL;
    }
}

late_initcall(black_screen_check_init);
module_exit(black_screen_exit);

#if IS_MODULE(CONFIG_OPLUS_FEATURE_THEIA)
MODULE_IMPORT_NS(VFS_internal_I_am_really_a_filesystem_and_am_NOT_a_driver);
#endif
MODULE_LICENSE("GPL v2");
