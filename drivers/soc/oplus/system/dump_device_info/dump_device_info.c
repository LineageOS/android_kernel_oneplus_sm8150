// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018-2020 Oplus. All rights reserved.
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/pstore.h>
#include <soc/qcom/socinfo.h>
#include <linux/soc/qcom/smem.h>
#include <linux/pstore.h>
#include <linux/notifier.h>
#include "../../../fs/pstore/internal.h"
#include <soc/oplus/system/oplus_project.h>



#define SMEM_CHIP_INFO 137

extern struct pstore_info *psinfo;
char console[8];
char baseband[8];
char serialno[8];
static char project_version[8];
static char pcb_version[8];
char soc_serialno[16];
#define BUILD_PROP  "/my_product/build.prop"
char build_version_key[24] = "ro.build.version.ota=";
char build_version[64];

static char buf[8192] = {'\0'};


#define GET_VERSION_INFO_TIMEOUT_MS     150000


struct get_version_info {
        
        struct delayed_work version_info_work;
        wait_queue_head_t info_thread_wq;
};

struct get_version_info g_version_info;


static int __init get_console_init(char *str)
{
        strcpy(console, str);
        return 0;
}


__setup("androidboot.console=", get_console_init);

static int __init get_baseband_init(char *str)
{
        strcpy(baseband,str);
        return 0;
}

__setup("androidboot.baseband=", get_baseband_init);

static int __init get_serialno_init(char *str)
{
        strcpy(serialno,str);
        pr_info("kernel serialno %s\n",str);
        return 0;
}
__setup("androidboot.serialno=", get_serialno_init);



/*device info init to black*/
static void  pstore_device_info_init(void )
{
        size_t oldsize;
        size_t size =0;

        struct ramoops_context *cxt = psinfo->data;
        struct pstore_record record;

        if (psinfo == NULL)
                return;

        size = cxt->device_info_size;

        pstore_record_init(&record, psinfo);
        record.type = PSTORE_TYPE_DEVICE_INFO;
        record.buf = psinfo->buf;
        record.size = size;

        oldsize = psinfo->bufsize;


        if (size > psinfo->bufsize)
                size = psinfo->bufsize;

        memset(record.buf, ' ', size);
        psinfo->write(&record);
  
        psinfo->bufsize = oldsize ;
}

static void board_soc_info_init(void)
{

        scnprintf(pcb_version, sizeof(pcb_version), "%x", get_PCB_Version()); 
        //strcpy(pcb_version, get_PCB_Version());

        scnprintf(project_version, sizeof(project_version), "%d", get_project());

}

static void pstore_write_device_info(const char *s, unsigned c)
{

        const char *e = s + c;

        if (psinfo == NULL)
                return;

        while (s < e) {
                struct pstore_record record;
                pstore_record_init(&record, psinfo);
                record.type = PSTORE_TYPE_DEVICE_INFO;

                if (c > psinfo->bufsize)
                        c = psinfo->bufsize;

                record.buf = (char *)s;
                record.size = c;
                psinfo->write(&record);
                s += c;
                c = e - s;
        }

}

static void write_device_info(const char *key, const char *value)
{
        pstore_write_device_info(key, strlen(key));
        pstore_write_device_info(": ", 2);
        pstore_write_device_info(value, strlen(value));
        pstore_write_device_info("\r\n", 2);
}


static void get_version_info_handle(struct work_struct *work)
{
        struct file *fp;
        int i = 0;
        ssize_t len = 0;
        char *substr;
        int old_fs;
        loff_t pos;
        printk("[get_version_info_handle]\n");
        fp = filp_open(BUILD_PROP, O_RDONLY, 0600);
        if (IS_ERR(fp)) {
                pr_info("open %s file fail fp:%p %d \n", BUILD_PROP, fp,PTR_ERR(fp));
                goto out;
        }

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        pos = 0;
        len = vfs_read(fp, buf, sizeof(buf), &pos);
        if (len < 0) {
                pr_info("read %s file error\n", BUILD_PROP);
        }

        substr = strstr(buf, build_version_key);
        pr_info("\n");
        pr_info("build_version:-%s--\n", substr);

        if(substr != NULL){
            while(*substr != '\n') {
                if (*substr == '=')
                    break;
                substr++;
            }

            while (*(++substr) != '\n') {
                build_version[i] = *(substr);
                i++;
            }
        }

        pr_info("build_version_value:%s--\n", build_version);
        write_device_info("software version", build_version);


out:
	if (IS_ERR(fp)) {
                pr_info("open is failed, cannot to read\n");
        }else{
                filp_close(fp, NULL);
                set_fs(old_fs);

        }

}

static int __init init_device_info(void)
{
        char socinfo_platform_version[16];
        pstore_device_info_init();
        
        board_soc_info_init();
        
        write_device_info("project version", project_version);
        write_device_info("pcb version", pcb_version);
        write_device_info("soc version", socinfo_get_id_string());
        
        scnprintf(soc_serialno, sizeof(soc_serialno), "%x", socinfo_get_serial_number());
        write_device_info("socinfo serial_number", soc_serialno);
        
        scnprintf(socinfo_platform_version, sizeof(soc_serialno), "%x", socinfo_get_platform_version());
        write_device_info("socinfo platform version", socinfo_platform_version);
                
        write_device_info("kernel version", linux_banner);
        write_device_info("boot command", saved_command_line);
                
        INIT_DELAYED_WORK(&g_version_info.version_info_work, get_version_info_handle);
        schedule_delayed_work(&g_version_info.version_info_work, msecs_to_jiffies(GET_VERSION_INFO_TIMEOUT_MS));
        
        return 0;
}

late_initcall(init_device_info);
