#ifndef __AWINIC_CALI_FS_H__
#define __AWINIC_CALI_FS_H__
#include <linux/miscdevice.h>
#include <linux/cdev.h>


#define MOHM_TO_FIXED_RE(x)		((x << 12) / 1000)
#define FIXED_RE_TO_MOHM(x)		((x * 1000) >> 12)

/*********************************************
* dsp
**********************************************/
#define AW_CALI_STORE_EXAMPLE
#define AWINIC_ADSP_ENABLE
/* ERRO CALI RE */
#define AW_ERRO_CALI_VALUE (0)

#define AWINIC_DSP_MSG
#define AWINIC_DSP_HMUTE
#define AWINIC_DSP_MSG_HDR_VER (1)
#define AW_CALI_RE_DEFAULT_TIMER (1000)

#define AW_CALI_RE_MIN_TIMER (1000)
#define AW_SINGLE_DEV (1)
#define AW_DEV_MAX (2)

#define AFE_PORT_ID_AWDSP_RX (0x1005) /*AFE_PORT_ID_TERT_MI2S_RX*/
#define AW_DSP_TRY_TIME (3)
#define AW_DSP_SLEEP_TIME (10)


#define INLINE_PARAM_ID_ENABLE_CALI			(0x00000001)
#define INLINE_PARAM_ID_ENABLE_HMUTE			(0x00000002)
#define INLINE_PARAM_ID_F0_Q				(0x00000003)

/*dsp params id*/
#define AFE_PARAM_ID_AWDSP_RX_SET_ENABLE		(0x10013D11)
#define AFE_PARAM_ID_AWDSP_RX_PARAMS			(0x10013D12)
#define AFE_PARAM_ID_AWDSP_TX_SET_ENABLE		(0x10013D13)
#define AFE_PARAM_ID_AWDSP_RX_VMAX_L			(0X10013D17)
#define AFE_PARAM_ID_AWDSP_RX_VMAX_R			(0X10013D18)
#define AFE_PARAM_ID_AWDSP_RX_CALI_CFG_L		(0X10013D19)
#define AFE_PARAM_ID_AWDSP_RX_CALI_CFG_R		(0x10013d1A)
#define AFE_PARAM_ID_AWDSP_RX_RE_L			(0x10013d1B)
#define AFE_PARAM_ID_AWDSP_RX_RE_R			(0X10013D1C)
#define AFE_PARAM_ID_AWDSP_RX_NOISE_L			(0X10013D1D)
#define AFE_PARAM_ID_AWDSP_RX_NOISE_R			(0X10013D1E)
#define AFE_PARAM_ID_AWDSP_RX_F0_L			(0X10013D1F)
#define AFE_PARAM_ID_AWDSP_RX_F0_R			(0X10013D20)
#define AFE_PARAM_ID_AWDSP_RX_REAL_DATA_L		(0X10013D21)
#define AFE_PARAM_ID_AWDSP_RX_REAL_DATA_R		(0X10013D22)
#define AFE_PARAM_ID_AWDSP_RX_MSG			(0X10013D2A)

enum aw882xx_dsp_msg_type {
	DSP_MSG_TYPE_DATA = 0,
	DSP_MSG_TYPE_CMD = 1,
};

enum aw882xx_dsp_cali_st {
	DSP_BACK_CALI_CFG = 0,
	DSP_SET_CALI_CFG = 1,
};

enum aef_module_type {
	AW_RX_MODULE = 0,
	AW_TX_MODULE = 1,
};

enum afe_param_id_adsp {
	INDEX_PARAMS_ID_RX_PARAMS = 0,
	INDEX_PARAMS_ID_RX_ENBALE,
	INDEX_PARAMS_ID_TX_ENABLE,
	INDEX_PARAMS_ID_RX_VMAX,
	INDEX_PARAMS_ID_RX_CALI_CFG,
	INDEX_PARAMS_ID_RX_RE,
	INDEX_PARAMS_ID_RX_NOISE,
	INDEX_PARAMS_ID_RX_F0,
	INDEX_PARAMS_ID_RX_REAL_DATA,
	INDEX_PARAMS_ID_AWDSP_RX_MSG,
	INDEX_PARAMS_ID_MAX
};

struct aw_dsp_msg_hdr {
	int32_t type;
	int32_t opcode_id;
	int32_t version;
	int32_t reseriver[3];
};

/*********misc device ioctl fo cali**********/
#define AW882XX_CALI_CFG_NUM (3)
#define AW882XX_CALI_DATA_NUM (6)
#define AW882XX_PARAMS_NUM (400)
#define AW882XX_KILO_PARAMS_NUM (1000)


struct cali_cfg {
	int32_t data[AW882XX_CALI_CFG_NUM];
};
struct cali_data {
	int32_t data[AW882XX_CALI_DATA_NUM];
};
struct params_data {
	int32_t data[AW882XX_PARAMS_NUM];
};
struct ptr_params_data {
	int len;
	int32_t *data;
};

struct f0_q_data {
	int32_t data[4];
};

#define AW882XX_IOCTL_MAGIC			'a'
#define AW882XX_IOCTL_SET_CALI_CFG		_IOWR(AW882XX_IOCTL_MAGIC, 1, struct cali_cfg)
#define AW882XX_IOCTL_GET_CALI_CFG		_IOWR(AW882XX_IOCTL_MAGIC, 2, struct cali_cfg)
#define AW882XX_IOCTL_GET_CALI_DATA		_IOWR(AW882XX_IOCTL_MAGIC, 3, struct cali_data)
#define AW882XX_IOCTL_SET_NOISE			_IOWR(AW882XX_IOCTL_MAGIC, 4, int32_t)
#define AW882XX_IOCTL_GET_F0			_IOWR(AW882XX_IOCTL_MAGIC, 5, int32_t)
#define AW882XX_IOCTL_SET_CALI_RE		_IOWR(AW882XX_IOCTL_MAGIC, 6, int32_t)
#define AW882XX_IOCTL_GET_CALI_RE		_IOWR(AW882XX_IOCTL_MAGIC, 7, int32_t)
#define AW882XX_IOCTL_SET_VMAX			_IOWR(AW882XX_IOCTL_MAGIC, 8, int32_t)
#define AW882XX_IOCTL_GET_VMAX			_IOWR(AW882XX_IOCTL_MAGIC, 9, int32_t)
#define AW882XX_IOCTL_SET_PARAM			_IOWR(AW882XX_IOCTL_MAGIC, 10, struct params_data)
#define AW882XX_IOCTL_ENABLE_CALI		_IOWR(AW882XX_IOCTL_MAGIC, 11, int8_t)
#define AW882XX_IOCTL_SET_PTR_PARAM_NUM		_IOWR(AW882XX_IOCTL_MAGIC, 12, struct ptr_params_data)
#define AW882XX_IOCTL_GET_F0_Q			_IOWR(AW882XX_IOCTL_MAGIC, 13, struct f0_q_data)
#define AW882XX_IOCTL_SET_DSP_HMUTE		_IOWR(AW882XX_IOCTL_MAGIC, 14, int32_t)
#define AW882XX_IOCTL_SET_CALI_CFG_FLAG		_IOWR(AW882XX_IOCTL_MAGIC, 15, int32_t)


struct aw_misc_cali {
	struct miscdevice misc_device;
};

#ifdef CONFIG_DEBUG_FS
struct aw_dbg_cali {
	struct dentry *dbg_dir;
	struct dentry *dbg_range;
	struct dentry *dbg_cali;
	struct dentry *dbg_status;
	struct dentry *dbg_f0;
};
#endif

enum{
	AW_CALI_MODE_NONE = 0,
	AW_CALI_MODE_DBGFS,
	AW_CALI_MODE_MISC,
	AW_CALI_MODE_MAX
};

/*********************************************
* struct aw882xx cali
**********************************************/
struct aw_cali {
	unsigned char cali_mode;
	uint8_t first_cali_num;
	uint8_t end_cali_num;
	int32_t cali_re;
	int32_t cali_f0;
	int32_t re[AW_DEV_MAX];
	int32_t f0[AW_DEV_MAX];
	struct cali_cfg store_cfg[AW_DEV_MAX];
	uint32_t cali_re_time;
#ifdef CONFIG_DEBUG_FS
	struct aw_dbg_cali dbg_fs;
#endif
	struct aw_misc_cali misc;
};

/*********************************************
* aw882xx cali functions
**********************************************/
void aw_cali_init(struct aw_cali *cali);
void aw_cali_deinit(struct aw_cali *cali);

int aw_write_data_to_dsp(int index, void *data, int len, int channel);
int aw_read_data_from_dsp(int index, void *data, int len, int channel);
int aw_send_afe_module_enable(void *buf, int size, uint8_t type);

void aw882xx_load_cali_re(struct aw_cali *cali);
void aw882xx_parse_cali_mode_dt(struct aw_cali *cali);
void aw882xx_parse_cali_way_dt(struct aw_cali *cali);




#endif
