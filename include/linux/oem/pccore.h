#ifndef __INCLUDE_PCCORE__
#define __INCLUDE_PCCORE__

#ifndef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#endif

#define PCC_TAG "pccore:"

#define PCC_PARAMS 4
#define NR_CLU 3

#define pcc_logv(fmt...) \
	do { \
		if (pcclog_lv < 1) \
			pr_info(PCC_TAG fmt); \
	} while (0)

#define pcc_logi(fmt...) \
	do { \
		if (pcclog_lv < 2) \
			pr_info(PCC_TAG fmt); \
	} while (0)

#define pcc_logw(fmt...) \
	do { \
		if (pcclog_lv < 3) \
			pr_warn(PCC_TAG fmt); \
	} while (0)

#define pcc_loge(fmt...) pr_err(PCC_TAG fmt)
#define pcc_logd(fmt...) pr_debug(PCC_TAG fmt)

static unsigned int cluster_pd[NR_CLU] = {18, 17, 21};
static unsigned int cpufreq_pd_0[18] = {
	0,//300000
	0,//403200
	0,//499200
	0,//576000
	0,//672000
	1,//768000
	1,//844800
	1,//940800
	2,//1036800
	2,//1113600
	3,//1209600
	3,//1305600
	3,//1382400
	3,//1478400
	4,//1555200
	4,//1632000
	5,//1708800
	5//1785600
};

static unsigned int cpufreq_pd_1[17] = {
	 0,//710400
	 1,//825600
	 1,//940800
	 2,//1056000
	 2,//1171200
	 3,//1286400
	 3,//1401600
	 3,//1497600
	 4,//1612800
	 4,//1708800
	 4,//1804800
	 5,//1920000
	 6,//2016000
	 6,//2131200
	 7,//2227200
	 8,//2323200
	 8//2419200
};

static unsigned int cpufreq_pd_2[21] = {
	0,// 825600
	1,// 940800
	1,//1056000
	2,//1171200
	2,//1286400
	2,//1401600
	3,//1497600
	3,//1612800
	3,//1708800
	3,//1804800
	4,//1920000
	4,//2016000
	4,//2131200
	5,//2227200
	6,//2323200
	6,//2419200
	7,//2534400
	7,//2649600
	8,//2745600
	8,//2841600
	8 //2956800 for 18865
};

#endif // __INCLUDE_PCCORE__
