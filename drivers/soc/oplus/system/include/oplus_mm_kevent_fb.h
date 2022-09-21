/***************************************************************
** Copyright (C),  2018,  OPLUS Mobile Comm Corp.,  Ltd
** File : oplus_mm_kevent_fb.h
** Description : MM kevent fb data
** Version : 1.0
** Date : 2018/12/03
**
** ------------------------------- Revision History: -----------
**  <author>        <data>        <version >        <desc>
**   Guo.Ling          2018/12/03        1.0           Build this moudle
**   LiPing-M          2019/01/29        1.1           Add SMMU for QCOM
******************************************************************/
#ifndef _OPLUS_MM_KEVENT_FB_
#define _OPLUS_MM_KEVENT_FB_

//feedback to display for dump log
#define NEED_FEEDBACK_TO_DISPLAY  1

#define MM_KEVENT_MAX_PAYLOAD_SIZE	150

enum {
	MM_FB_KEY_RATELIMIT_NONE = 0,
	MM_FB_KEY_RATELIMIT_30MIN = 60 * 30 * 1000,
	MM_FB_KEY_RATELIMIT_1H = MM_FB_KEY_RATELIMIT_30MIN * 2,
	MM_FB_KEY_RATELIMIT_1DAY = MM_FB_KEY_RATELIMIT_1H * 24,
};

enum OPLUS_MM_DIRVER_FB_EVENT_MODULE {
	OPLUS_MM_DIRVER_FB_EVENT_TO_DISPLAY = 0,
	OPLUS_MM_DIRVER_FB_EVENT_TO_ATLAS
};

#ifdef NEED_FEEDBACK_TO_DISPLAY
enum OPLUS_MM_DIRVER_FB_EVENT_ID {
	OPLUS_MM_DIRVER_FB_EVENT_ID_ESD = 401,
	OPLUS_MM_DIRVER_FB_EVENT_ID_VSYNC,
	OPLUS_MM_DIRVER_FB_EVENT_ID_HBM,
	OPLUS_MM_DIRVER_FB_EVENT_ID_FFLSET,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_CMDQ,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_UNDERFLOW,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_FENCE,
	OPLUS_MM_DIRVER_FB_EVENT_ID_MTK_JS,
	OPLUS_MM_DIRVER_FB_EVENT_ID_SMMU,
	OPLUS_MM_DIRVER_FB_EVENT_ID_GPU_FAULT,
	OPLUS_MM_DIRVER_FB_EVENT_ID_PANEL_MATCH_FAULT,
	OPLUS_MM_DIRVER_FB_EVENT_ID_ERROR = 420,
	OPLUS_MM_DIRVER_FB_EVENT_ID_INFO = 421,
	OPLUS_MM_DIRVER_FB_EVENT_ID_AUDIO = 801,
};
#endif //NEED_FEEDBACK_TO_DISPLAY

#define OPLUS_FB_ADSP_CRASH_RATELIMIT    180*1000 //ms

#define DIAGNOSIS_EVENT_ID_MASK   0x80000000

//------- multimedia bigdata feedback event id, start ------------
#define OPLUS_AUDIO_EVENTID_ADSP_CRASH           10001
#define OPLUS_DISPLAY_EVENTID_DRIVER_ERR         12002

#define OPLUS_DISPLAY_EVENTID_GPU_FAULT          12005

//this id just for test or debug
#define OPLUS_MM_EVENTID_TEST_OR_DEBUG           30000
//------- multimedia bigdata feedback event id, end ------------

#ifdef CONFIG_OPLUS_FEATURE_MM_FEEDBACK
int upload_mm_fb_kevent_to_atlas_limit(unsigned int event_id, unsigned char *payload, int limit_ms);

int upload_mm_fb_kevent_limit(enum OPLUS_MM_DIRVER_FB_EVENT_MODULE module,  unsigned int event_id,
			 const char *name, int rate_limit_ms, char *payload);

#define mm_fb_kevent(m, id, name, rate_limit_ms, fmt, ...) \
	do { \
		char kv_data[MM_KEVENT_MAX_PAYLOAD_SIZE] = ""; \
		scnprintf(kv_data, sizeof(kv_data), fmt, ##__VA_ARGS__); \
		upload_mm_fb_kevent_limit(m, id, name, rate_limit_ms, kv_data); \
	} while (0)

#ifdef NEED_FEEDBACK_TO_DISPLAY
#define mm_fb_display_kevent(name, rate_limit_ms, fmt, ...) \
		mm_fb_kevent(OPLUS_MM_DIRVER_FB_EVENT_TO_DISPLAY, OPLUS_DISPLAY_EVENTID_DRIVER_ERR, name, rate_limit_ms, fmt, ##__VA_ARGS__);
#else //NEED_FEEDBACK_TO_DISPLAY
#define mm_fb_display_kevent(name, rate_limit_ms, fmt, ...) \
		mm_fb_kevent(OPLUS_MM_DIRVER_FB_EVENT_TO_ATLAS, OPLUS_DISPLAY_EVENTID_DRIVER_ERR, name, rate_limit_ms, fmt, ##__VA_ARGS__)
#endif //NEED_FEEDBACK_TO_DISPLAY
#define mm_fb_display_kevent_named(rate_limit_ms, fmt, ...) \
	do { \
		char name[MM_KEVENT_MAX_PAYLOAD_SIZE]; \
		scnprintf(name, sizeof(name), "%s:%d", __func__, __LINE__); \
		mm_fb_display_kevent(name, rate_limit_ms, fmt, ##__VA_ARGS__); \
	} while (0)
int mm_fb_kevent_init(void);
void mm_fb_kevent_deinit(void);

#else //CONFIG_OPLUS_FEATURE_MM_FEEDBACK

int upload_mm_fb_kevent_to_atlas_limit(unsigned int event_id, unsigned char *payload, int limit_ms) {return 0;}
#define mm_fb_kevent(m, name, rate_limit_ms, fmt, ...)  do {} while (0)
#define mm_fb_display_kevent(name, rate_limit_ms, fmt, ...)  do {} while (0)
#define mm_fb_display_kevent_named(rate_limit_ms, fmt, ...)  do {} while (0)
#endif //CONFIG_OPLUS_FEATURE_MM_FEEDBACK

#endif /* _OPLUS_MM_KEVENT_FB_ */

