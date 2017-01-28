/*
 * MELFAS MMS400 Touchscreen for MediaTek
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 */

//Config debug msg : Must be disabled for production builds
#if 0	// 0 : disable, 1 : enable
#define DEBUG	
#endif

//Select Chip Model
#define CONFIG_TOUCHSCREEN_MELFAS_MMS438 //linfeng
//#define CONFIG_TOUCHSCREEN_MELFAS_MMS449
//#define CONFIG_TOUCHSCREEN_MELFAS_MMS458
#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
//#include <linux/io.h>

#include <linux/init.h>

//Include
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/limits.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio_event.h>
#include <linux/wakelock.h>
#include <asm/uaccess.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

//Include MTK
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/dma-mapping.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include <cust_eint.h>
#include "cust_gpio_usage.h"
#include "tpd.h"
#include "tpd_custom_melfas_mms400.h"

//Include platform data
#include "melfas_mms.h"

//Include register map
#include "melfas_mms400_reg.h"

//Chip info
#ifdef CONFIG_TOUCHSCREEN_MELFAS_MMS438
#define CHIP_MMS438
#define CHIP_NAME		"MMS438"
#define CHIP_FW_CODE	"M4H0"
#define FW_UPDATE_TYPE	"MMS438"
#endif
#ifdef CONFIG_TOUCHSCREEN_MELFAS_MMS449
#define CHIP_MMS449
#define CHIP_NAME		"MMS449"
#define CHIP_FW_CODE	"M4HP"
#define FW_UPDATE_TYPE	"MMS438"
#endif
#ifdef CONFIG_TOUCHSCREEN_MELFAS_MMS458
#define CHIP_MMS458
#define CHIP_NAME		"MMS458"
#define CHIP_FW_CODE	"M4HN"  //COMPARE TO MELFAS_MFSB
#define FW_UPDATE_TYPE	"MMS438"
#endif

//Config driver
#define MMS_USE_INPUT_OPEN_CLOSE	0	// 0 or 1
#define I2C_RETRY_COUNT			3	// 2~
#define RESET_ON_EVENT_ERROR		0	// 0 or 1
#define ESD_COUNT_FOR_DISABLE	7	// 7~
#define INPUT_REPORT_TYPE 		0	// 0 or 1 linfeng

//Features
#define MMS_USE_NAP_MODE			0	// 0 or 1
#define MMS_USE_GESTURE_WAKEUP_MODE	0	// 0 or 1
#define MMS_USE_TEST_MODE			1	// 0 or 1 (Optional - Sysfs test functions)
#define MMS_USE_CMD_MODE			1	// 0 or 1 (Optional - Sysfs command functions)
#define MMS_USE_DEV_MODE			1	// 0 or 1 (Optional - Dev mode for debugging)

//Input value
#define MAX_FINGER_NUM 			10
#define INPUT_AREA_MIN 			       0
#define INPUT_AREA_MAX 			255
#define INPUT_PRESSURE_MIN 		0
#define INPUT_PRESSURE_MAX 		255
#define INPUT_TOUCH_MAJOR_MIN 	0
#define INPUT_TOUCH_MAJOR_MAX 	255
#define INPUT_TOUCH_MINOR_MIN 	0
#define INPUT_TOUCH_MINOR_MAX 	255
#define INPUT_ANGLE_MIN 			0
#define INPUT_ANGLE_MAX 			255
#define INPUT_HOVER_MIN 			0
#define INPUT_HOVER_MAX 			255
#define INPUT_PALM_MIN 			       0
#define INPUT_PALM_MAX 			1

//Firmware update
#define INTERNAL_FW_PATH			"melfas/melfas_mms400.mfsb"		//path of firmware included in the kernel image (/firmware)
#define EXTERNAL_FW_PATH			"/sdcard/melfas_mms400.mfsb"	//path of firmware in external storage
#define MMS_USE_AUTO_FW_UPDATE	0	// 0 or 1
#define MMS_FW_MAX_SECT_NUM		4
#define MMS_FW_UPDATE_DEBUG		0	// 0 or 1
#define MMS_FW_UPDATE_SECTION	1	// 0 or 1
#define MMS_EXT_FW_FORCE_UPDATE	1	//0 or 1

//Command mode
#define CMD_LEN 					32
#define CMD_RESULT_LEN 			512
#define CMD_PARAM_NUM	 			8

//hyeog
#define MAX_SECTION_NUM		3

#if MMS_USE_CALLBACK
//Callback functions
struct mms_callbacks {
	void (*inform_charger) (struct mms_callbacks *, int);
	//void (*inform_display) (struct mms_callbacks *, int);
	//...
};

extern struct mms_callbacks *mms_inform_callbacks;
#endif

/**
* MTK
*/
#ifdef DEBUG
#define MMS_DEBUG(x...)	printk(MMS_DEVICE_NAME": "x)
#else
#define MMS_DEBUG(format, args...)	{}
#endif

/**
* Device info structure
*/
struct mms_ts_info {
	struct i2c_client *client;
	struct input_dev *input_dev;
	char phys[32];
	struct mms_platform_data *pdata;

	dev_t mms_dev;
	struct class *class;
	
	struct mutex lock;
	struct mutex lock_test;
	struct mutex lock_cmd;
	struct mutex lock_dev;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	bool irq_enabled;
	bool irq_pause;
	
	int irq;
	bool enabled;
	bool init;	
	char *fw_name;
	
	int power_state;
		
	u8 product_name[16];
	int max_x;
	int max_y;
	u8 node_x;
	u8 node_y;
	u8 node_key;
	u8 fw_version[8];
	u8 event_size;

	bool tkey_enable;

	u8 touch_state_scr[MAX_FINGER_NUM];
	u8 touch_state_key[MAX_FINGER_NUM];
	
	u8 nap_mode;
	u8 glove_mode;
	u8 charger_mode;
	u8 cover_mode;
	
	u8 esd_cnt;
	bool disable_esd;

	u8 *print_buf;	
	int *image_buf;
	
	bool test_busy;
	bool cmd_busy;
	bool dev_busy;

#if MMS_USE_CMD_MODE
	dev_t cmd_dev_t;
	struct device *cmd_dev;
	struct class *cmd_class;
	struct list_head cmd_list_head;
	u8 cmd_state;
	char cmd[CMD_LEN];
	char cmd_result[CMD_RESULT_LEN];
	int cmd_param[CMD_PARAM_NUM];
	int cmd_buffer_size;
#endif	

#if MMS_USE_DEV_MODE
	struct cdev cdev;
	u8 *dev_fs_buf;
#endif	

#if MMS_USE_CALLBACK
	void (*register_callback)(void *);
	struct mms_callbacks callbacks;
#endif

};

/**
* Firmware binary header info
*/
struct mms_bin_hdr {
	char	tag[8];
	u16	core_version;
	u16	section_num;
	u16	contains_full_binary;
	u16	reserved0;

	u32	binary_offset;
	u32	binary_length;

	u32	extention_offset;	
	u32	reserved1;
} __attribute__ ((packed));

struct mms_ext_hdr {
	U32	data_ID;
	U32	offset;
	U32	length;
	U32	next_item;
	U8	data[0];
} __attribute__ ((packed));
/**
* Firmware image info
*/
struct mms_fw_img {
	u16	type;
	u16	version;

	u16	start_page;
	u16	end_page;

	u32	offset;
	u32	length;
} __attribute__ ((packed));

/**
* Firmware update error code
*/
enum fw_update_errno{
	fw_err_file_read = -4,
	fw_err_file_open = -3,
	fw_err_file_type = -2,
	fw_err_download = -1,
	fw_err_none = 0,
	fw_err_uptodate = 1,
};

/**
* Declarations
*/
//main
void mms_reboot(struct mms_ts_info *info);
void mms_reboot_stop_irq(struct mms_ts_info *info);
int mms_i2c_read_normal(struct mms_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len);
int mms_i2c_read_dma(struct mms_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len);
int mms_i2c_read(struct mms_ts_info *info, char *write_buf, unsigned int write_len, char *read_buf, unsigned int read_len);
//int mms_i2c_read_next(struct mms_ts_info *info, char *read_buf, int start_idx, unsigned int read_len);
int mms_i2c_write_normal(struct mms_ts_info *info, char *write_buf, unsigned int write_len);
int mms_i2c_write_dma(struct mms_ts_info *info, char *write_buf, unsigned int write_len);
int mms_i2c_write(struct mms_ts_info *info, char *write_buf, unsigned int write_len);
int mms_enable(struct mms_ts_info *info);
int mms_disable(struct mms_ts_info *info);
void mms_irq_enable(struct mms_ts_info *info);
void mms_irq_disable(struct mms_ts_info *info);
int mms_get_ready_status(struct mms_ts_info *info);
int mms_get_fw_version(struct mms_ts_info *info, u8 *ver_buf);
int mms_get_fw_version_u16(struct mms_ts_info *info, u16 *ver_buf_u16);
int mms_interrupt_start(struct mms_ts_info *info);
int mms_disable_esd_alert(struct mms_ts_info *info);
int mms_enable_gesture_wakeup_mode(struct mms_ts_info *info);
int mms_enable_nomal_active_mode(struct mms_ts_info *info);
int mms_fw_update_from_kernel(struct mms_ts_info *info);
int mms_fw_update_from_storage(struct mms_ts_info *info, bool force);
int mms_suspend(struct device *dev);
int mms_resume(struct device *dev);
int mms_resume_dummy(struct device *dev);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mms_early_suspend(struct early_suspend *h);
static void mms_late_resume(struct early_suspend *h);
static void mms_late_resume_dummy(struct early_suspend *h);
#endif
void mms_tpd_interrupt_handler(void);

//mod
void mms_irq_enable(struct mms_ts_info *info);
void mms_irq_disable(struct mms_ts_info *info);
int mms_regulator_control(struct i2c_client *client, int enable);
int mms_power_on(struct mms_ts_info *info);
int mms_power_off(struct mms_ts_info *info);
void mms_clear_input(struct mms_ts_info *info);
void mms_report_input_event(struct mms_ts_info *info, u8 sz, u8 *buf);
int mms_wakeup_event_handler(struct mms_ts_info *info, u8 *rbuf, unsigned int size); //shahn
void mms_input_event_handler(struct mms_ts_info *info, u8 sz, u8 *buf);
#if MMS_USE_DEVICETREE
int mms_parse_devicetree(struct device *dev, struct mms_ts_info *info);
#endif
int mms_config_platform_data(struct i2c_client *client, struct mms_ts_info *info);
void mms_config_gpio(struct mms_ts_info *info);
void mms_config_input(struct mms_ts_info *info);
#if MMS_USE_CALLBACK
void mms_config_callback(struct mms_ts_info *info);
#endif

//fw_update
int mms_flash_fw(struct mms_ts_info *info, const u8 *fw_data, size_t fw_size, bool force, bool section);

//test
#if MMS_USE_DEV_MODE
int mms_dev_create(struct mms_ts_info *info);
int mms_get_log(struct mms_ts_info *info);
#endif
int mms_run_test(struct mms_ts_info *info, u8 test_type);
int mms_get_image(struct mms_ts_info *info, u8 image_type);
#if MMS_USE_TEST_MODE
int mms_sysfs_create(struct mms_ts_info *info);
void mms_sysfs_remove(struct mms_ts_info *info);
static const struct attribute_group mms_test_attr_group;
#endif

//cmd
#if MMS_USE_CMD_MODE
int mms_sysfs_cmd_create(struct mms_ts_info *info);
void mms_sysfs_cmd_remove(struct mms_ts_info *info);
static const struct attribute_group mms_cmd_attr_group;
#endif

//hyeog added 
void mms_fw_update_controller(struct mms_ts_info *info,const struct firmware *fw, 
			struct i2c_client *client);
