/*
 * MELFAS MMS400 Touchscreen for MediaTek
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 *
 * Model Dependent Functions
 * 
 */

#include "melfas_mms400.h"
#include <linux/kpd.h>
#include <linux/ctype.h>

//added by pengwei 20150715 begin
#include <linux/regulator/consumer.h>
//added by pengwei 20150715 end

/**
* MTK
*/
extern struct tpd_device *tpd;
extern struct input_dev *kpd_input_dev;
#ifdef TPD_HAVE_BUTTON
extern int mms_tpd_keys_local[TPD_KEY_COUNT];
extern int mms_tpd_keys_dim_local[TPD_KEY_COUNT][4];
#endif

static u8 key_gesture;
static u32 key_gesture_control_mask=0x31323334;
/**
* Enable IRQ
*/
void mms_irq_enable(struct mms_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	/*
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
    mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
    mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	*/
	
	mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	info->irq_pause = false;
	
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
		
}

/**
* Disable IRQ
*/
void mms_irq_disable(struct mms_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
	
	info->irq_pause = true;
	
	mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);

	/*
    mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_GPIO);
    mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	*/
	
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	
}

/**
* Control regulator
*/
int mms_regulator_control(struct i2c_client *client, int enable)
{
#if 0
	int ret = 0;

	//////////////////////////
	// MODIFY REQUIRED
	//

	static struct regulator *reg_l22;

	dev_dbg(&client->dev, "%s [START]\n", __func__);

	if (!reg_l22) {
		reg_l22 = regulator_get(&client->dev, "vd33");
		//reg_l22 = regulator_get(NULL, "8941_l22");		
		if (IS_ERR(reg_l22)) {
			dev_err(&client->dev, "%s [ERROR] regulator_get\n", __func__);
			goto ERROR;
		}
		
		ret = regulator_set_voltage(reg_l22, 3300000, 3300000);
		if (ret) {
			dev_err(&client->dev, "%s [ERROR] regulator_set_voltage\n", __func__);
			goto ERROR;
		}
	}

	if (enable) {
		ret = regulator_enable(reg_l22);
		if (ret) {
			dev_err(&client->dev, "%s [ERROR] regulator_enable [%d]\n", __func__, ret);
			goto ERROR;
		}
	}
	else{
		if (regulator_is_enabled(reg_l22)){
			ret = regulator_disable(reg_l22);
			if (ret) {
				dev_err(&client->dev, "%s [ERROR] regulator_disable [%d]\n", __func__, ret);
				goto ERROR;
			}
		}
	}

	regulator_put(reg_l22);


	//
	//////////////////////////

	dev_dbg(&client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	dev_err(&client->dev, "%s [ERROR]\n", __func__);
	return -1;
#endif
	return 0;
}

//added by pengwei 20150715 begin
static struct regulator *RegMmsCtp = NULL;
static bool Mms_CTP_hwPowerOn(char * powerId, int powerVolt)
{
    bool ret = false;
	
	if(RegMmsCtp == NULL)
		RegMmsCtp = regulator_get(tpd->tpd_dev, powerId);
	
	//printk("[Mms_CTP_hwPowerOn]after get, powerId:%s, powerVolt:%d, RegMmsCtp: %p\n", powerId, powerVolt, RegMmsCtp);

 	if(!IS_ERR(RegMmsCtp))
	{
        if(regulator_set_voltage(RegMmsCtp , powerVolt,powerVolt)!=0 )
		{
            printk("[Mms_CTP_hwPowerOn]fail to regulator_set_voltage, powerVolt:%d, powerID: %s\n", powerVolt, powerId);
        }
        if(regulator_enable(RegMmsCtp)!= 0) 
		{
            printk("[Mms_CTP_hwPowerOn]fail to regulator_enable, powerVolt:%d, powerID: %s\n", powerVolt, powerId);
            return ret;
        }
        ret = true;
    }
	else
   		 printk("[Mms_CTP_hwPowerOn]IS_ERR_OR_NULL RegMmsCtp %s\n",powerId);

    return ret;
}

static bool Mms_CTP_hwPowerDown(char * powerId)
{
    bool ret = false;

	if(RegMmsCtp == NULL)
		RegMmsCtp = regulator_get(tpd->tpd_dev, powerId);
	
    if(!IS_ERR(RegMmsCtp))
	{
        if(regulator_is_enabled(RegMmsCtp)) 
		{
            printk("[Mms_CTP_hwPowerDown]before disable %s is enabled\n", powerId);
        }
		if(regulator_disable(RegMmsCtp)!=0)
			 printk("[Mms_CTP_hwPowerDown]fail to regulator_disable, powerID: %s\n", powerId);
		
        ret = true;
    } 
	else 
	{
        printk("[Mms_CTP_hwPowerDown]%s fail to power down  due to RegMmsCtp == NULL RegMmsCtp 0x%p\n", powerId,RegMmsCtp);
    }
	
    return ret;
}
//added by pengwei 20150715 end


/**
* Turn off power supply
*/
int mms_power_off(struct mms_ts_info *info)
{
	//int ret = 0;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
		
	//////////////////////////
	// MODIFY REQUIRED
	//
	
	//Control regulator
	//mms_regulator_control(info->client, 0);
	
	//Control power switch
	if(info->power_state == 1)
	{
		#ifdef TPD_POWER_SOURCE_CUSTOM
		Mms_CTP_hwPowerDown(TPD_POWER_SOURCE_CUSTOM);
		#endif

		info->power_state = 0;
	}

	msleep(5);
	printk("[HST] %s() RST set to 0....\n", __func__);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ZERO);
	//GTP_GPIO_OUTPUT(GPIO_CTP_RST_PIN, 0); 
	
	//
	//////////////////////////
	
	msleep(10);
	
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;
	
ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);	
	return -1;
}

/**
* Turn on power supply
*/
int mms_power_on(struct mms_ts_info *info)
{
	//int ret = 0;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);
	
	//////////////////////////
	// MODIFY REQUIRED
	//
	
	//Control regulator
	//mms_regulator_control(info->client, 1);
	
	//Control power switch
	if(info->power_state == 0)
	{
		#ifdef TPD_POWER_SOURCE_CUSTOM
		Mms_CTP_hwPowerOn(TPD_POWER_SOURCE_CUSTOM, 2800000);
		#endif
		
		info->power_state = 1;			//info->init = true;
	}

	msleep(5);
	printk("[HST] %s() RST set to 1....\n", __func__);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	//	GTP_GPIO_OUTPUT(GPIO_CTP_RST_PIN,1); 

	
	//
	//////////////////////////

	//msleep(200);
	mdelay(200);
	
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
	return 0;

ERROR:
	dev_err(&info->client->dev, "%s [ERROR]\n", __func__);	
	return -1;
}

/**
* Clear touch input events
*/
void mms_clear_input(struct mms_ts_info *info)
{
	int i;
	
	for (i = 0; i< MAX_FINGER_NUM; i++) {
		/////////////////////////////////
		// MODIFY REQUIRED
		//
		 info->touch_state_scr[i] = 0; //linfeng
		
		#if INPUT_REPORT_TYPE
		/*
		input_mt_slot(info->input_dev, i);
		input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
		*/
		input_mt_slot(tpd->dev, i);
		input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, false);
		#else
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_mt_sync(tpd->dev);
		#endif
		
		//
		/////////////////////////////////
	}
	
	input_sync(tpd->dev); //linfeng info->input_dev
	
	return;
}

/**
* Input event handler - Report touch input event
*/
static void tpd_down(s32 x, s32 y)
{
	input_report_abs(tpd->dev, ABS_MT_PRESSURE, 100);
   input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 100);

	input_report_key(tpd->dev, BTN_TOUCH, 1);
   input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
   input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
   input_mt_sync(tpd->dev);
   TPD_EM_PRINT(x, y, x, y, 0, 1);

}

static void tpd_up(s32 x, s32 y)
{
    
    input_report_key(tpd->dev, BTN_TOUCH, 0);
    input_mt_sync(tpd->dev);
    TPD_EM_PRINT(x, y, x, y, 0, 0);

} //linfeng

void mms_input_event_handler(struct mms_ts_info *info, u8 sz, u8 *buf)
{
	struct i2c_client *client = info->client;
	int i, i_finger;
	int release = 0;

	//dev_dbg(&client->dev, "%s [START]\n", __func__);
	//dev_dbg(&client->dev, "%s - sz[%d] buf[0x%02X]\n", __func__, sz, buf[0]);
	printk("[MMS]%s [START]\n", __func__);
	printk("[MMS]%s - sz[%d] buf[0x%02X]\n", __func__, sz, buf[0]);
	

	//for (i = 1; i < sz; i += info->event_size) {
	for (i = 0; i < sz; i += info->event_size) {
		u8 *tmp = &buf[i];

		int id = (tmp[0] & 0xf) - 1;
		int x = tmp[2] | ((tmp[1] & 0xf) << 8); //linfeng
		int y = tmp[3] | (((tmp[1] >> 4) & 0xf) << 8);//linfeng 8,6
		int touch_major = tmp[4];
		int pressure = tmp[5];
						
		// Report input data
		if ((tmp[0] & MIP_EVENT_INPUT_SCREEN) == 0) {
			//Touchkey Event
			int key = tmp[0] & 0xf;
			int key_state = (tmp[0] & MIP_EVENT_INPUT_PRESS) ? 1 : 0;
			int key_code = 0;
			
			/////////////////////////////////
			// MODIFY REQUIRED
			//

			//Report touchkey event
			
			/*switch (key) {
				case 1:
					key_code = KEY_MENU;
					//dev_dbg(&client->dev, "Key : KEY_MENU\n");
					break;
				case 2:
					key_code = KEY_BACK;
					//dev_dbg(&client->dev, "Key : KEY_BACK\n");
					break;
				case 3:
					key_code = KEY_HOMEPAGE;
					//dev_dbg(&client->dev, "Key : KEY_MENU\n");
					break;
				default:
					dev_err(&client->dev, "%s [ERROR] Unknown key code [%d]\n", __func__, key);
					continue;
					break;
			}*/
		
			key_code = mms_tpd_keys_local[key - 1];	// key = 1~

			if(key_code==139)
			{
				if(key_state)
					tpd_down(key_1);
				else
					tpd_up(key_1);
			}
			if(key_code==172)
			{
				if(key_state)
					tpd_down(key_2);
				else
					tpd_up(key_2);
			}
			if(key_code==158)
			{
				if(key_state)
					tpd_down(key_3);
				else
					tpd_up(key_3);
			} //linfeng
			//input_report_key(info->input_dev, key_code, key_state);
			//input_report_key(tpd->dev, key_code, key_state);
			
			//dev_dbg(&client->dev, "%s - Key : ID[%d] Code[%d] State[%d]\n", __func__, key, key_code, key_state);
			printk("[MMS]%s - Key : ID[%d] Code[%d] State[%d]\n", __func__, key, key_code, key_state);
			
			//
			/////////////////////////////////			
		}
		else
		{
			//Touchscreen Event
			
			/////////////////////////////////
			// MODIFY REQUIRED
			//

			//Report touchscreen event
			if((tmp[0] & MIP_EVENT_INPUT_PRESS) == 0) {
				//Release
				info->touch_state_scr[id] = 0;
				printk("%s - Touch : ID[%d] Release\n", __func__, id);
				#if INPUT_REPORT_TYPE
				/*
				input_mt_slot(info->input_dev, id);
				input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, false);
				*/
				input_mt_slot(tpd->dev, id);
				input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, 0);
				
				#else
				release = 0;
				for(i_finger = 0; i_finger < MAX_FINGER_NUM; i_finger++){
					if(info->touch_state_scr[i_finger] != 0){
						release = 1;
					}
				}
				if(release == 0){
					input_report_key(tpd->dev, BTN_TOUCH, 0);
					input_mt_sync(tpd->dev);
					//dev_dbg(&client->dev, "%s - Touch : Release\n", __func__);
					printk("[MMS]%s - Touch : Release\n", __func__);
				}
				#endif
				//input_sync(tpd->dev);
				//input_sync(info->input_dev); //linfeng
				continue;
			}			
			
			//Press or Move
			#if INPUT_REPORT_TYPE
			/*
			input_mt_slot(info->input_dev, id);
			input_mt_report_slot_state(info->input_dev, MT_TOOL_FINGER, true);						
			input_report_abs(info->input_dev, ABS_MT_POSITION_X, x);
			input_report_abs(info->input_dev, ABS_MT_POSITION_Y, y);
			input_report_abs(info->input_dev, ABS_MT_PRESSURE, pressure);
			input_report_abs(info->input_dev, ABS_MT_TOUCH_MAJOR, touch_major);
			*/
			input_report_key(tpd->dev, BTN_TOUCH, 1);     //modify @ sean
			input_mt_slot(tpd->dev, id);
			input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER, 1);						
			input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
			input_report_abs(tpd->dev, ABS_MT_PRESSURE, pressure);
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, touch_major);
			#else
			input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
			input_report_key(tpd->dev, BTN_TOUCH, 1);
			input_report_abs(tpd->dev, ABS_MT_PRESSURE, pressure);
			input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, touch_major);
			input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
			input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
			input_mt_sync(tpd->dev);
			#endif
			
			info->touch_state_scr[id] = 1;
			
			printk("%s - Touch : ID[%d] X[%d] Y[%d] P[%d] M[%d] \n", __func__, id, x, y, pressure, touch_major);
			
			//
			/////////////////////////////////
		}
	
	}
	printk("[MMS] input_sync  xxxxxxxxxxxxxxxxxxxxxxxxxxxxx tpd->dev->name:%s",tpd->dev->name);
	//input_sync(info->input_dev);
	input_sync(tpd->dev); //linfeng
	
//EXIT:
	printk("%s [DONE]\n", __func__);
	return;
}

/**
* Wake-up event handler
*/
static unsigned char read_back_coordinate[28]={0};
int mms_wakeup_event_handler(struct mms_ts_info *info, u8 *rbuf, unsigned int size) //shahn
{
	u8 gesture_type = rbuf[1];
	//u8 *positionBuf_test = rbuf[2];
	u8 index = 0, tCoodinateSize = 0;
	int x[7] = {0, }; 
	int y[7] = {0, }; 
	int i =0,j=0;
    u8 positionBuf[32] ={0,};
	//step1 :save the coordinate to array:read_back_coordinate[28]  
	memcpy(positionBuf,&rbuf[2],28);
	
	if(gesture_type >= 1 && gesture_type < 24) //shahn
	{
		tCoodinateSize = (size-2)/3; 
		printk("[HST] size=%d tCoodinateSize= %d,\n",size,tCoodinateSize);
		//1.get the cooordinate.notice: there are 3byte for one coordinate (x,y) from fw to driver.
		for(index = 0; index < tCoodinateSize; index++)
		{
			x[index]= positionBuf[(index*3)+1] | ((positionBuf[(index*3)] & 0xf) << 8); 
			y[index]= positionBuf[(index*3)+2] | (((positionBuf[(index*3)] >> 4) & 0xf) << 8);
			printk("[HST]index=%d x=%d,y=%d \n",index,x[index],y[index]); 
		}
		//2.fetch out the coordinate data for app
		for(i = 0; i< tCoodinateSize*4; i+=4)
		{
			read_back_coordinate[i] = (x[i/4] & 0xff00)>>8;//x high byte
			read_back_coordinate[i+1] = (x[i/4] & 0x00ff);//x low byte
				
			read_back_coordinate[i+2] = (y[i/4] & 0xff00)>>8;//y high byte
			read_back_coordinate[i+3] = (y[i/4] & 0x00ff);//y low byte
		}	

		//only for test			
		for(j=0;j<28;j++)
			printk("[HST] read_back_coo[%d] =%d\n",j,read_back_coordinate[j]);
		
	}
	
	/////////////////////////////////
	// MODIFY REQUIRED
	//

	//Report wake-up event
	
	//
	//
	/////////////////////////////////
	   /* switch (gesture_type)
		{
		case 1:
			key_gesture=0xC1;
			printk("[HST] C\n");
			input_report_key(tpd->dev, KEY_GESTURE_C, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_C, 0);
			input_sync(tpd->dev);
			break;
		case 2:
			key_gesture=0xC2;
			printk("[HST] W\n");
			input_report_key(tpd->dev, KEY_GESTURE_W, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_W, 0);
			input_sync(tpd->dev);
			break;
		case 3:
			key_gesture=0xC6;
			printk("[HST] V\n");
			input_report_key(tpd->dev, KEY_GESTURE_V, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_V, 0);
			input_sync(tpd->dev);
			break;
		case 4:
			key_gesture=0xC3;
			printk("[HST] M\n");
			input_report_key(tpd->dev, KEY_GESTURE_M, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_M, 0);
			input_sync(tpd->dev);
			break;
		case 5:
			key_gesture=0xC5;
			printk("[HST] S\n");
			input_report_key(tpd->dev, KEY_GESTURE_S, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_S, 0);
			input_sync(tpd->dev);
			break;
		case 6:
			key_gesture=0xCA;
			printk("[HST] Z\n");
			input_report_key(tpd->dev, KEY_GESTURE_Z, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_Z, 0);
			input_sync(tpd->dev);
			break;
		case 7:
			key_gesture=0xC4;
			printk("[HST] O\n");
			input_report_key(tpd->dev, KEY_GESTURE_O, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_O, 0);
			input_sync(tpd->dev);
			break;
		case 8:
			key_gesture=0xC0;
			printk("[HST] E\n");
			input_report_key(tpd->dev, KEY_GESTURE_E, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_E, 0);
			input_sync(tpd->dev);
			break;
		case 9:
			key_gesture= 0xC9;
			printk("[HST] V 90\n");
			input_report_key(tpd->dev, KEY_GESTURE_V, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_V, 0);
			input_sync(tpd->dev);
			break;

		case 10:
			key_gesture= 0xC7;
			printk("[HST] V 180\n");
			input_report_key(tpd->dev, KEY_GESTURE_V, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_V, 0);
			input_sync(tpd->dev);
			break;					 
		case 20:
			key_gesture=0xB1;
			printk("[HST] right\n");
			input_report_key(tpd->dev, KEY_GESTURE_RIGHT, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_RIGHT, 0);
			input_sync(tpd->dev);
			break;
		case 21:
			key_gesture=0xB3;
			printk("[HST] down\n");
			input_report_key(tpd->dev, KEY_GESTURE_DOWN, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_DOWN, 0);
			input_sync(tpd->dev);
			break;
		case 22:
			key_gesture=0xB0;
			printk("[HST] left\n");
			input_report_key(tpd->dev, KEY_GESTURE_LEFT, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_LEFT, 0);
			input_sync(tpd->dev);
			break;
		case 23:
			key_gesture=0xB2;
			printk("[HST] up\n");
			input_report_key(tpd->dev, KEY_GESTURE_UP, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_UP, 0);
			input_sync(tpd->dev);	
			break;
		case 24:
			key_gesture=0xA0;
			printk("[HST] double tap\n");
			input_report_key(tpd->dev, KEY_GESTURE_U, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_U, 0);
			input_sync(tpd->dev);
			break;

		default:
			key_gesture=0;
			break;
           }*/

		switch (gesture_type)
           {
	           case 1:
	           case 2:
	           case 3:
	           case 4:
	           case 5:
	           case 6:
	           case 7:
			   case 8:
	           case 9:
	           case 10:			   	
	           case 20:
	           case 21:
	           case 22:
	           case 23:
	           case 24:
				input_report_key(kpd_input_dev, KEY_POWER, 1);
				input_sync(kpd_input_dev);
				input_report_key(kpd_input_dev, KEY_POWER, 0);
				input_sync(kpd_input_dev);
				break;
						 
		    default: 
				break;
           }

	
	printk("[HST] %s [DONE] key_gesture=%x\n", __func__, key_gesture);
	return 0;
	
}
/*
desc:add by jet inhuaqin 
         for create the sysfs nodes for application:
	1)/sys/gesture/debug  
	     :used for app read back the track coordinate
	2)/sys/gesture/gesture_enable 
	     :used for app to switch on/off the whole gesture funtion
	     
date:0309
*/
#if defined(MMS_USE_GESTURE_WAKEUP_MODE) 
#include <linux/kobject.h>
#include <linux/sysfs.h>

extern unsigned int mms_wakeup_control_flag;
static unsigned int mms_proc_operate_mode = 0;
/////just for read point
#define MMS_PROC_READ_DRAWDATA			10
static unsigned char gesture_point_readbuf[30]={0};




/*---------------------------------------------------------------------------*/
struct mms_gesture_entry {
    struct attribute attr;
    ssize_t (*show)(struct kobject *kobj, char *page);
    ssize_t (*store)(struct kobject *kobj, const char *page, size_t size);
};
struct mms_gesture_sysobj {
    struct kobject kobj;
    atomic_t enable;
} gesture_sysobj = {
    .enable = ATOMIC_INIT(0),
};

/*define sysfs entry */
/*---------------------------------------------------------------------------*/
static ssize_t mms_gesture_attr_show(struct kobject *kobj, struct attribute *attr, char *buffer) 
{
    struct mms_gesture_entry *entry = container_of(attr, struct mms_gesture_entry, attr);
    return entry->show(kobj, buffer);
}
/*---------------------------------------------------------------------------*/
static ssize_t mms_gesture_attr_store(struct kobject *kobj, struct attribute *attr, const char *buffer, size_t size) 
{
    struct mms_gesture_entry *entry = container_of(attr, struct mms_gesture_entry, attr);
    return entry->store(kobj, buffer, size);
}
/*---------------------------------------------------------------------------*/
static ssize_t mms_gesture_enable_show(struct kobject *kobj, char *buffer) 
{
	/*int buf_len=0;
	 buf_len += sprintf(buffer, "%d",tpd->gesture_enable);
    return buf_len;*/
}
/*---------------------------------------------------------------------------*/
static ssize_t mms_gesture_enable_store(struct kobject *kobj, const char *buffer, size_t size) 
{
    /*int res = sscanf(buffer, "%d", &tpd->gesture_enable);

    if (res != 1) {
        printk("mms_gesture_enable_store input string :'%s' \n",buffer);
    } else {
		printk("[mms_gesture_enable_store] tpd->gesture_enable %d\n", tpd->gesture_enable);
		//struct alsps_hw *hw = get_cust_alsps_hw();
		//if(hw->enable_ps)
		//		hw->enable_ps(tpd->gesture_enable);
		
		if(tpd->gesture_enable == 1)
			mms_wakeup_control_flag = 1;
		else if(tpd->gesture_enable == 0)
			mms_wakeup_control_flag = 0;
    }*/
     mms_wakeup_control_flag = 0;
    return size;
}
/*---------------------------------------------------------------------------*/
/*
read back coordinate 
*/
static ssize_t mms_gesture_debug_show(struct kobject *kobj, char *buffer) 
{
    ssize_t len = 0;
	int num_read=0;
	int i=0;
	
	/*printk("mms_gesture_debug_show start %d\n", mms_proc_operate_mode);
	switch (mms_proc_operate_mode) 
	{
		case MMS_PROC_READ_DRAWDATA:
			read_back_coordinate[0] = key_gesture;
			for(i = 0;i < 28;i++)
			{
				gesture_point_readbuf[1 + i] = read_back_coordinate[i];
				//len += sprintf(buffer+len,"%u",gesture_point_readbuf[i]);//for cat 
			}
			len  =  num_read= 29;
			memcpy(buffer,gesture_point_readbuf,num_read);
			break;
		default:
			break;
	}
	printk("mms_gesture_debug_show--end %d\n", num_read);*/
	
    return len;
}
/*---------------------------------------------------------------------------*/
static ssize_t mms_gesture_debug_store(struct kobject *kobj, const char *buffer, size_t size) 
{
	//Unformat a buffer into a list of arguments(int ,char ...)
	/*if(sscanf(buffer, "%d", &mms_proc_operate_mode) == 1)
	{
		printk("mms_gesture_debug_store success %d size=%ld\n",mms_proc_operate_mode,size);
		printk("mms_gesture_debug_store input string :'%s' \n",buffer);
	}
	else
	{	
		printk("mms_gesture_debug_store error format :'%s'\n",buffer);
		mms_proc_operate_mode=0xff;
		return -EFAULT;
	}	
	*/

    return size;
}
static void mms_gesture_sysfs_release(struct kobject *kobj)
{
    struct mms_gesture_sysobj       *ge_sysfs;
	
	ge_sysfs=container_of(kobj, struct mms_gesture_sysobj, kobj);   

    kfree(ge_sysfs);
}

/*---------------------------------------------------------------------------*/
struct sysfs_ops mms_gesture_sysfs_ops = {
    .show   = mms_gesture_attr_show,
    .store  = mms_gesture_attr_store,
};
/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/
struct mms_gesture_entry gesture_enable_entry = {
    { .name = "gesture_enable", .mode = 0664 },
    mms_gesture_enable_show,
    mms_gesture_enable_store,
};
/*---------------------------------------------------------------------------*/
struct mms_gesture_entry gesture_debug_entry = {
    { .name = "debug", .mode = 0664},
    mms_gesture_debug_show,
    mms_gesture_debug_store,
};
/*---------------------------------------------------------------------------*/
struct attribute *mms_gesture_attributes[] = {
    &gesture_enable_entry.attr,  /*enable setting*/     
    &gesture_debug_entry.attr,   
    NULL,
};
/*---------------------------------------------------------------------------*/
struct kobj_type mms_gesture_ktype = {
    .sysfs_ops = &mms_gesture_sysfs_ops,
	.release =mms_gesture_sysfs_release,
    .default_attrs = mms_gesture_attributes,
};
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
int mms_gesture_sysfs(void) 
{
    struct mms_gesture_sysobj *obj = &gesture_sysobj;
    int idx;

    memset(&obj->kobj, 0x00, sizeof(obj->kobj));

    atomic_set(&obj->enable, 0);
    
    obj->kobj.parent = kernel_kobj;
    if (kobject_init_and_add(&obj->kobj, &mms_gesture_ktype, NULL, "gesture")) {
        kobject_put(&obj->kobj);
		printk("error mms_gesture_sysfs ");
        return -ENOMEM;
    }
    kobject_uevent(&obj->kobj, KOBJ_ADD);

    return 0;
}
/*---------------------------------------------------------------------------*/

#endif 

static ssize_t store_melfas_gesture_control(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t size)
{	
	char *pvalue = NULL;
	unsigned int gesture_control = 0;
	size_t count = 0;
	
	gesture_control = simple_strtoul(buf, &pvalue, 10); 
	count = pvalue - buf;
	if (*pvalue && isspace(*pvalue))
		count++;

	if (count == size) 
	{
		printk("[HST] %s() gesture_control=%08x\n", __func__, gesture_control);
	}
	return size;
}

static ssize_t show_melfas_gesture_control(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", key_gesture_control_mask);
}

static DEVICE_ATTR(gesture_control, 0666, show_melfas_gesture_control, store_melfas_gesture_control);

static ssize_t store_melfas_gesture_data(struct device *dev, struct device_attribute *attr,
				  const char *buf, size_t size)
{
	return 0;
}

static ssize_t show_melfas_gesture_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", key_gesture);
}

static DEVICE_ATTR(gesture_data, 0666, show_melfas_gesture_data, store_melfas_gesture_data);

/**
* Sysfs attr info
*/
static struct attribute *gesture_attrs[] = {
	&dev_attr_gesture_control.attr,
	&dev_attr_gesture_data.attr,
	NULL,
};

/**
* Sysfs attr group info
*/
static const struct attribute_group gesture_group = {
	.attrs = gesture_attrs,
};

extern struct kset *devices_kset;
int mx_tsp_device_create(void)
{
	int ret=0;
	static struct kobject *mx_tsp = NULL;

	if (!mx_tsp)
		mx_tsp = kobject_create_and_add("mx-tsp",
						     &devices_kset->kobj);

	if (sysfs_create_group(mx_tsp, &gesture_group)) {
		printk("[HST] %s() [ERROR] sysfs_create_group\n", __func__);
		ret = -EAGAIN;
	}


	return ret;
}


#if MMS_USE_DEVICETREE
/**
* Parse device tree
*/
int mms_parse_devicetree(struct device *dev, struct mms_ts_info *info)
{
	//struct i2c_client *client = to_i2c_client(dev);
	//struct mms_ts_info *info = i2c_get_clientdata(client);
	struct device_node *np = dev->of_node;
	int ret;
	u32 val;
	
	dev_dbg(dev, "%s [START]\n", __func__);
	
	/////////////////////////////////
	// MODIFY REQUIRED
	//
	
	//Read property
	ret = of_property_read_u32(np, MMS_DEVICE_NAME",max_x", &val);
	if (ret) {
		dev_err(dev, "%s [ERROR] max_x\n", __func__);
		info->pdata->max_x = 1080;
	} 
	else {
		info->pdata->max_x = val;
	}

	ret = of_property_read_u32(np, MMS_DEVICE_NAME",max_y", &val);
	if (ret) {
		dev_err(dev, "%s [ERROR] max_y\n", __func__);
		info->pdata->max_y = 1920;
	}
	else {
		info->pdata->max_y = val;
	}
	
	//Get GPIO 
	ret = of_get_named_gpio(np, MMS_DEVICE_NAME",irq-gpio", 0);
	if (!gpio_is_valid(ret)) {
		dev_err(dev, "%s [ERROR] of_get_named_gpio : irq-gpio\n", __func__);
		goto ERROR;
	}
	else{
		info->pdata->gpio_intr = ret;
	}

	/*
	ret = of_get_named_gpio(np, MMS_DEVICE_NAME",reset-gpio", 0);
	if (!gpio_is_valid(ret)) {
		dev_err(dev, "%s [ERROR] of_get_named_gpio : reset-gpio\n", __func__);
		goto ERROR;
	}
	else{
		info->pdata->gpio_reset = ret;
	}
	*/
	
	//Config GPIO
	ret = gpio_request(info->pdata->gpio_intr, "irq-gpio");
	if (ret < 0) {
		dev_err(dev, "%s [ERROR] gpio_request : irq-gpio\n", __func__);
		goto ERROR;
	}	
	gpio_direction_input(info->pdata->gpio_intr);	

	ret = gpio_tlmm_config(GPIO_CFG(info->pdata->gpio_intr, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	if (ret < 0){
		dev_err(&info->client->dev, "%s [ERROR] gpio_tlmm_config gpio_intr\n", __func__);
		goto ERROR;
	}

	/*
	ret = gpio_request(info->pdata->gpio_reset, "reset-gpio");
	if (ret < 0) {
		dev_err(dev, "%s [ERROR] gpio_request : reset-gpio\n", __func__);
		goto ERROR;
	}		
	gpio_direction_output(info->pdata->gpio_reset, 1);
	*/
	
	//Set IRQ
	info->client->irq = gpio_to_irq(info->pdata->gpio_intr); 
	//dev_dbg(dev, "%s - gpio_to_irq : irq[%d]\n", __func__, info->client->irq);
		
	//
	/////////////////////////////////
	
	dev_dbg(dev, "%s [DONE]\n", __func__);	
	return 0;

ERROR:
	dev_err(dev, "%s [ERROR]\n", __func__);	
	return 1;
}
#endif

/**
* Config platform data
*/
int mms_config_platform_data(struct i2c_client *client, struct mms_ts_info *info)
{
	dev_dbg(&client->dev, "%s [START]\n", __func__);

	info->pdata = devm_kzalloc(&client->dev, sizeof(struct mms_platform_data), GFP_KERNEL);
	if (!info->pdata) {
		dev_err(&client->dev, "%s [ERROR] pdata devm_kzalloc\n", __func__);
		goto ERROR;
	}
	
	//Get Resolution
	info->pdata->max_x = TPD_RES_X;
	info->pdata->max_y = TPD_RES_Y;
	
	//Config GPIO
	info->pdata->gpio_reset = GPIO_CTP_RST_PIN;
	info->pdata->gpio_intr = GPIO_CTP_EINT_PIN;
	
	dev_dbg(&client->dev, "%s [DONE]\n", __func__);	
	return 0;

ERROR:
	dev_err(&client->dev, "%s [ERROR]\n", __func__); 
	return 1;
}

/**
* Config GPIO
*/
void mms_config_gpio(struct mms_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	//Config GPIO : Reset (Pin Name : CE)
	mt_set_gpio_mode(GPIO_CTP_RST_PIN, GPIO_CTP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CTP_RST_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_CTP_RST_PIN, GPIO_OUT_ONE);
	
	//Config GPIO : Interrupt (Pin Name : RESETB)
	mt_set_gpio_mode(GPIO_CTP_EINT_PIN, GPIO_CTP_EINT_PIN_M_EINT);
	mt_set_gpio_dir(GPIO_CTP_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(GPIO_CTP_EINT_PIN, GPIO_PULL_UP);
	//mt_set_gpio_pull_enable(GPIO_CTP_EINT_PIN, GPIO_PULL_DISABLE);

	mt_eint_set_hw_debounce(CUST_EINT_TOUCH_PANEL_NUM, CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_TOUCH_PANEL_NUM, EINTF_TRIGGER_FALLING, mms_tpd_interrupt_handler, 1);
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);

	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);	
	return;
}


/**
* Config input interface	
*/
void mms_config_input(struct mms_ts_info *info)
{
//	struct input_dev *input_dev = info->input_dev;

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);	
#if INPUT_REPORT_TYPE

	/////////////////////////////
	// MODIFY REQUIRED
	//

	//Screen
	/*
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);

	set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	
	input_mt_init_slots(input_dev, MAX_FINGER_NUM,0x02);

	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, info->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, info->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, INPUT_PRESSURE_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, INPUT_TOUCH_MAJOR_MAX, 0, 0);
	*/
	set_bit(BTN_TOUCH, tpd->dev->keybit);     //modify  @sean
	set_bit(EV_SYN, tpd->dev->evbit);
	set_bit(EV_ABS, tpd->dev->evbit);
	
	set_bit(INPUT_PROP_DIRECT, tpd->dev->propbit);
	
	input_mt_init_slots(tpd->dev, MAX_FINGER_NUM,0x02);
	
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, info->max_x, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, info->max_y, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, INPUT_PRESSURE_MAX, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR, 0, INPUT_TOUCH_MAJOR_MAX, 0, 0);
	
	//Key
	/*
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);

#if MMS_USE_NAP_MODE
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_POWER, input_dev->keybit);
#endif	
	*/
	set_bit(EV_KEY, tpd->dev->evbit);
	set_bit(KEY_BACK, tpd->dev->keybit);
	set_bit(KEY_MENU, tpd->dev->keybit);

	//
	/////////////////////////////
#else
//hyeog

	input_set_abs_params(tpd->dev, ABS_MT_TRACKING_ID, 0, 5, 0, 0);
	
#endif
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);	
	return;
}

#if MMS_USE_CALLBACK
/**
* Callback - get charger status
*/
void mms_callback_charger(struct mms_callbacks *cb, int charger_status)
{
	struct mms_ts_info *info = container_of(cb, struct mms_ts_info, callbacks);

	dev_dbg(&info->client->dev, "%s [START]\n", __func__);

	dev_info(&info->client->dev, "%s - charger_status[%d]\n", __func__, charger_status);
	
	//...
	
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);
}

/**
* Callback - add callback funtions here
*/
//...

/**
* Config callback functions
*/
void mms_config_callback(struct mms_ts_info *info)
{
	dev_dbg(&info->client->dev, "%s [START]\n", __func__);	

	info->register_callback = info->pdata->register_callback;

	//callback functions
	info->callbacks.inform_charger = mms_callback_charger;
	//info->callbacks.inform_display = mms_callback_display;
	//...
	
	if (info->register_callback){
		info->register_callback(&info->callbacks);
	}
	
	dev_dbg(&info->client->dev, "%s [DONE]\n", __func__);	
	return;
}
#endif

