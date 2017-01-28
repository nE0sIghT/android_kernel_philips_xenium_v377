/*
 * MELFAS MMS400 Touchscreen for MediaTek
 *
 * Copyright (C) 2014 MELFAS Inc.
 *
 */

#ifndef __MMS400_TOUCHPANEL_H__
#define __MMS400_TOUCHPANEL_H__

#include <mach/mt_pm_ldo.h>

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_POWER_SOURCE_CUSTOM		MT65XX_POWER_NONE     
#define TPD_I2C_BUS		1
#define TPD_I2C_ADDR		0x48
#define TPD_WAKEUP_TRIAL	60
#define TPD_WAKEUP_DELAY	100

//#define TPD_HAVE_TREMBLE_ELIMINATION
//#define TPD_HAVE_CALIBRATION

//#define TPD_LDO MT6323_POWER_LDO_VGP1


//Screen
//#define TPD_RES_X	1080
//#define TPD_RES_Y	1920
#define TPD_RES_X	480
#define TPD_RES_Y	854
#define LCD_X TPD_RES_X //480
#define LCD_Y TPD_RES_Y //854

//#define	TPD_XY_INVERT
//#define	TPD_X_INVERT
//#define	TPD_Y_INVERT

//Key
/* Define the virtual button mapping */
#define TPD_HAVE_BUTTON
#define TPD_BUTTON_HEIGH	(100)
#define TPD_KEY_COUNT		3
#define key_1           100,1340              //auto define  
#define key_2           280,1340
#define key_3           460,1340 //linfeng
#define TPD_KEYS			{KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#define TPD_KEYS_DIM    {{key_1,50,100},{key_2,50,100},{key_3,50,100}} //linfeng


/* Define the touch dimension */
#ifdef TPD_HAVE_BUTTON
#define TPD_TOUCH_HEIGH_RATIO	80
#define TPD_DISPLAY_HEIGH_RATIO	73
#endif

/* Define the 0D button mapping */
#ifdef TPD_HAVE_BUTTON
#define TPD_0D_BUTTON {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#else
#define TPD_0D_BUTTON {KEY_MENU, KEY_HOMEPAGE, KEY_BACK}
#endif

#endif /* TOUCHPANEL_H__ */

