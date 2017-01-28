#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/string.h>
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(320)
#define FRAME_HEIGHT 										(480)

#define REGFLAG_DELAY								0xFFFE
#define REGFLAG_END_OF_TABLE						0xFFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									1

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)        

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};


static struct LCM_setting_table lcm_initialization_setting[] = 
{
//YouTaiQi
#if 0
	{0x11,1 , {0x00}}, 
	{REGFLAG_DELAY, 120, {}}, 
	
	{0xf0,1 ,{0xc3}}, 
	{0xf0,1 ,{0x96}}, 
	{0x36,1 ,{0x48}}, //0x98
	{0x3a,1 ,{0x77}},
	{0xb9,1 ,{0x02}},

	{0xe8,8 ,{0x40,0x8a,0x00,0x00,0x29,0x19,0xa5,0x33}}, 
	{0xc2,1 ,{0xa7}}, 
	{0xc5,1 ,{0x2c}}, //3c
	//---------//
	{0xb1,2 , {0x80,0x10}}, 

	{0xe0,14 ,{0xf0,0x00,0x02,0x0a,0x0d,0x1d,0x35,0x55,0x45,0x3c,0x17,0x17,0x18,0x1b}}, 
	{0xe1,14 ,{0xf0,0x00,0x02,0x07,0x06,0x04,0x2e,0x44,0x45,0x0b,0x17,0x16,0x18,0x1b}}, 
	{0xf0,1 ,{0x3c}}, 
	{0xf0,1 ,{0x69}}, 

	{0x35,1 ,{0x00}},
	{REGFLAG_DELAY, 120, {}}, 

	{0x29,1 , {0x00}},
	{REGFLAG_DELAY, 50, {}},
	
#endif

	{0x11, 1, {0x00}}, 
	{REGFLAG_DELAY, 150, {}}, 
	
	{0xF0, 1, {0xC3}},
	{0xF0, 1, {0x96}},
	{0x36, 1, {0x48}}, 
	{0xB4, 1, {0x01}},
	{0x3A, 1, {0x77}},//0x55
	{0xB7, 1, {0x06}},
	{0xB9, 2, {0x02,0XC0}},
	{0x35, 1, {0x00}},
	{0x44, 2, {0x00,0x00}},
	
	{0xE8, 8, {0x40,0x8a,0x00,0x00,0x29,0x19,0xa5,0x33}},
	{0xC5, 1, {0x25}},
	{0xC2, 1, {0xA5}},  
	
	{0xE0, 14, {0xF0,0x00,0x03,0x0b,0x0c,0x29,0x2e,0x44,0x41,0x17,0x11,0x13,0x16,0x1b}},
	{0xE1, 14, {0xF0,0x00,0x02,0x06,0x06,0x24,0x2a,0x43,0x3e,0x2d,0x1a,0x16,0x13,0x17}},
	{0xEC, 3,  {0x00,0x00,0x01}},
	
	{0xF0, 1, {0x3C}},
	{0xF0, 1, {0x69}},

	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},

	// Setting ending by predefined flag
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_out_setting[] = 
{
    // Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 50, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = 
{
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    {REGFLAG_END_OF_TABLE, 0x00, {}}
};


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) 
	{
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			
            case REGFLAG_DELAY :
                MDELAY(table[i].count);
                break;
				
            case REGFLAG_END_OF_TABLE :
                break;
				
            default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
       	}
    }
	
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));

    params->type   = LCM_TYPE_DSI;
    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

    // enable tearing-free
    params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
    params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
    params->dsi.mode   = CMD_MODE;
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
#endif

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM				= LCM_ONE_LANE;//LCM_TWO_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	
	
	params->dsi.PLL_CLOCK = 140;//200; //this value must be in MTK suggested table

#if (LCM_DSI_CMD_MODE)
	
#else

	params->dsi.vertical_sync_active  = 3;//6;
	params->dsi.vertical_backporch    = 10;
	params->dsi.vertical_frontporch   = 10;
	params->dsi.vertical_active_line  = FRAME_HEIGHT;


	params->dsi.horizontal_sync_active = 10;
	params->dsi.horizontal_backporch  = 30;//120;
	params->dsi.horizontal_frontporch = 30;//100;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;
#endif

	params->dsi.clk_lp_per_line_enable = 0;
	params->dsi.esd_check_enable = 0;
	params->dsi.customization_esd_check_enable = 0;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
	params->dsi.lcm_esd_check_table[0].count        = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
}


static void lcm_init(void)
{
    SET_RESET_PIN(1);
	MDELAY(1);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);

	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);

}


static unsigned int lcm_compare_id(void);
static void lcm_resume(void)
{
//	lcm_compare_id();//test_id	
	
	lcm_init();
}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(&data_array, 7, 0);

}

#define LCM_ID_ST7796S	  0x7796
static unsigned int lcm_compare_id(void)
{
	unsigned int id = 0;
	unsigned char buffer[5];
	unsigned int array[8];
	
	SET_RESET_PIN(1);  //NOTE:should reset LCM firstly
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(100);

	array[0] = 0x00043700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	MDELAY(2);
	
	read_reg_v2(0xD3, buffer, 4);
	
	//id = ((buffer[2]<<8)|buffer[3]); //we only need ID
	id = ((buffer[1]<<8)|buffer[2]); //we only need ID

#if defined(BUILD_LK)
	printf("[ST7796S] %s, buffer[0] = %x;buffer[1] = %x;buffer[2] = %x;buffer[3] = %x;\n", __func__,buffer[0],buffer[1],buffer[2],buffer[3]);
	//printf("[ST7796S] %s, lcm id = %x\n",__func__, id);
#else
	printk("[ST7796S] %s, buffer[0] = %x;buffer[1] = %x;buffer[2] = %x;buffer[3] = %x;\n", __func__,buffer[0],buffer[1],buffer[2],buffer[3]);
#endif

	return (LCM_ID_ST7796S == id)?1:0;

}




LCM_DRIVER st7796_ytq_hvga_dsi_vdo_lcm_drv = 
{
    .name			= "st7796_ytq_hvga_dsi_vdo",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
	.compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
};
