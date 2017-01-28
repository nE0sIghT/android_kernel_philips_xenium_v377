#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/string.h>
#endif

#include "lcm_drv.h"

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)

#define REGFLAG_DELAY             							0xFFFE
#define REGFLAG_END_OF_TABLE      							0xFFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define ILI9881C_LCM_ID				0x9881

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
#if 0
	//DEZHIXIN Y85239 LCM
	{0xFF,3,{0x98,0x81,0x03}},//0XFF   4
	{0x01,1,{0x00}},//0X01	 2
	{0x02,1,{0x00}},//0X02	 2
	{0x03,1,{0x53}},//0X03	 2
	{0x04,1,{0x13}},//0X04	 2
	{0x05,1,{0x13}},//0X05	 2
	{0x06,1,{0x06}},//0X06	 2
	{0x07,1,{0x00}},//0X07	 2
	{0x08,1,{0x04}},//0X08	 2
	{0x09,1,{0x00}},//0X09	 2
	{0x0a,1,{0x00}},//0X0A	 2
	{0x0b,1,{0x00}},//0X0B	 2
	{0x0c,1,{0x00}},//0X0C	 2
	{0x0d,1,{0x00}},//0X0D	 2
	{0x0e,1,{0x00}},//0X0E	 2
	{0x0f,1,{0x00}},//0X0F	 2
	{0x10,1,{0x00}},//0X10	 2
	{0x11,1,{0x00}},//0X11	 2
	{0x12,1,{0x00}},//0X12	 2
	{0x13,1,{0x00}},//0X13	 2
	{0x14,1,{0x00}},//0X14	 2
	{0x15,1,{0x00}},//0X15	 2
	{0x16,1,{0x00}},//0X16	 2
	{0x17,1,{0x00}},//0X17	 2
	{0x18,1,{0x00}},//0X18	 2
	{0x19,1,{0x00}},//0X19	 2
	{0x1a,1,{0x00}},//0X1A	 2
	{0x1B,1,{0x00}},//0X1B	 2
	{0x1C,1,{0x00}},//0X1C	 2
	{0x1D,1,{0x00}},//0X1D	 2
	{0x1E,1,{0xC0}},//0X1E	 2
	{0x1F,1,{0x80}},//0X1F	 2
	{0x20,1,{0x04}},//0X20	 2
	{0x21,1,{0x0B}},//0X21	 2
	{0x22,1,{0x00}},//0X22	 2
	{0x23,1,{0x00}},//0X23	 2
	{0x24,1,{0x00}},//0X24	 2
	{0x25,1,{0x00}},//0X25	 2
	{0x26,1,{0x00}},//0X26	 2
	{0x27,1,{0x00}},//0X27	 2
	{0x28,1,{0x55}},
	{0x29,1,{0x03}},
	{0x2a,1,{0x00}},
	{0x2b,1,{0x00}},
	{0x2c,1,{0x00}},
	{0x2d,1,{0x00}},//0X   2
	{0x2e,1,{0x00}},
	{0x2f,1,{0x00}},
	{0x30,1,{0x00}},
	{0x31,1,{0x00}},
	{0x32,1,{0x00}},
	{0x33,1,{0x00}},
	{0x34,1,{0x04}},
	{0x35,1,{0x05}},//0X   2
	{0x36,1,{0x05}},
	{0x37,1,{0x00}},
	{0x38,1,{0x3C}},
	{0x39,1,{0x00}},
	{0x3a,1,{0x40}},
	{0x3b,1,{0x40}},//0X   2
	{0x3c,1,{0x00}},
	{0x3d,1,{0x00}},
	{0x3e,1,{0x00}},
	{0x3f,1,{0x00}},
	{0x40,1,{0x00}},
	{0x41,1,{0x00}},
	{0x42,1,{0x00}},//0X   2
	{0x43,1,{0x00}},
	{0x44,1,{0x00}},
	{0x50,1,{0x01}},
	{0x51,1,{0x23}},
	{0x52,1,{0x45}},
	{0x53,1,{0x67}},
	{0x54,1,{0x89}},
	{0x55,1,{0xAB}},
	{0x56,1,{0x01}},
	{0x57,1,{0x23}},
	{0x58,1,{0x45}},
	{0x59,1,{0x67}},
	{0x5a,1,{0x89}},
	{0x5b,1,{0xAB}},
	{0x5c,1,{0xCD}},
	{0x5d,1,{0xEF}},//0X   2
	{0x5e,1,{0x01}},
	{0x5f,1,{0x14}},
	{0x60,1,{0x15}},
	{0x61,1,{0x0C}},
	{0x62,1,{0x0D}},
	{0x63,1,{0x0E}},//0X   2
	{0x64,1,{0x0F}},
	{0x65,1,{0x10}},
	{0x66,1,{0x11}},
	{0x67,1,{0x08}},
	{0x68,1,{0x02}},
	{0x69,1,{0x0A}},
	{0x6a,1,{0x02}},
	{0x6b,1,{0x02}},//0X   2
	{0x6c,1,{0x02}},
	{0x6d,1,{0x02}},
	{0x6e,1,{0x02}},
	{0x6f,1,{0x02}},
	{0x70,1,{0x02}},//0X   2
	{0x71,1,{0x02}},
	{0x72,1,{0x06}},
	{0x73,1,{0x02}},
	{0x74,1,{0x02}},
	{0x75,1,{0x14}},
	{0x76,1,{0x15}},
	{0x77,1,{0x11}},
	{0x78,1,{0x10}},
	{0x79,1,{0x0F}},
	{0x7a,1,{0x0E}},//0X   2
	{0x7b,1,{0x0D}},
	{0x7c,1,{0x0C}},
	{0x7d,1,{0x06}},
	{0x7e,1,{0x02}},
	{0x7f,1,{0x0A}},
	{0x80,1,{0x02}},
	{0x81,1,{0x02}},
	{0x82,1,{0x02}},
	{0x83,1,{0x02}},
	{0x84,1,{0x02}},
	{0x85,1,{0x02}},
	{0x86,1,{0x02}},
	{0x87,1,{0x02}},
	{0x88,1,{0x08}},
	{0x89,1,{0x02}},
	{0x8A,1,{0x02}},
	
	{0xFF,3,{0x98,0x81,0x04}},//0X	 2
	{0x00,1,{0x00}},
	{0x6C,1,{0x15}},
	{0x6E,1,{0x3B}},
	{0x6F,1,{0x53}},
	{0x3A,1,{0xA4}},
	{0x8D,1,{0x15}},
	{0x87,1,{0xBA}},
	{0x26,1,{0x76}},
	{0xB2,1,{0xD1}},
	{0x88,1,{0x0B}},
	
	{0xFF,3,{0x98,0x81,0x01}},
	{0x22,1,{0x0A}},
	{0x31,1,{0x00}},
	{0x53,1,{0x5C}},
	{0x55,1,{0x88}},
	{0x50,1,{0x90}},
	{0x51,1,{0x90}},//0X   2
	{0x60,1,{0x14}},
	{0xA0,1,{0x09}},
	{0xA1,1,{0x26}}, //0X	2
	{0xA2,1,{0x37}},
	{0xA3,1,{0x15}},
	{0xA4,1,{0x18}},
	{0xA5,1,{0x2C}},
	{0xA6,1,{0x20}},//0X   2
	{0xA7,1,{0x21}},	   //VP203
	{0xA8,1,{0xAD}},	   //VP175
	{0xA9,1,{0x1A}},	   //VP144
	{0xAA,1,{0x28}},	   //VP111
	{0xAB,1,{0x95}},	   //VP80
	{0xAC,1,{0x19}},	   //VP52
	{0xAD,1,{0x19}},	   //VP36
	{0xAE,1,{0x4E}},	   //VP24
	{0xAF,1,{0x21}},	   //VP16
	{0xB0,1,{0x2C}},	   //VP12
	{0xB1,1,{0x55}},	   //VP8
	{0xB2,1,{0x5E}},	   //VP4
	{0xB3,1,{0x35}},	   //VP0
	{0xC0,1,{0x00}},	 //VN255 GAMMA N
	{0xC1,1,{0x26}},	   //VN251
	{0xC2,1,{0x37}},	   //VN247
	{0xC3,1,{0x14}},	   //VN243
	{0xC4,1,{0x19}},	   //VN239
	{0xC5,1,{0x2B}},	   //VN231
	{0xC6,1,{0x21}},	   //VN219
	{0xC7,1,{0x20}},	   //VN203
	{0xC8,1,{0xAD}},	   //VN175
	{0xC9,1,{0x1F}},	   //VN144
	{0xCA,1,{0x29}},	   //VN111
	{0xCB,1,{0x95}},	   //VN80
	{0xCC,1,{0x1F}},	   //VN52
	{0xCD,1,{0x20}},	   //VN36
	{0xCE,1,{0x58}},	   //VN24
	{0xCF,1,{0x30}},	   //VN16
	{0xD0,1,{0x2C}},	   //VN12
	{0xD1,1,{0x55}},	   //VN8
	{0xD2,1,{0x5E}},	   //VN4
	{0xD3,1,{0x39}},	   //VN0
	
	{0xFF,3,{0x98,0x81,0x00}},
	{0x35,1,{0x00}},	//TE on
	{0x3A,1,{0x77}},
	
	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},
	
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
#endif

#if 1
	//TXDT550UYPA-71  LCM
	#if 0
	{0xFF,3,{0x98,0x81,0x03}},		 
	{0x01,1,{0x08}},	   
	{0x02,1,{0x00}},	  
	{0x03,1,{0x73}},		
	{0x04,1,{0x73}},		
	{0x05,1,{0x14}},		 
	{0x06,1,{0x06}},		
	{0x07,1,{0x02}},		
	{0x08,1,{0x05}},		 
	{0x09,1,{0x14}},		 
	{0x0a,1,{0x14}},		
	{0x0b,1,{0x00}},		
	{0x0c,1,{0x14}},		
	{0x0d,1,{0x14}},		
	{0x0e,1,{0x00}},		
	{0x0f,1,{0x0C}},
			
	{0x10,1,{0x0C}},		
	{0x11,1,{0x0C}},	   
	{0x12,1,{0x0C}},		
	{0x13,1,{0x14}},		
	{0x14,1,{0x0c}},		
	{0x15,1,{0x00}},		
	{0x16,1,{0x00}},	   
	{0x17,1,{0x00}},		
	{0x18,1,{0x00}},		
	{0x19,1,{0x00}},		
	{0x1a,1,{0x00}},		
	{0x1b,1,{0x00}},		
	{0x1c,1,{0x00}},	   
	{0x1d,1,{0x00}},	   
	{0x1e,1,{0xc8}},	   
	{0x1f,1,{0x80}},
			
	{0x20,1,{0x02}},		
	{0x21,1,{0x00}},	   
	{0x22,1,{0x02}},	   
	{0x23,1,{0x00}},		
	{0x24,1,{0x00}},	   
	{0x25,1,{0x00}},	   
	{0x26,1,{0x00}},	   
	{0x27,1,{0x00}},	   
	{0x28,1,{0xfb}},	   
	{0x29,1,{0x43}},		
	{0x2a,1,{0x00}},	   
	{0x2b,1,{0x00}},	   
	{0x2c,1,{0x07}},	   
	{0x2d,1,{0x07}},	   
	{0x2e,1,{0xff}},	   
	{0x2f,1,{0xff}},
		   
	{0x30,1,{0x11}},	   
	{0x31,1,{0x00}},	   
	{0x32,1,{0x00}},	   
	{0x33,1,{0x00}},	   
	{0x34,1,{0x84}},	
	{0x35,1,{0x80}},	   
	{0x36,1,{0x07}},		
	{0x37,1,{0x00}},		
	{0x38,1,{0x00}},		
	{0x39,1,{0x00}},		
	{0x3a,1,{0x00}},		
	{0x3b,1,{0x00}},		
	{0x3c,1,{0x00}},		
	{0x3d,1,{0x00}},		
	{0x3e,1,{0x00}},		
	{0x3f,1,{0x00}},	
		
	{0x40,1,{0x00}},		
	{0x41,1,{0x88}},		 
	{0x42,1,{0x00}},		
	{0x43,1,{0x80}},		
	{0x44,1,{0x08}},	
		
	{0x50,1,{0x01}},		 
	{0x51,1,{0x23}},		 
	{0x52,1,{0x45}},		 
	{0x53,1,{0x67}},		
	{0x54,1,{0x89}},		
	{0x55,1,{0xab}},		
	{0x56,1,{0x01}},		
	{0x57,1,{0x23}},		
	{0x58,1,{0x45}},		
	{0x59,1,{0x67}},		
	{0x5a,1,{0x89}},		
	{0x5b,1,{0xab}},		
	{0x5c,1,{0xcd}},		
	{0x5d,1,{0xef}},
			
	{0x5e,1,{0x10}},		
	{0x5f,1,{0x02}},		
	{0x60,1,{0x08}},		
	{0x61,1,{0x09}},		
	{0x62,1,{0x10}},	   
	{0x63,1,{0x12}},		 
	{0x64,1,{0x11}},		 
	{0x65,1,{0x13}},		
	{0x66,1,{0x0c}},		
	{0x67,1,{0x02}},		
	{0x68,1,{0x02}},		
	{0x69,1,{0x02}},		
	{0x6a,1,{0x02}},		
	{0x6b,1,{0x02}},		
	{0x6c,1,{0x0e}},		
	{0x6d,1,{0x0d}},		
	{0x6e,1,{0x0f}},		
	{0x6f,1,{0x02}},
			
	{0x70,1,{0x02}},		
	{0x71,1,{0x06}},		
	{0x72,1,{0x07}},		
	{0x73,1,{0x02}},		
	{0x74,1,{0x02}},		
	{0x75,1,{0x02}},		
	{0x76,1,{0x07}},		
	{0x77,1,{0x06}},		
	{0x78,1,{0x11}},		
	{0x79,1,{0x13}},	   
	{0x7a,1,{0x10}},	   
	{0x7b,1,{0x12}},	   
	{0x7c,1,{0x0f}},		
	{0x7d,1,{0x02}},		
	{0x7e,1,{0x02}},		
	{0x7f,1,{0x02}},
			
	{0x80,1,{0x02}},	   
	{0x81,1,{0x02}},	   
	{0x82,1,{0x0d}},	   
	{0x83,1,{0x0e}},	   
	{0x84,1,{0x0c}},	   
	{0x85,1,{0x02}},	   
	{0x86,1,{0x02}},		
	{0x87,1,{0x09}},		
	{0x88,1,{0x08}},	   
	{0x89,1,{0x02}},	  
	{0x8A,1,{0x02}},
		 
	{0xFF,3,{0x98,0x81,0x04}},
	{0x00,1,{0x00}},	   
	{0x6C,1,{0x15}},	   
	{0x6E,1,{0x2B}},		
	{0x6F,1,{0x33}},		
	{0x3A,1,{0x24}},	   
	{0x8D,1,{0x14}},	   
	{0x87,1,{0xBA}},		
	{0x26,1,{0x76}},		
	{0xB2,1,{0xD1}},	   
	//{0xB5,1,{0x06}},
		 
	{0xFF,3,{0x98,0x81,0x01}},		 
	{0x22,1,{0x09}},	  
	{0x31,1,{0x0B}},		
	//{0x34,1,{0x01}},		
	{0x50,1,{0x96}},		
	{0x51,1,{0x96}},
	{0x53,1,{0x51}},
	{0x55,1,{0x8F}},		
	{0x60,1,{0x14}},	   
	//{0x61,1,{0x00}},		
	//{0x62,1,{0x19}},		
	//{0x63,1,{0x10}},		
	{0xA0,1,{0x08}},		
	{0xA1,1,{0x11}},		
	{0xA2,1,{0x1A}},	   
	{0xA3,1,{0x0C}},	   
	{0xA4,1,{0x0F}},		
	{0xA5,1,{0x18}},		
	{0xA6,1,{0x0F}},		
	{0xA7,1,{0x13}},		
	{0xA8,1,{0x65}},		
	{0xA9,1,{0x10}},	   
	{0xAA,1,{0x16}},	   
	{0xAB,1,{0x65}},	   
	{0xAC,1,{0x1D}},
			
	{0xAD,1,{0x15}},		
	{0xAE,1,{0x49}},	   
	{0xAF,1,{0x1E}},		
	{0xB0,1,{0x22}},	   
	{0xB1,1,{0x58}},		
	{0xB2,1,{0x68}},	   
	{0xB3,1,{0x39}},		
	{0xC0,1,{0x08}},
		   
	{0xC1,1,{0x15}},		
	{0xC2,1,{0x21}},	   
	{0xC3,1,{0x15}},	   
	{0xC4,1,{0x13}},		
	{0xC5,1,{0x2B}},		
	{0xC6,1,{0x1D}},		
	{0xC7,1,{0x1E}},		
	{0xC8,1,{0x7A}},		
	{0xC9,1,{0x27}},		
	{0xCA,1,{0x38}},		
	{0xCB,1,{0x79}},		
	{0xCC,1,{0x16}},
			
	{0xCD,1,{0x1C}},	   
	{0xCE,1,{0x4F}},		
	{0xCF,1,{0x25}},		
	{0xD0,1,{0x2D}},	   
	{0xD1,1,{0x5A}},	   
	{0xD2,1,{0x6A}},	   
	{0xD3,1,{0x39}},

	{0xFF,3,{0x98,0x81,0x00}},
	{0x35,1,{0x00}},	//TE on
	{0x36,1,{0x03}},

	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},

	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 50, {}},
	#endif

	//merge code by pengwei 20160427
	#if 1
	{0xFF,3,{0x98,0x81,0x03}},	 
	{0x01,1,{0x08}},	 
	{0x02,1,{0x00}},	 
	{0x03,1,{0x73}},		 
	{0x04,1,{0x73}},			 
	{0x05,1,{0x14}},		 
	{0x06,1,{0x06}},		 
	{0x07,1,{0x02}},		 
	{0x08,1,{0x05}},		 
	{0x09,1,{0x14}},		 
	{0x0a,1,{0x14}},	 
	{0x0b,1,{0x00}},			 
	{0x0c,1,{0x14}},			 
	{0x0d,1,{0x14}},			 
	{0x0e,1,{0x00}},			 
	{0x0f,1,{0x0C}},			 
	{0x10,1,{0x0C}},			 
	{0x11,1,{0x0C}},			 
	{0x12,1,{0x0C}},			 
	{0x13,1,{0x14}},			 
	{0x14,1,{0x0c}},			 
	{0x15,1,{0x00}},			 
	{0x16,1,{0x00}},			 
	{0x17,1,{0x00}},			 
	{0x18,1,{0x00}},			 
	{0x19,1,{0x00}},			 
	{0x1a,1,{0x00}},			 
	{0x1b,1,{0x00}},		 
	{0x1c,1,{0x00}},			 
	{0x1d,1,{0x00}},			 
	{0x1e,1,{0xc8}},			 
	{0x1f,1,{0x80}},			 
	{0x20,1,{0x02}},			 
	{0x21,1,{0x00}},			 
	{0x22,1,{0x02}},			 
	{0x23,1,{0x00}},			 
	{0x24,1,{0x00}},			 
	{0x25,1,{0x00}},			 
	{0x26,1,{0x00}},			 
	{0x27,1,{0x00}},			 
	{0x28,1,{0xfb}},		 
	{0x29,1,{0x43}},			 
	{0x2a,1,{0x00}},			 
	{0x2b,1,{0x00}},			 
	{0x2c,1,{0x07}},			 
	{0x2d,1,{0x07}},			 
	{0x2e,1,{0xff}},			 
	{0x2f,1,{0xff}},			 
	{0x30,1,{0x11}},			 
	{0x31,1,{0x00}},			 
	{0x32,1,{0x00}},			 
	{0x33,1,{0x00}},			 
	{0x34,1,{0x84}},			 
	{0x35,1,{0x80}},			 
	{0x36,1,{0x07}},			 
	{0x37,1,{0x00}},			 
	{0x38,1,{0x00}},			 
	{0x39,1,{0x00}},			 
	{0x3a,1,{0x00}},			 
	{0x3b,1,{0x00}},			 
	{0x3c,1,{0x00}},			 
	{0x3d,1,{0x00}},			 
	{0x3e,1,{0x00}},			 
	{0x3f,1,{0x00}},			 
	{0x40,1,{0x00}},			 
	{0x41,1,{0x88}},			 
	{0x42,1,{0x00}},			 
	{0x43,1,{0x80}},			 
	{0x44,1,{0x08}},			 
	{0x50,1,{0x01}},			 
	{0x51,1,{0x23}},			 
	{0x52,1,{0x45}},			 
	{0x53,1,{0x67}},			 
	{0x54,1,{0x89}},			 
	{0x55,1,{0xab}},		 
	{0x56,1,{0x01}},		 
	{0x57,1,{0x23}},		 
	{0x58,1,{0x45}},			 
	{0x59,1,{0x67}},			 
	{0x5a,1,{0x89}},			 
	{0x5b,1,{0xab}},			 
	{0x5c,1,{0xcd}},			 
	{0x5d,1,{0xef}},			 
	{0x5e,1,{0x10}},			 
	{0x5f,1,{0x02}},			 
	{0x60,1,{0x08}},			 
	{0x61,1,{0x09}},			 
	{0x62,1,{0x10}},			 
	{0x63,1,{0x12}},			 
	{0x64,1,{0x11}},			 
	{0x65,1,{0x13}},			 
	{0x66,1,{0x0c}},			 
	{0x67,1,{0x02}},			 
	{0x68,1,{0x02}},			 
	{0x69,1,{0x02}},			 
	{0x6a,1,{0x02}},			 
	{0x6b,1,{0x02}},			 
	{0x6c,1,{0x0e}},		 
	{0x6d,1,{0x0d}},			 
	{0x6e,1,{0x0f}},			 
	{0x6f,1,{0x02}},			 
	{0x70,1,{0x02}},			 
	{0x71,1,{0x06}},			 
	{0x72,1,{0x07}},			 
	{0x73,1,{0x02}},			 
	{0x74,1,{0x02}},			 
	{0x75,1,{0x02}},		 
	{0x76,1,{0x07}},			 
	{0x77,1,{0x06}},			 
	{0x78,1,{0x11}},		 
	{0x79,1,{0x13}},		 
	{0x7a,1,{0x10}},		 
	{0x7b,1,{0x12}},		 
	{0x7c,1,{0x0f}},		 
	{0x7d,1,{0x02}},		 
	{0x7e,1,{0x02}},		 
	{0x7f,1,{0x02}},		 
	{0x80,1,{0x02}},		 
	{0x81,1,{0x02}},		 
	{0x82,1,{0x0d}},			 
	{0x83,1,{0x0e}},			 
	{0x84,1,{0x0c}},			 
	{0x85,1,{0x02}},			 
	{0x86,1,{0x02}},			 
	{0x87,1,{0x09}},			 
	{0x88,1,{0x08}},			 
	{0x89,1,{0x02}},			 
	{0x8A,1,{0x02}},
			 
	{0xFF,3,{0x98,0x81,0x04}},	
	{0x00,1,{0x00}},
	{0x6C,1,{0x15}},			 
	{0x6E,1,{0x2D}},		 
	{0x6F,1,{0x33}},			 
	{0x3A,1,{0xA4}},			 
	{0x8D,1,{0x14}},			 
	{0x87,1,{0xBA}},			 
	{0x26,1,{0x76}},			 
	{0xB2,1,{0xD1}},			 
	{0xB5,1,{0x06}},			 
	{0x33,1,{0x44}},			 
	{0x69,1,{0x57}},			 
	{0x88,1,{0x0B}},
			 
	{0xFF,3,{0x98,0x81,0x01}},			 
	{0x22,1,{0x0A}},			 
	{0x31,1,{0x00}},			 
	{0x53,1,{0x75}},			 
	{0x55,1,{0x85}},			 
	{0x50,1,{0xB7}},			 
	{0x51,1,{0xB3}},		 
	{0x60,1,{0x14}},			 
	{0x61,1,{0x00}},			 
	{0x62,1,{0x19}},			 
	{0x63,1,{0x10}},			 
	{0xA0,1,{0x01}},			 
	{0xA1,1,{0x1A}},			 
	{0xA2,1,{0x2A}},			 
	{0xA3,1,{0x0F}},			 
	{0xA4,1,{0x12}},			 
	{0xA5,1,{0x26}},			 
	{0xA6,1,{0x1A}},			 
	{0xA7,1,{0x1C}},			 
	{0xA8,1,{0x9C}},			 
	{0xA9,1,{0x1A}},			 
	{0xAA,1,{0x2D}},			 
	{0xAB,1,{0x8E}},		 
	{0xAC,1,{0x1C}},		 
	{0xAD,1,{0x1C}},			 
	{0xAE,1,{0x4F}},			 
	{0xAF,1,{0x23}},			 
	{0xB0,1,{0x2A}},			 
	{0xB1,1,{0x57}},		 
	{0xB2,1,{0x62}},		 
	{0xB3,1,{0x28}},			 
	{0xC0,1,{0x0B}},			 
	{0xC1,1,{0x25}},			 
	{0xC2,1,{0x35}},			 
	{0xC3,1,{0x1A}},			 
	{0xC4,1,{0x1D}},			 
	{0xC5,1,{0x30}},			 
	{0xC6,1,{0x25}},			 
	{0xC7,1,{0x25}},			 
	{0xC8,1,{0xA4}},			 
	{0xC9,1,{0x20}},			 
	{0xCA,1,{0x26}},			 
	{0xCB,1,{0x8E}},			 
	{0xCC,1,{0x1B}},			 
	{0xCD,1,{0x1B}},			 
	{0xCE,1,{0x4F}},			 
	{0xCF,1,{0x23}},			 
	{0xD0,1,{0x2B}},			 
	{0xD1,1,{0x57}},			 
	{0xD2,1,{0x62}},			 
	{0xD3,1,{0x28}},
			 
	{0xFF,3,{0x98,0x81,0x00}},		 
	{0x35,1,{0x00}},	//TE on
	//{0x36,1,{0x03}},

	{0x11, 1, {0x00}},
	{REGFLAG_DELAY, 150, {}},

	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 20, {}},
	#endif
#endif


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
    {REGFLAG_DELAY, 10, {}},
    
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

	params->physical_width = 68;
	params->physical_height = 120;

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
    params->dsi.LANE_NUM				= LCM_THREE_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

    // Highly depends on LCD driver capability.
    // Not support in MT6573
    params->dsi.packet_size=256;
    params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

#if 0
	params->dsi.PLL_CLOCK = 260; //this value must be in MTK suggested table

	params->dsi.vertical_sync_active  = 8;
	params->dsi.vertical_backporch    = 20;
	params->dsi.vertical_frontporch   = 20;
	params->dsi.vertical_active_line  = FRAME_HEIGHT;


	params->dsi.horizontal_sync_active = 40;
	params->dsi.horizontal_backporch  = 120;
	params->dsi.horizontal_frontporch = 120;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.HS_TRAIL              = 15;
#else

	params->dsi.PLL_CLOCK = 250; //this value must be in MTK suggested table

	params->dsi.vertical_sync_active  = 2;
	params->dsi.vertical_backporch    = 14;
	params->dsi.vertical_frontporch   = 16;
	params->dsi.vertical_active_line  = FRAME_HEIGHT;


	params->dsi.horizontal_sync_active = 8;
	params->dsi.horizontal_backporch  = 24;
	params->dsi.horizontal_frontporch = 32;
	params->dsi.horizontal_active_pixel = FRAME_WIDTH;

	params->dsi.HS_TRAIL              = 120;
#endif

	//params->dsi.ssc_disable=0;
	//params->dsi.ssc_range=4;
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
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(100);

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
	
#if 0
	lcm_init();
#else
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);
#endif

}

#if (LCM_DSI_CMD_MODE)
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
#endif

static unsigned int lcm_compare_id(void)
{
	unsigned int id1 = 0,id2 = 0;
	unsigned int lcm_id = 0;
	unsigned char buffer[3];
	unsigned int array[8];

	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(100);

	array[0] = 0x00043902;
	array[1] = 0x018198FF;
	dsi_set_cmdq(array, 2, 1);
	MDELAY(1);
	
	array[0] = 0x00013700;// return byte number
	dsi_set_cmdq(array, 1, 1);
	MDELAY(1);

	read_reg_v2(0x00, buffer, 1);
	id1 = buffer[0];	

	read_reg_v2(0x01, buffer, 1);
	id2 = buffer[0];

	lcm_id = ((id1<<8) | id2);

#ifdef BUILD_LK
	printf("LK,ILI9881C: id1=0x%x,id2=0x%x\n",id1,id2);   
#else
	printk("KERNEL,ILI9881C: id1=0x%x,id2=0x%x\n",id1,id2);  
#endif

	return (ILI9881C_LCM_ID == lcm_id)?1:0;
}


LCM_DRIVER ili9881c_dzx_y85239_hd720_dsi_vdo_3lane_lcm_drv = 
{
    .name			= "ili9881c_dzx_y85239_hd720_dsi_vdo_3lane",
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
