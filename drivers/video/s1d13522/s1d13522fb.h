//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson-13522/s1d13522fb.h --
// Function header for Epson S1D13522 controller frame buffer drivers.
//
// Copyright(c) Seiko Epson Corporation 2009.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------

#ifndef __S1D13522FB_H__
#define __S1D13522FB_H__

#include <linux/kernel.h>
#include <linux/fb.h>

#include "s1d13522ioctl.h"
#include "s1d13522mem.h"
#include "s1d13522.h"

#ifndef FALSE
  #define FALSE 0
#endif

#ifndef TRUE
  #define TRUE  (!FALSE)
#endif

#define HRDY_TIMEOUT 200

#ifdef CONFIG_FB_EPSON_GPIO_GUMSTIX
#ifdef CONFIG_FB_EPSON_PCI
#undef CONFIG_FP_EPSON_PCI
#endif
#endif

#ifdef VIDEO_REFRESH_PERIOD
#undef VIDEO_REFRESH_PERIOD
#endif

// This is the refresh period for updating the display for RAM based LCDs
// and/or indirect interfaces.  It is set to (1 second / X).  This value can
// modified by the end-user.  Be aware that decreasing the refresh period
// increases CPU usage as well.
// There are HZ (typically 100) jiffies per second
#if CONFIG_FB_EPSON_VIRTUAL_FRAMEBUFFER_FREQ != 0
#define VIDEO_REFRESH_PERIOD   (HZ/CONFIG_FB_EPSON_VIRTUAL_FRAMEBUFFER_FREQ)
#endif

/* Copied from defio s1d13522fb.h file */

struct s1d13522fb_par {
	struct fb_info *info;
	struct s1d13522fb_board *board;
	wait_queue_head_t waitq;
	int panel_index;
	u32 pseudo_palette[16];
	int dirtyrange;
	u32 dx1,dx2,dy1,dy2;
	u32 panelw,panelh;
	struct mutex io_lock;
	void *epd_buffer_vfb;
	u32 epd_buffer_vfb_memorysize;
	int epd_buffer_vfb_stride;
	void *epd_buffer_host;	// native EPD format 4bpp
	u32   epd_buffer_host_memorysize;
	int   epd_buffer_host_stride;
	void *epd_buffer_host_current;
	u32 mode;
	u32 waveform;
	//regal options
	bool regal_on;
	bool regal_d_on;
	wait_queue_head_t vsync_wait;
       int vsync_flag;
       int vsync_timeout;
	struct s1d13522_mem_cache *malloc_cache;
};

// board specific routines
struct s1d13522fb_board {
	struct module *owner;
	int  (*init)(struct s1d13522fb_par *);
	int  (*wait_for_rdy)(struct s1d13522fb_par *, int timeoutms);
	void (*cleanup)(struct s1d13522fb_par *);
	void (*write_reg)(struct s1d13522fb_par *, u16, u16);
	u16  (*read_reg)(struct s1d13522fb_par *, u16);
	int  (*send_command)(struct s1d13522fb_par *, u16);
	int  (*send_cmdargs)(struct s1d13522fb_par *, u16, int, u16*);
	void (*burst_write)(struct s1d13522fb_par *, int, u16*);
	void (*burst_read)(struct s1d13522fb_par *, int, u16*);
	int  (*suspend)(struct s1d13522fb_par *);
	int  (*resume)(struct s1d13522fb_par *);
	int  (*setup_irq)(struct fb_info *);
};

int s1d13522fb_setcolreg(unsigned regno, unsigned red, unsigned green, unsigned blue, unsigned transp, struct fb_info *info);
void s1d13522_write_reg(struct s1d13522fb_par *par, u16 reg, u16 data);
u16  s1d13522_read_reg(struct s1d13522fb_par *par, u16 reg);
int s1d13522_send_cmdargs(struct s1d13522fb_par *par,u16 cmd, int argc, u16 *argv);
int s1d13522_send_command(struct s1d13522fb_par *par,u16 cmd);
void s1d13522fb_dpy_update(struct s1d13522fb_par *par);
void s1d13522fb_dpy_update_area(struct s1d13522fb_par *par);

// S1D13522 command defines
#define S1D13522_CMD_INIT_CMD_SET			0x00
#define S1D13522_CMD_INIT_PLL				0x01	// 4 parameters
#define S1D13522_CMD_RUN_SYS				0x02
#define S1D13522_CMD_STBY					0x04
#define S1D13522_CMD_SLP					0x05
#define S1D13522_CMD_INIT_SYS_RUN			0x06	// 1 parameter
#define S1D13522_CMD_INIT_DSPE_CFG			0x09	// 5 parameters
#define S1D13522_CMD_INIT_DSPE_TMG			0x0A	// 5 parameters
#define S1D13522_CMD_INIT_ROTMODE			0x0B	// 1 parameter
#define S1D13522_CMD_INIT_WAVE_DEV			0x0C	// 1 parameter
#define S1D13522_CMD_INIT_DSPE_TMG_ADV		0x0D	// 5 parameters
#define S1D13522_CMD_RD_REG					0x10	// 2 parameters
#define S1D13522_CMD_WR_REG					0x11	// 2 parameters

#define S1D13522_CMD_PIP_DISABLE			0x14
#define S1D13522_CMD_PIP_ENABLE				0x15	// 4 parameters
#define S1D13522_CMD_PIP_ADRCFG				0x16	// 2 parameters
#define S1D13522_CMD_PIP_XYSETUP			0x17	// 2 parameters
#define S1D13522_CMD_CSR_MAINCFG			0x18	// 4 parameters
#define S1D13522_CMD_CSR_XYSETUP			0x19	// 2 parameters
#define S1D13522_CMD_CSR_ADRCFG				0x1A	// 1 parameter

#define S1D13522_CMD_BST_RD_SDR				0x1C	// 4 parameters
#define S1D13522_CMD_BST_WR_SDR				0x1D	// 4 parameters
#define S1D13522_CMD_BST_END_SDR			0x1E

#define S1D13522_CMD_LD_IMG					0x20	// 1 parameter
#define S1D13522_CMD_LD_IMG_AREA			0x22	// 5 parameters
#define S1D13522_CMD_LD_IMG_END				0x23
#define S1D13522_CMD_LD_IMG_SETADR			0x25	// 2 parameters
#define S1D13522_CMD_LD_IMG_DSPEADR			0x26

#define S1D13522_CMD_WAIT_DSPE_TRG			0x28
#define S1D13522_CMD_WAIT_DSPE_FREND		0x29
#define S1D13522_CMD_WAIT_DSPE_LUTFREE		0x2A
#define S1D13522_CMD_WAIT_DSPE_MLUTFREE		0x2B	// 1 parameter

#define S1D13522_CMD_RD_WFM_INFO			0x30	// 2 parameters
#define S1D13522_CMD_UPD_INIT				0x32	// 1 parameter
#define S1D13522_CMD_UPD_FULL				0x33	// 1 parameter
#define S1D13522_CMD_UPD_FULL_AREA			0x34	// 5 parameters
#define S1D13522_CMD_UPD_PART				0x35	// 1 parameter
#define S1D13522_CMD_UPD_PART_AREA			0x36	// 5 parameters
#define S1D13522_CMD_UPD_GDRV_CLR			0x37
#define S1D13522_CMD_UPD_SET_IMGADR			0x38	// 2 parameters

#define S1D13522_CMD_PEN_DRAW				0x3A	// 5 parameters
#define S1D13522_CMD_PEN_MENU				0x3B	// 1 parameter
#define S1D13522_CMD_PEN_LINE				0x3C	// 5 parameters


// S1D13522 register interface defines
#define S1D13522_REG0000_REV_CODE				0x0000	// Revision Code register
#define S1D13522_REG0002_PROD_CODE				0x0002	// Product Code register
#define S1D13522_REG0006_POWER_SAVE_MODE		0x0006	// Power Save Mode register
#define S1D13522_REG0008_SOFTWARE_RESET			0x0008	// Software Reset register [Reserved!!!]
#define S1D13522_REG000A_SYSTEM_STATUS			0x000A	// System Status register
#define S1D13522_REG0010_PLL_CONFIG0			0x0010	// Pll Configuration register 0
#define S1D13522_REG0012_PLL_CONFIG1			0x0012	// Pll Configuration register 1
#define S1D13522_REG0014_PLL_CONFIG2			0x0014	// Pll Configuration register 2
#define S1D13522_REG0016_CLOCK_CONFIG			0x0016	// Clock Configuration register
#define S1D13522_REG001A_I2C_CLOCK_CONFIG		0x001A	// I2C Clock Configuration register
#define S1D13522_REG0310_IMG_BUFFER_SA0			0x0310	// Image Buffer Start Adress register 0
#define S1D13522_REG0312_IMG_BUFFER_SA1			0x0312	// Image Buffer Start Adress register 1
#define S1D13522_REG0320_TEMPERATURE_DEVICE		0x0320	// Temperature Device Select register
#define S1D13522_REG032A_POWER_CONTROL_CONFIG	0x032A	// Power Control Configuration register
#define S1D13522_REG0330_UPDATE_BUFFER_CONFIG	0x0330	// Panel Update Buffer Configuration register
#define S1D13522_REG0338_DISPLAY_STATUS			0x0338	// Display Engine Busy Status register

/* ends here */


// In Indirect Mode, a copy of the framebuffer is kept in system memory.
// A timer periodically writes this copy to the "real" framebuffer in
// hardware. This copy is called a virtual framebuffer.

//----------------------------------------------------------------------------
// Global structures used by s1d13522fb frame buffer code
//----------------------------------------------------------------------------
typedef struct
{
	volatile unsigned char *RegAddr;
	unsigned RegAddrMappedSize;
	u32 VirtualFramebufferAddr;
	int blank_mode;
	int display_changed;
	u32 pseudo_palette[16];
	struct mutex refresh_display_mutex;
#ifdef VIDEO_REFRESH_PERIOD
	struct timer_list timer;
	struct work_struct work;
#endif
	struct work_struct INTwork;

}FB_INFO_S1D13522;

//extern struct fb_fix_screeninfo s1d13522fb_fix;
extern struct fb_info s1d13522_fb;
extern FB_INFO_S1D13522 s1d13522fb_info;
extern char *s1d13522fb_version;

//-----------------------------------------------------------------------------
// Global Function Prototypes
//-----------------------------------------------------------------------------

#ifdef CONFIG_FB_EPSON_DEBUG_PRINTK
#define dbg_info(fmt, args...) do { dev_dbg(devlog, fmt, ## args); } while (0)
    //#define dbg_info(fmt, args...) do { printk(KERN_ERR fmt, ## args); } while (0)
#else
#define dbg_info(fmt, args...) do { } while (0)
#endif


#ifdef CONFIG_FB_EPSON_DEBUG_PRINTK
#define assert(expr) \
        if(!(expr)) { \
        printk( "Assertion failed! %s,%s,%s,line=%d\n",\
        #expr,__FILE__,__FUNCTION__,__LINE__); \
        BUG(); \
        }
#else
#define assert(expr)
#endif

struct s1d13522_platform_data {
	int (*s1d13522_power)(int on);
};

void __devexit s1d13522fb_exit(void);

#ifdef CONFIG_FB_EPSON_PROC
int  s1d13522proc_init(void);
void s1d13522proc_terminate(void);
#endif

#ifdef CONFIG_FB_EPSON_PCI
int  __devinit s1d13522pci_init(long *physicalAddress);
void __devexit s1d13522pci_terminate(void);
#endif

int  s1d13522fb_init(void);
int  s1d13522fb_re_init(void);
int  s1d13522if_InterfaceReInit(FB_INFO_S1D13522 *info);
int  __devinit s1d13522if_InterfaceInit(FB_INFO_S1D13522 *info);
void __devexit s1d13522if_InterfaceTerminate(FB_INFO_S1D13522 *info);
int  s1d13522if_cmd(unsigned ioctlcmd,s1d13522_ioctl_cmd_params *params,int numparams);
void s1d13522if_BurstWrite16(u16 *ptr16, unsigned copysize16);
void s1d13522if_BurstRead16(u16 *ptr16, unsigned copysize16);
u16  s1d13522if_ReadReg16(u16 Index);
void s1d13522if_WriteReg16(u16 Index, u16 Value);
int  s1d13522if_WaitForHRDY(void);
void s1d13522fb_do_refresh_display(unsigned cmd,unsigned mode);
void s1d13522fb_display_init(unsigned cmd,unsigned mode, struct s1d13522fb_par *par);
void s1d13522fb_defaultDisplay(struct s1d13522fb_par *par);
int s1d13522fb_InitRegisters(void);
int s1d13522_init_cmdfw(unsigned ioctlcmd);
int s1d13522if_Cmd_PackPara(unsigned ioctlcmd, u16 *params, int numparams);
void s1d13522if_BurstWrite16fb_Area(int left, int top,  int mWidth, int mHeight, u16 *ptr16, dma_addr_t handle);
void s1d13522if_BurstWrite16fb(u16 *ptr16, dma_addr_t handle, unsigned copysize16);
void waveformTransfer(u16 *ptr, unsigned copysize);
int poweron(int on);


int s1d13522if_WriteTps65185bReg8(u8 Index, u8 Value);
int s1d13522if_PMIC_Write_Reg(u8 index, u8 val);

u8 s1d13522if_ReadTps65185bReg8(u8 Index);
u8 s1d13522if_PMIC_Read_Reg(u8 Index);

//regal support
bool s1d13522fb_regal_on ( struct s1d13522fb_par *par );		// enables Regal pre-processing
bool s1d13522fb_regal_off( struct s1d13522fb_par *par );		// disables Regal pre-processing
bool s1d13522fb_regal_get( struct s1d13522fb_par *par );		// returns current Regal state
bool s1d13522fb_regal_gb_process( struct s1d13522fb_par *par );		// trigger the Regal gray buffer processing

bool s1d13522fb_regal_d_on ( struct s1d13522fb_par *par );		// enables Regal-D pre-processing
bool s1d13522fb_regal_d_off( struct s1d13522fb_par *par );		// disables Regal-D pre-processing
bool s1d13522fb_regal_d_get( struct s1d13522fb_par *par );		// returns current Regal-D state
bool s1d13522fb_regal_d_gb_process( struct s1d13522fb_par *par );	// trigger the Regal-D gray buffer processing

#endif  //__S1D13522FB_H__
