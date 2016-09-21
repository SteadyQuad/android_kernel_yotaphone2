//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson-13522/s1d1352ioctl.h -- IOCTL definitions for Epson
// S1D13522 controller frame buffer driver.
//
// Copyright(c) Seiko Epson Corporation 2009.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------
#ifndef S1D13522_IOCTL_H
#define S1D13522_IOCTL_H

/* ioctls
    0x45 is 'E'                                                          */

struct s1d13522_ioctl_hwc
{
        unsigned addr;
        unsigned value;
        void* buffer;
};

struct s1d13522_ioctl_temperature
{
        u16 index;
        signed short value[3];
};

#define S1D13522_REGREAD                0x4540
#define S1D13522_REGWRITE               0x4541
#define S1D13522_MEMBURSTREAD           0x4546
#define S1D13522_MEMBURSTWRITE          0x4547
#define S1D13522_VBUF_REFRESH           0x4548

#define S1D13522_VBUF_REFRESH_GLOBAL       0x4549
#define S1D13522_VBUF_REFRESH_AREA           0x454A
#define S1D13522_VBUF_REFRESH_PIP_GLOBAL 0x454B
#define S1D13522_VBUF_REFRESH_PIP_AREA      0x454C
#define S1D13522_MOVE_PIP_POSITION 0x454D
#define S1D13522_REMOVE_PIP      0x454E
#define S1D13522_GET_TEMP      0x454F
#define S1D13522_SET_VCOM      0x4550
#define S1D13522_PRG_INIT   0x4551
#define S1D13522_PRG_FLASH   0x4552
#define S1D13522_PRG_EXIT   0x4553

#define S1D13522_SET_TEMP      0x4554
#define S1D13522_SET_BORDER_COLOR      0x4555
#define S1D13522_UPDATE_BORDER       0x4556
#define S1D13522_GET_PHONE_COLOR       0x4557 // added coz of new requirement.

#define S1D13522_CURSOR_ENABLE 0x4558
#define S1D13522_CURSOR_MOVE 0x4559
#define S1D13522_CURSOR_DISABLE 0x4560

#define S1D13522_SET_REGAL_ON               0x4561
#define S1D13522_SET_REGAL_OFF              0x4562
#define S1D13522_GET_REGAL_STATE            0x4563
#define S1D13522_SET_REGAL_D_ON             0x4564
#define S1D13522_SET_REGAL_D_OFF            0x4565
#define S1D13522_GET_REGAL_D_STATE          0x4566

#define S1D13522_UPDATE_AREA                0x4580
typedef struct {
    int cmd;
    int waveform;
    unsigned short left;
    unsigned short top;
    unsigned short width;
    unsigned short height;
    unsigned short *imgdata;
} S1D13522_UPDATE_AREA_params_t;


// System commands
#define INIT_CMD_SET                    0x00	// Initialize instruction code (is serial flash not used)
#define INIT_PLL_STANDBY                0x01	// Init PLL and Go to Standby Mode
#define RUN_SYS                         0x02	// Go to Run Mode
#define STBY                            0x04	// Go to Standby Mode
#define SLP                             0x05	// Go to Sleep Mode
#define INIT_SYS_RUN                    0x06
//#define INIT_SYS_STBY                   0x07
//#define INIT_SDRAM                      0x08
#define INIT_DSPE_CFG                   0x09
#define INIT_DSPE_TMG                   0x0A
#define INIT_ROTMODE                    0x0B
#define INIT_WAVEDEV			0x0C

// Register and memory access commands
#define RD_REG                          0x10	// Read Register
#define WR_REG                          0x11	// Write Register

// PIP and Cursor Init Commands
#define PIP_DISABLE			0x14
#define PIP_ENABLE			0x15
#define PIP_ADRCFG			0x16
#define PIP_XYSETUP			0x17
#define CSR_MAINCFG			0x18
#define CSR_XYSETUP			0x19
#define CSR_ADRCFG			0x1A

// Burst access commands
#define BST_RD_SDR                      0x1C
#define BST_WR_SDR                      0x1D
#define BST_END_SDR                     0x1E

// Image loading commands
#define LD_IMG                          0x20
#define LD_IMG_AREA                     0x22
#define LD_IMG_END                      0x23
#define LD_IMG_WAIT                     0x24
#define LD_IMG_SETADR                   0x25
#define LD_IMG_DSPEADR                  0x26

// Polling commands
#define WAIT_DSPE_TRG                   0x28
#define WAIT_DSPE_FREND                 0x29
#define WAIT_DSPE_LUTFREE               0x2A
#define WAIT_DSPE_MLUTFREE              0x2B

// Waveform update commands
#define RD_WFM_INFO                     0x30
#define UPD_INIT                        0x32
#define UPD_FULL                        0x33
#define UPD_FULL_AREA                   0x34
#define UPD_PART                        0x35
#define UPD_PART_AREA                   0x36
#define UPD_GDRV_CLR                    0x37
#define UPD_SET_IMGADR                  0x38

#define UPD_USER                        0x50     //update mode passed via sysfs


#pragma pack(1)

typedef struct
{
        __u16 param[8];
}s1d13522_ioctl_cmd_params;

typedef struct
{
        signed short param[8];
}ioctl_cmd_par_signed;


#pragma pack()

#define S1D13522_INIT_CMD_SET           (0x4500 | INIT_CMD_SET)
#define S1D13522_INIT_PLL_STANDBY       (0x4500 | INIT_PLL_STANDBY)
#define S1D13522_RUN_SYS                (0x4500 | RUN_SYS)
#define S1D13522_STBY                   (0x4500 | STBY)
#define S1D13522_SLP                    (0x4500 | SLP)
#define S1D13522_INIT_SYS_RUN           (0x4500 | INIT_SYS_RUN)
//#define S1D13522_INIT_SYS_STBY          (0x4500 | INIT_SYS_STBY)
//#define S1D13522_INIT_SDRAM             (0x4500 | INIT_SDRAM)
#define S1D13522_INIT_DSPE_CFG          (0x4500 | INIT_DSPE_CFG)
#define S1D13522_INIT_DSPE_TMG          (0x4500 | INIT_DSPE_TMG)
#define S1D13522_INIT_ROTMODE           (0x4500 | INIT_ROTMODE)
#define S1D13522_INIT_WAVEDEV		(0x4500 | INIT_WAVEDEV)

#define S1D13522_RD_REG                 (0x4500 | RD_REG)
#define S1D13522_WR_REG                 (0x4500 | WR_REG)

#define S1D13522_PIP_DISABLE		(0x4500 | PIP_DISABLE)
#define S1D13522_PIP_ENABLE		(0x4500 | PIP_ENABLE)
#define S1D13522_PIP_ADRCFG		(0x4500 | PIP_ADRCFG)
#define S1D13522_PIP_XYSETUP		(0x4500 | PIP_XYSETUP)
#define S1D13522_CSR_MAINCFG		(0x4500 | CSR_MAINCFG)
#define S1D13522_CSR_XYSETUP		(0x4500 | CSR_XYSETUP)
#define S1D13522_CSR_ADRCFG		(0x4500 | CSR_ADRCFG)

// Burst access commands
#define S1D13522_BST_RD_SDR             (0x4500 | BST_RD_SDR)
#define S1D13522_BST_WR_SDR             (0x4500 | BST_WR_SDR)
#define S1D13522_BST_END_SDR            (0x4500 | BST_END_SDR)

// Image loading IOCTL commands
#define S1D13522_LD_IMG                 (0x4500 | LD_IMG)
#define S1D13522_LD_IMG_AREA            (0x4500 | LD_IMG_AREA)
#define S1D13522_LD_IMG_END             (0x4500 | LD_IMG_END)
#define S1D13522_LD_IMG_WAIT            (0x4500 | LD_IMG_WAIT)
#define S1D13522_LD_IMG_SETADR          (0x4500 | LD_IMG_SETADR)
#define S1D13522_LD_IMG_DSPEADR         (0x4500 | LD_IMG_DSPEADR)

// Polling commands
#define S1D13522_WAIT_DSPE_TRG          (0x4500 | WAIT_DSPE_TRG)
#define S1D13522_WAIT_DSPE_FREND        (0x4500 | WAIT_DSPE_FREND)
#define S1D13522_WAIT_DSPE_LUTFREE      (0x4500 | WAIT_DSPE_LUTFREE)
#define S1D13522_WAIT_DSPE_MLUTFREE     (0x4500 | WAIT_DSPE_MLUTFREE)

// Waveform update IOCTL commands
#define S1D13522_RD_WFM_INFO            (0x4500 | RD_WFM_INFO)
#define S1D13522_UPD_INIT               (0x4500 | UPD_INIT)
#define S1D13522_UPD_FULL               (0x4500 | UPD_FULL)
#define S1D13522_UPD_FULL_AREA          (0x4500 | UPD_FULL_AREA)
#define S1D13522_UPD_PART               (0x4500 | UPD_PART)
#define S1D13522_UPD_PART_AREA          (0x4500 | UPD_PART_AREA)
#define S1D13522_UPD_GDRV_CLR           (0x4500 | UPD_GDRV_CLR)
#define S1D13522_UPD_SET_IMGADR         (0x4500 | UPD_SET_IMGADR)


#endif // S1D13522_IOCTL_H

