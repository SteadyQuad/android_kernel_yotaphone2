//-----------------------------------------------------------------------------
// linux/drivers/video/epson-13522/s1d13522fb_panels.h
//
// This is the panel information header for the Epson S1D13522 linux frame buffer driver.
//
// Copyright(C) SEIKO EPSON CORPORATION 2011-2013. All rights reserved.
//
// This driver software is distributed as is, without any warranty of any kind,
// either express or implied as further specified in the GNU Public License. This
// software may be used and distributed according to the terms of the GNU Public
// License, version 2. See the file COPYING in the main directory of this archive
// for more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.
//
//----------------------------------------------------------------------------
//
// NOTE: All chip-specific features, such as hardware information needed
//       to generate proper timings, color depth, screen resolution, etc.,
//       are placed in chip-specific header files and C modules.
//
//-----------------------------------------------------------------------------

#ifndef __S1D13522FB_PANELS_H__
#define __S1D13522FB_PANELS_H__


// struct used to track panel specific parameters
struct panel_info {
	int w;
	int h;
	u16 sdcfg;
	u16 gdcfg;
	u16 lutfmt;
	u16 fsynclen;
	u16 fendfbegin;
	u16 lsynclen;
	u16 lendlbegin;
	u16 pixclk;
};

// table of panel specific parameters to be indexed into by the board drivers
// the desired panel can be indexed using the S1D13522_PANEL_INDEX define
static struct panel_info panel_table[] = {
	{	/* 0 standard 6" on TFT backplane 26MHz clock */
		.w = 800,
		.h = 600,
		.sdcfg = (100 | (1 << 8) | (1 << 9)),
		.gdcfg = 2,
		.lutfmt = (4 | (1 << 7)),
		.fsynclen = 4,
		.fendfbegin = (10 << 8) | 4,
		.lsynclen = 10,
		.lendlbegin = (87 << 8) | 4,
		.pixclk = 6,
	},
	{	/* 1 custom 3.7" flexible on PET or steel */
		.w = 320,
		.h = 240,
		.sdcfg = (67 | (0 << 8) | (0 << 9) | (0 << 10) | (0 << 12)),
		.gdcfg = 3,
		.lutfmt = (4 | (1 << 7)),
		.fsynclen = 0,
		.fendfbegin = (80 << 8) | 4,
		.lsynclen = 10,
		.lendlbegin = (80 << 8) | 20,
		.pixclk = 14,
	},
	{	/* 2 standard 9.7" on TFT backplane 26MHz clock */
		.w = 1200,
		.h = 825,
		.sdcfg = (100 | (1 << 8) | (1 << 9) | (0 << 10) | (0 << 12)),
		.gdcfg = 2,
		.lutfmt = (4 | (1 << 7)),
		.fsynclen = 4,
		.fendfbegin = (9 << 8) | 4,
		.lsynclen = 10,
		.lendlbegin = (69 << 8) | 6,
		.pixclk = 3,
	},
	{	/* 3 custom panel to be tweaked thru cmd line */
		.w = 123,
		.h = 456,
		.sdcfg = (100 | (1 << 8) | (1 << 9) | (0 << 10) | (0 << 12)),
		.gdcfg = 2,
		.lutfmt = (4 | (1 << 7)),
		.fsynclen = 0,
		.fendfbegin = (4 << 8) | 4,
		.lsynclen = 4,
		.lendlbegin = (60 << 8) | 10,
		.pixclk = 3,
	},
	{	/* 4 standard 9.7" on TFT backplane 25MHz clock */
		.w = 1200,
		.h = 825,
		.sdcfg = (100 | (1 << 8) | (1 << 9) | (0 << 10) | (0 << 12)),
		.gdcfg = 2,
		.lutfmt = (4 | (1 << 7)),
		.fsynclen = 0,
		.fendfbegin = (4 << 8) | 4,
		.lsynclen = 4,
		.lendlbegin = (60 << 8) | 10,
		.pixclk = 3,
	},
	{	/* 5 standard 6" on TFT backplane 25MHz clock */
		.w = 800,
		.h = 600,
		.sdcfg = (100 | (1 << 8) | (1 << 9)),
		.gdcfg = 2,
		.lutfmt = (4 | (1 << 7)),
		.fsynclen = 4,
		.fendfbegin = (10 << 8) | 4,
		.lsynclen = 10,
		.lendlbegin = (87 << 8) | 4,
		.pixclk = 6,
	},
	{	/* 6 V220 85Hz 6" on TFT backplane 26MHz clock */
		.w = 800,
		.h = 600,
		.sdcfg = (100 | (1 << 8) | (1 << 9)),
		.gdcfg = 2,
		.lutfmt = (4 | (1 << 7)),
		.fsynclen = 1,
		.fendfbegin = (7 << 8) | 4,
		.lsynclen = 2,
		.lendlbegin = (43 << 8) | 4,
		.pixclk = 4,
	},
	{	/* 7 V220 85Hz 4.7" on TFT backplane 26MHz clock */
//		.w = 960,
//		.h = 540,
		.w = 540,
		.h = 960,
		.sdcfg = (100 | (1 << 8) | (1 << 9)),
		.gdcfg = 2,
		.lutfmt = (4 | (1 << 7)),
		.fsynclen = 1,
		.fendfbegin = (7 << 8) | 4,
		.lsynclen = 2,
		.lendlbegin = (43 << 8) | 4,
		.pixclk = 4,
	}

};


#endif	//__S1D13522FB_PANELS_H__
