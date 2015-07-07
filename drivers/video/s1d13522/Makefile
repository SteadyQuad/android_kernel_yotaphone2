#
# Makefile for the Epson s1d13522 framebuffer device driver
#

# DEFIO
#obj-$(CONFIG_FB_EPSON_S1D13522)			+= s1d13522fb.o
#obj-y			+= s1d13522fb.o
#s1d13522fb-objs					+= s1d13522base.o
#obj-y					+= s1d13522base.o
# Ends here

#DEFIO
obj-$(CONFIG_FB_EPSON_S1D13522)		+= epson13522fb.o
epson13522fb-objs			+= s1d13522fb.o
epson13522fb-objs			+= s1d13522fb_regal.o
#DEFIO

epson13522fb-$(CONFIG_FB_EPSON_INDIRECT_INTERFACE)+= s1d13522if.o
epson13522fb-$(CONFIG_FB_EPSON_PCI) 		+= s1d13522pci.o
#epson13522fb-$(CONFIG_FB_EPSON_PROC) 		+= s1d13522proc.o
epson13522fb-$(CONFIG_FB_EPSON_GPIO_GUMSTIX)	+= s1d13522ifgpio.o
epson13522fb-$(CONFIG_FB_EPSON_USB) 		+= s1d13522usb.o
#s1d13522fb-y			 		+= s1d13522spi.o
epson13522fb-$(CONFIG_FB_EPSON_SPI) 		+= s1d13522spi.o
#EXTRA_CFLAGS					+= -D'S1D13xxxFB_INCLUDE="s1d13522.h"'

