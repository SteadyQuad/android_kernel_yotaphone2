//-----------------------------------------------------------------------------
//
// linux/drivers/video/epson-13522/s1d13522proc.c -- proc handling for frame buffer
// driver for Epson S1D13522 LCD controller.
//
// Copyright(c) Seiko Epson Corporation 2009.
// All rights reserved.
//
// This file is subject to the terms and conditions of the GNU General Public
// License. See the file COPYING in the main directory of this archive for
// more details.
//
//----------------------------------------------------------------------------

#ifdef CONFIG_FB_EPSON_PROC

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include "s1d13522fb.h"

//-----------------------------------------------------------------------------
// Local Globals
//-----------------------------------------------------------------------------
static struct proc_dir_entry *s1d13522fb_dir;
static struct proc_dir_entry *info_file;
static struct proc_dir_entry *regio_file;
static struct proc_dir_entry *pmicreg_file;
static struct proc_dir_entry *ft_file;
static struct proc_dir_entry *init_file;
static unsigned long ProcRegIndex = 2;	// Default to chip version reg
static unsigned Cmd = 0;
static unsigned Argnum = 0;
static u16 Args[5];

static struct proc_dir_entry *cmd_file;
static struct proc_dir_entry *memdump_file;
static unsigned long ProcDumpAddr = 0;
static unsigned long ProcDumpCount = 0;
//static struct s1d13522fb_par* gs1d13522par;

#ifdef CONFIG_FB_EPSON_GPIO_GUMSTIX
extern void init_gpio(void);
extern int dump_gpio(char *buf);
#endif

static int cmd_open(struct inode *inode, struct file *file);
static int cmd_show(struct seq_file *seq, void *offset);
static ssize_t cmd_write(struct file *file, const char __user *buf,size_t length, loff_t *ppos);
static int memdump_open(struct inode *inode, struct file *file);
static int memdump_show(struct seq_file *seq, void *offset);
static ssize_t memdump_write(struct file *file, const char __user *buf,size_t length, loff_t *ppos);

static int info_seq_show(struct seq_file *seq, void *offset)
{
	seq_printf(seq,"%s\n"
#ifdef CONFIG_FB_EPSON_VIRTUAL_FRAMEBUFFER
			"Virtual Framebuffer Frequency: %dHz\n\n"
#endif
			"Syntax when writing to reg:  [index=hex] [val=hex]\n"
			"To read a register, only set index=hex and then read from reg.\n"
			"For example, to read register 0xAB:\n"
			"   echo index=AB > /proc/s1d13522fb/regio\n"
			"   cat /proc/s1d13522fb/regio\n\n",
			s1d13522fb_version
#ifdef CONFIG_FB_EPSON_VIRTUAL_FRAMEBUFFER
			,CONFIG_FB_EPSON_VIRTUAL_FRAMEBUFFER_FREQ
#endif
			);
	 return 0;
}

static int
info_open_fs(struct inode *inode, struct file *file)
{
	return single_open(file, info_seq_show, PDE(inode)->data);
}

static const struct file_operations s1d13522fb_info_fops = {
	.owner = THIS_MODULE,
	.open = info_open_fs,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
 };

static int ft_seq_show(struct seq_file *seq, void *offset)
{
	seq_printf(seq,"s1d13522fb_do_refresh_display(UPD_FULL,WF_MODE_GC16)\n\n");
	s1d13522fb_do_refresh_display(UPD_FULL,WF_MODE_GC16);
	return 0;
}

static int
ft_open_fs(struct inode *inode, struct file *file)
{
	return single_open(file, ft_seq_show, PDE(inode)->data);
}

static const struct file_operations s1d13522fb_ft_fops = {
	.owner = THIS_MODULE,
	.open = ft_open_fs,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int init_seq_show(struct seq_file *seq, void *offset)
{
#ifdef CONFIG_FB_EPSON_GPIO_GUMSTIX
		init_gpio();
#endif
#ifdef CONFIG_FB_EPSON_USB
		s1d13522fb_init();
#endif
	return 0;
}

static int
init_open_fs(struct inode *inode, struct file *file)
{
	return single_open(file, init_seq_show, PDE(inode)->data);
}

static const struct file_operations s1d13522fb_init_fops = {
	.owner = THIS_MODULE,
	.open = init_open_fs,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int regio_seq_show(struct seq_file *seq, void *offset)
{
	unsigned val16 = s1d13522if_ReadReg16(ProcRegIndex);
	seq_printf(seq, "Register I/O: REG[%0lXh]=%04Xh\n\n",
			ProcRegIndex, val16);
	return 0;
}

/*
 * echo index=hex > /proc/s1d13522fb/regio
 * echo index=hex val=hex > /proc/s1d13522fb/regio
 */

static ssize_t
regio_write(struct file *file, const char __user *buf,
                 size_t length, loff_t *ppos)
{
	char buffer[512];
	unsigned val,index;

	if (!buf || length > sizeof(buffer) - 1)
		return -EINVAL;

	if (copy_from_user(buffer, buf, length))
		return -EFAULT;

	buffer[length] = '\0';

	if (sscanf(buffer,"index=%x val=%x",&index,&val) == 2) {
		ProcRegIndex = index;
		s1d13522if_WriteReg16(ProcRegIndex,val);
	} else if (sscanf(buffer,"index=%x",&index) == 1) {
		ProcRegIndex = index;
	}

	return length;
}

static int
regio_open_fs(struct inode *inode, struct file *file)
{
	return single_open(file, regio_seq_show, PDE(inode)->data);
}


static const struct file_operations s1d13522fb_regio_fops = {
	.owner = THIS_MODULE,
	.open = regio_open_fs,
	.write = regio_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

//-----------------------------------------------------------------------------
// /proc/s1d13522fb/cmd
// Provides the cmd proc function which allows commands to the sent to the S1D13522
//
// Read Syntax:
//      echo index=hex > /proc/s1d13522fb/cmd
//      cat /proc/s1d13522fb/cmd
//
// Write Syntax:
//
//      echo cmd=hex argnum=hex arg1=hex arg2=hex arg3=hex arg4=hex arg5=hex > /proc/s1d13522fb/cmd
//
//-----------------------------------------------------------------------------
static int cmd_show(struct seq_file *seq, void *offset)
{
        seq_printf(seq, "Not Supported\n\n");
        return 0;
}

static int cmd_open(struct inode *inode, struct file *file)
{
        return single_open(file, cmd_show, PDE(inode)->data);
}


static ssize_t cmd_write(struct file *file, const char __user *buf, size_t length, loff_t *ppos)
{
        char buffer[512];
        unsigned command,argc, a0, a1, a2, a3, a4;
	s1d13522_ioctl_cmd_params cmd_params;

        if (!buf || length > sizeof(buffer) - 1)
                return -EINVAL;

        if (copy_from_user(buffer, buf, length))
                return -EFAULT;

        buffer[length] = '\0';

        sscanf(buffer, "cmd=%x argnum=%x arg1=%x arg2=%x arg3=%x arg4=%x arg5=%x", &command, &argc, &a0, &a1, &a2, &a3, &a4);

        Cmd = command;
        Argnum = argc;
        Args[0] = a0;
        Args[1] = a1;
        Args[2] = a2;
        Args[3] = a3;
        Args[4] = a4;
	cmd_params.param[0] = a0;
	cmd_params.param[1] = a1;
	cmd_params.param[2] = a2;
	cmd_params.param[3] = a3;
	cmd_params.param[4] = a4;
      //  if (Argnum)
                //s1d13522_send_cmdargs(gs1d13522par, Cmd, Argnum, Args);
		s1d13522if_cmd(Cmd,&cmd_params,Argnum);
//        else
//                s1d13522_send_command(gs1d13522par, Cmd );

        return length;
}

static const struct file_operations s1d13522fb_cmd_fops = {
        .open = cmd_open,
        .write = cmd_write,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

//-----------------------------------------------------------------------------
// /proc/s1d13522fb/memdump
// Provides the memdump proc function which dumps S1D13522 memory within
// the specified range.
//
//      echo addr=hex count=hex > /proc/s1d13522fb/memdump
//      cat /proc/s1d13522fb/memdump
//
//-----------------------------------------------------------------------------
static int memdump_open(struct inode *inode, struct file *file)
{
        return single_open(file, memdump_show, PDE(inode)->data);
}


static int memdump_show(struct seq_file *seq, void *offset)
{
        unsigned long tempaddr = ProcDumpAddr;
        unsigned long tempcount = ProcDumpCount;
	s1d13522_ioctl_cmd_params cmd_params;
        u16 args[5];
        int i;
        u16 data;

        seq_printf(seq, "S1D13522 Memory Dump:\n");
        seq_printf(seq, "  Addr=%0lXh, 16-bit count=%0lXh\n", ProcDumpAddr, ProcDumpCount);

        //setup burst read command
        args[0] = (u16)tempaddr;
        args[1] = (u16)(tempaddr>>16);
        args[2] = (u16)(tempcount);
        args[3] = (u16)(tempcount >> 16);
	cmd_params.param[0] =(u16) tempaddr;
	cmd_params.param[1] = (u16) (tempaddr>>16);
	cmd_params.param[2] = (u16)(tempcount);
	cmd_params.param[3] =(u16)	(tempcount >> 16);

        //s1d13522_send_cmdargs(gs1d13522par, S1D13522_CMD_BST_RD_SDR, 4, args);
//s1d13522if_cmd(Cmd,&cmd_params,Argnum);
        s1d13522if_cmd(0x1C, &cmd_params, 4) ;

        //read the data
//        mutex_lock(&(gs1d13522par->io_lock));
        for (i = 0; i < tempcount; i++) {
                //data = s1d13522_read_reg(gs1d13522par, 0x154);
                data = s1d13522if_ReadReg16(0x154);
                seq_printf(seq, "  Address: %0lXh\t%04X\n", tempaddr + (i*2), data);
        }
  //      mutex_unlock(&(gs1d13522par->io_lock));
        seq_printf(seq, "\n");

        //wait for the end
        //s1d13522_send_cmdargs(gs1d13522par, S1D13522_CMD_BST_END_SDR, 0, args);
        s1d13522if_cmd(0x1E, &cmd_params,0);

        return 0;
}

static ssize_t memdump_write(struct file *file, const char __user *buf, size_t length, loff_t *ppos)
{
        char buffer[512];
        unsigned addr, count;

        if (!buf || length > sizeof(buffer) - 1)
                return -EINVAL;

        if (copy_from_user(buffer, buf, length))
                return -EFAULT;

        buffer[length] = '\0';

        if (sscanf(buffer, "addr=%x count=%x", &addr, &count) == 2) {
                ProcDumpAddr = addr;
                ProcDumpCount = count;
                //printk("Got addr then count\n\n");
        } else if (sscanf(buffer, "count=%x addr=%x", &count, &addr) == 2) {
                ProcDumpAddr = addr;
                ProcDumpCount = count;
                //printf("Got count then addr\n\n");
        } else {
                printk("To perform a S1D13522 Memory Dump:\n");
                printk("   echo addr=hex count=hex> /proc/s1d13522fb/memdump\n\n");
        }

        return length;
}


static const struct file_operations s1d13522fb_memdump_fops = {
        .open = memdump_open,
        .write = memdump_write,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};

static int pmicreg_seq_show(struct seq_file *seq, void *offset)
{
        unsigned val16 = s1d13522if_PMIC_Read_Reg(ProcRegIndex);
        seq_printf(seq, "PMIC Register I/O: REG[%0lXh]=%04Xh\n\n",
                        ProcRegIndex, val16);
        return 0;
}

/*
 * echo index=hex > /proc/s1d13522fb/regio
 * echo index=hex val=hex > /proc/s1d13522fb/regio
 */

static ssize_t
pmicreg_write(struct file *file, const char __user *buf,
                 size_t length, loff_t *ppos)
{
        char buffer[512];
        unsigned val,index;

        if (!buf || length > sizeof(buffer) - 1)
                return -EINVAL;

        if (copy_from_user(buffer, buf, length))
                return -EFAULT;

        buffer[length] = '\0';

        if (sscanf(buffer,"index=%x val=%x",&index,&val) == 2) {
                ProcRegIndex = index;
                s1d13522if_PMIC_Write_Reg(ProcRegIndex,val);
        } else if (sscanf(buffer,"index=%x",&index) == 1) {
                ProcRegIndex = index;
        }

        return length;
}
static int
pmicreg_open_fs(struct inode *inode, struct file *file)
{
        return single_open(file, pmicreg_seq_show, PDE(inode)->data);
}


static const struct file_operations s1d13522fb_pmicreg_fops = {
        .owner = THIS_MODULE,
        .open = pmicreg_open_fs,
        .write = pmicreg_write,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};


//-----------------------------------------------------------------------------
// /proc setup
//-----------------------------------------------------------------------------
int s1d13522proc_init(void)
{
	// First setup a subdirectory for s1d13522fb
	s1d13522fb_dir = proc_mkdir("s1d13522fb", NULL);

	if (!s1d13522fb_dir)
		return -ENOMEM;

	/* 'info' [R] */
	info_file =  proc_create_data("info", S_IRUGO, s1d13522fb_dir,
				  &s1d13522fb_info_fops,NULL);

	if (info_file == NULL)
		 goto err_remove_post_info;

	/* 'init' [R] */
	init_file =  proc_create_data("init", S_IRUGO, s1d13522fb_dir,
				  &s1d13522fb_init_fops,NULL);

	if (init_file == NULL)
		 goto err_remove_post_info;

	/* 'ft' [R] */
	ft_file =  proc_create_data("ft", S_IRUGO, s1d13522fb_dir,
				  &s1d13522fb_ft_fops,NULL);

	if (ft_file == NULL)
		 goto err_remove_post_info;

	/* 'regio' [R/W] */
	regio_file =  proc_create_data("regio", S_IFREG | S_IRUGO | S_IWUSR, s1d13522fb_dir,
				  &s1d13522fb_regio_fops,NULL);

	if (regio_file == NULL)
		 goto err_remove_post_info;

        /* 'pmicreg' [R/W] */
        pmicreg_file =  proc_create_data("pmicreg", S_IFREG | S_IRUGO | S_IWUSR, s1d13522fb_dir,
                                  &s1d13522fb_pmicreg_fops,NULL);

        if (pmicreg_file == NULL)
                 goto err_remove_post_info;


        /* 'cmd' [R/W] */
        cmd_file =  proc_create_data("cmd", S_IFREG | S_IRUGO | S_IWUSR, s1d13522fb_dir, &s1d13522fb_cmd_fops, NULL);
        if (cmd_file == NULL)
                 goto err_remove_post_info;


        /* 'memdump' [R/W] */
        memdump_file =  proc_create_data("memdump", S_IFREG | S_IRUGO | S_IWUSR, s1d13522fb_dir, &s1d13522fb_memdump_fops, NULL);
        if (memdump_file == NULL)
                 goto err_remove_post_info;


	return 0;

err_remove_post_info:;
	s1d13522proc_terminate();
	return -ENOMEM;
}

//-----------------------------------------------------------------------------
//
//-----------------------------------------------------------------------------
void s1d13522proc_terminate(void)
{
	if (s1d13522fb_dir) {
		if (init_file)
			remove_proc_entry("init",s1d13522fb_dir);
		if (info_file)
			remove_proc_entry("info",s1d13522fb_dir);
		if (ft_file)
			remove_proc_entry("ft",s1d13522fb_dir);
		if (regio_file)
			remove_proc_entry("regio",s1d13522fb_dir);
		remove_proc_entry("s1d13522fb",NULL);
	}

	s1d13522fb_dir = ft_file = info_file = init_file = regio_file = NULL;
}

#endif // CONFIG_FB_EPSON_PROC

