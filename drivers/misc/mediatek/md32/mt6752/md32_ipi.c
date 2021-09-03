/*
* Copyright (C) 2015 MediaTek Inc.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <asm/barrier.h>
#include <mach/mt_reg_base.h>
#include <mach/md32_helper.h>
#include <mach/md32_ipi.h>
#include <md32_irq.h>


struct ipi_desc ipi_desc[MD32_NR_IPI];
struct share_obj *md32_send_obj, *md32_rcv_obj;
struct mutex md32_ipi_mutex;

extern unsigned char *md32_send_buff;
extern unsigned char *md32_recv_buff;

/*md32 reload workaround*/
#define MD32_REBOOT

#ifdef MD32_REBOOT
#define MD32_TCM_CFG	(MD32_BASE+0x08)
#define MD32_CLK_SRAM_PDN	(MD32_CLK_CTRL_BASE + 0x02C)
#define MD32_CLK_CG_CRTL	(MD32_CLK_CTRL_BASE + 0x030)
#define MD32_REBOOT_THRESHOLD 50000000 /*default 5 sec*/

#include <linux/vmalloc.h>		/* needed by vmalloc */
#include <linux/slab.h>			/* needed by vmalloc */
#include <linux/fs.h>			/* needed by file_operations* */
#include <linux/module.h>		/* needed by all modules */
#include <linux/init.h>			/* needed by module macros */
#include <linux/miscdevice.h>	/* needed by miscdevice* */
#include <linux/uaccess.h>		/* needed by copy_to_user */
#include <linux/workqueue.h>
#include <mach/sync_write.h>
#include <mach/wd_api.h>
struct wd_api *md32_wd_api = NULL;
static struct work_struct work_md32_reload;
static struct workqueue_struct *wq_md32_reload;
#endif

#define MD32_IPI_PRINT_THRESHOLD 10000000 /*default 1 sec*/
unsigned char md32_init_count = 0;
unsigned int md32_timeout = 0;
int md32_reboot_flag = 0;



#ifdef MD32_REBOOT
int reload_md32(const char *IMAGE_PATH, void *dst)
{
	struct file *filp = NULL;
	unsigned char *buf = NULL;
	struct inode *inode;
	off_t fsize;
	mm_segment_t fs;

	pr_debug("[MD32 RELOAD] Open MD32 image %s\n", IMAGE_PATH);
	filp = filp_open(IMAGE_PATH, O_RDONLY, 0644);

	if (IS_ERR(filp)) {
		pr_debug("[MD32 RELOAD] Open MD32 image %s FAIL!\n", IMAGE_PATH);
		filp_close(filp, NULL);
		return -1;
	} else {
		inode = filp->f_dentry->d_inode;
		fsize = inode->i_size;
		pr_debug("[MD32 RELOAD] file %s size: %i\n", IMAGE_PATH, (int)fsize);
		buf = vmalloc((size_t) fsize + 1);
		if (!buf) {
			pr_debug("[MD32 RELOAD] boot up failed, can't allocate memory,buf\n");
			vfree(buf);
			return -1;
		}
		fs = get_fs();
		set_fs(KERNEL_DS);
		filp->f_op->read(filp, buf, fsize, &(filp->f_pos));
		set_fs(fs);
		buf[fsize] = '\0';
		memcpy(dst, buf, fsize);
		filp_close(filp, NULL);
		vfree(buf);
		return fsize;
	}
}

int reload_get_md32_img_sz(const char *IMAGE_PATH)
{
	struct file *filp = NULL;
	struct inode *inode;
	off_t fsize = 0;
	pr_debug("[MD32 RELOAD] Open MD32 image %s\n", IMAGE_PATH);
	filp = filp_open(IMAGE_PATH, O_RDONLY, 0644);

	if (IS_ERR(filp)) {
		pr_debug("[MD32 RELOAD] Open MD32 image %s FAIL!\n", IMAGE_PATH);
		filp_close(filp, NULL);
		return -1;
	} else {
		inode = filp->f_dentry->d_inode;
		fsize = inode->i_size;
		filp_close(filp, NULL);
		return fsize;
	}
}

void reload_boot_up_md32(void)
{
	mt_reg_sync_writel(0x1, MD32_BASE);
}

int reload_md32_program(void)
{
	int i;
	unsigned int sw_rstn;
	int ret = 0;
	int d_sz, p_sz;
	int d_num, p_num;
	volatile unsigned int reg;

	sw_rstn = readl(MD32_BASE);

	if (sw_rstn == 0x1)
		pr_debug("[MD32 RELOAD] reboot now...\n");

	/* reset MD32 */
	pr_debug("[MD32 RELOAD] reset MD32\n");
	mt_reg_sync_writel(0x0, MD32_BASE);
	/* enable MD32 peripheral CG*/
	mt_reg_sync_writel(0x1FF, MD32_CLK_CG_CRTL);
	/* MD32 TCM settings*/
	mt_reg_sync_writel(0x1, MD32_TCM_CFG);

	/* MD32 TCM power on*/
	reg = readl(MD32_CLK_SRAM_PDN);
	for (i = 0; i < 7; ++i) {
		reg &= ~(0x1 << i);
		mt_reg_sync_writel(reg, MD32_CLK_SRAM_PDN);
	}
	pr_debug("[MD32 RELOAD] load dmem\n");
	d_sz = reload_get_md32_img_sz(MD32_DATA_IMAGE_PATH);
	if (d_sz < 0) {
		pr_debug("[MD32 RELOAD] boot up failed, can not get data image size\n");
		ret = -1;
		goto error;
	}
	d_sz = ((d_sz + 63) >> 6) << 6;

	pr_debug("[MD32 RELOAD] reload dmem\n");
	ret = reload_md32(MD32_DATA_IMAGE_PATH, md32_data_image);
	if (ret < 0) {
		pr_debug("[MD32 RELOAD] boot up failed, load data image failed!\n");
		ret = -1;
		goto error;
	}

	if (d_sz > MD32_DTCM_SIZE)
		d_sz = MD32_DTCM_SIZE;

	memcpy(MD32_DTCM, md32_data_image, d_sz);
	pr_debug("[MD32 RELOAD] reload dmem done...\n");
	p_sz = reload_get_md32_img_sz(MD32_PROGRAM_IMAGE_PATH);
	if (p_sz < 0) {
		pr_debug("[MD32 RELOAD]boot up failed, can not get program image size\n");
		ret = -1;
		goto error;
	}
	p_sz = ((p_sz + 63) >> 6) << 6;

	ret = reload_md32(MD32_PROGRAM_IMAGE_PATH, md32_program_image);
	if (ret < 0) {
		pr_debug("[MD32 RELOAD] boot up failed, load program image failed!\n");
		ret = -1;
		goto error;
	}

	if (p_sz > MD32_PTCM_SIZE)
		p_sz = MD32_PTCM_SIZE;

	memcpy(MD32_PTCM, md32_program_image, p_sz);
	pr_debug("[MD32 RELOAD] MD32 reload pmem done...\n");

	/* power off useless TCM */
	p_num = (p_sz / (32*1024)) + 1;
	reg = readl(MD32_CLK_SRAM_PDN);
	for (i = p_num; i < 3; ++i) {
		reg |= (0x1 << i);
		mt_reg_sync_writel(reg, MD32_CLK_SRAM_PDN);
	}
	/* power off useless TCM */
	d_num = (d_sz / (32*1024)) + 1;
	reg = readl(MD32_CLK_SRAM_PDN);
	for (i = 6-d_num; i > 2; --i) {
		reg |= (0x1 << i);
		mt_reg_sync_writel(reg, MD32_CLK_SRAM_PDN);
	}

	reload_boot_up_md32();

	return ret;

error:
	pr_debug("[MD32 RELOAD] boot up failed!!! try to reload md32\n");
	queue_work(wq_md32_reload, &work_md32_reload);

	return ret;
}
#endif

static void ipi_md2host(void)
{
	HOST_TO_MD32_REG = 0x1;
}

void md32_ipi_handler(void)
{
	pr_debug("md32_ipi_handler = %d, init = %d\n", md32_rcv_obj->id, md32_init_count);
	if (ipi_desc[md32_rcv_obj->id].handler && md32_rcv_obj->id < MD32_NR_IPI) {
		memcpy_from_md32(md32_recv_buff, (void *)md32_rcv_obj->share_buf,
				 md32_rcv_obj->len);
		ipi_desc[md32_rcv_obj->id].handler(md32_rcv_obj->id, md32_recv_buff,
						   md32_rcv_obj->len);
	} else {
		pr_err("[MD32]md32 ipi ID abnormal or handler is null\n");
		pr_err("[MD32]md32_rcv_obj address = 0x%p\n", md32_rcv_obj);
		pr_err("[MD32]md32_send_obj address = 0x%p\n", md32_send_obj);
	}
	MD32_TO_SPM_REG = 0x0;
	pr_debug("md32_ipi_handler %d done\n", md32_rcv_obj->id);

#ifdef MD32_REBOOT
	if (IPI_LOGGER == md32_rcv_obj->id) {
		md32_init_count++;
		md32_reboot_flag++;
		if (md32_reboot_flag > 1) {
			md32_reboot_flag = 0;
			pr_debug("[MD32 RELOAD]starting reload wq...\n");
			queue_work(wq_md32_reload, &work_md32_reload);
		}
	}
#endif
}

void md32_ipi_init(void)
{
	mutex_init(&md32_ipi_mutex);
	md32_rcv_obj = MD32_DTCM;
	md32_send_obj = md32_rcv_obj + 1;
	pr_debug("md32_rcv_obj = 0x%p\n", md32_rcv_obj);
	pr_debug("md32_send_obj = 0x%p\n", md32_send_obj);
	memset(md32_send_obj, 0, SHARE_BUF_SIZE);
#ifdef MD32_REBOOT
	wq_md32_reload = create_workqueue("MD32_RELOAD_WQ");
	INIT_WORK(&work_md32_reload, reload_md32_program);
#endif
}

/*
  @param id:	   IPI ID
  @param handler:  IPI handler
  @param name:	 IPI name
*/
ipi_status md32_ipi_registration(ipi_id id, ipi_handler_t handler, const char *name)
{
	if (id < MD32_NR_IPI) {
		ipi_desc[id].name = name;

		if (handler == NULL)
			return ERROR;

		ipi_desc[id].handler = handler;
		return DONE;
	} else {
		return ERROR;
	}
}

/*
  @param id:	   IPI ID
  @param buf:	  the pointer of data
  @param len:	  data length
  @param wait:	 If true, wait (atomically) until data have been gotten by Host
*/
ipi_status md32_ipi_send(ipi_id id, void *buf, unsigned int len, unsigned int wait)
{
	unsigned int sw_rstn;
	sw_rstn = readl(MD32_BASE);
	if (sw_rstn == 0x0) {
		pr_debug("md32_ipi_send: MD32 not enabled, ipi id = %d\n", id);
		return ERROR;
	}

	if (id < MD32_NR_IPI) {
		if (len > sizeof(md32_send_obj->share_buf) || buf == NULL)
			return ERROR;

		if (HOST_TO_MD32_REG)
			return BUSY;

		mutex_lock(&md32_ipi_mutex);


		if (HOST_TO_MD32_REG) {
			mutex_unlock(&md32_ipi_mutex);
			pr_debug("md32_ipi_send: HOST_TO_MD32_REG busy, ipi id = %d\n", id);
			return BUSY;
		}

		memcpy(md32_send_buff, buf, len);

		memcpy_to_md32((void *)md32_send_obj->share_buf, md32_send_buff, len);
		md32_send_obj->len = len;
		md32_send_obj->id = id;
		dsb();
		ipi_md2host();

		if (wait) {
			md32_timeout = 0;
			while (HOST_TO_MD32_REG) {
				md32_timeout++;
				ndelay(1);
				if (md32_timeout % MD32_IPI_PRINT_THRESHOLD == 0)
					pr_debug("md32_ipi_send: MD32 time out, ipi id = %d\n", id);
#ifdef MD32_REBOOT
				if ((md32_timeout >= MD32_REBOOT_THRESHOLD) &&
					(md32_timeout % MD32_REBOOT_THRESHOLD == 0)) {
					pr_debug("md32_ipi_send: MD32 time out, starting reload wq...\n");
					queue_work(wq_md32_reload, &work_md32_reload);
				}
#endif
			}
		}
		mutex_unlock(&md32_ipi_mutex);
	} else
		return ERROR;

	return DONE;
}
