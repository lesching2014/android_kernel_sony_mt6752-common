/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#define DEBUG
#define NFC_DEBUG

/*****************************************************************************
** Include
******************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/device.h>
#include <linux/pn544.h>
//<2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
#include <linux/wakelock.h>
//>2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

//#include <mach/mt_devs.h>
//#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/eint.h>
#include <mach/mt_pm_ldo.h>
#include <cust_gpio_usage.h>
#include <cust_eint.h>
#include <cust_eint_md1.h>
#include <cust_i2c.h>

/*****************************************************************************
** Macro-Define
******************************************************************************/
#define NFC_CLIENT_TIMING 400
/*========================================================
** For information print out
**========================================================*/
#if defined( NFC_TRACE_ENABLE )
    #undef    NFC_TRACE_ENABLE
#endif
  #define   NFC_TRACE_ENABLE

#if defined( NFC_MSG_ENABLE )
    #undef    NFC_MSG_ENABLE
#endif
  #define   NFC_MSG_ENABLE

#if defined( NFC_DEBUG_ENABLE )
    #undef    NFC_DEBUG_ENABLE
#endif
  #define   NFC_DEBUG_ENABLE

#if defined( NFC_MSG )
    #undef    NFC_MSG
#endif
#if defined( NFC_MSG_ENABLE )
  #if defined( BUILD_LK )
    #define   NFC_MSG(srt,arg...)       dprintf(CRITICAL,srt,##arg)
  #else
    #define   NFC_MSG(srt,arg...)       printk(KERN_INFO srt,##arg)
  //#define   NFC_MSG(srt,arg...)       printk(srt,##arg)
  #endif
#else
    #define   NFC_MSG(srt,arg...)       {/* Do Nothing */}
#endif

#if defined( NFC_DBG )
    #undef    NFC_DBG
#endif
#if defined( NFC_DEBUG_ENABLE )
  #if defined( BUILD_LK )
    #define   NFC_DBG(srt,arg...)       dprintf(CRITICAL,srt,##arg)
  #else
    #define   NFC_DBG(srt,arg...)       printk(KERN_DEBUG srt,##arg)
  //#define   NFC_DBG(srt,arg...)       printk(srt,##arg)
  #endif
#else
    #define   NFC_DBG(srt,arg...)       {/* Do Nothing */}
#endif

  #if defined( BUILD_LK )
    #define   NFC_ERR(srt,arg...)       dprintf(CRITICAL,srt,##arg)
  #else
    #define   NFC_ERR(srt,arg...)       printk(KERN_ERR srt,##arg)
  #endif

#if defined( NFC_TRACE_ENABLE )
  #if defined( BUILD_LK )
    #define   NFC_TRACE(srt,arg...)     dprintf(CRITICAL,srt,##arg)
  #else
    #define   NFC_TRACE(srt,arg...)     printk(srt,##arg)
  #endif
#else
    #define   NFC_TRACE(srt,arg...)     {/* Do Nothing */}
#endif


  #if defined( I2C_DMA_USAGE )
    #undef    I2C_DMA_USAGE
  #endif
    #define   I2C_DMA_USAGE

#if defined( I2C_DMA_USAGE )
    #include  <linux/dma-mapping.h>
#endif

/*========================================================
**
**========================================================*/
  #if defined( I2C_NFC_SLAVE_7_BIT_ADDR )
    #define   PN544_DEV_ADDR            (I2C_NFC_SLAVE_7_BIT_ADDR)
  #else
    #define   PN544_DEV_ADDR            (0x28)  //(0x2B)  /* PN547C2: 0x2B, PN547: 0x28*/
  #endif
    #define   PN544_SLAVE_ADDR          (PN544_DEV_ADDR<<1)

  #if defined( I2C_NFC_CHANNEL )
    #define   I2C_BUS_ID                I2C_NFC_CHANNEL
  #else
    #define   I2C_BUS_ID                1   /* I2C1 */
  #endif
    #define   NFC_DEV_NAME              "pn544"
    #define   I2C_ID_NAME               "pn544"

    #define   MAX_BUFFER_SIZE           512
    #define   ENABLE_DELAY              50
    #define   DISABLE_DELAY             70

    #define   VEN_ENABALE               1
    #define   VEN_DISABALE              0

  #if !defined( CUST_EINT_IRQ_NFC_SENSITIVE )
    #define   CUST_EINT_IRQ_NFC_SENSITIVE   CUST_EINT_LEVEL_SENSITIVE
  #endif
  #if !defined( CUST_EINT_IRQ_NFC_POLARITY )
    #define   CUST_EINT_IRQ_NFC_POLARITY    CUST_EINT_POLARITY_HIGH
  #endif

  typedef struct st_pn544_dev
  {
    wait_queue_head_t   read_wq;
    struct mutex        read_mutex;
//    struct mutex        rw_mutex;
    struct i2c_client * client;
    struct miscdevice   pn544_device;
    //<2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    struct wake_lock wake_lock;
    //>2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    unsigned int        ven_gpio;
    unsigned int        firm_gpio;
    unsigned int        irq_gpio;
    bool                irq_enabled;
    spinlock_t          irq_enabled_lock;
  } PN544_DEV;

    static PN544_DEV  * g_pn544_dev = NULL;

/* For DMA */
  #if defined( I2C_DMA_USAGE )
    static char   * I2CDMAWriteBuf = NULL;
    static uintptr_t   I2CDMAWriteBuf_pa;  // = NULL;
    static char   * I2CDMAReadBuf = NULL;
    static uintptr_t  I2CDMAReadBuf_pa;   // = NULL;
  #endif /* End.. (I2C_DMA_USAGE) */

/*****************************************************************************
**
******************************************************************************/
static void pn544_disable_irq(PN544_DEV *pn544_dev)
{
unsigned long   flags;

    NFC_TRACE("%s +++\n", __func__);

    spin_lock_irqsave( &pn544_dev->irq_enabled_lock, flags );
    if( pn544_dev->irq_enabled )
    {
      NFC_TRACE("%s: pn544_dev->irq_enabled\n", __func__);
      mt_eint_mask( CUST_EINT_IRQ_NFC_NUM );  //disable_irq_nosync(pn544_dev->client->irq);
      pn544_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore( &pn544_dev->irq_enabled_lock, flags );

    NFC_TRACE("%s ---\n", __func__);
}

//<2014/08/14-Yuting Shih. Add the IRQ enable.
/*****************************************************************************
**
******************************************************************************/
static void pn544_enable_irq(PN544_DEV *pn544_dev)
{
unsigned long   flags;

    NFC_TRACE("%s +++\n", __func__);

    spin_lock_irqsave( &pn544_dev->irq_enabled_lock, flags );
    if( !( pn544_dev->irq_enabled ))
    {
      NFC_TRACE("%s: !( pn544_dev->irq_enabled )\n", __func__);
      mt_eint_unmask( CUST_EINT_IRQ_NFC_NUM );  //enable_irq( pn544_dev->client->irq );
      pn544_dev->irq_enabled = true;
    }
    spin_unlock_irqrestore( &pn544_dev->irq_enabled_lock, flags );

    NFC_TRACE("%s ---\n", __func__);
}
//>2014/08/14-Yuting Shih.

/*****************************************************************************
**
******************************************************************************/
#if 1
static void pn544_dev_irq_handler(void)
{
PN544_DEV * pn544_dev = g_pn544_dev;

    NFC_TRACE("%s: irq %d\n", __func__, mt_get_gpio_in( GPIO_IRQ_NFC_PIN ));

    if( NULL == pn544_dev )
    {
      NFC_ERR("pn544_dev null.\n");
      return;
    }

    if( mt_get_gpio_in( GPIO_IRQ_NFC_PIN ))  //if(gpio_get_value(pn544_dev->irq_gpio))
    {
/*
    while(1){
      if( mt_get_gpio_in( GPIO_IRQ_NFC_PIN )){
*/      
      NFC_TRACE("pn544 device has EINT.\n");

      pn544_disable_irq( pn544_dev );
    /* Wake up waiting readers */
      //wake_up( &pn544_dev->read_wq );
      wake_up_interruptible( &pn544_dev->read_wq );
/*	
        return;
      } else {
        NFC_TRACE("pn544 device has no EINT.\n");
      }
      msleep(10);
    }
*/    
    }
    else
    {
      NFC_TRACE("pn544 device no EINT.\n");
    }
}
#else
static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
PN544_DEV *pn544_dev = dev_id;

    NFC_TRACE("%s, irq: %d \n", __func__, mt_get_gpio_in( GPIO_IRQ_NFC_PIN ));

    if( mt_get_gpio_in( GPIO_IRQ_NFC_PIN ))  //if(gpio_get_value(pn544_dev->irq_gpio)) {
    {
      pn544_disable_irq( pn544_dev );
    /* Wake up waiting readers */
      //wake_up_interruptible_sync(&pn544_dev->read_wq);
      wake_up_interruptible( &pn544_dev->read_wq );
    }
    else
    {
      NFC_TRACE( "%s, no irq \n", __func__);
    }

    return  IRQ_HANDLED;
}
#endif

/*****************************************************************************
**
******************************************************************************/
static ssize_t pn544_dev_read(struct file *filp, char __user *buf,
    size_t count, loff_t *offset)
{
PN544_DEV *pn544_dev = filp->private_data;
#if !defined( I2C_DMA_USAGE )
char  tmp[MAX_BUFFER_SIZE];
#endif
int   ret = 0;
#if defined( NFC_DEBUG )
int   i;
#endif /* NFC_DEBUG */
unsigned short  addr;
__u32           ext_flag;

//    mutex_lock( &pn544_dev->rw_mutex );
    NFC_TRACE("[NFC Jazz]%s: Enter for reading %zu bytes.\n", __func__, count );

    if( count > MAX_BUFFER_SIZE )
      count = MAX_BUFFER_SIZE;

  //NFC_DBG("%s : reading %zu bytes.\n", __func__, count );

    //<2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    current->flags |= PF_NOFREEZE;
    current->flags &= ~PF_FROZEN;
    //>2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
	
    mutex_lock( &pn544_dev->read_mutex );

    NFC_TRACE("%s: Mutex Lock\n", __func__);


    if( !mt_get_gpio_in( GPIO_IRQ_NFC_PIN ))  //if (!gpio_get_value(pn544_dev->irq_gpio))
    {
      NFC_TRACE("%s: no irq, irq_gpio: %d, irq_enabled: %d\n", __func__,
            mt_get_gpio_in( GPIO_IRQ_NFC_PIN ), pn544_dev->irq_enabled );

      if( filp->f_flags & O_NONBLOCK )
      {
        ret = -EAGAIN;
        goto fail;
      }

    //pn544_enable_irq( pn544_dev );
      if( !( pn544_dev->irq_enabled ))
      {
        NFC_TRACE("%s: enable_irq\n", __func__ );

        pn544_enable_irq(pn544_dev);
        //pn544_dev->irq_enabled = true;
        //mt_eint_unmask( CUST_EINT_IRQ_NFC_NUM );  //enable_irq( pn544_dev->client->irq );
      }

      NFC_TRACE("%s: start wait!\n", __func__ );
      //<<JB
//      mutex_unlock( &pn544_dev->rw_mutex );
//      ret = wait_event_interruptible( pn544_dev->read_wq,
//                                      mt_get_gpio_in( GPIO_IRQ_NFC_PIN ));  //(gpio_get_value(pn544_dev->irq_gpio)) );
        //<2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
        //ret = wait_event_interruptible( pn544_dev->read_wq, pn544_dev->irq_enabled == false);
	  
        ret = wait_event_interruptible(pn544_dev->read_wq,(mt_get_gpio_in(GPIO_IRQ_NFC_PIN)) );
        //>2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode


      //>>JB
//      mutex_lock( &pn544_dev->rw_mutex );
      NFC_TRACE("%s, ret: 0x%X\n", __func__, ret );

      pn544_disable_irq( pn544_dev );

      if( !mt_get_gpio_in( GPIO_IRQ_NFC_PIN ))  //if( !gpio_get_value(pn544_dev->irq_gpio))
      {
        ret = -EIO;
        goto fail;
      }
    }

    addr      = pn544_dev->client->addr;
    ext_flag  = pn544_dev->client->ext_flag;

#if defined( I2C_DMA_USAGE )
  pn544_dev->client->addr &= I2C_MASK_FLAG;
  //  pn544_dev->client->addr     &= I2C_MASK_FLAG;
  //pn544_dev->client->addr     |= I2C_DMA_FLAG;
  //pn544_dev->client->addr     |= I2C_ENEXT_FLAG;
    pn544_dev->client->ext_flag |= I2C_DMA_FLAG;
  //pn544_dev->client->ext_flag |= I2C_DIRECTION_FLAG;
  //pn544_dev->client->ext_flag |= I2C_A_FILTER_MSG;
  pn544_dev->client->timing = NFC_CLIENT_TIMING;

    /* Read data */
    ret = i2c_master_recv( pn544_dev->client, (unsigned char *)I2CDMAReadBuf_pa, count );
#else
    /* Read data */
//    i = 0;
//    while (ret < 0 && i < 3) {
//        i++;
        pn544_dev->client->addr&=I2C_MASK_FLAG;
        ret = i2c_master_recv( pn544_dev->client, (unsigned char *)tmp, count );
//    }
#endif

    pn544_dev->client->addr     = addr;
    pn544_dev->client->ext_flag = ext_flag;

    mutex_unlock( &pn544_dev->read_mutex );

    NFC_TRACE("%s: Mutex unLock\n", __func__);


    if( ret < 0 )
    {
      NFC_ERR("%s: i2c_master_recv returned %d\n", __func__, ret );
//      mutex_unlock( &pn544_dev->rw_mutex );
      return ret;
    }
    if( ret > count )
    {
      NFC_ERR("%s: received too many bytes from i2c (%d)\n", __func__, ret );
//      mutex_unlock( &pn544_dev->rw_mutex );
      return -EIO;
    }

#if defined( I2C_DMA_USAGE )
    if( copy_to_user( buf, I2CDMAReadBuf, ret ))
#else
    if( copy_to_user( buf, tmp, ret ))
#endif
    {
      pr_warning("%s : failed to copy to user space.\n", __func__);
//      mutex_unlock( &pn544_dev->rw_mutex );
      return -EFAULT;
    }

#if defined( NFC_DEBUG )
    NFC_TRACE( "%s: bytes[%d] ", __func__, (int)count );
       #if defined( I2C_DMA_USAGE )
	if(I2CDMAReadBuf!=NULL)
        #else
	if(tmp!=NULL)
       #endif
	{
		for( i = 0; i < count; i++ )
		{
		#if defined( I2C_DMA_USAGE )
		  NFC_TRACE("%02X ", *(I2CDMAReadBuf + i));
		#else
		  NFC_TRACE("%02X ", tmp[i] );
		#endif
		}
	}
	else
		NFC_TRACE(" tmp==Null \n");
    NFC_TRACE(" \n");
#endif /* NFC_DEBUG */

    NFC_TRACE("%s complete, irq: %d\n", __func__, mt_get_gpio_in( GPIO_NFC_EINT_PIN ));
//    mutex_unlock( &pn544_dev->rw_mutex );

    //<2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    wake_lock_timeout(&pn544_dev->wake_lock, msecs_to_jiffies(1500));
    //>2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

    return ret;

fail:
    mutex_unlock( &pn544_dev->read_mutex );

    NFC_TRACE("%s error ---, ret: 0x%X\n", __func__, ret );
//    mutex_unlock( &pn544_dev->rw_mutex );

    return ret;
}

/*****************************************************************************
**
******************************************************************************/
static ssize_t pn544_dev_write(struct file *filp, const char __user *buf,
    size_t count, loff_t *offset)
{
#if 1
PN544_DEV * pn544_dev;
#if !defined( I2C_DMA_USAGE )
char  tmp[MAX_BUFFER_SIZE];
#endif
#if defined( NFC_DEBUG )
int   i;
#endif /* NFC_DEBUG */
int   ret;
unsigned short  addr;
__u32           ext_flag;

//    mutex_lock( &pn544_dev->rw_mutex );
    NFC_TRACE("[NFC]%s: Enter for writing %zu bytes.\n", __func__, count );

    pn544_dev = filp->private_data;

    if( count > MAX_BUFFER_SIZE )
      count = MAX_BUFFER_SIZE;

#if defined( I2C_DMA_USAGE )
    if( copy_from_user( I2CDMAWriteBuf, buf, count ))
#else
    if( copy_from_user( tmp, buf, count ))
#endif
    {
      NFC_ERR( "%s : failed to copy from user space\n", __func__ );
      return -EFAULT;
    }

  //NFC_DBG("%s : writing %zu bytes.\n", __func__, count );
#if defined( NFC_DEBUG )
    NFC_TRACE("%s: bytes[%d] ", __func__, (int)count );
       #if defined( I2C_DMA_USAGE )
	if(I2CDMAReadBuf!=NULL)
        #else
	if(tmp!=NULL)
       #endif
	{
		for( i = 0; i < count; i++ )
		{
		#if defined( I2C_DMA_USAGE )
		  NFC_TRACE("%02X ", *(I2CDMAWriteBuf + i));
		#else
		  NFC_TRACE("%02X ", tmp[i] );
		#endif
		}
	}else
		  NFC_TRACE(" temp==NULL\n");
    NFC_TRACE(" \n");
#endif /* NFC_DEBUG */

    addr      = pn544_dev->client->addr;
    ext_flag  = pn544_dev->client->ext_flag;
	NFC_TRACE("%02X ",addr);
#if defined( I2C_DMA_USAGE )
   // pn544_dev->client->addr     &= I2C_MASK_FLAG;
    pn544_dev->client->addr     |= I2C_DMA_FLAG;
  //pn544_dev->client->addr     |= I2C_ENEXT_FLAG;
    pn544_dev->client->ext_flag |= I2C_DMA_FLAG;
  //pn544_dev->client->ext_flag |= I2C_DIRECTION_FLAG;
  //pn544_dev->client->ext_flag |= I2C_A_FILTER_MSG;
#endif

    /* Write data */
#if defined( I2C_DMA_USAGE )
      ret = i2c_master_send( pn544_dev->client, (unsigned char *)I2CDMAWriteBuf_pa, count );
	//ret = i2c_master_send( pn544_dev->client, (unsigned char *)I2CDMAWriteBuf, count );
#else
    ret = i2c_master_send( pn544_dev->client, (unsigned char *)tmp, count );
#endif

    if( ret != count )
    {
      NFC_TRACE("%s : i2c_master_send returned %d\n", __func__, ret );
//      mutex_unlock( &pn544_dev->rw_mutex );
      ret = -EIO;
    }

    pn544_dev->client->addr     = addr;
    pn544_dev->client->ext_flag = ext_flag;

    NFC_TRACE("%s : complete, result = %d\n", __func__, ret );
//    mutex_unlock( &pn544_dev->rw_mutex );
    return ret;
#else	
	PN544_DEV * pn544_dev;
	//char write_buf[MAX_BUFFER_SIZE];
	int ret = 0, ret_tmp = 0, count_ori = 0,count_remain = 0, idx = 0;
	pn544_dev = filp->private_data;
    count_ori = count;
    count_remain = count_ori;
	NFC_TRACE("[NFC]%s: Enter for writing %zu bytes.\n", __func__, count );
	if (count > MAX_BUFFER_SIZE)
	{
		count = MAX_BUFFER_SIZE;
		count_remain -= count;
	}
    do
    {

        if (copy_from_user(I2CDMAWriteBuf, &buf[(idx*512)], count)) 
        {
            printk(KERN_DEBUG "%s : failed to copy from user space\n", __func__);
            return -EFAULT;
    	}
    	//printk(KERN_DEBUG "%s : writing %zu bytes, remain bytes %zu.\n", __func__, count, count_remain);
    	printk(KERN_DEBUG "%s : writing %zu bytes, remain bytes %d.\n", __func__, count, count_remain);
    	
    	/* Write data */
        pn544_dev->client->addr = (pn544_dev->client->addr & I2C_MASK_FLAG);// | I2C_DMA_FLAG;

        pn544_dev->client->ext_flag |= I2C_DMA_FLAG;
        pn544_dev->client->timing = NFC_CLIENT_TIMING;

        ret_tmp = i2c_master_send(pn544_dev->client, (unsigned char *)(uintptr_t)I2CDMAWriteBuf_pa, count);

        if (ret_tmp != count) 
        {
            printk(KERN_DEBUG "%s : i2c_master_send returned %d\n", __func__, ret);
            ret = -EIO;
            return ret;
        }
    	
        ret += ret_tmp;
        printk(KERN_DEBUG "%s : %d,%d,%d\n", __func__, ret_tmp,ret,count_ori);
    	
        if( ret ==  count_ori)
    	{
            printk(KERN_DEBUG "%s : ret== count_ori \n", __func__);
            break;		
    	}
    	else
    	{
            if(count_remain > MAX_BUFFER_SIZE)
            {
                count = MAX_BUFFER_SIZE;
    		    count_remain -= MAX_BUFFER_SIZE;
            }
            else
            {
                count = count_remain;
                count_remain = 0;
            }
            idx++;		
    	   
            printk(KERN_DEBUG "%s :remain_bytes, %d,%d,%d,%d,%d\n", __func__, ret_tmp,ret,(int)count,count_ori,idx);
    	}
    	
	}
	while(1);

	printk(KERN_DEBUG "%s : writing %d bytes. Status %d \n", __func__, count_ori,ret);
	return ret;
#endif
}

/*****************************************************************************
**
******************************************************************************/
static int pn544_dev_open(struct inode *inode, struct file *filp)
{
PN544_DEV *pn544_dev = container_of( filp->private_data,
                          PN544_DEV,
                          pn544_device );

    NFC_TRACE("[NFC]%s: Enter...\n", __func__ );

    filp->private_data = pn544_dev;

    NFC_TRACE("%s : %d, %d\n", __func__, imajor( inode ), iminor( inode ));

    return 0;
}

/*****************************************************************************
**
******************************************************************************/
static long pn544_dev_ioctl(struct file *filp,
          unsigned int cmd, unsigned long arg)
{
PN544_DEV *pn544_dev = filp->private_data;

    NFC_TRACE("[NFC]%s: Enter...\n", __func__ );

    switch( cmd )
    {
      case PN544_SET_PWR:
      {
	 //Add NFC_ID Check function in SP meta<--JackHu20141017-->
	 if( arg == 3 )
        { /* power on */
          pr_info("%s power on check NFC ID in SP meta\n", __func__);
          if(0!=mt_set_gpio_out( GPIO_NFC_FIRM_PIN, GPIO_OUT_ZERO ))  //gpio_direction_output(pn544_dev->firm_gpio, 0);
          {
	   	pr_info("%s set GPIO_NFC_FIRM_PIN Fail \n", __func__);
          	return -EINVAL;
          }
          if(0!=mt_set_gpio_out( GPIO_NFC_VENB_PIN, GPIO_OUT_ONE ))   //gpio_direction_output(pn544_dev->ven_gpio, VEN_ENABALE);
          {
	   	pr_info("%s set GPIO_NFC_VENB_PIN Fail \n", __func__);
          	return -EINVAL;
          }
          msleep( ENABLE_DELAY);
	    if(g_pn544_dev==NULL)
	    {
	        pr_info("%s check g_pn544_dev Fail \n", __func__);
		 return -EINVAL;
	    }
        }
	 
        else if( arg == 2 )
        {
        /* power on with firmware download (requires hw reset) */
          pr_info( "%s power on with firmware\n", __func__ );
          //NFC_TRACE("[NFC]%s power on with firmware\n", __func__ );
          mt_set_gpio_out( GPIO_NFC_VENB_PIN, GPIO_OUT_ONE );   //gpio_direction_output( pn544_dev->ven_gpio, VEN_ENABALE );
          mt_set_gpio_out( GPIO_NFC_FIRM_PIN, GPIO_OUT_ONE );   //gpio_direction_output( pn544_dev->firm_gpio, 1 );
          msleep( ENABLE_DELAY );
          mt_set_gpio_out( GPIO_NFC_VENB_PIN, GPIO_OUT_ZERO );  //gpio_direction_output(pn544_dev->ven_gpio, VEN_DISABALE);
          msleep( DISABLE_DELAY );
          mt_set_gpio_out( GPIO_NFC_VENB_PIN, GPIO_OUT_ONE );   //gpio_direction_output(pn544_dev->ven_gpio, VEN_ENABALE);
          msleep( ENABLE_DELAY );
        }
        else if( arg == 1 )
        { /* power on */
        /*--- [ALL][Main][NFC][DMS][41958][LuboLu] Fix turn-off NFC can't suspend issue. 20140728 begin ---*/
        //<2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
        pn544_enable_irq(filp->private_data);
        //>2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
        //irq_set_irq_wake( CUST_EINT_IRQ_NFC_NUM, 1 );   //irq_set_irq_wake( pn544_dev->client->irq, 1 );
        /*--- [ALL][Main][NFC][DMS][41958][LuboLu] 20140728 end ---*/
          pr_info("%s power on\n", __func__);
          NFC_TRACE("[NFC]%s power on\n", __func__ );
          mt_set_gpio_out( GPIO_NFC_FIRM_PIN, GPIO_OUT_ZERO );  //gpio_direction_output(pn544_dev->firm_gpio, 0);
          mt_set_gpio_out( GPIO_NFC_VENB_PIN, GPIO_OUT_ONE );   //gpio_direction_output(pn544_dev->ven_gpio, VEN_ENABALE);
          msleep( ENABLE_DELAY);
        }
        else if( arg == 0 )
        { /* power off */
        /*--- [ALL][Main][NFC][DMS][41958][LuboLu] Fix turn-off NFC can't suspend issue. 20140728 begin ---*/
        //<2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
        pn544_disable_irq(filp->private_data);
        //>2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
        //irq_set_irq_wake( CUST_EINT_IRQ_NFC_NUM, 0 );   //irq_set_irq_wake( pn544_dev->client->irq, 0 );
        /*--- [ALL][Main][NFC][DMS][41958][LuboLu] 20140728 end ---*/
          pr_info("%s power off\n", __func__ );
          NFC_TRACE("[NFC]%s power off\n", __func__ );
          mt_set_gpio_out( GPIO_NFC_FIRM_PIN, GPIO_OUT_ZERO );  //gpio_direction_output(pn544_dev->firm_gpio, 0);
          mt_set_gpio_out( GPIO_NFC_VENB_PIN, GPIO_OUT_ZERO );  //gpio_direction_output(pn544_dev->ven_gpio, VEN_DISABALE);
          msleep( DISABLE_DELAY );
        }
        else
        {
          pr_err("%s bad arg %lu\n", __func__, arg );
          return -EINVAL;
        }
      } break;
	//Add NFC_ID Check function in SP meta<--JackHu20141017-->
      default:
      {
        pr_err("%s bad ioctl %u\n", __func__, cmd );
        return -EINVAL;
      }
    }
    return 0;
}

static const struct file_operations pn544_dev_fops = {
    .owner  = THIS_MODULE,
    .llseek = no_llseek,
    .read   = pn544_dev_read,
    .write  = pn544_dev_write,
    .open   = pn544_dev_open,
    .unlocked_ioctl = pn544_dev_ioctl,
};

/*****************************************************************************
**
******************************************************************************/
//static struct kobject * android_nfc_kobj;
static int create_sysfs_interfaces(void)
{
//int   ret;
//
//    android_nfc_kobj = kobject_create_and_add( "android_nfc", NULL );
//    if( android_nfc_kobj == NULL )
//    {
//      printk( KERN_DEBUG "Subsystem register failed.\n");
//      ret = -ENOMEM;
//      return  ret;
//    }
//
//    ret = sysfs_create_file( android_nfc_kobj, &dev_attr_get_swp_sim.attr );
//    if( ret )
//    {
//      printk( KERN_DEBUG "sysfs create file failed 1.\n");
//      return  ret;
//    }
//
//    ret = sysfs_create_file( android_nfc_kobj, &dev_attr_set_swp_sim.attr );
//    if( ret )
//    {
//      printk( KERN_DEBUG "sysfs create file failed 2.\n");
//      return  ret;
//    }

    return  0;
} /* End.. create_sysfs_interfaces() */

/*****************************************************************************
**
******************************************************************************/
static int remove_sysfs_interfaces(void)
{
//    sysfs_remove_file( android_nfc_kobj, &dev_attr_set_swp_sim.attr );
//    sysfs_remove_file( android_nfc_kobj, &dev_attr_get_swp_sim.attr );
//    kobject_del( android_nfc_kobj );
    return  0;
} /* End.. remove_sysfs_interfaces() */

/*****************************************************************************
**
******************************************************************************/
static int pn544_parse_dt(struct device *dev, struct pn544_i2c_platform_data *pdata)
{
struct device_node *np = dev->of_node;
int r = 0;
/*
    NFC_TRACE("[NFC]%s: Enter...\n", __func__ );

    pr_debug("%s, get VEN gpio number\n", __func__ );
    pdata->ven_gpio = of_get_named_gpio( np, "mediatek,ven-gpio", 0 );
    if(( !gpio_is_valid( pdata->ven_gpio )))
      return -EINVAL;

    pr_debug("%s, get IRQ gpio number\n", __func__ );
    pdata->irq_gpio = of_get_named_gpio( np, "mediatek,irq-gpio", 0 );
    if(( !gpio_is_valid( pdata->irq_gpio )))
      return -EINVAL;

    pr_debug( "%s, get Firm gpio number\n", __func__ );
    pdata->firm_gpio = of_get_named_gpio( np, "mediatek,firm-gpio", 0 );
    if(( !gpio_is_valid( pdata->firm_gpio )))
      return -EINVAL;*/
	  pdata->ven_gpio=GPIO_NFC_VENB_PIN;
	  pdata->irq_gpio=GPIO_IRQ_NFC_PIN;
	  pdata->firm_gpio=GPIO_NFC_FIRM_PIN;
    return r;
}
#if 1
static struct pn544_i2c_platform_data   pn544_pdata =
{
    .irq_gpio   = GPIO_IRQ_NFC_PIN,   /* GPIO137, NFC_IRQ  */
    .ven_gpio   = GPIO_NFC_VENB_PIN,  /* GPIO128, NFC_VEN = GPIO_NFC_EINT_PIN */
    .firm_gpio  = GPIO_NFC_FIRM_PIN,  /* GPIO119, NFC_DWL_REQ */
};

static struct i2c_board_info __initdata pn544_i2c_device[] =
{
  {
    I2C_BOARD_INFO( "pn544", ( PN544_SLAVE_ADDR >> 1 )),
    //.platform_data = &pn544_pdata,
    //.of_node  = NULL,
    //.irq      = CUST_EINT_IRQ_NFC_NUM,
  },
};

static struct platform_device pn544_plat_device =
{
    .name   = "pn544",
    .id     = -1,
};
#endif
/*****************************************************************************
**
******************************************************************************/
static int pn544_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
struct pn544_i2c_platform_data *platform_data;
PN544_DEV *pn544_dev;
int r = 0;
int ret;

    NFC_TRACE("[NFC]%s: Enter...\n", __func__ );
	printk("[NFC]%s: Enter...\n", __func__ );
    client->timing    = 400; /* 400 KHz */

    if( client->dev.of_node )
    {
      platform_data = devm_kzalloc( &client->dev, sizeof(*platform_data), GFP_KERNEL );
      if( !platform_data )
      {
        dev_err( &client->dev, "pn544 probe: Failed to allocate memory\n");
        return -ENOMEM;
      }
      r = pn544_parse_dt( &client->dev, platform_data );
      if( r )
        return r;
    }
    else
    {
      platform_data = client->dev.platform_data;
    }
	//Jack
	//platform_data=&pn544_pdata;
	
	//Jack
    NFC_TRACE("[NFC]%s step 1...\n", __func__ );

    if( !platform_data )
    {
      NFC_ERR( "%s, Can not get platform_data\n", __func__ );
      return -EINVAL;
    }

    dev_dbg( &client->dev, "pn544 probe: %s, inside pn544 flags = %x\n",
          __func__, client->flags );

    if( platform_data == NULL )
    {
      pr_err("%s : nfc probe fail\n", __func__ );
      return  -ENODEV;
    }
    NFC_TRACE("[NFC]%s step 2...\n", __func__ );

    if( !i2c_check_functionality( client->adapter, I2C_FUNC_I2C ))
    {
      pr_err( "%s : need I2C_FUNC_I2C\n", __func__ );
      return  -ENODEV;
    }
    NFC_TRACE("[NFC]%s step 3...\n", __func__ );


    pn544_dev = kzalloc( sizeof( *pn544_dev ), GFP_KERNEL );
    if( pn544_dev == NULL )
    {
      dev_err( &client->dev, "failed to allocate memory for module data\n");
      ret = -ENOMEM;
      goto err_exit;
    }

    NFC_TRACE("[NFC]%s step 4...\n", __func__ );

    pn544_dev->irq_gpio   = platform_data->irq_gpio;
    pn544_dev->ven_gpio   = platform_data->ven_gpio;
    pn544_dev->firm_gpio  = platform_data->firm_gpio;
    pn544_dev->client     = client;

  /* init mutex and queues */
    init_waitqueue_head( &pn544_dev->read_wq );
    mutex_init( &pn544_dev->read_mutex );
    spin_lock_init( &pn544_dev->irq_enabled_lock );

//    mutex_init( &pn544_dev->rw_mutex );

    pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
    pn544_dev->pn544_device.name  = NFC_DEV_NAME;
    pn544_dev->pn544_device.fops  = &pn544_dev_fops;
    ret = misc_register( &pn544_dev->pn544_device );
    if( ret )
    {
      pr_err("%s : misc_register failed\n", __FILE__);
      goto err_misc_register;
    }
    NFC_TRACE("[NFC]%s step 5...\n", __func__ );

#if defined( I2C_DMA_USAGE )
    if( I2CDMAWriteBuf == NULL )
    {
	  #ifdef CONFIG_64BIT    
	  I2CDMAWriteBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAWriteBuf_pa, GFP_KERNEL);
      #else
      I2CDMAWriteBuf = (char *)dma_alloc_coherent( NULL, MAX_BUFFER_SIZE, &I2CDMAWriteBuf_pa, GFP_KERNEL );
	  #endif
      if( I2CDMAWriteBuf == NULL )
      {
        NFC_ERR("%s : failed to allocate dma write buffer\n", __func__ );
        goto err_request_irq_failed;
      }
    }
    NFC_TRACE("[NFC]%s step 5-1...\n", __func__ );

    if( I2CDMAReadBuf == NULL )
    {
	  #ifdef CONFIG_64BIT 	
	  I2CDMAReadBuf = (char *)dma_alloc_coherent(&client->dev, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL);
	  #else
      I2CDMAReadBuf = (char *)dma_alloc_coherent( NULL, MAX_BUFFER_SIZE, (dma_addr_t *)&I2CDMAReadBuf_pa, GFP_KERNEL );
	  #endif
      if( I2CDMAReadBuf == NULL )
      {
        NFC_ERR("%s : failed to allocate dma read buffer\n", __func__ );
        goto err_request_irq_failed;
      }
    }
    NFC_TRACE("[NFC]%s step 5-2...\n", __func__ );
#endif /* End.. (I2C_DMA_USAGE) */

  /* request irq.  the irq is set whenever the chip has data available
   * for reading.  it is cleared when all data has been read.
   */
    mt_set_gpio_mode( GPIO_IRQ_NFC_PIN, GPIO_NFC_EINT_PIN_M_EINT );  /* GPIO_IRQ_NFC_PIN = platform_data->irq_gpio */
    mt_set_gpio_dir( GPIO_IRQ_NFC_PIN, GPIO_DIR_IN );
    //mt_set_gpio_pull_enable( GPIO_IRQ_NFC_PIN, GPIO_PULL_DISABLE );
    mt_set_gpio_pull_enable( GPIO_IRQ_NFC_PIN, GPIO_PULL_ENABLE );
    //mt_set_gpio_pull_select( GPIO_IRQ_NFC_PIN, GPIO_PULL_UP );
    mt_set_gpio_pull_select( GPIO_IRQ_NFC_PIN, GPIO_PULL_DOWN );
//
//mt_eint_set_sens( CUST_EINT_IRQ_NFC_NUM, CUST_EINT_IRQ_NFC_SENSITIVE );   /* CUST_EINT_IRQ_NFC_NUM = client->irq */
//mt_eint_set_polarity( CUST_EINT_IRQ_NFC_NUM, CUST_EINT_IRQ_NFC_POLARITY );
//
    msleep(10);
	
    mt_set_gpio_mode( GPIO_NFC_VENB_PIN, GPIO_MODE_00 );  /* GPIO_NFC_VENB_PIN = platform_data->ven_gpio */
    mt_set_gpio_dir( GPIO_NFC_VENB_PIN, GPIO_DIR_OUT );
    mt_set_gpio_out( GPIO_NFC_VENB_PIN, GPIO_OUT_ZERO );
    msleep(10);

    mt_set_gpio_mode( GPIO_NFC_FIRM_PIN, GPIO_MODE_00 );  /* GPIO_NFC_FIRM_PIN = platform_data->firm_gpio */
    mt_set_gpio_dir( GPIO_NFC_FIRM_PIN, GPIO_DIR_OUT );
    mt_set_gpio_out( GPIO_NFC_FIRM_PIN, GPIO_OUT_ZERO );
    msleep(10);

    g_pn544_dev = pn544_dev;

#if 1
//    mt_eint_set_hw_debounce( GPIO_IRQ_NFC_PIN, CUST_EINT_IRQ_NFC_DEBOUNCE_CN );
    mt_eint_registration( CUST_EINT_IRQ_NFC_NUM, CUST_EINT_IRQ_NFC_TYPE, pn544_dev_irq_handler, 0 );
    //mt_eint_unmask( CUST_EINT_IRQ_NFC_NUM );
    pn544_dev->irq_enabled = true;
    pn544_disable_irq( pn544_dev );
#else
    ret = request_irq( client->irq, pn544_dev_irq_handler,
                IRQF_TRIGGER_HIGH, client->name, pn544_dev );
    if( ret )
    {
      dev_err( &client->dev, "request_irq failed\n" );
      goto err_request_irq_failed;
    }
#endif
    NFC_TRACE("[NFC]%s step 6...\n", __func__ );
    i2c_set_clientdata( client, pn544_dev );

    //<2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    wake_lock_init(&pn544_dev->wake_lock, WAKE_LOCK_SUSPEND, "pn544_nfc_read");
    //>2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

/*=========================================
**
**=========================================*/
    ret = create_sysfs_interfaces();
    if( ret < 0 )
    {
      NFC_TRACE("sysfs create error.\n");
      goto err_sysfs;
    }
    NFC_TRACE("[NFC]%s Success...\n", __func__ );

    return 0;

err_sysfs:
    remove_sysfs_interfaces();
err_request_irq_failed:
    misc_deregister( &pn544_dev->pn544_device );
err_misc_register:
    mutex_destroy( &pn544_dev->read_mutex );
//    mutex_destroy( &pn544_dev->rw_mutex );
err_exit:
    kfree( pn544_dev );
//  gpio_free(platform_data->firm_gpio);
    mt_set_gpio_out( GPIO_NFC_VENB_PIN, GPIO_OUT_ZERO );
    mt_set_gpio_out( GPIO_NFC_FIRM_PIN, GPIO_OUT_ZERO );
//err_firm:
//  gpio_free( platform_data->ven_gpio );
//err_ven:
//  gpio_free( platform_data->irq_gpio );
    NFC_TRACE("[NFC]%s some error...\n", __func__ );
    //<2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    wake_lock_destroy(&pn544_dev->wake_lock);
    //>2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

    return  ret;
}

/*****************************************************************************
**
******************************************************************************/
static int pn544_remove(struct i2c_client *client)
{
PN544_DEV *pn544_dev;

    NFC_TRACE("[NFC]%s: Enter...\n", __func__ );

    pn544_dev = i2c_get_clientdata( client );

#if defined( I2C_DMA_USAGE )
    if( I2CDMAWriteBuf )
    {
	  #ifdef CONFIG_64BIT 	
	  dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAWriteBuf, I2CDMAWriteBuf_pa);
	  #else
      dma_free_coherent( NULL, MAX_BUFFER_SIZE, I2CDMAWriteBuf, I2CDMAWriteBuf_pa );
	  #endif
      I2CDMAWriteBuf    = NULL;
      I2CDMAWriteBuf_pa = 0;
    }

    if( I2CDMAReadBuf )
    {
	  #ifdef CONFIG_64BIT
	  dma_free_coherent(&client->dev, MAX_BUFFER_SIZE, I2CDMAReadBuf, I2CDMAReadBuf_pa);
	  #else
      dma_free_coherent( NULL, MAX_BUFFER_SIZE, I2CDMAReadBuf, I2CDMAReadBuf_pa );
	  #endif
      I2CDMAReadBuf     = NULL;
      I2CDMAReadBuf_pa  = 0;
    }
#endif /* End.. (I2C_DMA_USAGE) */

    mt_set_gpio_out( GPIO_NFC_VENB_PIN, GPIO_OUT_ZERO );
    mt_set_gpio_out( GPIO_NFC_FIRM_PIN, GPIO_OUT_ZERO );

    //<2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    //free_irq( client->irq, pn544_dev );
    //>2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    misc_deregister( &pn544_dev->pn544_device );
    mutex_destroy( &pn544_dev->read_mutex );
//    mutex_destroy( &pn544_dev->rw_mutex );
    //<2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode
    wake_lock_destroy(&pn544_dev->wake_lock);
    //>2015/9/7-jazzchiang--B[All][Main][NFC][DMS07217345] To enhance the message handling in suspend mode

    gpio_free( pn544_dev->irq_gpio );
    gpio_free( pn544_dev->ven_gpio );
    gpio_free( pn544_dev->firm_gpio );

    kfree( pn544_dev );

    return 0;
}

/*****************************************************************************
**
******************************************************************************/
/* i2c_register_board_info will be not to do over here */


static struct of_device_id pn544_match_table[] = {
    { .compatible = "mediatek,pn544" },
    {}
};

static const struct i2c_device_id pn544_id[] = {
    { "pn544", 0},
    {}
};

static struct i2c_driver pn544_driver = {
    .id_table = pn544_id,
    .probe    = pn544_probe,
    .remove   = pn544_remove,
    .driver   =
    {
      .owner  = THIS_MODULE,
      .name   = "pn544",
      .of_match_table = pn544_match_table,
    },
};

/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
int vRet = 0;

  //pr_info("[NFC] %s: Loading pn544 driver...\n", __func__ );
    printk("[NFC] %s: Loading pn544 driver...Jacack\n", __func__ );

/* i2c_register_board_info will be not to do over here */
#if 1
    i2c_register_board_info( 1, &pn544_i2c_device, ARRAY_SIZE(pn544_i2c_device));
#endif

    if( i2c_add_driver( &pn544_driver ))
    {
    //pr_info("[NFC] PN544 add I2C driver error\n");
      printk("[NFC] PN544 add I2C driver error\n");
      vRet = -1;
    }
    else
    {
    //pr_info("[NFC] PN544 add I2C driver success\n");
      printk("[NFC] PN544 add I2C driver success\n");
    }
    return vRet;
}
module_init( pn544_dev_init );

static void __exit pn544_dev_exit(void)
{
  //pr_info("[NFC] Unloading pn544 driver\n");
    printk("[NFC] Unloading pn544 driver\n");
    i2c_del_driver( &pn544_driver );
}
module_exit( pn544_dev_exit );

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
