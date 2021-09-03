
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>
#ifdef CONFIG_COMPAT
#include <linux/fs.h>
#include <linux/compat.h>
#endif
#include "kd_flashlight.h"
// <<< 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M
//# << 2014/12/9-zihweishen, porting sub flash
#include <cust_i2c.h>
#include <linux/i2c.h>
#include <linux/leds.h>


#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif
//# >> 2014/12/9-zihweishen, porting sub flash
// >>> 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M
/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "[sub_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        pr_warning(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      pr_notice(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        pr_info(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              pr_debug(TAG_NAME "<%s>\n", __FUNCTION__)
#define PK_TRC_VERBOSE(fmt, arg...) pr_debug(TAG_NAME fmt, ##arg)
#define PK_ERROR(fmt, arg...)       pr_err(TAG_NAME "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif

// <<< 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M
//# << 2014/12/9-zihweishen, porting sub flash
static u32 strobe_Res = 0;
static int gDuty=0;
static int g_timeOutTimeMs=0;
static struct hrtimer g_timeOutTimer;

// <<< 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX
//<2015/02/24 ShermanWei,MTK suggestion
//# << 2014/12/9-zihweishen, modify for preflashlight is torsh mode, flashlight is flashmode
////static int gIsTorch[18]={0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//# >> 2014/12/9-zihweishen, modify for preflashlight is torsh mode, flashlight is flashmode
// <<< 2015/04/21-youchihwang, Magic Beam feture
static int gIsTorch[18] ={     1,      0,      0,      0,      0,      0,      0,     0,      0,       1,     1,       1,      1,      0,      0,      0,      0,      0};
static int gLedDuty[18]={0x30, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10,  0x20, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00};
// >>> 2015/04/21-youchihwang, Magic Beam feture
//>2015/02/24 ShermanWei,MTK suggestion
// >>> 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */
static struct work_struct workTimeOut;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

extern struct i2c_client *RT4505_i2c_client;
int FL_Sub_Disable(void);

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Sub_Disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}

enum hrtimer_restart Sub_ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}

void Sub_timerInit(void)
{
    INIT_WORK(&workTimeOut, work_timeOutFunc);
    g_timeOutTimeMs=1000; //1s
    hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
    g_timeOutTimer.function=Sub_ledTimeOutCallback;
}

extern int RT4505_write_reg(struct i2c_client *client, u8 reg, u8 val);
extern int RT4505_read_reg(struct i2c_client *client, u8 reg,u8 *val);

#if 0
static int RT4505_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
    int ret=0;
	
    struct RT4505_chip_data *chip = i2c_get_clientdata(client);

    mutex_lock(&chip->lock);
    ret =  i2c_smbus_write_byte_data(client, reg, val);
    mutex_unlock(&chip->lock);

    if (ret < 0) PK_ERR("failed writting at 0x%02x\n", reg);
    
    return ret;
}

static int RT4505_read_reg(struct i2c_client *client, u8 reg)
{
    int val=0;
    
    struct RT4505_chip_data *chip = i2c_get_clientdata(client);

    mutex_lock(&chip->lock);
    val =  i2c_smbus_read_byte_data(client, reg);
    mutex_unlock(&chip->lock);

    return val;
}
#endif

int FL_Sub_Enable(void)
{
    int buf[2];

    buf[0]=0x04;
    if(gIsTorch[gDuty]==1)
    {
        buf[1]=0x10;
    }  
    else
    {
        buf[1]=0x20;
    }  
    RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
    PK_DBG("youchihwang sub flash FL_Sub_Enable line=%d\n",__LINE__);
    return 0;
}

int FL_Sub_Disable(void)
{
	int buf[2];

    buf[0]=0x04;
    buf[1]=0x00;
    mt_set_gpio_out(GPIO131, GPIO_OUT_ZERO);
    RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
    PK_DBG("youchihwang sub flash FL_Sub_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_Sub_dim_duty(kal_uint32 duty)
{
//<2015/02/24 ShermanWei,MTK suggestion
    int buf[2];

// <<< 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX
    // <<< 2015/04/22-youchihwang, Magic Beam feture
    if(duty>17)
        duty=17;
    // >>> 2015/03/16-youchihwang, Magic Beam feture
// >>> 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX
    if(duty<0)
        duty=0;
    gDuty=duty;
// <<< 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX
    // <<< 2015/03/16-youchihwang, Magic Beam feture
    if(gIsTorch[gDuty] == 1)
    {
        buf[0]=3;
        buf[1]=gLedDuty[duty];
        RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
    }
    else
    {
        buf[0] = 1;
	    buf[1] = gLedDuty[duty];
	    RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
    }
    // >>> 2015/03/16-youchihwang, Magic Beam feture
// >>> 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX
//>2015/02/24 ShermanWei,MTK suggestion
    PK_DBG(" FL_Sub_dim_duty line=%d\n",__LINE__);
    return 0;
}

// [Lavender][FlashLED][akenhsu] Add way to custom Front Flash current for Camera Team Calibration 20150130 BEGIN 
extern u8 g_front_flash_level;
// [Lavender][FlashLED][akenhsu] 20150130 END

int FL_Sub_Init(void)
{
    mt_set_gpio_mode(GPIO125, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO125, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO125, GPIO_OUT_ONE);

    mt_set_gpio_mode(GPIO124, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO124, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO124, GPIO_OUT_ZERO);

    //# << 2014/12/9-zihweishen, init SKY81296 FLASH_EN pin
    mt_set_gpio_mode(GPIO131, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO131, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO131, GPIO_OUT_ZERO);
    //# >> 2014/12/9-zihweishen, init SKY81296 FLASH_EN pin

    //# << 2014/12/9-zihweishen, setting FL2 Flash Mode Current = 750mA
    int buf[2];
    buf[0]=1;
// [Lavender][FlashLED][akenhsu] Add way to custom Front Flash current for Camera Team Calibration 20150130 BEGIN
//    buf[1]=0xA;
    buf[1] = g_front_flash_level;
// [Lavender][FlashLED][akenhsu] 20150130 END
    RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
    //# >> 2014/12/9-zihweishen, setting FL2 Flash Mode Current = 750mA

    //# << 2014/12/9-zihweishen, setting FL2 Flash Mode Time-Out = 570mS
    buf[0]=2;
    buf[1]=0x60;
    RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
    //# >> 2014/12/9-zihweishen, setting FL2 Flash Mode Time-Out = 570mS

    //# << 2014/12/9-zihweishen, setting FL2 Movie Mode Current Setting = 100mA
    buf[0]=3;
    buf[1]=0x30;
    RT4505_write_reg(RT4505_i2c_client, buf[0], buf[1]);
    //# >> 2014/12/9-zihweishen, setting FL2 Movie Mode Current Setting = 100mA

    PK_DBG(" FL_Sub_Init line=%d\n",__LINE__);
    return 0;
}

int FL_Sub_Uninit(void)
{
    FL_Sub_Disable();
    return 0;
}

//# >> 2014/12/9-zihweishen, porting sub flash
// >>> 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M

// <<< 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M
static int sub_strobe_ioctl(MUINT32 cmd, MUINT32 arg)
//static int sub_strobe_ioctl(unsigned int cmd, unsigned long arg)
// >>> 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M
{
// <<< 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M
//# << 2014/12/9-zihweishen, porting sub flash
    int i4RetValue = 0;
    int ior_shift;
    int iow_shift;
    int iowr_shift;
    ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
    iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
    iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));

	PK_DBG("sub dummy ioctl");

    switch(cmd)
    {
    	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
    		PK_DBG("youchihwang sub flash FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
    		g_timeOutTimeMs=arg;
    		break;

    	case FLASH_IOC_SET_DUTY :
    		PK_DBG("youchihwang sub flash FLASHLIGHT_DUTY: %d\n",arg);
    		FL_Sub_dim_duty(arg);
    		break;

    	case FLASH_IOC_SET_STEP:
    		PK_DBG("youchihwang sub flash FLASH_IOC_SET_STEP: %d\n",arg);
    		break;

    	case FLASH_IOC_SET_ONOFF :
    	    PK_DBG("youchihwang sub flash FLASHLIGHT_ONOFF: %d\n",arg);
    	    if(arg==1)
    	    {
// <<< 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX
                // <<< 2015/03/16-youchihwang, Magic Beam feture
                if(g_timeOutTimeMs!=0)
                {
                    ktime_t ktime;
                    ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
                    hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
                }
                // >>> 2015/03/16-youchihwang, Magic Beam feture
// >>> 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX
    		    FL_Sub_Enable();
    	    }
    	    else
    	    {
    	        FL_Sub_Disable();
// <<< 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX
                // <<< 2015/03/18-youchihwang, Magic Beam feture
    		    hrtimer_cancel( &g_timeOutTimer );
		        // >>> 2015/03/18-youchihwang, Magic Beam feture
// >>> 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX
    	    }
    	    break;
    		
	    default :
    	        PK_DBG("youchihwang sub flash  No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
//# >> 2014/12/9-zihweishen, porting sub flash
// >>> 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M
    return 0;
}

static int sub_strobe_open(void *pArg)
{
// <<< 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M 
//# << 2014/12/9-zihweishen, porting sub flash
    int i4RetValue = 0;

    PK_DBG("sub dummy open");
    
    if (0 == strobe_Res)
    {
	    FL_Sub_Init();
// <<< 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX
        // <<< 2015/03/16-youchihwang, Magic Beam feture
	    Sub_timerInit();
        // >>> 2015/03/16-youchihwang, Magic Beam feture
// >>> 2016/04/10-dangyiwang, Add FP20572 Style portrait magic beam UX
    }

    spin_lock_irq(&g_strobeSMPLock);

    if(strobe_Res)
    {
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }

    spin_unlock_irq(&g_strobeSMPLock);
//# >> 2014/12/9-zihweishen, porting sub flash
// >>> 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M
    return 0;

}

static int sub_strobe_release(void *pArg)
{
    PK_DBG("sub dummy release");
// <<< 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M
//# << 2014/12/9-zihweishen, porting sub flash
    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

        FL_Sub_Uninit();
    }
//# >> 2014/12/9-zihweishen, porting sub flash
// >>> 2016/02/24-dangyiwang, Proting lm3644 from cosmos/lavender L to M
    return 0;

}

FLASHLIGHT_FUNCTION_STRUCT	subStrobeFunc=
{
	sub_strobe_open,
	sub_strobe_release,
	sub_strobe_ioctl
};


MUINT32 subStrobeInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &subStrobeFunc;
    }
    return 0;
}





