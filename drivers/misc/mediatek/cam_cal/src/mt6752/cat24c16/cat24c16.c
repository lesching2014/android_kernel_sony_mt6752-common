/*
 * Driver for CAM_CAL
 *
 *
 */

#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include "kd_camera_hw.h"
#include "cam_cal.h"
#include "cam_cal_define.h"
#include "cat24c16.h"
//#include <asm/system.h>  // for SMP
#include <linux/dma-mapping.h>
#ifdef CONFIG_COMPAT
/* 64 bit */
#include <linux/fs.h>
#include <linux/compat.h>
#endif


//#define CAM_CALGETDLT_DEBUG
#define CAM_CAL_DEBUG
#ifdef CAM_CAL_DEBUG
#include <linux/xlog.h>
#define PFX "cat2416c"
//#define CAM_CALDB printk
#define CAM_CALINF(fmt, arg...)    xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " fmt, __FUNCTION__, ##arg)
#define CAM_CALDB(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG   , PFX, "[%s] " fmt, __FUNCTION__, ##arg)
#define CAM_CALERR(fmt, arg...)    xlog_printk(ANDROID_LOG_ERROR   , PFX, "[%s] " fmt, __FUNCTION__, ##arg)
#else
#define CAM_CALDB(x,...)
#endif

static DEFINE_SPINLOCK(g_CAM_CALLock); // for SMP
//<2014/10/22-kylechang, Modify EEPROM Driver for Cosmos
#define CAM_CAL_I2C_BUSNUM 3
extern  u8 OTPData[1400];
 //u8 OTPData[]; 
 //>2014/10/22-kylechang

/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_ICS_REVISION 1 //seanlin111208
/*******************************************************************************
*
********************************************************************************/
#define CAM_CAL_DRVNAME "CAM_CAL_DRV"
#define CAM_CAL_I2C_GROUP_ID 0
/*******************************************************************************
*
********************************************************************************/
//<2014/10/22-kylechang, Modify EEPROM Driver for Cosmos
static struct i2c_board_info __initdata kd_cam_cal_dev={ I2C_BOARD_INFO(CAM_CAL_DRVNAME, 0xaa>>1)};  //ORIG=0x6d , Kyle
//>2014/10/22-kylechang

static struct i2c_client * g_pstI2Cclient = NULL;

//81 is used for V4L driver
static dev_t g_CAM_CALdevno = MKDEV(CAM_CAL_DEV_MAJOR_NUMBER,0);
static struct cdev * g_pCAM_CAL_CharDrv = NULL;
//static spinlock_t g_CAM_CALLock;
//spin_lock(&g_CAM_CALLock);
//spin_unlock(&g_CAM_CALLock);

static struct class *CAM_CAL_class = NULL;
static atomic_t g_CAM_CALatomic;
//static DEFINE_SPINLOCK(kdcam_cal_drv_lock);
//spin_lock(&kdcam_cal_drv_lock);
//spin_unlock(&kdcam_cal_drv_lock);

#define LSCOTPDATASIZE 512

int otp_flag=0;

static kal_uint8 lscotpdata[LSCOTPDATASIZE];
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#if 0
#define iWriteCAM_CAL(addr, bytes, para,) iWriteReg((u16) addr , (u32) para , 1, IMX214OTP_DEVICE_ID)
#else

/*******************************************************************************
*
********************************************************************************/
//Address: 2Byte, Data: 1Byte
int iWriteCAM_CAL(u16 a_u2Addr  , u32 a_u4Bytes, u8 puDataInBytes)
{   
    #if 0
    int  i4RetValue = 0;
	int retry = 3;
    char puSendCmd[3] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF) ,puDataInBytes};
	g_pstI2Cclient->addr = (IMX214OTP_DEVICE_ID>> 1);

	do {
        CAM_CALDB("[CAM_CAL][iWriteCAM_CAL] Write 0x%x=0x%x \n",a_u2Addr, puDataInBytes);
		i4RetValue = i2c_master_send(g_pstI2Cclient, puSendCmd, 3);
        if (i4RetValue != 3) {
            CAM_CALDB("[CAM_CAL] I2C send failed!!\n");
        }
        else {
            break;
    	}
        mdelay(10);
    } while ((retry--) > 0);    
   //CAM_CALDB("[CAM_CAL] iWriteCAM_CAL done!! \n");
   #else
   CAM_CALDB("[CAM_CAL][iWriteCAM_CAL] Write 0x%x=0x%x \n",a_u2Addr, puDataInBytes);
   iWriteReg(a_u2Addr,puDataInBytes,1,IMX214OTP_DEVICE_ID);
   #endif
   return 0;
}
#endif

//Address: 2Byte, Data: 1Byte
int iReadCAM_CAL(u16 a_u2Addr, u32 ui4_length, u8 * a_puBuff)
{
    #if 0
    int  i4RetValue = 0;
    char puReadCmd[2] = {(char)(a_u2Addr >> 8) , (char)(a_u2Addr & 0xFF)};
	g_pstI2Cclient->addr = (IMX214OTP_DEVICE_ID>> 1);

    //CAM_CALDB("[CAM_CAL] iReadCAM_CAL!! \n");   
    //CAM_CALDB("[CAM_CAL] i2c_master_send \n");
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 2);
	
    if (i4RetValue != 2)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
        return -1;
    }

    //CAM_CALDB("[CAM_CAL] i2c_master_recv \n");
    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, ui4_length);
	CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
    if (i4RetValue != ui4_length)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
        return -1;
    } 

    //CAM_CALDB("[CAM_CAL] iReadCAM_CAL done!! \n");
	#else
	int  i4RetValue = 0;

	i4RetValue=iReadReg(a_u2Addr,a_puBuff,IMX214OTP_DEVICE_ID);
	
	if (i4RetValue !=0)
		{
			CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
			return -1;
		} 	
	CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);
	#endif
    return 0;
}

int iReadCAM_CAL_8(u8 a_u2Addr, u8 * a_puBuff, u16 i2c_id)
{
    int  i4RetValue = 0;
    char puReadCmd[1] = {(char)(a_u2Addr)};

    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient->addr = i2c_id>>1;
	g_pstI2Cclient->timing=400;
	g_pstI2Cclient->addr = g_pstI2Cclient->addr & (I2C_MASK_FLAG | I2C_WR_FLAG);
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    i4RetValue = i2c_master_send(g_pstI2Cclient, puReadCmd, 1);
	
    if (i4RetValue != 1)
    {
        CAM_CALDB("[CAM_CAL] I2C send read address failed!! \n");
	    CAM_CALDB("[CAMERA SENSOR] I2C send failed, addr = 0x%x, data = 0x%x !! \n", a_u2Addr,  *a_puBuff );

        return -1;
    }

    i4RetValue = i2c_master_recv(g_pstI2Cclient, (char *)a_puBuff, 1);
	//CAM_CALDB("[CAM_CAL][iReadCAM_CAL] Read 0x%x=0x%x \n", a_u2Addr, a_puBuff[0]);

    if (i4RetValue != 1)
    {
        CAM_CALDB("[CAM_CAL] I2C read data failed!! \n");
        return -1;
    } 

    return 0;
}

//go to page
#if 1
static kal_uint8 IMX214_GotoPage(kal_uint8 page)
{
}


static kal_uint8 check_IMX214_otp_valid_AFPage(void)
{

}


static kal_uint8 check_IMX214_otp_valid_AWBPage(void)
{
}

#endif

kal_bool check_IMX214_otp_valid_LSC_Page(kal_uint8 page)
{
	kal_uint8 LSC_OK = 0x00;
	u8 readbuff, i;
	kal_uint16 LSC_lengthL,LSC_lengthH;

	iReadCAM_CAL_8(0x00,&readbuff,0xA0);
	
	LSC_OK = readbuff;
	printk("+++dang test+++ LSC_OK context is : %d\n", LSC_OK);
// <<< 2016/04/08-dangyiwang, remove check, because LSC_OK value is not "1" but OTP data is correct
	/*if (LSC_OK==1)
	{
		CAM_CALDB("can read LSC otp from page0~page5\n");
		CAM_CALDB("page number %d is valid\n",LSC_OK);	 
	}
	else
	    return KAL_FALSE;*/
// >>> 2016/04/08-dangyiwang, remove check, because LSC_OK value is not "1" but OTP data is correct

	iReadCAM_CAL_8(0x15,&LSC_lengthL,0xA0);
	iReadCAM_CAL_8(0x16,&LSC_lengthH,0xA0);
	CAM_CALDB("LSC_length is=0x%x\n", ((LSC_lengthH<<8)&0xFF00) |(LSC_lengthL&0x00FF));
	
	return KAL_TRUE;
}

#if 1
 kal_bool IMX214_Read_LSC_Otp(kal_uint8 pagestart,kal_uint8 pageend,u16 Outdatalen,unsigned char * pOutputdata)
 {
 
 kal_uint8 page = 0;
 kal_uint16 byteperpage = 256;
 kal_uint16 number = 0;
 kal_uint16 LSCOTPaddress = 0x00 ; 
 u8 readbuff;
 int i = 0;
 if(otp_flag){

	for(i=0;i<Outdatalen;i++)
		pOutputdata[i]=OTPData[i];
	printk("otg_flag==1, Copy OTPData[] to pOutputdata[] l!\n");
 }
else{
 	
   if (!check_IMX214_otp_valid_LSC_Page(page))
	{
	 CAM_CALDB("check_IMX214_otp_valid_LSC_Page fail!\n");
	 return KAL_FALSE;
		 
	}

   for(page = pagestart; page<=pageend; page++)
   {
	 CAM_CALDB("read page=%d\n",page);

	 if(page==0){
	     for(i = 23; i < byteperpage;i++)
	     {
	       iReadCAM_CAL_8(LSCOTPaddress+i,&readbuff,0xA0);
	       pOutputdata[number]=readbuff;
		   number+=1;
	     }
	   
	 }
	 else if(page==5){
	 	 for(i = 0; i < 131;i++)
	     {
	       iReadCAM_CAL_8(LSCOTPaddress+i,&readbuff,0xAA);
	       pOutputdata[number]=readbuff;
		   number+=1;

	     }
	 }
	 else{
	    for(i = 0; i < byteperpage;i++)
	    {
	      iReadCAM_CAL_8(LSCOTPaddress+i,&readbuff,(page*2+0xA0));
		  pOutputdata[number]=readbuff;
		  number+=1;

	    }
	 }

    }
         //<2014/10/22-kylechang, Modify EEPROM Driver for Cosmos
	if(number==1388)
	{
		for(i=0;i<Outdatalen;i++)
			OTPData[i]=pOutputdata[i];

		otp_flag=1;
		printk("otg_flag==1, Copy pOutputdata[1388] to OTPData[1388]  l!\n");
	}
	//>2014/10/22-kylechang

}
     CAM_CALDB("LSC read finished number= %d\n",--number);
	 //otp_flag=0;//<2014/10/22-kylechang>
	 return KAL_TRUE;
 
 }
#endif


 void IMX214_ReadOtp(kal_uint8 page,kal_uint16 address,unsigned char *iBuffer,unsigned int buffersize)
{
		kal_uint16 i = 0;
		u8 readbuff ;
		int ret ;
		CAM_CALDB("[CAM_CAL]ENTER page:0x%x address:0x%x buffersize:%d\n ",page, address, buffersize);
		if (IMX214_GotoPage(page))
		{
			for(i = 0; i<buffersize; i++)
			{				
				ret= iReadCAM_CAL_8(address+i,&readbuff,0xA0);
				CAM_CALDB("[CAM_CAL]address+i = 0x%x,readbuff = 0x%x\n",(address+i),readbuff );
				*(iBuffer+i) =(unsigned char)readbuff;
			}
		}
}

//<2014/10/22-kylechang, Modify EEPROM Driver for Cosmos
 void IMX214_Read_AWB_AF(kal_uint8 page,kal_uint16 address,unsigned char *iBuffer,unsigned int buffersize)
{
		kal_uint16 i = 0;
		u8 readbuff ;
		int ret ;
		CAM_CALDB("[IMX214_Read_AWB_AF]ENTER page:0x%x address:0x%x buffersize:%d\n ",page, address, buffersize);

		for(i = 0; i<buffersize; i++)
		{				
			ret= iReadCAM_CAL_8(address+i,&readbuff,  (page*2+0xA0
) );
			CAM_CALDB("[CAM_CAL]address+i = 0x%x,readbuff = 0x%x\n",(address+i),readbuff );
			*(iBuffer+i) =(unsigned char)readbuff;
		}
}
//>2014/10/22-kylechang

//Burst Write Data
static int iWriteData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
	CAM_CALDB("not implemented!");
	return 0;
}

//Burst Read Data
 int iReadData(unsigned int  ui4_offset, unsigned int  ui4_length, unsigned char * pinputdata)
{
   int  i4RetValue = 0;
    kal_uint8 page = 0, pageS=0,pageE=5;;
	//1. check which page is valid
	
	if(ui4_length ==1)
    {
       if(check_IMX214_otp_valid_LSC_Page(page))
		
	    IMX214_ReadOtp(page, ui4_offset, pinputdata, ui4_length);
   	   else
   		{
	   		 CAM_CALDB("[CAM_CAL]No AF OTP Data!\n");
	  		 return -1;
        }
	   	
   	}
//<2014/10/22-kylechang, Modify EEPROM Driver for Cosmos
	else if(ui4_length ==8)//Read AWB
	{
		IMX214_Read_AWB_AF(0, ui4_offset, pinputdata, ui4_length);
	}
	else if(ui4_length ==2) //Read AF
	{
		IMX214_Read_AWB_AF(5, ui4_offset, pinputdata, ui4_length);
	}
//>2014/10/22-kylechang
	else{
		
		CAM_CALDB("[CAM_CAL]Read LSC data 1388bit!\n");

		if(IMX214_Read_LSC_Otp(pageS,pageE,ui4_length,pinputdata))
			CAM_CALDB("[CAM_CAL]LSC OTP Data Read Sucess!\n");
		else
			CAM_CALDB("[CAM_CAL]LSC OTP Data Read Fail!\n");
	}
    //2. read otp

    //for(i4RetValue = 0;i4RetValue<ui4_length;i4RetValue++)
    //{
    //	CAM_CALDB( "[[CAM_CAL]]pinputdata[%d]=%x\n", i4RetValue,*(pinputdata+i4RetValue));
    //}
    CAM_CALDB(" [[CAM_CAL]]page = %d,ui4_length = %d,ui4_offset =%d\n ",page,ui4_length,ui4_offset);
    CAM_CALDB("[S24EEPORM] iReadData done\n" );
   return 0;
}



#ifdef CONFIG_COMPAT
static int compat_put_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data->u4Offset);
    err |= put_user(i, &data32->u4Offset);
    err |= get_user(i, &data->u4Length);
    err |= put_user(i, &data32->u4Length);
    /* Assume pointer is not change */
#if 1
    err |= get_user(p, &data->pu1Params);
    err |= put_user(p, &data32->pu1Params);
#endif
    return err;
}
static int compat_get_cal_info_struct(
            COMPAT_stCAM_CAL_INFO_STRUCT __user *data32,
            stCAM_CAL_INFO_STRUCT __user *data)
{
    compat_uptr_t p;
    compat_uint_t i;
    int err;

    err = get_user(i, &data32->u4Offset);
    err |= put_user(i, &data->u4Offset);
    err |= get_user(i, &data32->u4Length);
    err |= put_user(i, &data->u4Length);
    err |= get_user(p, &data32->pu1Params);
    err |= put_user(compat_ptr(p), &data->pu1Params);

    return err;
}

static long cat24c16_Ioctl_Compat(struct file *filp, unsigned int cmd, unsigned long arg)
{
    long ret;
    CAM_CALDB("[CAMERA SENSOR] COMPAT_CAM_CALIOC_G_READ\n");
    COMPAT_stCAM_CAL_INFO_STRUCT __user *data32;
    stCAM_CAL_INFO_STRUCT __user *data;
    int err;
	  CAM_CALDB("[CAMERA SENSOR] cat24c16_Ioctl_Compat,%p %p %x ioc size %d\n",filp->f_op ,filp->f_op->unlocked_ioctl,cmd,_IOC_SIZE(cmd) );

    if (!filp->f_op || !filp->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {

    case COMPAT_CAM_CALIOC_G_READ:
    {
        data32 = compat_ptr(arg);
        data = compat_alloc_user_space(sizeof(*data));
        if (data == NULL)
            return -EFAULT;

        err = compat_get_cal_info_struct(data32, data);
        if (err)
            return err;

        ret = filp->f_op->unlocked_ioctl(filp, CAM_CALIOC_G_READ,(unsigned long)data);
        err = compat_put_cal_info_struct(data32, data);


        if(err != 0)
            CAM_CALERR("[CAMERA SENSOR] compat_put_acdk_sensor_getinfo_struct failed\n");
        return ret;
    }
    default:
        return -ENOIOCTLCMD;
    }
}


#endif



/*******************************************************************************
*
********************************************************************************/
#define NEW_UNLOCK_IOCTL
#ifndef NEW_UNLOCK_IOCTL
static int CAM_CAL_Ioctl(struct inode * a_pstInode,
struct file * a_pstFile,
unsigned int a_u4Command,
unsigned long a_u4Param)
#else 
static long CAM_CAL_Ioctl(
    struct file *file, 
    unsigned int a_u4Command, 
    unsigned long a_u4Param
)
#endif
{
    int i4RetValue = 0;
    u8 * pBuff = NULL;
    u8 * pu1Params = NULL;
    stCAM_CAL_INFO_STRUCT *ptempbuf;
	CAM_CALDB("[S24CAM_CAL] ioctl\n");

#ifdef CAM_CALGETDLT_DEBUG
    struct timeval ktv1, ktv2;
    unsigned long TimeIntervalUS;
#endif

    if(_IOC_NONE == _IOC_DIR(a_u4Command))
    {
    }
    else
    {
        pBuff = (u8 *)kmalloc(sizeof(stCAM_CAL_INFO_STRUCT),GFP_KERNEL);

        if(NULL == pBuff)
        {
            CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
            return -ENOMEM;
        }

        if(_IOC_WRITE & _IOC_DIR(a_u4Command))
        {
            if(copy_from_user((u8 *) pBuff , (u8 *) a_u4Param, sizeof(stCAM_CAL_INFO_STRUCT)))
            {    //get input structure address
                kfree(pBuff);
                CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
                return -EFAULT;
            }
        }
    }

    ptempbuf = (stCAM_CAL_INFO_STRUCT *)pBuff;
    pu1Params = (u8*)kmalloc(ptempbuf->u4Length,GFP_KERNEL);
    if(NULL == pu1Params)
    {
        kfree(pBuff);
        CAM_CALDB("[S24CAM_CAL] ioctl allocate mem failed\n");
        return -ENOMEM;
    }
     CAM_CALDB(" init Working buffer address 0x%p  command is 0x%x\n", pu1Params, a_u4Command);


    if(copy_from_user((u8*)pu1Params ,  (u8*)ptempbuf->pu1Params, ptempbuf->u4Length))
    {
        kfree(pBuff);
        kfree(pu1Params);
        CAM_CALDB("[S24CAM_CAL] ioctl copy from user failed\n");
        return -EFAULT;
    } 
    
    switch(a_u4Command)
    {
        case CAM_CALIOC_S_WRITE:    
            CAM_CALDB("[S24CAM_CAL] Write CMD \n");
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv1);
#endif            
            i4RetValue = iWriteData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Write data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            
            break;
        case CAM_CALIOC_G_READ:
            CAM_CALDB("[S24CAM_CAL] Read CMD \n");
#ifdef CAM_CALGETDLT_DEBUG            
            do_gettimeofday(&ktv1);
#endif     
            CAM_CALDB("[CAM_CAL] offset %d \n", ptempbuf->u4Offset);
            CAM_CALDB("[CAM_CAL] length %d \n", ptempbuf->u4Length);
            //CAM_CALDB("[CAM_CAL] Before read Working buffer address 0x%p \n", pu1Params);

			//otp_flag=0;//Kyle ORIG=1;//<2014/10/22-kylechang>
            //i4RetValue = iReadData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pWorkingBuff);
			i4RetValue = iReadData((u16)ptempbuf->u4Offset, ptempbuf->u4Length, pu1Params);
            //i4RetValue = selective_read_region(ptempbuf->u4Offset, pu1Params, CATC24C16_DEVICE_ID,ptempbuf->u4Length);
			
            CAM_CALDB("[S24CAM_CAL] After read Working buffer data  0x%x \n", *pu1Params);


#ifdef CAM_CALGETDLT_DEBUG
            do_gettimeofday(&ktv2);
            if(ktv2.tv_sec > ktv1.tv_sec)
            {
                TimeIntervalUS = ktv1.tv_usec + 1000000 - ktv2.tv_usec;
            }
            else
            {
                TimeIntervalUS = ktv2.tv_usec - ktv1.tv_usec;
            }
            printk("Read data %d bytes take %lu us\n",ptempbuf->u4Length, TimeIntervalUS);
#endif            

            break;
        default :
      	     CAM_CALDB("[S24CAM_CAL] No CMD \n");
            i4RetValue = -EPERM;
        break;
    }

    if(_IOC_READ & _IOC_DIR(a_u4Command))
    {
        //copy data to user space buffer, keep other input paremeter unchange.
        CAM_CALDB("[S24CAM_CAL] to user length %d \n", ptempbuf->u4Length);
        CAM_CALDB("[S24CAM_CAL] to user  Working buffer address 0x%p \n", pu1Params);
        if(copy_to_user((u8 __user *) ptempbuf->pu1Params , (u8 *)pu1Params , ptempbuf->u4Length))
        {
            kfree(pBuff);
            kfree(pu1Params);
            CAM_CALDB("[S24CAM_CAL] ioctl copy to user failed\n");
            return -EFAULT;
        }
    }

    kfree(pBuff);
    kfree(pu1Params);
    return i4RetValue;
}


static u32 g_u4Opened = 0;
//#define
//Main jobs:
// 1.check for device-specified errors, device not ready.
// 2.Initialize the device if it is opened for the first time.
static int CAM_CAL_Open(struct inode * a_pstInode, struct file * a_pstFile)
{
    CAM_CALDB("[S24CAM_CAL] CAM_CAL_Open\n");
    spin_lock(&g_CAM_CALLock);
    if(g_u4Opened)
    {
        spin_unlock(&g_CAM_CALLock);
		CAM_CALDB("[S24CAM_CAL] Opened, return -EBUSY\n");
        return -EBUSY;
    }
    else
    {
        g_u4Opened = 1;
        atomic_set(&g_CAM_CALatomic,0);
    }
    spin_unlock(&g_CAM_CALLock);

	//<2014/11/18-kylechang, Fix Power Consumption
	/*
    //<2014/10/22-kylechang, Modify EEPROM Driver for Cosmos
    //if(TRUE != hwPowerOn(MT65XX_POWER_LDO_VCAMA, VOL_2800, CAM_CAL_DRVNAME))
    if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, Vol_1800, CAM_CAL_DRVNAME))
    //>2014/10/22-kylechang
    {
        CAM_CALDB("[S24CAM_CAL] Fail to enable analog gain\n");
        return -EIO;
    }
	*/
	//<2014/11/18-kylechang

    return 0;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
static int CAM_CAL_Release(struct inode * a_pstInode, struct file * a_pstFile)
{
    spin_lock(&g_CAM_CALLock);

    g_u4Opened = 0;

    atomic_set(&g_CAM_CALatomic,0);

    spin_unlock(&g_CAM_CALLock);

    return 0;
}

static const struct file_operations g_stCAM_CAL_fops =
{
    .owner = THIS_MODULE,
    .open = CAM_CAL_Open,
    .release = CAM_CAL_Release,
    //.ioctl = CAM_CAL_Ioctl
#ifdef CONFIG_COMPAT
    .compat_ioctl = cat24c16_Ioctl_Compat,
#endif
    .unlocked_ioctl = CAM_CAL_Ioctl
};

#define CAM_CAL_DYNAMIC_ALLOCATE_DEVNO 1
inline static int RegisterCAM_CALCharDrv(void)
{
    struct device* CAM_CAL_device = NULL;

#if CAM_CAL_DYNAMIC_ALLOCATE_DEVNO
    if( alloc_chrdev_region(&g_CAM_CALdevno, 0, 1,CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Allocate device no failed\n");

        return -EAGAIN;
    }
#else
    if( register_chrdev_region(  g_CAM_CALdevno , 1 , CAM_CAL_DRVNAME) )
    {
        CAM_CALDB("[S24CAM_CAL] Register device no failed\n");

        return -EAGAIN;
    }
#endif

    //Allocate driver
    g_pCAM_CAL_CharDrv = cdev_alloc();

    if(NULL == g_pCAM_CAL_CharDrv)
    {
        unregister_chrdev_region(g_CAM_CALdevno, 1);

        CAM_CALDB("[S24CAM_CAL] Allocate mem for kobject failed\n");

        return -ENOMEM;
    }

    //Attatch file operation.
    cdev_init(g_pCAM_CAL_CharDrv, &g_stCAM_CAL_fops);

    g_pCAM_CAL_CharDrv->owner = THIS_MODULE;

    //Add to system
    if(cdev_add(g_pCAM_CAL_CharDrv, g_CAM_CALdevno, 1))
    {
        CAM_CALDB("[S24CAM_CAL] Attatch file operation failed\n");

        unregister_chrdev_region(g_CAM_CALdevno, 1);

        return -EAGAIN;
    }

    CAM_CAL_class = class_create(THIS_MODULE, "CAM_CALdrv");
    if (IS_ERR(CAM_CAL_class)) {
        int ret = PTR_ERR(CAM_CAL_class);
        CAM_CALDB("Unable to create class, err = %d\n", ret);
        return ret;
    }
    CAM_CAL_device = device_create(CAM_CAL_class, NULL, g_CAM_CALdevno, NULL, CAM_CAL_DRVNAME);

    return 0;
}

inline static void UnregisterCAM_CALCharDrv(void)
{
    //Release char driver
    cdev_del(g_pCAM_CAL_CharDrv);

    unregister_chrdev_region(g_CAM_CALdevno, 1);

    device_destroy(CAM_CAL_class, g_CAM_CALdevno);
    class_destroy(CAM_CAL_class);
}


//////////////////////////////////////////////////////////////////////
#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
#elif 0
static int CAM_CAL_i2c_detect(struct i2c_client *client, struct i2c_board_info *info);
#else
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int CAM_CAL_i2c_remove(struct i2c_client *);

static const struct i2c_device_id CAM_CAL_i2c_id[] = {{CAM_CAL_DRVNAME,0},{}};   



static struct i2c_driver CAM_CAL_i2c_driver = {
    .probe = CAM_CAL_i2c_probe,                                   
    .remove = CAM_CAL_i2c_remove,                           
//   .detect = CAM_CAL_i2c_detect,                           
    .driver.name = CAM_CAL_DRVNAME,
    .id_table = CAM_CAL_i2c_id,                             
};

#ifndef CAM_CAL_ICS_REVISION
static int CAM_CAL_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info) {         
    strcpy(info->type, CAM_CAL_DRVNAME);
    return 0;
}
#endif
static int CAM_CAL_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id) {             

int i4RetValue = 0;
u8 readbuff;


    CAM_CALDB("[S24CAM_CAL] Attach I2C \n");
//    spin_lock_init(&g_CAM_CALLock);

    //get sensor i2c client
    spin_lock(&g_CAM_CALLock); //for SMP
    g_pstI2Cclient = client;
    g_pstI2Cclient->addr = IMX214OTP_DEVICE_ID>>1;
    spin_unlock(&g_CAM_CALLock); // for SMP    
    
    CAM_CALDB("[CAM_CAL] g_pstI2Cclient->addr = 0x%8x \n",g_pstI2Cclient->addr);
    //Register char driver
    i4RetValue = RegisterCAM_CALCharDrv();

    if(i4RetValue){
        CAM_CALDB("[S24CAM_CAL] register char device failed!\n");
        return i4RetValue;
    }


    CAM_CALDB("[S24CAM_CAL] Attached!! \n");

    //Kyle Start

        if(TRUE != hwPowerOn(CAMERA_POWER_VCAM_D2, Vol_1800, CAM_CAL_DRVNAME))
       {
             printk("+++KYLE+++CAM_CAL_i2c_probe[S24CAM_CAL] Fail to hwPowerOn\n");
             return -EIO;
        }
	mdelay(2);


         iReadCAM_CAL_8(0xE3,&readbuff,0xAA);
	 printk("+++KYLE+++ CAM_CAL_i2c_probe:  Mid= 0x%x \n", readbuff);

	 iReadCAM_CAL_8(0xE4,&readbuff,0xAA);
	 printk("+++KYLE+++ CAM_CAL_i2c_probe:  Lens ID= 0x%x \n", readbuff);

 	 iReadCAM_CAL_8(0xE6,&readbuff,0xAA);
   	printk("+++KYLE+++ CAM_CAL_i2c_probe:  VCM Driver ID= 0x%x \n", readbuff);
        // 0xAA Read OK

	 iReadCAM_CAL_8(0x01,&readbuff,0xA0);
	printk("+++KYLE+++ CAM_CAL_i2c_probe:  k Version 0x01= 0x%x \n", readbuff);

	iReadCAM_CAL_8(0x02,&readbuff,0xA0);
	 printk("+++KYLE+++ CAM_CAL_i2c_probe:  k Version 0x02= 0x%x \n", readbuff);
   
  	 iReadCAM_CAL_8(0x03,&readbuff,0xA0);
	printk("+++KYLE+++ CAM_CAL_i2c_probe:  k Version 0x03= 0x%x \n", readbuff);
   
  	 iReadCAM_CAL_8(0x04,&readbuff,0xA0);
	printk("+++KYLE+++ CAM_CAL_i2c_probe:  k Version 0x04= 0x%x \n", readbuff);


   //<2014/11/18-kylechang, Fix Power Consumption
   if(TRUE != hwPowerDown(CAMERA_POWER_VCAM_D2, CAM_CAL_DRVNAME))
   {
	   printk("[IMX214 CAM_CAL] Fail to PowerDown CAMERA_POWER_VCAM_D2\n");
	   return -EIO;
   }
   //<2014/11/18-kylechang

   //Kyle End


	
    return 0;                                                                                       
} 

static int CAM_CAL_i2c_remove(struct i2c_client *client)
{
    return 0;
}

static int CAM_CAL_probe(struct platform_device *pdev)
{
    return i2c_add_driver(&CAM_CAL_i2c_driver);
}

static int CAM_CAL_remove(struct platform_device *pdev)
{
    i2c_del_driver(&CAM_CAL_i2c_driver);
    return 0;
}

// platform structure
static struct platform_driver g_stCAM_CAL_Driver = {
    .probe		= CAM_CAL_probe,
    .remove	= CAM_CAL_remove,
    .driver		= {
        .name	= CAM_CAL_DRVNAME,
        .owner	= THIS_MODULE,
    }
};


static struct platform_device g_stCAM_CAL_Device = {
    .name = CAM_CAL_DRVNAME,
    .id = 0,
    .dev = {
    }
};

static int __init CAM_CAL_i2C_init(void)
{
    i2c_register_board_info(CAM_CAL_I2C_BUSNUM, &kd_cam_cal_dev, 1);
    if(platform_driver_register(&g_stCAM_CAL_Driver)){
        CAM_CALDB("failed to register S24CAM_CAL driver\n");
        return -ENODEV;
    }

    if (platform_device_register(&g_stCAM_CAL_Device))
    {
        CAM_CALDB("failed to register S24CAM_CAL driver, 2nd time\n");
        return -ENODEV;
    }	

    return 0;
}

static void __exit CAM_CAL_i2C_exit(void)
{
	platform_driver_unregister(&g_stCAM_CAL_Driver);
}

module_init(CAM_CAL_i2C_init);
module_exit(CAM_CAL_i2C_exit);

MODULE_DESCRIPTION("CAM_CAL driver");
MODULE_AUTHOR("Sean Lin <Sean.Lin@Mediatek.com>");
MODULE_LICENSE("GPL");


