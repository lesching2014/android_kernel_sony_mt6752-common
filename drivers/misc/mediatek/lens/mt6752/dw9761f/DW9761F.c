/*
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "DW9761F.h"
#include "../camera/kd_camera_hw.h"
#include <linux/xlog.h>
#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

// in K2, main=3, sub=main2=1
#define LENS_I2C_BUSNUM 3

#define DW9761F_DRVNAME "DW9761F"
#define DW9761F_VCM_WRITE_ID           0x1C
#define PLATFORM_DRIVER_NAME "lens_actuator_dw9761f"

static struct i2c_board_info kd_lens_dev __initdata = { I2C_BOARD_INFO(DW9761F_DRVNAME, DW9761F_VCM_WRITE_ID) };

#define DW9761F_DEBUG
#ifdef DW9761F_DEBUG
#define DW9761FDB pr_err
#else
#define DW9761FDB(x, ...)
#endif

static spinlock_t g_DW9761F_SpinLock;

static struct i2c_client *g_pstDW9761F_I2Cclient;

static dev_t g_DW9761F_devno;
static struct cdev *g_pDW9761F_CharDrv;
static struct class *actuator_class;

static int g_s4DW9761F_Opened;
static long g_i4MotorStatus;
static long g_i4Dir;
static unsigned long g_u4DW9761F_INF;
static unsigned long g_u4DW9761F_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int g_sr = 3;

static int i2c_read(u8 a_u2Addr, u8 *a_puBuff)
{
	int i4RetValue = 0;
	char puReadCmd[1] = { (char)(a_u2Addr) };
	i4RetValue = i2c_master_send(g_pstDW9761F_I2Cclient, puReadCmd, 1);
	if (i4RetValue != 2) {
		DW9761FDB(" I2C write failed!!\n");
		return -1;
	}
	/*  */
	i4RetValue = i2c_master_recv(g_pstDW9761F_I2Cclient, (char *)a_puBuff, 1);
	if (i4RetValue != 1) {
		DW9761FDB(" I2C read failed!!\n");
		return -1;
	}

	return 0;
}

u8 s4DW9761F_read_data(u8 addr)
{
	u8 get_byte = 0;
	i2c_read(addr, &get_byte);
	DW9761FDB("[DW9761F]  get_byte %d\n", get_byte);
	return get_byte;
}

static int s4DW9761F_ReadReg(unsigned short *a_pu2Result)
{
	/* int  i4RetValue = 0; */
	/* char pBuff[2]; */

	*a_pu2Result = (s4DW9761F_read_data(0x03) << 8) + (s4DW9761F_read_data(0x04) & 0xff);

	DW9761FDB("[DW9761F]  s4DW9761F_ReadReg %d\n", *a_pu2Result);
	return 0;
}

static int s4DW9761F_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[3] = { 0x03, (char)(a_u2Data >> 8), (char)(a_u2Data & 0xFF) };

	DW9761FDB("[DW9761F]  write %d\n", a_u2Data);

	g_pstDW9761F_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
	i4RetValue = i2c_master_send(g_pstDW9761F_I2Cclient, puSendCmd, 3);

	if (i4RetValue < 0) {
		DW9761FDB("[DW9761F] I2C send failed!!\n");
		return -1;
	}

	return 0;
}

inline static int getDW9761FInfo(__user stDW9761F_MotorInfo * pstMotorInfo)
{
	stDW9761F_MotorInfo stMotorInfo;
	stMotorInfo.u4MacroPosition = g_u4DW9761F_MACRO;
	stMotorInfo.u4InfPosition = g_u4DW9761F_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = TRUE;

	if (g_i4MotorStatus == 1) {
		stMotorInfo.bIsMotorMoving = 1;
	} else {
		stMotorInfo.bIsMotorMoving = 0;
	}

	if (g_s4DW9761F_Opened >= 1) {
		stMotorInfo.bIsMotorOpen = 1;
	} else {
		stMotorInfo.bIsMotorOpen = 0;
	}

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stDW9761F_MotorInfo))) {
		DW9761FDB("[DW9761F] copy to user failed when getting motor information\n");
	}

	return 0;
}
//void initdrv()
//{
//	char puSendCmd2[2] = { 0x01, 0x39 };
//	char puSendCmd3[2] = { 0x05, 0x65 };
//	i2c_master_send(g_pstDW9761F_I2Cclient, puSendCmd2, 2);
//	i2c_master_send(g_pstDW9761F_I2Cclient, puSendCmd3, 2);
//}

inline static int moveDW9761F(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4DW9761F_MACRO) || (a_u4Position < g_u4DW9761F_INF)) {
		DW9761FDB("[DW9761F] out of range\n");
		return -EINVAL;
	}

	if (g_s4DW9761F_Opened == 1) {
		unsigned short InitPos;
//		initdrv();
		ret = s4DW9761F_ReadReg(&InitPos);

		if (ret == 0) {
			DW9761FDB("[DW9761F] Init Pos %6d\n", InitPos);

			spin_lock(&g_DW9761F_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(&g_DW9761F_SpinLock);
		} else {
			spin_lock(&g_DW9761F_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(&g_DW9761F_SpinLock);
		}

		spin_lock(&g_DW9761F_SpinLock);
		g_s4DW9761F_Opened = 2;
		spin_unlock(&g_DW9761F_SpinLock);

	}

	if (g_u4CurrPosition < a_u4Position) {
		spin_lock(&g_DW9761F_SpinLock);
		g_i4Dir = 1;
		spin_unlock(&g_DW9761F_SpinLock);
	} else if (g_u4CurrPosition > a_u4Position) {
		spin_lock(&g_DW9761F_SpinLock);
		g_i4Dir = -1;
		spin_unlock(&g_DW9761F_SpinLock);
	} else {
		return 0;
	}

	spin_lock(&g_DW9761F_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(&g_DW9761F_SpinLock);

	DW9761FDB("[DW9761F] move [curr] %d [target] %d\n", (int)g_u4CurrPosition, (int)g_u4TargetPosition);

	spin_lock(&g_DW9761F_SpinLock);
	g_sr = 3;
	g_i4MotorStatus = 0;
	spin_unlock(&g_DW9761F_SpinLock);

	if (s4DW9761F_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(&g_DW9761F_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(&g_DW9761F_SpinLock);
	} else {
		DW9761FDB("[DW9761F] set I2C failed when moving the motor\n");
		spin_lock(&g_DW9761F_SpinLock);
		g_i4MotorStatus = -1;
		spin_unlock(&g_DW9761F_SpinLock);
	}

	return 0;
}

inline static int setDW9761FInf(unsigned long a_u4Position)
{
	spin_lock(&g_DW9761F_SpinLock);
	g_u4DW9761F_INF = a_u4Position;
	spin_unlock(&g_DW9761F_SpinLock);
	return 0;
}

inline static int setDW9761FMacro(unsigned long a_u4Position)
{
	spin_lock(&g_DW9761F_SpinLock);
	g_u4DW9761F_MACRO = a_u4Position;
	spin_unlock(&g_DW9761F_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
static long DW9761F_Ioctl(struct file *a_pstFile,
			   unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case DW9761FIOC_G_MOTORINFO:
		i4RetValue = getDW9761FInfo((__user stDW9761F_MotorInfo *) (a_u4Param));
		break;

	case DW9761FIOC_T_MOVETO:
		i4RetValue = moveDW9761F(a_u4Param);
		break;

	case DW9761FIOC_T_SETINFPOS:
		i4RetValue = setDW9761FInf(a_u4Param);
		break;

	case DW9761FIOC_T_SETMACROPOS:
		i4RetValue = setDW9761FMacro(a_u4Param);
		break;

	default:
		DW9761FDB("[DW9761F] No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}


/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
/* 3.Update f_op pointer. */
/* 4.Fill data structures into private_data */
/* CAM_RESET */
static int DW9761F_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
    int  i4RetValue = 0;

    char puSendCmd2[2] = {(char)(0x02) , (char)(0x02)};
    char puSendCmd3[2] = {(char)(0x06) , (char)(0x80)};
    char puSendCmd4[2] = {(char)(0x07) , (char)(0x39)};

	DW9761FDB("[DW9761F] DW9761F_Open - Start\n");
	if (g_s4DW9761F_Opened) {
		DW9761FDB("[DW9761F] the device is opened\n");
		return -EBUSY;
	}
	spin_lock(&g_DW9761F_SpinLock);
	g_s4DW9761F_Opened = 1;
	spin_unlock(&g_DW9761F_SpinLock);

    DW9761FDB("[DW9761F] write init setting");

    //g_pstDW9761F_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstDW9761F_I2Cclient, puSendCmd2, 2);

    //g_pstDW9761F_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstDW9761F_I2Cclient, puSendCmd3, 2);

    //g_pstDW9761F_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
    i4RetValue = i2c_master_send(g_pstDW9761F_I2Cclient, puSendCmd4, 2);

	DW9761FDB("[DW9761F] DW9761F_Open - End\n");
	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
#define BEGIN_STEPS 250
#define END_STEPS 50
#define JUMP_STEPS 20
#define STEP_PERIOD_MS 13
static int DW9761F_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	unsigned long 	CurrPosition = g_u4CurrPosition;

	DW9761FDB("[DW9761F] DW9761F_Release - Start, CurrentPosition = %d\n", (int)g_u4CurrPosition);

	if (g_s4DW9761F_Opened) {
		DW9761FDB("[DW9761F] feee\n");
		g_sr = 5;

		if (CurrPosition > BEGIN_STEPS + JUMP_STEPS) {
			CurrPosition = BEGIN_STEPS;
			s4DW9761F_WriteReg(CurrPosition);
			mdelay(STEP_PERIOD_MS);
		}
		while (CurrPosition >= END_STEPS) {
			CurrPosition -= JUMP_STEPS;
			s4DW9761F_WriteReg(CurrPosition);
			mdelay(STEP_PERIOD_MS);
		}

		spin_lock(&g_DW9761F_SpinLock);
		g_s4DW9761F_Opened = 0;
		spin_unlock(&g_DW9761F_SpinLock);

	}
	DW9761FDB("[DW9761F] DW9761F_Release - End\n");

	return 0;
}

static const struct file_operations g_stDW9761F_fops = {
	.owner = THIS_MODULE,
	.open = DW9761F_Open,
	.release = DW9761F_Release,
	.unlocked_ioctl = DW9761F_Ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = DW9761F_Ioctl,
#endif
};

inline static int Register_DW9761F_CharDrv(void)
{
	struct device *vcm_device = NULL;

	DW9761FDB("[DW9761F] Register_DW9761F_CharDrv - Start\n");

	/* Allocate char driver no. */
	if (alloc_chrdev_region(&g_DW9761F_devno, 0, 1, DW9761F_DRVNAME)) {
		DW9761FDB("[DW9761F] Allocate device no failed\n");

		return -EAGAIN;
	}
	/* Allocate driver */
	g_pDW9761F_CharDrv = cdev_alloc();

	if (NULL == g_pDW9761F_CharDrv) {
		unregister_chrdev_region(g_DW9761F_devno, 1);

		DW9761FDB("[DW9761F] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}
	/* Attatch file operation. */
	cdev_init(g_pDW9761F_CharDrv, &g_stDW9761F_fops);

	g_pDW9761F_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pDW9761F_CharDrv, g_DW9761F_devno, 1)) {
		DW9761FDB("[DW9761F] Attatch file operation failed\n");

		unregister_chrdev_region(g_DW9761F_devno, 1);

		return -EAGAIN;
	}

	actuator_class = class_create(THIS_MODULE, "actuatordrvDW9761F");
	if (IS_ERR(actuator_class)) {
		int ret = PTR_ERR(actuator_class);
		DW9761FDB("Unable to create class, err = %d\n", ret);
		return ret;
	}

	vcm_device = device_create(actuator_class, NULL, g_DW9761F_devno, NULL, DW9761F_DRVNAME);

	if (NULL == vcm_device) {
		return -EIO;
	}

	DW9761FDB("[DW9761F] Register_DW9761F_CharDrv - End\n");
	return 0;
}

inline static void Unregister_DW9761F_CharDrv(void)
{
	DW9761FDB("[DW9761F] Unregister_DW9761F_CharDrv - Start\n");

	/* Release char driver */
	cdev_del(g_pDW9761F_CharDrv);

	unregister_chrdev_region(g_DW9761F_devno, 1);

	device_destroy(actuator_class, g_DW9761F_devno);

	class_destroy(actuator_class);

	DW9761FDB("[DW9761F] Unregister_DW9761F_CharDrv - End\n");
}

/* //////////////////////////////////////////////////////////////////// */

static int DW9761F_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int DW9761F_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id DW9761F_i2c_id[] = { {DW9761F_DRVNAME, 0}, {} };

struct i2c_driver DW9761F_i2c_driver = {
	.probe = DW9761F_i2c_probe,
	.remove = DW9761F_i2c_remove,
	.driver.name = DW9761F_DRVNAME,
	.id_table = DW9761F_i2c_id,
};

#if 0
static int DW9761F_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, DW9761F_DRVNAME);
	return 0;
}
#endif
static int DW9761F_i2c_remove(struct i2c_client *client)
{
	return 0;
}

/* Kirby: add new-style driver {*/
static int DW9761F_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i4RetValue = 0;

	DW9761FDB("[DW9761F] DW9761F_i2c_probe\n");

	/* Kirby: add new-style driver { */
	g_pstDW9761F_I2Cclient = client;

	g_pstDW9761F_I2Cclient->addr = g_pstDW9761F_I2Cclient->addr >> 1;

	/* Register char driver */
	i4RetValue = Register_DW9761F_CharDrv();

	if (i4RetValue) {

		DW9761FDB("[DW9761F] register char device failed!\n");

		return i4RetValue;
	}

	spin_lock_init(&g_DW9761F_SpinLock);

	DW9761FDB("[DW9761F] Attached!!\n");

	return 0;
}

static int DW9761F_probe(struct platform_device *pdev)
{
	return i2c_add_driver(&DW9761F_i2c_driver);
}

static int DW9761F_remove(struct platform_device *pdev)
{
	i2c_del_driver(&DW9761F_i2c_driver);
	return 0;
}

static int DW9761F_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int DW9761F_resume(struct platform_device *pdev)
{
	return 0;
}

/* platform structure */
static struct platform_driver g_stDW9761F_Driver = {
	.probe = DW9761F_probe,
	.remove = DW9761F_remove,
	.suspend = DW9761F_suspend,
	.resume = DW9761F_resume,
	.driver = {
		   .name = PLATFORM_DRIVER_NAME,
		   .owner = THIS_MODULE,
		   }
};
static struct platform_device g_stDW9761F_device = {
    .name = PLATFORM_DRIVER_NAME,
    .id = 0,
    .dev = {}
};
static int __init DW9761F_i2C_init(void)
{
	i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);
  if(platform_device_register(&g_stDW9761F_device)){
    DW9761FDB("failed to register AF driver\n");
    return -ENODEV;
  }
	if (platform_driver_register(&g_stDW9761F_Driver)) {
		DW9761FDB("failed to register DW9761F driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit DW9761F_i2C_exit(void)
{
	platform_driver_unregister(&g_stDW9761F_Driver);
}
module_init(DW9761F_i2C_init);
module_exit(DW9761F_i2C_exit);

MODULE_DESCRIPTION("DW9761F lens module driver");
MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
MODULE_LICENSE("GPL");
