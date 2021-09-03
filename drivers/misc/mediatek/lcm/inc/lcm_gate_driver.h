/*****************************************************************************
** LCM Gate Driver
**============================================================================
** Novatek NT50358
** Dual Channel DC-DC Converters
******************************************************************************/
#ifndef LCM_GATE_DRIVER_H
#define LCM_GATE_DRIVER_H

/*****************************************************************************
** Head files
******************************************************************************/
#if defined( LCM_GATE_DRIVER_C )
    #include <cust_gpio_usage.h>
  #if !defined( FPGA_EARLY_PORTING )
    #include  <cust_i2c.h>
  #endif

  #if defined( BUILD_LK )
    #include <platform/mt_gpio.h>
    #include <platform/mt_i2c.h>
    #include <platform/mt_pmic.h>
  #elif defined( BUILD_UBOOT )
    #include <asm/arch/mt_gpio.h>
  #else
    #include <mach/mt_pm_ldo.h>
    #include <mach/mt_gpio.h>

    #include  <linux/kernel.h>
    #include  <linux/module.h>
    #include  <linux/fs.h>
    #include  <linux/slab.h>
    #include  <linux/init.h>
    #include  <linux/list.h>
    #include  <linux/i2c.h>
    #include  <linux/irq.h>
  //#include  <linux/jiffies.h>
    #include  <linux/uaccess.h>
  //#include  <linux/delay.h>
    #include  <linux/interrupt.h>
    #include  <linux/io.h>
    #include  <linux/platform_device.h>
  #endif
#endif /* End.. (LCM_GATE_DRIVER_C) */

/*****************************************************************************
** Macro Define
******************************************************************************/
  #if !defined( TRUE )
    #define   TRUE      1
  #endif
  #if !defined( FALSE )
    #define   FALSE     0
  #endif
  #if defined( GLOBAL )
    #undef    GLOBAL
  #endif

    #define   LCM_GATE_I2C_BUS          I2C_I2C_LCD_BIAS_CHANNEL  /* For I2C channel 0 */
    #define   LCM_GATE_I2C_NAME         "lcm_gate"
    #define   LCM_GATE_DEV_ADDR         (0x3E)  //I2C_I2C_LCD_BIAS_SLAVE_7_BIT_ADDR
    #define   LCM_GATE_SLAVE_ADDR       (LCM_GATE_DEV_ADDR<<1)
//[Arima_Lavender][RaymondLin] add one LCM enable pin for SP run 20150402 begin	
#if defined(LAVENDER)
     #if !defined( FPGA_EARLY_PORTING )
        #define   GPIO_GATE_EP_PIN          GPIO_LCD_BIAS_ENP_PIN
        #define   GPIO_GATE_EN_PIN          GPIO_LCD_BIAS_ENN_PIN	
     #endif /* End.. !(FPGA_EARLY_PORTING) */  
#else //Cosmos project
  #if !defined( FPGA_EARLY_PORTING )
    #define   GPIO_GATE_EN_PIN          GPIO_LCD_BIAS_ENP_PIN
  #endif /* End.. !(FPGA_EARLY_PORTING) */
 #endif 
//[Arima_Lavender][RaymondLin] add one LCM enable pin for SP run 20150402 end
    #define   LCMGATE_TRACE_ENABLE_N
    #define   LCMGATE_DEBUG_ENABLE

#if defined( LCM_GATE_DRIVER_C )
  #if defined( BUILD_LK )
    #define   LCMGATE_PRINT             dprintf
  #else
    #define   LCMGATE_PRINT             printk
  #endif

#if defined( LCMGATE_TRACE_ENABLE )
  #if defined( BUILD_LK )
    #define   LCMGATE_MSG(srt,arg...)   dprintf(CRITICAL,srt,##arg)
  #else
    #define   LCMGATE_MSG(srt,arg...)   printk(KERN_INFO srt,##arg)
  #endif
#else
    #define   LCMGATE_MSG(srt,arg...)   {/* Do Nothing */}
#endif

#if defined( LCMGATE_DEBUG_ENABLE )
  #if defined( BUILD_LK )
    #define   LCMGATE_DBG(srt,arg...)   dprintf(CRITICAL,srt,##arg)
  #else
    #define   LCMGATE_DBG(srt,arg...)   printk(KERN_INFO srt,##arg)
  #endif
#else
    #define   LCMGATE_DBG(srt,arg...)   {/* Do Nothing */}
#endif

    #define   GLOBAL
#else
    #define   GLOBAL        extern
#endif /* End.. (LCM_GATE_DRIVER_C) */

/*****************************************************************************
** GLobal Variable
******************************************************************************/
#if defined( LCM_GATE_DRIVER_C )
#if !defined( FPGA_EARLY_PORTING )
  #if !defined( BUILD_LK )
    static struct i2c_board_info __initdata   lcm_gate_board_info = { I2C_BOARD_INFO( LCM_GATE_I2C_NAME, ( LCM_GATE_SLAVE_ADDR >> 1 ))};
    static struct i2c_client  * lcm_gate_i2c_client = NULL;
  #endif /* End.. !(BUILD_LK) */
#endif /* End.. !(FPGA_EARLY_PORTING) */
#endif /* End.. (LCM_GATE_DRIVER_C) */

/*****************************************************************************
** Function Prototype
******************************************************************************/
#if defined( LCM_GATE_DRIVER_C )
#if !defined( FPGA_EARLY_PORTING )
  #if !defined( BUILD_LK )
    static int lcm_gate_probe(struct i2c_client *client, const struct i2c_device_id *id);
    static int lcm_gate_remove(struct i2c_client *client);
  #endif /* End.. !(BUILD_LK) */
#endif /* End.. !(FPGA_EARLY_PORTING) */
#endif /* End.. (LCM_GATE_DRIVER_C) */

    GLOBAL void   lcm_gate_enable(int enable);
#if !defined( FPGA_EARLY_PORTING )
  #if !defined( BUILD_LK )
    GLOBAL int    lcm_gate_write_bytes(unsigned char addr,unsigned char value);
  #else
    GLOBAL int    lcm_gate_write_bytes(kal_uint8 addr, kal_uint8 value);
  #endif /* End.. !(BUILD_LK) */
#endif /* End.. !(FPGA_EARLY_PORTING) */

/*****************************************************************************
** Data Structure
******************************************************************************/
#if defined( LCM_GATE_DRIVER_C )
#if !defined( FPGA_EARLY_PORTING )
  #if !defined( BUILD_LK )
    struct lcm_gate_dev {
        struct i2c_client * client;
    };

    static const struct i2c_device_id   lcm_gate_id[] = {
        { LCM_GATE_I2C_NAME, 0 },
        { }
    };

    static struct i2c_driver lcm_gate_i2c_driver = {
        .id_table   = lcm_gate_id,
        .probe      = lcm_gate_probe,
        .remove     = lcm_gate_remove,
      //.detect     = lcm_gate_detect,
        .driver     = {
            .owner  = THIS_MODULE,
            .name   = "lcm_gate",
        },
    };

  #endif /* End.. !(BUILD_LK) */
#endif /* End.. !(FPGA_EARLY_PORTING) */
#endif /* End.. (LCM_GATE_DRIVER_C) */

  #if defined( GLOBAL )
    #undef    GLOBAL
  #endif

/*****************************************************************************
**
******************************************************************************/
#endif /* End.. !(LCM_GATE_DRIVER_H) */
