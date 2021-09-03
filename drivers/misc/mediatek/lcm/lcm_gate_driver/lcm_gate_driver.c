/*****************************************************************************
** LCM Gate Driver
**============================================================================
** Novatek NT50358
** Dual Channel DC-DC Converters
******************************************************************************/
#ifndef LCM_GATE_DRIVER_C
#define LCM_GATE_DRIVER_C

/*****************************************************************************
**
******************************************************************************/
#include  "lcm_gate_driver.h"
//[Arima_Lavender][RaymondLin] LCM power control pin control for SP run begin	
#include <linux/delay.h>
//[Arima_Lavender][RaymondLin] LCM power control pin control for SP run end
/*****************************************************************************
** Extern Area
******************************************************************************/


/*****************************************************************************
** Function
******************************************************************************/
void lcm_gate_enable(int enable)
{
//[Arima_Lavender][RaymondLin] add one LCM enable pin for SP run 20150402 begin
#if defined(LAVENDER)
   #if !defined( FPGA_EARLY_PORTING )
    mt_set_gpio_mode( GPIO_GATE_EP_PIN, GPIO_MODE_00 );
    mt_set_gpio_dir( GPIO_GATE_EP_PIN, GPIO_DIR_OUT );
    mt_set_gpio_mode( GPIO_GATE_EN_PIN, GPIO_MODE_00 );
    mt_set_gpio_dir( GPIO_GATE_EN_PIN, GPIO_DIR_OUT );

  if( TRUE == enable )
  	{
  	  mt_set_gpio_out( GPIO_GATE_EP_PIN, GPIO_OUT_ONE );
//[Arima_Lavender][RaymondLin] LCM power control pin control for SP run begin	  
	  	  mdelay( 1 );
//[Arima_Lavender][RaymondLin] LCM power control pin control for SP run end		  
         mt_set_gpio_out( GPIO_GATE_EN_PIN, GPIO_OUT_ONE );
  	}
  else
  	{
//[Arima_Lavender][RaymondLin] LCM power control pin control for SP run begin	
	mt_set_gpio_out( GPIO_GATE_EN_PIN, GPIO_OUT_ZERO );
		mdelay( 1 );
  	mt_set_gpio_out( GPIO_GATE_EP_PIN, GPIO_OUT_ZERO );
//[Arima_Lavender][RaymondLin] LCM power control pin control for SP run end	
  	}
   #endif
#else //Cosmos project
#if !defined( FPGA_EARLY_PORTING )
    mt_set_gpio_mode( GPIO_GATE_EN_PIN, GPIO_MODE_00 );
    mt_set_gpio_dir( GPIO_GATE_EN_PIN, GPIO_DIR_OUT );

  if( TRUE == enable )
    mt_set_gpio_out( GPIO_GATE_EN_PIN, GPIO_OUT_ONE );
  else
    mt_set_gpio_out( GPIO_GATE_EN_PIN, GPIO_OUT_ZERO );
#endif
#endif
//[Arima_Lavender][RaymondLin] add one LCM enable pin for SP run 20150402 end
}

#if !defined( FPGA_EARLY_PORTING )
#if !defined( BUILD_LK )
/**********************************************************
**
***********************************************************/
//static
int lcm_gate_write_bytes(unsigned char addr, unsigned char value)
{
int ret = 0;
struct i2c_client *client = lcm_gate_i2c_client;
char write_data[2]={0};

    write_data[0] = addr;
    write_data[1] = value;

    ret = i2c_master_send( client, write_data, 2 );
    if( ret < 0 )
      printk("lcm_gate write data fail !!\n");

    return ret ;
}

/**********************************************************
**
***********************************************************/
static int lcm_gate_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    printk("%s...\n", __func__ );

    printk("TPS(lcm_gate): info ==> name = %s, addr = 0x%02X\n",client->name,client->addr);
    lcm_gate_i2c_client = client;

    return 0;
}

/**********************************************************
**
***********************************************************/
static int lcm_gate_remove(struct i2c_client *client)
{
    printk("%s...\n", __func__ );

    lcm_gate_i2c_client = NULL;
    i2c_unregister_device(client );

    return 0;
}

/**************************************************************************
** module load/unload record keeping
***************************************************************************/
static int __init lcm_gate_i2c_init(void)
{
    printk("%s...\n", __func__ );

    i2c_register_board_info( LCM_GATE_I2C_BUS, &lcm_gate_board_info, 1 );
    printk("lcm_gate_i2c_init board info\n");

    if( i2c_add_driver( &lcm_gate_i2c_driver ))
    {
      printk("lcm_gate add I2C driver error\n");
      return -1;
    }
    else
    {
      printk("lcm_gate add I2C driver success\n");
    }

    return 0;
}

static void __exit lcm_gate_i2c_exit(void)
{
    printk("%s...\n", __func__ );

    i2c_del_driver( &lcm_gate_i2c_driver );
}


module_init(lcm_gate_i2c_init);
module_exit(lcm_gate_i2c_exit);

MODULE_AUTHOR("Yuting Shih");
MODULE_DESCRIPTION("MTK LCM GATE I2C Driver");
MODULE_LICENSE("GPL");

#else
/**********************************************************
**
***********************************************************/
//static
int lcm_gate_write_bytes(kal_uint8 addr, kal_uint8 value)
{
struct mt_i2c_t lcm_gate_i2c;
kal_uint32  ret_code = I2C_OK;
kal_uint16  len = 2;
kal_uint8   write_data[2];

    write_data[0] = addr;
    write_data[1] = value;

    lcm_gate_i2c.id   = LCM_GATE_I2C_BUS;   //I2C0;
  /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    lcm_gate_i2c.addr = ( LCM_GATE_SLAVE_ADDR >> 1 );
    lcm_gate_i2c.mode = ST_MODE;
    lcm_gate_i2c.speed = 100;

    ret_code = i2c_write( &lcm_gate_i2c, write_data, len );
  //dprintf("%s: i2c_write: ret_code: %d\n", __func__, ret_code );

    return ret_code;
}

#endif /* End.. !(BUILD_LK) */
#endif /* End.. !(FPGA_EARLY_PORTING) */

/*****************************************************************************
**
******************************************************************************/
#undef LCM_GATE_DRIVER_C
#endif
