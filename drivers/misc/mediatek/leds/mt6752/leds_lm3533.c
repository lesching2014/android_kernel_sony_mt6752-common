#ifndef LED_LM3533_C
#define LED_LM3533_C
/*******************************************************************************
**
********************************************************************************/
#include "leds_lm3533.h"
// <<< 2016/02/26-dangyiwang, Proting lm3533 from cosmos/lavender L to M
//[Arima_5888][RaymondLin] Seperate INX and Truly for bacjkight current control due to HW's request begin
#if defined(LAVENDER)
#include <mach/mt_gpio.h>
#define GPIO_LCM_ID_PIN 97
#endif
//[Arima_5888][RaymondLin] Seperate INX and Truly for bacjkight current control due to HW's request end
// >>> 2016/02/26-dangyiwang, Proting lm3533 from cosmos/lavender L to M
static int lm3533_debug_enable = 1;
#define   LEDS_DEBUG(format, args...) do{ \
                if(lm3533_debug_enable) \
                {\
                    printk(KERN_DEBUG format,##args);\
                }\
              }while(0)

/*******************************************************************************
**
********************************************************************************/
//<2015/06/17-youchihwang, add i2c address probe
static unsigned short lm3533_i2c_auto_probe_address[] = {0x36, 0x38}; //I2C Address 0x36 : LM3533TME-40
                                                                      //I2C Address 0x38 : LM3533TME-40A
//>2015/06/17-youchihwang,
/*******************************************************************************
**
********************************************************************************/
static struct i2c_driver lm3533_i2c_driver =
{
  .driver = {
    .name   = "lm3533",
    .owner  = THIS_MODULE,
  },
  .id_table = lm3533_i2c_ids,
  .probe    = lm3533_i2c_probe,
  .remove   = lm3533_i2c_remove,
};

// <<<<<----- 2015/04/28-youchihwang
// DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
// ALPS02045680
static int pattern_id = 0;
#define LED_TRACE_INFO
// ----->>>>> 2015/04/28-youchihwang
// DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
// ALPS02045680
/*******************************************************************************
**
********************************************************************************/
static int lm3533_write_reg(u8 reg, u8 writeData)
{
u8    data_buf[2] = { 0x00 };
int   ret = 0;

  data_buf[0] = reg;
  data_buf[1] = writeData;
// [L][I2C][akenhsu] Modify I2C speed of LED Control IC to 400KHz 20150408 BEGIN
#if defined(LAVENDER)
  lm3533_i2c_client->timing = 400; // I2C 400KHz
#endif // LAVENDER
// [L][I2C][akenhsu] 20150408 END
  ret = i2c_master_send( lm3533_i2c_client, (const char*)data_buf, 2 );
  if( ret < 0 )
  {
    LEDS_DEBUG("Send command(0x%02X) error!!\n", data_buf[0]);
    return -EFAULT;
  }

  return 0;
} /* End.. lm3533_write_reg() */

/*******************************************************************************
**
********************************************************************************/
static int lm3533_read_reg(u8 reg, u8 *returnData)
{
u8    data_buf[2] = { 0x00 };
int   ret = 0;

  data_buf[0] = reg;
// [L][I2C][akenhsu] Modify I2C speed of LED Control IC to 400KHz 20150408 BEGIN
#if defined(LAVENDER)
  lm3533_i2c_client->timing = 400; // I2C 400KHz
#endif // LAVENDER
// [L][I2C][akenhsu] 20150408 END
  ret = i2c_master_send( lm3533_i2c_client, (const char*)data_buf, 1 );
  if( ret < 0 )
  {
    LEDS_DEBUG("Send command(0x%02X) error!!\n", data_buf[0]);
    return -EFAULT;
  }

  ret = i2c_master_recv( lm3533_i2c_client, (char*)returnData, 1 );
  if( ret < 0 )
  {
    LEDS_DEBUG("Read reg(0x%02X) data error!!\n", data_buf[0]);
    return -EFAULT;
  }

  return 0;
} /* End.. lm3533_read_reg() */

/*******************************************************************************
**
********************************************************************************/
static void lm3533_device_enable( int enable )
{
#if defined( LED_TRACE_INFO )
    printk("[LED] %s: %d \n", __func__, enable );
#endif
    mt_set_gpio_mode( GPIO_LCM_LED_EN, GPIO_LCM_LED_EN_M_GPIO );
    mt_set_gpio_dir( GPIO_LCM_LED_EN, GPIO_DIR_OUT );
    if( TRUE == enable )
      mt_set_gpio_out( GPIO_LCM_LED_EN, GPIO_OUT_ONE );
    else
      mt_set_gpio_out( GPIO_LCM_LED_EN, GPIO_OUT_ZERO );
} /* End.. lm3533_device_enable() */

/*************************************************************************
** Show last android LED when turning off all TS LED
**************************************************************************/
void led_switch(void)
{
struct nled_setting   nled_tmp_setting = { 0, 0, 0 };
struct led_trigger  * p_led_trigger;
int   idx = MT65XX_LED_TYPE_RED;

#if defined( LED_TRACE_INFO )
  printk("[LED] lm3533 %s \n", __func__ );
#endif

  for( idx = MT65XX_LED_TYPE_RED; idx <= MT65XX_LED_TYPE_BLUE; idx++ )
  {
    printk("led_switch g_leds_data[%d] = %d\n", idx, g_leds_data[idx]->level );
    if( g_leds_data[idx]->level != 0 )
    {
      p_led_trigger = g_leds_data[idx]->cdev.trigger;

    //if( p_led_trigger == NULL )
    //  printk("p_led_trigger == NULL\n");
    //else
    //  printk("name=%s\n", p_led_trigger->name);
    //printk("delay_on=%d,delay_off=%d\n", g_leds_data[idx]->delay_on, g_leds_data[idx]->delay_off);

      if(( p_led_trigger != NULL ) && (!strcmp( p_led_trigger->name, "timer" ))
      && ( g_leds_data[idx]->delay_on != 0 ) && ( g_leds_data[idx]->delay_off != 0 ))
      {
        nled_tmp_setting.nled_mode      = NLED_BLINK;
        nled_tmp_setting.blink_off_time = g_leds_data[idx]->delay_off;
        nled_tmp_setting.blink_on_time  = g_leds_data[idx]->delay_on;
        if( idx == MT65XX_LED_TYPE_RED )
          blink_set_lm3533( MT65XX_LED_LM3533_L3, &nled_tmp_setting, g_leds_data[idx]->level );
        else if( idx == MT65XX_LED_TYPE_GREEN )
          blink_set_lm3533( MT65XX_LED_LM3533_L2, &nled_tmp_setting, g_leds_data[idx]->level );
        else if( idx == MT65XX_LED_TYPE_BLUE )
          blink_set_lm3533( MT65XX_LED_LM3533_L1, &nled_tmp_setting, g_leds_data[idx]->level );
      }
      else
      {
        if( idx == MT65XX_LED_TYPE_RED )
          brightness_set_lm3533( MT65XX_LED_LM3533_L3, g_leds_data[idx]->level );
        else if( idx == MT65XX_LED_TYPE_GREEN )
          brightness_set_lm3533( MT65XX_LED_LM3533_L2, g_leds_data[idx]->level );
        else if( idx == MT65XX_LED_TYPE_BLUE )
          brightness_set_lm3533( MT65XX_LED_LM3533_L1, g_leds_data[idx]->level );
      }
    }
  }
} /* End.. led_switch() */

/*************************************************************************
** Add VALUE_BUTTON_3 and VALUE_PATTERN_2 for LED illumination
**************************************************************************/
void led_pulse_work_callback(struct work_struct *work)
{
u8  enablevalue = 0;

#if defined( LED_TRACE_INFO )
  printk("[LED] lm3533 %s \n", __func__ );
#endif
  if( is_keep_light )
  {
  /* Disable pattern */
    lm3533_write_reg( 0x28, 0x00 );
  }
  else
  {
    lm3533_read_reg( 0x27, &enablevalue );

  /* Disable R/G/B LED */
    enablevalue = enablevalue & 0x03;
    lm3533_write_reg( 0x27, enablevalue );

  /* Show last android LED when turning off all TS LED */
    if( pattern_enable == KAL_FALSE )
      led_switch();

    pattern_enable = KAL_FALSE;
  }
} /* End.. led_pulse_work_callback() */

static enum hrtimer_restart led_pulse_timer_func(struct hrtimer *timer)
{
  queue_work( led_pulse_workqueue, &led_pulse_work );

  return HRTIMER_NORESTART;
} /* End.. led_pulse_timer_func() */

/*************************************************************************
** Add flash_mode property for LED VALUE_BUTTON_2 type
** Fix led illumination issues for SONY requirements
**      static int pattern_set_lm3533(struct cust_mt65xx_led *cust, int pattern_id)
** Connect native(lights.c) to driver(leds.c)
** Add falsh_mode for lm3533 light
**************************************************************************/
static int pattern_set_lm3533(struct cust_mt65xx_led *cust, int pattern_id, int flash_mode, u8 red_data, u8 green_data, u8 blue_data)
{
//u8  data_buf[2];
u8  enablevalue = 0, data_idx = 0, idx;
// <<<<<----- 2015/04/28-youchihwang
// DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
// ALPS02045680
return 0;
// ----->>>>> 2015/04/28-youchihwang
// DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
// ALPS02045680

  printk("[LED]LM3533 pattern_set_lm3533 pattern_id=%d, red=0x%02X, green=0x%02X, blue=0x%02X\n",
        pattern_id, red_data, green_data, blue_data );

  mutex_lock( &lm3533_i2c_access );

  /* Disable R/G/B */
  lm3533_read_reg( 0x27, &enablevalue );
  printk("[LED]LM3533 pattern_set_lm3533 enablevalue=0x%02X\n", enablevalue );
  enablevalue = enablevalue & 0x03;
  lm3533_write_reg( 0x27, enablevalue );

  is_keep_light     = KAL_FALSE;
  keep_light_color  = 0x00;
  hrtimer_cancel( &led_pulse_timer );

  /* Show last android LED when turning off all TS LED */
  if(( 0 == blue_data ) && ( 0 == green_data ) && ( 0 == red_data ))
  {
    pattern_enable = KAL_FALSE;
    lm3533_write_reg( 0x28, 0x00 );
    queue_work( led_pulse_workqueue, &led_pulse_work );
    mutex_unlock( &lm3533_i2c_access );
    return 0;
  }
  pattern_enable = KAL_TRUE;

  if( blue_data )
  {
    enablevalue = enablevalue | 0x04;
    lm3533_write_reg( 0x42, blue_data );  /* Set current for blue LED */
  }

  if( green_data )
  {
    enablevalue = enablevalue | 0x08;
    lm3533_write_reg( 0x43, green_data ); /* Set current for green LED */
  }

  if( red_data )
  {
    enablevalue = enablevalue | 0x10;
    lm3533_write_reg( 0x44, red_data ); /* Set current for red LED */
  }

  /* Set pattern data */
  switch( pattern_id )
  {
    case 1:
    {
      if( flash_mode == 0 )
        data_idx = 2;
    } break;
    case 4:
    {
      data_idx = 3;
    } break;
    case 5:
    {
      data_idx = 1;
    } break;
    case 6:
    {
      data_idx = 4;
    } break;
  }

  for( idx = 0; idx < pattern_data[data_idx].count; idx++ )
  {
    lm3533_write_reg( pattern_data[data_idx].config_data[idx].cmd, pattern_data[data_idx].config_data[idx].data );
  }

  lm3533_write_reg( 0x28, 0x15 );
  lm3533_write_reg( 0x27, enablevalue );

  /* Enable timer to turn off R/G/B LED */
  switch( data_idx )
  {
    case 2:
    {
      is_keep_light = KAL_TRUE;
      keep_light_color = enablevalue;
      hrtimer_start( &led_pulse_timer, ktime_set( 0, (500 * 1000000)), HRTIMER_MODE_REL );
    } break;
    case 3:
    {
      hrtimer_start( &led_pulse_timer, ktime_set( 1, (500 * 1000000)), HRTIMER_MODE_REL );
    } break;
    case 4:
    {
      hrtimer_start( &led_pulse_timer, ktime_set( 4, (500 * 1000000)), HRTIMER_MODE_REL );
    } break;
  }

  mutex_unlock( &lm3533_i2c_access );

  return 0;
} /* End.. pattern_set_lm3533() */

/*************************************************************************
** Modify R/G/B LED's level can be controlled by android
**************************************************************************/
//static int blink_set_lm3533(int led_num, struct nled_setting* led, int level)
int blink_set_lm3533(int led_num, struct nled_setting* led, int level)
{
// <<< 2016/05/12-dangyiwang, Feature FP019344 Illumination. update notification blink function
        struct i2c_client *client = lm3533_i2c_client;
        u32 low_time = 0, high_time = 0;
        u8  data_buf[2], patternvalue = 0;

#if defined( LED_TRACE_INFO )
        printk("[LED] %s: led_num = %d, level = %d\n", __func__, led_num, level );
#else
        printk("[LED]LM3533 blink_set_lm3533 led_num=%d\n", led_num );
#endif

        mutex_lock( &lm3533_i2c_access );
        lm3533_device_enable( TRUE );

        if (1000 >= led->blink_on_time)
                high_time = led->blink_on_time / 16;
        else if (10000 >= led->blink_on_time)
                high_time = ((led->blink_on_time - 1000) / 131) + 0x3C;
        else
                high_time = ((led->blink_on_time - 10000) / 524) + 0x7F;

        if (0xFF < high_time)
                high_time = 0xFF;


        if (1000 >= led->blink_off_time)
                low_time = led->blink_off_time / 16;
        else if (10000 >= led->blink_off_time)
                low_time = ((led->blink_off_time - 1000) / 131) + 0x3C;
        else
                low_time = ((led->blink_off_time - 10000) / 524) + 0x7F;


        if (0xFF < low_time)
                low_time = 0xFF;

        if (level > 0xFF)
                level = 0xFF;

        data_buf[0] = 0x28;
        i2c_master_send (client, (const char*) data_buf, 1);
        i2c_master_recv (client, (char*) &patternvalue, 1);
        printk("[LED] patternvalue = 0x%X\n", patternvalue);


        lm3533_write_reg (0x28, 0);  // disable all led pattern to re-blink at the same time

        if (led_num == MT65XX_LED_LM3533_L3) {
                
                //
                // set red led related parameters
                //
                lm3533_write_reg (0x90, 0);    // Delay time
                lm3533_write_reg (0x91, (u8) low_time);    // Low time
                lm3533_write_reg (0x92, (u8) high_time);    // High time
                lm3533_write_reg (0x93, 0);    // Low level brightness
                lm3533_write_reg (0x94, 0);    // Rise time
                lm3533_write_reg (0x95, 0);    // Fall time
                //lm3533_write_reg (0x44, level);

                patternvalue |= 0x10;
        }
        else if (led_num == MT65XX_LED_LM3533_L2) {
                
                //
                // set green led related parameters
                //
                lm3533_write_reg (0x80, 0);    // Delay time
                lm3533_write_reg (0x81, (u8) low_time);    // Low time
                lm3533_write_reg (0x82, (u8) high_time);    // High time
                lm3533_write_reg (0x83, 0);    // Low level brightness
                lm3533_write_reg (0x84, 0);    // Rise time
                lm3533_write_reg (0x85, 0);    // Fall time
                //lm3533_write_reg (0x43, level);

                patternvalue |= 0x04;
        }
        else if (led_num == MT65XX_LED_LM3533_L1) {
                
                //
                // set blue led related parameters
                //
                lm3533_write_reg (0x70, 0);    // Delay time
                lm3533_write_reg (0x71, (u8) low_time);    // Low time
                lm3533_write_reg (0x72, (u8) high_time);   // High time
                lm3533_write_reg (0x73, 0);    // Low level brightness
                lm3533_write_reg (0x74, 0);    // Rise time
                lm3533_write_reg (0x75, 0);    // Fall time
                //lm3533_write_reg (0x42, level);

                patternvalue |= 0x01;
        }

        printk ("[LED] led_num = 0x%X, low_time = 0x%X, high_time = 0x%X\n", led_num, low_time, high_time);
        printk ("[LED] level = 0x%X, patternvalue = 0x%X\n", level, patternvalue);
        lm3533_write_reg (0x28, patternvalue); /* Pattern enable */
        mutex_unlock (&lm3533_i2c_access);

        return 0;
// >>> 2016/05/12-dangyiwang, Feature FP019344 Illumination. update notification blink function
} /* End.. blink_set_lm3533() */

/*************************************************************************
** Using SONY brightness curve
**************************************************************************/
// <<< 2016/02/26-dangyiwang, Proting lm3533 from cosmos/lavender L to M
#if defined( USING_MAP_TABLE_LM3533 )
//[Arima_Lavender][RaymondLin] Adjust Lavender Backlight curve begin
#if defined(LAVENDER)
static int brightness_map_sony_lm3533(int level)
{
    int   vRet = 0;

    if( level == 0 )
        vRet = 0;
    else if( level < 10 )
        vRet = 3;
	else if( level< 150)
	   vRet = (level-10)*(104-3)/(149-10)+3;
    else
        vRet = (level-150)*(255-105)/(255-150)+105;
    
    //printk("[Steven] backlight level =%d, lm3533 level = %d \n", level, vRet);

    return vRet;
} /* End.. brightness_map_sony_lm3533() */
#else
//<2015/05/12-stevenchen, Adjust LCM backlight curve to linear
static int brightness_map_sony_lm3533(int level)
{
        int    vRet = 0;

        if (level == 0)
                vRet = 0;
        else if( level < 10 )
                vRet = 3;
        else
                //<2015/08/20-youchihwang, adjust LCM backlight slope that max level 255 map to vRet 220 (440ns)
				#if defined(COSMOS)
                        vRet = (level-10)*(220-3)/(255-10)+3;
				#elif  defined(LAVENDER)
                        vRet = (level-10)*(255-3)/(255-10)+3;
				#endif
                //>2015/08/20-youchihwang, adjust LCM backlight slope that max level 255 map to vRet 220 (440ns)
				
        //printk("[Steven] backlight level =%d, lm3533 level = %d \n", level, vRet);

        return vRet;
} /* End.. brightness_map_sony_lm3533() */
//>2015/05/12-stevenchen
#endif
//[Arima_Lavender][RaymondLin] Adjust Lavender Backlight curve end
#endif /* End.. (brightness_map_sony_lm3533) */
// >>> 2016/02/26-dangyiwang, Proting lm3533 from cosmos/lavender L to M
/*************************************************************************
** Fix backlight issue
**************************************************************************/
int brightness_set_lm3533(int led_num, int level)
{
// <<< 2016/05/12-dangyiwang, Feature FP019344 Illumination. update notification blink function
        struct i2c_client *client = lm3533_i2c_client;
        u8    enablevalue = 0;
        u8    patternvalue = 0;

#if defined (LED_TRACE_INFO)
        printk("[LED] %s: led_num = %d, level = %d\n", __func__, led_num, level );
#endif

        LEDS_DEBUG ("[LED]LM3533#%d:%d\n", led_num, level);
        mutex_lock (&lm3533_i2c_access);
        lm3533_device_enable (TRUE);
        lm3533_read_reg (0x27, &enablevalue);

        lm3533_read_reg (0x28, &patternvalue);
       
        if (client == NULL) {
                LEDS_DEBUG ("i2c client is null!!\n");
                mutex_unlock (&lm3533_i2c_access);
                return 0;
        }

        if ((led_num == MT65XX_LED_LM3533_L1) || (led_num == MT65XX_LED_LM3533_L2) || (led_num == MT65XX_LED_LM3533_L3)) {


                lm3533_write_reg (0x27, (enablevalue & 0x03));// disable all led brightness to re-blink at the same time

                if (level == 0) {
                        enablevalue &= ~(1 << led_num);
                        patternvalue &= ~(0x3 << ((led_num - 2) << 1));
                        lm3533_write_reg (0x28, patternvalue);
                }
                else {
	                if (level > 0xFF)
  	                        level = 0xFF;
                        enablevalue |= (1 << led_num);
                }
                lm3533_write_reg ((0x40 + led_num), level); /* B/G/R led */
        }
        else if ((led_num == MT65XX_LED_LM3533_H1) || (led_num == MT65XX_LED_LM3533_H2)) {
                
#if defined (USING_MAP_TABLE_LM3533)
        level = brightness_map_sony_lm3533 (level);
#else
                if (level >= 0xFF)
                        level = 0xFF;
                else if (level == 0)
                        level = 0;
                else
                        level = 125 + (level >> 1);
#endif

//<2015/05/12-stevenchen, Adjust LCM backlight curve to linear
    //<2015/02/13-youchihwang, adjust LCM backlight lowest current
    //if (level == 20)
    //    level = 3;
    //>2015/02/13-youchihwang, adjust LCM backlight lowest current
//>2015/05/12-stevenchen
                if (level == 0)
                        enablevalue &= 0xFC;
                else
                        enablevalue |= 0x03;

                lm3533_write_reg (0x40, level);  /* Bank A */
                lm3533_write_reg (0x41, level);  /* Bank B */
        }

    
        lm3533_write_reg (0x27, enablevalue);

#if defined( LED_TRACE_INFO )
        printk("[LED]LM3533 2 enablevalue = 0x%02X\n", enablevalue );
#else
        LEDS_DEBUG("[LED]LM3533 2 enablevalue = 0x%x\n", enablevalue );
#endif

        mutex_unlock (&lm3533_i2c_access);

        return 0;
// >>> 2016/05/12-dangyiwang, Feature FP019344 Illumination. update notification blink function
} /* End.. brightness_set_lm3533() */

/*************************************************************************
**
**************************************************************************/
static int lm3533_reg_init(struct lm3533 *lm3533)
{
// <<< 2016/02/26-dangyiwang, Proting lm3533 from cosmos/lavender L to M
//[Arima_5888][RaymondLin] Seperate INX and Truly for bacjkight current control due to HW's request begin
#if defined(LAVENDER)
int lcm_id=0;
#endif
//[Arima_5888][RaymondLin] Seperate INX and Truly for bacjkight current control due to HW's request end
// >>> 2016/02/26-dangyiwang, Proting lm3533 from cosmos/lavender L to M
#if defined( LED_TRACE_INFO )
  printk("[LED] %s ...\n", __func__ );
#endif

//<2014/12/10-stevenchen, [All] Rollback backlight current to 19.4mA according to HW's request
//<2014/12/05-stevenchen, [All] Pull up backlight current to 20.2mA
  /* LCM backlight */
 //[Arima_lavender][RaymondLin] modify backlight current to 21mA according to HW's request begin 
  #if defined(LAVENDER)
// <<< 2016/02/26-dangyiwang, Proting lm3533 from cosmos/lavender L to M  
//[Arima_5888][RaymondLin] Seperate INX and Truly for bacjkight current control due to HW's request begin
   lcm_id = mt_get_gpio_in(GPIO_LCM_ID_PIN);
   if(lcm_id == 1)
    {
    	printk("[LED][Raymond][Truly LCM] \n");      
		lm3533_write_reg( 0x1F, 0x12 );  // 0x14:21mA, 0x13:20.2mA, 0x12:19.4mA   /* Set bank A full-scale current (5mA ~ 29.8mA) */
    }
    else
    {
    	printk("[LED][Raymond][Innolux LCM] \n");      
//[Arima_lavender][RaymondLin] modify Innolux backlight current to 19.4mA according to HW's request begin
		lm3533_write_reg( 0x1F, 0x12 );  // 0x14:21mA, 0x13:20.2mA, 0x12:19.4mA   /* Set bank A full-scale current (5mA ~ 29.8mA) */        
//[Arima_lavender][RaymondLin] modify Innolux backlight current to 19.4mA according to HW's request end
  
    }
//[Arima_5888][RaymondLin] Seperate INX and Truly for bacjkight current control due to HW's request end
// >>> 2016/02/26-dangyiwang, Proting lm3533 from cosmos/lavender L to M
  #else
  lm3533_write_reg( 0x1F, 0x12 );  // 0x13:20.2mA, 0x12:19.4mA   /* Set bank A full-scale current (5mA ~ 29.8mA) */
  #endif  
//[Arima_lavender][RaymondLin] modify backlight current to 21mA according to HW's request end  
#if defined( USING_MAP_TABLE_LM3533 )
  lm3533_write_reg( 0x1A, 0x0A );  /* Bank A & Bank B => brightness mode, linear mode */
#endif
  lm3533_write_reg( 0x40, 0xA0 ); /* Brightness value for bank A */
 //[Arima_lavender][RaymondLin] modify backlight current to 21mA according to HW's request begin 
  #if defined(LAVENDER)
// <<< 2016/02/26-dangyiwang, Proting lm3533 from cosmos/lavender L to M
  //[Arima_5888][RaymondLin] Seperate INX and Truly for bacjkight current control due to HW's request begin
  if(lcm_id == 1)
    {
    	printk("[LED][Raymond][Truly LCM] \n");      
		lm3533_write_reg( 0x20, 0x12 ); // 21mA   /* Set bank B full-scale current (5mA ~ 29.8mA) */
    }
    else
    {
    	printk("[LED][Raymond][Innolux LCM] \n");      
//[Arima_lavender][RaymondLin] modify Innolux backlight current to 19.4mA according to HW's request begin
		lm3533_write_reg( 0x20, 0x12 ); // 21mA   /* Set bank B full-scale current (5mA ~ 29.8mA) */     
//[Arima_lavender][RaymondLin] modify Innolux backlight current to 19.4mA according to HW's request end
    }
  //[Arima_5888][RaymondLin] Seperate INX and Truly for bacjkight current control due to HW's request end
// >>> 2016/02/26-dangyiwang, Proting lm3533 from cosmos/lavender L to M
  #else
  lm3533_write_reg( 0x20, 0x12 ); // 19.4mA   /* Set bank B full-scale current (5mA ~ 29.8mA) */
  #endif
 //[Arima_lavender][RaymondLin] modify backlight current to 21mA according to HW's request end
  lm3533_write_reg( 0x41, 0xA0 ); /* Brightness value for bank B */
//[Arima_Lavender][RaymondLin] pull up Boost OVP to 32V begin
#if defined(LAVENDER)
  lm3533_write_reg( 0x2C, 0x0D ); /* Boost OVP 0x0D:32V, 0x0B:24V */
#else
  lm3533_write_reg( 0x2C, 0x0B ); /* Boost OVP 0x0D:32V, 0x0B:24V */
#endif  
//[Arima_Lavender][RaymondLin] pull up Boost OVP to 32V end
//>2014/12/05-stevenchen
//>2014/12/10-stevenchen

  // <<< 2016/03/23-dangyiwang. Enabling CABC (PWM Input for Content Adjustable Brightness Control)
//[Arima_Lavender][RaymondLin] Enable CABC function 20150319 begin
//#if defined(LAVENDER)
//   lm3533_write_reg( 0x14, 0x39 ); /*  */
//   lm3533_write_reg( 0x10, 0x90 ); /*  */
//#endif  
//[Arima_Lavender][RaymondLin] Enable CABC function 20150319 end
  lm3533_write_reg (0x14, 0x1);
  lm3533_write_reg (0x15, 0x1);
  // >>> 2016/03/30-dangyiwang. Enabling CABC (PWM Input for Content Adjustable Brightness Control)

  /* Blue LED */
/*[Lavender][LED][JasonHsing] Increase LED current of Lavender 20150302 BEGIN*/
#if defined(COSMOS)
/*[Lavender][LED][JasonHsing] 20150302 END*/

  // <<<<<----- 2015/04/28-youchihwang
  // DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
  // ALPS02045680
  lm3533_write_reg( 0x21, 0x01 ); // 5.8mA   /* Bank C full-scale current */
  // ----->>>>> 2015/04/28-youchihwang
  // DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
  // ALPS02045680

  
/*[Lavender][LED][JasonHsing] Increase LED current of Lavender 20150302 BEGIN*/
#elif defined(LAVENDER)
  lm3533_write_reg( 0x21, 0x13 ); // 20.2mA  /* Bank C full-scale current */
#endif
/*[Lavender][LED][JasonHsing] 20150302 END*/

  lm3533_write_reg( 0x1B, 0x04 ); /* Bank C => disable pattern generator, brightness mode, linear mode */
  lm3533_write_reg( 0x42, 0xFF ); /* Brightness value for bank C */

  /* Green LED */
/*[Lavender][LED][JasonHsing] Increase LED current of Lavender 20150302 BEGIN*/
#if defined(COSMOS)
/*[Lavender][LED][JasonHsing] 20150302 END*/

  // <<<<<----- 2015/04/28-youchihwang
  // DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
  // ALPS02045680
  lm3533_write_reg( 0x22, 0x010 ); // 17.8mA /* bank D full-scale current */
  // ----->>>>> 2015/04/28-youchihwang
  // DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
  // ALPS02045680

/*[Lavender][LED][JasonHsing] Increase LED current of Lavender 20150302 BEGIN*/
#elif defined(LAVENDER)
  lm3533_write_reg( 0x22, 0x13 ); // 20.2mA   /* bank D full-scale current */
#endif
/*[Lavender][LED][JasonHsing] 20150302 END*/

  lm3533_write_reg( 0x1C, 0x04 ); /* Bank D => disable pattern generator, brightness mode, linear mode */
  lm3533_write_reg( 0x43, 0xFF ); /* Brightness value for bank D */

  /* Red LED */
/*[Lavender][LED][JasonHsing] Increase LED current of Lavender 20150302 BEGIN*/
#if defined(COSMOS)
/*[Lavender][LED][JasonHsing] 20150302 END*/

  // <<<<<----- 2015/04/28-youchihwang
  // DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
  // ALPS02045680
  lm3533_write_reg( 0x23, 0x06 ); // 9.8mA /* bank E full-scale current */
  // ----->>>>> 2015/04/28-youchihwang
  // DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
  // ALPS02045680

/*[Lavender][LED][JasonHsing] Increase LED current of Lavender 20150302 BEGIN*/
#elif defined(LAVENDER)
  lm3533_write_reg( 0x23, 0x13 ); // 20.2mA  /* bank E full-scale current */
#endif
/*[Lavender][LED][JasonHsing] 20150302 END*/

  lm3533_write_reg( 0x1D, 0x04 ); /* bank E => disable pattern generator, brightness mode, linear mode */
  lm3533_write_reg( 0x44, 0xFF ); /* brightness value for bank E */

#if defined( MTK_KERNEL_POWER_OFF_CHARGING )
  if(( KERNEL_POWER_OFF_CHARGING_BOOT != get_boot_mode())
  && ( LOW_POWER_OFF_CHARGING_BOOT != get_boot_mode()))
#endif
  {
  	// <<< 2016/05/18-dangyiwang, remove breathing light when IC initialization
    /* Blue LED */
    //lm3533_write_reg( 0x71, 0xFF ); /* low-time */
    //lm3533_write_reg( 0x74, 0x02 ); /* rise-time */
    //lm3533_write_reg( 0x75, 0x03 ); /* fall-time */

    /* Green LED */
    //lm3533_write_reg( 0x81, 0xFF ); /* low-time */
    //lm3533_write_reg( 0x84, 0x02 ); /* rise-time */
    //lm3533_write_reg( 0x85, 0x03 ); /* fall-time */

    /* Red LED */
    //lm3533_write_reg( 0x91, 0xFF ); /* low-time */
    //lm3533_write_reg( 0x94, 0x02 ); /* rise-time */
    //lm3533_write_reg( 0x95, 0x03 ); /* fall-time */

    //lm3533_write_reg( 0x28, 0x15 );
	// >>> 2016/05/18-dangyiwang, remove breathing light when IC initialization

    //<2014/12/19-youchihwang, off led when off mode charging
    lm3533_write_reg (0x27, 0x03 );
    //<2014/12/19-youchihwang, off led when off mode charging

  /* Fix no LED indicator when rebooting with USB / AC charger */
    is_show_power_on_pulse = KAL_TRUE;
  }

  return 0;
} /* End.. lm3533_device_init() */

/*************************************************************************
**
**************************************************************************/
//<2015/06/17-youchihwang, add i2c address probe
/**
 * lm3533_i2c_auto_probe - auto probe the i2c addresses
 * @client: i2c device
 *
 * Auto probing the i2c address of the array lm3533_i2c_auto_probe_address.
 */
static int lm3533_i2c_auto_probe(struct i2c_client *client)
{
        int ret                   = 0; //return value
        u8  chip_register_address = 0x40;
        u8  temp                  = 0;

        printk("[LEDS LM3533] kernel : lm3533_i2c_auto_probe + \n");
        for (temp = 0; temp < ARRAY_SIZE(lm3533_i2c_auto_probe_address); temp++) {
                client->addr = lm3533_i2c_auto_probe_address[temp];
                printk("[LED] kernel : auto probe address : 0x%x\n", client->addr);

                //test lm3533 brightness register A 0x40 whether can be accessed
                chip_register_address = 0x40;
                ret = i2c_master_send(client, (const char*)&chip_register_address, 1);
                printk("[LED] kernel : auto probe address 0x%x result : 0x%x\n", client->addr, ret);

                if (ret > 0) {
                        printk("[LED] kernel Valid lm3533 i2c address : 0x%x\n", client->addr);
                        printk("[LED] kernel 0x36 : LM3533TME-40  I2C Address \n");
                        printk("[LED] kernel 0x38 : LM3533TME-40A I2C Address \n");
                        printk("[LEDS LM3533] kernel : lm3533_i2c_auto_probe - \n");
                        return ret;
                }
        }
        printk("[LEDS LM3533] kernel : lm3533_i2c_auto_probe - \n");

        return ret;
}
//>2015/06/17-youchihwang
/*************************************************************************
**
**************************************************************************/
static int lm3533_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
        struct lm3533     *lm3533;
        struct i2c_client *new_client;
        int               ret;

        printk("[LED] %s ...\n", __func__ );

        lm3533 = kzalloc( sizeof(struct lm3533), GFP_KERNEL );
        if (!lm3533)
                return -ENOMEM;

        //<2015/06/17-youchihwang, add i2c address probe
        if (!(lm3533_i2c_auto_probe(i2c) > 0))
                printk("[LED] kernel : There is no valid lm3533 \n");
        //>2015/06/17-youchihwang, add i2c address probe

        lm3533->dev = &i2c->dev;
        lm3533->i2c = i2c;  //obj->client = client;
        new_client  = lm3533->i2c;

        // [L][I2C][akenhsu] Modify I2C speed of LED Control IC to 400KHz 20150408 BEGIN
        #if defined(LAVENDER)
                new_client->timing = 400; // I2C 400KHz
        #endif // LAVENDER
        // [L][I2C][akenhsu] 20150408 END

        i2c_set_clientdata (new_client, lm3533);
        lm3533_i2c_client = new_client;

        /*
        if (ret = lm3533_create_attr (&mt65xx_leds_driver.driver)) {
                LEDS_DEBUG("create attribute ret = %d\n", ret);
        }
        */

        #if 0
                lm3533_device_enable (TRUE);
        #else
                mt_set_gpio_mode (GPIO_LCM_LED_EN, GPIO_LCM_LED_EN_M_GPIO );
                mt_set_gpio_dir (GPIO_LCM_LED_EN, GPIO_DIR_OUT );
                mt_set_gpio_out (GPIO_LCM_LED_EN, GPIO_OUT_ZERO );
                mdelay( 1 );
                mt_set_gpio_out( GPIO_LCM_LED_EN, GPIO_OUT_ONE );
        #endif

        /* [5860] Add VALUE_BUTTON_3 and VALUE_PATTERN_2 for LED illumination */
        led_pulse_workqueue = create_singlethread_workqueue( "led_pulse" );
        INIT_WORK( &led_pulse_work, led_pulse_work_callback );
        hrtimer_init( &led_pulse_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
        led_pulse_timer.function = led_pulse_timer_func;

        ret = lm3533_reg_init( lm3533 );
        if( ret )
        {
                kfree( lm3533 );
                return ret;
        }

        return 0;
} /* End.. lm3533_i2c_probe() */

/*************************************************************************
**
**************************************************************************/
static int lm3533_i2c_remove(struct i2c_client *i2c)
{
struct lm3533 *lm3533 = i2c_get_clientdata( i2c );

/* [5860] Show a white LED short pulse when user turns phone on */
  lm3533_write_reg( 0x28, 0x00 );
  lm3533_write_reg( 0x27, 0x00 );

  lm3533_i2c_client = NULL;
  i2c_unregister_device( i2c );
  kfree( lm3533 );

  lm3533_device_enable( FALSE );
  return 0;
} /* End.. lm3533_i2c_remove() */

/*************************************************************************
**
**************************************************************************/
static ssize_t store_lm3533(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
lm3533_led_data * ledData = (lm3533_led_data *)buf;

  LEDS_DEBUG("[LED] size=%d, pattern_id=%d  , red=%x,  green=%x , blue=%x\n",
        (int)size,ledData->pattern_id, ledData->red, ledData->green,  ledData->blue);
    // <<<<<----- 2015/04/28-youchihwang
    // DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
    // ALPS02045680
    pattern_id = ledData->pattern_id;        
    //pattern_set_lm3533(NULL, ledData->pattern_id, ledData->flash_mode ,ledData->red, ledData->green, ledData->blue);
    // ----->>>>> 2015/04/28-youchihwang
    // DMS06418317--[Ar]RT-GST-<5919_Notification>-<The phone Notification LED light is blue when receiving a missed call/Email/Message.>
    // ALPS02045680
//pattern_set_lm3533(NULL, ledData->pattern_id, ledData->red, ledData->green, ledData->blue);

  return size;
} /* End.. store_lm3533() */
static DEVICE_ATTR( lm3533, 0664, NULL, store_lm3533 );

/*************************************************************************
** [xssm]Power saving in LED
**************************************************************************/
static ssize_t show_psnotification(struct device *dev,struct device_attribute *attr, char *buf)
{
  LEDS_DEBUG("show_psnotification=%d\n", notification_value_enable );
  return sprintf(buf, "%u", notification_value_enable);
}

/*************************************************************************
**
**************************************************************************/
static ssize_t show_psattention(struct device *dev,struct device_attribute *attr, char *buf)
{
  LEDS_DEBUG("show_psnotification=%d \n", attention_value_enable );
  return sprintf(buf, "%u", attention_value_enable);
}

/*************************************************************************
**
**************************************************************************/
static ssize_t store_psnotification(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
  char *pvalue = NULL;
  notification_value_enable = simple_strtoul( buf, &pvalue, 10 );
  LEDS_DEBUG("store_psnotification=%s", buf );
  return size;
}

/*************************************************************************
**
**************************************************************************/
static ssize_t store_psattention(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
char *pvalue = NULL;

  attention_value_enable = simple_strtoul( buf, &pvalue, 10 );
  LEDS_DEBUG("store_psattention=%s", buf );
  return size;
}
static DEVICE_ATTR( psnotification, 0664, show_psnotification, store_psnotification );
static DEVICE_ATTR( psattention, 0664, show_psattention, store_psattention );

/*************************************************************************
**
**************************************************************************/
static ssize_t store_lm3533_reg_info(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
char *pvalue = NULL;
unsigned int reg_value = 0;
unsigned int reg_address = 0;

  LEDS_DEBUG("store_lm3533_reg_info:size:%d,address:0x%s\n", (int)size,buf );

  if( buf != NULL && size != 0 )
  {
    reg_address = simple_strtoul( buf,&pvalue, 16 );

    LEDS_DEBUG("store_lm3533_reg_info:register:0x%x\n", reg_address );

    if(( reg_address >= 0x10 ) && ( reg_address <= 0xB2 ))
    {
      if( *pvalue && ( *pvalue == '#' ))
      {
        reg_value = simple_strtoul(( pvalue + 1), NULL, 16 );
        lm3533_write_reg( reg_address, reg_value );
        LEDS_DEBUG("set_lm3533_reg_info register:[0x%x]=0x%x\n", reg_address, reg_value );
      }
      else if( *pvalue && ( *pvalue == '@' ))
      {
        lm3533_read_reg( reg_address, &reg_value );
        LEDS_DEBUG("get_lm3533_reg_info register:[0x%x]=0x%x\n",reg_address,reg_value);
      }
    }
  }

  return size;
}

/*************************************************************************
**
**************************************************************************/
static ssize_t show_lm3533_reg_info(struct device *dev, struct device_attribute *attr, char *buf)
{
u8 reg_value    = 0;
u8 reg_address  = 0x27;

  LEDS_DEBUG("show_lm3533_reg_info\n");

  lm3533_read_reg( reg_address, &reg_value );
  LEDS_DEBUG("get_lm3533_reg_info register:[0x%02X]=0x%02X\n", reg_address, reg_value );

  return sprintf( buf, "reg(0x%x)=0x%x\n", reg_address, reg_value );
}
static DEVICE_ATTR( lm3533_reg_info, 0664, show_lm3533_reg_info, store_lm3533_reg_info );


/*************************************************************************
**
**************************************************************************/
int lm3533_create_file(int num)
{
int vRet = 0;

#if defined( LED_TRACE_INFO )
  printk("[LED] %s ...\n", __func__ );
#endif

  vRet = device_create_file( g_leds_data[num]->cdev.dev, &dev_attr_lm3533 );
  if( vRet )
  {
    LEDS_DEBUG("[LED]device_create_file lm3533 fail!\n");
  }	

  /* Power saving in LED */
  vRet = device_create_file( g_leds_data[num]->cdev.dev, &dev_attr_psnotification );
  if( vRet )
  {
    LEDS_DEBUG("[LED]device_create_file psnotification fail!\n");
  }	

  vRet = device_create_file( g_leds_data[num]->cdev.dev, &dev_attr_psattention );
  if( vRet )
  {
    LEDS_DEBUG("[LED]device_create_file psnotification fail!\n");
  }	

  /* Debugging code by adb command */
  vRet = device_create_file( g_leds_data[num]->cdev.dev, &dev_attr_lm3533_reg_info );
  if( vRet )
  {
    LEDS_DEBUG("[LED]device_create_file lm3533_reg_info fail!\n");
  }	

  return vRet;
}

#if defined( LED_LM3533_EX_INIT )
/*************************************************************************
**
**************************************************************************/
int lm3533_i2c_init(int debug_log)
{
  printk("[LED] %s ......\n", __func__ );

  lm3533_debug_enable = debug_log;

  i2c_register_board_info( I2C_LM3533_BUS, &i2c_lm3533, 1 );
  if( i2c_add_driver( &lm3533_i2c_driver ))
  {
    printk("[LED] LM3533 add I2C driver error.\n" );
    return -1;
  }
  else
  {
    printk("[LED] LM3533 add I2C driver success.\n");
  }
  return 0;
}

/*************************************************************************
**
**************************************************************************/
void lm3533_i2c_exit(void)
{
  printk("[LED] %s ......\n", __func__ );
  i2c_del_driver( &lm3533_i2c_driver );
}

#else
/*************************************************************************
**
**************************************************************************/
static int __init lm3533_i2c_init(void)
{
  LEDS_DEBUG("[LED]%s\n", __func__ );

  i2c_register_board_info( I2C_LM3533_BUS, &i2c_lm3533, 1 );
  if( i2c_add_driver( &lm3533_i2c_driver ))
  {
    printk("[LED] lm3533 add I2C driver error\n");
    return -1;
  }
  else
  {
    printk("[LED] lm3533 add I2C driver success\n");
  }
  return 0;
}

/*************************************************************************
**
**************************************************************************/
static void __exit lm3533_i2c_exit(void)
{
  LEDS_DEBUG("[LED]%s\n", __func__ );
  i2c_del_driver( &lm3533_i2c_driver );
}

module_param( lm3533_debug_enable, int, 0644 );
module_init( lm3533_i2c_init );
module_exit( lm3533_i2c_exit );

MODULE_AUTHOR("ARIMA Inc.");
MODULE_DESCRIPTION("LED driver for TI LM3533 chip");
MODULE_LICENSE("GPL");
MODULE_ALIAS("leds-lm3533");

#endif /* End.. (LED_LM3533_EX_INIT) */

/*************************************************************************
**
**************************************************************************/
#undef LED_LM3533_C
#endif /* End.. !(LED_LM3533_C) */
