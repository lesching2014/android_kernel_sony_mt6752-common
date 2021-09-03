#ifndef NT35532_FHD_DSI_VDO_INNOLUX_C
#define NT35532_FHD_DSI_VDO_INNOLUX_C

#if !defined( BUILD_LK )
    #include <linux/string.h>
    #include <linux/kernel.h>
#endif
    #include "lcm_drv.h"

#if defined( BUILD_LK )
    #include  <platform/upmu_common.h>
    #include  <platform/mt_gpio.h>
    #include  <platform/mt_pmic.h>
    #include  <string.h>
#elif defined( BUILD_UBOOT )
    #include  <asm/arch/mt_gpio.h>
#else
    #include  <mach/mt_pm_ldo.h>
    #include  <mach/mt_gpio.h>
#endif
    #include <cust_gpio_usage.h>
    #include "lcm_gate_driver.h"

#if !defined( TRUE )
    #define   TRUE      1
#endif

#if !defined( FALSE )
    #define   FALSE     0
#endif

#if !defined( GPIO_LCM_ID )
    #define   GPIO_LCM_ID               (GPIO97|0x80000000)
#endif

#if !defined( GPIO_LCM_ID_M_GPIO )
    #define   GPIO_LCM_ID_M_GPIO        GPIO_MODE_00
#endif

    #define   LCM_TRACE_ENABLE_N
    #define   LCM_DEBUG_ENABLE_N

#if defined( LCMDRV_TRACE_ENABLE )
  #if defined( BUILD_LK )
    #define   LCM_MSG(srt,arg...)       dprintf(CRITICAL,srt,##arg)
  #else
    #define   LCM_MSG(srt,arg...)       printk(KERN_INFO srt,##arg)
  #endif
#else
    #define   LCM_MSG(srt,arg...)       {/* Do Nothing */}
#endif

#if defined( LCMDRV_DEBUG_ENABLE )
  #if defined( BUILD_LK )
    #define   LCM_DBG(srt,arg...)       dprintf(CRITICAL,srt,##arg)
  #else
    #define   LCM_DBG(srt,arg...)       printk(KERN_DEBUG srt,##arg)
  #endif
#else
    #define   LCM_DBG(srt,arg...)       {/* Do Nothing */}
#endif

    #define   GP0550IA00260_AM31

/*****************************************************************************
** Local Functions
******************************************************************************/
    static const unsigned int BL_MIN_LEVEL = 20;
    static LCM_UTIL_FUNCS   lcm_util = { 0 };

    #define   SET_RESET_PIN(v)          (lcm_util.set_reset_pin((v)))
    #define   MDELAY(n)                 (lcm_util.mdelay(n))
    #define   UDELAY(n)                 (lcm_util.udelay(n))

    #define   dsi_set_cmdq(pdata, queue_size, force_update)     \
                                        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
    #define   dsi_set_cmdq_V2(cmd, count, ppara, force_update)  \
                                        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
    #define   wrtie_cmd(cmd)            lcm_util.dsi_write_cmd(cmd)
    #define   write_regs(addr, pdata, byte_nums)    \
                                        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
    #define   read_reg(cmd)             lcm_util.dsi_dcs_read_lcm_reg(cmd)
    #define   read_reg_v2(cmd, buffer, buffer_size) \
                                        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

/*****************************************************************************
** Local Constants
******************************************************************************/
    #define   LCM_PANEL_SOURCE          0   /* 1: Main panel, 0: Second source panel */
    #define   LCM_ID_NT35532            (0x32)

    #define   LCM_DSI_CMD_MODE          0

#if defined( FPGA_EARLY_PORTING )
    #define   FRAME_WIDTH               (480)
    #define   FRAME_HEIGHT              (800)
#else
    #define   FRAME_WIDTH               (1080)
    #define   FRAME_HEIGHT              (1920)
#endif

    #define   LCM_HSYNC_NUM             10    /** Shall be larger than 3 **/
    #define   LCM_HBP_NUM               20
    #define   LCM_HFP_NUM               40

    #define   LCM_VSYNC_NUM             4     /** Shall be larger than 3 ? **/
    #define   LCM_VBP_NUM               10
    #define   LCM_VFP_NUM               10

    #define   REGFLAG_UDELAY            (0xFB)  /* RELOAD CMD1 */
    #define   REGFLAG_DELAY             (0xFC)
    #define   REGFLAG_END_OF_TABLE      (0xFD)  /* END OF REGISTERS MARKER */

  //#define   REGFLAG_RESET_LOW         (0xFE)  /* RD_CMDSTATUS: Read the Current Register Set */
  //#define   REGFLAG_RESET_HIGH        (0xFF)  /* CMD Page Select */

#if defined( BUILD_LK )
  #if 0
    extern void DSI_clk_HS_mode(unsigned char enter);
  #endif
#endif

/*****************************************************************************
** Local Varibble
******************************************************************************/
    struct LCM_setting_table
    {
        unsigned char   cmd;
        unsigned char   count;
        unsigned char   para_list[64];
    };

  //static unsigned int         lcm_esd_test_innolux = FALSE; /* Only for ESD test */
  //static unsigned char        lcd_id_pins_value_innolux = 0xFF;
    static const unsigned char  LCD_MODULE_ID_innolux = 0x00;
    LCM_DSI_MODE_SWITCH_CMD     lcm_switch_mode_cmd_nt35532_innolux;

  /*==========================================================
  **
  **==========================================================*/
  static struct LCM_setting_table lcm_suspend_setting[] =
  {
  /* Display OFF */
    { 0x28, 0, {}},
    { REGFLAG_DELAY, 25, {}},
  /* Sleep In */
    { 0x10, 0, {}},
    { REGFLAG_DELAY, 125, {}},
  /* End */
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };

  /* update initial param for IC nt35520 0.01 */
  static struct LCM_setting_table lcm_initialization_setting[] =
  {
  /* Return To CMD1 */
  //{ 0xFF, 1, { 0x00 }},
  //{ REGFLAG_UDELAY, 5, {}},
  /* Sleep Out */
    { 0x11, 0, {}},
    { REGFLAG_DELAY, 125, {}},
  /* Display ON */
    { 0x29, 0, {}},
    { REGFLAG_DELAY, 25, {}},
  /* End */
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };

#if 0
  static struct LCM_setting_table lcm_set_window[] =
  {
    { 0x2A, 4, { 0x00, 0x00, (FRAME_WIDTH  >> 8), (FRAME_WIDTH  & 0xFF)}},
    { 0x2B, 4, { 0x00, 0x00, (FRAME_HEIGHT >> 8), (FRAME_HEIGHT & 0xFF)}},
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };
#endif
#if 0
  static struct LCM_setting_table lcm_sleep_out_setting[] =
  {
  /* Sleep Out */
    { 0x11, 1, { 0x00 }},
    { REGFLAG_DELAY, 125, {}},
  /* Display ON */
    { 0x29, 1, { 0x00 }},
    { REGFLAG_DELAY, 25, {}},
  /* End */
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };

  static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] =
  {
  /* Display OFF */
    { 0x28, 1, { 0x00 }},
    { REGFLAG_DELAY, 25, {}},
  /* Sleep In */
    { 0x10, 1, { 0x00 }},
    { REGFLAG_DELAY, 125, {}},
  /* ENTER_DSTB_MODE */
    { 0x4F, 1, { 0x01 }}, /* Enter the Deep Standby Mode */
    { REGFLAG_DELAY, 125, {}}
  /* End */
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };
#endif

  static struct LCM_setting_table lcm_backlight_level_setting[] =
  {
    { 0x51, 1, { 0xFF }},
    { REGFLAG_END_OF_TABLE, 0x00, {}}
  };

/**********************************************************
**
***********************************************************/
static void push_table_innolux(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
unsigned int i;

    for( i = 0; i < count; i++ )
    {
    unsigned cmd;

      cmd = table[i].cmd;
      switch( cmd )
      {
        case REGFLAG_DELAY :
        {
          if( table[i].count <= 10 )
            MDELAY( table[i].count );
          else
            MDELAY( table[i].count );
        } break;

        case REGFLAG_UDELAY :
        {
          UDELAY( table[i].count );
        } break;

        case REGFLAG_END_OF_TABLE :
        {
        } break;

        default:
        {
          dsi_set_cmdq_V2( cmd, table[i].count, table[i].para_list, force_update );
        } break;
      }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------
static void lcm_set_util_funcs_innolux(const LCM_UTIL_FUNCS *util)
{
    memcpy( &lcm_util, util, sizeof( LCM_UTIL_FUNCS ));
}

/**********************************************************
**
***********************************************************/
static void lcm_get_params_innolux(LCM_PARAMS *params)
{
    memset( params, 0x00, sizeof( LCM_PARAMS ));

    params->type   = LCM_TYPE_DSI;

    params->width  = FRAME_WIDTH;
    params->height = FRAME_HEIGHT;

#if( LCM_DSI_CMD_MODE )
    params->dsi.mode   = CMD_MODE;
    params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
    params->dsi.mode   = SYNC_PULSE_VDO_MODE;
    params->dsi.switch_mode = CMD_MODE;
#endif
    params->dsi.switch_mode_enable = 0;

  /** DSI **/
  /* Command mode setting */
    params->dsi.LANE_NUM    = LCM_FOUR_LANE;
  /* The following defined the fomat for data coming from LCD engine. */
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

  /* Highly depends on LCD driver capability. */
    params->dsi.packet_size = 256;

  /* Video mode timing */
    params->dsi.PS  = LCM_PACKED_PS_24BIT_RGB888;

    params->dsi.vertical_sync_active    = LCM_VSYNC_NUM;
    params->dsi.vertical_backporch      = LCM_VBP_NUM;
    params->dsi.vertical_frontporch     = LCM_VFP_NUM;
    params->dsi.vertical_active_line    = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active  = LCM_HSYNC_NUM;
    params->dsi.horizontal_backporch    = LCM_HBP_NUM;
    params->dsi.horizontal_frontporch   = LCM_HFP_NUM;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;
  //params->dsi.ssc_disable             = 1;
#if !defined( FPGA_EARLY_PORTING )
  #if( LCM_DSI_CMD_MODE )
    params->dsi.PLL_CLOCK = 500; //this value must be in MTK suggested table
  #else
    params->dsi.PLL_CLOCK = 450; //this value must be in MTK suggested table
  //params->dsi.PLL_CLOCK = 175; //For test.
  #endif
#else
    params->dsi.pll_div1  = 0x00;
    params->dsi.pll_div2  = 0x00;
    params->dsi.fbk_div   = 0x01;
#endif

    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable      = 0;
    params->dsi.lcm_esd_check_table[0].cmd          = 0x53;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x24;

}

/**********************************************************
** Power manager
***********************************************************/
#define   LCM_ENABLE          1
#define   LCM_DISABLE         0
#define   LCM_POWER_VOL       VOL_DEFAULT //VOL_3000
static void lcm_power_enable_innolux(int enable)
{
#if !defined( FPGA_EARLY_PORTING )
    if( LCM_ENABLE == enable )
    {
    #if 0
      #if defined( BUILD_LK )
        mt6325_upmu_set_rg_vgp1_en( 1 );
      #else
        hwPowerOn( MT6325_POWER_LDO_VGP1, LCM_POWER_VOL, "LCM_DRV" );
      #endif
    #endif
    }
    else
    {
    #if 0
      #if defined( BUILD_LK )
        mt6325_upmu_set_rg_vgp1_en( 0 );
      #else
        hwPowerDown( MT6325_POWER_LDO_VGP1, "LCM_DRV" );
      #endif
    #endif
    }
#endif
}

/**********************************************************
**
***********************************************************/
static void lcm_gate_power_innolux(int enable)
{
#if !defined( FPGA_EARLY_PORTING )
#if defined( BUILD_LK )
kal_uint8     cmd = 0x00, data = 0xFF;
#else
unsigned char cmd = 0x00, data = 0xFF;
#endif
int   vRet = 0;

    if( LCM_ENABLE == enable )
    {
        lcm_gate_enable( TRUE );
        UDELAY( 10 );

        cmd   = 0x00; /* AVDD */
        data  = 0x0F; //0x0E; /* 0x0E: +5.4V, 0x0F: +5.5V */
        vRet = lcm_gate_write_bytes( cmd, data );
      #if defined( BUILD_LK )
        if( vRet )
          dprintf( 0, "[LK]%s: lcm_gate set cmd = 0x%02X error\n", __func__, cmd );
        else
          dprintf( 0, "[LK]%s: lcm_gate set cmd = 0x%02X success\n", __func__, cmd );
      #else
        if( vRet < 0 )
          printk("[KERNEL]%s: lcm_gate set cmd = 0x%02X error\n", __func__, cmd );
        else
          printk("[KERNEL]%s: lcm_gate set cmd = 0x%02X success\n", __func__, cmd );
      #endif

        cmd   = 0x01; /* AVEE */
        data  = 0x0F; //0x0E; /* 0x0E: -5.4V, 0x0F: -5.5V */
        vRet = lcm_gate_write_bytes( cmd, data );
      #if defined( BUILD_LK )
        if( vRet )
          dprintf( 0, "[LK]%s: lcm_gate set cmd = 0x%02X error\n", __func__, cmd );
        else
          dprintf( 0, "[LK]%s: lcm_gate set cmd = 0x%02X success\n", __func__, cmd );
      #else
        if( vRet < 0 )
          printk("[KERNEL]%s: lcm_gate set cmd = 0x%02X error\n", __func__, cmd );
        else
          printk("[KERNEL]%s: lcm_gate set cmd = 0x%02X success\n", __func__, cmd );
      #endif
    }
    else
    {
        lcm_gate_enable( FALSE );
        UDELAY( 10 );
    }
#endif /* End.. !(FPGA_EARLY_PORTING) */
}

/**********************************************************
**
***********************************************************/
static void lcm_init_power_innolux(void)
{
    lcm_power_enable_innolux( LCM_ENABLE );
}

static void lcm_suspend_power_innolux(void)
{
    lcm_power_enable_innolux( LCM_DISABLE );
}

/**********************************************************
**
***********************************************************/
static void lcm_resume_power_innolux(void)
{
    lcm_power_enable_innolux( LCM_ENABLE );
}

/**********************************************************
**
***********************************************************/
static void lcm_init_innolux(void)
{
#if !defined( FPGA_EARLY_PORTING )
    lcm_gate_power_innolux( LCM_ENABLE );
    MDELAY( 12 );
#endif /* End.. !(FPGA_EARLY_PORTING) */

    SET_RESET_PIN( 1 );
    MDELAY( 3 );
    SET_RESET_PIN( 0 );
    MDELAY( 10 );
    SET_RESET_PIN( 1 );
    MDELAY( 15 );

  /* when phone initial, config output high, enable backlight drv chip */
    push_table_innolux( lcm_initialization_setting, sizeof( lcm_initialization_setting ) / sizeof( struct LCM_setting_table ), 1 );
}

/**********************************************************
**
***********************************************************/
static void lcm_suspend_innolux(void)
{
    lcm_gate_enable( FALSE );
    push_table_innolux( lcm_suspend_setting, sizeof( lcm_suspend_setting ) / sizeof( struct LCM_setting_table ), 1 );
  //SET_RESET_PIN( 0 );
    MDELAY( 10 );
}

/**********************************************************
**
***********************************************************/
static void lcm_resume_innolux(void)
{
    lcm_init_innolux();
}

#if( LCM_DSI_CMD_MODE )
/**********************************************************
**
***********************************************************/
static void lcm_update_innolux(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
{
unsigned int x0 = x;
unsigned int y0 = y;
unsigned int x1 = x0 + width - 1;
unsigned int y1 = y0 + height - 1;

unsigned char x0_MSB = (( x0 >> 8) & 0xFF );
unsigned char x0_LSB = ( x0 & 0xFF );
unsigned char x1_MSB = (( x1 >> 8) & 0xFF );
unsigned char x1_LSB = ( x1 & 0xFF );
unsigned char y0_MSB = (( y0 >> 8 ) & 0xFF );
unsigned char y0_LSB = ( y0 & 0xFF );
unsigned char y1_MSB = (( y1 >> 8 ) & 0xFF );
unsigned char y1_LSB = ( y1 & 0xFF );

unsigned int data_array[16];

    data_array[0] = 0x00053902;
    data_array[1] = ( x1_MSB << 24 ) | ( x0_LSB << 16 ) | ( x0_MSB << 8 ) | 0x2A;
    data_array[2] = ( x1_LSB );
    dsi_set_cmdq( data_array, 3, 1 );

    data_array[0] = 0x00053902;
    data_array[1] = ( y1_MSB << 24 ) | ( y0_LSB << 16 ) | ( y0_MSB << 8 ) | 0x2B;
    data_array[2] = ( y1_LSB );
    dsi_set_cmdq( data_array, 3, 1 );

    data_array[0] = 0x002C3909;
    dsi_set_cmdq( data_array, 1, 0 );
}
#endif

/**********************************************************
**
***********************************************************/
static unsigned int lcm_compare_id_innolux(void)
{
unsigned int  id = 0;
unsigned int  array[16];
unsigned int  lcm_id_gpio, vRet = 0;
unsigned char buffer[2];

#if defined( BUILD_LK )
  #if 0
    DSI_clk_HS_mode( 1 );
    MDELAY( 10 );
    DSI_clk_HS_mode( 0 );
  #endif
    mt_set_gpio_mode( GPIO_LCM_ID, GPIO_LCM_ID_M_GPIO );
    mt_set_gpio_dir( GPIO_LCM_ID, GPIO_DIR_IN );
  //SET_GPIO_MODE( GPIO_LCM_ID, GPIO_LCM_ID_M_GPIO );
  //SET_GPIO_DIR_IN( GPIO_LCM_ID );
    MDELAY( 1 );
#endif

    SET_RESET_PIN( 1 );
    MDELAY( 3 );
    SET_RESET_PIN( 0 );
    MDELAY( 10 );
    SET_RESET_PIN( 1 );
    MDELAY( 15 );

    lcm_id_gpio = mt_get_gpio_in( GPIO_LCM_ID );

    array[0] = 0x00023700;  /* read id return two byte,version and id */
    dsi_set_cmdq( array, 1, 1 );

    read_reg_v2( 0xF4, buffer, 2 );
    id = buffer[0]; /* We only need ID */
#if defined( BUILD_LK )
    dprintf( 0, "[LK]%s: LCM chip ID = 0x%04X, ID pin = %d\n", __func__, id, lcm_id_gpio );
#else
    printk("[Kernel]%s: LCM chip ID = 0x%04X, ID pin = %d\n", __func__, id, lcm_id_gpio );
#endif

    vRet = 0;

//<2014/10/17-stevenchen, Add Truly R61335 LCM driver
    if( LCM_PANEL_SOURCE == lcm_id_gpio ) /* Innolux NT35532 */
	vRet=1;
//>2014/10/17-stevenchen

    return vRet;
}

/**********************************************************
** return TRUE : Need recovery
** return FALSE: No need recovery
***********************************************************/
static unsigned int lcm_esd_check_innolux(void)
{
#if !defined( BUILD_LK )
int   array[4];
char  buffer[4];

    array[0] = 0x00013700;
    dsi_set_cmdq( array, 1, 1 );

    read_reg_v2( 0x53, buffer, 1 );

    if( buffer[0] != 0x24 )
    {
      printk("[LCM ERROR][0x53] = 0x%02x\n", buffer[0] );
      return TRUE;
    }
    else
    {
      printk("[LCM NORMAL][0x53] = 0x%02x\n", buffer[0] );
      return FALSE;
    }
#else
    return FALSE;
#endif

}

/**********************************************************
**
***********************************************************/
unsigned int lcm_ata_check_nt35532_innolux(unsigned char *buffer)
{
#if !defined( BUILD_LK )
unsigned int ret = 0;
unsigned int x0 = FRAME_WIDTH / 4;
unsigned int x1 = FRAME_WIDTH * 3 / 4;

unsigned char x0_MSB = (( x0 >> 8 ) & 0xFF );
unsigned char x0_LSB = ( x0 & 0xFF );
unsigned char x1_MSB = (( x1 >> 8 ) & 0xFF );
unsigned char x1_LSB = ( x1 & 0xFF );

unsigned int data_array[3];
unsigned char read_buf[4];

    printk("ATA check size = 0x%02X, 0x%02X, 0x%02X, 0x%02X\n", x0_MSB, x0_LSB, x1_MSB, x1_LSB );

    data_array[0] = 0x0005390A;  /* HS packet */
    data_array[1] = ( x1_MSB << 24 ) | ( x0_LSB << 16 ) | ( x0_MSB << 8 ) | 0x2A;
    data_array[2] = ( x1_LSB );
    dsi_set_cmdq( data_array, 3, 1 );

    data_array[0] = 0x00043700; /* read id return two byte,version and id */
    dsi_set_cmdq( data_array, 1, 1 );

    read_reg_v2( 0x2A, read_buf, 4 );

    if(( read_buf[0] == x0_MSB ) && ( read_buf[1] == x0_LSB )
    && ( read_buf[2] == x1_MSB ) && ( read_buf[3] == x1_LSB ))
      ret = 1;
    else
      ret = 0;

    x0 = 0;
    x1 = FRAME_WIDTH - 1;

    x0_MSB = (( x0 >> 8 ) & 0xFF );
    x0_LSB = ( x0 & 0xFF );
    x1_MSB = (( x1 >> 8 ) & 0xFF );
    x1_LSB = ( x1 & 0xFF );

    data_array[0] = 0x0005390A; /* HS packet  */
    data_array[1] = ( x1_MSB << 24 ) | ( x0_LSB << 16 ) | ( x0_MSB << 8 ) | 0x2A;
    data_array[2] = ( x1_LSB );
    dsi_set_cmdq( data_array, 3, 1 );

    return ret;
#else
    return 0;
#endif
}

/**********************************************************
**
***********************************************************/
static void lcm_setbacklight_innolux(unsigned int level)
{
#if defined( BUILD_LK )
    dprintf( 0, "%s, LK nt35532 backlight: level = %d\n", __func__, level );
#else
    printk("%s, Kernel nt35532 backlight: level = %d\n", __func__, level );
#endif
  /* Refresh value of backlight level. */
    lcm_backlight_level_setting[0].para_list[0] = level;

    push_table_innolux( lcm_backlight_level_setting, sizeof( lcm_backlight_level_setting ) / sizeof( struct LCM_setting_table ), 1 );
}

/**********************************************************
**
***********************************************************/
void* lcm_switch_mode_nt35532_innolux(int mode)
{
#if !defined( BUILD_LK )
/* customization: 1. V2C config 2 values, C2V config 1 value; 2. config mode control register */
    if( mode == 0 )
    { /* V2C */
      lcm_switch_mode_cmd_nt35532_innolux.mode    = CMD_MODE;
      lcm_switch_mode_cmd_nt35532_innolux.addr    = 0xBB; /* mode control addr */
      lcm_switch_mode_cmd_nt35532_innolux.val[0]  = 0x13; /* enabel GRAM firstly, ensure writing one frame to GRAM */
      lcm_switch_mode_cmd_nt35532_innolux.val[1]  = 0x10; /* disable video mode secondly */
    }
    else
    { /* V2C */
      lcm_switch_mode_cmd_nt35532_innolux.mode    = SYNC_PULSE_VDO_MODE;
      lcm_switch_mode_cmd_nt35532_innolux.addr    = 0xBB;
      lcm_switch_mode_cmd_nt35532_innolux.val[0]  = 0x03; /* disable GRAM and enable video mode */
    }

    return (void*)( &lcm_switch_mode_cmd_nt35532_innolux );
#else
    return NULL;
#endif
}

/**********************************************************
**
***********************************************************/
LCM_DRIVER nt35532_fhd_dsi_vdo_innolux_lcm_drv=
{
    .name             = "nt35532_fhd_dsi_vdo_innolux_drv",
    .set_util_funcs   = lcm_set_util_funcs_innolux,
    .get_params       = lcm_get_params_innolux,
    .init             = lcm_init_innolux, /*tianma init fun.*/
    .suspend          = lcm_suspend_innolux,
    .resume           = lcm_resume_innolux,
    .compare_id       = lcm_compare_id_innolux,
    .init_power       = lcm_init_power_innolux,
    .resume_power     = lcm_resume_power_innolux,
    .suspend_power    = lcm_suspend_power_innolux,
    .esd_check        = lcm_esd_check_innolux,
    .set_backlight    = lcm_setbacklight_innolux,
    .ata_check        = lcm_ata_check_nt35532_innolux,
#if( LCM_DSI_CMD_MODE )
    .update           = lcm_update_innolux,
#endif
    .switch_mode      = lcm_switch_mode_nt35532_innolux,
};

#undef NT35532_FHD_DSI_VDO_INNOLUX_C
#endif
