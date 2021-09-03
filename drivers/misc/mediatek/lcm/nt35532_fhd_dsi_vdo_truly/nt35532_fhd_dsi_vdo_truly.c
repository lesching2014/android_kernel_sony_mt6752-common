//<2014/09/23-stevenchen, Update TRULY LCM driver
#ifndef NT35532_FHD_DSI_VDO_TRULY_C
#define NT35532_FHD_DSI_VDO_TRULY_C

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
//  #define   dsi_set_cmdq_V3(para_tbl,size,force_update)   \
//                                      lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
    #define   wrtie_cmd(cmd)            lcm_util.dsi_write_cmd(cmd)
    #define   write_regs(addr, pdata, byte_nums)    \
                                        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
    #define   read_reg(cmd)             lcm_util.dsi_dcs_read_lcm_reg(cmd)
    #define   read_reg_v2(cmd, buffer, buffer_size) \
                                        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

/*****************************************************************************
** Local Constants
******************************************************************************/
    #define   LCM_PANEL_SOURCE          1   /* 1: Main resource panel, 0: Second resource panel */
    #define   LCM_ID_NT35532            (0x32)

    #define   LCM_DSI_CMD_MODE          0

#if defined( FPGA_EARLY_PORTING )
    #define   FRAME_WIDTH               (480)
    #define   FRAME_HEIGHT              (800)
#else
    #define   FRAME_WIDTH               (1080)
    #define   FRAME_HEIGHT              (1920)
#endif

    #define   LCM_HSYNC_NUM             (16)    /** Shall be larger than 3 **/
    #define   LCM_HBP_NUM               (105)   //20
    #define   LCM_HFP_NUM               (105)   //40

    #define   LCM_VSYNC_NUM             (1)     /** Shall be larger than 3 ? **/
    #define   LCM_VBP_NUM               (16)    //10
    #define   LCM_VFP_NUM               (16)    //10

    #define   REGFLAG_UDELAY            (0xFE)  //(0xFB)  /* RELOAD CMD1 */
    #define   REGFLAG_DELAY             (0xFC)
    #define   REGFLAG_END_OF_TABLE      (0xFD)  /* END OF REGISTERS MARKER */

  //#define   REGFLAG_RESET_LOW         (0xFE)  /* RD_CMDSTATUS: Read the Current Register Set */
  //#define   REGFLAG_RESET_HIGH        (0xFF)  /* CMD Page Select */

    #define   LCM_ESD_CHECK_REG         (0x53)
    #define   LCM_ESD_CHECK_VAL         (0x00)  //(0x24)

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

  //static unsigned int         lcm_esd_test_truly = FALSE; /* Only for ESD test */
  //static unsigned char        lcd_id_pins_value_truly = 0xFF;
    static const unsigned char  LCD_MODULE_ID_truly = 0x00;
    LCM_DSI_MODE_SWITCH_CMD     lcm_switch_mode_cmd_truly;

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
  /* CMD Page Select */
    { 0xFF, 1, { 0x05 }}, /*  CMD2 Page4 GIP Timing */
    { REGFLAG_UDELAY, 100, {}},
  /* Forward / Backward */
  //{ 0xA2, 1, { 0xB2 }}, //0xB6
  /* Interval Scan Frame Setting(Inversion) */
    { 0x9F, 1, { 0x0F }}, /* 0F:Column, 03:4dot, 01:2dot, 00:1dot */
  /* SET VSOUT = VDD */
    { 0xD7, 1, { 0x31 }},
  /* SET HSOUT=NT50198 PSYNC */
    { 0xD8, 1, { 0x7E }},
  /* DEFINE CGOUT MAPPING */
    { 0x00, 1, { 0x40 }},
    { 0x01, 1, { 0x40 }},
    { 0x02, 1, { 0x08 }},
    { 0x03, 1, { 0x06 }},
    { 0x04, 1, { 0x40 }},
    { 0x05, 1, { 0x16 }},
    { 0x06, 1, { 0x1E }},
    { 0x07, 1, { 0x18 }},
    { 0x08, 1, { 0x20 }},
    { 0x09, 1, { 0x1A }},
    { 0x0A, 1, { 0x22 }},
    { 0x0B, 1, { 0x1C }},
    { 0x0C, 1, { 0x24 }},
    { 0x0D, 1, { 0x40 }},
    { 0x0E, 1, { 0x04 }},
    { 0x0F, 1, { 0x05 }},
    { 0x10, 1, { 0x0E }},
    { 0x11, 1, { 0x10 }},
    { 0x12, 1, { 0x40 }},
    { 0x13, 1, { 0x40 }},
  /* */
    { 0x14, 1, { 0x40 }},
    { 0x15, 1, { 0x40 }},
    { 0x16, 1, { 0x09 }},
    { 0x17, 1, { 0x07 }},
    { 0x18, 1, { 0x40 }},
    { 0x19, 1, { 0x17 }},
    { 0x1A, 1, { 0x1F }},
    { 0x1B, 1, { 0x19 }},
    { 0x1C, 1, { 0x21 }},
    { 0x1D, 1, { 0x1B }},
    { 0x1E, 1, { 0x23 }},
    { 0x1F, 1, { 0x1D }},
    { 0x20, 1, { 0x25 }},
    { 0x21, 1, { 0x40 }},
    { 0x22, 1, { 0x04 }},
    { 0x23, 1, { 0x05 }},
    { 0x24, 1, { 0x0F }},
    { 0x25, 1, { 0x11 }},
    { 0x26, 1, { 0x40 }},
    { 0x27, 1, { 0x40 }},
  /* */
    { 0x28, 1, { 0x40 }},
    { 0x29, 1, { 0x40 }},
    { 0x2A, 1, { 0x11 }},
    { 0x2B, 1, { 0x0F }},
    { 0x2D, 1, { 0x40 }},
    { 0x2F, 1, { 0x25 }},
    { 0x30, 1, { 0x1D }},
    { 0x31, 1, { 0x23 }},
    { 0x32, 1, { 0x1B }},
    { 0x33, 1, { 0x21 }},
    { 0x34, 1, { 0x19 }},
    { 0x35, 1, { 0x1F }},
    { 0x36, 1, { 0x17 }},
    { 0x37, 1, { 0x40 }},
    { 0x38, 1, { 0x04 }},
    { 0x39, 1, { 0x05 }},
    { 0x3A, 1, { 0x07 }},
    { 0x3B, 1, { 0x09 }},
    { 0x3D, 1, { 0x40 }},
    { 0x3F, 1, { 0x40 }},
  /* */
    { 0x40, 1, { 0x40 }},
    { 0x41, 1, { 0x40 }},
    { 0x42, 1, { 0x10 }},
    { 0x43, 1, { 0x0E }},
    { 0x44, 1, { 0x40 }},
    { 0x45, 1, { 0x24 }},
    { 0x46, 1, { 0x1C }},
    { 0x47, 1, { 0x22 }},
    { 0x48, 1, { 0x1A }},
    { 0x49, 1, { 0x20 }},
    { 0x4A, 1, { 0x18 }},
    { 0x4B, 1, { 0x1E }},
    { 0x4C, 1, { 0x16 }},
    { 0x4D, 1, { 0x40 }},
    { 0x4E, 1, { 0x04 }},
    { 0x4F, 1, { 0x05 }},
    { 0x50, 1, { 0x06 }},
    { 0x51, 1, { 0x08 }},
    { 0x52, 1, { 0x40 }},
    { 0x53, 1, { 0x40 }},
 /* RESOLUTION & RTN & BF/FP=16 */
    { 0x8D, 1, { 0x00 }},
    { 0x8E, 1, { 0x00 }},
    { 0x8F, 1, { 0xC0 }},
    { 0x90, 1, { 0x7B }}, /* RTN  USER */
    { 0x91, 1, { 0x11 }},
    { 0x92, 1, { 0x0F }},
  /* GOA TIMING(ENGINEER CMD) */
    { 0x54, 1, { 0x0E }},
    { 0x55, 1, { 0x0A }},
    { 0x56, 1, { 0x06 }},
    { 0x58, 1, { 0x02 }},
    { 0x59, 1, { 0x3F }},
    { 0x5A, 1, { 0x3F }},
    { 0x5B, 1, { 0x02 }},
    { 0x5C, 1, { 0x02 }},
    { 0x5D, 1, { 0x01 }},
    { 0x5E, 1, { 0x20 }}, /* delay, 1H:0x20, 2H:0x21 */
    { 0x5F, 1, { 0x1B }}, /* delay, 1H:0x1B, 2H:0x1A */
    { 0x60, 1, { 0x1C }},
    { 0x61, 1, { 0x1C }},
    { 0x62, 1, { 0x31 }}, /* delay, 1H-0x31, 2H:0x29 */
    { 0x63, 1, { 0x02 }},
    { 0x64, 1, { 0x02 }},
    { 0x65, 1, { 0x00 }},
    { 0x66, 1, { 0x79 }}, /* delay, 1H-0x79, 2H:0x6A */
    { 0x67, 1, { 0x11 }},
    { 0x68, 1, { 0x02 }},
    { 0x69, 1, { 0x12 }},
    { 0x6A, 1, { 0x05 }}, /* delay, 1H:0x05, 2H:0x04 */
    { 0x6B, 1, { 0x28 }},
    { 0x6C, 1, { 0x08 }},
    { 0x6D, 1, { 0x00 }},
    { 0x6F, 1, { 0x04 }},
    { 0x70, 1, { 0x02 }},
  /* */
    { 0xB7, 1, { 0x11 }},
    { 0xBC, 1, { 0x01 }},
    { 0xBD, 1, { 0x10 }},
    { 0xCC, 1, { 0xA2 }},
    { 0xCE, 1, { 0x88 }},
    { 0xCF, 1, { 0x88 }},
    { 0xD1, 1, { 0x00 }},
    { 0xD2, 1, { 0x00 }},
    { 0xD3, 1, { 0x00 }},
    { 0xD5, 1, { 0x11 }},
    { 0x99, 1, { 0x00 }},
  /* MTP NON RELOAD */
    { 0xFB, 1, { 0x01 }},
  /*=================================
  ** Gamma
  **=================================*/
  /* CMD Page Select */
    { 0xFF, 1, { 0x01 }}, /* CMD2 PAGE0 */
    { REGFLAG_UDELAY, 500, {}},
  /* ===== R+ =====*/
    { 0x75, 1, { 0x00 }}, /* 255 <-0(SPEC) */
    { 0x76, 1, { 0x03 }},
    { 0x77, 1, { 0x00 }}, /* 254 <-1 */
    { 0x78, 1, { 0x0D }},
    { 0x79, 1, { 0x00 }}, /* 252 <-3 */
    { 0x7A, 1, { 0x24 }},
    { 0x7B, 1, { 0x00 }}, /* 250 <-5 */
    { 0x7C, 1, { 0x36 }},
    { 0x7D, 1, { 0x00 }}, /* 248 <-7 */
    { 0x7E, 1, { 0x45 }},
    { 0x7F, 1, { 0x00 }}, /* 246 <-9 */
    { 0x80, 1, { 0x55 }},
    { 0x81, 1, { 0x00 }}, /* 244 <-11 */
    { 0x82, 1, { 0x65 }},
    { 0x83, 1, { 0x00 }}, /* 242 <-13 */
    { 0x84, 1, { 0x72 }},
    { 0x85, 1, { 0x00 }}, /* 240 <-15 */
    { 0x86, 1, { 0x7F }},
    { 0x87, 1, { 0x00 }}, /* 232 <-23 */
    { 0x88, 1, { 0xAD }},
    { 0x89, 1, { 0x00 }}, /* 224 <-31 */
    { 0x8A, 1, { 0xD3 }},
    { 0x8B, 1, { 0x01 }}, /* 208 <-47 */
    { 0x8C, 1, { 0x15 }},
    { 0x8D, 1, { 0x01 }}, /* 192 <-63 */
    { 0x8E, 1, { 0x4C }},
    { 0x8F, 1, { 0x01 }}, /* 160 <-95 */
    { 0x90, 1, { 0xA4 }},
    { 0x91, 1, { 0x01 }}, /* 128 <-127 */
    { 0x92, 1, { 0xEB }},
    { 0x93, 1, { 0x01 }}, /* 127 <-128 */
    { 0x94, 1, { 0xEE }},
    { 0x95, 1, { 0x02 }}, /* 95  <-160 */
    { 0x96, 1, { 0x2E }},
    { 0x97, 1, { 0x02 }}, /* 63  <-192 */
    { 0x98, 1, { 0x73 }},
    { 0x99, 1, { 0x02 }}, /* 47  <-208 */
    { 0x9A, 1, { 0x9C }},
    { 0x9B, 1, { 0x02 }}, /* 31  <-224 */
    { 0x9C, 1, { 0xD2 }},
    { 0x9D, 1, { 0x02 }}, /* 23  <-232 */
    { 0x9E, 1, { 0xF5 }},
    { 0x9F, 1, { 0x03 }}, /* 15  <-240 */
    { 0xA0, 1, { 0x24 }},
    { 0xA2, 1, { 0x03 }}, /* 13  <-242 */
    { 0xA3, 1, { 0x33 }},
    { 0xA4, 1, { 0x03 }}, /* 11  <-244 */
    { 0xA5, 1, { 0x43 }},
    { 0xA6, 1, { 0x03 }}, /* 9   <-246 */
    { 0xA7, 1, { 0x55 }},
    { 0xA9, 1, { 0x03 }}, /* 7   <-248 */
    { 0xAA, 1, { 0x6A }},
    { 0xAB, 1, { 0x03 }}, /* 5   <-250 */
    { 0xAC, 1, { 0x8B }},
    { 0xAD, 1, { 0x03 }}, /* 3   <-252 */
    { 0xAE, 1, { 0xB2 }},
    { 0xAF, 1, { 0x03 }}, /* 1   <-254 */
    { 0xB0, 1, { 0xF0 }},
    { 0xB1, 1, { 0x03 }}, /* 0   <-255 */
    { 0xB2, 1, { 0xFF }},
  /* ===== R- =====*/
    { 0xB3, 1, { 0x00 }}, /* 255 <-0 */
    { 0xB4, 1, { 0x03 }},
    { 0xB5, 1, { 0x00 }}, /* 254 <-1 */
    { 0xB6, 1, { 0x0D }},
    { 0xB7, 1, { 0x00 }}, /* 252 <-3 */
    { 0xB8, 1, { 0x24 }},
    { 0xB9, 1, { 0x00 }}, /* 250 <-5 */
    { 0xBA, 1, { 0x36 }},
    { 0xBB, 1, { 0x00 }}, /* 248 <-7 */
    { 0xBC, 1, { 0x45 }},
    { 0xBD, 1, { 0x00 }}, /* 246 <-9 */
    { 0xBE, 1, { 0x55 }},
    { 0xBF, 1, { 0x00 }}, /* 244 <-11 */
    { 0xC0, 1, { 0x65 }},
    { 0xC1, 1, { 0x00 }}, /* 242 <-13 */
    { 0xC2, 1, { 0x72 }},
    { 0xC3, 1, { 0x00 }}, /* 240 <-15 */
    { 0xC4, 1, { 0x7F }},
    { 0xC5, 1, { 0x00 }}, /* 232 <-23 */
    { 0xC6, 1, { 0xAD }},
    { 0xC7, 1, { 0x00 }}, /* 224 <-31 */
    { 0xC8, 1, { 0xD3 }},
    { 0xC9, 1, { 0x01 }}, /* 208 <-47 */
    { 0xCA, 1, { 0x15 }},
    { 0xCB, 1, { 0x01 }}, /* 192 <-63 */
    { 0xCC, 1, { 0x4C }},
    { 0xCD, 1, { 0x01 }}, /* 160 <-95 */
    { 0xCE, 1, { 0xA4 }},
    { 0xCF, 1, { 0x01 }}, /* 128 <-127 */
    { 0xD0, 1, { 0xEB }},
    { 0xD1, 1, { 0x01 }}, /* 127 <-128 */
    { 0xD2, 1, { 0xEE }},
    { 0xD3, 1, { 0x02 }}, /* 95  <-160 */
    { 0xD4, 1, { 0x2E }},
    { 0xD5, 1, { 0x02 }}, /* 63  <-192 */
    { 0xD6, 1, { 0x73 }},
    { 0xD7, 1, { 0x02 }}, /* 47  <-208 */
    { 0xD8, 1, { 0x9C }},
    { 0xD9, 1, { 0x02 }}, /* 31  <-224 */
    { 0xDA, 1, { 0xD2 }},
    { 0xDB, 1, { 0x02 }}, /* 23  <-232 */
    { 0xDC, 1, { 0xF5 }},
    { 0xDD, 1, { 0x03 }}, /* 15  <-240 */
    { 0xDE, 1, { 0x24 }},
    { 0xDF, 1, { 0x03 }}, /* 13  <-242 */
    { 0xE0, 1, { 0x33 }},
    { 0xE1, 1, { 0x03 }}, /* 11  <-244 */
    { 0xE2, 1, { 0x43 }},
    { 0xE3, 1, { 0x03 }}, /* 9   <-246 */
    { 0xE4, 1, { 0x55 }},
    { 0xE5, 1, { 0x03 }}, /* 7   <-248 */
    { 0xE6, 1, { 0x6A }},
    { 0xE7, 1, { 0x03 }}, /* 5   <-250 */
    { 0xE8, 1, { 0x8B }},
    { 0xE9, 1, { 0x03 }}, /* 3   <-252 */
    { 0xEA, 1, { 0xB2 }},
    { 0xEB, 1, { 0x03 }}, /* 1   <-254 */
    { 0xEC, 1, { 0xF0 }},
    { 0xED, 1, { 0x03 }}, /* 0   <-255 */
    { 0xEE, 1, { 0xFF }},
  /* ===== G+ =====*/
    { 0xEF, 1, { 0x00 }}, /* 255  <-0 */
    { 0xF0, 1, { 0x03 }},
    { 0xF1, 1, { 0x00 }}, /* 254  <-1 */
    { 0xF2, 1, { 0x0D }},
    { 0xF3, 1, { 0x00 }}, /* 252  <-3 */
    { 0xF4, 1, { 0x24 }},
    { 0xF5, 1, { 0x00 }}, /* 250  <-5 */
    { 0xF6, 1, { 0x36 }},
    { 0xF7, 1, { 0x00 }}, /* 248  <-7 */
    { 0xF8, 1, { 0x45 }},
    { 0xF9, 1, { 0x00 }}, /* 246  <-9 */
    { 0xFA, 1, { 0x55 }},
  /* CMD Page Select */
    { 0xFF, 1, { 0x02 }}, /* CMD2 PAGE1 */
    { REGFLAG_UDELAY, 500, {}},
  /* */
    { 0x00, 1, { 0x00 }}, /* 244  <-11 */
    { 0x01, 1, { 0x65 }},
    { 0x02, 1, { 0x00 }}, /* 242  <-13 */
    { 0x03, 1, { 0x72 }},
    { 0x04, 1, { 0x00 }}, /* 240  <-15 */
    { 0x05, 1, { 0x7F }},
    { 0x06, 1, { 0x00 }}, /* 232  <-23 */
    { 0x07, 1, { 0xAD }},
    { 0x08, 1, { 0x00 }}, /* 224  <-31 */
    { 0x09, 1, { 0xD3 }},
    { 0x0A, 1, { 0x01 }}, /* 208  <-47 */
    { 0x0B, 1, { 0x15 }},
    { 0x0C, 1, { 0x01 }}, /* 192  <-63 */
    { 0x0D, 1, { 0x4C }},
    { 0x0E, 1, { 0x01 }}, /* 160  <-95 */
    { 0x0F, 1, { 0xA4 }},
    { 0x10, 1, { 0x01 }}, /* 128 <-127 */
    { 0x11, 1, { 0xEB }},
    { 0x12, 1, { 0x01 }}, /* 127 <-128 */
    { 0x13, 1, { 0xEE }},
    { 0x14, 1, { 0x02 }}, /* 95  <-160 */
    { 0x15, 1, { 0x2E }},
    { 0x16, 1, { 0x02 }}, /* 63  <-192 */
    { 0x17, 1, { 0x73 }},
    { 0x18, 1, { 0x02 }}, /* 47  <-208 */
    { 0x19, 1, { 0x9C }},
    { 0x1A, 1, { 0x02 }}, /* 31  <-224 */
    { 0x1B, 1, { 0xD2 }},
    { 0x1C, 1, { 0x02 }}, /* 23  <-232 */
    { 0x1D, 1, { 0xF5 }},
    { 0x1E, 1, { 0x03 }}, /* 15  <-240 */
    { 0x1F, 1, { 0x24 }},
    { 0x20, 1, { 0x03 }}, /* 13  <-242 */
    { 0x21, 1, { 0x33 }},
    { 0x22, 1, { 0x03 }}, /* 11  <-244 */
    { 0x23, 1, { 0x43 }},
    { 0x24, 1, { 0x03 }}, /* 9   <-246 */
    { 0x25, 1, { 0x55 }},
    { 0x26, 1, { 0x03 }}, /* 7   <-248 */
    { 0x27, 1, { 0x6A }},
    { 0x28, 1, { 0x03 }}, /* 5   <-250 */
    { 0x29, 1, { 0x8B }},
    { 0x2A, 1, { 0x03 }}, /* 3   <-252 */
    { 0x2B, 1, { 0xB2 }},
    { 0x2D, 1, { 0x03 }}, /* 1   <-254 */
    { 0x2F, 1, { 0xF0 }},
    { 0x30, 1, { 0x03 }}, /* 0   <-255 */
    { 0x31, 1, { 0xFF }},
  /* ===== G- =====*/
    { 0x32, 1, { 0x00 }}, /* 255  <-0 */
    { 0x33, 1, { 0x03 }},
    { 0x34, 1, { 0x00 }}, /* 254  <-1 */
    { 0x35, 1, { 0x0D }},
    { 0x36, 1, { 0x00 }}, /* 252  <-3 */
    { 0x37, 1, { 0x24 }},
    { 0x38, 1, { 0x00 }}, /* 250  <-5 */
    { 0x39, 1, { 0x36 }},
    { 0x3A, 1, { 0x00 }}, /* 248  <-7 */
    { 0x3B, 1, { 0x45 }},
    { 0x3D, 1, { 0x00 }}, /* 246  <-9 */
    { 0x3F, 1, { 0x55 }},
    { 0x40, 1, { 0x00 }}, /* 244  <-11 */
    { 0x41, 1, { 0x65 }},
    { 0x42, 1, { 0x00 }}, /* 242  <-13 */
    { 0x43, 1, { 0x72 }},
    { 0x44, 1, { 0x00 }}, /* 240  <-15 */
    { 0x45, 1, { 0x7F }},
    { 0x46, 1, { 0x00 }}, /* 232  <-23 */
    { 0x47, 1, { 0xAD }},
    { 0x48, 1, { 0x00 }}, /* 224  <-31 */
    { 0x49, 1, { 0xD3 }},
    { 0x4A, 1, { 0x01 }}, /* 208  <-47 */
    { 0x4B, 1, { 0x15 }},
    { 0x4C, 1, { 0x01 }}, /* 192  <-63 */
    { 0x4D, 1, { 0x4C }},
    { 0x4E, 1, { 0x01 }}, /* 160  <-95 */
    { 0x4F, 1, { 0xA4 }},
    { 0x50, 1, { 0x01 }}, /* 128  <-127 */
    { 0x51, 1, { 0xEB }},
    { 0x52, 1, { 0x01 }}, /* 127  <-128 */
    { 0x53, 1, { 0xEE }},
    { 0x54, 1, { 0x02 }}, /* 95  <-160 */
    { 0x55, 1, { 0x2E }},
    { 0x56, 1, { 0x02 }}, /* 63  <-192 */
    { 0x58, 1, { 0x73 }},
    { 0x59, 1, { 0x02 }}, /* 47  <-208 */
    { 0x5A, 1, { 0x9C }},
    { 0x5B, 1, { 0x02 }}, /* 31  <-224 */
    { 0x5C, 1, { 0xD2 }},
    { 0x5D, 1, { 0x02 }}, /* 23  <-232 */
    { 0x5E, 1, { 0xF5 }},
    { 0x5F, 1, { 0x03 }}, /* 15  <-240 */
    { 0x60, 1, { 0x24 }},
    { 0x61, 1, { 0x03 }}, /* 13  <-242 */
    { 0x62, 1, { 0x33 }},
    { 0x63, 1, { 0x03 }}, /* 11  <-244 */
    { 0x64, 1, { 0x43 }},
    { 0x65, 1, { 0x03 }}, /* 9   <-246 */
    { 0x66, 1, { 0x55 }},
    { 0x67, 1, { 0x03 }}, /* 7   <-248 */
    { 0x68, 1, { 0x6A }},
    { 0x69, 1, { 0x03 }}, /* 5   <-250 */
    { 0x6A, 1, { 0x8B }},
    { 0x6B, 1, { 0x03 }}, /* 3   <-252 */
    { 0x6C, 1, { 0xB2 }},
    { 0x6D, 1, { 0x03 }}, /* 1   <-254 */
    { 0x6E, 1, { 0xF0 }},
    { 0x6F, 1, { 0x03 }}, /* 0   <-255 */
    { 0x70, 1, { 0xFF }},
  /* ===== B+ =====*/
    { 0x71, 1, { 0x00 }}, /* 255 <-0 */
    { 0x72, 1, { 0x03 }},
    { 0x73, 1, { 0x00 }}, /* 254 <-1 */
    { 0x74, 1, { 0x0D }},
    { 0x75, 1, { 0x00 }}, /* 252 <-3 */
    { 0x76, 1, { 0x24 }},
    { 0x77, 1, { 0x00 }}, /* 250 <-5 */
    { 0x78, 1, { 0x36 }},
    { 0x79, 1, { 0x00 }}, /* 248 <-7 */
    { 0x7A, 1, { 0x45 }},
    { 0x7B, 1, { 0x00 }}, /* 246 <-9 */
    { 0x7C, 1, { 0x55 }},
    { 0x7D, 1, { 0x00 }}, /* 244 <-11 */
    { 0x7E, 1, { 0x65 }},
    { 0x7F, 1, { 0x00 }}, /* 242 <-13 */
    { 0x80, 1, { 0x72 }},
    { 0x81, 1, { 0x00 }}, /* 240 <-15 */
    { 0x82, 1, { 0x7F }},
    { 0x83, 1, { 0x00 }}, /* 232 <-23 */
    { 0x84, 1, { 0xAD }},
    { 0x85, 1, { 0x00 }}, /* 224 <-31 */
    { 0x86, 1, { 0xD3 }},
    { 0x87, 1, { 0x01 }}, /* 208 <-47 */
    { 0x88, 1, { 0x15 }},
    { 0x89, 1, { 0x01 }}, /* 192 <-63 */
    { 0x8A, 1, { 0x4C }},
    { 0x8B, 1, { 0x01 }}, /* 160 <-95 */
    { 0x8C, 1, { 0xA4 }},
    { 0x8D, 1, { 0x01 }}, /* 128 <-127 */
    { 0x8E, 1, { 0xEB }},
    { 0x8F, 1, { 0x01 }}, /* 127 <-128 */
    { 0x90, 1, { 0xEE }},
    { 0x91, 1, { 0x02 }}, /* 95  <-160 */
    { 0x92, 1, { 0x2E }},
    { 0x93, 1, { 0x02 }}, /* 63  <-192 */
    { 0x94, 1, { 0x73 }},
    { 0x95, 1, { 0x02 }}, /* 47  <-208 */
    { 0x96, 1, { 0x9C }},
    { 0x97, 1, { 0x02 }}, /* 31  <-224 */
    { 0x98, 1, { 0xD2 }},
    { 0x99, 1, { 0x02 }}, /* 23  <-232 */
    { 0x9A, 1, { 0xF5 }},
    { 0x9B, 1, { 0x03 }}, /* 15  <-240 */
    { 0x9C, 1, { 0x24 }},
    { 0x9D, 1, { 0x03 }}, /* 13  <-242 */
    { 0x9E, 1, { 0x33 }},
    { 0x9F, 1, { 0x03 }}, /* 11  <-244 */
    { 0xA0, 1, { 0x43 }},
    { 0xA2, 1, { 0x03 }}, /* 9   <-246 */
    { 0xA3, 1, { 0x55 }},
    { 0xA4, 1, { 0x03 }}, /* 7   <-248 */
    { 0xA5, 1, { 0x6A }},
    { 0xA6, 1, { 0x03 }}, /* 5   <-250 */
    { 0xA7, 1, { 0x8B }},
    { 0xA9, 1, { 0x03 }}, /* 3   <-252 */
    { 0xAA, 1, { 0xB2 }},
    { 0xAB, 1, { 0x03 }}, /* 1   <-254 */
    { 0xAC, 1, { 0xF0 }},
    { 0xAD, 1, { 0x03 }}, /* 0   <-255 */
    { 0xAE, 1, { 0xFF }},
  /* ===== B- =====*/
    { 0xAF, 1, { 0x00 }}, /* 255 <-0 */
    { 0xB0, 1, { 0x03 }},
    { 0xB1, 1, { 0x00 }}, /* 254 <-1 */
    { 0xB2, 1, { 0x0D }},
    { 0xB3, 1, { 0x00 }}, /* 252 <-3 */
    { 0xB4, 1, { 0x24 }},
    { 0xB5, 1, { 0x00 }}, /* 250 <-5 */
    { 0xB6, 1, { 0x36 }},
    { 0xB7, 1, { 0x00 }}, /* 248 <-7 */
    { 0xB8, 1, { 0x45 }},
    { 0xB9, 1, { 0x00 }}, /* 246 <-9 */
    { 0xBA, 1, { 0x55 }},
    { 0xBB, 1, { 0x00 }}, /* 244 <-11 */
    { 0xBC, 1, { 0x65 }},
    { 0xBD, 1, { 0x00 }}, /* 242 <-13 */
    { 0xBE, 1, { 0x72 }},
    { 0xBF, 1, { 0x00 }}, /* 240 <-15 */
    { 0xC0, 1, { 0x7F }},
    { 0xC1, 1, { 0x00 }}, /* 232 <-23 */
    { 0xC2, 1, { 0xAD }},
    { 0xC3, 1, { 0x00 }}, /* 224 <-31 */
    { 0xC4, 1, { 0xD3 }},
    { 0xC5, 1, { 0x01 }}, /* 208 <-47 */
    { 0xC6, 1, { 0x15 }},
    { 0xC7, 1, { 0x01 }}, /* 192 <-63 */
    { 0xC8, 1, { 0x4C }},
    { 0xC9, 1, { 0x01 }}, /* 160 <-95 */
    { 0xCA, 1, { 0xA4 }},
    { 0xCB, 1, { 0x01 }}, /* 128 <-127 */
    { 0xCC, 1, { 0xEB }},
    { 0xCD, 1, { 0x01 }}, /* 127 <-128 2C */
    { 0xCE, 1, { 0xEE }},
    { 0xCF, 1, { 0x02 }}, /* 95  <-160 */
    { 0xD0, 1, { 0x2E }},
    { 0xD1, 1, { 0x02 }}, /* 63  <-192 */
    { 0xD2, 1, { 0x73 }},
    { 0xD3, 1, { 0x02 }}, /* 47  <-208 */
    { 0xD4, 1, { 0x9C }},
    { 0xD5, 1, { 0x02 }}, /* 31  <-224 */
    { 0xD6, 1, { 0xD2 }},
    { 0xD7, 1, { 0x02 }}, /* 23  <-232 */
    { 0xD8, 1, { 0xF5 }},
    { 0xD9, 1, { 0x03 }}, /* 15  <-240 */
    { 0xDA, 1, { 0x24 }},
    { 0xDB, 1, { 0x03 }}, /* 13  <-242 */
    { 0xDC, 1, { 0x33 }},
    { 0xDD, 1, { 0x03 }}, /* 11  <-244 */
    { 0xDE, 1, { 0x43 }},
    { 0xDF, 1, { 0x03 }}, /* 9   <-246 */
    { 0xE0, 1, { 0x55 }},
    { 0xE1, 1, { 0x03 }}, /* 7   <-248 */
    { 0xE2, 1, { 0x6A }},
    { 0xE3, 1, { 0x03 }}, /* 5   <-250 */
    { 0xE4, 1, { 0x8B }},
    { 0xE5, 1, { 0x03 }}, /* 3   <-252 */
    { 0xE6, 1, { 0xB2 }},
    { 0xE7, 1, { 0x03 }}, /* 1   <-254 */
    { 0xE8, 1, { 0xF0 }},
    { 0xE9, 1, { 0x03 }}, /* 0   <-255 */
    { 0xEA, 1, { 0xFF }},
  /* -------- gamma end -------- */
  /* CMD Page Select */
    { 0xFF, 1, { 0x01 }}, /* CMD2 PAGE0 */
    { REGFLAG_UDELAY, 100, {}},
  /* PUMP CLK SETTING */
    { 0x00, 1, { 0x01 }},
    { 0x01, 1, { 0x55 }},
    { 0x02, 1, { 0x59 }},
  /* VGL&VGH PUMP SETTING */
    { 0x04, 1, { 0x0C }},
    { 0x05, 1, { 0x2A }}, 
    { 0x06, 1, { 0x5A }},
    { 0x07, 1, { 0xA9 }},
  /* GVDDP=4.87V/GVDDN=4.87V */
    { 0x0D, 1, { 0xC0 }},
    { 0x0E, 1, { 0xC0 }},
  //{ 0x0F, 1, { 0x60 }},
  /* */
  //{ 0xFF, 1, { 0xEE }}, /* Detect GNDDP/N */
  //{ 0xFB, 1, { 0x01 }},
  //{ 0x30, 1, { 0x60 }},
  /* VGHO REGULATOR=18V VGLO REGULATOR = -12V */
    { 0x10, 1, { 0x03 }},
    { 0x11, 1, { 0x5A }}, /* 5A:15V, 78:18V, 50:14V, 64:16V, 46:13V, STEP:100mV */
    { 0x12, 1, { 0x46 }}, /* 5A:-12V, 46:-10V, 3C:-9V, STEP:100mV */
  /* VCOMC3 = -2V */
    { 0x13, 1, { 0x7D }}, /* 78:-1.2V, 6A:-1.06V, STEP:10mV */
    { 0x14, 1, { 0x7E }}, /* 78:-1.2V, STEP:10mV */
    { 0x15, 1, { 0x40 }},
  /* AVDDR = 5.13V / AVEER = -5.1V */
    { 0x16, 1, { 0x16 }},
    { 0x17, 1, { 0x16 }},
  /* POWER ON VCOM */
    { 0x5B, 1, { 0x40 }},
  //{ 0x60, 1, { 0x77 }},
  /* RESX_OPTION */
    { 0x0F, 1, { 0xE0 }},
  /*RELOAD CMD1 */
    { 0xFB, 1, { 0x01 }}, /* MTP NON RELOAD */
  /* CMD Page Select */
    { 0xFF, 1, { 0x00 }}, /* Return To CMD1 */
    { REGFLAG_UDELAY, 100, {}},
  /*RELOAD CMD1 */
    { 0xFB, 1, { 0x01 }}, /* MTP NON RELOAD */
  /* VBP = 16, VFP = 16 */
    { 0xD3, 1, { LCM_VBP_NUM }},
    { 0xD4, 1, { LCM_VFP_NUM }},
  /* HBP = 105, HFP = 105 */
    { 0xD5, 1, { LCM_HBP_NUM }},
    { 0xD6, 1, { LCM_HFP_NUM }},

  /* Rotate 180 degree */
    { 0x36, 1, { 0xC0 }},


  /* Sleep Out */
    { 0x11, 0, {}},
    { REGFLAG_DELAY, 125, {}},
  /* Display ON */
    { 0x29, 0, {}},
    { REGFLAG_DELAY, 25, {}},
#if 0
  /* ===== Test mode ===== */
  /* CMD Page Select */
    { 0xFF, 1, { 0x05 }}, /* Return To CMD1 */
    { REGFLAG_UDELAY, 100, {}},
  /* */
    { 0xEC, 1, { 0x21 }}, /*  Test mode  */
    { REGFLAG_DELAY, 100, {}},
#endif
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
static void push_table_truly(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
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
static void lcm_set_util_funcs_truly(const LCM_UTIL_FUNCS *util)
{
    memcpy( &lcm_util, util, sizeof( LCM_UTIL_FUNCS ));
}

/**********************************************************
**
***********************************************************/
static void lcm_get_params_truly(LCM_PARAMS *params)
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
    params->dsi.LANE_NUM    = LCM_FOUR_LANE;  /* 4 data lane */
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
    params->dsi.PLL_CLOCK = 462;  /* 920Mbps */ //LCM_DSI_6589_PLL_CLOCK_461_5; //LCM_DSI_6589_PLL_CLOCK_468;
  #endif
#else
    params->dsi.pll_div1  = 0x00;
    params->dsi.pll_div2  = 0x00;
    params->dsi.fbk_div   = 0x01;
#endif

    params->dsi.clk_lp_per_line_enable = 0;
    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable      = 0;
    params->dsi.lcm_esd_check_table[0].cmd          = LCM_ESD_CHECK_REG;
    params->dsi.lcm_esd_check_table[0].count        = 1;
    params->dsi.lcm_esd_check_table[0].para_list[0] = LCM_ESD_CHECK_VAL;

}

/**********************************************************
** Power manager
***********************************************************/
#define   LCM_ENABLE          1
#define   LCM_DISABLE         0
#define   LCM_POWER_VOL       VOL_DEFAULT //VOL_3000
static void lcm_power_enable_truly(int enable)
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
static void lcm_gate_power_truly(int enable)
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
static void lcm_init_power_truly(void)
{
    lcm_power_enable_truly( LCM_ENABLE );
}

static void lcm_suspend_power_truly(void)
{
    lcm_power_enable_truly( LCM_DISABLE );
}

/**********************************************************
**
***********************************************************/
static void lcm_resume_power_truly(void)
{
    lcm_power_enable_truly( LCM_ENABLE );
}

/**********************************************************
**
***********************************************************/
static void lcm_init_truly(void)
{
#if !defined( FPGA_EARLY_PORTING )
    lcm_gate_power_truly( LCM_ENABLE );
    MDELAY( 12 );
#endif /* End.. !(FPGA_EARLY_PORTING) */

    SET_RESET_PIN( 1 );
    MDELAY( 3 );
    SET_RESET_PIN( 0 );
    MDELAY( 20 );
    SET_RESET_PIN( 1 );
    MDELAY( 20 );

  /* when phone initial, config output high, enable backlight drv chip */
    push_table_truly( lcm_initialization_setting, sizeof( lcm_initialization_setting ) / sizeof( struct LCM_setting_table ), 1 );
}

/**********************************************************
**
***********************************************************/
static void lcm_suspend_truly(void)
{
    lcm_gate_enable( FALSE );
    push_table_truly( lcm_suspend_setting, sizeof( lcm_suspend_setting ) / sizeof( struct LCM_setting_table ), 1 );
  //SET_RESET_PIN( 0 );
    MDELAY( 10 );
}

/**********************************************************
**
***********************************************************/
static void lcm_resume_truly(void)
{
    lcm_init_truly();
}

#if( LCM_DSI_CMD_MODE )
/**********************************************************
**
***********************************************************/
static void lcm_update_truly(unsigned int x, unsigned int y, unsigned int width, unsigned int height)
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
static unsigned int lcm_compare_id_truly(void)
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
    dprintf( 0, "[LK]%s: LCM chip ID = 0x%04X\n", __func__, id );
#else
    printk("[Kernel]%s: LCM chip ID = 0x%04X\n", __func__, id );
#endif

    vRet = 0;

//<2014/10/17-stevenchen, Add Truly R61335 LCM driver
    if( LCM_PANEL_SOURCE == lcm_id_gpio ) /* Truly */
    {
	if( id == LCM_ID_NT35532 ) /* Truly NT35532 */
	    vRet=1;
    }
    else /* Innolux */
	vRet=0;
//>2014/10/17-stevenchen

    return vRet;
}

/**********************************************************
** return TRUE : Need recovery
** return FALSE: No need recovery
***********************************************************/
static unsigned int lcm_esd_check_truly(void)
{
#if !defined( BUILD_LK )
int   array[4];
char  buffer[4];

    array[0] = 0x00013700;
    dsi_set_cmdq( array, 1, 1 );

    read_reg_v2( LCM_ESD_CHECK_REG, buffer, 1 );

    if( buffer[0] != LCM_ESD_CHECK_VAL )
    {
      printk("[LCM ERROR]%s: 0x%02X = 0x%02X\n", __func__, LCM_ESD_CHECK_REG, buffer[0] );
      return TRUE;
    }
    else
    {
      printk("[LCM NORMAL]%s: 0x%02X = 0x%02X\n", __func__, LCM_ESD_CHECK_REG, buffer[0] );
      return FALSE;
    }
#else
    return FALSE;
#endif

}

/**********************************************************
**
***********************************************************/
unsigned int lcm_ata_check_truly(unsigned char *buffer)
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
static void lcm_setbacklight_truly(unsigned int level)
{
#if defined( BUILD_LK )
    dprintf( 0, "%s, LK nt35532 backlight: level = %d\n", __func__, level );
#else
    printk("%s, Kernel nt35532 backlight: level = %d\n", __func__, level );
#endif
  /* Refresh value of backlight level. */
    lcm_backlight_level_setting[0].para_list[0] = level;

    push_table_truly( lcm_backlight_level_setting, sizeof( lcm_backlight_level_setting ) / sizeof( struct LCM_setting_table ), 1 );
}

/**********************************************************
**
***********************************************************/
void* lcm_switch_mode_truly(int mode)
{
#if !defined( BUILD_LK )
/* customization: 1. V2C config 2 values, C2V config 1 value; 2. config mode control register */
    if( mode == 0 )
    { /* V2C */
      lcm_switch_mode_cmd_truly.mode    = CMD_MODE;
      lcm_switch_mode_cmd_truly.addr    = 0xBB; /* mode control addr */
      lcm_switch_mode_cmd_truly.val[0]  = 0x13; /* enabel GRAM firstly, ensure writing one frame to GRAM */
      lcm_switch_mode_cmd_truly.val[1]  = 0x10; /* disable video mode secondly */
    }
    else
    { /* V2C */
      lcm_switch_mode_cmd_truly.mode    = SYNC_PULSE_VDO_MODE;
      lcm_switch_mode_cmd_truly.addr    = 0xBB;
      lcm_switch_mode_cmd_truly.val[0]  = 0x03; /* disable GRAM and enable video mode */
    }

    return (void*)( &lcm_switch_mode_cmd_truly );
#else
    return NULL;
#endif
}

/**********************************************************
**
***********************************************************/
LCM_DRIVER nt35532_fhd_dsi_vdo_truly_lcm_drv=
{
    .name             = "nt35532_fhd_dsi_vdo_truly_drv",
    .set_util_funcs   = lcm_set_util_funcs_truly,
    .get_params       = lcm_get_params_truly,
    .init             = lcm_init_truly,   /*tianma init fun.*/
    .suspend          = lcm_suspend_truly,
    .resume           = lcm_resume_truly,
    .compare_id       = lcm_compare_id_truly,
    .init_power       = lcm_init_power_truly,
    .resume_power     = lcm_resume_power_truly,
    .suspend_power    = lcm_suspend_power_truly,
    .esd_check        = lcm_esd_check_truly,
    .set_backlight    = lcm_setbacklight_truly,
    .ata_check        = lcm_ata_check_truly,
#if( LCM_DSI_CMD_MODE )
    .update           = lcm_update_truly,
#endif
    .switch_mode      = lcm_switch_mode_truly,
};

#undef NT35532_FHD_DSI_VDO_TRULY_C
#endif
//>2014/09/23-stevenchen
