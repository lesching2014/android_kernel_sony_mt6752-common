#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_alsps.h>

static struct alsps_hw cust_alsps_hw = {
    .i2c_num    = 1,
    .polling_mode_ps =1,///0,
    .polling_mode_als =1,
    .power_id   = MT6325_POWER_LDO_VIO28,///MT6323_POWER_LDO_VIO28,///MT65XX_POWER_NONE,    /*LDO is not used*/
    .power_vol  = VOL_2800,///VOL_3300,///VOL_DEFAULT,          /*LDO is not used*/
    .i2c_addr   = {0x92, 0x48, 0x78, 0x00},
    .als_level  = { 0,  1,  1,   7,  15,  15,  100, 1000, 2000,  3000,  6000, 10000, 14000, 18000, 20000},
    .als_value  = {40, 40, 90,  90, 160, 160,  225,  320,  640,  1280,  1280,  2600,  2600, 2600,  10240, 10240},
/*
//<2014/09/30-ShermanWei,for PDP2HW
#if defined HW_PDP1
    .ps_threshold_low = 1500,///400,
    .ps_threshold_high = 2000,///800,
//<2014/11/25-Quakentsai, for calibration APK interface
#elif (defined(HW_PDP2))
    .ps_threshold_low = 7930,///400,
    .ps_threshold_high = 8010,///800,
#elif (defined(HW_DP))
    .ps_threshold_low = 2000,///400,
    .ps_threshold_high = 3000,///800,
#elif (defined(HW_SP))
    .ps_threshold_low = 2000,///400,
    .ps_threshold_high = 3000,///800,
#elif (defined(HW_AP))
    .ps_threshold_low = 2000,///400,
    .ps_threshold_high = 3000,///800,
#else
//>2014/11/25-Quakentsai, for calibration APK interface
    .ps_threshold_low = 8000,
    .ps_threshold_high = 10000,
#endif
//>2014/09/30-ShermanWei,
*/
    .ps_threshold_high = 3000,
    .ps_threshold_low = 2000,
};
struct alsps_hw *get_cust_alsps_hw(void) {
    return &cust_alsps_hw;
}

