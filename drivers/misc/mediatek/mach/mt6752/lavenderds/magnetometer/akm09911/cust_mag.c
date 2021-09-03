#include <linux/types.h>
#include <mach/mt_pm_ldo.h>
#include <cust_mag.h>

static struct mag_hw cust_mag_hw = {
    .i2c_num = 1,
// [Lavender][E-Compas][akenhsu] Fix different orientation of E-Compas of Lavender 20150225 BEGIN
#if defined(COSMOS)
// [Lavender][E-Compas][akenhsu] 20150225 END
	//<2014/09/26-JackHu, Modify M sensor direction
    .direction = 2,
	//<2014/09/26-JackHu
// [Lavender][E-Compas][akenhsu] Fix different orientation of E-Compas of Lavender 20150225 BEGIN
#elif defined(LAVENDER)
    .direction = 7,
#endif
// [Lavender][E-Compas][akenhsu] 20150225 END
    .power_id = MT6325_POWER_LDO_VIO28,  /*!< LDO is not used */
	//<2014/11/04-JackHu, Modify M sensor power setting
    .power_vol= VOL_2800,        /*!< LDO is not used */
	//<2014/11/04-JackHu
};
struct mag_hw* get_cust_mag_hw(void) 
{
    return &cust_mag_hw;
}
