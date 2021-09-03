#include "tpd.h"

int tpd_calibrate_en = 0;
module_param(tpd_calibrate_en, int, 00664);

int tpd_show_version = 0;
module_param(tpd_show_version, int, 00664);
//[SM20] zihwei add store touch vendor and firmware begin 2015/11/12
/*[Lavender][bozhi_lin] store touch vendor and firmware version to tpd_show_vendor_firmware 20150213 begin*/
#if defined(TPD_REPORT_VENDOR_FW)
char *tpd_show_vendor_firmware = NULL;
module_param(tpd_show_vendor_firmware, charp, 00664);
#endif
/*[Lavender][bozhi_lin] 20150213 end*/
//[SM20] zihwei add store touch vendor and firmware end 2015/11/12

/* switch touch panel into single scan mode for decreasing interference */
void tpd_switch_single_mode(void)
{
#ifdef HAVE_SINGLE_MULTIPLE_SCAN_MODE
	_tpd_switch_single_mode();
#endif
}
EXPORT_SYMBOL(tpd_switch_single_mode);

/* switch touch panel into multiple scan mode for better performance */
void tpd_switch_multiple_mode(void)
{
#ifdef HAVE_SINGLE_MULTIPLE_SCAN_MODE
	_tpd_switch_multiple_mode();
#endif
}
EXPORT_SYMBOL(tpd_switch_multiple_mode);

/* switch touch panel into deep sleep mode */
void tpd_switch_sleep_mode(void)
{
#ifdef HAVE_SLEEP_NORMAL_MODE
	_tpd_switch_sleep_mode();
#endif
}
EXPORT_SYMBOL(tpd_switch_sleep_mode);

/* switch touch panel back to normal mode */
void tpd_switch_normal_mode(void)
{
#ifdef HAVE_SLEEP_NORMAL_MODE
	_tpd_switch_normal_mode();
#endif
}
EXPORT_SYMBOL(tpd_switch_normal_mode);
