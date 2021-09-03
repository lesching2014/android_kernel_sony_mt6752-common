#ifndef _CUST_BATTERY_METER_H
#define _CUST_BATTERY_METER_H

#include <mach/mt_typedefs.h>

// ============================================================
// define
// ============================================================
//#define SOC_BY_AUXADC
#define SOC_BY_HW_FG
//#define SOC_BY_SW_FG
//#define HW_FG_FORCE_USE_SW_OCV

//#define CONFIG_DIS_CHECK_BATTERY
//#define FIXED_TBAT_25

/* ADC resistor  */
#define R_BAT_SENSE 4
#define R_I_SENSE 4
#define R_CHARGER_1 330
#define R_CHARGER_2 39

#define TEMPERATURE_T0             110
#define TEMPERATURE_T1             0
#define TEMPERATURE_T2             25
#define TEMPERATURE_T3             50
#define TEMPERATURE_T              255 // This should be fixed, never change the value

#define FG_METER_RESISTANCE 	5

/*[Lavender][bozhi_lin] apply Glymii ZCV table from MTK 20150211 begin*/
#if defined(COSMOS)
//<2014/10/21-tedwu, Customize the battery charging tables.
/* Qmax for battery  */
#define Q_MAX_POS_50_SONY_1282_1203     2657
#define Q_MAX_POS_25_SONY_1282_1203     2586
#define Q_MAX_POS_0_SONY_1282_1203      2200
#define Q_MAX_NEG_10_SONY_1282_1203     345

#define Q_MAX_POS_50_H_CURRENT_SONY_1282_1203	2604
#define Q_MAX_POS_25_H_CURRENT_SONY_1282_1203	2534
#define Q_MAX_POS_0_H_CURRENT_SONY_1282_1203	2156
#define Q_MAX_NEG_10_H_CURRENT_SONY_1282_1203	338

#define Q_MAX_POS_50_SONY_1294_6964 	2657
#define Q_MAX_POS_25_SONY_1294_6964 	2613
#define Q_MAX_POS_0_SONY_1294_6964		1605
#define Q_MAX_NEG_10_SONY_1294_6964 	342

#define Q_MAX_POS_50_H_CURRENT_SONY_1294_6964	2604
#define Q_MAX_POS_25_H_CURRENT_SONY_1294_6964	2561
#define Q_MAX_POS_0_H_CURRENT_SONY_1294_6964	1573
#define Q_MAX_NEG_10_H_CURRENT_SONY_1294_6964	 335

/* Discharge Percentage */
#define OAM_D5		 0		//  1 : D5,   0: D2
//>2014/10/21-tedwu

#elif defined(LAVENDER)
/* Qmax for battery  */
//<2015/09/11-JackHu, Porting GM2.0 for Lavender MR0
#define Q_MAX_POS_50	2993
#define Q_MAX_POS_25	2928
#define Q_MAX_POS_0		2579
#define Q_MAX_NEG_10	298

#define Q_MAX_POS_50_H_CURRENT	2933
#define Q_MAX_POS_25_H_CURRENT	2869
#define Q_MAX_POS_0_H_CURRENT	2527
#define Q_MAX_NEG_10_H_CURRENT	292
//<2015/09/11-JackHu

/* Discharge Percentage */
#define OAM_D5		 0		//  1 : D5,   0: D2
//<2015/03/19-stevenchen, Fix kernel build error for open source kernel package.
#else
/* Qmax for battery  */
#define Q_MAX_POS_50	2336
#define Q_MAX_POS_25	2335
#define Q_MAX_POS_0		942
#define Q_MAX_NEG_10	50

#define Q_MAX_POS_50_H_CURRENT	2289
#define Q_MAX_POS_25_H_CURRENT	2288
#define Q_MAX_POS_0_H_CURRENT	  923
#define Q_MAX_NEG_10_H_CURRENT	49


/* Discharge Percentage */
#define OAM_D5		 1		//  1 : D5,   0: D2
//>2015/03/19-stevenchen
#endif
/*[Lavender][bozhi_lin] 20150211 end*/

/* battery meter parameter */
#define CHANGE_TRACKING_POINT
//<2015/09/11-JackHu, Porting GM2.0 for Lavender MR0
#define CUST_TRACKING_POINT  0//1
//<2015/09/11-JackHu
#define CUST_R_SENSE         56  //<2014/10/21-tedwu, Customize the battery charging tables.//>
#define CUST_HW_CC 		    0
#define AGING_TUNING_VALUE   103
#define CUST_R_FG_OFFSET    0

#define OCV_BOARD_COMPESATE	0 //mV
#define R_FG_BOARD_BASE		1000
#define R_FG_BOARD_SLOPE	1000 //slope
//<2015/03/17-tedwu, change car_tune_value by hardware team request.
#if defined(COSMOS)
#define CAR_TUNE_VALUE		102 //1.02   //97 //1.00
#elif defined(LAVENDER)
#define CAR_TUNE_VALUE		97 //1.00
#else
#define CAR_TUNE_VALUE		97 //1.00
#endif
//>2015/03/17-tedwu

/* HW Fuel gague  */
#define CURRENT_DETECT_R_FG	10  //1mA
#define MinErrorOffset       1000
#define FG_VBAT_AVERAGE_SIZE 18
#define R_FG_VALUE 			10 // mOhm, base is 20
//<2015/09/11-JackHu, Porting GM2.0 for Lavender MR0
/* fg 2.0 */
#define DIFFERENCE_HWOCV_RTC		30
#define DIFFERENCE_HWOCV_SWOCV		10
#define DIFFERENCE_SWOCV_RTC		10
#define MAX_SWOCV			3

#define DIFFERENCE_VOLTAGE_UPDATE	20
#define AGING1_LOAD_SOC			70
#define AGING1_UPDATE_SOC		30
#define BATTERYPSEUDO100		97
#define BATTERYPSEUDO1			4

#define Q_MAX_BY_SYS			//8. Qmax varient by system drop voltage.
#define Q_MAX_SYS_VOLTAGE		3350
#define SHUTDOWN_GAUGE0
#define SHUTDOWN_GAUGE1_XMINS
#define SHUTDOWN_GAUGE1_MINS		60

#define SHUTDOWN_SYSTEM_VOLTAGE		3400
#define CHARGE_TRACKING_TIME		60
#define DISCHARGE_TRACKING_TIME		10

#define RECHARGE_TOLERANCE		10
/* SW Fuel Gauge */
#define MAX_HWOCV			5
#define MAX_VBAT			90
#define DIFFERENCE_HWOCV_VBAT		30
//<2015/09/11-JackHu


//<2015/05/09-tedwu, [DMS06420703] Cannot restart phone at 13% battery
#if defined(COSMOS)
#define HW_FG_FORCE_USE_SW_OCV
#define CUST_POWERON_DELTA_CAPACITY_TOLRANCE	3  //25
#define CUST_POWERON_LOW_CAPACITY_TOLRANCE		5
#define CUST_POWERON_MAX_VBAT_TOLRANCE			94 //90
#define CUST_POWERON_DELTA_VBAT_TOLRANCE		30
#else
//<2015/02/16-tedwu, [DMS06327336][DMS06328235]Customize the tolerance of battery capacity and voltage.
#define CUST_POWERON_DELTA_CAPACITY_TOLRANCE	10 //25
#define CUST_POWERON_LOW_CAPACITY_TOLRANCE		5
#define CUST_POWERON_MAX_VBAT_TOLRANCE			94 //90
#define CUST_POWERON_DELTA_VBAT_TOLRANCE		30
//>2015/02/16-tedwu
#endif
//>2015/05/09-tedwu

/* Disable Battery check for HQA */
#ifdef CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
#define FIXED_TBAT_25
#endif

//<2015/03/28-tedwu, check temperature for wakeup time
#if defined(COSMOS) || defined(LAVENDER)
//#if defined(COSMOS)
//<2015/08/13-JackHu, remove temperature detect for GM2.0
//#define CHK_TEMPERATURE_FOR_WAKEUP_PERIOD
//<2015/08/13-JackHu
#endif
//<2015/01/18-tedwu, For battery over temperature protection.
//<2015/05/09-tedwu, [DMS06420703] Cannot restart phone at 13% battery
/* Dynamic change wake up period of battery thread when suspend*/
#define VBAT_NORMAL_WAKEUP		3600		//3.6V
#define VBAT_LOW_POWER_WAKEUP		3500		//3.5v
#if defined(COSMOS)
#define NORMAL_WAKEUP_PERIOD		5400  //10*60 = 10 min  //<2015/05/13-tedwu, change 30 to 10min //>
#else
#define NORMAL_WAKEUP_PERIOD		5400 		//90 * 60 = 90 min
#endif
#define LOW_POWER_WAKEUP_PERIOD		200 //300		//5 * 60 = 5 min
#define CLOSE_POWEROFF_WAKEUP_PERIOD	30	//30 s
#if defined(CHK_TEMPERATURE_FOR_WAKEUP_PERIOD)
#define TEMPERATURE_CHANGED_CHK_PERIOD    120
#define TEMPERATURE_CHANGED_WAKEUP_PERIOD  40
#define HIGH_TEMPERATURE_WAKEUP_PERIOD     30
#define MAX_TEMPERATURE_WAKEUP_PERIOD      20
#endif
//>2015/05/09-tedwu
//>2015/01/18-tedwu
//>2015/03/28-tedwu

/*[Lavender][bozhi_lin] always restore battery capacity when boot-up 20150526 begin*/
#if defined(LAVENDER)

#else
#define INIT_SOC_BY_SW_SOC
#endif
/*[Lavender][bozhi_lin] 20150526 end*/
//#define SYNC_UI_SOC_IMM			//3. UI SOC sync to FG SOC immediately
#define MTK_ENABLE_AGING_ALGORITHM	//6. Q_MAX aging algorithm
#define MD_SLEEP_CURRENT_CHECK	//5. Gauge Adjust by OCV 9. MD sleep current check
//#define Q_MAX_BY_CURRENT		//7. Qmax varient by current loading.

#define DISABLE_RFG_EXIST_CHECK
#endif	//#ifndef _CUST_BATTERY_METER_H
