#ifndef _CUST_BAT_H_
#define _CUST_BAT_H_

/* stop charging while in talking mode */
#define STOP_CHARGING_IN_TAKLING
//<2015/02/02-tedwu, Change charging voltage during call mode.
#define TALKING_RECHARGE_VOLTAGE 3850
//>2015/02/02-tedwu
#define TALKING_SYNC_TIME		   60

/* Battery Temperature Protection */
#define MTK_TEMPERATURE_RECHARGE_SUPPORT
//<2015/01/18-tedwu, For battery over temperature protection.
#define MAX_CHARGE_TEMPERATURE  55//50
#define MAX_CHARGE_TEMPERATURE_MINUS_X_DEGREE	55//47
//<2016/06/07-stevenchen, Fix OSS build error
#define MAX_RECHARGE_TEMPERATURE  45 //42    /* After STOP charge. */
//>2016/06/07-stevenchen

#define MIN_CHARGE_TEMPERATURE  5//0
#define MIN_CHARGE_TEMPERATURE_PLUS_X_DEGREE	8//6  /* After STOP charge. */
//<2016/06/07-stevenchen, Fix OSS build error
#define MIN_RECHARGE_TEMPERATURE   5 //(-5)  /* After STOP charge. */
//>2016/06/07-stevenchen
#define ERR_CHARGE_TEMPERATURE  0xFF
//>2015/01/18-tedwu

/* Linear Charging Threshold */
#define V_PRE2CC_THRES	 		3400	//mV
//<2015/02/02-tedwu, Change charging voltage during call mode.
#define V_CC2TOPOFF_THRES		3900
//>2015/02/02-tedwu
#define RECHARGING_VOLTAGE      4110
#define CHARGING_FULL_CURRENT    150	//mA
//<2015/01/18-tedwu, For battery over temperature protection.
/* Over 45 degree and under 55 degree
**  - Under 4.0V, 450mA
**  - Over 4.0V, Stop charging */
#define TEMP_CHECK_CHARGING_VOLTAGE   4000
#define TEMP_CHECK_CHARGER_CURRENT    CHARGE_CURRENT_400_00_MA
//>2015/01/18-tedwu

/*[Lavender][bozhi_lin] Meet SPEC, modify safety charging current to 800mA 20150526 begin*/
/*[Lavender][bozhi_lin] for battery temperature safety set different current and voltage 20150415 begin*/
//<2015/03/29-tedwu, for new charge algorithm.
#if defined(COSMOS)
#define SAFETY_CHARGER_IN_CURRENT  CHARGE_CURRENT_500_00_MA
#define SAFETY_CHARGER_CURRENT     CHARGE_CURRENT_775_00_MA
#elif defined(LAVENDER)
#define SAFETY_CHARGER_IN_CURRENT  CHARGE_CURRENT_800_00_MA
#define SAFETY_CHARGER_CURRENT     CHARGE_CURRENT_800_00_MA
#endif
//>2015/03/29-tedwu
/*[Lavender][bozhi_lin] 20150415 end*/
/*[Lavender][bozhi_lin] 20150526 end*/

/* Charging Current Setting */
//#define CONFIG_USB_IF 						   
#define USB_CHARGER_CURRENT_SUSPEND			0		// def CONFIG_USB_IF
#define USB_CHARGER_CURRENT_UNCONFIGURED	CHARGE_CURRENT_70_00_MA	// 70mA
#define USB_CHARGER_CURRENT_CONFIGURED		CHARGE_CURRENT_500_00_MA	// 500mA

//<2015/02/01-tedwu, Change charging currents.
#define USB_CHARGER_IN_CURRENT				CHARGE_CURRENT_500_00_MA
#define USB_CHARGER_CURRENT					CHARGE_CURRENT_675_00_MA  //37.4/56=667.86mA //<2015/02/01-tedwu,//>
//<2015/03/29-tedwu, change ac charger current.
#if defined(COSMOS)
//For battery pack Sony Energy Devices 1282-1203.
#define AC_CHARGER_CURRENT					CHARGE_CURRENT_1750_00_MA
#elif defined(LAVENDER)
#define AC_CHARGER_CURRENT					CHARGE_CURRENT_1375_00_MA
#else
#define AC_CHARGER_CURRENT					CHARGE_CURRENT_1375_00_MA
#endif
//>2015/03/29-tedwu
#define NON_STD_AC_CHARGER_IN_CURRENT		CHARGE_CURRENT_500_00_MA
#define NON_STD_AC_CHARGER_CURRENT			CHARGE_CURRENT_675_00_MA  //37.4/56=667.86mA //<2015/02/01-tedwu,//>
//<2015/11/11-JackHu, change charging host current to 1A for improve ESTA
//#define CHARGING_HOST_CHARGER_IN_CURRENT    CHARGE_CURRENT_500_00_MA
//#define CHARGING_HOST_CHARGER_CURRENT       CHARGE_CURRENT_675_00_MA  //37.4/56=667.86mA //<2015/02/01-tedwu,//>
#define CHARGING_HOST_CHARGER_IN_CURRENT    CHARGE_CURRENT_1000_00_MA
#define CHARGING_HOST_CHARGER_CURRENT       CHARGE_CURRENT_1000_00_MA
//<2015/11/11-JackHu
#define APPLE_0_5A_CHARGER_CURRENT          CHARGE_CURRENT_500_00_MA
#define APPLE_1_0A_CHARGER_CURRENT          CHARGE_CURRENT_650_00_MA
#define APPLE_2_1A_CHARGER_CURRENT          CHARGE_CURRENT_800_00_MA
//>2015/02/01-tedwu

/* Precise Tunning */
#define BATTERY_AVERAGE_DATA_NUMBER	3	
#define BATTERY_AVERAGE_SIZE 	30

/* charger error check */
//#define BAT_LOW_TEMP_PROTECT_ENABLE         // stop charging if temp < MIN_CHARGE_TEMPERATURE
#define V_CHARGER_ENABLE 0				// 1:ON , 0:OFF	
#define V_CHARGER_MAX 6500				// 6.5 V
#define V_CHARGER_MIN 4400				// 4.4 V

/* Tracking TIME */
#define ONEHUNDRED_PERCENT_TRACKING_TIME	10	// 10 second
#define NPERCENT_TRACKING_TIME	   			20	// 20 second
#define SYNC_TO_REAL_TRACKING_TIME  		60	// 60 second
#define V_0PERCENT_TRACKING							3450 //3450mV

/* Battery Notify */
#define BATTERY_NOTIFY_CASE_0001_VCHARGER
#define BATTERY_NOTIFY_CASE_0002_VBATTEMP
//#define BATTERY_NOTIFY_CASE_0003_ICHARGING
//#define BATTERY_NOTIFY_CASE_0004_VBAT
//<2014/12/10-tedwu, turn on the notification.
#define BATTERY_NOTIFY_CASE_0005_TOTAL_CHARGINGTIME
//>2014/12/10-tedwu

/* High battery support */
#define HIGH_BATTERY_VOLTAGE_SUPPORT
//<2014/09/27-tedwu, Disable PMIC low voltage workaround on HW PDP2.
//<2015/01/20-tedwu, Add workaround for OVP_vbat of OVP chip due to layout.
#if defined(COSMOS)
  #if defined(HW_PDP1)
  #define PMIC_LOW_VOLTAGE_WORKAROUND  // TODO: Ted: could use PMIC version number to instead.
  //<2015/01/27-tedwu, Disable workaround for OVP_vbat
  /*
  #elif defined(HW_PDP2) || defined(HW_DP) || defined(HW_SP)
  #define OVP_VBAT_WORKAROUND
  */
  //>2015/01/27-tedwu
  #endif //defined(HW_XXX)
#endif //defined(COSMOS)
//>2015/01/20-tedwu
//>2014/09/27-tedwu
//>2014/08/14-tedwu

/* JEITA parameter */
//#define MTK_JEITA_STANDARD_SUPPORT
#define CUST_SOC_JEITA_SYNC_TIME 30
#define JEITA_RECHARGE_VOLTAGE  4110	// for linear charging
#ifdef HIGH_BATTERY_VOLTAGE_SUPPORT
#define JEITA_TEMP_ABOVE_POS_60_CV_VOLTAGE		BATTERY_VOLT_04_240000_V
#define JEITA_TEMP_POS_45_TO_POS_60_CV_VOLTAGE		BATTERY_VOLT_04_240000_V
#define JEITA_TEMP_POS_10_TO_POS_45_CV_VOLTAGE		BATTERY_VOLT_04_340000_V
#define JEITA_TEMP_POS_0_TO_POS_10_CV_VOLTAGE		BATTERY_VOLT_04_240000_V
#define JEITA_TEMP_NEG_10_TO_POS_0_CV_VOLTAGE		BATTERY_VOLT_04_040000_V
#define JEITA_TEMP_BELOW_NEG_10_CV_VOLTAGE		BATTERY_VOLT_04_040000_V
#else
#define JEITA_TEMP_ABOVE_POS_60_CV_VOLTAGE		BATTERY_VOLT_04_100000_V
#define JEITA_TEMP_POS_45_TO_POS_60_CV_VOLTAGE	BATTERY_VOLT_04_100000_V
#define JEITA_TEMP_POS_10_TO_POS_45_CV_VOLTAGE	BATTERY_VOLT_04_200000_V
#define JEITA_TEMP_POS_0_TO_POS_10_CV_VOLTAGE	BATTERY_VOLT_04_100000_V
#define JEITA_TEMP_NEG_10_TO_POS_0_CV_VOLTAGE	BATTERY_VOLT_03_900000_V
#define JEITA_TEMP_BELOW_NEG_10_CV_VOLTAGE		BATTERY_VOLT_03_900000_V
#endif
/* For JEITA Linear Charging only */
#define JEITA_NEG_10_TO_POS_0_FULL_CURRENT  120	//mA 
#define JEITA_TEMP_POS_45_TO_POS_60_RECHARGE_VOLTAGE  4000
#define JEITA_TEMP_POS_10_TO_POS_45_RECHARGE_VOLTAGE  4100
#define JEITA_TEMP_POS_0_TO_POS_10_RECHARGE_VOLTAGE   4000
#define JEITA_TEMP_NEG_10_TO_POS_0_RECHARGE_VOLTAGE   3800
#define JEITA_TEMP_POS_45_TO_POS_60_CC2TOPOFF_THRESHOLD	4050
#define JEITA_TEMP_POS_10_TO_POS_45_CC2TOPOFF_THRESHOLD	4050
#define JEITA_TEMP_POS_0_TO_POS_10_CC2TOPOFF_THRESHOLD	4050
#define JEITA_TEMP_NEG_10_TO_POS_0_CC2TOPOFF_THRESHOLD	3850


/* For CV_E1_INTERNAL */
#define CV_E1_INTERNAL

/* Disable Battery check for HQA */
#ifdef CONFIG_MTK_DISABLE_POWER_ON_OFF_VOLTAGE_LIMITATION
#define CONFIG_DIS_CHECK_BATTERY
#endif

#ifdef CONFIG_MTK_FAN5405_SUPPORT
// <<< 2016/02/06-youchihwang, Porting charger from Cosmos/Lavender Android L to Android M
//<2014/09/04-tedwu, Change the I2C port number to charger IC.
#define FAN5405_BUSNUM 0
//>2014/09/04-tedwu
#endif
// >>> 2016/02/06-youchihwang, Porting charger from Cosmos/Lavender Android L to Android M
/*[Lavender][bozhi_lin] set AC timer to 5 hours and USB to 100 hours 20150415 begin*/
#define AC_CHARGING_TIMER   (5 * 60 * 60)	// 5 hours
#define USB_CHARGING_TIMER  (100 * 60 * 60) // 100 hours = 6000 mins = 360000 seconds
/*[Lavender][bozhi_lin] 20150415 end*/

/*[Lavender][bozhi_lin] enable charging maintenance 20150522 begin*/
#define CHARGING_MAINTAIN
/*[Lavender][bozhi_lin] 20150522 end*/

/*[Lavender][bozhi_lin] fix once enter maintain state, the battery capacity will always keep 100 until remove usb cable 20150608 begin*/
#if defined(CHARGING_MAINTAIN)
#define V_FULL2CC_THRES			4150
#endif
/*[Lavender][bozhi_lin] 20150608 end*/
//<2015/07/16-JackHu, check voltage to determine add charging timer fix DMS06428196.
/*[Lavender][bozhi_lin] add voltage check to avoid trigger charging timeout 20150708 begin*/
#define CHARGING_TIMEOUT_ACTIVE_PROTECT_VOLTAGE	4100
#define CHARGING_TIMEOUT_COMPENSATION_TIME	(1 * 60 * 60)
/*[Lavender][bozhi_lin] 20150708 end*/
//<2015/07/16-JackHu
#endif /* _CUST_BAT_H_ */ 
