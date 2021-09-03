include $(srctree)/drivers/misc/mediatek/Makefile.custom
#// <<< 2016/02/06-youchihwang, Porting charger from Cosmos/Lavender Android L to Android M
#<2014/11/10-tedwu, Added for charging IC fan54015.
obj-$(CONFIG_MTK_PMIC)          += pmic.o upmu_common.o pmic_auxadc.o pmic_chr_type_det.o mt6311.o

#ifeq ($(ARIMA_HW_VERSION),HW_PDP1)
#     obj-$(CONFIG_MTK_PMIC)     += tps6128x.o
#endif
#>2014/10/30-tedwu
#// >>> 2016/02/06-youchihwang, Porting charger from Cosmos/Lavender Android L to Android M
ifeq ($(CONFIG_MTK_PMIC_DVT_SUPPORT),y)
     obj-$(CONFIG_MTK_PMIC)     += pmic_dvt.o
endif

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_hal.o 

ifeq ($(CONFIG_MTK_BQ24261_SUPPORT),y)
     obj-$(CONFIG_MTK_SMART_BATTERY) += bq24261.o charging_hw_bq24261.o
else
     ifeq ($(CONFIG_MTK_SWCHR_SUPPORT),y)
          obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_mt_sw_charger.o                
     else
          ifeq ($(CONFIG_MTK_NCP1854_SUPPORT),y)
               obj-$(CONFIG_MTK_SMART_BATTERY) += ncp1854.o charging_hw_ncp1854.o             
          else
#// <<< 2016/02/06-youchihwang, Porting charger from Cosmos/Lavender Android L to Android M
               #//<2014/11/3-youchihwang, FP19364 Support USB Host mode, [FAQ06610] [USB] USB OTG
               #<2014/08/14-tedwu, Added for charging IC fan54015.
               ifeq ($(CONFIG_MTK_FAN5405_SUPPORT),y)
               #//>2014/11/3-youchihwang, FP19364 Support USB Host mode, [FAQ06610] [USB] USB OTG
                    obj-$(CONFIG_MTK_SMART_BATTERY) += fan54015.o charging_hw_fan54015.o
               else
                   ifeq ($(CONFIG_MTK_RT9536_SUPPORT),y)
                       obj-$(CONFIG_MTK_SMART_BATTERY) += rt9536/
                   else 
                       ifeq ($(CONFIG_MTK_BQ24196_SUPPORT),y)
                           obj-$(CONFIG_MTK_SMART_BATTERY) += bq24196.o charging_hw_bq24196.o
                       else
                           obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_pmic.o 
                       endif
                   endif
               #>2014/08/14-tedwu
               endif
#// >>> 2016/02/06-youchihwang, Porting charger from Cosmos/Lavender Android L to Android M
          endif
      endif
endif
