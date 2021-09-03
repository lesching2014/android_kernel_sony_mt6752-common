/*[i53M][Zihweishen]  Modify GPIO setting function for android M begin 2016/01/26*/
//#define GTP_GPIO_AS_INT(pin) tpd_gpio_as_int(pin)
#define GTP_GPIO_AS_INPUT(pin)          do {\
	if (pin == GPIO_CTP_EINT_PIN)\
		mt_set_gpio_mode(pin, GPIO_CTP_EINT_PIN_M_GPIO);\
	else\
		mt_set_gpio_mode(pin, GPIO_CTP_RST_PIN_M_GPIO);\
	mt_set_gpio_dir(pin, GPIO_DIR_IN);\
	mt_set_gpio_pull_enable(pin, GPIO_PULL_DISABLE);\
} while (0)

#define GTP_GPIO_AS_INT(pin)            do {\
	mt_set_gpio_mode(pin, GPIO_CTP_EINT_PIN_M_EINT);\
	mt_set_gpio_dir(pin, GPIO_DIR_IN);\
	mt_set_gpio_pull_enable(pin, GPIO_PULL_DISABLE);\
} while (0)

//#define GTP_GPIO_OUTPUT(pin, level) tpd_gpio_output(pin, level)
#define GTP_GPIO_OUTPUT(pin, level)      do {\
	if (pin == GPIO_CTP_EINT_PIN)\
		mt_set_gpio_mode(pin, GPIO_CTP_EINT_PIN_M_GPIO);\
	else\
		mt_set_gpio_mode(pin, GPIO_CTP_RST_PIN_M_GPIO);\
	mt_set_gpio_dir(pin, GPIO_DIR_OUT);\
	mt_set_gpio_out(pin, level);\
} while (0)
/*[i53M][Zihweishen] Modify GPIO setting function end 2016/01/26*/


//<<Mel - 4/10, Modify Touch firmware resolution.
/* Zihwei -> Modify Touch firmware resolution. 2015/03/31*/
//#define ELAN_X_MAX 	 576//960//576  
//#define ELAN_Y_MAX	 1024//1856 //1792//960 
#define ELAN_X_MAX 	 1088
#define ELAN_Y_MAX	 1984
/* Zihwei <- Modify Touch firmware resolution. 2015/03/31*/
//>>Mel - 4/10, Modify Touch firmware resolution.


//#define LCM_X_MAX	simple_strtoul(LCM_WIDTH, NULL, 0)//896
//#define LCM_Y_MAX	simple_strtoul(LCM_HEIGHT, NULL, 0)//1728
#define LCM_X_MAX	1080//896//simple_strtoul(896, NULL, 0)//896
#define LCM_Y_MAX	1920//1728//simple_strtoul(1728, NULL, 0)//1728

#define ELAN_KEY_BACK	0x81 // Elan Key's define
#define ELAN_KEY_HOME	0x41
#define ELAN_KEY_MENU	0x21
//#define ELAN_KEY_SEARCH	0x11


#ifndef _LINUX_ELAN_KTF2K_H
#define _LINUX_ELAN_KTF2K_H

#define ELAN_KTF2K_NAME "elan-ktf2k"

struct elan_ktf2k_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int (*power)(int on);
};

//softkey is reported as AXIS
//#define SOFTKEY_AXIS_VER

//Orig. point at upper-right, reverse it.
//#define REVERSE_X_AXIS
struct osd_offset{
	int left_x;
	int right_x;
	unsigned int key_event;
};

//Elan add for OSD bar coordinate
#if defined( SOFTKEY_AXIS_VER )
static struct osd_offset OSD_mapping[] = {
  {35, 99, KEY_MENU},	//menu_left_x, menu_right_x, KEY_MENU
  {203, 267, KEY_HOME},	//home_left_x, home_right_x, KEY_HOME
  {373, 437, KEY_BACK},	//back_left_x, back_right_x, KEY_BACK
  {541, 605, KEY_SEARCH},	//search_left_x, search_right_x, KEY_SEARCH
};

static int key_pressed = -1;
#endif /* SOFTKEY_AXIS_VER */

#endif /* _LINUX_ELAN_KTF2K_H */
