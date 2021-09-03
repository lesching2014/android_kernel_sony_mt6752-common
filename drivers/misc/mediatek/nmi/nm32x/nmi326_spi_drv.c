/*****************************************************************************
 Copyright(c) 2012 NMI Inc. All Rights Reserved

 File name : nmi326_spi_drv.c

 Description : NM326 SPI interface

 History :
 ----------------------------------------------------------------------
 2012/11/27 	ssw		initial
*******************************************************************************/

#include <linux/version.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <asm/io.h>
#include <asm/page.h>
#include <linux/irq.h>
#include <asm/irq.h>
//#include <mach/gpio.h>
//#include <asm/mach/map.h>	//L remake
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/poll.h>
//#include <plat/gpio.h>
//#include <plat/mux.h>

#include "nmi326_spi_drv.h"
#include "nmi326.h"

// Thunder added for Power saving. 20150331
extern void isdbt_gpio_power_off(void);

static struct spi_device *nmi326_spi;

/*This should be done in platform*/

unsigned char tmp_Arry[8192] = {0};

int nmi326_spi_read(u8 *buf, size_t len)
{

	struct spi_message msg;
	struct spi_transfer	transfer[2];
	unsigned char status = 0;
	int r_len;
	//unsigned char temp_buf[256] = {0};
	//unsigned char txArry[32] = {0};

	memset(&msg, 0, sizeof(msg));
	memset(transfer, 0, sizeof(transfer));

	spi_message_init(&msg);
	msg.spi = nmi326_spi;
#if 0
	transfer[0].tx_buf = (unsigned char *)NULL;
	transfer[0].rx_buf = (unsigned char *)buf;
	transfer[0].len = len;
	transfer[0].bits_per_word = 8;
	transfer[0].delay_usecs = 0;
#else
	//transfer[1].tx_buf = txArry;
	transfer[1].tx_buf = tmp_Arry;	
	transfer[1].rx_buf = (unsigned char *)buf;
	transfer[1].len = len;
	transfer[1].bits_per_word = 8;
	transfer[1].delay_usecs = 0;
#endif

	spi_message_add_tail(&(transfer[1]), &msg);
	status = spi_sync(nmi326_spi, &msg);

	if (status==0)
	{
		//r_len = msg.actual_length;
		r_len = len;
	}
	else
	{
		r_len =status;
	}

	return r_len;
}

int nmi326_spi_write(u8 *buf, size_t len)
{

	struct spi_message msg;
	struct spi_transfer	transfer[2];
	unsigned char status = 0;
	int w_len;
	//unsigned char rxArry[32] = {0};

	memset(&msg, 0, sizeof(msg));
	memset(transfer, 0, sizeof(transfer));

	spi_message_init(&msg);
	msg.spi = nmi326_spi;
#if 0
	transfer[0].tx_buf = (unsigned char *)buf;
	transfer[0].rx_buf = (unsigned char *)NULL;
	transfer[0].len = len;
	transfer[0].bits_per_word = 8;
	transfer[0].delay_usecs = 0;
#else
	transfer[0].tx_buf = (unsigned char *)buf;
	//transfer[0].rx_buf = (unsigned char *)NULL;
	//transfer[0].rx_buf = rxArry;
	transfer[0].rx_buf = tmp_Arry;
	transfer[0].len = len;
	transfer[0].bits_per_word = 8;
	transfer[0].delay_usecs = 0;
#endif

	spi_message_add_tail(&(transfer[0]), &msg);
	status = spi_sync(nmi326_spi, &msg);

	if (status==0)
		w_len = len;
	else
		w_len =status;

	return w_len;
}

void nmi326_spi_read_chip_id(void)
{
	unsigned char b[20];
	unsigned long adr = 0x6400;
	int retry;
	size_t len;
	unsigned char sta = 0;
	//unsigned char sta[4];
	unsigned long val = 0;
	int ret_size;

  nmi326_spi_set_chip_id(0x00);
	b[0] = 0x70;								/* word access */
	b[1] = 0x00;
	b[2] = 0x04;
	b[3] = (uint8_t)(adr >> 16);
	b[4] = (uint8_t)(adr >> 8);
	b[5] = (uint8_t)(adr);
	len = 6;

	//chip.inp.hlp.write(0x61, b, len);
	ret_size = nmi326_spi_write(b,len);
	spi_dbg("chip id ret_size 1= %d\n",ret_size);
	/**
		Wait for complete
	**/
	retry = 10;

	do {
		//chip.inp.hlp.read(0x61, &sta, 1);
		spi_dbg("chip id retry = %d\n",retry);

		ret_size = nmi326_spi_read(&sta,1);
		//ret_size = nmi326_spi_read(&sta,4);
		if (sta == 0xff)
			break;
		else
			spi_dbg("\n Thunder sta = %d ,ret_size= %d\n",sta,ret_size);
			//spi_dbg("\n Thunder sta = %d %d %d %d ,ret_size= %d\n",(int)sta[0],(int)sta[1],(int)sta[2],(int)sta[3],ret_size);

		mdelay(3);
	}while (retry--);

	spi_dbg("\nchip id ret_size 2= %d\n",ret_size);
	spi_dbg("chip id sta = %d\n",sta);
	//spi_dbg("chip id sta = %d\n",(int)sta[0]);

	if (sta== 0xff) {
		/**
			Read the Count
		**/
		//chip.inp.hlp.read(0x61, b, 3);
		nmi326_spi_read(b,3);
		len = b[2] | (b[1] << 8) | (b[0] << 16);
		if (len == 4) {
			//chip.inp.hlp.read(0x61, (uint8_t *)&val, 4);
			nmi326_spi_read((unsigned char*)&val,4);
			b[0] = 0xff;
			//chip.inp.hlp.write(0x61, b, 1);
			nmi326_spi_write(b,1);

      nmi326_spi_set_chip_id(val);
			//spi_dbg("===============================\n",val);
			spi_dbg("===============================\n");
			spi_dbg("NMI326 Chip ID = [0x%x] on SPI\n",(int) val);
			spi_dbg("NMI326 Chip ID on SPI\n");
			spi_dbg("===============================\n");
		} else {
			//spi_dbg("Error, SPI bus, bad count (%d)\n", len);
			spi_dbg("Error, SPI bus, bad count \n");
		}
	}
 	else {
		spi_dbg("Error, SPI bus, not complete\n");
	}
}
#define NMI_DTV_CHIP_ID_1    0x10325f0

unsigned long g_nmi_dtv_chip_id = 0;


void nmi326_spi_set_chip_id(unsigned long chip_id)
{
  g_nmi_dtv_chip_id = chip_id;
}

int nmi326_spi_get_chip_id(unsigned long *chip_id)
{
#if 1	// Thunder added for DTV meta tools error. 20150106
	if (g_nmi_dtv_chip_id != NMI_DTV_CHIP_ID_1)
	{
		nmi326_spi_read_chip_id();
	}
#endif

	*chip_id = g_nmi_dtv_chip_id;

  	return (g_nmi_dtv_chip_id == NMI_DTV_CHIP_ID_1);
}

static int  nmi326_spi_probe(struct spi_device *spi)
{
	int retval = 0;

	spi_dbg(" **** nmi326_spi_probe.\n");
	nmi326_spi = spi;
	nmi326_spi->mode = (SPI_MODE_0) ;
	nmi326_spi->bits_per_word = 8 ;

	retval = spi_setup( nmi326_spi );
	if( retval != 0 ) {
		spi_dbg( "ERROR : %d\n", retval );
	}
	else {
		spi_dbg( "Done : %d\n", retval );
	}

	nmi326_spi_read_chip_id();
	
	// Thunder added for Power saving. 20150331
	mdelay(3);
	isdbt_gpio_power_off();
	
	return 0;
}

static int __exit nmi326_spi_remove(struct spi_device *spi)
{
	return 0;
}

static struct spi_driver nmi326_spi_driver = {
	.driver = {
		.name	= "nmispi",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= nmi326_spi_probe,
	.remove		= __exit_p(nmi326_spi_remove),
};


static const char banner[] __initdata =  "NMI326 SPI Driver Version 1.0\n";

#if 1
static struct spi_board_info spi_board_info[] = {
    {
        .modalias = "nmispi",
        .platform_data = NULL,
        .bus_num = 0,
        .max_speed_hz	= 5*1000*1000,
        .chip_select = 0,
        //.mode = SPI_MODE_0,
        .mode = SPI_MODE_1,
        //.controller_data = (void*)NULL,
    },
};
#endif

int nmi326_spi_init( void )
{
	int retval = 0;

	spi_dbg("%s",banner);
	spi_register_board_info(spi_board_info,ARRAY_SIZE(spi_board_info));
	retval = spi_register_driver( &nmi326_spi_driver );
	if( retval < 0 ) {
		spi_dbg( "spi_register_driver ERROR : %d\n", retval );

		goto exit;
	}

	spi_dbg( "(%d) init Done.\n",  __LINE__ );

	return 0;

exit :
	return retval;
}

void nmi326_spi_exit( void )
{
	printk( "[%s]\n", __func__ );

	spi_unregister_driver( &nmi326_spi_driver );
}

//module_init(nmi326_spi_init);
//module_exit(nmi326_spi_exit);



