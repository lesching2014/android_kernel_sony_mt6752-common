#include <linux/kernel.h>
#include <linux/string.h>

#include <mach/mtk_rtc.h>
#include <mach/wd_api.h>

#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <mt6752/include/mach/mt_rtc_hw.h>
#include <mt6752/include/mach/mtk_rtc_hal.h>

extern void wdt_arch_reset(char);
extern u16 rtc_read(u16 addr);
extern void rtc_write(u16 addr, u16 data);
extern void rtc_write_trigger(void);

//<20160229-bacalli, DMS06688883 [Ar]ST1-GST-<5916_Others>-<The phone will hang more than one minute when press the power key 30 seconds.>
static uint32_t *warmboot_addr_p;
//>20160229-bacalli

//<20160519-ericlin, support oemS features.
#define S1_WARMBOOT_MAGIC_VAL (0xBEEF)
#define S1_WARMBOOT_NORMAL    (0x7651)
#define S1_WARMBOOT_S1        (0x6F53)
#define S1_WARMBOOT_FB        (0x7700)
#define S1_WARMBOOT_NONE      (0x0000)
#define S1_WARMBOOT_CLEAR     (0xABAD)
#define S1_WARMBOOT_TOOL      (0x7001)
#define S1_WARMBOOT_RECOVERY  (0x7711)
#define S1_WARMBOOT_FOTA      (0x6F46)

//>20160519-ericlin
//Bacal, 20160229, reboot for oemF start
static void *remap_lowmem(phys_addr_t start, phys_addr_t size)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	prot = pgprot_noncached(PAGE_KERNEL);

	pages = kmalloc(sizeof(struct page *) * page_count, GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n", __func__, page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vmap(pages, page_count, VM_MAP, prot);
	kfree(pages);
	if (!vaddr) {
		pr_err("%s: Failed to map %u pages\n", __func__, page_count);
		return NULL;
	}

	return vaddr + offset_in_page(start);
}
//Bacal, 20160229, reboot for oemF end	

//<20160229-bacalli, DMS06688883 [Ar]ST1-GST-<5916_Others>-<The phone will hang more than one minute when press the power key 30 seconds.>
static int __init warmboot_addr_console_early_init(void){
    warmboot_addr_p = remap_lowmem(0x1020209C, sizeof(*warmboot_addr_p));
}
console_initcall(warmboot_addr_console_early_init);
//>20160229-bacalli


void arch_reset(char mode, const char *cmd)
{
	char reboot = 0;
	int res = 0;
	struct wd_api *wd_api = NULL;
#ifdef CONFIG_FPGA_EARLY_PORTING
	return;
#else

	res = get_wd_api(&wd_api);
	pr_warn("arch_reset: cmd = %s\n", cmd ? : "NULL");

	if (cmd && !strcmp(cmd, "charger")) {
		/* do nothing */
	} else if (cmd && !strcmp(cmd, "recovery")) {
 #ifndef CONFIG_MTK_FPGA
		rtc_mark_recovery();
 #endif
	} else if (cmd && !strcmp(cmd, "bootloader")) {
 #ifndef CONFIG_MTK_FPGA
		rtc_mark_fast();
 #endif
	}
#ifdef CONFIG_MTK_KERNEL_POWER_OFF_CHARGING
	else if (cmd && !strcmp(cmd, "kpoc")){
		rtc_mark_kpoc();
    }
#endif
    //Bacal, 20160229, reboot for oemF start
    else if (cmd && !strcmp(cmd, "oemF")){
        // Bacal, 20160229, DMS06417163 OMVFOTA-The MUT could not reboot in the process of upgrading for Cosmos brown. start
        u16 temp_warmboot_addr_p;
        temp_warmboot_addr_p=rtc_read(RTC_AL_DOM);
        pr_warn("[RTC Register] rtc_read(RTC_AL_DOM)(original) = %xh\n", temp_warmboot_addr_p);
        temp_warmboot_addr_p = 0x1111;
        rtc_write(RTC_AL_DOM, temp_warmboot_addr_p);
        rtc_write_trigger();
		pr_warn("[RTC Register] rtc_write(RTC_AL_DOM, %xh)\n", temp_warmboot_addr_p);
		pr_warn("[RTC Register] rtc_read(RTC_AL_DOM) = %xh\n", rtc_read(RTC_AL_DOM)); 
		//Bacal, 20160229, DMS06417163 end.
	     
        pr_warn(" [BACAL] arch_reset: Warmboot reasen= %s\n", cmd);
        if(warmboot_addr_p){
		    *warmboot_addr_p=0x6f46beef;
        }
		//<20160519-ericlin, support oemS features.        
	  }else if( (cmd && !strcmp(cmd, "oemS")) || (cmd && !strcmp(cmd, "oem-53")) ) {

		    //write_magic(S1_WARMBOOT_MAGIC_VAL | (S1_WARMBOOT_S1 << 16), 0);
        if(warmboot_addr_p){
		    	*warmboot_addr_p=S1_WARMBOOT_MAGIC_VAL | (S1_WARMBOOT_S1 << 16);    
		    }   
		//>20160519-ericlin		    	
    }else {
        pr_warn(" [BACAL] arch_reset: Warmboot reasen= %s\n", cmd);
        if(warmboot_addr_p){
            *warmboot_addr_p=0x7651beef;
        }
    //Bacal, 20160229, reboot for oemF end
        reboot = 1;
    }
	if (res)
		pr_warn("arch_reset, get wd api error %d\n", res);
	else
		wd_api->wd_sw_reset(reboot);
 #endif
}
