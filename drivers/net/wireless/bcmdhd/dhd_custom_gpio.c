/*
 * Customer code to add GPIO control during WLAN start/stop
 *
 * Copyright (C) 1999-2016, Broadcom Corporation
 * 
 *      Unless you and Broadcom execute a separate written software license
 * agreement governing use of this software, this software is licensed to you
 * under the terms of the GNU General Public License version 2 (the "GPL"),
 * available at http://www.broadcom.com/licenses/GPLv2.php, with the
 * following added to such license:
 * 
 *      As a special exception, the copyright holders of this software give you
 * permission to link this software with independent modules, and to copy and
 * distribute the resulting executable under terms of your choice, provided that
 * you also meet, for each linked independent module, the terms and conditions of
 * the license of that module.  An independent module is a module which is not
 * derived from this software.  The special exception does not apply to any
 * modifications of the software.
 * 
 *      Notwithstanding the above, under no circumstances may you combine this
 * software in any way with any other Broadcom software provided under a license
 * other than the GPL, without Broadcom's express prior written consent.
 *
 *
 * <<Broadcom-WL-IPTag/Open:>>
 *
 * $Id: dhd_custom_gpio.c 591129 2015-10-07 05:22:14Z $
 */

#include <typedefs.h>
#include <linuxver.h>
#include <osl.h>
#include <bcmutils.h>
#include <dngl_stats.h>
#include <dhd.h>
#include <dhd_linux.h>

#include <wlioctl.h>

#define WL_ERROR(x) printf x
#define WL_TRACE(x)

#if defined(OOB_INTR_ONLY)

#if defined(BCMLXSDMMC)
extern int sdioh_mmc_irq(int irq);
#endif /* (BCMLXSDMMC)  */

/* Customer specific Host GPIO defintion  */
static int dhd_oob_gpio_num = -1;

module_param(dhd_oob_gpio_num, int, 0644);
MODULE_PARM_DESC(dhd_oob_gpio_num, "DHD oob gpio number");

/* This function will return:
 *  1) return :  Host gpio interrupt number per customer platform
 *  2) irq_flags_ptr : Type of Host interrupt as Level or Edge
 *
 *  NOTE :
 *  Customer should check his platform definitions
 *  and his Host Interrupt spec
 *  to figure out the proper setting for his platform.
 *  Broadcom provides just reference settings as example.
 *
 */
int dhd_customer_oob_irq_map(void *adapter, unsigned long *irq_flags_ptr)
{
	int  host_oob_irq = 0;

#if defined(CUSTOMER_HW2)
	host_oob_irq = wifi_platform_get_irq_number(adapter, irq_flags_ptr);

#else
#if defined(CUSTOM_OOB_GPIO_NUM)
	if (dhd_oob_gpio_num < 0) {
		dhd_oob_gpio_num = CUSTOM_OOB_GPIO_NUM;
	}
#endif /* CUSTOMER_OOB_GPIO_NUM */

	if (dhd_oob_gpio_num < 0) {
		WL_ERROR(("%s: ERROR customer specific Host GPIO is NOT defined \n",
		__FUNCTION__));
		return (dhd_oob_gpio_num);
	}

	WL_ERROR(("%s: customer specific Host GPIO number is (%d)\n",
	         __FUNCTION__, dhd_oob_gpio_num));

#endif 

	return (host_oob_irq);
}
#endif 

/* Customer function to control hw specific wlan gpios */
int
dhd_customer_gpio_wlan_ctrl(void *adapter, int onoff)
{
	int err = 0;

	return err;
}

#ifdef GET_CUSTOM_MAC_ENABLE
#define MAC_ADDRESS_LEN 12

int wifi_get_mac_addr_qcom(unsigned char *buf){
	int ret = 0;
	int i;
	struct file *fp = NULL;
	unsigned char c_mac[MAC_ADDRESS_LEN];
	char fname[]="/factory/wifimac.txt";
    int count = 0;

	WL_ERROR(("%s Enter\n", __FUNCTION__));

	while ((fp = dhd_os_open_image(fname)) == NULL && count <5){
        WL_ERROR(("%s: unable to open %s, try %d time(s), at most 5 times\n",__FUNCTION__, fname, count+1));
        msleep(500);
        count++;
    }
    if (fp == NULL){
        WL_ERROR(("%s: still unable to open %s\n",__FUNCTION__, fname));
        return 1;
    }

	if ( dhd_os_get_image_block(c_mac, MAC_ADDRESS_LEN, fp) != MAC_ADDRESS_LEN ){
		WL_ERROR(("%s: Error on reading mac address from %s \n",__FUNCTION__, fname));
		dhd_os_close_image(fp);
		return 1;
	}
	dhd_os_close_image(fp);

	for (i =0; i< MAC_ADDRESS_LEN ; i+=2){
		c_mac[i] = bcm_isdigit(c_mac[i]) ? c_mac[i]-'0' : bcm_toupper(c_mac[i])-'A'+10;
		c_mac[i+1] = bcm_isdigit(c_mac[i+1]) ? c_mac[i+1]-'0' : bcm_toupper(c_mac[i+1])-'A'+10;

		buf[i/2] = c_mac[i]*16 + c_mac[i+1];
	}

	WL_TRACE(("%s: read from file mac address: %x:%x:%x:%x:%x:%x\n",
			 __FUNCTION__, buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]));

	return ret;
}

/* Function to get custom MAC address */
int
dhd_custom_get_mac_address(void *adapter, unsigned char *buf)
{
	int ret = 0;

	WL_TRACE(("%s Enter\n", __FUNCTION__));
	if (!buf)
		return -EINVAL;

	/* Customer access to MAC address stored outside of DHD driver */
#if (defined(CUSTOMER_HW2) || 0) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 35))
    ret = wifi_get_mac_addr_qcom(buf);
#endif

#ifdef EXAMPLE_GET_MAC
	/* EXAMPLE code */
	{
		struct ether_addr ea_example = {{0x00, 0x11, 0x22, 0x33, 0x44, 0xFF}};
		bcopy((char *)&ea_example, buf, sizeof(struct ether_addr));
	}
#endif /* EXAMPLE_GET_MAC */

	return ret;
}
#endif /* GET_CUSTOM_MAC_ENABLE */

struct cntry_locales_custom {
	char iso_abbrev[WLC_CNTRY_BUF_SZ];
	char custom_locale[WLC_CNTRY_BUF_SZ];
	int32 custom_locale_rev;
};

//#if !defined(WL_WIRELESS_EXT)
//struct cntry_locales_custom {
//	char iso_abbrev[WLC_CNTRY_BUF_SZ];	/* ISO 3166-1 country abbreviation */
//	char custom_locale[WLC_CNTRY_BUF_SZ];	/* Custom firmware locale */
//	int32 custom_locale_rev;		/* Custom local revisin default -1 */
//};
//#endif /* WL_WIRELESS_EXT */

/* Customized Locale table : OPTIONAL feature */
const struct cntry_locales_custom translate_custom_table[] = {
/* Table should be filled out based on custom platform regulatory requirement */
#ifdef EXAMPLE_TABLE
	{"",   "XY", 4},  /* Universal if Country code is unknown or empty */
	{"US", "US", 69}, /* input ISO "US" to : US regrev 69 */
	{"CA", "US", 69}, /* input ISO "CA" to : US regrev 69 */
	{"EU", "EU", 5},  /* European union countries to : EU regrev 05 */
	{"AT", "EU", 5},
	{"BE", "EU", 5},
	{"BG", "EU", 5},
	{"CY", "EU", 5},
	{"CZ", "EU", 5},
	{"DK", "EU", 5},
	{"EE", "EU", 5},
	{"FI", "EU", 5},
	{"FR", "EU", 5},
	{"DE", "EU", 5},
	{"GR", "EU", 5},
	{"HU", "EU", 5},
	{"IE", "EU", 5},
	{"IT", "EU", 5},
	{"LV", "EU", 5},
	{"LI", "EU", 5},
	{"LT", "EU", 5},
	{"LU", "EU", 5},
	{"MT", "EU", 5},
	{"NL", "EU", 5},
	{"PL", "EU", 5},
	{"PT", "EU", 5},
	{"RO", "EU", 5},
	{"SK", "EU", 5},
	{"SI", "EU", 5},
	{"ES", "EU", 5},
	{"SE", "EU", 5},
	{"GB", "EU", 5},
	{"KR", "XY", 3},
	{"AU", "XY", 3},
	{"CN", "XY", 3}, /* input ISO "CN" to : XY regrev 03 */
	{"TW", "XY", 3},
	{"AR", "XY", 3},
	{"MX", "XY", 3},
	{"IL", "IL", 0},
	{"CH", "CH", 0},
	{"TR", "TR", 0},
	{"NO", "NO", 0},
#endif /* EXMAPLE_TABLE */
#if defined(CUSTOMER_HW2)
#if defined(BCM4359_CHIP)
	{"",   "XZ", 11},  /* Universal if Country code is unknown or empty, ch 12,13,36,40,ch 52-165 all passive */
	{"AE", "AE", 6},  /* UNITED ARAB EMIRATES : ch 1-13, ch 36-48, ch 52-140 radar passive, ch 144-165 */
	{"BH", "BH", 4},  /* BAHRAIN : ch 1-13, ch 36-64, ch 149-165 */
	{"CN", "CN", 38},  /* CHINA : ch 1-13, ch 36-48, ch 52-64 radar passive, ch 149-165 */
	{"CZ", "CZ", 4},  /* CZECH REPUBLIC : ch 1-13 , ch 36-48, ch 52-140 radar passive*/
	{"DE", "DE", 7},  /* GERMANY : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"DK", "DK", 4},  /* DENMARK : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"ES", "ES", 4},  /* SPAIN : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"FR", "FR", 5},  /* FRANCE : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"FI", "FI", 4},  /* FINLAND : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"GB", "GB", 6},  /* UNITED KINGDOM : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"HK", "HK", 2},  /* HONG KONG : ch 1-13, ch 36-48, ch 52-140 radar passive, ch 144-165 */
	{"HU", "HU", 4},  /* HUNGARY :ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"ID", "ID", 1},  /* INDONESIA : ch 1-13, ch 149-161 */
	{"IL", "IL", 7},  /* ISRAEL : ch 1-13, ch 36-48, ch 52-64 radar passive */
	{"IN", "IN", 3},  /* INDIA : ch 1-13, ch 36-48, ch 52-64 radar passive, ch 149-165*/
	{"IT", "IT", 4},  /* ITALY : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"IS", "IS", 4},  /* ICELAND : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"JP", "JP", 45},  /* JAPAN : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"MY", "MY", 3},  /* MALAYSIA : ch 1-13, ch 36-48, ch 52-64 radar passive, ch 149-165*/
	{"NL", "NL", 4},  /* NETHERLANDS : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"NO", "NO", 4},  /* NORWAY : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"PT", "PT", 4},  /* PORTUGAL : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"RU", "RU", 13},  /* RUSSIAN FEDERATION : ch 1-13, ch 36-48, ch 52-64+132-140 radar passive, ch 149-165 */
	{"SA", "SA", 0},  /* SAUDI ARABIA : ch 1-13, ch 36-48, ch 52-140 radar passive, ch 144-161 */
	{"SE", "SE", 4},  /* SWEDEN : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"SG", "SG", 0},  /* SINGAPORE : ch 1-13, ch 36-48, ch 52-140 radar passive, ch 144-165 */
	{"TH", "TH", 5},  /* THAILAND :  ch 1-13, ch 36-48, ch 52-140 radar passive, ch 144-165 */
	{"TR", "TR", 7},  /* TURKEY : ch 1-13, ch 36-48, ch 52-140 radar passive */
	{"TW", "CA", 2},  /* NCC 2015 open Band 1 temp set to CANADA : ch 1-11, ch 36-48, ch 56-140 radar passive, ch 144-165 active */
	{"UA", "UA", 8},  /* UKRAINE : ch 1-13, ch 36-48, ch 52-132 radar passive, ch 149-165 */
	{"US", "US", 1},  /* UNITED STATES : ch 1-11, ch 36-48, ch 149-165 */
	{"VN", "VN", 4},  /* VIET NAM : ch 1-13, ch 36-48, ch 52-140 radar passive, ch 144-165 */
#endif
#if defined(BCM4335_CHIP)
	{"",   "XZ", 11},  /* Universal if Country code is unknown or empty */
#endif
	{"AE", "AE", 1},
	{"AR", "AR", 1},
	{"AT", "AT", 1},
	{"AU", "AU", 2},
	{"BE", "BE", 1},
	{"BG", "BG", 1},
	{"BN", "BN", 1},
	{"CA", "CA", 2},
	{"CH", "CH", 1},
	{"CY", "CY", 1},
	{"CZ", "CZ", 1},
	{"DE", "DE", 3},
	{"DK", "DK", 1},
	{"EE", "EE", 1},
	{"ES", "ES", 1},
	{"FI", "FI", 1},
	{"FR", "FR", 1},
	{"GB", "GB", 1},
	{"GR", "GR", 1},
	{"HR", "HR", 1},
	{"HU", "HU", 1},
	{"IE", "IE", 1},
	{"IS", "IS", 1},
	{"IT", "IT", 1},
	{"ID", "ID", 1},
	{"JP", "JP", 8},
	{"KR", "KR", 24},
	{"KW", "KW", 1},
	{"LI", "LI", 1},
	{"LT", "LT", 1},
	{"LU", "LU", 1},
	{"LV", "LV", 1},
	{"MA", "MA", 1},
	{"MT", "MT", 1},
	{"MX", "MX", 1},
	{"NL", "NL", 1},
	{"NO", "NO", 1},
	{"PL", "PL", 1},
	{"PT", "PT", 1},
	{"PY", "PY", 1},
	{"RO", "RO", 1},
	{"SE", "SE", 1},
	{"SI", "SI", 1},
	{"SK", "SK", 1},
	{"TR", "TR", 7},
	{"TW", "TW", 1},
	{"IR", "XZ", 11},	/* Universal if Country code is IRAN, (ISLAMIC REPUBLIC OF) */
	{"SD", "XZ", 11},	/* Universal if Country code is SUDAN */
	{"SY", "XZ", 11},	/* Universal if Country code is SYRIAN ARAB REPUBLIC */
	{"GL", "XZ", 11},	/* Universal if Country code is GREENLAND */
	{"PS", "XZ", 11},	/* Universal if Country code is PALESTINIAN TERRITORY, OCCUPIED */
	{"TL", "XZ", 11},	/* Universal if Country code is TIMOR-LESTE (EAST TIMOR) */
	{"MH", "XZ", 11},	/* Universal if Country code is MARSHALL ISLANDS */
#ifdef BCM4330_CHIP
	{"RU", "RU", 1},
	{"US", "US", 5}
#endif
#endif 
};


/* Customized Locale convertor
*  input : ISO 3166-1 country abbreviation
*  output: customized cspec
*/
#ifdef CUSTOM_COUNTRY_CODE
void get_customized_country_code(void *adapter, char *country_iso_code,
  wl_country_t *cspec, u32 flags)
#else
void get_customized_country_code(void *adapter, char *country_iso_code, wl_country_t *cspec)
#endif /* CUSTOM_COUNTRY_CODE */
{
#if defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 39))
	struct cntry_locales_custom *cloc_ptr;

	if (!cspec)
		return;
#ifdef CUSTOM_COUNTRY_CODE
	cloc_ptr = wifi_platform_get_country_code(adapter, country_iso_code,
	           flags);
#else
	cloc_ptr = wifi_platform_get_country_code(adapter, country_iso_code);
#endif /* CUSTOM_COUNTRY_CODE */
	if (cloc_ptr) {
		strlcpy(cspec->ccode, cloc_ptr->custom_locale, WLC_CNTRY_BUF_SZ);
		cspec->rev = cloc_ptr->custom_locale_rev;
	}
	return;
#else
	int size, i;

	size = ARRAYSIZE(translate_custom_table);

	if (cspec == 0)
		 return;

	if (size == 0)
		 return;

	for (i = 0; i < size; i++) {
		if (strcmp(country_iso_code, translate_custom_table[i].iso_abbrev) == 0) {
        WL_ERROR(("[WLDBG] match customized country code table iso = %s\n",translate_custom_table[i].iso_abbrev));
        WL_ERROR(("[WLDBG] set wifi country = %s set rev =%d \n"
                                ,translate_custom_table[i].custom_locale,translate_custom_table[i].custom_locale_rev));
			memcpy(cspec->ccode,
				translate_custom_table[i].custom_locale, WLC_CNTRY_BUF_SZ);
			cspec->rev = translate_custom_table[i].custom_locale_rev;
			return;
		}
	}
// #ifdef EXAMPLE_TABLE
	/* if no country code matched return first universal code from translate_custom_table */
	memcpy(cspec->ccode, translate_custom_table[0].custom_locale, WLC_CNTRY_BUF_SZ);
	cspec->rev = translate_custom_table[0].custom_locale_rev;
// #endif /* EXMAPLE_TABLE */
	return;
#endif /* defined(CUSTOMER_HW2) && (LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 36)) */
}
