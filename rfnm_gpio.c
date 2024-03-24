
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_gpt.h"
#include "fsl_gpio.h"

#include "fsl_common.h"
#include "rfnm_m7.h"

#include "/home/davide/imx-rfnm-bsp/build/tmp/work-shared/imx8mp-rfnm/kernel-source/include/linux/rfnm-gpio.h"


volatile unsigned int *gpio[7];

struct rfnm_gpio_map_bit {
	uint8_t bank[2];
	uint8_t num[2];
};

void rfnm_gpio_clear(uint8_t dgb_id, uint32_t gpio_map_id) {

	int bank, num;

	if(dgb_id == 0) {
		bank = (gpio_map_id >> R_DBG_S_PRI_BANK) & 0xff;
		num = (gpio_map_id >> R_DBG_S_PRI_NUM) & 0xff;
	} else {
		bank = (gpio_map_id >> R_DBG_S_SEC_BANK) & 0xff;
		num = (gpio_map_id >> R_DBG_S_SEC_NUM) & 0xff;
	}

	if(bank == 6) {
		// la9310
		*(gpio[bank] + 2) &= ~(1 << num);
 	} else {
		// imx
		*gpio[bank] &= ~(1 << num);
	}
}

void rfnm_gpio_set(uint8_t dgb_id, uint32_t gpio_map_id) {

	int bank, num;

	if(dgb_id == 0) {
		bank = (gpio_map_id >> R_DBG_S_PRI_BANK) & 0xff;
		num = (gpio_map_id >> R_DBG_S_PRI_NUM) & 0xff;
	} else {
		bank = (gpio_map_id >> R_DBG_S_SEC_BANK) & 0xff;
		num = (gpio_map_id >> R_DBG_S_SEC_NUM) & 0xff;
	}

	if(bank == 6) {
		// la9310
		*(gpio[bank] + 2) |= (1 << num);
 	} else {
		// imx
		*gpio[bank] |= (1 << num);
	}
}

void rfnm_gpio_output(uint8_t dgb_id, uint32_t gpio_map_id) {

	int bank, num;

	if(dgb_id == 0) {
		bank = (gpio_map_id >> R_DBG_S_PRI_BANK) & 0xff;
		num = (gpio_map_id >> R_DBG_S_PRI_NUM) & 0xff;
	} else {
		bank = (gpio_map_id >> R_DBG_S_SEC_BANK) & 0xff;
		num = (gpio_map_id >> R_DBG_S_SEC_NUM) & 0xff;
	}

	if(bank == 6) {
		// la9310
		*(gpio[bank]) |= (1 << num);
 	} else {
		// imx
		*(gpio[bank] + 0x1) |= (1 << num);
	}
}

void rfnm_gpio_init(void) {

	// imx
	gpio[1] = (volatile unsigned int *) GPIO1_BASE;
	gpio[2] = (volatile unsigned int *) GPIO2_BASE;
	gpio[3] = (volatile unsigned int *) GPIO3_BASE;
	gpio[4] = (volatile unsigned int *) GPIO4_BASE;
	gpio[5] = (volatile unsigned int *) GPIO5_BASE;

	// la9310
	gpio[6] = (volatile unsigned int *) PCIE_GPIO_ADDR_FROM_M7;

}