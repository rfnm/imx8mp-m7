#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_gpt.h"
#include "fsl_gpio.h"

#include "fsl_common.h"

#include "rfnm_m7.h"
#include "/home/davide/imx-rfnm-bsp/build/tmp/work-shared/imx8mp-rfnm/kernel-source/include/linux/rfnm-gpio.h"



int rfnm_fe_generic_init(struct rfnm_m7_dgb * dgb_dt) {

	int i;

    rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_LA_FE_CLK);
	rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_GPIO2_2); 
	rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_GPIO2_3);


#if 1
	rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_GPIO4_22);
	rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_GPIO4_23);
	rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_GPIO4_31);

	rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_GPIO4_9);

	rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_GPIO4_0);
	rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_GPIO4_1);
	rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_GPIO4_2);
	rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_GPIO4_4);
	rfnm_gpio_output(dgb_dt->dgb_id, RFNM_DGB_GPIO4_5);

#endif 
#if 1

	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_22);
	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_23);
	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_31);

	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_9);

	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_0);
	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_1);
	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_2);
	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_4);
	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO4_5);

#endif

	rfnm_gpio_set(dgb_dt->dgb_id, RFNM_DGB_GPIO2_2); // ff mr
	rfnm_gpio_clear(dgb_dt->dgb_id, RFNM_DGB_GPIO2_3); //ff oe

	memset(&dgb_dt->fe.latch_val_last_written, 0xff, sizeof(dgb_dt->fe.latch_val_last_written));

	// trigger all latches to set initial status
	for(i = 1; i < 7; i++) {
		rfnm_fe_manual_clock(dgb_dt->dgb_id, i);
	}

	return 0;
}

// phy demux 1 gpio4 22
// phy demux 2 gpio4 23
// phy demux 3 gpio4 31

// gate rba gpio4 28

extern volatile unsigned int *gpio[7];

#define GOFAST // gofast is only hardcoded to work on the first daughterboard

void rfnm_fe_manual_clock(int dgb_id, int id) {

#ifdef GOFAST
    uint32_t cur_gpio4 = *gpio[4];
    uint32_t cur_gpio_la = *(gpio[6] + 2);

    cur_gpio_la = *(gpio[6] + 2);
    cur_gpio_la |= (1 << 24);
    *(gpio[6] + 2) = cur_gpio_la;
#else
    // enable latch before changing output    
	rfnm_gpio_set(dgb_id, RFNM_DGB_LA_FE_CLK);
#endif

    // change output
#ifdef GOFAST
    switch(id) {
	case 1:
		cur_gpio4 &= ~(1 << 22);
		cur_gpio4 &= ~(1 << 23);
		cur_gpio4 &= ~(1 << 31);
		break;
	case 2:
		cur_gpio4 |= (1 << 22);
		cur_gpio4 &= ~(1 << 23);
		cur_gpio4 &= ~(1 << 31);
		break;
	case 3:
		cur_gpio4 &= ~(1 << 22);
		cur_gpio4 |= (1 << 23);
		cur_gpio4 &= ~(1 << 31);
		break;
	case 4:
		cur_gpio4 |= (1 << 22);
		cur_gpio4 |= (1 << 23);
		cur_gpio4 &= ~(1 << 31);
		break;
	case 5:
		cur_gpio4 &= ~(1 << 22);
		cur_gpio4 &= ~(1 << 23);
		cur_gpio4 |= (1 << 31);
		break;
	case 6:
		cur_gpio4 |= (1 << 22);
		cur_gpio4 &= ~(1 << 23);
		cur_gpio4 |= (1 << 31);
		break;
    case 7:
		cur_gpio4 |= (1 << 22);
        cur_gpio4 |= (1 << 23);
        cur_gpio4 |= (1 << 31);
		break;
	}
    *gpio[4] = cur_gpio4;
#else
    switch(id) {
	case 1:
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_22);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_23);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_31);
		break;
	case 2:
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_22);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_23);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_31);
		break;
	case 3:
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_22);
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_23);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_31);
		break;
	case 4:
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_22);
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_23);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_31);
		break;
	case 5:
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_22);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_23);
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_31);
		break;
	case 6:
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_22);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_23);
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_31);
		break;
    case 7:
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_22);
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_23);
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_31);
		break;
	}
#endif



	// latch read-in
#ifdef GOFAST    
    cur_gpio_la &= ~(1 << 24);
    *(gpio[6] + 2) = cur_gpio_la;
	cur_gpio_la |= (1 << 24);
    *(gpio[6] + 2) = cur_gpio_la;
#else
    rfnm_gpio_clear(dgb_id, RFNM_DGB_LA_FE_CLK);
	rfnm_gpio_set(dgb_id, RFNM_DGB_LA_FE_CLK);
#endif    
}

void rfnm_fe_load_data_bit(int dgb_id, int latch, int bit) {
	if(bit) {
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_9);
	} else {
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_9);
	}

	int latch_datain_clock_gpio;

	switch(latch) {
	case 1:
		//latch_datain_clock_gpio = 0;
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_0);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_0);
		break;
	case 2:
		//latch_datain_clock_gpio = 1;
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_1);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_1);
		break;
	case 3:
		//latch_datain_clock_gpio = 2;
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_2);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_2);
		break;
	case 4:
		//latch_datain_clock_gpio = 4;
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_4);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_4);
		break;
	case 5:
		//latch_datain_clock_gpio = 5;
		rfnm_gpio_set(dgb_id, RFNM_DGB_GPIO4_5);
		rfnm_gpio_clear(dgb_id, RFNM_DGB_GPIO4_5);
		break;
	}

	//cur_gpio4 |= (1 << latch_datain_clock_gpio);
	//cur_gpio4 &= ~(1 << latch_datain_clock_gpio);

	

}

void rfnm_fe_load_latch(struct rfnm_m7_dgb * dgb_dt, int id, int txrx) {

	uint32_t lval = dgb_dt->fe_tdd[txrx].latch_val[id - 1];

	if(lval == dgb_dt->fe.latch_val_last_written[id - 1]) {
		return;
	}

    //printk("loading latch %d, %x\n", id, lval);

	//printk("bits ");
	int l, q, bit;
	for(l = dgb_dt->fe_tdd[txrx].num_latches[id] - 1; l >= 0; l--) {
		for(q = 7; q >= 0; q--) {
			bit = (lval & (1 << ((8 * l) + q)) ) >> ((8 * l) + q);
			rfnm_fe_load_data_bit(dgb_dt->dgb_id, id, bit);
			//printk("%d ", bit);
		}
	}

	//printk("\n");

	//rfnm_fe_manual_clock(id);
}


void rfnm_fe_load_latches(struct rfnm_m7_dgb * dgb_dt, int txrx) {
	int i;
	for(i = 1; i < 7; i++) {		
		rfnm_fe_load_latch(dgb_dt, i, txrx); 
	}
}

void rfnm_fe_trigger_latches(struct rfnm_m7_dgb * dgb_dt, int txrx) {
	int i = 0, clear_with_last_latch = 0;

    if(dgb_dt->fe_tdd[txrx].load_order[0]) {
        while(dgb_dt->fe_tdd[txrx].load_order[i]) {
            volatile int lid = dgb_dt->fe_tdd[txrx].load_order[i] - 1;
            if(dgb_dt->fe.latch_val_last_written[lid] != dgb_dt->fe_tdd[txrx].latch_val[lid]) {
                rfnm_fe_manual_clock(dgb_dt->dgb_id, lid + 1);
                dgb_dt->fe.latch_val_last_written[lid] = dgb_dt->fe_tdd[txrx].latch_val[lid];
                clear_with_last_latch = 1;
            }
            i++;
        }
    } else {
        for(i = 1; i < 7; i++) {
            if(dgb_dt->fe.latch_val_last_written[i - 1] != dgb_dt->fe_tdd[txrx].latch_val[i - 1]) {
                rfnm_fe_manual_clock(dgb_dt->dgb_id, i);
                dgb_dt->fe.latch_val_last_written[i - 1] = dgb_dt->fe_tdd[txrx].latch_val[i - 1];
                clear_with_last_latch = 1;
            }
        }
    }

    if(clear_with_last_latch) {
        rfnm_fe_manual_clock(dgb_dt->dgb_id, 7);
    }

	
}