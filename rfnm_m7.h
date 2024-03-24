#ifndef __H_RFNM_M7
#define __H_RFNM_M7

#define PCIE_CTRL_ADDR_OFFSET (0x1C00F740 - 0x18000000)
#define PCIE_CTRL_ADDR_FROM_M7 (0xD0000000 + PCIE_CTRL_ADDR_OFFSET) // D400 F740
#define PCIE_GPIO_ADDR_FROM_M7 (0xD0000000 + 0x2300000)

#define INTERRUPT_LATENCY_OFFSET (5.5 * 3.84)


typedef struct {
	uint32_t target_phytimer_ts;
	uint32_t issued_phytimer_ts;
	// GPT3 is used as TTI trigger inside the i.MX. To enable it, set 
	// tti_period_ts > 0. TTI will then trigger at the same time as
	// target_phytimer_ts. tti_period_ts > 0 should only be sent once,
	// unless you need to change the period, send tti_period_ts = 0
	// to disable the interrupt, set tti_period_ts = 1
	uint32_t tti_period_ts;
	uint32_t mode;
} rf_ctrl_s;

#define RFNM_PACKED_STRUCT( __Declaration__ ) __Declaration__ __attribute__((__packed__))


RFNM_PACKED_STRUCT(
	struct fe_s {
		uint32_t latch_val[6];
		uint32_t latch_val_last_written[6];
		uint32_t num_latches[7];
		uint32_t align[1];
		uint32_t load_order[8];
	};
);


RFNM_PACKED_STRUCT(
	struct rfnm_m7_dgb {
        struct fe_s fe;
		struct fe_s fe_tdd[2];
		uint32_t m7_tdd_initialized;
        uint32_t dgb_id;
        uint32_t tdd_available;
	} 
); 


void rfnm_fe_load_latches(struct rfnm_m7_dgb * dgb_dt, int txrx);
void rfnm_fe_trigger_latches(struct rfnm_m7_dgb * dgb_dt, int txrx);
void rfnm_fe_manual_clock(int dgb_id, int id);
int rfnm_fe_generic_init(struct rfnm_m7_dgb * dgb_dt);

#endif