#ifndef __H_RFNM_M7
#define __H_RFNM_M7

#define PCIE_CTRL_ADDR_OFFSET (0x1C00F740 - 0x18000000)
#define PCIE_CTRL_ADDR_FROM_M7 (0xD0000000 + PCIE_CTRL_ADDR_OFFSET) // D400 F740
#define PCIE_GPIO_ADDR_FROM_M7 (0xD0000000 + 0x2300000)
#define VSPA_MEM_ADDR_OFFSET (0x1F400000 - 0x18000000)
#define VSPA_MEM_ADDR_FROM_M7 (0xD0000000 + VSPA_MEM_ADDR_OFFSET)

#define INTERRUPT_LATENCY_OFFSET (5.5 * 3.84)


typedef uint32_t vspa_complex_fixed16;
#define FFT_SIZE 512
#define DMA_RX_size		(256)

#define RFNM_RX_BUF_CNT 15
#define ERROR_MAX 0x9

#define RFNM_ADC_BUFCNT (4096*4) // 4096 ~= 10ms


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

struct rfnm_bufdesc_tx {
	vspa_complex_fixed16 buf[FFT_SIZE];
	uint32_t dac_id;
	uint32_t phytimer;
	uint32_t cc;
	uint32_t axiq_done;
	uint32_t iqcomp_done;
	// (64 - (4 * 5)) / 4 = 22
	uint32_t pad_to_64[11];
};

struct rfnm_bufdesc_rx {
	vspa_complex_fixed16 buf[DMA_RX_size];
	uint32_t adc_id;
	uint32_t phytimer;
	uint32_t cc;
	uint32_t axiq_done;
	uint32_t iqcomp_done;
	uint32_t read;
	// (64 - (4 * 5)) / 4 = 22
	uint32_t pad_to_64[10];
};

struct rfnm_la9310_status {
	uint16_t rx_buf[16]; //use 8 to keep align, rather than RFNM_RX_BUF_CNT
	uint32_t age;
	uint32_t tx_buf_id;
	uint32_t rx_buf_id;
	uint32_t g_errors[ERROR_MAX];
    uint32_t pad_to_64[4];
    uint32_t pad_again[8]; //<-- only for M7
};

RFNM_PACKED_STRUCT(
	struct rfnm_m7_status {
		uint32_t tx_buf_id;
		uint32_t rx_head;
	} 
); 
/*
RFNM_PACKED_STRUCT(
    struct rfnm_rx_usb_head {
        uint32_t magic;
		uint32_t phytimer;
		uint32_t dropped;
		uint32_t adc_cc;
		uint32_t rx_cc;
		uint32_t adc_id;
    }
);
*/


void rfnm_fe_load_latches(struct rfnm_m7_dgb * dgb_dt, int txrx);
void rfnm_fe_trigger_latches(struct rfnm_m7_dgb * dgb_dt, int txrx);
void rfnm_fe_manual_clock(int dgb_id, int id);
int rfnm_fe_generic_init(struct rfnm_m7_dgb * dgb_dt);



#define PCIE_DMA_BASE (0x33800000 + 0x0)

#define DMA_READ_ENGINE_EN_OFF          (PCIE_DMA_BASE + 0x38002C)
#define DMA_READ_INT_MASK_OFF           (PCIE_DMA_BASE + 0x3800A8)
#define DMA_CH_CONTROL1_OFF_RDCH_0      (PCIE_DMA_BASE + 0x380300)

#define DMA_TRANSFER_SIZE_OFF_RDCH_0    (PCIE_DMA_BASE + 0x380308)
#define DMA_SAR_LOW_OFF_RDCH_0          (PCIE_DMA_BASE + 0x38030C)
#define DMA_SAR_HIGH_OFF_RDCH_0         (PCIE_DMA_BASE + 0x380310)
#define DMA_DAR_LOW_OFF_RDCH_0          (PCIE_DMA_BASE + 0x380314)
#define DMA_DAR_HIGH_OFF_RDCH_0         (PCIE_DMA_BASE + 0x380318)

#define DMA_READ_DOORBELL_OFF           (PCIE_DMA_BASE + 0x380030)

#define DMA_READ_INT_CLEAR_OFF          (PCIE_DMA_BASE + 0x3800AC)










#endif