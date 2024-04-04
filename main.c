/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_gpt.h"
#include "fsl_gpio.h"

#include "fsl_common.h"
#include "rfnm_m7.h"

volatile bool gptIsrFlag = false;

volatile struct rfnm_m7_dgb m7_dgb __attribute__((section(".NonCacheable"))) __attribute__((aligned(16)));


int pp_txrx = 0;

rf_ctrl_s rf_ctrl;

void GPT1_IRQHandler(void)
{
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F, Cortex-M7, Cortex-M7F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U || __CORTEX_M == 7U)
    __DSB();
#endif


    if(GPT_GetStatusFlags(GPT1, kGPT_InputCapture2Flag)) {

        GPT_ClearStatusFlags(GPT1, kGPT_InputCapture2Flag);

        uint32_t gpt_time_at_cap, gpt_trigger_time;
        
        memcpy(&rf_ctrl, PCIE_CTRL_ADDR_FROM_M7, sizeof(rf_ctrl_s));

        uint32_t cap_to_trigger = rf_ctrl.target_phytimer_ts - rf_ctrl.issued_phytimer_ts;

        if(rf_ctrl.issued_phytimer_ts > rf_ctrl.target_phytimer_ts) {
            cap_to_trigger += UINT32_MAX;
        }

        gpt_time_at_cap = GPT_GetInputCaptureValue(GPT1, kGPT_InputCapture_Channel2);

        gpt_trigger_time = gpt_time_at_cap + (cap_to_trigger / 16);
        gpt_trigger_time -= INTERRUPT_LATENCY_OFFSET;

        GPT_SetOutputCompareValue(GPT1, kGPT_OutputCompare_Channel1, gpt_trigger_time);
        GPT_EnableInterrupts(GPT1, kGPT_OutputCompare1InterruptEnable);

        if(rf_ctrl.mode == 0xaaaaaaaa) {
            pp_txrx = 0;
        }

        if(rf_ctrl.mode == 0xbbbbbbbb) {
            pp_txrx = 1;
        }
 
        rfnm_fe_load_latches(&m7_dgb, pp_txrx);

        if(rf_ctrl.tti_period_ts) {
            GPT_StopTimer(GPT3); 
        }

    } else if(GPT_GetStatusFlags(GPT1, kGPT_OutputCompare1Flag)) {

        if(rf_ctrl.tti_period_ts && rf_ctrl.tti_period_ts > 1) {
            // As per RM writing to the output compare register should reset the timer, 
            // but that doesn't seem to happen in practice... ???
            GPT_SetOutputCompareValue(GPT3, kGPT_OutputCompare_Channel1, rf_ctrl.tti_period_ts / 16);

            GPT_StartTimer(GPT3);
            
            //GPIO_PinWrite(GPIO5, 17U, 1U);
            //GPIO_PinWrite(GPIO5, 17U, 0U);
        }

        GPT_DisableInterrupts(GPT1, kGPT_OutputCompare1InterruptEnable);
        GPT_ClearStatusFlags(GPT1, kGPT_OutputCompare1Flag);


        
        

        //for(volatile int i = i; i < 100; i++) {
        //    i++;
        //    i--;
        // }
        
        rfnm_fe_trigger_latches(&m7_dgb, pp_txrx);

        //last_pp_txrx = !last_pp_txrx;

        

    }






}






int main(void)
{
    uint32_t captureVal = 0;
    gpt_config_t gptConfig;

    //while(1);
    
    //gpt_clock_source_t gptClkSource = kGPT_ClockSource_Ext;

    /* Board pin, clock, debug console init */
    /* M7 has its local cache and enabled by default,
     * need to set smart subsystems (0x28000000 ~ 0x3FFFFFFF)
     * non-cacheable before accessing this address region */
    BOARD_InitMemory();


    /* Board specific RDC settings */
    BOARD_RdcInit();

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    //BOARD_InitDebugConsole();

    /* Print a note to terminal */
    //PRINTF("\r\nGPT input capture example\r\n");
    //PRINTF("\r\nOnce the input signal is received the input capture value is printed\r\n");

    GPT_GetDefaultConfig(&gptConfig);

    gptConfig.clockSource = kGPT_ClockSource_Ext;
    gptConfig.enableFreeRun = true;

    GPT_Init(GPT1, &gptConfig);

    gptConfig.enableFreeRun = false;
    gptConfig.enableMode = true;

    GPT_Init(GPT3, &gptConfig);
    GPT_EnableInterrupts(GPT3, kGPT_OutputCompare1InterruptEnable);

    GPT_StartTimer(GPT1);    
#ifndef RFNM_CHECK_ALIGNMENT
    GPT_SetInputOperationMode(GPT1, kGPT_InputCapture_Channel2, kGPT_InputOperation_BothEdge);
#else
    GPT_SetInputOperationMode(GPT1, kGPT_InputCapture_Channel2, kGPT_InputOperation_RiseEdge);
#endif
    
    rfnm_gpio_init();

//    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
//    GPIO_PinInit(GPIO4, 30U, &led_config);



// transfer size -> MBps single ch -> MBps single ch with partial reset as you are writing -> MBps with dual channel

// 0x400 -> 334 -> 360 -> 676 MBps
// 0x800 -> 458 -> 492 -> 787 MBps
// 0x1000 -> 588 -> 605
// 0x2000 -> 669 -> 684
// 0x4000 -> 730 -> 737 -> 780 MBps


    uint32_t *reg;
    volatile int cnt = 0;

    reg = DMA_READ_ENGINE_EN_OFF; 
    *reg = 0x0;

    reg = DMA_READ_ENGINE_EN_OFF; 
    *reg = 0x1;
    reg = DMA_READ_INT_MASK_OFF; 
    *reg = 0x0;
    
    uint32_t can_setup_dma[2] = {1, 1};

    volatile struct rfnm_bufdesc_rx *bufdesc = (struct rfnm_bufdesc_rx *) VSPA_MEM_ADDR_FROM_M7;
    volatile struct rfnm_la9310_status *rfnm_la9310_status = (struct rfnm_la9310_status *) 0x00900000;
    volatile struct rfnm_m7_status *rfnm_m7_status = (struct rfnm_la9310_status *) (0x00900000 + 0x100);
    //volatile struct rfnm_rx_usb_head *rfnm_rx_usb_head = (struct rfnm_la9310_status *) 0x96400000;
    
    
    int32_t dma_buf_last_map[2] = {-1, -1};

    int16_t buffs_to_ign[RFNM_IGN_BUFCNT];
    int16_t last_ign_buf = 0;

    for(int i = 0; i < RFNM_IGN_BUFCNT; i++) {
        buffs_to_ign[i] = -1;
    }

    rfnm_la9310_status[0].age = 0;
    while(rfnm_la9310_status[0].age == 0) {}

    rfnm_m7_status->rx_head = 0;

    
    


    while(1) {

        for(int dma = 0; dma < 2; dma++) {
            uint32_t off = 0x200 * dma;

            reg = DMA_CH_CONTROL1_OFF_RDCH_0 + off;
            //while((*reg & 0x60) != 0x60) { }
            if((*reg & 0x60) == 0x60) {
                can_setup_dma[dma] = 1;
            }
            
            if(!can_setup_dma[dma]) {
                continue;
            }

            if(dma_buf_last_map[dma] != -1) {
                bufdesc[dma_buf_last_map[dma]].read = 1;
                dma_buf_last_map[dma] = -1;
            }

            

            uint16_t ages[4];
            
            for(int i = 0; i < 4; i++) {
                ages[i] = rfnm_la9310_status[i].age;
            }

            int is_ages_overflowing = 0;
            
            for(int i = 1; i < 4; i++) {
                if(ages[0] > ages[i] && (ages[0] - ages[i]) > INT16_MAX) {
                    is_ages_overflowing = 1;
                    break;
                } else if(ages[i] > ages[0] && (ages[i] - ages[0]) > INT16_MAX) {
                    is_ages_overflowing = 1;
                    break;
                }
            }

            int winning_age_val = ages[0];
            int winning_age_id = 0;
            for(int i = 1; i < 4; i++) {
                if(
                    (is_ages_overflowing && winning_age_val >= ages[i]) || 
                    (!is_ages_overflowing && winning_age_val <= ages[i])
                ) {
                    winning_age_id = i;
                    winning_age_val = ages[i];
                }
            }

//            if(winning_age_id == 0 && skip[0]) {
//                continue;
//            }

            rfnm_m7_status->tx_buf_id = rfnm_la9310_status[winning_age_id].tx_buf_id;


            int win_b = -1;
            int win_age = 0;

            for(int b = 0; b < RFNM_RX_BUF_CNT; b++) {
                uint16_t t_age = rfnm_la9310_status[winning_age_id].rx_buf[b];
                if(0xffff != t_age && t_age > win_age) {
                    int must_ignore = 0;
                    for(int i = 0; i < RFNM_IGN_BUFCNT; i++) {
                        if(b == buffs_to_ign[i]) {
                            must_ignore = 1;
                        }
                    }
                    if(!must_ignore) {
                        win_b = b;
                        win_age = t_age;
                    }
                    
                    
                }
            }

#if 0
            int read_b = -1;
            for(int b = 0; b < RFNM_RX_BUF_CNT; b++) {
                if(read_b >= 0) {
                    if(rfnm_buf_age[b] > rfnm_buf_age[read_b]) {
                        read_b = b;
                    }
                } else {
                    if(rfnm_buf_age[b] > 0) {
                        read_b = b;
                    }
                }
            }
#endif

            if(win_b < 0) {
                continue;
            }

            
            reg = DMA_CH_CONTROL1_OFF_RDCH_0 + off; 
            //*reg = 0x04000008;
            *reg = 0x00000008 ;//| (1<<25);

            reg = DMA_READ_INT_CLEAR_OFF;
            *reg = 0x1;

            reg = DMA_TRANSFER_SIZE_OFF_RDCH_0 + off; 
            //*reg =  0x40000;
            *reg =  sizeof(struct rfnm_bufdesc_rx);
            reg = DMA_SAR_LOW_OFF_RDCH_0 + off; 
            //*reg = 0x1C010A00;
            *reg =  0x1F400000 + (win_b * sizeof(struct rfnm_bufdesc_rx)); 
            //*reg =  0x1c000000;
            //*reg = 0x1f000000;
            reg = DMA_SAR_HIGH_OFF_RDCH_0 + off; 
            *reg =  0x00000000;
            reg = DMA_DAR_LOW_OFF_RDCH_0 + off; 
            *reg =  0x96400000 + (rfnm_m7_status->rx_head * sizeof(struct rfnm_bufdesc_rx));
            reg = DMA_DAR_HIGH_OFF_RDCH_0 + off; 
            *reg =  0x00000000;

            
            reg = DMA_READ_DOORBELL_OFF; 
            *reg =  dma;

            can_setup_dma[dma] = 0;
            dma_buf_last_map[dma] = win_b;

            buffs_to_ign[last_ign_buf] = win_b;

            if(++last_ign_buf == RFNM_IGN_BUFCNT) {
                last_ign_buf = 0;
            }

            //rfnm_buf_age[(winning_age_id * 32) + win_b] = 0xffff;

            //rfnm_buf_age[read_b] = 0; // meh what a shitshow
            //rfnm_buf_age[(win_b * 32)] = 0xffff; // meh what a shitshow

            cnt++;

            if(++rfnm_m7_status->rx_head == RFNM_ADC_BUFCNT) {
                rfnm_m7_status->rx_head = 0;
            }
        }

        

        

        

        

        

    }


















     
    memset(&m7_dgb, 0, sizeof(struct rfnm_m7_dgb));


    while (true)
    {

        int i;
#if 0
        for(i = 1; i < 3; i++) {
                rfnm_fe_manual_clock(0, i);
        }
        rfnm_fe_manual_clock(0, 7);
#endif
        if(m7_dgb.tdd_available && !m7_dgb.m7_tdd_initialized) {
            rfnm_fe_generic_init(&m7_dgb);

            GPT_EnableInterrupts(GPT1, kGPT_InputCapture2InterruptEnable);
            EnableIRQ(GPT1_IRQn);
            GPT_StartTimer(GPT1);

            m7_dgb.m7_tdd_initialized = 1;
        }

        

        //GPIO_PinWrite(GPIO4, 30U, 0U);
        //SDK_DelayAtLeastUs(10000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

        //GPIO_PinWrite(GPIO4, 30U, 1U);
        //SDK_DelayAtLeastUs(10000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
        /* Check whether occur interupt */
        if (true == gptIsrFlag)
        {
            captureVal = GPT_GetInputCaptureValue(GPT1, kGPT_InputCapture_Channel2);
            //PRINTF("\r\n Capture value =%x\r\n", captureVal);
            gptIsrFlag = false;
        }
        else
        {
  //          __WFI();
        }
    }
}
