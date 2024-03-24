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
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_GPT_IRQn GPT1_IRQn
/* GPT channel used for input capture */
#define BOARD_GPT_INPUT_CAPTURE_CHANNEL kGPT_InputCapture_Channel2
/* Interrupt to enable and flag to read; depends on the GPT channel used */
#define EXAMPLE_GPT_CAPTURE_IRQHandler     GPT1_IRQHandler
#//define BOARD_GPT_CHANNEL_INTERRUPT_ENABLE kGPT_InputCapture2InterruptEnable
//#define BOARD_GPT_CHANNEL_FLAG             kGPT_InputCapture2Flag

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

volatile bool gptIsrFlag = false;

volatile struct rfnm_m7_dgb m7_dgb __attribute__((section(".NonCacheable"))) __attribute__((aligned(16)));


int pp_txrx = 0;

rf_ctrl_s rf_ctrl;

void EXAMPLE_GPT_CAPTURE_IRQHandler(void)
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

        gpt_time_at_cap = GPT_GetInputCaptureValue(GPT1, BOARD_GPT_INPUT_CAPTURE_CHANNEL);

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
    GPT_SetInputOperationMode(GPT1, BOARD_GPT_INPUT_CAPTURE_CHANNEL, kGPT_InputOperation_BothEdge);
#else
    GPT_SetInputOperationMode(GPT1, BOARD_GPT_INPUT_CAPTURE_CHANNEL, kGPT_InputOperation_RiseEdge);
#endif
    
    rfnm_gpio_init();

//    gpio_pin_config_t led_config = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
//    GPIO_PinInit(GPIO4, 30U, &led_config);

     
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
            EnableIRQ(DEMO_GPT_IRQn);
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
            captureVal = GPT_GetInputCaptureValue(GPT1, BOARD_GPT_INPUT_CAPTURE_CHANNEL);
            //PRINTF("\r\n Capture value =%x\r\n", captureVal);
            gptIsrFlag = false;
        }
        else
        {
  //          __WFI();
        }
    }
}
