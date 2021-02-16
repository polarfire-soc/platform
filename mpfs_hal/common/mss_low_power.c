/*******************************************************************************
 * Copyright 2019-2020 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * MPFS HAL Embedded Software
 *
 */

/***************************************************************************
 * @file mss_low_power.c
 * @author Microchip-FPGA Embedded Systems Solutions
 * @brief low power mode entry and exit sequence
 *
 */
#include <stddef.h>
#include <stdbool.h>
#include "mpfs_hal/mss_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 *   MSS_enter_lp_mode(void)
 *
 *   Preconditions
 *   Prior to entry, MSS PLL's not being used must have low power lock
 *   bit set to 1, so it is not included in the low power lock test
 *   i.e step 5 on entry, step 4 on exit
 *
 *   MSS Low Power Mode Entry
 *   1.  Write MSSIO_CONTROL_CR.lp_stop_clocks_out_mss to 1
 *   2.  Write MSSIO_CONTROL_CR.lp_state_persist_mss to 1
 *   3.  Write MSSIO_CONTROL_CR.lp_state_bypass_mss to 1
 *   4.  Write MSSIO_CONTROL_CR.lp_state_mss to 1
 *   5.  Poll MSSIO_CONTROL_CR.lp_stop_clocks_in_mss, waiting for it to be 1
 *   6.  Write MSSIO_CONTROL_CR.lp_state_ip_mss to 1
 *   7.  Write MSSIO_CONTROL_CR.lp_state_op_mss to 1
 *
 *   MSS Low Power Mode Exit
 *   1.  Write MSSIO_CONTROL_CR.lp_state_ip_mss to 0
 *   2.  Write MSSIO_CONTROL_CR.lp_state_mss to 0
 *   3.  Write MSSIO_CONTROL_CR.lp_state_op_mss to 0
 *   4.  Poll MSSIO_CONTROL_CR.lp_stop_clocks_in_mss, waiting for it to be 0
 *   5.  Poll the PLL lock status of each of the three PLLs (MSS, DDR and SGMII) or
 *       as many as required depending on the which ones are being used in the user
 *       application, waiting for lock to be required:
 *   a.  Poll PLL_STATUS_SR.CPU_LOCK_NOW, waiting for it to be 1
 *   b.  Poll PLL_STATUS_SR.DFI_LOCK_NOW, waiting for it to be 1
 *   c.  Poll PLL_STATUS_SR.SGMII_LOCK_NOW, waiting for it to be 1
 *   6.  Write MSSIO_CONTROL_CR.lp_stop_clocks_out_mss to 0
 *   7.  Write MSSIO_CONTROL_CR.lp_state_persist_mss to 0
 *   8.  Write MSSIO_CONTROL_CR.lp_state_bypass_mss to 0
 *
 *
 * @return
 */
__attribute__((section(".ram_codetext"))) uint32_t MSS_enter_lp_mode(void)
{
    uint32_t error = 0;
    uint32_t timeout_ms = 0;    /* if 0, no timeout */
    uint16_t mb_offset = 10U;   /* not sure we use this */

    /*
     *  Note: This code is located in RAM
     *  It has previously been copied there using the routine
     *  copy_switch_code()
     *
     */

    /*
     * Enable_1mhz bit needs to be set, so clock is not lost
     * by System controller firmware turning off clocks.
     */
    SYSREG->CLOCK_CONFIG_CR |= (0x1U<<8U);

    /*
     * Switch the clock so using the 80Mhz SCB clock
     */
    /*
     * Feed clock from MSS PLL to MSS
     */
    /*
      * MSS Clock mux selections
      *  [31:5]  Reserved
      *  [4]     clk_standby_sel
      *  [3:2]   mssclk_mux_md
      *  step 7: 7)  MSS Processor writes mssclk_mux_sel_int<0>=1 to select the MSS PLL clock.
      *  [1:0]   mssclk_mux_sel        MSS glitchless mux select
      *                                      00 - msspll_fdr_0=clk_standby
      *                                           msspll_fdr_1=clk_standby
      *                                      01 - msspll_fdr_0=pllout0
      *                                           msspll_fdr_1=clk_standby
      *                                      10 - msspll_fdr_0=clk_standby
      *                                           msspll_fdr_1=pllout1
      *                                      11 - msspll_fdr_0=pllout0
      *                                           msspll_fdr_1=pllout1
      *
      *
      */
    MSS_SCB_CFM_MSS_MUX->MSSCLKMUX          = 0;
    /*
     * Change the MSS clock as required.
     *
     * CLOCK_CONFIG_CR
     * [5:0]
     * Sets the master synchronous clock divider
     * bits [1:0] CPU clock divider
     * bits [3:2] AXI clock divider
     * bits [5:4] AHB/APB clock divider
     * 00=/1 01=/2 10=/4 11=/8 (AHB/APB divider may not be set to /1)
     * Reset = 0x3F
     *
     * fixme: Noted in Simulation, value of CLOCK_CONFIG_CR = 0x10
     * Anupama is not pre setting this (in test.c run befoe simulation), how come
     * it is this value (should be 0x3F according to docs)?
     * Given MSS clk= 80Mhz, implies CPU = 80Mhz, AXI = 80Mhz, AHB/APB = 40Mhz
     */
    SYSREG->CLOCK_CONFIG_CR = (0x0U<<0U) | (0x0U<<2U) | (0x1U<<4U) | (0x1U<<8U); /* fixme: actually clock is 1MHz at thei point */

    /*
     * Put MSS in LP mode
     */
    /*
       lp_state_mss         :1;
       lp_state_ip_mss      :1;
       lp_state_op_mss      :1;
       lp_state_persist_mss :1;
       lp_state_bypass_mss  :1;
       lp_pll_locked_mss    :1;
       lp_stop_clocks_out_mss :1;
       lp_stop_clocks_in_mss :1;
       mss_dce              :3;
       mss_core_up          :1;
       mss_flash_valid      :1;
       mss_io_en            :1;
    */
    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR |= (0x01<<6); /* stop clocks */
    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR |= (0x01U << 3U); /* set  lp_state_persist_mss */
    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR |= (0x01U << 4U); /* set  lp_bypass_mss */
    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR |= (0x01U << 0U); /* set  lp_state_mss */


    /*
     * Now wait until clocks are stopped
     */
    uint32_t loop;
    while((SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR & (0x01<<7)) == 0U)
    {
        loop++;
        if (loop >= 0x100) /* Dummy loop*/
            loop = 0;
    }

    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR |= (0x01U << 1U); /* set  lp_state_ip_mss */
    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR |= (0x01U << 2U); /* set  lp_state_op_mss */
    /*
     * set any persist I/O you require
     */
    /*
     * e.g.
    SYSREG->mssio_bank4_io_cfg_0_1_cr |= mssio_bank4_io_cfg_0_1_cr_rpc_io_cfg_0_lp_persist_en_MASK;
    or
    SYSREG->mssio_bank4_io_cfg_0_1_cr |= mssio_bank4_io_cfg_0_1_cr_rpc_io_cfg_0_lp_bypass_en_MASK;
    */

    return(error);
}

/**
 * MSS_exit_lp_mode()
 *
 * Exact reverse process compared to MSS_exit_lp_mode() is carried out while
 * exiting the low power mode.
 * At the end the PLL configurations are changed to select the appropriate
 * clock settings.
 * @return
 */
__attribute__((section(".ram_codetext"))) uint32_t MSS_exit_lp_mode(void)
{
    uint32_t error = 0;
    uint32_t loop;

    /*
     * set bits in order as Eugene observed the SC firmware doing
     */
    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR &= ~(0x01U << 1U); /* unset  lp_state_ip_mss */
    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR &= ~(0x01U << 0U); /* unset  lp_state_mss */
    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR &= ~(0x01U << 2U); /* unset  lp_state_op_mss */
    /*
     * Now 4.   Poll MSSIO_CONTROL_CR.lp_stop_clocks_in_mss, waiting for it to be 0
     */
    while((SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR & (0x01<<7)) != 0U)
    {
        loop++;
        if (loop >= 0x100)  /* need code here, otherwise debugger will freeze */
            loop = 0;
    }
    /*
     * Check PLL's lock
     */
    if (MSS_SCB_DDR_PLL->PLL_CTRL & (0x01U << 24U))
    {
        while((MSS_SCB_MSS_PLL->PLL_CTRL & (0x01U << 25U)) == 0U)
        {
            loop++;
            if (loop >= 0x100)  /* need code here, otherwise debugger will freeze */
                loop = 0;
        }
    }
    if (MSS_SCB_DDR_PLL->PLL_CTRL & (0x01U << 24U))
    {
        while((MSS_SCB_DDR_PLL->PLL_CTRL & (0x01U << 25U)) == 0U)
        {
            loop++;
            if (loop >= 0x100)  /* need code here, otherwise debugger will freeze */
                loop = 0;
        }
    }
    if (MSS_SCB_SGMII_PLL->PLL_CTRL & (0x01U << 24U))
    {
        while((MSS_SCB_SGMII_PLL->PLL_CTRL & (0x01U << 25U)) == 0U)
        {
            loop++;
            if (loop >= 0x100)  /* need code here, otherwise debugger will freeze */
                loop = 0;
        }
    }
    /*
     * Enable clocks
     */
    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR &= ~(0x01<<6);      /* enable clocks */
    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR &= ~(0x01U << 3U); /* unset  lp_state_persist_mss */
    SCB_REGS->MSSIO_CONTROL_CR.MSSIO_CONTROL_CR &= ~(0x01U << 4U); /* unset  lp_state_bypass_mss */

    /*
     * Switch MSS clock back-in, once locked
     */
    MSS_SCB_CFM_MSS_MUX->MSSCLKMUX          = 0;
    mss_mux_post_mss_pll_config();

    return(error);
}

#ifdef __cplusplus
}
#endif
