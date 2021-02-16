/*******************************************************************************
 * Copyright 2019-2020 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * MPFS HAL Embedded Software
 *
 */

/***************************************************************************
 * @file mss_util.h
 * @author Microchip-FPGA Embedded Systems Solutions
 * @brief MACROs defines and prototypes associated with utility functions
 *
 */
#ifndef G5SOC_LP_H
#define G5SOC_LP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/***************************************************************************//**
  MSS_enter_lp_mode() Puts MSS into the low power mode.

  Example:
  @code

      MSS_enter_lp_mode();

  @endcode

 */
uint32_t MSS_enter_lp_mode(void);

/***************************************************************************//**
  MSS_exit_lp_mode() configure DDR PLL

  Example:
  @code

      MSS_exit_lp_mode();

  @endcode

 */
uint32_t MSS_exit_lp_mode(void);

#ifdef __cplusplus
}
#endif

#endif  /* G5SOC_LP_H */
