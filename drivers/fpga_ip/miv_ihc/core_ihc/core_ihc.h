/*******************************************************************************
 * Copyright 2019-2021 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 *
 * PolarFire SoC Microprocessor Subsystem bare metal software driver
 * public API.
 *
 */
/*=========================================================================*//**
  @mainpage PolarFire FPGA-IP CoreIPC Bare Metal Driver.

  ==============================================================================
  Introduction
  ==============================================================================
  The Bare metal CoreIPC driver faciliates the use of the CoreIPC FPGA-IP which
..helps inter hart/processor communication as part of a IP/SW stack.
  ==============================================================================
  Hardware Flow Dependencies
  ==============================================================================
  The CoreIPC IP must be present in your FPGA design and connected per UG.

  ==============================================================================
  Theory of Operation
  ==============================================================================
  The CoreIPC faciliates sending messages between independant OS running on
  seperate harts or processors.
    - Initialization and configuration functions
    - Polled transmit and receive functions
    - Interrupt driven transmit and receive functions

  --------------------------------
  Initialization and Configuration
  --------------------------------
  At start-up, one hart should be designated to initialise the CoreIPC.
  This hart must have access to the fuull memory range.
  Local contexts can request re-initialisation and notify their status

  --------------------------------------
  Polled Transmit and Receive Operations
  --------------------------------------
  Will add more detail here

  ---------------------------
  Interrupt Driven Operations
  ---------------------------
  The driver can -
    - Concentrator
    - message avaialble
    - message cleared

  -----------
  CoreIPC Status
  -----------
  Registers used to determine status:
     add detail here


 *//*=========================================================================*/
#ifndef __MSS_CORE_IHC_H_
#define __MSS_CORE_IHC_H_ 1

#include <stddef.h>
#include <stdint.h>
#include "core_ihc_regs.h"

#ifdef __cplusplus
extern "C" {
#endif

extern IHC_TypeDef * IHC[];

typedef enum IHC_CHANNEL_
{
    IHC_CHANNEL_TO_HART0              = 0x00,       /*!< 0 default behavior */
    IHC_CHANNEL_TO_HART1              = 0x01,       /*!< 1 u mode */
    IHC_CHANNEL_TO_HART2              = 0x02,       /*!< 2 s mode */
    IHC_CHANNEL_TO_HART3              = 0x03,       /*!< 2 s mode */
    IHC_CHANNEL_TO_HART4              = 0x04,       /*!< 2 s mode */
    IHC_CHANNEL_TO_CONTEXTA           = 0x05,       /*!< 2 s mode */
    IHC_CHANNEL_TO_CONTEXTB           = 0x06,       /*!< 2 s mode */
}   IHC_CHANNEL;

typedef enum IHC_CHANNEL_SIDE_
{
    IHC_CHANNEL_SIDE_LOCAL            = 0x00,       /*!< 0 local side */
    IHC_CHANNEL_SIDE_REMOTE           = 0x01,       /*!< 1 remote side */
}   IHC_CHANNEL_SIDE_SIDE;


/***************************************************************************//**
  mss_ihc_plex_instance.
 */
typedef struct mss_ihc_plex_instance_{
    uint32_t    my_hart_id;
    uint32_t    padding;
    QUEUE_IHC_INCOMING  q_in_handler_t;   /*!< Pointer to function used to handle incoming data */
}mss_ihc_plex_instance;

uint32_t
queue_incoming
(
    uint32_t remote_hart_id,
    uint32_t my_hart_id,
    uint8_t * message
);

void IHC_global_init(void);
uint32_t context_to_hart_id(IHC_CHANNEL channel, IHC_CHANNEL_SIDE_SIDE side);
extern uint32_t IHC_tx_message(IHC_CHANNEL channel, uint32_t *message);
uint32_t IHC_rx_message(IHC_CHANNEL channel, QUEUE_IHC_INCOMING handle_incoming);
void int_rx_handler(void);
uint8_t IHC_local_context_init( uint32_t remote_hart_id, QUEUE_IHC_INCOMING  handler);
void IHCA_global_init(void);
uint8_t IHCA_local_context_init(uint32_t hart_to_configure, QUEUE_IHC_INCOMING  handler);
uint32_t example_incoming_handler(uint32_t remote_hart_id,  uint32_t * incoming_msg);
void  message_present_isr(void);
void  message_cleared_isr(void);
bool IHCA_is_ack_irq(void);


typedef void (*ihc_handler_t)(void);

#ifndef IHC_INCOMING_HANDLER
#define IHC_INCOMING_HANDLER example_incoming_handler
#endif

/**
 * Temp during dev
 * @return
 */
uint32_t print_mem_map_core_inc(void);

#ifdef DEBUG_IHC
void test_state_machine(void);
#endif


#ifdef __cplusplus
}
#endif

#endif /* __MSS_CORE_IHC_H_ */