/*******************************************************************************
 * Copyright 2019-2021 Microchip FPGA Embedded Systems Solutions.
 *
 * SPDX-License-Identifier: MIT
 *
 * PolarFire SoC Microprocessor Subsystem Inter-Hart Communication bare metal software driver
 * implementation.
 *
 */
#include <string.h>
#include <stdio.h>
#include "mpfs_hal/mss_hal.h"
#include "core_ihc.h"
#include "drivers/mss/mss_mmuart/mss_uart.h"

IHC_TypeDef             IHC_H0_IP_GROUP ;
IHC_TypeDef             IHC_H1_IP_GROUP ;
IHC_TypeDef             IHC_H2_IP_GROUP ;
IHC_TypeDef             IHC_H3_IP_GROUP ;
IHC_TypeDef             IHC_H4_IP_GROUP ;

IHC_TypeDef * IHC[]={ &IHC_H0_IP_GROUP , &IHC_H1_IP_GROUP, &IHC_H2_IP_GROUP, &IHC_H3_IP_GROUP, &IHC_H4_IP_GROUP};

/**
 * \brief IHC configuration
 *
 */
const uint64_t ihc_base_addess[][5U] = {
        /* hart 0 */
        {0x0,
        IHC_LOCAL_H0_REMOTE_H1,
        IHC_LOCAL_H0_REMOTE_H2,
        IHC_LOCAL_H0_REMOTE_H3,
        IHC_LOCAL_H0_REMOTE_H4},
        /* hart 1 */
        {IHC_LOCAL_H1_REMOTE_H0,
        0x0,
        IHC_LOCAL_H1_REMOTE_H2,
        IHC_LOCAL_H1_REMOTE_H3,
        IHC_LOCAL_H1_REMOTE_H4},
        /* hart 2 */
        {IHC_LOCAL_H2_REMOTE_H0,
        IHC_LOCAL_H2_REMOTE_H1,
        0x0,
        IHC_LOCAL_H2_REMOTE_H3,
        IHC_LOCAL_H2_REMOTE_H4},
        /* hart 3 */
        {IHC_LOCAL_H3_REMOTE_H0,
        IHC_LOCAL_H3_REMOTE_H1,
        IHC_LOCAL_H3_REMOTE_H2,
        0x0,
        IHC_LOCAL_H3_REMOTE_H4},
        /* hart 4 */
        {IHC_LOCAL_H4_REMOTE_H0,
        IHC_LOCAL_H4_REMOTE_H1,
        IHC_LOCAL_H4_REMOTE_H2,
        IHC_LOCAL_H4_REMOTE_H3,
        0x0},
};

/**
 * \brief IHC configuration
 *
 */
const uint64_t ihca_base_addess[5U] = {

        IHCA_LOCAL_H0,
        IHCA_LOCAL_H1,
        IHCA_LOCAL_H2,
        IHCA_LOCAL_H3,
        IHCA_LOCAL_H4
};

/**
 * \brief Remote harts connected via channel to a local hart
 *
 */
const uint32_t ihca_remote_harts[5U] = {
        IHCA_H0_REMOTE_HARTS,
        IHCA_H1_REMOTE_HARTS,
        IHCA_H2_REMOTE_HARTS,
        IHCA_H3_REMOTE_HARTS,
        IHCA_H4_REMOTE_HARTS
};

mss_ihc_plex_instance my_instance;

/*******************************************************************************
 * Local functions.
 */
#ifdef DEBUG_IHC
static uint32_t copy_message_sm(uint32_t remote_hart_id, uint32_t my_hart_id);
#endif

static uint32_t IHCA_parse_incoming_hartid(uint32_t my_hart_id, bool is_ack);

/*******************************************************************************
 * Public API Functions
 ******************************************************************************/

/**
 * Init IHC Module
 * This must be called from monitor core ( needs access to all cores/registers
 * @param none
 * @return none
 */
void IHC_global_init(void)
{
    uint32_t remote_hart_id = 0;
    uint32_t my_hart_id = 0;

    while(my_hart_id < 5U)
    {
        remote_hart_id = 0;
        IHC[my_hart_id]->local_h_setup.msg_in_handler = NULL;
        while(remote_hart_id < 5U)
        {
            IHC[my_hart_id]->HART_IHC[remote_hart_id] = (IHC_IP_TypeDef *)ihc_base_addess[my_hart_id][remote_hart_id];
            IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG |= MPIE_EN; //fix me
            IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG |= ACKIE_EN; //fix me
            #ifdef DEBUG_IHC
            IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG = 0U;
            IHC[my_hart_id]->interrupt_concentrator->MSG_AVAIL_STAT.MSG_AVAIL = 0;
            #endif
            remote_hart_id++;
        }
        IHC[my_hart_id]->interrupt_concentrator = (IPCA_IP_TypeDef *)ihca_base_addess[my_hart_id];
        my_hart_id++;
    }
}


/**
 * This is called from the local hart
 * @param handler
 * @return
 */
uint8_t IHC_local_context_init(uint32_t hart_to_configure, QUEUE_IHC_INCOMING  handler)
{
    uint8_t result = false;

    (void)result;

    /*
     * Set-up enables in concentrator
     */
    IHC[hart_to_configure]->local_h_setup.msg_in_handler = handler;
    // todo - add support for int enable as required- IHC[hart_to_configure]->HART_IHC[0]->CTR_REG.CTL_REG = MPIE_EN;

    return(true);
}

uint8_t IHCA_local_context_init(uint32_t hart_to_configure, QUEUE_IHC_INCOMING  handler)
{
    uint8_t result = false;

    (void)result;

    IHC[hart_to_configure]->local_h_setup.connected_harts = ihca_remote_harts[hart_to_configure];

    if(handler != NULL)
    {
    	IHC[hart_to_configure]->interrupt_concentrator->INT_EN.INT_EN = ihca_remote_harts[hart_to_configure];
    }

    return(true);
}

#define IHC_INCOMING_HANDLER example_incoming_handler
uint32_t example_incoming_handler(uint32_t remote_hart_id,  uint32_t * incoming_msg)
{

    (void)remote_hart_id;
    (void)incoming_msg;
    return(0U);
}

/**
 * Send a message to another hart using CoreIHC
 * @param remote_hart_id
 * @param message
 * @return MP_BUSY / MESSAGE_SENT
 */
uint32_t IHC_tx_message(IHC_CHANNEL channel, uint32_t *message)
{
    uint32_t i, ret_value;

    uint32_t my_hart_id = context_to_hart_id(read_csr(mhartid), IHC_CHANNEL_SIDE_REMOTE);
    uint32_t remote_hart_id = context_to_hart_id(channel, IHC_CHANNEL_SIDE_REMOTE);
    uint32_t message_size = IHC[my_hart_id]->HART_IHC[remote_hart_id]->size_msg;

    /*
     * return if RMP bit 1 indicating busy
     */
    if (RMP_MESSAGE_PRESENT == (IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG & RMP_MASK))
    {
        ret_value = MP_BUSY;
    }
    else
    {
        /*
         * Fill the buffer
        */
        for(i = 0;i < message_size; i++)
        {
            IHC[my_hart_id]->HART_IHC[remote_hart_id]->mesg_out[i] = message[i];
        }
        /*
         * set the MP bit. This will notify other of incoming hart message
         */
        IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG |= RMP_MESSAGE_PRESENT;
        /*
         * report status
         */
        ret_value = MESSAGE_SENT;
    }

    return (ret_value);
}

/**
 * Called by application ( eg rp message )
 * @param remote_hart_id
 * @param handle_incoming This is a point to a function that is provided by
 * upper layer. It will read/copy the incoming message.
 * @return
 */
uint32_t IHC_rx_message(IHC_CHANNEL channel, QUEUE_IHC_INCOMING handle_incoming)
{
    uint32_t ret_value;
    uint32_t my_hart_id = read_csr(mhartid);
    uint32_t remote_hart_id = context_to_hart_id(channel, IHC_CHANNEL_SIDE_REMOTE);
    uint32_t message_size = IHC[my_hart_id]->HART_IHC[remote_hart_id]->size_msg;

    /*
     * check if we have a message
     */
    if (MP_MESSAGE_PRESENT == (IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG & MP_MASK))
    {
        handle_incoming(remote_hart_id, (uint32_t *)&IHC[my_hart_id]->HART_IHC[remote_hart_id]->mesg_in[0U], message_size);
        {
            /*
             * set MP to 0
             * Note this generates an interrupt on the other hart if it has RMPIE
             * bit set in the control register
             */
            IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG &= ~MP_MASK;
            ret_value = MESSAGE_RX;
        }

        /* Check if ACKIE_EN is set*/
        if(IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG & ACKIE_EN)
        {
            //set ACK
            IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG |= ACK_INT;
        }
    }
    else
    {
        /*
         * report status
         */
        ret_value = NO_MESSAGE_RX;
    }

    return (ret_value);
}

/*******************************************************************************
 * Interrupt support functions
 ******************************************************************************/

bool IHCA_is_ack_irq(void)
{
    uint32_t my_hart_id = read_csr(mhartid);
    uint32_t is_ack;

    if(IHC[my_hart_id]->interrupt_concentrator->MSG_AVAIL_STAT.MSG_AVAIL & 0x2AA)
    {
        return true;
    }
    else
    {
        return false;
    }
}


/**
 *
 */
void  message_present_isr(void)
{
    uint32_t my_hart_id = read_csr(mhartid);

    /*
     * Check all our channels
     */
	uint32_t origin_hart = IHCA_parse_incoming_hartid(my_hart_id, false);
	if(origin_hart != 99U)
	{
		/*
		 * process incoming packet
		 */
		IHC_rx_message(origin_hart, IHC[my_hart_id]->local_h_setup.msg_in_handler );
	}

    /*
     * clear the interrupt
     */
}

void message_cleared_isr(void)
{
    uint32_t my_hart_id = read_csr(mhartid);

    /*
     * Check all our channels
     */
	uint32_t origin_hart = IHCA_parse_incoming_hartid(my_hart_id, true);
	if(origin_hart != 99U)
	{
        IHC[my_hart_id]->HART_IHC[origin_hart]->CTR_REG.CTL_REG &= ~ACK_CLR;
	}
}

/*******************************************************************************
 * local functions
 ******************************************************************************/

/**
 * Check where the message is coming from
 * @return returns hart ID of incoming message
 */
static uint32_t IHCA_parse_incoming_hartid(uint32_t my_hart_id, bool is_ack)
{

    static uint32_t hart_id = 0U;
    uint32_t return_hart_id = 99U;

    while(hart_id < 5U)
    {
        if (IHC[my_hart_id]->local_h_setup.connected_harts && (0x01U << hart_id))
        {
            if(!is_ack)
            {
                if(IHC[my_hart_id]->interrupt_concentrator->MSG_AVAIL_STAT.MSG_AVAIL & (0x01U << (hart_id * 2)))
                {
                    return_hart_id = hart_id;
                    break;    
                }
            }
            else
            {
                if(IHC[my_hart_id]->interrupt_concentrator->MSG_AVAIL_STAT.MSG_AVAIL & (0x01U << ((hart_id * 2) + 1)))
                {
                    return_hart_id = hart_id;
                    break;    
                }                
            }
        }
        hart_id++;
    }
    return(return_hart_id);
}

/**
 * Returns remote hart ID
 * @param channel
 * @return
 */
uint32_t context_to_hart_id(IHC_CHANNEL channel, IHC_CHANNEL_SIDE_SIDE side)
{
    uint32_t hart = 0xFFU;
    uint32_t hart_idx = 0U;
    uint32_t harts_in_context = LIBERO_SETTING_CONTEXT_B_HART_EN;
	if(side == IHC_CHANNEL_SIDE_LOCAL)
	{
		harts_in_context = LIBERO_SETTING_CONTEXT_A_HART_EN;
	}
	else
	{
		harts_in_context = LIBERO_SETTING_CONTEXT_B_HART_EN;
	}

#ifndef LIBERO_SETTING_CONTEXT_A_HART_EN
#error "Use newer mss configurator to configure"
#else
    switch(channel)
    {
        case IHC_CHANNEL_TO_HART0:
        case IHC_CHANNEL_TO_HART1:
        case IHC_CHANNEL_TO_HART2:
        case IHC_CHANNEL_TO_HART3:
        case IHC_CHANNEL_TO_HART4:
            hart = channel;
            break;

        case IHC_CHANNEL_TO_CONTEXTA:
        	if(side == IHC_CHANNEL_SIDE_LOCAL)
        	{
        		harts_in_context = LIBERO_SETTING_CONTEXT_B_HART_EN;
        	}
        	else
        	{
        		harts_in_context = LIBERO_SETTING_CONTEXT_A_HART_EN;
        	}
            __attribute__((fallthrough)); /* deliberately fall through */
        case IHC_CHANNEL_TO_CONTEXTB:
            /* if asserts either override wit value in mms_sw_config.h or use mss configurator to configure */
            ASSERT(LIBERO_SETTING_CONTEXT_A_HART_EN > 0U);
            ASSERT(LIBERO_SETTING_CONTEXT_B_HART_EN > 0U);
            while(hart_idx < 5U)
            {
                if  (harts_in_context & (1U<<hart_idx))
                {
                    hart = hart_idx;
                    break;
                }
                hart_idx++;
            }
            break;
    }
#endif
    return (hart);
}


/*******************************************************************************
 * Test Functions
 ******************************************************************************/

#ifdef DEBUG_IHC
/**
 * State machine emulates IP
 */
void test_state_machine(void)
{
    uint32_t remote_hart_id = 1;
    uint32_t my_hart_id = 0;

    while(my_hart_id < 5U)
    {
        remote_hart_id = 0U;
        while(remote_hart_id < 5U)
        {
            if(remote_hart_id == my_hart_id)
            {
                /* skip when local/remote the same */
            }
            else
            {
                copy_message_sm(remote_hart_id, my_hart_id);
            }
            remote_hart_id++;
        }
        my_hart_id++;
    }

    return;
}
#endif /* #ifdef DEBUG_IHC */

#ifdef DEBUG_IHC
/**
 * State machine to send to remote hart
 * @param remote_hart_id
 * @param my_hart_id
 * @return
 */
static uint32_t copy_message_sm(uint32_t remote_hart_id, uint32_t my_hart_id)//IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG &= ~MP_MASK;
{
    uint32_t ret_value = 0U;
    static uint8_t state[5U] = { 0U, 0U, 0U, 0U, 0U };

    switch(state[my_hart_id])
    {
        case 0: /* free to send */
            /*
             * Copy message and set MP bit if new message to be sent
             */
            if ((IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG & RMP_MESSAGE_PRESENT)&&\
                    ((IHC[remote_hart_id]->HART_IHC[my_hart_id]->CTR_REG.CTL_REG & MP_MESSAGE_PRESENT) == 0U))
            {
                memcpy((uint32_t *)&IHC[remote_hart_id]->HART_IHC[my_hart_id]->mesg_in[0U],(uint32_t *)&IHC[my_hart_id]->HART_IHC[remote_hart_id]->mesg_out[0U] , IHC_MESSAGE_SIZE);
                IHC[remote_hart_id]->HART_IHC[my_hart_id]->CTR_REG.CTL_REG |= MP_MESSAGE_PRESENT;
                state[my_hart_id] = 1U;

                /* If MPIE enabled */
                if(IHC[remote_hart_id]->HART_IHC[my_hart_id]->CTR_REG.CTL_REG & MPIE_EN)
                {
                    IHC[remote_hart_id]->interrupt_concentrator->MSG_AVAIL_STAT.MSG_AVAIL |= (0x01U << my_hart_id);
                    raise_soft_interrupt(remote_hart_id);
                    	
                }
            }
            break;
        case 1: /* wait for remote to clear */
            /*
              * Clear RMP bit.
              */
             if (((IHC[remote_hart_id]->HART_IHC[my_hart_id]->CTR_REG.CTL_REG & MP_MESSAGE_PRESENT)==0U)&&\
                     ((IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG & RMP_MESSAGE_PRESENT) == RMP_MESSAGE_PRESENT))
             {
                 IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG &= ~RMP_MESSAGE_PRESENT;
                 state[my_hart_id] = 0U;
                if(IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG & MPIE_EN)
                {
                    IHC[remote_hart_id]->interrupt_concentrator->MSG_AVAIL_STAT.MSG_AVAIL &= (0x00U << my_hart_id);
                }
             }
             break;
    }

    return (ret_value);
}
#endif /* #ifdef DEBUG_IHC */

/**
 * Print memory map, used during dev to allow easy definition
 * @return
 */
#if 0
uint32_t print_mem_map_core_inc(void)
{
    char info_string[100];
    uint32_t remote_hart_id = 1;
    uint32_t my_hart_id = 0;
    HLS_DATA* hls = (HLS_DATA*)(uintptr_t)get_tp_reg();
#ifdef  MPFS_HAL_SHARED_MEM_ENABLED
    HART_SHARED_DATA * hart_share = (HART_SHARED_DATA *)hls->shared_mem;
#endif

    while(my_hart_id < 5U)
    {
#ifdef PRINT_IHC_VERBOSE
        sprintf(info_string, "\r\n************** My Hart %u ************\r\n",\
                                (int)my_hart_id);
        spinlock(&hart_share->mutex_uart0);
        MSS_UART_polled_tx(hart_share->g_mss_uart0_lo, (const uint8_t*)info_string,(uint32_t)strlen(info_string));
        spinunlock(&hart_share->mutex_uart0);
        remote_hart_id = 0U;
        while(remote_hart_id < 5U)
        {
            if(remote_hart_id == my_hart_id)
            {
                /* skip */
            }
            else
            {
                /*
                 * Init
                 */
#ifdef DEBUG_IHC
                IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG = 0U;
#endif

                sprintf(info_string, "\r\nHart %u, Remote Hart %u,version address 0x%lx\r\n",\
                        my_hart_id, remote_hart_id, (uint64_t)&IHC[my_hart_id]->HART_IHC[remote_hart_id].version);
                spinlock(&hart_share->mutex_uart0);
                MSS_UART_polled_tx(hart_share->g_mss_uart0_lo, (const uint8_t*)info_string,(uint32_t)strlen(info_string));
                spinunlock(&hart_share->mutex_uart0);

                sprintf(info_string, "\r\nHart %u, Remote Hart %u,CTR_REG address 0x%lx\r\n",\
                        my_hart_id, remote_hart_id, (uint64_t)&IHC[my_hart_id]->HART_IHC[remote_hart_id].CTR_REG);
                spinlock(&hart_share->mutex_uart0);
                MSS_UART_polled_tx(hart_share->g_mss_uart0_lo, (const uint8_t*)info_string,(uint32_t)strlen(info_string));
                spinunlock(&hart_share->mutex_uart0);

                sprintf(info_string, "\r\nHart %u, Remote Hart %u,mesg_in address 0x%lx\r\n",\
                        my_hart_id, remote_hart_id, (uint64_t)&IHC[my_hart_id]->HART_IHC[remote_hart_id]->mesg_in);
                spinlock(&hart_share->mutex_uart0);
                MSS_UART_polled_tx(hart_share->g_mss_uart0_lo, (const uint8_t*)info_string,(uint32_t)strlen(info_string));
                spinunlock(&hart_share->mutex_uart0);

                sprintf(info_string, "\r\nHart %u, Remote Hart %u,mesg out address 0x%lx\r\n",\
                        my_hart_id, remote_hart_id, (uint64_t)&IHC[my_hart_id]->HART_IHC[remote_hart_id]->mesg_out);
                spinlock(&hart_share->mutex_uart0);
                MSS_UART_polled_tx(hart_share->g_mss_uart0_lo, (const uint8_t*)info_string,(uint32_t)strlen(info_string));
                spinunlock(&hart_share->mutex_uart0);
            }
            remote_hart_id++;
        }
        sprintf(info_string, "\r\nHart %u, Remote Hart %u,interrupt_concentrator address 0x%lx\r\n",\
                my_hart_id, remote_hart_id, (uint64_t)&IHC[my_hart_id]->interrupt_concentrator);
        spinlock(&hart_share->mutex_uart0);
        MSS_UART_polled_tx(hart_share->g_mss_uart0_lo, (const uint8_t*)info_string,(uint32_t)strlen(info_string));
        spinunlock(&hart_share->mutex_uart0);
#else   /* PRINT_IHC_VERBOSE */
        sprintf(info_string, "\r\n************** My Hart %u ************\r\n",\
                                (int)my_hart_id);
        spinlock(&hart_share->mutex_uart0);
        MSS_UART_polled_tx(hart_share->g_mss_uart0_lo, (const uint8_t*)info_string,(uint32_t)strlen(info_string));
        spinunlock(&hart_share->mutex_uart0);
        remote_hart_id = 0U;
        while(remote_hart_id < 5U)
        {
            if(remote_hart_id == my_hart_id)
            {
                /* skip */
            }
            else
            {
                /*
                 * Init
                 */
#ifdef DEBUG_IHC
                IHC[my_hart_id]->HART_IHC[remote_hart_id]->CTR_REG.CTL_REG = 0U;
#endif

                sprintf(info_string, "\r\n#define IHC_LOCAL_H%u_REMOTE_H%u    0x%lx\r\n",\
                                        my_hart_id, remote_hart_id, (uint64_t)IHC[my_hart_id]->HART_IHC[remote_hart_id]);
                spinlock(&hart_share->mutex_uart0);
                MSS_UART_polled_tx(hart_share->g_mss_uart0_lo, (const uint8_t*)info_string,(uint32_t)strlen(info_string));
                spinunlock(&hart_share->mutex_uart0);
            }
            remote_hart_id++;
        }

        sprintf(info_string, "\r\n#define IHCA_LOCAL_H%u_REMOTE_H%u    0x%lx\r\n",\
                my_hart_id, remote_hart_id, (uint64_t)IHC[my_hart_id]->interrupt_concentrator);
        spinlock(&hart_share->mutex_uart0);
        MSS_UART_polled_tx(hart_share->g_mss_uart0_lo, (const uint8_t*)info_string,(uint32_t)strlen(info_string));
        spinunlock(&hart_share->mutex_uart0);
#endif  /* PRINT_IHC_VERBOSE */
        my_hart_id++;
    }


    return(0U);
}
#endif
