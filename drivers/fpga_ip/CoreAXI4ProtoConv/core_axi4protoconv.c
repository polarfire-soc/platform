/*******************************************************************************
 * (c) Copyright 2024 Microchip FPGA Embedded Systems Solutions.
 * 
 * SPDX-License-Identifier: MIT
 *
 * @file core_axi4protoconv.c
 * @author Microchip FPGA Embedded Systems Solutions
 * @brief CoreAXI4ProtoConv IP bare metal driver implementation.
 * See file "core_axi4protoconv.h" for description of the functions implemented
 * in this file.
 *
 */

#include "coreaxi4protoconv_regs.h"
#include "core_axi4protoconv.h"
#include "coreaxi4protoconv_user_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Null parameters with appropriate type definitions
 */

#define NULL_INSTANCE       (( PCDMA_instance_t* ) 0)
#define NULL_VAL            ( 0)

/*******************************************************************************
 * PCDMA_init()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
void
PCDMA_init
(
    PCDMA_instance_t  * this_PCDMA,
    addr_t base_addr
)
{
        HAL_ASSERT( this_PCDMA != NULL_INSTANCE );
        HAL_ASSERT( base_addr != 0u );

        if( this_PCDMA != NULL_INSTANCE )
        {
            /* Set base address of CoreAXI4ProtoConv hardware. */
            this_PCDMA->base_address = base_addr;
        }
}

#ifndef S2MM_ENABLE
/*******************************************************************************
 * PCDMA_S2MM_configure()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
void PCDMA_S2MM_configure
(
    PCDMA_instance_t  * this_PCDMA,
    uint32_t xfr_size,
    uint64_t start_add,
    uint16_t cmd_id,
    uint8_t burst_type
)
{

        HAL_ASSERT( this_PCDMA != NULL_INSTANCE );
        HAL_ASSERT( xfr_size != 0u );
        HAL_ASSERT( cmd_id <= 0x3ff );
        HAL_ASSERT( burst_type <= 0x1 );

        if((xfr_size != 0u) && (cmd_id <= 0x3ff) && (burst_type <= 0x1) )
        {
            /* Configure transfer size for the total number of bytes to be
             * transmitted.
             */
            HAL_set_32bit_reg( this_PCDMA->base_address, \
                                   COREAXI4PROTOCONV_REGS_S2MM_LEN, xfr_size );

            /* Configure the lower 32-bits address of the AXI4 memory mapped
             * interface.
             */
            HAL_set_32bit_reg( this_PCDMA->base_address, \
                                   COREAXI4PROTOCONV_REGS_S2MM_ADDR0, \
                                   (uint32_t) (0xffffffff & start_add));

            /* Configure the upper 32-bits address of the AXI4 memory mapped
             * interface.
             */
            HAL_set_32bit_reg( this_PCDMA->base_address, \
                                   COREAXI4PROTOCONV_REGS_S2MM_ADDR1, \
                                   (uint32_t) (0xffffffff & (start_add >> 32)));

            /* Configure the burst type */
            HAL_set_32bit_reg( this_PCDMA->base_address, \
                                   COREAXI4PROTOCONV_REGS_S2MM_CTRL,  \
                                   ((cmd_id << 16) | (burst_type << 1)));
        }

}

/*******************************************************************************
 * PCDMA_S2MM_start_transfer()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
void PCDMA_S2MM_start_transfer
(
    PCDMA_instance_t  * this_PCDMA
)
{
            uint32_t control_reg_value;

            HAL_ASSERT( this_PCDMA != NULL_INSTANCE );

            if( this_PCDMA != NULL_INSTANCE )
            {
               control_reg_value = HAL_get_32bit_reg( this_PCDMA->base_address,\
                                         COREAXI4PROTOCONV_REGS_S2MM_CTRL);

               HAL_set_32bit_reg( this_PCDMA->base_address, \
                                       COREAXI4PROTOCONV_REGS_S2MM_CTRL, \
                                                  (control_reg_value | 0x1) );
            }

}

/*******************************************************************************
 * PCDMA_S2MM_get_status()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
uint32_t PCDMA_S2MM_get_status
(
    PCDMA_instance_t  * this_PCDMA
)
{
            uint32_t status;

            HAL_ASSERT( this_PCDMA != NULL_INSTANCE );

            if( this_PCDMA != NULL_INSTANCE )
            {
                status = HAL_get_32bit_reg( this_PCDMA->base_address, \
                                             COREAXI4PROTOCONV_REGS_S2MM_STS);
            }
            return (status);
}

/*******************************************************************************
 * PCDMA_S2MM_enable_irq()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
void PCDMA_S2MM_enable_irq
(
   PCDMA_instance_t  * this_PCDMA,
   uint32_t irq_type
)
{
        uint32_t reg_value;

        HAL_ASSERT( this_PCDMA != NULL_INSTANCE );

        reg_value = HAL_get_32bit_reg( this_PCDMA->base_address, \
                                          COREAXI4PROTOCONV_REGS_S2MM_INTRENB);
        reg_value |= irq_type;

        HAL_set_32bit_reg( this_PCDMA->base_address, \
                               COREAXI4PROTOCONV_REGS_S2MM_INTRENB, reg_value);
}

/*******************************************************************************
 * PCDMA_S2MM_disable_irq()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
void PCDMA_S2MM_disable_irq
(
   PCDMA_instance_t  * this_PCDMA,
   uint32_t irq_type
)
{
        uint32_t reg_value;

        HAL_ASSERT( this_PCDMA != NULL_INSTANCE );

        reg_value = HAL_get_32bit_reg( this_PCDMA->base_address, \
                                          COREAXI4PROTOCONV_REGS_S2MM_INTRENB);
        reg_value &= ~irq_type;

        HAL_set_32bit_reg( this_PCDMA->base_address, \
                               COREAXI4PROTOCONV_REGS_S2MM_INTRENB, reg_value);
}

/*******************************************************************************
 * PCDMA_S2MM_get_int_src()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
uint32_t PCDMA_S2MM_get_int_src
(
   PCDMA_instance_t  * this_PCDMA
)
{
    return (HAL_get_32bit_reg( this_PCDMA->base_address, \
                                    COREAXI4PROTOCONV_REGS_S2MM_INTRSRC));
}

/*******************************************************************************
 * PCDMA_S2MM_clr_int_src()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
void PCDMA_S2MM_clr_int_src
(
   PCDMA_instance_t  * this_PCDMA,
   uint32_t irq_type
)
{

        HAL_ASSERT( this_PCDMA != NULL_INSTANCE );

        HAL_set_32bit_reg( this_PCDMA->base_address, \
                                COREAXI4PROTOCONV_REGS_S2MM_INTRSRC, irq_type);
}

#ifdef S2MM_UNDEF_BSTLEN

/*******************************************************************************
 * PCDMA_S2MM_get_len()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
uint32_t PCDMA_S2MM_get_len
(
   PCDMA_instance_t  * this_PCDMA
)
{
    return (HAL_get_32bit_reg( this_PCDMA->base_address, \
                                    COREAXI4PROTOCONV_REGS_S2MM_LEN));
}

#endif
#endif

#ifndef MM2S_ENABLE
/*******************************************************************************
 * PCDMA_MM2S_configure()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
void PCDMA_MM2S_configure
(
    PCDMA_instance_t  * this_PCDMA,
    uint32_t xfr_size,
    uint64_t start_add,
    uint16_t cmd_id,
    uint8_t burst_type
)
{

        HAL_ASSERT( this_PCDMA != NULL_INSTANCE );
        HAL_ASSERT( xfr_size != 0 );
        HAL_ASSERT( cmd_id <= 0x3ff );
        HAL_ASSERT( burst_type <= 0x1 );

        if((xfr_size != 0) && (cmd_id <= 0x3ff) && (burst_type <= 0x1) )
        {
            /* Configure transfer size for the total number of bytes to be
             * transmitted.
             */
            HAL_set_32bit_reg( this_PCDMA->base_address, \
                                  COREAXI4PROTOCONV_REGS_MM2S_LEN, xfr_size );

            /* Configure the lower 32-bits address of the AXI4 memory mapped
             * interface.
             */
            HAL_set_32bit_reg( this_PCDMA->base_address, \
                                  COREAXI4PROTOCONV_REGS_MM2S_ADDR0, \
                                        (uint32_t) (0xffffffff & start_add));

            /* Configure the upper 32-bits address of the AXI4 memory mapped
             * interface.
             */
            HAL_set_32bit_reg( this_PCDMA->base_address, \
                                  COREAXI4PROTOCONV_REGS_MM2S_ADDR1, \
                                   (uint32_t) (0xffffffff & (start_add >> 32)));

            /* Configure the burst type */
            HAL_set_32bit_reg( this_PCDMA->base_address, \
                                  COREAXI4PROTOCONV_REGS_MM2S_CTRL, \
                                         ((cmd_id << 16) | (burst_type << 1)));
        }

}

/*******************************************************************************
 * PCDMA_MM2S_start_transfer()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
void PCDMA_MM2S_start_transfer
(
    PCDMA_instance_t  * this_PCDMA
)
{
            uint32_t control_reg_value;

            HAL_ASSERT( this_PCDMA != NULL_INSTANCE );

            if( this_PCDMA != NULL_INSTANCE )
            {
               control_reg_value = HAL_get_32bit_reg( this_PCDMA->base_address,\
                                         COREAXI4PROTOCONV_REGS_MM2S_CTRL);

               HAL_set_32bit_reg( this_PCDMA->base_address, \
                                       COREAXI4PROTOCONV_REGS_MM2S_CTRL, \
                                               ( control_reg_value | 0x1) );
            }
}

/*******************************************************************************
 * PCDMA_MM2S_get_status()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
uint32_t PCDMA_MM2S_get_status
(
    PCDMA_instance_t  * this_PCDMA
)
{
            uint32_t status;

            HAL_ASSERT( this_PCDMA != NULL_INSTANCE );

            if( this_PCDMA != NULL_INSTANCE )
            {
                status = HAL_get_32bit_reg( this_PCDMA->base_address, \
                                              COREAXI4PROTOCONV_REGS_MM2S_STS);
            }
            return status;
}

/*******************************************************************************
 * PCDMA_MM2S_enable_irq()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
void PCDMA_MM2S_enable_irq
(
   PCDMA_instance_t  * this_PCDMA,
   uint32_t irq_type
)
{
        uint32_t reg_value;

        HAL_ASSERT( this_PCDMA != NULL_INSTANCE );

        reg_value = HAL_get_32bit_reg( this_PCDMA->base_address, \
                                          COREAXI4PROTOCONV_REGS_MM2S_INTRENB);
        reg_value |= irq_type;

        HAL_set_32bit_reg( this_PCDMA->base_address, \
                                COREAXI4PROTOCONV_REGS_MM2S_INTRENB, reg_value);
}

/*******************************************************************************
 * PCDMA_MM2S_disable_irq()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
void PCDMA_MM2S_disable_irq
(
   PCDMA_instance_t  * this_PCDMA,
   uint32_t irq_type
)
{
        uint32_t reg_value;

        HAL_ASSERT( this_PCDMA != NULL_INSTANCE );

        reg_value = HAL_get_32bit_reg( this_PCDMA->base_address, \
                                  COREAXI4PROTOCONV_REGS_MM2S_INTRENB);
        reg_value &= ~irq_type;

        HAL_set_32bit_reg( this_PCDMA->base_address, \
                               COREAXI4PROTOCONV_REGS_MM2S_INTRENB, reg_value);
}

/*******************************************************************************
 * PCDMA_MM2S_get_int_src()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
uint32_t PCDMA_MM2S_get_int_src
(
   PCDMA_instance_t  * this_PCDMA
)
{
    return (HAL_get_32bit_reg( this_PCDMA->base_address, \
                                    COREAXI4PROTOCONV_REGS_MM2S_INTRSRC));
}

/*******************************************************************************
 * PCDMA_MM2S_clr_int_src()
 * See "core_axi4protoconv.h" file for details of how to use this function.
 */
void PCDMA_MM2S_clr_int_src
(
   PCDMA_instance_t  * this_PCDMA,
   uint32_t irq_type
)
{

        HAL_ASSERT( this_PCDMA != NULL_INSTANCE );

        HAL_set_32bit_reg( this_PCDMA->base_address, \
                               COREAXI4PROTOCONV_REGS_MM2S_INTRSRC, irq_type);
}

#endif

#ifdef __cplusplus
}
#endif
