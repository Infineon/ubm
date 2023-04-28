/***************************************************************************//**
 * \file mtb_ubm_ifc.h
 * \version 0.5
 *
 * \brief
 * Provides the UBM middleware interfaces functions.
 *
 *******************************************************************************
 * (c) (2021-2023), Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 * This software, including source code, documentation and related materials
 * ("Software") is owned by Cypress Semiconductor Corporation or one of its
 * affiliates ("Cypress") and is protected by and subject to worldwide patent
 * protection (United States and foreign), United States copyright laws and
 * international treaty provisions. Therefore, you may use this Software only
 * as provided in the license agreement accompanying the software package from
 * which you obtained this Software ("EULA").
 *
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software source
 * code solely for use in connection with Cypress's integrated circuit products.
 * Any reproduction, modification, translation, compilation, or representation
 * of this Software except as specified above is prohibited without the express
 * written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer of such
 * system or application assumes all risk of such use and in doing so agrees to
 * indemnify Cypress against all liability.
 ******************************************************************************/

#ifndef MTB_UBM_IFC_H
#define MTB_UBM_IFC_H

#include "mtb_ubm_types.h"

#ifdef __cplusplus
extern "C" {
#endif


/** I2C address. This is the address for UBM FRU. */
#define MTB_UBM_I2C_SLAVE_FRU_ADDRESS           (0xAEU >> 1)

/** I2C address mask. */
#define MTB_UBM_I2C_SLAVE_ADDRESS_MASK          (0x00U)

/** I2C speed. As this is a slave, i2c speed is primarily dictated by the master.*/
#define MTB_UBM_I2C_SLAVE_FREQUENCY             (400000U)

/* I2C slave interrupt priority */
#define MTB_UBM_I2C_SLAVE_IRQ_PRIORITY          (6U)

bool mtb_ubm_i2c_init(mtb_stc_ubm_context_t* ubm_context,
                      mtb_stc_ubm_controller_t* ctrl_context,
                      const mtb_stc_ubm_backplane_cfg_t* ubm_backplane_config,
                      const mtb_stc_ubm_backplane_control_signals_t* ubm_backplane_control_signals);

#ifdef __cplusplus
}
#endif

#endif /* MTB_UBM_IFC_H */
