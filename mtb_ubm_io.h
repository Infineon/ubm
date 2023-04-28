/***************************************************************************//**
 * \file mtb_ubm_io.h
 * \version 0.5
 *
 * \brief
 * Provides the UBM middleware I/O functions.
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

#ifndef MTB_UBM_IO_H
#define MTB_UBM_IO_H

#include "mtb_ubm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Period for 100uS is 100, but time required for interrupt handling this time
 * is reducing, practically it was measured to require lower period value. */
#define MTB_UBM_TIMER_PERIOD_100US              (85U)
#define MTB_UBM_FREQUNCY_1M                     (1000000U)

#define MTB_UBM_PRSNT_SHIFT                     (0U)
#define MTB_UBM_IFDET_SHIFT                     (1U)
#define MTB_UBM_IFDET2_SHIFT                    (2U)

#define MTB_UBM_I2C_RESET_DELAY                 (10000U)
#define MTB_UBM_FULL_RESET_DELAY                (50000U)

bool mtb_ubm_io_dfc_init(mtb_stc_ubm_controller_t* ctrl_context,
                         const mtb_stc_ubm_dfc_signals_t* signals,
                         const mtb_stc_ubm_routing_t* routing_info,
                         const mtb_stc_ubm_backplane_cfg_t* config);
bool mtb_ubm_io_hfc_init(mtb_stc_ubm_controller_t* ctrl_context,
                         const mtb_stc_ubm_hfc_signals_t* signals,
                         const mtb_stc_ubm_backplane_cfg_t* config);
bool mtb_ubm_io_timer_init(mtb_stc_ubm_context_t* context);


#ifdef __cplusplus
}
#endif

#endif /* MTB_UBM_IO_H */
