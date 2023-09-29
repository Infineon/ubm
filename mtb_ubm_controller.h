/***************************************************************************//**
 * \file mtb_ubm_controller.h
 * \version 1.0
 *
 * \brief
 * Provides common API declarations for the UBM controller.
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

#if !defined(MTB_UBM_CONTROLLER_H)
#define MTB_UBM_CONTROLLER_H

#include "mtb_ubm_types.h"

#ifdef __cplusplus
extern "C" {
#endif

mtb_ubm_cmd_t mtb_ubm_get_packet_command(const mtb_stc_ubm_hfc_t* hfc_context);
void mtb_ubm_controller_handle_request(mtb_stc_ubm_context_t* ubm_context,
                                       mtb_stc_ubm_hfc_t* hfc_context);
mtb_en_ubm_op_state_t mtb_ubm_get_op_state(const mtb_stc_ubm_context_t* ubm_context);
void mtb_ubm_set_op_state(mtb_stc_ubm_context_t* ubm_context,
                          mtb_en_ubm_op_state_t state);
void mtb_ubm_update_change_count(const mtb_stc_ubm_context_t* ubm_context,
                                 mtb_stc_ubm_controller_t* ctrl_context,
                                 const mtb_stc_ubm_dfc_t* dfc_context,
                                 mtb_en_ubm_change_count_source_t source);
mtb_en_ubm_lc_sts_t mtb_ubm_process_pcie_reset_request(const mtb_stc_ubm_context_t* ubm_context,
                                                       const mtb_stc_ubm_hfc_t* hfc_context,
                                                       const mtb_stc_ubm_controller_t* ctrl_context,
                                                       mtb_stc_ubm_dfc_t* dfc_context);

#ifdef __cplusplus
}
#endif

#endif /* MTB_UBM_CONTROLLER_H */
