/***************************************************************************//**
 * \file mtb_ubm_bootloader.c
 * \version 0.5
 *
 * \brief
 * Provides the UBM middleware bootloader API implementation.
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

#include "mtb_ubm_bootloader.h"

#if(MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
 /*******************************************************************************
 * Function Name: mtb_ubm_init_flash_geometry
 ****************************************************************************//**
 *
 *  Initializes the UBM flash area to Update mode.
 *
 * \param context
 *  Pointer to the context structure.
 *
 * \return
 *  "true" if initialization succeeds, and "false" if it fails.
 *
 *******************************************************************************/
bool mtb_ubm_init_flash_geometry(mtb_stc_ubm_context_t* context)
{
    bool status = false;

    /* Calculate address of start upgrade slot */
    uint32_t start_addr = MTB_UBM_UPGRADE_AREA_START + MTB_UBM_UPGRADE_AREA_SIZE;

    /* Check generated define */
    if ((start_addr != 0U) && (MTB_UBM_UPGRADE_AREA_SIZE != 0U) && \
                              ((start_addr % CY_FLASH_SIZEOF_ROW) == 0))
    {
        context->flash_layout.addr_start_upgrade_area = start_addr;
        context->flash_layout.size_upgrade_area = MTB_UBM_UPGRADE_AREA_SIZE;
        
        status = true;
    }

    (void)cyhal_flash_init(&context->flash_layout.flash_obj);

    context->flash_layout.offset_row_buffer = 0U;

    return status;
}
#endif /* MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED */

/*END FILE*/
