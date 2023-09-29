/***************************************************************************//**
 * \file mtb_ubm_bootloader.c
 * \version 1.0
 *
 * \brief
 * Provides API implementation for the UBM middleware bootloader.
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

#if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
/*******************************************************************************
* Function Name: mtb_ubm_init_flash_geometry
****************************************************************************//**
*
*  Initializes the UBM flash area to Update mode.
*
* \param context
*  The pointer to the context structure.
*
* \return
*  See \ref mtb_en_ubm_status_t.
*
*******************************************************************************/
mtb_en_ubm_status_t mtb_ubm_init_flash_geometry(mtb_stc_ubm_context_t* context)
{
    mtb_en_ubm_status_t status = MTB_UBM_STATUS_SUCCESS;

    /* Check the configured macros */
    if ((0U == MTB_UBM_UPGRADE_AREA_SIZE) || (0U != (MTB_UBM_UPGRADE_IMAGE_START_ADDRESS % CY_FLASH_SIZEOF_ROW)))
    {
        CY_MISRA_DEVIATE_LINE('MISRA C-2012 Rule 14.3', 'Validating invariant expression, as it is defined on the application level.');
        status = MTB_UBM_STATUS_BOOTLOADER_START_ADDR_ERR;
    }

    if (MTB_UBM_STATUS_SUCCESS == status)
    {
        cy_rslt_t result;

        result = cyhal_flash_init(&context->flash_layout.flash_obj);

        if (CY_RSLT_SUCCESS == result)
        {
            context->flash_layout.offset_row_buffer = 0U;
        }
        else
        {
            status = MTB_UBM_STATUS_BOOTLOADER_FLASH_INIT_ERR;
        }
    }

    return status;
}


#endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */

/*END FILE*/
