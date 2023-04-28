/***************************************************************************//**
 * \file mtb_ubm.c
 * \version 0.5
 *
 * \brief
 * Provides the UBM middleware API implementation.
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

#include "mtb_ubm.h"
#include <string.h>
#include "mtb_ubm_controller.h"
#include "mtb_ubm_bootloader.h"
#include "mtb_ubm_io.h"

/*******************************************************************************
*            Internal API
*******************************************************************************/
static bool validate_ubm_config(const mtb_stc_ubm_backplane_cfg_t* config,
                                const mtb_stc_ubm_backplane_control_signals_t* signals);


/*******************************************************************************
* Function Name: mtb_ubm_init
****************************************************************************//**
*
*  Initializes the UBM middleware.
* 
* \warning
*  <b> Next UBM Controller commands are not implemented: </b>
*  - <b> Change Count </b>
*  - <b> DFC Status and Control Descriptor Index </b>
*  - <b> DFC Status and Control Descriptor </b>
*  - <b> Features write command </b>
*
* \param config
*  Pointer to the backplane configuration structure.
*
* \param signals
*  Pointer to the signals configuration structure.
*
* \param context
*  Pointer to the context structure.
*
* \return
*  "false" if initialization succeeds, and "true" if it fails.
*
*******************************************************************************/
bool mtb_ubm_init(const mtb_stc_ubm_backplane_cfg_t* config,
                 const mtb_stc_ubm_backplane_control_signals_t* signals,
                 mtb_stc_ubm_context_t* context)
{
    bool result = true;

    result = validate_ubm_config(config, signals);

    if (!result)
    {
        result = mtb_ubm_io_timer_init(context);
    }

    if (!result)
    {
        /* Save Silicon Identity and Version data */
        (void)memcpy(&context->silicon_identity, &config->silicon_identity, sizeof(mtb_stc_ubm_siv_t));

        /* Save Backplane Info data */
        (void)memcpy(&context->backplane_info, &config->backplane_info, sizeof(mtb_stc_ubm_bp_info_t));

        /* Save Capabilities data */
        (void)memcpy(&context->capabilities, &config->capabilities, sizeof(mtb_stc_ubm_capabilities_t));

        for (uint8_t i = 0; i < MTB_UBM_HFC_NUM; i++)
        {
            context->ctrl[i].index = i;

            /* Initialize DFC signals */
            result = mtb_ubm_io_dfc_init(&context->ctrl[i], &signals->dfc_io[i],
                            &config->hfc_routing[i], config);

            if (!result)
            {
                result = mtb_ubm_io_hfc_init(&context->ctrl[i], &signals->hfc_io[i], config);
            }

            if (!result)
            {
                /* Set Operational State to INITIALIZING */
                mtb_ubm_set_op_state(&context->ctrl[i], MTB_UBM_OP_STATE_INITIALIZING);
            }
            else
            {
                /* Initialization error */
                break;
            }
        }

        if (!result)
        {
            /* Initialize FRU */
            result = (CY_EM_EEPROM_SUCCESS != Cy_Em_EEPROM_Init(config->fru_config, &context->fru_context));
        }

        if (!result)
        {
            for (uint8_t i = 0; i < MTB_UBM_HFC_NUM; i++)
            {
                /* Setup and enable UBM Controller I2C slave interface */
                result = mtb_ubm_i2c_init(context, &context->ctrl[i], config, signals);

                if (!result)
                {
                    /* Set Operational State to READY */
                    mtb_ubm_set_op_state(&context->ctrl[i], MTB_UBM_OP_STATE_READY);
                }
                else
                {
                    /* Initialization error */
                    break;
                }
            }
        }

        if (!result)
        {
            uint8_t ram_fru[MTB_UBM_FRU_SIZE];
            cy_en_em_eeprom_status_t status = CY_EM_EEPROM_WRITE_FAIL;

            (void)memset(ram_fru, 0x00L, MTB_UBM_FRU_SIZE);

            mtb_ubm_fru_init_common_header(&ram_fru[MTB_UBM_FRU_CH_ADDR]);
            mtb_ubm_fru_init_overview_area(config, &ram_fru[MTB_UBM_FRU_CH_LEN]);
            mtb_ubm_fru_init_route_information_descriptor(MTB_UBM_ROUTES_NUM, config->hfc_routing, &ram_fru[MTB_UBM_FRU_RI_H_LEN + MTB_UBM_FRU_OA_LEN + MTB_UBM_FRU_CH_LEN]);
            mtb_ubm_fru_init_route_information_header(MTB_UBM_ROUTES_NUM, &ram_fru[MTB_UBM_FRU_OA_LEN + MTB_UBM_FRU_CH_LEN]);

            status = Cy_Em_EEPROM_Write(0U, ram_fru, MTB_UBM_FRU_SIZE, &context->fru_context);

            if (status != CY_EM_EEPROM_SUCCESS)
            {
                result = true;
            }
        }

        if (!result)
        {
            context->starting_slot = config->starting_slot;
        }
        #if(MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
        if (!result)
        {
            (void)mtb_ubm_init_flash_geometry(context);
        }
        #endif /*MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED*/
    }


    return result;
}


/*******************************************************************************
* Function Name: validate_ubm_config
****************************************************************************//**
*
*  Checks if UBM backplane configuration parameters are valid.
*
* \param config
*  Pointer to the backplane configuration structure.
*
* \param signals
*  Pointer to the signals configuration structure.
*
* \return
*  "false" if parameters are valid, and "true" if not.
*
*******************************************************************************/
static bool validate_ubm_config(const mtb_stc_ubm_backplane_cfg_t* config,
                                const mtb_stc_ubm_backplane_control_signals_t* signals)
{
    bool result = true;

    for (uint8_t i = 0U; i < MTB_UBM_DFC_NUM; i++)
    {
        if ( ((signals->dfc_io[i].prsnt != NC) && (config->capabilities.prsnt_reported == true )) &&
             ((signals->dfc_io[i].ifdet != NC) && (config->capabilities.ifdet_reported == true )) &&
             ((signals->dfc_io[i].ifdet2 != NC) && (config->capabilities.ifdet2_reported == true )) )
        {
            result = false;
        }
    }

    return result;
}

