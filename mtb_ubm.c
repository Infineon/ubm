/***************************************************************************//**
 * \file mtb_ubm.c
 * \version 1.0
 *
 * \brief
 * Provides API implementation for the UBM middleware.
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


/** Invalid index value. */
#define UBM_INVALID_INDEX      (0xFFU)
/** Invalid address value. */
#define UBM_INVALID_ADDRESS    (0xFFU)


/*******************************************************************************
*            Internal API
*******************************************************************************/
static mtb_en_ubm_status_t validate_ubm_config(const mtb_stc_ubm_backplane_cfg_t* config,
                                               const mtb_stc_ubm_backplane_control_signals_t* signals);
static void process_port_route_information(const mtb_stc_ubm_backplane_cfg_t* config,
                                           mtb_stc_ubm_context_t* ubm_context);
static uint8_t init_pcie_reset_field(const mtb_stc_ubm_capabilities_t* capabilities,
                                     const mtb_stc_ubm_features_t* features);


/*******************************************************************************
* Function Name: mtb_ubm_init
****************************************************************************//**
*
*  Initializes the UBM middleware.
*
* \note If an error occurs, the state of MCU peripherals in use may be undefined.
*
* \param config
*  The pointer to the backplane configuration structure.
*
* \param signals
*  The pointer to the signals configuration structure.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \return
*  See \ref mtb_en_ubm_status_t.
*
*******************************************************************************/
mtb_en_ubm_status_t mtb_ubm_init(const mtb_stc_ubm_backplane_cfg_t* config,
                                 const mtb_stc_ubm_backplane_control_signals_t* signals,
                                 mtb_stc_ubm_context_t* ubm_context)
{
    mtb_en_ubm_status_t ubm_status;

    ubm_status = validate_ubm_config(config, signals);

    if (MTB_UBM_STATUS_SUCCESS == ubm_status)
    {
        ubm_status = mtb_ubm_io_timer_init(ubm_context);
    }

    #if (MTB_UBM_SES_CB_ACTIVE)
    if (MTB_UBM_STATUS_SUCCESS == ubm_status)
    {
        ubm_context->ses_control_cb = config->ses_event_handler;
    }
    #endif /* MTB_UBM_SES_CB_ACTIVE */

    if (MTB_UBM_STATUS_SUCCESS == ubm_status)
    {
        bool status_success = true;

        /* Save the numbers of HFCs, DFCs, and Routes */
        ubm_context->num_of_hfc = config->num_of_hfc;
        ubm_context->num_of_dfc = config->num_of_dfc;
        ubm_context->num_of_routes = config->num_of_routes;

        /* Save Silicon Identity and Version data */
        (void)memcpy(&ubm_context->silicon_identity, &config->silicon_identity, sizeof(mtb_stc_ubm_siv_t));

        /* Save Backplane Info data */
        (void)memcpy(&ubm_context->backplane_info, &config->backplane_info, sizeof(mtb_stc_ubm_bp_info_t));

        /* Save Capabilities data */
        (void)memcpy(&ubm_context->capabilities, &config->capabilities, sizeof(mtb_stc_ubm_capabilities_t));

        /* Initialize DFC I/O signals */
        for (uint32_t dfc_index = 0U;
             (dfc_index < config->num_of_dfc) && status_success;
             dfc_index++)
        {
            (void)memset(&ubm_context->dfc[dfc_index], 0x00, sizeof(mtb_stc_ubm_dfc_t));
            ubm_context->dfc[dfc_index].index = (uint8_t)dfc_index;
            ubm_status = mtb_ubm_io_dfc_init(ubm_context, dfc_index, &signals->dfc_io[dfc_index], config);
            status_success = (MTB_UBM_STATUS_SUCCESS == ubm_status);
        }

        /* Initialize HFC I/O signals */
        for (uint32_t hfc_index = 0U;
             ((hfc_index < config->num_of_hfc) && status_success);
             hfc_index++)
        {
            (void)memset(&ubm_context->hfc[hfc_index], 0x00, sizeof(mtb_stc_ubm_hfc_t));
            ubm_context->hfc[hfc_index].index = (uint8_t)hfc_index;
            ubm_status = mtb_ubm_io_hfc_init(ubm_context, hfc_index, &signals->hfc_io[hfc_index]);
            status_success = (MTB_UBM_STATUS_SUCCESS == ubm_status);
        }

        if (MTB_UBM_STATUS_SUCCESS == ubm_status)
        {
            mtb_ubm_set_op_state(ubm_context, MTB_UBM_OP_STATE_INITIALIZING);
            process_port_route_information(config, ubm_context);

            /* Initialize FRU */
            uint8_t ram_fru[MTB_UBM_FRU_SIZE];
            cy_en_em_eeprom_status_t eeprom_status;

            eeprom_status = Cy_Em_EEPROM_Init(config->fru_config, &ubm_context->fru_context);

            if (CY_EM_EEPROM_SUCCESS != eeprom_status)
            {
                ubm_status = MTB_UBM_STATUS_FRU_EEPROM_ERR;
            }

            if (MTB_UBM_STATUS_SUCCESS == ubm_status)
            {
                (void)memset(ram_fru, 0x00, MTB_UBM_FRU_SIZE);

                mtb_ubm_fru_init_common_header(&ram_fru[MTB_UBM_FRU_CH_ADDR]);
                mtb_ubm_fru_init_overview_area(config, &ram_fru[MTB_UBM_FRU_OA_ADDR]);
                mtb_ubm_fru_init_ri_descriptor(config->num_of_routes,
                                               config->route_information,
                                               &ram_fru[MTB_UBM_FRU_RI_D_ADDR]);
                mtb_ubm_fru_init_ri_header(config->num_of_routes, &ram_fru[MTB_UBM_FRU_RI_H_ADDR]);

                uint8_t ram_fru_cmp[MTB_UBM_FRU_SIZE];

                eeprom_status = Cy_Em_EEPROM_Read(MTB_UBM_FRU_START_ADDRESS,
                                                  &ram_fru_cmp,
                                                  MTB_UBM_FRU_SIZE,
                                                  &ubm_context->fru_context);

                if (((CY_EM_EEPROM_SUCCESS == eeprom_status) || (CY_EM_EEPROM_REDUNDANT_COPY_USED == eeprom_status)) &&
                    (memcmp(&ram_fru_cmp, &ram_fru, MTB_UBM_FRU_SIZE) != 0))
                {
                    eeprom_status = Cy_Em_EEPROM_Write(MTB_UBM_FRU_START_ADDRESS,
                                                       ram_fru,
                                                       MTB_UBM_FRU_SIZE,
                                                       &ubm_context->fru_context);
                }

                if ((CY_EM_EEPROM_SUCCESS != eeprom_status) && (CY_EM_EEPROM_REDUNDANT_COPY_USED != eeprom_status))
                {
                    ubm_status = MTB_UBM_STATUS_FRU_EEPROM_ERR;
                }
            }
        }

        #if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
        if (MTB_UBM_STATUS_SUCCESS == ubm_status)
        {
            ubm_status = mtb_ubm_init_flash_geometry(ubm_context);
        }
        #endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */

        for (uint32_t hfc_index = 0U; ((hfc_index < config->num_of_hfc) && status_success); hfc_index++)
        {
            /* Set up and enable the UBM Controller I2C slave interface */
            ubm_status = mtb_ubm_ifc_i2c_init(ubm_context, hfc_index, signals);
            status_success = (MTB_UBM_STATUS_SUCCESS == ubm_status);
        }

        if (MTB_UBM_STATUS_SUCCESS == ubm_status)
        {
            ubm_context->starting_slot = config->starting_slot;
            for (uint32_t ctrl_index = 0U; ctrl_index < ubm_context->num_of_ctrls; ctrl_index++)
            {
                mtb_ubm_update_change_count(ubm_context,
                                            &ubm_context->ctrl[ctrl_index],
                                            NULL,
                                            MTB_UBM_CC_SOURCE_CTRL_RESET);
            }

            mtb_ubm_set_op_state(ubm_context, MTB_UBM_OP_STATE_READY);

            for (uint32_t dfc_index = 0U; dfc_index < ubm_context->num_of_dfc; dfc_index++)
            {
                const mtb_stc_ubm_dfc_t* dfc = &ubm_context->dfc[dfc_index];

                if ((NC != dfc->dfc_io.prsnt) || (NC != dfc->dfc_io.ifdet) || (NC != dfc->dfc_io.ifdet2))
                {
                    for (uint32_t ctrl_index = 0U; ctrl_index < dfc->ctrl_count; ctrl_index++)
                    {
                        mtb_ubm_update_change_count(ubm_context,
                                                    &ubm_context->ctrl[dfc->ctrl_list[ctrl_index]],
                                                    dfc,
                                                    MTB_UBM_CC_SOURCE_DRIVE_TYPE);
                    }
                }
            }
        }
    }

    return ubm_status;
}


/*******************************************************************************
* Function Name: validate_ubm_config
****************************************************************************//**
*
*  Checks if the UBM backplane configuration parameters are valid.
*
* \param config
*  The pointer to the backplane configuration structure.
*
* \param signals
*  The pointer to the signals configuration structure.
*
* \return
*  See \ref mtb_en_ubm_status_t.
*
*******************************************************************************/
static mtb_en_ubm_status_t validate_ubm_config(const mtb_stc_ubm_backplane_cfg_t* config,
                                               const mtb_stc_ubm_backplane_control_signals_t* signals)
{
    mtb_en_ubm_status_t status = MTB_UBM_STATUS_SUCCESS;
    const uint8_t* config_struct = NULL;
    uint32_t sum_of_bytes = 0U;

    if ((0U == config->num_of_hfc) || (config->num_of_hfc > MTB_UBM_HFC_MAX_NUM))
    {
        status = MTB_UBM_STATUS_HFC_NUM_ERR;
    }
    else if ((0U == config->num_of_dfc) || (config->num_of_dfc > MTB_UBM_DFC_MAX_NUM))
    {
        status = MTB_UBM_STATUS_DFC_NUM_ERR;
    }
    else if ((0U == config->num_of_routes) || (config->num_of_routes > MTB_UBM_ROUTES_MAX_NUM))
    {
        status = MTB_UBM_STATUS_ROUTES_NUM_ERR;
    }
    else
    {
        /* Keep initial status MTB_UBM_STATUS_SUCCESS */
    }

    if (MTB_UBM_STATUS_SUCCESS == status)
    {
        for (uint32_t hfc_index = 0U; hfc_index < config->num_of_hfc; hfc_index++)
        {
            config_struct = (const uint8_t*)&signals->hfc_io[hfc_index];
            sum_of_bytes = 0U;

            for (uint32_t byte = 0U; byte < sizeof(signals->hfc_io[hfc_index]); byte++)
            {
                sum_of_bytes += config_struct[byte];
            }

            if (0U == sum_of_bytes)
            {
                status = MTB_UBM_STATUS_HFC_IO_CONF_ERR;
                break;
            }
        }
    }

    if (MTB_UBM_STATUS_SUCCESS == status)
    {
        for (uint32_t dfc_index = 0U; dfc_index < config->num_of_dfc; dfc_index++)
        {
            config_struct = (const uint8_t*)&signals->dfc_io[dfc_index];
            sum_of_bytes = 0U;

            for (uint32_t byte = 0U; byte < sizeof(signals->dfc_io[dfc_index]); byte++)
            {
                sum_of_bytes += config_struct[byte];
            }

            if (0U == sum_of_bytes)
            {
                status = MTB_UBM_STATUS_DFC_IO_CONF_ERR;
                break;
            }
        }
    }

    if (MTB_UBM_STATUS_SUCCESS == status)
    {
        for (uint32_t route_index = 0U; route_index < config->num_of_routes; route_index++)
        {
            config_struct = (const uint8_t*)&config->route_information[route_index];
            sum_of_bytes = 0U;

            for (uint32_t byte = 0U; byte < sizeof(config->route_information[route_index]); byte++)
            {
                sum_of_bytes += config_struct[byte];
            }

            if (0U == sum_of_bytes)
            {
                status = MTB_UBM_STATUS_ROUTES_CONF_ERR;
                break;
            }
        }
    }

    if (MTB_UBM_STATUS_SUCCESS == status)
    {
        for (uint32_t dfc_index = 0U; dfc_index < config->num_of_dfc; dfc_index++)
        {
            if (config->capabilities.prsnt_reported && (NC == signals->dfc_io[dfc_index].prsnt))
            {
                status = MTB_UBM_STATUS_CAP_PRSNT_ERR;
            }
            else if (config->capabilities.ifdet_reported && (NC == signals->dfc_io[dfc_index].ifdet))
            {
                status = MTB_UBM_STATUS_CAP_IFDET_ERR;
            }
            else if (config->capabilities.ifdet2_reported && (NC == signals->dfc_io[dfc_index].ifdet2))
            {
                status = MTB_UBM_STATUS_CAP_IFDET2_ERR;
            }
            else
            {
                /* Keep initial status MTB_UBM_STATUS_SUCCESS */
            }

            if (MTB_UBM_STATUS_SUCCESS != status)
            {
                break;
            }
        }
    }

    return status;
}


/*******************************************************************************
* Function Name: process_port_route_information
****************************************************************************//**
*
*  Initializes the internal context structure based on Port Route Information.
*
* \param config
*  The pointer to the backplane configuration structure.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
*******************************************************************************/
static void process_port_route_information(const mtb_stc_ubm_backplane_cfg_t* config,
                                           mtb_stc_ubm_context_t* ubm_context)
{
    const mtb_stc_ubm_routing_t* route_config;
    mtb_stc_ubm_hfc_t* hfc_context;
    mtb_stc_ubm_dfc_t* dfc_context;
    mtb_stc_ubm_controller_t* ctrl_context;
    bool controller_not_found;

    (void)memset(ubm_context->ctrl, 0xFF, sizeof(mtb_stc_ubm_controller_t) * MTB_UBM_CTRLS_MAX_NUM);
    ubm_context->num_of_ctrls = 0U;

    for (uint32_t route_num = 0U; route_num < config->num_of_routes; route_num++)
    {
        route_config = &config->route_information[route_num];
        controller_not_found = true;

        for (uint32_t ctrl_index = 0U; (ctrl_index < MTB_UBM_CTRLS_MAX_NUM) && controller_not_found; ctrl_index++)
        {
            hfc_context = &ubm_context->hfc[route_config->hfc_identifier];
            dfc_context = &ubm_context->dfc[route_config->drive_connector_idx];
            ctrl_context = &ubm_context->ctrl[ctrl_index];

            /* Look for existing and matching controller instance */
            if ((route_config->hfc_identifier == ctrl_context->hfc_index) &&
                (route_config->ubm_ctrl_slave_addr == ctrl_context->slave_address))
            {
                dfc_context->ctrl_list[dfc_context->ctrl_count] = (uint8_t)ctrl_index;
                dfc_context->ctrl_count++;
                ctrl_context->dfc_list[ctrl_context->dfc_count] = route_config->drive_connector_idx;
                ctrl_context->scd[route_config->drive_connector_idx].dfc_change_count =
                    (config->capabilities.dfc_change_count_supported) ? 1U : 0U;
                ctrl_context->scd[route_config->drive_connector_idx].drive_type =
                    &dfc_context->drive_type_installed;
                ctrl_context->scd[route_config->drive_connector_idx].bifurcate_port =
                    config->bifurcate_port;
                ctrl_context->scd[route_config->drive_connector_idx].pcie_reset =
                    init_pcie_reset_field(&ubm_context->capabilities, &ctrl_context->features);
                ctrl_context->dfc_count++;
                controller_not_found = false;
            }
            /* Look for empty controller instance */
            else if ((UBM_INVALID_INDEX == ctrl_context->hfc_index) &&
                     (UBM_INVALID_ADDRESS == ctrl_context->slave_address))
            {
                (void)memcpy(&ctrl_context->features,
                             &config->overview_area->ubm_controller_features,
                             sizeof(mtb_stc_ubm_features_t));
                dfc_context->ctrl_list[dfc_context->ctrl_count] = (uint8_t)ctrl_index;
                dfc_context->ctrl_count++;
                ctrl_context->dfc_count = 0U;
                ctrl_context->hfc_index = route_config->hfc_identifier;
                ctrl_context->slave_address = route_config->ubm_ctrl_slave_addr;
                ctrl_context->dfc_list[ctrl_context->dfc_count] = route_config->drive_connector_idx;
                ctrl_context->domain = route_config->domain;
                (void)memset(ctrl_context->scd,
                             0x00,
                             sizeof(mtb_stc_ubm_status_and_control_descriptor_t) * MTB_UBM_DFC_MAX_NUM);
                (void)memset(&ctrl_context->change_count, 0x00, sizeof(mtb_stc_ubm_change_count_t));
                ctrl_context->scd[route_config->drive_connector_idx].dfc_change_count =
                    (config->capabilities.dfc_change_count_supported) ? 1U : 0U;
                ctrl_context->scd[route_config->drive_connector_idx].drive_type =
                    &dfc_context->drive_type_installed;
                ctrl_context->scd[route_config->drive_connector_idx].bifurcate_port =
                    config->bifurcate_port;
                ctrl_context->scd[route_config->drive_connector_idx].pcie_reset =
                    init_pcie_reset_field(&ubm_context->capabilities, &ctrl_context->features);
                ctrl_context->dfc_count++;
                ctrl_context->legacy_mode =
                    config->overview_area->ubm_controller_features.cprsnt_legacy_mode;
                ctrl_context->port_type = route_config->port_type;
                ctrl_context->last_command_status = MTB_UBM_LC_STS_FAILED;
                hfc_context->ctrl_list[hfc_context->ctrl_count] = (uint8_t)ctrl_index;
                hfc_context->ctrl_slave_address[hfc_context->ctrl_count] = ctrl_context->slave_address;
                hfc_context->ctrl_count++;
                ubm_context->num_of_ctrls++;
                controller_not_found = false;
            }
            else
            {
                /* Found existing but not matching controller instance. Skip it. */
            }
        }
    }
}


/*******************************************************************************
* Function Name: init_pcie_reset_field
****************************************************************************//**
*
*  Initializes the PCIe Reset field of the Status and Control Descriptor.
*
* \param capabilities
*  The pointer to the UBM Capabilities configuration.
*
* \param features
*  The pointer to the UBM Features configuration.
*
* * \return
*  The initial value of the PCIe Reset field.
*
*******************************************************************************/
static uint8_t init_pcie_reset_field(const mtb_stc_ubm_capabilities_t* capabilities,
                                     const mtb_stc_ubm_features_t* features)
{
    uint8_t result = MTB_UBM_PCIE_RESET_FIELD_NOP;

    if (capabilities->pcie_reset_control)
    {
        if (capabilities->clock_routing)
        {
            if (capabilities->perst_override_supported)
            {
                if ((MTB_UBM_DFC_PERST_OVERRIDE_NO_OVERRIDE == features->perst_management_override) ||
                    (MTB_UBM_DFC_PERST_OVERRIDE_MANAGED == features->perst_management_override))
                {
                    result = MTB_UBM_PCIE_RESET_FIELD_HOLD;
                }
                else if (MTB_UBM_DFC_PERST_OVERRIDE_AUTO == features->perst_management_override)
                {
                    result = MTB_UBM_PCIE_RESET_FIELD_NOP;
                }
                else
                {
                    /* Unexpected perst_management_override value.
                     * Keep initial value MTB_UBM_PCIE_RESET_FIELD_NOP
                     */
                }
            }
            else
            {
                result = MTB_UBM_PCIE_RESET_FIELD_HOLD;
            }
        }
        else
        {
            if (capabilities->perst_override_supported)
            {
                if ((MTB_UBM_DFC_PERST_OVERRIDE_NO_OVERRIDE == features->perst_management_override) ||
                    (MTB_UBM_DFC_PERST_OVERRIDE_AUTO == features->perst_management_override))
                {
                    result = MTB_UBM_PCIE_RESET_FIELD_NOP;
                }
                else if (MTB_UBM_DFC_PERST_OVERRIDE_MANAGED == features->perst_management_override)
                {
                    result = MTB_UBM_PCIE_RESET_FIELD_HOLD;
                }
                else
                {
                    /* Unexpected perst_management_override value.
                     * Keep initial value MTB_UBM_PCIE_RESET_FIELD_NOP
                     */
                }
            }
            else
            {
                result = MTB_UBM_PCIE_RESET_FIELD_NOP;
            }
        }
    }

    return result;
}
