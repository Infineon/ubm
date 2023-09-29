/***************************************************************************//**
 * \file mtb_ubm_ifc.c
 * \version 1.0
 *
 * \brief
 * Provides functions for the UBM middleware interfaces.
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

#include "cyhal_scb_common.h"
#include "cyhal_i2c.h"
#include "mtb_ubm_ifc.h"
#include "mtb_ubm_fru.h"
#include "mtb_ubm_controller.h"
#include <string.h>


/** Read the FRU transaction packet length. */
#define FRU_READ_TRANSACTION_LEN            (1U)

/** Read the Command transaction packet length. */
#define CMD_READ_TRANSACTION_LEN            (2U)

/** Read the PMDT Command transaction packet length. */
#define PMDT_READ_TRANSACTION_LEN           (4U)

/** Invalid data for the read buffer. */
#define INVALID_DATA                        (0xFF)

/** Invalid controller index. */
#define INVALID_CTRL_INDEX                  (0xFFU)


#define SCD_SES_BYTE0                       (0U)
#define SCD_SES_BYTE1                       (1U)
#define SCD_SES_BYTE2                       (2U)
#define SCD_SES_BYTE3                       (3U)
#define SCD_CMD_SAS_ELEM_STS_CODE_Pos       (0U)
#define SCD_CMD_SAS_ELEM_STS_CODE_Msk       (0x0FU)
#define SCD_CMD_SAS_RESET_SWAP_Pos          (4U)
#define SCD_CMD_SAS_RESET_SWAP_Msk          (0x10U)
#define SCD_CMD_SAS_DISABLE_Pos             (5U)
#define SCD_CMD_SAS_DISABLE_Msk             (0x20U)
#define SCD_CMD_SAS_PRDFAIL_Pos             (6U)
#define SCD_CMD_SAS_PRDFAIL_Msk             (0x40U)
#define SCD_CMD_SAS_SELECT_Pos              (7U)
#define SCD_CMD_SAS_SELECT_Msk              (0x80U)
#define SCD_CMD_SAS_RR_ABORT_Pos            (0U)
#define SCD_CMD_SAS_RR_ABORT_Msk            (0x01U)
#define SCD_CMD_SAS_RR_Pos                  (1U)
#define SCD_CMD_SAS_RR_Msk                  (0x02U)
#define SCD_CMD_SAS_IN_FAILED_ARRAY_Pos     (2U)
#define SCD_CMD_SAS_IN_FAILED_ARRAY_Msk     (0x04U)
#define SCD_CMD_SAS_IN_CRIT_ARRAY_Pos       (3U)
#define SCD_CMD_SAS_IN_CRIT_ARRAY_Msk       (0x08U)
#define SCD_CMD_SAS_CONS_CHECK_Pos          (4U)
#define SCD_CMD_SAS_CONS_CHECK_Msk          (0x10U)
#define SCD_CMD_SAS_HOT_SPARE_Pos           (5U)
#define SCD_CMD_SAS_HOT_SPARE_Msk           (0x20U)
#define SCD_CMD_SAS_RSVD_DEVICE_Pos         (6U)
#define SCD_CMD_SAS_RSVD_DEVICE_Msk         (0x40U)
#define SCD_CMD_SAS_OK_Pos                  (7U)
#define SCD_CMD_SAS_OK_Msk                  (0x80U)
#define SCD_CMD_SAS_REPORT_Pos              (0U)
#define SCD_CMD_SAS_REPORT_Msk              (0x01U)
#define SCD_CMD_SAS_IDENT_Pos               (1U)
#define SCD_CMD_SAS_IDENT_Msk               (0x02U)
#define SCD_CMD_SAS_REMOVE_Pos              (2U)
#define SCD_CMD_SAS_REMOVE_Msk              (0x04U)
#define SCD_CMD_SAS_INSERT_Pos              (3U)
#define SCD_CMD_SAS_INSERT_Msk              (0x08U)
#define SCD_CMD_SAS_MISSING_Pos             (4U)
#define SCD_CMD_SAS_MISSING_Msk             (0x10U)
#define SCD_CMD_SAS_ENCL_BYP_B_Pos          (4U)
#define SCD_CMD_SAS_ENCL_BYP_B_Msk          (0x10U)
#define SCD_CMD_SAS_ENCL_BYP_A_Pos          (5U)
#define SCD_CMD_SAS_ENCL_BYP_A_Msk          (0x20U)
#define SCD_CMD_SAS_DO_NOT_REMOVE_Pos       (6U)
#define SCD_CMD_SAS_DO_NOT_REMOVE_Msk       (0x40U)
#define SCD_CMD_SAS_ACTIVE_Pos              (7U)
#define SCD_CMD_SAS_ACTIVE_Msk              (0x80U)
#define SCD_CMD_SAS_APP_BYP_A_Pos           (7U)
#define SCD_CMD_SAS_APP_BYP_A_Msk           (0x80U)
#define SCD_CMD_SAS_DEV_BYP_B_Pos           (0U)
#define SCD_CMD_SAS_DEV_BYP_B_Msk           (0x01U)
#define SCD_CMD_SAS_DEV_BYP_A_Pos           (1U)
#define SCD_CMD_SAS_DEV_BYP_A_Msk           (0x02U)
#define SCD_CMD_SAS_BYP_B_Pos               (2U)
#define SCD_CMD_SAS_BYP_B_Msk               (0x04U)
#define SCD_CMD_SAS_BYP_A_Pos               (3U)
#define SCD_CMD_SAS_BYP_A_Msk               (0x08U)
#define SCD_CMD_SAS_DEVICE_OFF_Pos          (4U)
#define SCD_CMD_SAS_DEVICE_OFF_Msk          (0x10U)
#define SCD_CMD_SAS_FAULT_REQUEST_Pos       (5U)
#define SCD_CMD_SAS_FAULT_REQUEST_Msk       (0x20U)
#define SCD_CMD_SAS_FAULT_SENSED_Pos        (6U)
#define SCD_CMD_SAS_FAULT_SENSED_Msk        (0x40U)
#define SCD_CMD_SAS_APP_BYP_B_Pos           (7U)
#define SCD_CMD_SAS_APP_BYP_B_Msk           (0x80U)



/*******************************************************************************
*            Internal API
*******************************************************************************/
static cyhal_i2c_command_rsp_t handle_i2c_address(void* callback_arg,
                                                  cyhal_i2c_addr_event_t event,
                                                  uint8_t address);
static void handle_i2c_event(void* callback_arg, cyhal_i2c_event_t event);
static cy_rslt_t configure_write_buffer(mtb_stc_ubm_hfc_t* hfc_context);
static cy_rslt_t configure_read_buffer(mtb_stc_ubm_hfc_t* hfc_context);


/*******************************************************************************
* Function Name: mtb_ubm_ifc_i2c_init
****************************************************************************//**
*
*  The function initializes the UBM middleware I2C instance specified
*  by the arguments.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_index
*  The index of the UBM controller context structure.
*
* \param ubm_backplane_control_signals
*  The pointer to the configuration structure of backplane control signals.
*
* \return
*  See \ref mtb_en_ubm_status_t.
*
*******************************************************************************/
mtb_en_ubm_status_t mtb_ubm_ifc_i2c_init(mtb_stc_ubm_context_t* ubm_context,
                                         uint32_t hfc_index,
                                         const mtb_stc_ubm_backplane_control_signals_t* ubm_backplane_control_signals)
{
    cy_rslt_t result;
    mtb_en_ubm_status_t status;
    mtb_stc_ubm_hfc_t* hfc_context = &ubm_context->hfc[hfc_index];

    /* Initialize the I2C slave resources */
    result = cyhal_i2c_init(&hfc_context->i2c.scb_i2c_obj,
                            ubm_backplane_control_signals->hfc_io[hfc_context->index].sda,
                            ubm_backplane_control_signals->hfc_io[hfc_context->index].scl,
                            NULL);

    if (CY_RSLT_SUCCESS == result)
    {
        cyhal_i2c_adv_cfg_t i2c_slave_config =
        {
            { CYHAL_I2C_MODE_SLAVE, MTB_UBM_I2C_SLAVE_FRU_ADDRESS, MTB_UBM_I2C_SLAVE_FREQUENCY },
            MTB_UBM_I2C_SLAVE_ADDRESS_MASK,
            true
        };

        /* Configure the I2C slave */
        result = cyhal_i2c_configure_adv(&hfc_context->i2c.scb_i2c_obj,
                                         &i2c_slave_config);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        /* Configure the buffer to which the master writes data to */
        result = configure_write_buffer(hfc_context);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        /* Configure the buffer from which the master reads data from */
        result = configure_read_buffer(hfc_context);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        /* Register I2C slave event callback */
        hfc_context->i2c.cb_arg.ubm_context = ubm_context;
        hfc_context->i2c.cb_arg.hfc_index = (uint8_t)hfc_index;
        cyhal_i2c_register_callback(&hfc_context->i2c.scb_i2c_obj,
                                    handle_i2c_event,
                                    &hfc_context->i2c.cb_arg);

        /* Enable the I2C events */
        cyhal_i2c_enable_event(&hfc_context->i2c.scb_i2c_obj, CYHAL_I2C_SLAVE_WR_CMPLT_EVENT,
                               MTB_UBM_I2C_SLAVE_IRQ_PRIORITY, true);

        cyhal_i2c_enable_event(&hfc_context->i2c.scb_i2c_obj, CYHAL_I2C_SLAVE_RD_CMPLT_EVENT,
                               MTB_UBM_I2C_SLAVE_IRQ_PRIORITY, true);

        cyhal_i2c_enable_event(&hfc_context->i2c.scb_i2c_obj, CYHAL_I2C_SLAVE_ERR_EVENT,
                               MTB_UBM_I2C_SLAVE_IRQ_PRIORITY, true);

        /* Register the I2C slave address callback */
        cyhal_i2c_register_address_callback(&hfc_context->i2c.scb_i2c_obj,
                                            handle_i2c_address,
                                            &hfc_context->i2c.cb_arg);

        /* Enable the address event */
        cyhal_i2c_enable_address_event(&hfc_context->i2c.scb_i2c_obj, CYHAL_I2C_ADDR_MATCH_EVENT,
                                       MTB_UBM_I2C_SLAVE_IRQ_PRIORITY, true);

        status = MTB_UBM_STATUS_SUCCESS;
    }
    else
    {
        /* Release the allocated resources */
        cyhal_i2c_free(&hfc_context->i2c.scb_i2c_obj);

        status = MTB_UBM_STATUS_2WIRE_IO_CONFIG_ERR;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_i2c_address
****************************************************************************//**
*
*  Handles the I2C address received event.
*
* \param callback_arg
*  A callback argument, the pointer to the context structures.
*
* \param event
*  The event to be handled.
*
* \param address
*  The received I2C address.
*
* \return
*  "CYHAL_I2C_CMD_ACK" - if the received address matches the configured address,
*  "CYHAL_I2C_CMD_NAK" - otherwise.
*
*******************************************************************************/
static cyhal_i2c_command_rsp_t handle_i2c_address(void* callback_arg, cyhal_i2c_addr_event_t event, uint8_t address)
{
    cyhal_i2c_command_rsp_t response = CYHAL_I2C_CMD_NAK;

    /* Check the slave address match event */
    if (0U != ((uint32_t)CYHAL_I2C_ADDR_MATCH_EVENT & (uint32_t)event))
    {
        CY_MISRA_DEVIATE_LINE('MISRA C-2012 Rule 11.5', 'Pointer passed via callback_arg is aligned correcly, as contains address of a known object.');
        const mtb_stc_ubm_i2c_callback_arg_t* arg = (mtb_stc_ubm_i2c_callback_arg_t*)callback_arg;
        mtb_stc_ubm_context_t* ubm_context = arg->ubm_context;
        mtb_stc_ubm_hfc_t* hfc_context = &ubm_context->hfc[arg->hfc_index];

        hfc_context->selected_ctrl_index = INVALID_CTRL_INDEX;
        for (uint32_t ctrl_index = 0U; ctrl_index < hfc_context->ctrl_count; ctrl_index++)
        {
            if ((hfc_context->ctrl_slave_address[ctrl_index] >> 1U) == address)
            {
                hfc_context->selected_ctrl_index = (uint8_t)ctrl_index;
                break;
            }
        }

        if ((MTB_UBM_I2C_SLAVE_FRU_ADDRESS == address) ||
            (INVALID_CTRL_INDEX != hfc_context->selected_ctrl_index))
        {
            hfc_context->selected_slave_address = address;
            response = CYHAL_I2C_CMD_ACK;
        }
    }

    return response;
}


/*******************************************************************************
* Function Name: handle_i2c_event
****************************************************************************//**
*
*  Handles I2C slave events.
*
* \param callback_arg
*  A callback argument, the pointer to the context structures.
*
* \param event
*  The event to be handled.
*
*******************************************************************************/
static void handle_i2c_event(void* callback_arg, cyhal_i2c_event_t event)
{
    /* Check for errors */
    if (0U == ((uint32_t)CYHAL_I2C_SLAVE_ERR_EVENT & (uint32_t)event))
    {
        CY_MISRA_DEVIATE_LINE('MISRA C-2012 Rule 11.5', 'Pointer passed via callback_arg is aligned correcly, as contains address of a known object.');
        const mtb_stc_ubm_i2c_callback_arg_t* arg = (mtb_stc_ubm_i2c_callback_arg_t*)callback_arg;
        mtb_stc_ubm_context_t* ubm_context = arg->ubm_context;
        mtb_stc_ubm_hfc_t* hfc_context = &ubm_context->hfc[arg->hfc_index];

        /* Check the write complete event */
        if (0U != ((uint32_t)CYHAL_I2C_SLAVE_WR_CMPLT_EVENT & (uint32_t)event))
        {
            hfc_context->i2c.write_data_length = cyhal_i2c_slave_readable(&hfc_context->i2c.scb_i2c_obj);

            if (MTB_UBM_I2C_SLAVE_FRU_ADDRESS == hfc_context->selected_slave_address)
            {
                if (FRU_READ_TRANSACTION_LEN == hfc_context->i2c.write_data_length)
                {
                    /* The FRU read request */
                    hfc_context->i2c.read_request = true;
                    mtb_ubm_fru_handle_request(ubm_context, hfc_context);

                    /* Prepare the read buffer for the read transaction */
                    (void)cyhal_i2c_slave_config_read_buffer(&hfc_context->i2c.scb_i2c_obj,
                                                             hfc_context->i2c.read_buffer,
                                                             (uint16_t)hfc_context->i2c.read_data_length);
                }
                else
                {
                    /* Unsupported request */
                }
            }
            else
            {
                mtb_ubm_cmd_t cmd = mtb_ubm_get_packet_command(hfc_context);

                if (((MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER == cmd) &&
                     (PMDT_READ_TRANSACTION_LEN == hfc_context->i2c.write_data_length)) ||
                    ((MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER != cmd) &&
                     (CMD_READ_TRANSACTION_LEN == hfc_context->i2c.write_data_length)))
                {
                    /* Controller read request */
                    hfc_context->i2c.read_request = true;
                    mtb_ubm_controller_handle_request(ubm_context, hfc_context);

                    /* Prepare the read buffer for the read transaction */
                    (void)cyhal_i2c_slave_config_read_buffer(&hfc_context->i2c.scb_i2c_obj,
                                                             hfc_context->i2c.read_buffer,
                                                             (uint16_t)hfc_context->i2c.read_data_length);
                }
                else if (((MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER == cmd) &&
                          (PMDT_READ_TRANSACTION_LEN < hfc_context->i2c.write_data_length)) ||
                         ((MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER != cmd) &&
                          (CMD_READ_TRANSACTION_LEN < hfc_context->i2c.write_data_length)))
                {
                    /* Controller write request */
                    hfc_context->i2c.read_request = false;
                    mtb_ubm_controller_handle_request(ubm_context, hfc_context);
                }
                else
                {
                    /* Unexpected data length */
                }
            }

            /* Clear and configure the write buffer */
            (void)configure_write_buffer(hfc_context);
        }

        /* Check the read complete event */
        if (0U != ((uint32_t)CYHAL_I2C_SLAVE_RD_CMPLT_EVENT & (uint32_t)event))
        {
            /* Clear and configure the read buffer */
            (void)configure_read_buffer(hfc_context);
        }
    }
}


/*******************************************************************************
* Function Name: configure_write_buffer
****************************************************************************//**
*
*  Configures the buffer to which the master writes data to.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return
*  The result of the cyhal_i2c_slave_config_write_buffer request.
*
*******************************************************************************/
static cy_rslt_t configure_write_buffer(mtb_stc_ubm_hfc_t* hfc_context)
{
    cy_rslt_t result;

    hfc_context->i2c.write_data_length = 0U;
    (void)memset(hfc_context->i2c.write_buffer, 0, MTB_UBM_I2C_WRITE_BUFFER_SIZE);
    result = cyhal_i2c_slave_config_write_buffer(&hfc_context->i2c.scb_i2c_obj,
                                                 hfc_context->i2c.write_buffer,
                                                 MTB_UBM_I2C_WRITE_BUFFER_SIZE);
    return result;
}


/*******************************************************************************
* Function Name: configure_read_buffer
****************************************************************************//**
*
*  Configures the buffer from which the master reads data from.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return
*  The result of the cyhal_i2c_slave_config_read_buffer request.
*
*******************************************************************************/
static cy_rslt_t configure_read_buffer(mtb_stc_ubm_hfc_t* hfc_context)
{
    cy_rslt_t result;

    hfc_context->i2c.read_data_length = 0U;
    (void)memset(hfc_context->i2c.read_buffer, INVALID_DATA, MTB_UBM_I2C_READ_BUFFER_SIZE);
    result = cyhal_i2c_slave_config_read_buffer(&hfc_context->i2c.scb_i2c_obj,
                                                hfc_context->i2c.read_buffer,
                                                MTB_UBM_I2C_READ_BUFFER_SIZE);
    return result;
}


#if (MTB_UBM_SES_CB_ACTIVE)
/*******************************************************************************
* Function Name: mtb_ubm_ifc_ses_app_event
****************************************************************************//**
*
*  Calls the APP event if the SES register has changed.
*
* \param app_callback
*
*
* \param dfc_index
*  The index of the DFC record changed.
*
* \param ses_control_data
*  The pointer to the received SES Array Device Slot Control Element data.
*
* \param ses_status_data
*  The pointer to the SES Array Device Slot Status Element data.
*
*******************************************************************************/
void mtb_ubm_ifc_ses_app_event(mtb_ubm_ses_app_cb_t app_callback,
                               uint8_t dfc_index,
                               const uint8_t* ses_control_data,
                               uint8_t* ses_status_data)
{
    if (NULL != app_callback)
    {
        mtb_stc_ubm_ses_app_cb_context_t app_context;
        mtb_stc_ubm_ses_control_t ses_control;
        mtb_stc_ubm_ses_status_t ses_status;
        uint8_t byte;
        bool result;

        (void)memset(&ses_control, 0, sizeof(mtb_stc_ubm_ses_control_t));
        (void)memset(&ses_status, 0, sizeof(mtb_stc_ubm_ses_status_t));

        byte = ses_control_data[SCD_SES_BYTE0];
        ses_control.reset_swap = (0U != (byte & SCD_CMD_SAS_RESET_SWAP_Msk));
        ses_control.disable = (0U != (byte & SCD_CMD_SAS_DISABLE_Msk));
        ses_control.predicted_failure = (0U != (byte & SCD_CMD_SAS_PRDFAIL_Msk));
        ses_control.select = (0U != (byte & SCD_CMD_SAS_SELECT_Msk));

        byte = ses_control_data[SCD_SES_BYTE1];
        ses_control.request_rebuild_remap_aborted = (0U != (byte & SCD_CMD_SAS_RR_ABORT_Msk));
        ses_control.request_rebuild_remap = (0U != (byte & SCD_CMD_SAS_RR_Msk));
        ses_control.request_in_failed_array = (0U != (byte & SCD_CMD_SAS_IN_FAILED_ARRAY_Msk));
        ses_control.request_in_critical_array = (0U != (byte & SCD_CMD_SAS_IN_CRIT_ARRAY_Msk));
        ses_control.request_consistency_check = (0U != (byte & SCD_CMD_SAS_CONS_CHECK_Msk));
        ses_control.request_hot_spare = (0U != (byte & SCD_CMD_SAS_HOT_SPARE_Msk));
        ses_control.request_reserved_device = (0U != (byte & SCD_CMD_SAS_RSVD_DEVICE_Msk));
        ses_control.request_ok = (0U != (byte & SCD_CMD_SAS_OK_Msk));

        byte = ses_control_data[SCD_SES_BYTE2];
        ses_control.request_identify = (0U != (byte & SCD_CMD_SAS_IDENT_Msk));
        ses_control.request_remove = (0U != (byte & SCD_CMD_SAS_REMOVE_Msk));
        ses_control.request_insert = (0U != (byte & SCD_CMD_SAS_INSERT_Msk));
        ses_control.request_missing = (0U != (byte & SCD_CMD_SAS_MISSING_Msk));
        ses_control.do_not_remove = (0U != (byte & SCD_CMD_SAS_DO_NOT_REMOVE_Msk));
        ses_control.request_active = (0U != (byte & SCD_CMD_SAS_ACTIVE_Msk));

        byte = ses_control_data[SCD_SES_BYTE3];
        ses_control.enable_bypass_b = (0U != (byte & SCD_CMD_SAS_BYP_B_Msk));
        ses_control.enable_bypass_a = (0U != (byte & SCD_CMD_SAS_BYP_A_Msk));
        ses_control.device_off = (0U != (byte & SCD_CMD_SAS_DEVICE_OFF_Msk));
        ses_control.request_fault = (0U != (byte & SCD_CMD_SAS_FAULT_REQUEST_Msk));

        app_context.dfc_index = dfc_index;
        app_context.ses_control = &ses_control;
        app_context.ses_status = &ses_status;
        result = app_callback(&app_context);

        if (result)
        {
            ses_status_data[SCD_SES_BYTE0] = (ses_status.element_status_code << SCD_CMD_SAS_ELEM_STS_CODE_Pos) &
                                             SCD_CMD_SAS_ELEM_STS_CODE_Msk;
            ses_status_data[SCD_SES_BYTE0] |= (uint8_t)ses_status.swap << SCD_CMD_SAS_RESET_SWAP_Pos;
            ses_status_data[SCD_SES_BYTE0] |= (uint8_t)ses_status.disabled << SCD_CMD_SAS_DISABLE_Pos;
            ses_status_data[SCD_SES_BYTE0] |= (uint8_t)ses_status.predicted_failure << SCD_CMD_SAS_PRDFAIL_Pos;

            ses_status_data[SCD_SES_BYTE1] = (uint8_t)ses_status.rebuild_remap_abort << SCD_CMD_SAS_RR_ABORT_Pos;
            ses_status_data[SCD_SES_BYTE1] |= (uint8_t)ses_status.rebuild_remap << SCD_CMD_SAS_RR_Pos;
            ses_status_data[SCD_SES_BYTE1] |= (uint8_t)ses_status.in_failed_array << SCD_CMD_SAS_IN_FAILED_ARRAY_Pos;
            ses_status_data[SCD_SES_BYTE1] |= (uint8_t)ses_status.in_critical_array << SCD_CMD_SAS_IN_CRIT_ARRAY_Pos;
            ses_status_data[SCD_SES_BYTE1] |= (uint8_t)ses_status.consistency_check << SCD_CMD_SAS_CONS_CHECK_Pos;
            ses_status_data[SCD_SES_BYTE1] |= (uint8_t)ses_status.hot_spare << SCD_CMD_SAS_HOT_SPARE_Pos;
            ses_status_data[SCD_SES_BYTE1] |= (uint8_t)ses_status.reserved_device << SCD_CMD_SAS_RSVD_DEVICE_Pos;
            ses_status_data[SCD_SES_BYTE1] |= (uint8_t)ses_status.ok << SCD_CMD_SAS_OK_Pos;

            ses_status_data[SCD_SES_BYTE2] = (uint8_t)ses_status.report << SCD_CMD_SAS_REPORT_Pos;
            ses_status_data[SCD_SES_BYTE2] |= (uint8_t)ses_status.identify << SCD_CMD_SAS_IDENT_Pos;
            ses_status_data[SCD_SES_BYTE2] |= (uint8_t)ses_status.rmv << SCD_CMD_SAS_REMOVE_Pos;
            ses_status_data[SCD_SES_BYTE2] |= (uint8_t)ses_status.ready_to_insert << SCD_CMD_SAS_INSERT_Pos;
            ses_status_data[SCD_SES_BYTE2] |= (uint8_t)ses_status.enclosure_bypassed_b << SCD_CMD_SAS_ENCL_BYP_B_Pos;
            ses_status_data[SCD_SES_BYTE2] |= (uint8_t)ses_status.enclosure_bypassed_a << SCD_CMD_SAS_ENCL_BYP_A_Pos;
            ses_status_data[SCD_SES_BYTE2] |= (uint8_t)ses_status.do_not_remove << SCD_CMD_SAS_DO_NOT_REMOVE_Pos;
            ses_status_data[SCD_SES_BYTE2] |= (uint8_t)ses_status.app_client_bypassed_a << SCD_CMD_SAS_APP_BYP_A_Pos;

            ses_status_data[SCD_SES_BYTE3] = (uint8_t)ses_status.device_bypassed_b << SCD_CMD_SAS_DEV_BYP_B_Pos;
            ses_status_data[SCD_SES_BYTE3] |= (uint8_t)ses_status.device_bypassed_a << SCD_CMD_SAS_DEV_BYP_A_Pos;
            ses_status_data[SCD_SES_BYTE3] |= (uint8_t)ses_status.bypassed_b << SCD_CMD_SAS_BYP_B_Pos;
            ses_status_data[SCD_SES_BYTE3] |= (uint8_t)ses_status.bypassed_a << SCD_CMD_SAS_BYP_A_Pos;
            ses_status_data[SCD_SES_BYTE3] |= (uint8_t)ses_status.device_off << SCD_CMD_SAS_DEVICE_OFF_Pos;
            ses_status_data[SCD_SES_BYTE3] |= (uint8_t)ses_status.fault_requested << SCD_CMD_SAS_FAULT_REQUEST_Pos;
            ses_status_data[SCD_SES_BYTE3] |= (uint8_t)ses_status.fault_sensed << SCD_CMD_SAS_FAULT_SENSED_Pos;
            ses_status_data[SCD_SES_BYTE3] |= (uint8_t)ses_status.app_client_bypassed_b << SCD_CMD_SAS_APP_BYP_B_Pos;
        }
    }
}


#endif // MTB_UBM_SES_CB_ACTIVE
