/***************************************************************************//**
 * \file mtb_ubm_ifc.c
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

#include "cyhal_scb_common.h"
#include "cyhal_i2c.h"
#include "mtb_ubm_ifc.h"
#include "mtb_ubm_fru.h"
#include "mtb_ubm_controller.h"
#include <string.h>

CY_MISRA_DEVIATE_BLOCK_START('MISRA C-2012 Rule 10.1', 6, \
'Checked manually. Intentional expressions of type enum are used as an operand to the arithmetic operator "&".');

CY_MISRA_DEVIATE_BLOCK_START('MISRA C-2012 Rule 10.4', 1, \
'Checked manually. Intentional arithmetic conversion on enum type.');

CY_MISRA_DEVIATE_BLOCK_START('MISRA C-2012 Rule 10.5', 1, \
'Checked manually. Intentional type cast to an enum type.');


/** Read FRU transaction packet length. */
#define FRU_READ_TRANSACTION_LEN    (1U)

/** Read Command transaction packet length. */
#define CMD_READ_TRANSACTION_LEN    (2U)

/** Read PMDT Command transaction packet length. */
#define PMDT_READ_TRANSACTION_LEN   (4U)

/** Invalid data for read buffer. */
#define INVALID_DATA                (0xFF)


/*******************************************************************************
*            Internal API
*******************************************************************************/
static cyhal_i2c_command_rsp_t handle_i2c_address(void* callback_arg,
                                                  cyhal_i2c_addr_event_t event,
                                                  uint8_t address);
static void handle_i2c_event(void* callback_arg, cyhal_i2c_event_t event);
static cy_rslt_t configure_write_buffer(mtb_stc_ubm_controller_t* ctrl_context);
static cy_rslt_t configure_read_buffer(mtb_stc_ubm_controller_t* ctrl_context);


/*******************************************************************************
* Function Name: mtb_ubm_i2c_init
****************************************************************************//**
*
*  The function initializes the UBM middleware I2C instance specified
*  by the arguments.
*
* \param ubm_context
*  Pointer to the UBM context structure.
*
* \param ctrl_context
*  Pointer to the UBM controller context structure.
*
* \param ubm_backplane_config
*  Pointer to backplane configuration structure.
*
* \param ubm_backplane_control_signals
*  Pointer to backplane control signals configuration structure.
*
* \return
*  "false" if initialization succeeds, and "true" if it fails.
*
*******************************************************************************/
bool mtb_ubm_i2c_init(mtb_stc_ubm_context_t* ubm_context,
                      mtb_stc_ubm_controller_t* ctrl_context,
                      const mtb_stc_ubm_backplane_cfg_t* ubm_backplane_config,
                      const mtb_stc_ubm_backplane_control_signals_t* ubm_backplane_control_signals)
{
    cy_rslt_t result;
    bool status;

    /* Initialize I2C slave resources */
    result = cyhal_i2c_init(&ctrl_context->i2c.scb_i2c_obj,
                            ubm_backplane_control_signals->hfc_io[ctrl_context->index].sda,
                            ubm_backplane_control_signals->hfc_io[ctrl_context->index].scl,
                            NULL);

    if (CY_RSLT_SUCCESS == result)
    {
        cyhal_i2c_adv_cfg_t i2c_slave_config =
        {
            { CYHAL_I2C_MODE_SLAVE, MTB_UBM_I2C_SLAVE_FRU_ADDRESS, MTB_UBM_I2C_SLAVE_FREQUENCY },
            MTB_UBM_I2C_SLAVE_ADDRESS_MASK,
            true
        };

        /* Configure I2C slave */
        result = cyhal_i2c_configure_adv(&ctrl_context->i2c.scb_i2c_obj,
                                         &i2c_slave_config);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        /* Configure buffer to which master writes data to */
        result = configure_write_buffer(ctrl_context);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        /* Configure buffer from which master reads data from */
        result = configure_read_buffer(ctrl_context);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        cyhal_i2c_event_t events;

        /* Register I2C slave event callback */
        ctrl_context->i2c.cb_arg.ubm_context = ubm_context;
        ctrl_context->i2c.cb_arg.ctrl_context = ctrl_context;
        cyhal_i2c_register_callback(&ctrl_context->i2c.scb_i2c_obj,
                                    handle_i2c_event,
                                    &ctrl_context->i2c.cb_arg);

        /* Enable I2C events */
        events = (cyhal_i2c_event_t)(CYHAL_I2C_SLAVE_WR_CMPLT_EVENT | \
                                     CYHAL_I2C_SLAVE_RD_CMPLT_EVENT | \
                                     CYHAL_I2C_SLAVE_ERR_EVENT);
        cyhal_i2c_enable_event(&ctrl_context->i2c.scb_i2c_obj, events,
                               MTB_UBM_I2C_SLAVE_IRQ_PRIORITY, true);

        /* Register I2C slave address callback */
        cyhal_i2c_register_address_callback(&ctrl_context->i2c.scb_i2c_obj,
                                            handle_i2c_address,
                                            &ctrl_context->i2c.cb_arg);

        /* Enable address event */
        cyhal_i2c_enable_address_event(&ctrl_context->i2c.scb_i2c_obj, CYHAL_I2C_ADDR_MATCH_EVENT,
                                       MTB_UBM_I2C_SLAVE_IRQ_PRIORITY, true);

        /* Save configured controller address */
        ctrl_context->i2c.controller_address =
            ubm_backplane_config->hfc_routing[ctrl_context->index].ubm_ctrl_slave_addr;

        status = false;
    }
    else
    {
        /* Release the allocated resources */
        cyhal_i2c_free(&ctrl_context->i2c.scb_i2c_obj);

        status = true;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_i2c_address
****************************************************************************//**
*
*  Handles I2C address received event.
*
* \param callback_arg
*  Callback argument, pointer to the context structures.
*
* \param event
*  Event to be handled.
* 
* \param address
*  Received I2C address.
*
* \return
*  "CYHAL_I2C_CMD_ACK" if the received address matches one that is configured, and
*  "CYHAL_I2C_CMD_NAK" otherwise.
*
*******************************************************************************/
static cyhal_i2c_command_rsp_t handle_i2c_address(void* callback_arg, cyhal_i2c_addr_event_t event, uint8_t address)
{
    cyhal_i2c_command_rsp_t response = CYHAL_I2C_CMD_NAK;

    /* Check slave address match event */
    if (0UL != (CYHAL_I2C_ADDR_MATCH_EVENT & event))
    {
        mtb_stc_ubm_i2c_callback_arg_t* arg = (mtb_stc_ubm_i2c_callback_arg_t*)callback_arg;
        mtb_stc_ubm_controller_t* ctrl_context = arg->ctrl_context;

        /* Check if received address matches FRU or controller address */
        if ((MTB_UBM_I2C_SLAVE_FRU_ADDRESS == address) ||
            (ctrl_context->i2c.controller_address == address))
        {
            ctrl_context->i2c.received_address = address;
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
*  Callback argument, pointer to the context structures.
*
* \param event
*  Event to be handled.
*
*******************************************************************************/
static void handle_i2c_event(void* callback_arg, cyhal_i2c_event_t event)
{
    /* Check for errors */
    if (0UL == (CYHAL_I2C_SLAVE_ERR_EVENT & event))
    {
        CY_MISRA_DEVIATE_LINE('MISRA C-2012 Rule 11.5', 'Cast callback_arg parameter is safe, as it was intentional saved previously.');
        mtb_stc_ubm_i2c_callback_arg_t* arg = (mtb_stc_ubm_i2c_callback_arg_t*)callback_arg;
        mtb_stc_ubm_controller_t* ctrl_context = arg->ctrl_context;

        /* Check write complete event */
        if (0UL != (CYHAL_I2C_SLAVE_WR_CMPLT_EVENT & event))
        {
            mtb_stc_ubm_context_t* ubm_context = arg->ubm_context;

            ctrl_context->i2c.write_data_length =
                Cy_SCB_I2C_SlaveGetWriteTransferCount(ctrl_context->i2c.scb_i2c_obj.base,
                                                      &ctrl_context->i2c.scb_i2c_obj.context);

            if (MTB_UBM_I2C_SLAVE_FRU_ADDRESS == ctrl_context->i2c.received_address)
            {
                if (FRU_READ_TRANSACTION_LEN == ctrl_context->i2c.write_data_length)
                {
                    /* FRU read request */
                    ctrl_context->i2c.read_request = true;
                    mtb_ubm_fru_handle_request(ubm_context, ctrl_context);

                    /* Prepare read buffer for the read transaction */
                    (void)cyhal_i2c_slave_config_read_buffer(&ctrl_context->i2c.scb_i2c_obj,
                                                             ctrl_context->i2c.read_buffer,
                                                             (uint16_t)ctrl_context->i2c.read_data_length);
                }
                else
                {
                    /* Unsupported request */
                }
            }
            else if (ctrl_context->i2c.controller_address == ctrl_context->i2c.received_address)
            {
                mtb_ubm_cmd_t cmd = mtb_ubm_get_packet_command(ctrl_context);

                if (((MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER == cmd) &&
                     (PMDT_READ_TRANSACTION_LEN == ctrl_context->i2c.write_data_length)) ||
                    ((MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER != cmd) &&
                     (CMD_READ_TRANSACTION_LEN == ctrl_context->i2c.write_data_length)))
                {
                    /* Controller read request */
                    ctrl_context->i2c.read_request = true;
                    mtb_ubm_controller_handle_request(ubm_context, ctrl_context);

                    /* Prepare read buffer for the read transaction */
                    (void)cyhal_i2c_slave_config_read_buffer(&ctrl_context->i2c.scb_i2c_obj,
                                                             ctrl_context->i2c.read_buffer,
                                                             (uint16_t)ctrl_context->i2c.read_data_length);
                }
                else if (((MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER == cmd) &&
                          (PMDT_READ_TRANSACTION_LEN < ctrl_context->i2c.write_data_length)) ||
                         ((MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER != cmd) &&
                          (CMD_READ_TRANSACTION_LEN < ctrl_context->i2c.write_data_length)))
                {
                    /* Controller write request */
                    ctrl_context->i2c.read_request = false;
                    mtb_ubm_controller_handle_request(ubm_context, ctrl_context);
                }
                else
                {
                    /* Unexpected data length */
                }
            }
            else
            {
                /* Unexpected address */
            }

            /* Clear and configure write buffer */
            (void)configure_write_buffer(ctrl_context);
        }

        /* Check read complete event */
        if (0UL != (CYHAL_I2C_SLAVE_RD_CMPLT_EVENT & event))
        {
            /* Clear and configure read buffer */
            (void)configure_read_buffer(ctrl_context);
        }
    }
}


/*******************************************************************************
* Function Name: configure_write_buffer
****************************************************************************//**
*
*  Configures buffer to which master writes data to.
*
* \param ctrl_context
*  Pointer to the UBM controller context structure.
*
* \return
*  Result of the cyhal_i2c_slave_config_write_buffer request
*
*******************************************************************************/
static cy_rslt_t configure_write_buffer(mtb_stc_ubm_controller_t* ctrl_context)
{
    cy_rslt_t result;

    ctrl_context->i2c.write_data_length = 0U;
    (void)memset(ctrl_context->i2c.write_buffer, 0, MTB_UBM_I2C_WRITE_BUFFER_SIZE);
    result = cyhal_i2c_slave_config_write_buffer(&ctrl_context->i2c.scb_i2c_obj,
                                                 ctrl_context->i2c.write_buffer,
                                                 MTB_UBM_I2C_WRITE_BUFFER_SIZE);
    return result;
}


/*******************************************************************************
* Function Name: configure_read_buffer
****************************************************************************//**
*
*  Configures buffer from which master reads data from.
*
* \param ctrl_context
*  Pointer to the UBM controller context structure.
*
* \return
*  Result of the cyhal_i2c_slave_config_read_buffer request
*
*******************************************************************************/
static cy_rslt_t configure_read_buffer(mtb_stc_ubm_controller_t* ctrl_context)
{
    cy_rslt_t result;

    ctrl_context->i2c.read_data_length = 0U;
    (void)memset(ctrl_context->i2c.read_buffer, INVALID_DATA, MTB_UBM_I2C_READ_BUFFER_SIZE);
    result = cyhal_i2c_slave_config_read_buffer(&ctrl_context->i2c.scb_i2c_obj,
                                                ctrl_context->i2c.read_buffer,
                                                MTB_UBM_I2C_READ_BUFFER_SIZE);
    return result;
}


CY_MISRA_BLOCK_END('MISRA C-2012 Rule 10.1');
CY_MISRA_BLOCK_END('MISRA C-2012 Rule 10.4');
CY_MISRA_BLOCK_END('MISRA C-2012 Rule 10.5');

