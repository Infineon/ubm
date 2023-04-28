/***************************************************************************//**
 * \file mtb_ubm_io.c
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

#include "mtb_ubm_io.h"
#include "mtb_ubm_fru.h"
#include "cyhal_timer.h"
#include <string.h>


 /* GPIO interrupt priority */
#define GPIO_EVENT_IRQ_PRIORITY          (6U)


 /*******************************************************************************
 *            Internal API
 *******************************************************************************/
static void perst_event_callback(void* callback_arg, cyhal_gpio_event_t event);
static void device_detection_event_callback(void* callback_arg, cyhal_gpio_event_t event);

static void two_wire_reset_event_callback(void* callback_arg, cyhal_gpio_event_t event);


static void isr_timer(void* callback_arg, cyhal_timer_event_t event)
{
    mtb_stc_ubm_context_t* context = (mtb_stc_ubm_context_t*)callback_arg;
    (void)event;

    cyhal_timer_reset(&context->timer_obj);

    for (uint8_t i = 0U; i < MTB_UBM_HFC_NUM; i++)
    {
        if ((context->ctrl[i].reset.asserted_count > 0U) && (cyhal_gpio_read(context->ctrl[i].reset.pin) == true))
        {
            context->ctrl[i].reset.asserted_count += 1U;
        }
    }
}


 /*******************************************************************************
 * Function Name: mtb_ubm_io_timer_init
 ****************************************************************************//**
 *
 *  Initializes the DFC I/O timer.
 *
 *******************************************************************************/
bool mtb_ubm_io_timer_init(mtb_stc_ubm_context_t* context)
{
    cy_rslt_t result;

    const cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0U,
        .period = MTB_UBM_TIMER_PERIOD_100US,
        .direction = CYHAL_TIMER_DIR_UP,
        .is_compare = false,
        .is_continuous = true,
        .value = 0U
    };
 
    result = cyhal_timer_init(&context->timer_obj, NC, NULL);

    cyhal_timer_reset(&context->timer_obj);
 
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_timer_configure(&context->timer_obj, &timer_cfg);
    }
 
    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_timer_set_frequency(&context->timer_obj, MTB_UBM_FREQUNCY_1M);
    }
 
    if (CY_RSLT_SUCCESS == result)
    {
        cyhal_timer_register_callback(&context->timer_obj, isr_timer, NULL);
        context->timer_obj.tcpwm.callback_data.callback_arg = (void *)context;
 
        cyhal_timer_enable_event(&context->timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT, GPIO_EVENT_IRQ_PRIORITY, true);

        result = cyhal_timer_start(&context->timer_obj);
    }

    return (CY_RSLT_SUCCESS != result);
}


 /*******************************************************************************
 * Function Name: mtb_ubm_io_dfc_init
 ****************************************************************************//**
 *
 *  Initializes the DFC I/O signals.
 *
 * \param ctrl_context
 *  Pointer to the UBM controller context structure.
 *
 * \param signals
 *  Pointer to the DFC I/O signals configuration structure.
 * 
 * \param routing_info
 *  Pointer to the rouring info structure.
 * 
 * \param config
 *  Configuration structure.
 *
 * \return
 *  "false" if initialization succeeds, and "true" if it fails.
 *
 *******************************************************************************/
bool mtb_ubm_io_dfc_init(mtb_stc_ubm_controller_t* ctrl_context,
                                const mtb_stc_ubm_dfc_signals_t* signals,
                                const mtb_stc_ubm_routing_t* routing_info,
                                const mtb_stc_ubm_backplane_cfg_t* config)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Save DFC signals configuration */
    (void)memcpy(&ctrl_context->dfc_io, signals, sizeof(mtb_stc_ubm_dfc_signals_t));

    /* Initialize Power Disable signal */
    if ((NC != signals->pwrdis) && (0U != signals->pwrdis))
    {
        result = cyhal_gpio_init(signals->pwrdis, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
    }

    /* Initialize PERST A signal */
    if ((CY_RSLT_SUCCESS == result) && (NC != signals->persta) && (0U != signals->persta))
    {
        result = cyhal_gpio_init(signals->persta, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
    }

    /* Initialize PERST B signal */
    if ((CY_RSLT_SUCCESS == result) && (NC != signals->perstb) && (0U != signals->perstb))
    {
        result = cyhal_gpio_init(signals->perstb, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
    }

    if ((CY_RSLT_SUCCESS == result) && (NC != signals->hpt0) && (0U != signals->hpt0))
    {
        /* Initialize HPT0 signal */
        if (config->hpt0_signal_support == true)
        {
            if (0x00U != routing_info->quad_pcie_support)
            {
                result = cyhal_gpio_init(signals->hpt0, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW, true);
            }
            else
            {
                result = cyhal_gpio_init(signals->hpt0, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW, false);
            }
        }
    }
    
    if ((CY_RSLT_SUCCESS == result) && (NC != signals->dualporten) && (0U != signals->dualporten))
    {
        /* Initialize DUALPORTEN signal */
        if (config->capabilities.dual_port == true)
        {
            result = cyhal_gpio_init(signals->dualporten, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULLUP, true);
        }
        else
        {
            result = cyhal_gpio_init(signals->dualporten, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULLUP, false);
        }
    }

    /* Initialize PRSNT signal */
    if ((CY_RSLT_SUCCESS == result) && (NC != signals->prsnt) && (0U != signals->prsnt))
    {
        result = cyhal_gpio_init(signals->prsnt, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, false);
    }

    /* Initialize IFDET signal */
    if ((CY_RSLT_SUCCESS == result) && (NC != signals->ifdet) && (0U != signals->ifdet))
    {
        result = cyhal_gpio_init(signals->ifdet, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, false);
    }

    /* Initialize IFDET2 signal */
    if ((CY_RSLT_SUCCESS == result) && (NC != signals->ifdet2) && (0U != signals->ifdet2))
    {
        result = cyhal_gpio_init(signals->ifdet2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, false);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        ctrl_context->cb_args.device_det.callback = device_detection_event_callback;
        ctrl_context->cb_args.device_det.callback_arg = (void*)ctrl_context;

        ctrl_context->detected_device = 0U;

        if ((config->capabilities.prsnt_reported) != 0U)
        {
            cyhal_gpio_register_callback(signals->prsnt, &ctrl_context->cb_args.device_det);
            cyhal_gpio_enable_event(signals->prsnt, CYHAL_GPIO_IRQ_BOTH,
                                        GPIO_EVENT_IRQ_PRIORITY, true);
            ctrl_context->detected_device |= ((uint8_t)cyhal_gpio_read(signals->prsnt) << MTB_UBM_PRSNT_SHIFT);
        }
        else
        {
            ctrl_context->dfc_io.prsnt = NC;
        }

        if ((config->capabilities.ifdet_reported) != 0U)
        {
            cyhal_gpio_register_callback(signals->ifdet, &ctrl_context->cb_args.device_det);
            cyhal_gpio_enable_event(signals->ifdet, CYHAL_GPIO_IRQ_BOTH,
                                    GPIO_EVENT_IRQ_PRIORITY, true);
            ctrl_context->detected_device |= ((uint8_t)cyhal_gpio_read(signals->ifdet) << MTB_UBM_IFDET_SHIFT);
        }
        else
        {
            ctrl_context->dfc_io.ifdet = NC;
        }

        if ((config->capabilities.ifdet2_reported) != 0U)
        {
            cyhal_gpio_register_callback(signals->ifdet2, &ctrl_context->cb_args.device_det);
            cyhal_gpio_enable_event(signals->ifdet2, CYHAL_GPIO_IRQ_BOTH,
                                        GPIO_EVENT_IRQ_PRIORITY, true);
            ctrl_context->detected_device |= ((uint8_t)cyhal_gpio_read(signals->ifdet2) << MTB_UBM_IFDET2_SHIFT);
        }
        else
        {
            ctrl_context->dfc_io.ifdet2 = NC;
        }
    }

    if ((CY_RSLT_SUCCESS == result) && (NC != signals->actdetect))
    {
        result = cyhal_gpio_init(signals->actdetect, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, false);
    }

    return (CY_RSLT_SUCCESS != result);
}


/*******************************************************************************
* Function Name: mtb_ubm_io_hfc_init
****************************************************************************//**
*
*  Initializes the HFC I/O signals.
*
* \param ctrl_context
*  Pointer to the UBM controller context structure.
*
* \param signals
*  Pointer to the HFC I/O signals configuration structure.
*
* \param config
*  Configuration structure.
*
* \return
*  "false" if initialization succeeds, and "true" if it fails.
*
*******************************************************************************/
bool mtb_ubm_io_hfc_init(mtb_stc_ubm_controller_t* ctrl_context,
                         const mtb_stc_ubm_hfc_signals_t* signals,
                         const mtb_stc_ubm_backplane_cfg_t* config)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Save HFC signals configuration */
    (void)memcpy(&ctrl_context->hfc_io, signals, sizeof(mtb_stc_ubm_hfc_signals_t));

    /* Initialize PERST signal */
    if (NC != signals->perst)
    {
        result = cyhal_gpio_init(signals->perst, CYHAL_GPIO_DIR_INPUT,
                                 CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW, false);

        if (CY_RSLT_SUCCESS == result)
        {
            /* Register RERST gpio event callback */
            ctrl_context->cb_args.perst.callback = perst_event_callback;
            ctrl_context->cb_args.perst.callback_arg = (void*)ctrl_context;
            cyhal_gpio_register_callback(signals->perst, &ctrl_context->cb_args.perst);
            cyhal_gpio_enable_event(signals->perst, CYHAL_GPIO_IRQ_BOTH,
                                    GPIO_EVENT_IRQ_PRIORITY, true);
        }
    }

    if ((CY_RSLT_SUCCESS == result) && (NC != signals->i2c_reset))
    {
        ctrl_context->reset.pin = signals->i2c_reset;
        ctrl_context->reset.asserted_count = 0U;

        result = cyhal_gpio_init(signals->i2c_reset, CYHAL_GPIO_DIR_INPUT,
                                 CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW, false);

        ctrl_context->cb_args.two_wire_reset.callback = two_wire_reset_event_callback;
        ctrl_context->cb_args.two_wire_reset.callback_arg = (void*)ctrl_context;

        cyhal_gpio_register_callback(signals->i2c_reset, &ctrl_context->cb_args.two_wire_reset);
        cyhal_gpio_enable_event(signals->i2c_reset, CYHAL_GPIO_IRQ_BOTH,
                                    GPIO_EVENT_IRQ_PRIORITY, true);
    }

    if ((CY_RSLT_SUCCESS == result) && (NC != signals->change_detect))
    {
        result = cyhal_gpio_init(signals->change_detect, CYHAL_GPIO_DIR_BIDIRECTIONAL,
                                 CYHAL_GPIO_DRIVE_PULLUPDOWN, false);

        if (CY_RSLT_SUCCESS == result)
        {
            ctrl_context->legacy_mode = config->overview_area->ubm_controller_features.cprsnt_legacy_mode;
        }
    }

    return (CY_RSLT_SUCCESS != result);
}


/*******************************************************************************
* Function Name: perst_event_callback
****************************************************************************//**
*
*  The interrupt handler for PERST I/O event.
*
* \param callback_arg
*  Callback argument, pointer to the controller context structure.
* 
* \param event
*  Event to be handled.
*
*******************************************************************************/
static void perst_event_callback(void* callback_arg, cyhal_gpio_event_t event)
{
    mtb_stc_ubm_controller_t* context = (mtb_stc_ubm_controller_t*)callback_arg;

    /* Check HFC PERST gpio events */
    if (0UL != (CYHAL_GPIO_IRQ_RISE & event))
    {
        if (NC != context->dfc_io.persta)
        {
            /* Set corresponding DFC PERSTA */
            cyhal_gpio_write(context->dfc_io.persta, true);
        }

        if (NC != context->dfc_io.perstb)
        {
            /* Set corresponding DFC PERSTB */
            cyhal_gpio_write(context->dfc_io.perstb, true);
        }
    }
    else if (0UL != (CYHAL_GPIO_IRQ_FALL & event))
    {
        if (NC != context->dfc_io.persta)
        {
            /* Set corresponding DFC PERSTA */
            cyhal_gpio_write(context->dfc_io.persta, false);
        }

        if (NC != context->dfc_io.perstb)
        {
            /* Set corresponding DFC PERSTB */
            cyhal_gpio_write(context->dfc_io.perstb, false);
        }
    }
}


/*******************************************************************************
* Function Name: two_wire_reset_event_callback
****************************************************************************//**
*
*  The interrupt handler for 2WireReset I/O event.
*
* \param callback_arg
*  Callback argument, pointer to the controller context structure.
* 
* \param event
*  Event to be handled.
*
*******************************************************************************/
static void two_wire_reset_event_callback(void* callback_arg, cyhal_gpio_event_t event)
{
    mtb_stc_ubm_controller_t* context = (mtb_stc_ubm_controller_t*)callback_arg;

    if (0UL != (CYHAL_GPIO_IRQ_RISE & event))
    {
        context->reset.asserted_count++;
    }
    else if (0UL != (CYHAL_GPIO_IRQ_FALL & event))
    {
        if ((context->reset.asserted_count >= MTB_UBM_I2C_RESET_DELAY) && (context->reset.asserted_count < MTB_UBM_FULL_RESET_DELAY))
        {
            cyhal_i2c_clear(&context->i2c.scb_i2c_obj);
        }
        else if (context->reset.asserted_count >= MTB_UBM_FULL_RESET_DELAY)
        {
            __NVIC_SystemReset();
        }
        else
        {
            context->reset.asserted_count = 0U;
        }
    }
}


/*******************************************************************************
* Function Name: device_detection_event_callback
****************************************************************************//**
*
*  The interrupt handler for device detection I/O event.
*
* \param callback_arg
*  Callback argument, pointer to the controller context structure.
* 
* \param event
*  Event to be handled.
*
*******************************************************************************/
static void device_detection_event_callback(void* callback_arg, cyhal_gpio_event_t event)
{
    mtb_stc_ubm_controller_t* context = (mtb_stc_ubm_controller_t*)callback_arg;
    (void)event;

    context->detected_device = 0U;

    if (context->dfc_io.prsnt != NC)
    {
        context->detected_device |= ((uint8_t)cyhal_gpio_read(context->dfc_io.prsnt) << MTB_UBM_PRSNT_SHIFT);
    }

    if (context->dfc_io.ifdet != NC)
    {
        context->detected_device |= ((uint8_t)cyhal_gpio_read(context->dfc_io.ifdet) << MTB_UBM_IFDET_SHIFT);
    }

    if (context->dfc_io.ifdet2 != NC)
    {
        context->detected_device |= ((uint8_t)cyhal_gpio_read(context->dfc_io.ifdet2) << MTB_UBM_IFDET2_SHIFT);
    }

    if (context->legacy_mode == false)
    {
        /* Indicate to the host that connected device has changed */
        cyhal_gpio_write(context->hfc_io.change_detect, true);
    }
    else if ((context->legacy_mode == true) && (cyhal_gpio_read(context->hfc_io.change_detect) == true))
    {
        /* If we have legacy CPRSNT/CHANGE_DETECT mode and CHANGE_DETECT logic is applied */
        cyhal_gpio_write(context->hfc_io.change_detect, false);
    }
}

