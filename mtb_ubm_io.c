/***************************************************************************//**
 * \file mtb_ubm_io.c
 * \version 1.0
 *
 * \brief
 * Provides functions for the UBM middleware I/O.
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
#include "mtb_ubm_controller.h"
#include "cyhal_timer.h"


/* GPIO interrupt priority */
#define GPIO_EVENT_IRQ_PRIORITY             (6U)

/* Timer interrupt priority */
#define TIMER_EVENT_IRQ_PRIORITY            (6U)

/* The DFC drive type detection interval in milliseconds. */
#define DRIVE_TYPE_DETECTION_INTERVAL_MS    (50U)

/** The number of confirmation samples to detect a change event of the drive type. */
#define DRIVE_TYPE_DETECTION_DELAY_COUNT    (2U)

/** The value of the drive type counter if stopped. */
#define DRIVE_TYPE_COUNTER_STOPPED_VALUE    (0xFFU)

/* Drives types */
#define MTB_UBM_DFC_EMPTY                   (0x07U)
#define MTB_UBM_SAS_SATA                    (0x04U)
#define MTB_UBM_QUAD_PCI                    (0x05U)
#define MTB_UBM_SFF_TA_1001_PCIe            (0x01U)
#define MTB_UBM_GEN_Z                       (0x03U)

/*******************************************************************************
*            Internal API
*******************************************************************************/
static void perst_event_callback(void* callback_arg, cyhal_gpio_event_t event);
static void two_wire_reset_event_callback(void* callback_arg, cyhal_gpio_event_t event);
static void perst_periodic_handler(mtb_stc_ubm_context_t* ubm_context);
static void drive_type_periodic_handler(mtb_stc_ubm_context_t* ubm_context);
static void pcie_reset_handler_upon_device_install(mtb_stc_ubm_context_t* ubm_context,
                                                   mtb_stc_ubm_dfc_t* dfc_context,
                                                   uint8_t drive_type);


/*******************************************************************************
* Function Name: isr_i2c_reset_timer
****************************************************************************//**
*
*  The callback function of the 2Wire Reset timer.
*
* \param callback_arg
*  A callback argument, the pointer to the UBM context structure.
*
* \param event
*  The timer event.
*
*******************************************************************************/
static void isr_i2c_reset_timer(void* callback_arg, cyhal_timer_event_t event)
{
    CY_MISRA_DEVIATE_LINE('MISRA C-2012 Rule 11.5', 'Pointer passed via callback_arg is aligned correcly, as contains address of a known object.');
    mtb_stc_ubm_context_t* ubm_context = (mtb_stc_ubm_context_t*)callback_arg;
    (void)event;

    for (uint32_t hfc_index = 0U; hfc_index < ubm_context->num_of_hfc; hfc_index++)
    {
        mtb_stc_ubm_input_t* reset = &ubm_context->hfc[hfc_index].reset;

        if ((reset->asserted_count > 0U) && !cyhal_gpio_read(reset->pin))
        {
            reset->asserted_count++;
        }
    }
}


/*******************************************************************************
* Function Name: isr_periodic_timer
****************************************************************************//**
*
*  The callback function of the periodic timer.
*
* \param callback_arg
*  A callback argument, the pointer to the UBM context structure.
*
* \param event
*  The timer event.
*
*******************************************************************************/
static void isr_periodic_timer(void* callback_arg, cyhal_timer_event_t event)
{
    CY_MISRA_DEVIATE_LINE('MISRA C-2012 Rule 11.5', 'Pointer passed via callback_arg is aligned correcly, as contains address of a known object.');
    mtb_stc_ubm_context_t* ubm_context = (mtb_stc_ubm_context_t*)callback_arg;
    (void)event;

    perst_periodic_handler(ubm_context);
    drive_type_periodic_handler(ubm_context);
}


/*******************************************************************************
* Function Name: mtb_ubm_io_timer_init
****************************************************************************//**
*
*  Initializes the DFC I/O timer.
*
* \param context
*  The pointer to the UBM context structure.
*
* \return
*  See \ref mtb_en_ubm_status_t.
*
*******************************************************************************/
mtb_en_ubm_status_t mtb_ubm_io_timer_init(mtb_stc_ubm_context_t* context)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    mtb_en_ubm_status_t status = MTB_UBM_STATUS_SUCCESS;

    cyhal_timer_cfg_t timer_cfg =
    {
        .compare_value = 0U,
        .period        = MTB_UBM_TIMER_PERIOD_100US,
        .direction     = CYHAL_TIMER_DIR_UP,
        .is_compare    = false,
        .is_continuous = true,
        .value         = 0U
    };

    result = cyhal_timer_init(&context->i2c_reset_timer, NC, NULL);

    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_timer_reset(&context->i2c_reset_timer);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_timer_configure(&context->i2c_reset_timer, &timer_cfg);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        result = cyhal_timer_set_frequency(&context->i2c_reset_timer, MTB_UBM_FREQUENCY_1M);
    }

    if (CY_RSLT_SUCCESS == result)
    {
        cyhal_timer_register_callback(&context->i2c_reset_timer, isr_i2c_reset_timer, NULL);
        context->i2c_reset_timer.tcpwm.callback_data.callback_arg = (void*)context;

        cyhal_timer_enable_event(&context->i2c_reset_timer,
                                 CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                                 TIMER_EVENT_IRQ_PRIORITY,
                                 true);
    }

    if (CY_RSLT_SUCCESS != result)
    {
        status = MTB_UBM_STATUS_2WIRE_RESET_TIMER_ERR;
    }

    if (MTB_UBM_STATUS_SUCCESS == status)
    {
        result = cyhal_timer_init(&context->periodic_timer, NC, NULL);

        if (CY_RSLT_SUCCESS == result)
        {
            result = cyhal_timer_reset(&context->periodic_timer);
        }

        if (CY_RSLT_SUCCESS == result)
        {
            timer_cfg.period = MTB_UBM_PCIE_TIMER_PERIOD;

            result = cyhal_timer_configure(&context->periodic_timer, &timer_cfg);
        }

        if (CY_RSLT_SUCCESS == result)
        {
            result = cyhal_timer_set_frequency(&context->periodic_timer, MTB_UBM_PCIE_TIMER_FREQUENCY);
        }

        if (CY_RSLT_SUCCESS == result)
        {
            cyhal_timer_register_callback(&context->periodic_timer, isr_periodic_timer, NULL);
            context->periodic_timer.tcpwm.callback_data.callback_arg = (void*)context;

            cyhal_timer_enable_event(&context->periodic_timer,
                                     CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                                     TIMER_EVENT_IRQ_PRIORITY,
                                     true);

            result = cyhal_timer_start(&context->periodic_timer);
        }

        if (CY_RSLT_SUCCESS != result)
        {
            status = MTB_UBM_STATUS_IO_TIMER_ERR;
        }
    }

    return status;
}


/*******************************************************************************
* Function Name: mtb_ubm_io_dfc_init
****************************************************************************//**
*
*  Initializes the DFC I/O signals.
*
* \param context
*  The pointer to the UBM context structure.
*
* \param dfc_index
*  The index of the DFC connector.
*
* \param signals
*  The pointer to the DFC I/O signals configuration structure.
*
* \param config
*  The configuration structure.
*
* \return
*  See \ref mtb_en_ubm_status_t.
*
*******************************************************************************/
mtb_en_ubm_status_t mtb_ubm_io_dfc_init(mtb_stc_ubm_context_t* context,
                                        uint32_t dfc_index,
                                        const mtb_stc_ubm_dfc_signals_t* signals,
                                        const mtb_stc_ubm_backplane_cfg_t* config)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    mtb_en_ubm_status_t status = MTB_UBM_STATUS_SUCCESS;
    mtb_stc_ubm_dfc_t* dfc = &context->dfc[dfc_index];

    /* Save the DFC signals configuration */
    (void)memcpy(&dfc->dfc_io, signals, sizeof(mtb_stc_ubm_dfc_signals_t));

    if (NC != signals->dualporten)
    {
        /* Initialize the DUALPORTEN signal */
        if (config->capabilities.dual_port)
        {
            result = cyhal_gpio_init(signals->dualporten, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_PULLUP, false);
        }
        else
        {
            result = cyhal_gpio_init(signals->dualporten, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, true);
        }

        if (CY_RSLT_SUCCESS != result)
        {
            status = MTB_UBM_STATUS_IO_DUALPORTEN_ERR;
        }
    }

    /* Initialize the PERST A signal */
    if ((MTB_UBM_STATUS_SUCCESS == status) && (NC != signals->persta))
    {
        if (config->capabilities.pcie_reset_control || config->capabilities.clock_routing)
        {
            result = cyhal_gpio_init(signals->persta, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
        }
        else
        {
            result = cyhal_gpio_init(signals->persta, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
        }

        if (CY_RSLT_SUCCESS != result)
        {
            status = MTB_UBM_STATUS_IO_DFC_PERSTA_ERR;
        }
    }

    /* Initialize the PERST B signal */
    if ((MTB_UBM_STATUS_SUCCESS == status) && (NC != signals->perstb))
    {
        if ((config->capabilities.pcie_reset_control || config->capabilities.clock_routing) &&
            config->capabilities.dual_port)
        {
            result = cyhal_gpio_init(signals->perstb, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);
        }
        else
        {
            result = cyhal_gpio_init(signals->perstb, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);
        }

        if (CY_RSLT_SUCCESS != result)
        {
            status = MTB_UBM_STATUS_IO_DFC_PERSTB_ERR;
        }
    }

    /* Initialize the REFCLKEN signal */
    if ((MTB_UBM_STATUS_SUCCESS == status) && (config->capabilities.clock_routing) && (NC != signals->refclken))
    {
        result = cyhal_gpio_init(signals->refclken,
                                 CYHAL_GPIO_DIR_OUTPUT,
                                 CYHAL_GPIO_DRIVE_STRONG,
                                 MTB_UBM_SIGNAL_TO_ENABLE_REFCLK_MUX);
    }
    else
    {
        result = cyhal_gpio_init(signals->refclken,
                                 CYHAL_GPIO_DIR_OUTPUT,
                                 CYHAL_GPIO_DRIVE_STRONG,
                                 MTB_UBM_SIGNAL_TO_DISABLE_REFCLK_MUX);
    }

    if (CY_RSLT_SUCCESS != result)
    {
        status = MTB_UBM_STATUS_IO_REFCLKEN_ERR;
    }

    /* Initialize the Power Disable signal */
    if ((MTB_UBM_STATUS_SUCCESS == status) && (NC != signals->pwrdis))
    {
        result = cyhal_gpio_init(signals->pwrdis, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, false);

        if (CY_RSLT_SUCCESS != result)
        {
            status = MTB_UBM_STATUS_IO_PWRDIS_ERR;
        }
    }

    dfc->drive_type_installed = 0U;

    /* Initialize the PRSNT signal */
    if ((MTB_UBM_STATUS_SUCCESS == status) && config->capabilities.prsnt_reported && (NC != signals->prsnt))
    {
        result = cyhal_gpio_init(signals->prsnt, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);

        if (CY_RSLT_SUCCESS == result)
        {
            dfc->drive_type_installed |= ((uint8_t)cyhal_gpio_read(signals->prsnt) << MTB_UBM_PRSNT_SHIFT);
        }
        else
        {
            status = MTB_UBM_STATUS_IO_PRSNT_ERR;
        }
    }
    else
    {
        dfc->drive_type_installed |= (1U << MTB_UBM_PRSNT_SHIFT);
        dfc->dfc_io.prsnt = NC;
    }

    /* Initialize the IFDET signal */
    if ((MTB_UBM_STATUS_SUCCESS == status) && config->capabilities.ifdet_reported && (NC != signals->ifdet))
    {
        result = cyhal_gpio_init(signals->ifdet, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);

        if (CY_RSLT_SUCCESS == result)
        {
            dfc->drive_type_installed |= ((uint8_t)cyhal_gpio_read(signals->ifdet) << MTB_UBM_IFDET_SHIFT);
        }
        else
        {
            status = MTB_UBM_STATUS_IO_IFDET_ERR;
        }
    }
    else
    {
        dfc->drive_type_installed |= (1U << MTB_UBM_IFDET_SHIFT);
        dfc->dfc_io.ifdet = NC;
    }

    /* Initialize the IFDET2 signal */
    if ((MTB_UBM_STATUS_SUCCESS == status) && config->capabilities.ifdet2_reported && (NC != signals->ifdet2))
    {
        result = cyhal_gpio_init(signals->ifdet2, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, false);

        if (CY_RSLT_SUCCESS == result)
        {
            dfc->drive_type_installed |= ((uint8_t)cyhal_gpio_read(signals->ifdet2) << MTB_UBM_IFDET2_SHIFT);
        }
        else
        {
            status = MTB_UBM_STATUS_IO_IFDET2_ERR;
        }
    }
    else
    {
        dfc->drive_type_installed |= (1U << MTB_UBM_IFDET2_SHIFT);
        dfc->dfc_io.ifdet2 = NC;
    }

    dfc->drive_type_counter = DRIVE_TYPE_COUNTER_STOPPED_VALUE;

    return status;
}


/*******************************************************************************
* Function Name: mtb_ubm_io_hfc_init
****************************************************************************//**
*
*  Initializes the HFC I/O signals.
*
* \param context
*  The pointer to the UBM context structure.
*
* \param hfc_index
*  The index of the HFC structure.
*
* \param signals
*  The pointer to the HFC I/O signals configuration structure.
*
* \return
*  See \ref mtb_en_ubm_status_t.
*
*******************************************************************************/
mtb_en_ubm_status_t mtb_ubm_io_hfc_init(mtb_stc_ubm_context_t* context,
                                        uint32_t hfc_index,
                                        const mtb_stc_ubm_hfc_signals_t* signals)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    mtb_en_ubm_status_t status = MTB_UBM_STATUS_SUCCESS;
    mtb_stc_ubm_hfc_t* hfc = &context->hfc[hfc_index];

    /* Save the HFC signals configuration */
    (void)memcpy(&hfc->hfc_io, signals, sizeof(mtb_stc_ubm_hfc_signals_t));

    hfc->cb_args.arg.ubm_context = context;
    hfc->cb_args.arg.hfc_index = (uint8_t)hfc_index;

    /* Initialize the PERST signal */
    if (NC != signals->perst)
    {
        result = cyhal_gpio_init(signals->perst, CYHAL_GPIO_DIR_INPUT,
                                 CYHAL_GPIO_DRIVE_PULLUP, true);

        if (CY_RSLT_SUCCESS == result)
        {
            /* Register the RERST gpio event callback */
            hfc->cb_args.perst.callback = perst_event_callback;
            hfc->cb_args.perst.callback_arg = (void*)&hfc->cb_args.arg;

            cyhal_gpio_register_callback(signals->perst, &hfc->cb_args.perst);
            cyhal_gpio_enable_event(signals->perst, CYHAL_GPIO_IRQ_BOTH,
                                    GPIO_EVENT_IRQ_PRIORITY, true);
        }
        else
        {
            status = MTB_UBM_STATUS_IO_HFC_PERST_ERR;
        }
    }

    if ((MTB_UBM_STATUS_SUCCESS == status) && (NC != signals->i2c_reset))
    {
        hfc->reset.pin = signals->i2c_reset;
        hfc->reset.asserted_count = 0U;

        result = cyhal_gpio_init(signals->i2c_reset, CYHAL_GPIO_DIR_INPUT,
                                 CYHAL_GPIO_DRIVE_PULLUP, true);

        hfc->cb_args.two_wire_reset.callback = two_wire_reset_event_callback;
        hfc->cb_args.two_wire_reset.callback_arg = (void*)&hfc->cb_args.arg;

        cyhal_gpio_register_callback(signals->i2c_reset, &hfc->cb_args.two_wire_reset);
        cyhal_gpio_enable_event(signals->i2c_reset, CYHAL_GPIO_IRQ_BOTH,
                                GPIO_EVENT_IRQ_PRIORITY, true);

        if (CY_RSLT_SUCCESS != result)
        {
            status = MTB_UBM_STATUS_IO_2WIRE_RESET_ERR;
        }
    }

    if ((MTB_UBM_STATUS_SUCCESS == status) && (NC != signals->change_detect))
    {
        result = cyhal_gpio_init(signals->change_detect, CYHAL_GPIO_DIR_OUTPUT,
                                 CYHAL_GPIO_DRIVE_OPENDRAINDRIVESLOW, false);

        if (CY_RSLT_SUCCESS != result)
        {
            status = MTB_UBM_STATUS_IO_CHANGE_DETECT_ERR;
        }
    }

    if ((MTB_UBM_STATUS_SUCCESS == status) && (NC != signals->bp_type))
    {
        result = cyhal_gpio_init(signals->bp_type, CYHAL_GPIO_DIR_OUTPUT,
                                 CYHAL_GPIO_DRIVE_PULLUP, true);

        if (CY_RSLT_SUCCESS != result)
        {
            status = MTB_UBM_STATUS_IO_BP_TYPE_ERR;
        }
    }

    return status;
}


/*******************************************************************************
* Function Name: perst_event_callback
****************************************************************************//**
*
*  The interrupt handler for the PERST I/O event.
*
* \param callback_arg
*  Callback argument, pointer to the HFC context structure.
*
* \param event
*  The event to be handled.
*
*******************************************************************************/
static void perst_event_callback(void* callback_arg, cyhal_gpio_event_t event)
{
    CY_MISRA_DEVIATE_LINE('MISRA C-2012 Rule 11.5', 'Pointer passed via callback_arg is aligned correcly, as contains address of a known object.');
    const mtb_stc_ubm_hfc_gpio_callback_arg_t* arg = (mtb_stc_ubm_hfc_gpio_callback_arg_t*)callback_arg;
    mtb_stc_ubm_context_t* ubm_context = arg->ubm_context;
    const mtb_stc_ubm_hfc_t* hfc_context = &arg->ubm_context->hfc[arg->hfc_index];

    for (uint32_t ctrl_index = 0U; ctrl_index < hfc_context->ctrl_count; ctrl_index++)
    {
        mtb_stc_ubm_controller_t* ctrl_context = &ubm_context->ctrl[hfc_context->ctrl_list[ctrl_index]];

        for (uint32_t dfc_index = 0U; dfc_index < ctrl_context->dfc_count; dfc_index++)
        {
            mtb_stc_ubm_dfc_t* dfc_context = &ubm_context->dfc[ctrl_context->dfc_list[dfc_index]];
            /* Check HFC PERST gpio events */
            if (0U != ((uint32_t)CYHAL_GPIO_IRQ_RISE & (uint32_t)event))
            {
                /* SRIS and automation release conditions */
                if (ubm_context->capabilities.pcie_reset_control &&
                    (!ubm_context->capabilities.clock_routing ||
                     (MTB_UBM_DFC_PERST_OVERRIDE_AUTO == ctrl_context->features.perst_management_override)))
                {
                    ctrl_context->scd[dfc_context->index].pcie_reset = MTB_UBM_PCIE_RESET_FIELD_INIT;
                    (void)mtb_ubm_process_pcie_reset_request(ubm_context,
                                                             hfc_context,
                                                             ctrl_context,
                                                             dfc_context);
                }
            }
            else if (0U != ((uint32_t)CYHAL_GPIO_IRQ_FALL & (uint32_t)event))
            {
                if (ubm_context->capabilities.pcie_reset_control)
                {
                    /* Asserted the pcie reset signal */
                    cyhal_gpio_write(dfc_context->dfc_io.persta, false);

                    cyhal_gpio_write(dfc_context->dfc_io.perstb, false);

                    dfc_context->dfc_perst_a_b[0U].signal_released = false;
                    dfc_context->dfc_perst_a_b[1U].signal_released = false;

                    cyhal_gpio_write(dfc_context->dfc_io.refclken, false);
                }
            }
            else
            {
                /* Other events */
            }
        }
    }
}


/*******************************************************************************
* Function Name: two_wire_reset_event_callback
****************************************************************************//**
*
*  The interrupt handler for the 2WireReset I/O event.
*
* \param callback_arg
*  A callback argument, the pointer to the HFC context structure.
*
* \param event
*  The event to be handled.
*
*******************************************************************************/
static void two_wire_reset_event_callback(void* callback_arg, cyhal_gpio_event_t event)
{
    CY_MISRA_DEVIATE_LINE('MISRA C-2012 Rule 11.5', 'Pointer passed via callback_arg is aligned correcly, as contains address of a known object.');
    const mtb_stc_ubm_hfc_gpio_callback_arg_t* arg = (mtb_stc_ubm_hfc_gpio_callback_arg_t*)callback_arg;
    mtb_stc_ubm_context_t* ubm_context = arg->ubm_context;
    mtb_stc_ubm_hfc_t* hfc_context = &arg->ubm_context->hfc[arg->hfc_index];
    mtb_stc_ubm_input_t* reset = &hfc_context->reset;

    if (0U != ((uint32_t)CYHAL_GPIO_IRQ_FALL & (uint32_t)event))
    {
        reset->asserted_count++;

        if (cyhal_timer_read(&ubm_context->i2c_reset_timer) == 0U)
        {
            (void)cyhal_timer_start(&ubm_context->i2c_reset_timer);
        }
    }
    else if (0U != ((uint32_t)CYHAL_GPIO_IRQ_RISE & (uint32_t)event))
    {
        if ((reset->asserted_count >= MTB_UBM_I2C_RESET_DELAY) &&
            (reset->asserted_count < MTB_UBM_FULL_RESET_DELAY))
        {
            (void)cyhal_i2c_clear(&hfc_context->i2c.scb_i2c_obj);
            reset->asserted_count = 0U;
        }
        else if (reset->asserted_count >= MTB_UBM_FULL_RESET_DELAY)
        {
            __NVIC_SystemReset();
        }
        else
        {
            reset->asserted_count = 0U;
        }

        bool timer_idle = true;
        for (uint32_t hfc_index = 0U; hfc_index < ubm_context->num_of_hfc; hfc_index++)
        {
            if (ubm_context->hfc[hfc_index].reset.asserted_count > 0U)
            {
                timer_idle = false;
                break;
            }
        }
        if (timer_idle)
        {
            (void)cyhal_timer_stop(&ubm_context->i2c_reset_timer);
            (void)cyhal_timer_reset(&ubm_context->i2c_reset_timer);
        }
    }
    else
    {
        /* Other events */
    }
}


/*******************************************************************************
* Function Name: perst_periodic_handler
****************************************************************************//**
*
*  The handler for the timer callback of the PERST# I/O periodic.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
*******************************************************************************/
static void perst_periodic_handler(mtb_stc_ubm_context_t* ubm_context)
{
    const mtb_stc_ubm_hfc_t* hfc_context;

    for (uint32_t hfc_index = 0U; hfc_index < ubm_context->num_of_hfc; hfc_index++)
    {
        hfc_context = &ubm_context->hfc[hfc_index];
        if (cyhal_gpio_read(hfc_context->hfc_io.perst))
        {
            for (uint32_t ctrl_index = 0U; ctrl_index < hfc_context->ctrl_count; ctrl_index++)
            {
                mtb_stc_ubm_controller_t* ctrl_context = &ubm_context->ctrl[hfc_context->ctrl_list[ctrl_index]];

                for (uint32_t dfc_index = 0U; dfc_index < ctrl_context->dfc_count; dfc_index++)
                {
                    mtb_stc_ubm_dfc_t* dfc_context = &ubm_context->dfc[ctrl_context->dfc_list[dfc_index]];

                    if (dfc_context->dfc_perst_a_b[(uint8_t)ctrl_context->domain].scd_last_change)
                    {
                        uint32_t counter = ++dfc_context->dfc_perst_a_b[(uint8_t)ctrl_context->domain].delay_counter;
                        uint32_t value = dfc_context->dfc_perst_a_b[(uint8_t)ctrl_context->domain].delay_val;

                        if (counter >= value)
                        {
                            /* De-asserted the pcie reset signal*/
                            if (MTB_UBM_PORT_DOMAIN_PRIMARY == ctrl_context->domain)
                            {
                                cyhal_gpio_write(dfc_context->dfc_io.persta, true);
                            }
                            else
                            {
                                cyhal_gpio_write(dfc_context->dfc_io.perstb, true);
                            }
                            /* To avoid the overwrite block command handler, the PCIe field can be written in both 
                             * places. */
                            ctrl_context->last_command_status = MTB_UBM_LC_STS_BUSY;
                            /* Set the PCIe field to 0 */
                            ctrl_context->scd[dfc_context->index].pcie_reset = 0U;
                            mtb_ubm_update_change_count(ubm_context,
                                                        ctrl_context,
                                                        dfc_context,
                                                        MTB_UBM_CC_SOURCE_PCIE_RESET);

                            ctrl_context->last_command_status = MTB_UBM_LC_STS_SUCCESS;

                            dfc_context->dfc_perst_a_b[(uint8_t)ctrl_context->domain].signal_released = true;
                            dfc_context->dfc_perst_a_b[(uint8_t)ctrl_context->domain].scd_last_change = false;
                            dfc_context->dfc_perst_a_b[(uint8_t)ctrl_context->domain].delay_counter = 0U;
                            dfc_context->dfc_perst_a_b[(uint8_t)ctrl_context->domain].delay_val = 0U;
                        }
                    }
                }
            }
        }
    }
}


/*******************************************************************************
* Function Name: drive_type_periodic_handler
****************************************************************************//**
*
*  The handler for the periodic callback to detect the drive type.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
*******************************************************************************/
static void drive_type_periodic_handler(mtb_stc_ubm_context_t* ubm_context)
{
    static uint32_t counter_ms = 0U;

    if ((++counter_ms % DRIVE_TYPE_DETECTION_INTERVAL_MS) == 0U)
    {
        mtb_stc_ubm_dfc_t* dfc_context;
        uint8_t detected_device;

        for (uint32_t dfc_index = 0U; dfc_index < ubm_context->num_of_dfc; dfc_index++)
        {
            dfc_context = &ubm_context->dfc[dfc_index];
            detected_device = 0U;

            if (NC != dfc_context->dfc_io.prsnt)
            {
                detected_device |= ((uint8_t)cyhal_gpio_read(dfc_context->dfc_io.prsnt) << MTB_UBM_PRSNT_SHIFT);
            }
            else
            {
                detected_device |= (1U << MTB_UBM_PRSNT_SHIFT);
            }

            if (NC != dfc_context->dfc_io.ifdet)
            {
                detected_device |= ((uint8_t)cyhal_gpio_read(dfc_context->dfc_io.ifdet) << MTB_UBM_IFDET_SHIFT);
            }
            else
            {
                detected_device |= (1U << MTB_UBM_IFDET_SHIFT);
            }

            if (NC != dfc_context->dfc_io.ifdet2)
            {
                detected_device |= ((uint8_t)cyhal_gpio_read(dfc_context->dfc_io.ifdet2) << MTB_UBM_IFDET2_SHIFT);
            }
            else
            {
                detected_device |= (1U << MTB_UBM_IFDET2_SHIFT);
            }

            if (dfc_context->drive_type_installed != detected_device)
            {
                dfc_context->drive_type_installed = detected_device;
                dfc_context->drive_type_counter = DRIVE_TYPE_DETECTION_DELAY_COUNT;
            }
            else
            {
                if (DRIVE_TYPE_COUNTER_STOPPED_VALUE != dfc_context->drive_type_counter)
                {
                    dfc_context->drive_type_counter--;
                }
            }

            if (dfc_context->drive_type_counter == 0U)
            {
                pcie_reset_handler_upon_device_install(ubm_context, dfc_context, dfc_context->drive_type_installed);

                mtb_stc_ubm_controller_t* ctrl_context;
                for (uint32_t ctrl_index = 0U; ctrl_index < dfc_context->ctrl_count; ctrl_index++)
                {
                    ctrl_context = &ubm_context->ctrl[dfc_context->ctrl_list[ctrl_index]];
                    mtb_ubm_update_change_count(ubm_context, ctrl_context, dfc_context, MTB_UBM_CC_SOURCE_DRIVE_TYPE);
                }
                dfc_context->drive_type_counter = DRIVE_TYPE_COUNTER_STOPPED_VALUE;
            }
        }

        counter_ms = 0U;
    }
}


/*******************************************************************************
* Function Name: pcie_reset_handler_upon_device_install
****************************************************************************//**
*
*  Performs an automation PCIe reset upon instalation of a new DFC.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param dfc_context
*  The pointer to the DFC context structure.
*
* \param drive_type
*  The drive type of the installed device.
*
*******************************************************************************/
static void pcie_reset_handler_upon_device_install(mtb_stc_ubm_context_t* ubm_context,
                                                   mtb_stc_ubm_dfc_t* dfc_context,
                                                   uint8_t drive_type)
{
    mtb_stc_ubm_controller_t* ctrl_context;

    if (MTB_UBM_DFC_EMPTY == drive_type)
    {
        /* Asserted the PCIe reset signal */
        cyhal_gpio_write(dfc_context->dfc_io.persta, false);

        cyhal_gpio_write(dfc_context->dfc_io.perstb, false);

        dfc_context->dfc_perst_a_b[MTB_UBM_PORT_DOMAIN_PRIMARY].signal_released = false;
        dfc_context->dfc_perst_a_b[MTB_UBM_PORT_DOMAIN_SECONDARY].signal_released = false;

        cyhal_gpio_write(dfc_context->dfc_io.refclken, false);

        /* Update the PCIe reset field */
        for (uint32_t ctrl_index = 0U; ctrl_index < dfc_context->ctrl_count; ctrl_index++)
        {
            ctrl_context = &ubm_context->ctrl[dfc_context->ctrl_list[ctrl_index]];
            if (ubm_context->capabilities.pcie_reset_control)
            {
                if (ubm_context->capabilities.clock_routing)
                {
                    ctrl_context->scd[dfc_context->index].pcie_reset = MTB_UBM_PCIE_RESET_FIELD_HOLD;
                }
                else
                {
                    ctrl_context->scd[dfc_context->index].pcie_reset = MTB_UBM_PCIE_RESET_FIELD_NOP;
                }
            }
        }
    }
    else if (MTB_UBM_SAS_SATA == drive_type)
    {
        /* Asserted the PCIe reset signal */
        cyhal_gpio_write(dfc_context->dfc_io.persta, false);

        cyhal_gpio_write(dfc_context->dfc_io.perstb, false);

        dfc_context->dfc_perst_a_b[MTB_UBM_PORT_DOMAIN_PRIMARY].signal_released = false;
        dfc_context->dfc_perst_a_b[MTB_UBM_PORT_DOMAIN_SECONDARY].signal_released = false;

        cyhal_gpio_write(dfc_context->dfc_io.refclken, false);
    }
    else if ((MTB_UBM_QUAD_PCI == drive_type) ||
             (MTB_UBM_SFF_TA_1001_PCIe == drive_type) ||
             (MTB_UBM_GEN_Z == drive_type))
    {
        if (ubm_context->capabilities.pcie_reset_control)
        {
            if (ubm_context->capabilities.clock_routing)
            {
                for (uint32_t ctrl_index = 0U; ctrl_index < dfc_context->ctrl_count; ctrl_index++)
                {
                    ctrl_context = &ubm_context->ctrl[dfc_context->ctrl_list[ctrl_index]];

                    if (MTB_UBM_DFC_PERST_OVERRIDE_AUTO == ctrl_context->features.perst_management_override)
                    {
                        mtb_en_ubm_lc_sts_t request_status;
                        uint8_t prev_pcie_reset_field = ctrl_context->scd[dfc_context->index].pcie_reset;

                        /* Automation release of the DFC_PERSTA/B signal */
                        ctrl_context->scd[dfc_context->index].pcie_reset = MTB_UBM_PCIE_RESET_FIELD_INIT;
                        request_status = mtb_ubm_process_pcie_reset_request(ubm_context,
                                                                            &ubm_context->hfc[ctrl_context->hfc_index],
                                                                            ctrl_context,
                                                                            dfc_context);

                        if (MTB_UBM_LC_STS_FAILED == request_status)
                        {
                            ctrl_context->scd[dfc_context->index].pcie_reset = prev_pcie_reset_field;
                        }
                    }
                }
            }
            else /* SRIS condition */
            {
                for (uint32_t ctrl_index = 0U; ctrl_index < dfc_context->ctrl_count; ctrl_index++)
                {
                    ctrl_context = &ubm_context->ctrl[dfc_context->ctrl_list[ctrl_index]];

                    if ((MTB_UBM_DFC_PERST_OVERRIDE_AUTO == ctrl_context->features.perst_management_override) ||
                        (MTB_UBM_DFC_PERST_OVERRIDE_NO_OVERRIDE == ctrl_context->features.perst_management_override))
                    {
                        mtb_en_ubm_lc_sts_t request_status;
                        uint8_t prev_pcie_reset_field = ctrl_context->scd[dfc_context->index].pcie_reset;

                        /* Automation release of the DFC_PERSTA/B signal */
                        ctrl_context->scd[dfc_context->index].pcie_reset = MTB_UBM_PCIE_RESET_FIELD_INIT;
                        request_status = mtb_ubm_process_pcie_reset_request(ubm_context,
                                                                            &ubm_context->hfc[ctrl_context->hfc_index],
                                                                            ctrl_context,
                                                                            dfc_context);

                        if (MTB_UBM_LC_STS_FAILED == request_status)
                        {
                            ctrl_context->scd[dfc_context->index].pcie_reset = prev_pcie_reset_field;
                        }
                    }
                    else
                    {
                        ctrl_context->scd[dfc_context->index].pcie_reset = MTB_UBM_PCIE_RESET_FIELD_HOLD;
                    }
                }
            }
        }
    }
    else
    {
        /* Reserved or Other SFF-TA-1002 drive facing connector not supported */
    }
}
