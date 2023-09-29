/***************************************************************************//**
 * \file mtb_ubm_config.h
 * \version 1.0
 *
 * \brief
 * Provides the UBM middleware compile time parameters
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

#if !defined(MTB_UBM_CONFIG_H)
#define MTB_UBM_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/** The maximun number of the HFCs supported */
#define MTB_UBM_HFC_MAX_NUM                     (4U)
/** The maximum number of the DFCs supported */
#define MTB_UBM_DFC_MAX_NUM                     (8U)
/** The maximum number of the routes supported */
#define MTB_UBM_ROUTES_MAX_NUM                  (32U)

/** Enable SES control application callback functionality */
#ifndef MTB_UBM_SES_CB_ACTIVE
    #define MTB_UBM_SES_CB_ACTIVE               (1U)
#endif /* MTB_UBM_SES_CB_ACTIVE */

/** Programmable update modes: */
/** Update mode not supported. */
#define MTB_UBM_UPDATE_NOT_SUPPORTED            (0U)
/** Update mode supported while devices remain online. UBM v0.5.0 does not support this option.
    If selected update mode will behave like option MTB_UBM_UPDATE_DEVICES_OFFLINE. */
#define MTB_UBM_UPDATE_DEVICES_ONLINE           (1U)
/** Update mode supported while devices are offline. */
#define MTB_UBM_UPDATE_DEVICES_OFFLINE          (2U)
/** Update mode support is vendor specific. */
#define MTB_UBM_UPDATE_VENDOR_SPECIFIC          (3U)
/** UBM Controller programming update mode capabilities */
#define MTB_UBM_UPDATE_MODE_CAPABILITIES  MTB_UBM_UPDATE_DEVICES_OFFLINE

#if (MTB_UBM_UPDATE_MODE_CAPABILITIES == MTB_UBM_UPDATE_DEVICES_ONLINE)
    #warning "UBM v1.0.0 does not support update while devices remain online"
#endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES == MTB_UBM_UPDATE_DEVICES_ONLINE) */

#if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
/** The starting address of the user application. This is the value from the MCUBoot
 *  configuration file ubm_flash_map/psoc62_swap_single_custom.json. */
#define MTB_UBM_UPGRADE_AREA_START_ADDRESS      (0x10018000U)

/** The size of the user application. This is the value from the MCUBoot
 *  configuration file ubm_flash_map/psoc62_swap_single_custom.json. */
#define MTB_UBM_UPGRADE_AREA_SIZE               (0x20000U)

/** The starting address of the upgrade image area. */
#define MTB_UBM_UPGRADE_IMAGE_START_ADDRESS     (MTB_UBM_UPGRADE_AREA_START_ADDRESS + MTB_UBM_UPGRADE_AREA_SIZE)
#endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */

/** Special chips are used to control the RefClk signal of the PCIe. The UBM
 * controller controls the logic outputs of these chips. For different chips,
 * the signal for switching on may be different. If this macro is true, then RefClk
 * is turned on by a high signal, if false, then by a low signal. */
#define MTB_UBM_SIGNAL_TO_ENABLE_REFCLK_MUX     (true)

#ifdef __cplusplus
}
#endif

#endif /* MTB_UBM_CONFIG_H */
