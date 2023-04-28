/***************************************************************************//**
 * \file mtb_ubm_config.h
 * \version 1.0
 *
 * \brief
 * Provides the UBM middleware compile time parameters
 *
 *******************************************************************************
 * (c) (2021-2022), Cypress Semiconductor Corporation (an Infineon company) or
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

/** The number of the HFCs in the backplane */
#define MTB_UBM_HFC_NUM       (4U)
/** The number of the DFCs in the backplane */
#define MTB_UBM_DFC_NUM       (4U)
/** The number of the routes in the backplane */
#define MTB_UBM_ROUTES_NUM    (32U)
/** Not supported. */
#define MTB_UBM_UPDATE_NOT_SUPPORTED    0U
/** Supported while devices remain online. UBM v0.5.0 does not support this option.
*   If select this option it will behave like options MTB_UBM_UPDATE_DEVICES_OFFLINE.
*/
#define MTB_UBM_UPDATE_DEVICES_ONLINE   1U
/** Supported while devices are offline. */
#define MTB_UBM_UPDATE_DEVICES_OFFLINE  2U
/** Support is vendor specific. */
#define MTB_UBM_UPDATE_VENDOR_SPECIFIC  3U 
/** UBM Controller programming update mode capabilities */
#define MTB_UBM_UPDATE_MODE_CAPABILITIES  MTB_UBM_UPDATE_DEVICES_OFFLINE

#if(MTB_UBM_UPDATE_MODE_CAPABILITIES == MTB_UBM_UPDATE_DEVICES_ONLINE)
    #warning "UBM v0.5.0 does not support update while devices remain online"
#endif

#if(MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
/** The starting address of the user application. This is the value from the MCUBoot
 *  configuration file ubm_flash_map/psoc62_swap_single_custom.json. */
#define MTB_UBM_UPGRADE_AREA_START      (0x10018000U)

 /** The size of the user application. This is the value from the MCUBoot
  *  configuration file ubm_flash_map/psoc62_swap_single_custom.json. */
#define MTB_UBM_UPGRADE_AREA_SIZE       (0x20000U)
#endif

#ifdef __cplusplus
}
#endif

#endif /* MTB_UBM_CONFIG_H */
