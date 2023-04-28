/***************************************************************************//**
 * \file mtb_ubm.h
 * \version 0.5
 *
 * \brief
 * Provides the UBM middleware API declarations.
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
#ifndef MTB_UBM_H
#define MTB_UBM_H

/**
* \mainpage
* \section section_mainpage_overview Overview
* The implementation of the UBM middleware for ModusToolbox&trade; is currently 
* at the Preliminary level. Features may change without notice.
* Contact <a href="https:/\/www.infineon.com/cms/en/about-infineon/company/contacts/support/">
* <b>Infineon Product Support</b></a> for additional details.
*
* The UBM middleware provides the implementation of the Universal Backplane Management (UBM) - a common backplane management framework
* for a host to determine SAS/SATA/PCIe backplane capabilities, Drive Facing Connector (DFC) Status and Control information, and to read
* the port routing of the Drive Facing Connectors to Host Facing Connectors (HFC) of the backplane.
* 
* <b>Features:</b>
* * Status and Control over Drive Facing Connector I/O
* * High speed lane port routing assignments to Host Facing Connectors
* * Backplane capabilities including:
*     * PCIe Reference Clock expectations
*     * PCIe Reset expectations
*     * Power Disable support
*     * Dual Port support
* * Controller programmable firmware update
* 
* \section section_ubm_general General Description
* 
* Include mtb_ubm.h to get access to all the functions and other declarations in this middleware.
*
* See the \ref subsection_ubm_bootloader to start with the bootloader application and the \ref subsection_ubm_main for the main application.
*
* Refer to the <a href="https://members.snia.org/document/dl/27167"><b>SFF-TA-1005</b></a> for the UBM specification. Refer to the release notes for limitations and compatibility information.
* 
* \section section_ubm_quick_start Quick Start Guide
* \subsection subsection_ubm_bootloader Basic UBM application with Update mode
* The UBM Middleware supports Update mode. This feature is based on the 
* MCUBoot bootloader, and the transport layer, which provides UBM Middleware for 
* transferring a new application image to the slot.
* \note Strongly recommended - learn more about MCUBoot and multi-application projects from 
*       the following sources:
*       1. [MCUBoot documentation](https://github.com/mcu-tools/mcuboot/tree/main/docs)
*       2. [MCUBoot/Infineon](https://github.com/mcu-tools/mcuboot/tree/cypress/boot/cypress/MCUBootApp)
*       3. [MCUBoot basic application](https://github.com/Infineon/mtb-example-psoc6-mcuboot-basic)
*       4. [Dual application guide](https://www.infineon.com/dgdl/Infineon-AN215656_PSoC_6_MCU_Dual-CPU_System_Design-ApplicationNotes-v09_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0d3180c4655f&utm_source=cypress&utm_medium=referral&utm\_campaign=202110_globe_en_all_integration-application_note)
*
* Using this [guide](https://github.com/Infineon/cce-mtb-psoc61-mcuboot-bootloader) to create two applications:
* - The bootloader - is a program that checks the slots in the flash memory and downloads a new firmware image if it finds one.
* - The blinky application - is the user's main application that will be updated to run UBM Middleware.
* 
* After creating a sample project: 
* - Update “bootloader” application with the changes mentioned in "Basic UBM application with Update mode".
* - Replace main.c of “blinky” application with the changes mentioned in "UBM Controller Application"
* 
* 1. Include the UBM middleware into the project using the ModusToolbox&trade; Library Manager.
* 2. Copy the configuration file from <em> mtb_shared/ubm/release[version]/ubm_flash_map/[config].json </em> to the Bootloader application
*    \a "flashmap" folder.
* 3. Update FLASH_MAP variable in \a "shared_config.mk" to previously copied JSON file
* ~~~ makefile
* # Flashmap JSON file name
* FLASH_MAP=psoc62_swap_single_custom.json
* ~~~
* 4. Update configurational defines in configurational file present in \a [app_name]/blinky/imports/ubm/mtb_ubm_config.h:
*
* \note The configurational file mtb_ubm_config.h is added to the Blinky application 
*       in folder "imports" after the UBM middleware is added using Library Manager and 
*       below values are auto updated,
*       \code #define MTB_UBM_UPGRADE_AREA_START      (0x10018000U) \endcode
*       \code #define MTB_UBM_UPGRADE_AREA_SIZE          (0x20000U) \endcode
*       You should update these addresses when json configuration file is changed.
*       
* 5. Update the main makefile of Blinky application - modify a rule for post-build process:
* ~~~ makefile
* POSTBUILD=\
* cp -f $(BINARY_OUT_PATH).hex $(BINARY_OUT_PATH)_raw.hex;\
* rm -f $(BINARY_OUT_PATH).hex;\
* $(CY_ELF_TO_HEX_TOOL) --change-addresses=$(HEADER_OFFSET) --remove-section .cy_em_eeprom $(CY_ELF_TO_HEX_OPTIONS) $(BINARY_OUT_PATH).elf $(BINARY_OUT_PATH)_unsigned.hex;\
* $(PYTHON_PATH) $(IMGTOOL_PATH) $(SIGN_ARGS) $(BINARY_OUT_PATH)_unsigned.hex $(BINARY_OUT_PATH).hex;
* ~~~
* \warning The cy_em_eeprom section must be deleted because it can break the signing process.
* \warning Known issue: The main Makefile of Blinky application has one incorrect path. Change this path:
* ~~~ makefile
* ALL_FILES=*
* # Correct path:
* SOURCES+=$(wildcard $(SEARCH_mcuboot)/boot/bootutil/src/$(ALL_FILES).c)\
*          $(wildcard $(MCUBOOT_CY_PATH)/libs/watchdog/$(ALL_FILES).c)\
*          $(wildcard $(MCUBOOT_CY_PATH)/platforms/img_confirm/$(FAMILY)/$(ALL_FILES).c)
* ~~~
* 6. Update the \a main.c file in Blinky application as it described \ref subsection_ubm_main.
* \subsection subsection_ubm_main UBM Controller Application
* 1.  When the UBM middleware was added into the project the configurational file \a mtb_ubm_config.h was added to the Blinky application into the \a "imports" folder.
*    This file contains main configurational defines, e.g. the number of the HFCs or the number of DFCs, etc.
*    Update the necessary parameters according to your backplane configuration. Example for 2 HFCs, 2 DFCs and 2 Routes:
*        \snippet guide_snippet/main.c snippet_ubm_application_config
* 
* 2. Include the UBM middleware header file in your \a main.c file:
*        \snippet guide_snippet/main.c snippet_ubm_application_include
* 
* 3. Add the mtb_ubm_init() function call in your \a main.c file, for example:
*        \snippet guide_snippet/main.c snippet_mtb_ubm_init
* 
* 4. The following parameters are requred for the mtb_ubm_init() function:
*        \snippet guide_snippet/main.c snippet_mtb_ubm_param
* 
* 5. The first parameter mtb_stc_ubm_backplane_cfg_t contains the configurational values requred to set up the UBM middleware, e.g:
*        \snippet guide_snippet/main.c snippet_ubm_application_backplane_config
*    The first parameter requires configuring the Emulated EEPROM as a storage for FRU data. For more details, please refer to Em_EEPROM API manual. An example of the configuration:
*        \snippet guide_snippet/main.c snippet_ubm_application_eeprom
*    Then, set the the overview area parameters as desired. For the the parameters details, refer to the UBM Overview Area section
*    in the <a href="https://members.snia.org/document/dl/27167">SFF-TA-1005</a> specification. An example of the configuration:
*        \snippet guide_snippet/main.c snippet_ubm_application_overview_area
*    For the explanation of the Silicon Identity, Backplane Info and Capabilities parameters, refer to the corresponding section of the <a href="https://members.snia.org/document/dl/27167">SFF-TA-1005</a> specification.
*    Also, you can set parameters related to the HPT0 signal support and starting slot in the configurational structure.
*    \note While this Beta release does not support the Port Route Descriptors it is required to configure the UBM controller 2wire address parameter.
* 
* 6. The second parameter mtb_stc_ubm_backplane_control_signals_t contains the GPIO definition for HFC and DFC IO signals.
*    The number of elements of this configurational array should match the number of HFCs and the number of DFCs configured.
*    This signal definitions are a combination of <a href="https://members.snia.org/document/dl/27167">SFF-TA-1005</a>, <a href="https://members.snia.org/document/dl/26489">SFF-8639</a>,
*    <a href="https://members.snia.org/document/dl/26900">SFF-TA-1001</a>, <a href="https://members.snia.org/document/dl/26837">SFF-TA-1009</a>,
*    <a href="https://webstore.ansi.org/standards/incits/ansiincits3762003">SAS</a> and <a href="https://sata-io.org/">SATA</a> standards.
*    Change the pin settings to match your specifications. An example of the configuration:
*    \snippet guide_snippet/main.c snippet_ubm_application_control_signals
*    \warning Do not use the selected IO pins elsewhere in the application. Verify that the selected IO pins are not used in the ModusToolbox&trade; Device Configurator.
              Otherwise this can lead to an unexpected behaviour of the application.
* 
* 7. The third parameter mtb_stc_ubm_context_t contains internal middleware data structures and is not intended to be changed by the application.
*    \note Keep the context variable accessible during the whole application execution time.
* 
* 8. At this step, your application configuration is complete to be built and programmed using the ModusToolbox&trade; software.
* 
* \defgroup group_ubm_functions Functions
* \brief
* This section describes the UBM Function Prototypes.
* 
* \defgroup group_ubm_enums Enumerated Types
* \brief
* This section describes the enumeration types defined by the UBM.
*
* \defgroup group_ubm_data_structures Data Structures
* \brief
* This section describes the enumeration types defined by the UBM.
* \{
*    \defgroup group_ubm_fru_data_structures FRU Data Structures
*    \brief
*    This section describes the data types defined by the UBM FRU.
* \}
* \defgroup group_ubm_macros Macroses
* \brief
* This section describes the Macroses defined by the UBM.
* \{
*    \defgroup group_ubm_macros_fru FRU defines
*    \brief
*    This section describes the Macroses defined by the UBM for FRU options.
*    \{
*       \defgroup group_ubm_macros_fru_oa FRU Overview Area defines
*       \brief
*       This section describes the Macroses defined by the UBM for FRU Overview Area options.
*    \}
* \}
*/

#ifdef __cplusplus
extern "C" {
#endif

#include "mtb_ubm_types.h"
#include "mtb_ubm_ifc.h"
#include "mtb_ubm_fru.h"

/**
 * \addtogroup group_ubm_macros
 * @{
 */

/** UBM specification version */
#define MTB_UBM_SPEC_VER            (0x14u)

/** @} group_ubm_macros */

/**
 * \addtogroup group_ubm_functions
 * @{
 */

bool mtb_ubm_init(const mtb_stc_ubm_backplane_cfg_t* config, const mtb_stc_ubm_backplane_control_signals_t* signals, mtb_stc_ubm_context_t* context);

/** @} group_ubm_functions */

#ifdef __cplusplus
}
#endif

#endif /* MTB_UBM_H */
