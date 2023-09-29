/***************************************************************************//**
 * \file mtb_ubm.h
 * \version 1.0
 *
 * \brief
 * Provides API declarations for the UBM middleware.
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
* The UBM middleware provides the implementation of the Universal Backplane Management (UBM) - a common backplane
* management framework for a host, to determine the SAS/SATA/PCIe backplane capabilities, Drive Facing Connector (DFC)
* Status, and Control information, and to read the port routing of the Drive Facing Connectors to
* Host Facing Connectors (HFC) of the backplane.
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
* See the \ref subsection_ubm_bootloader to start with the bootloader application and the \ref subsection_ubm_main
* for the main application.
*
* Refer to the <a href="https://members.snia.org/document/dl/27167"><b>SFF-TA-1005</b></a> for the UBM specification.
* Refer to the release notes for limitations and compatibility information.
*
* \subsection subsection_ubm_pcie_reset_clock_routing The control of the PCIe reset and clock routing
*
* The PCIe reset signal is controlled directly by the UBM controller. The PCIe RefClk signal is driven by
* a specific chip. (e.g., https://www.renesas.com/tw/en/document/dst/9db403d-datasheet?r=73671). For more details,
* refer to the backplane schematic. The UBM Middleware simply provides the control signal for the specific multiplexer.
* The pins for the above signals are assigned via the configuration structure.
*
* \subsection subsection_ubm_firmare_upload_host_tool The requirements to Firmware Host Tool.
* \note The UBM Middleware does not provide any Firmware Host Tool. This is a vendor-specific application
* to be implemented by the vendor side.
*
* As described later, the UBM middleware provides a transport layer for downloading a new firmware image.
* The Host update tool (software tool) is used for transfer. This instrument must meet the following requirements:
* 1. The Host tool supports the 2WIRE interface for communication with the UBM controller.
* 2. The update process complies with the procedure described in SFF-TA-1005 Rev 1.4, section 5.21.
* 3. The Host tool calculates the flash memory parameters based on the data received by
*    the "Get Non-Volatile Storage Geometry Subcommand".
* 4. The Host tool can parse binary, hexadecimal files into packets for transmission. According to
*    the UBM specification, the Host transmits sector indexes (flash memory lines) that can exceed the maximum length
*    of the UBM command. Sector indexes are divided into packets. For more details, see the "Program Subcommand"
*    description.
* 5. To record data in flash memory, pauses of 30 and 16 milliseconds are set for "Program Subcommand" and
*    "Erase Subcommand" respectively. For more details on the time parameters of operations with flash memory,
*    see the manual for the target device.
*    \note A pause for "Program Subcommand" is necessary only after the last data packet is received and
*    the index sector is completely formed.
*
* 6. The UBM specification contains "Verify Image Subcommand" but it does not define the verification method.
*    A checksum check of the entire application, including additional areas of the MCUBoot bootloader, is applied
*    in the middleware. The Host tool calculates the checksum of the binary file previously signed by MCUBoot. Write
*    the calculated CRC bytes at address BASE ADDRESS + 0x200. The CRC endianness must be little. The following is
*    the function for calculating the checksum on Python:
* \code {.py}
* # calculate_checksum: Calculate checksum CRC32 algorithm.
* # parameters:
* # ih - (input hex) This is the array of the binary data of the
* #       image.
* # len - length of the image.
* def calculate_checksum(ih, len):
*     i = 1
*     crc = 0xFFFFFFFF
*
*     crcTable =\
*     [
*         0x00000000, 0x105ec76f, 0x20bd8ede, 0x30e349b1,
*         0x417b1dbc, 0x5125dad3, 0x61c69362, 0x7198540d,
*         0x82f63b78, 0x92a8fc17, 0xa24bb5a6, 0xb21572c9,
*         0xc38d26c4, 0xd3d3e1ab, 0xe330a81a, 0xf36e6f75,
*     ]
*
*     for i in range(len):
*         crc = crc ^ ih[i]
*         crc = (crc >> 4) ^ crcTable[crc & 0xF]
*         crc = (crc >> 4) ^ crcTable[crc & 0xF]
*     crc = ~crc
*     return crc & 0xffffffff
* \endcode
*
* \section section_ubm_quick_start Quick Start Guide
* \subsection subsection_ubm_bootloader Basic UBM application with Update mode
* The UBM Middleware supports Update mode. This feature is based on the
* MCUBoot bootloader, and the transport layer, which provides the UBM Middleware for
* transferring a new application image to the slot.
* \note Strongly recommended - learn more about MCUBoot and multi-application projects from
*       the following sources:
*       1. [MCUBoot documentation](https://github.com/mcu-tools/mcuboot/tree/main/docs)
*       2. [MCUBoot/Infineon](https://github.com/mcu-tools/mcuboot/tree/cypress/boot/cypress/MCUBootApp)
*       3. [MCUBoot basic application](https://github.com/Infineon/mtb-example-psoc6-mcuboot-basic)
*       4. [Dual application guide](https://documentation.infineon.com/html/psoc6/yvt1667482437523.html)
*
* Use this [guide](https://github.com/Infineon/cce-mtb-psoc61-mcuboot-bootloader) to create two applications:
* - The bootloader - a program that checks the slots in the flash memory and downloads a new firmware image
*   when it finds one.
* - The blinky application - the user's main application that will be updated to run the UBM Middleware.
*
* After creating a sample project:
* - Apply the modifications described later in this section to update the "bootloader" and "blinky" application.
* - Update the main.c file of the "blinky" application with the alterations outlined in the following section
*   \ref subsection_ubm_main.
*
* 1. Include the UBM middleware into the project using the ModusToolbox&trade; Library Manager.
* 2. Copy the configuration file from <em> mtb_shared/ubm/release[version]/ubm_flash_map/[config].json </em> to
*    the Bootloader application
*    \a "flashmap" folder.
* 3. Update the FLASH_MAP variable in \a "shared_config.mk" to the previously copied JSON file
* ~~~ makefile
* # Flashmap JSON file name
* FLASH_MAP=psoc62_swap_single_custom.json
* ~~~
* 4. Update the configurational defines in the configurational file
*    in \a [app_name]/blinky/imports/ubm/mtb_ubm_config.h:
* \note The configurational file mtb_ubm_config.h is added to the Blinky application
*       in the folder "imports" after the UBM middleware is added using the Library Manager and
*       the following values are auto-updated:
*       \code #define MTB_UBM_UPGRADE_AREA_START_ADDRESS (0x10018000U) \endcode
*       \code #define MTB_UBM_UPGRADE_AREA_SIZE          (0x20000U) \endcode
*       Update these addresses when the JSON configuration file is changed.
*
* 5. Update the main makefile of the Blinky application - modify the rule for the post-build processing:
* ~~~ makefile
* POSTBUILD=\
* cp -f $(BINARY_OUT_PATH).hex $(BINARY_OUT_PATH)_raw.hex;\
* rm -f $(BINARY_OUT_PATH).hex;\
* $(CY_ELF_TO_HEX_TOOL) --change-addresses=$(HEADER_OFFSET) --remove-section .cy_em_eeprom $(CY_ELF_TO_HEX_OPTIONS) $(BINARY_OUT_PATH).elf $(BINARY_OUT_PATH)_unsigned.hex;\
* $(PYTHON_PATH) $(IMGTOOL_PATH) $(SIGN_ARGS) $(BINARY_OUT_PATH)_unsigned.hex $(BINARY_OUT_PATH).hex;
* ~~~
* \warning Delete the cy_em_eeprom section because it can break the signing process.
* \warning Known issue: The main Makefile of Blinky application has one incorrect path. Change this path:
* ~~~ makefile
* ALL_FILES=*
* # Correct path:
* SOURCES+=$(wildcard $(SEARCH_mcuboot)/boot/bootutil/src/$(ALL_FILES).c)\
*          $(wildcard $(MCUBOOT_CY_PATH)/libs/watchdog/$(ALL_FILES).c)\
*          $(wildcard $(MCUBOOT_CY_PATH)/platforms/img_confirm/$(FAMILY)/$(ALL_FILES).c)
* ~~~
* 6. Update the \a main.c file in the Blinky application as described in \ref subsection_ubm_main.
* \subsection subsection_ubm_main UBM Controller Application
* 1. When the UBM middleware is added to the project, the configurational file \a mtb_ubm_config.h is added to
*    the Blinky application to the \a "imports" folder.
*    This file contains the main configurational defines, e.g. the number of the HFCs or the number of DFCs, etc.
*    Update the necessary parameters according to your backplane configuration.
*    An example for 2 HFCs, 2 DFCs, and 2 Routes:
*        \snippet guide_snippet/main.c snippet_ubm_application_config
*
* 2. Include the UBM middleware header file in your \a main.c file:
*        \snippet guide_snippet/main.c snippet_ubm_application_include
*
* 3. Add the mtb_ubm_init() function call in your \a main.c file, for example:
*        \snippet guide_snippet/main.c snippet_mtb_ubm_init
*
* 4. The following parameters are required for the mtb_ubm_init() function:
*        \snippet guide_snippet/main.c snippet_mtb_ubm_param
*
* 5. The first parameter mtb_stc_ubm_backplane_cfg_t contains the configurational values required to set up
*    the UBM middleware, for example:
*        \snippet guide_snippet/main.c snippet_ubm_application_backplane_config
*    The first parameter requires configuring the Emulated EEPROM as a storage for FRU data. For more details,
*    refer to the <a href="https://github.com/Infineon/emeeprom">Emulated EEPROM Middleware Library</a> documentation.
*    An example of the configuration:
*        \snippet guide_snippet/main.c snippet_ubm_application_eeprom
*    Then, set the the overview area parameters as desired. For the the parameters details, refer to
*    the UBM Overview Area section in the <a href="https://members.snia.org/document/dl/27167">SFF-TA-1005</a>
*    specification. An example of the configuration:
*        \snippet guide_snippet/main.c snippet_ubm_application_overview_area
*    For the explanation of the Silicon Identity, Backplane Info, and Capabilities parameters, refer to
*    the corresponding section of the <a href="https://members.snia.org/document/dl/27167">SFF-TA-1005</a>
*    specification.
*
* 6. The second parameter mtb_stc_ubm_backplane_control_signals_t contains the GPIO definition for HFC and DFC IO
*    signals. The number of elements of this configurational array must match the number of HFCs and the number of
*    DFCs configured. These signal definitions are a combination of
*    <a href="https://members.snia.org/document/dl/27167">SFF-TA-1005</a>,
*    <a href="https://members.snia.org/document/dl/26489">SFF-8639</a>,
*    <a href="https://members.snia.org/document/dl/26900">SFF-TA-1001</a>,
*    <a href="https://members.snia.org/document/dl/26837">SFF-TA-1009</a>,
*    <a href="https://webstore.ansi.org/standards/incits/ansiincits3762003">SAS</a> and
*    <a href="https://sata-io.org/">SATA</a> standards.
*    Change the pin settings to match your specification. An example of the configuration:
*    \snippet guide_snippet/main.c snippet_ubm_application_control_signals
*    \warning Do not use the selected IO pins elsewhere in the application. Verify that the selected IO pins
*             are not used in the ModusToolbox&trade; Device Configurator.
*    \warning The mtb_ubm_init() function can detect if the IO pin was already initialized with the HAL function
*             cyhal_gpio_init() and returned the appropriate return code, but it cannot detect if the IO pin
*             was initialized by the Peripheral Driver Library directly.
*
* 7. The third parameter mtb_stc_ubm_context_t contains internal middleware data structures and will not be changed
*    by the application.
*    \note Keep the context variable accessible during the whole application execution time.
*
* 8. Your application configuration is complete for building and programmed using the ModusToolbox&trade; software.
*    For the error codes returned by the mtb_ubm_init() function, refer to \ref mtb_en_ubm_status_t.
*
* \subsection subsection_led_indication LED indication
* 
* The current implementation of the UBM Middleware does not include the LED blinking algorithm but assumes
* it is implemented by the application itself.
* 
* The DFC Status and Control Descriptor command contains the SES Array Device Slot Element, which can be used
* as an input to an LED blinking algorithm. So, the UBM Middleware has an optional functionality to notify
* the application when a valid SES Array Device Slot Control Element was received.
* 
* To enable this feature:
* 1. Set the related macro in \a mtb_ubm_config.h:
* \code {.c}
* #define MTB_UBM_SES_CB_ACTIVE           (1)
* \endcode
* 
* 2. Declare the application handler of the \ref mtb_ubm_ses_app_cb_t type in your application:
*    \snippet main.c snippet_ubm_app_ses_handler_declaration
* 
* 3. Configure the application handler in the mtb_stc_ubm_backplane_cfg_t configurational structure:
* \code {.c}
* ubm_backplane_configuration.ses_event_handler = ubm_app_handler;
* \endcode
* 
* Now, the configured handler function will be called every time a valid SES Array Device Slot Control Element
* is received.
* The handler argument \ref mtb_stc_ubm_ses_app_cb_context_t contains:
* - The DFC index of the DFC Status and Control Descriptor command, 
* - The pointer to the received SES Array Device Slot Control Element,
* - The pointer to the SES Array Device Slot Status Element.
* 
* If the application needs/updates the SES Array Device Slot Status Element for the following read back, the handler
* function will return \a true. The SES Array Device Slot Status is ignored and not saved if the handler function
* returns \a false.
* 
* For the implementation example of the applicaion handler, refer to a corresponding code example.
* 
* \section section_ubm_commands UBM Controller Commands
*
* Each of added to the configuration Controllers supports the following commands:
* <table>
* <caption id="multi_row">UBM Controller Commands</caption>
* <tr><th>Command code                      <th>Read/Write        <th>Command name  <th>Description
* <tr><td colspan="4" align="center">Generic commands</td></tr>
* <tr><td align="center">00h <td>Read only <td>Operational State <td>Returns the operating state of the UBM Controller.
* <tr><td align="center">01h <td>Read only <td>Last Command Status <td>Returns the last command execution status
*                                                                      of the UBM Controller.
* <tr><td align="center">02h <td>Read only <td>Silicon Identity and Version <td>Returns UBM Controller
*                                                                               identification data.
* <tr><td align="center">03h <td>Read only <td>Programming Update Mode Capabilities <td>Returns the Programming
*                                                                                       Update Mode capabilities of
*                                                                                       the UBM Controller.
* <tr><td colspan="4" align="center">Programmable Update Mode Commands</td></tr>
* <tr><td align="center">20h <td>Read/Write <td>Enter Programmable Update Mode <td>Indicates the sequence to unlock
*                                                                                  and transfer to
*                                                                                  Programmable Update Mode.
* <tr><td align="center">21h <td>Read/Write <td>Programmable Mode Data Transfer <td>Indicates the method to exchange
*                                                                                   multiple bytes of the command,
*                                                                                   status, and data.
* <tr><td align="center">22h <td>Read/Write <td>Exit Programmable Update Mode <td>Indicates to transfer out of
*                                                                                 Programmable Update Mode.
* <tr><td colspan="4" align="center">Backplane Management Commands</td></tr>
* <tr><td align="center">30h <td>Read only <td>Host Facing Connector Info <td>Returns the Host Facing Connector
*                                                                             information.
* <tr><td align="center">31h <td>Read only <td>Backplane Info <td>Returns the backplane number and type,
*                                                                 which are unique in the chassis.
* <tr><td align="center">32h <td>Read only <td>Starting Slot <td>Returns the Starting Slot, which is applied to
*                                                                the Slot Offset found in the UBM Port Route
*                                                                Information of the UBM FRU.
* <tr><td align="center">33h <td>Read only <td>Capabilities <td>Returns the backplane capabilities.
* <tr><td align="center">34h <td>Read only <td>Features <td>Returns the backplane features.
* <tr><td align="center">35h <td>Read/Write <td>Change Count <td>The counter used to manage UBM Controller interrupts.
* <tr><td align="center">36h <td>Read/Write <td>DFC Status and Control Descriptor Index <td>Controls the DFC Status
*                                                                                           and Control Descriptor
*                                                                                           to access.
* <tr><td align="center">40h <td>Read/Write <td>DFC Status and Control Descriptor <td>Indicates the DFC Status and
*                                                                                     Control Descriptor data for the
*                                                                                     current DFC Status and Control
*                                                                                     Descriptor Index.
* </table>
* All commands implemented per <a href="https://members.snia.org/document/dl/27167">SFF-TA-1005 Rev 1.4</a>.
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
* This section describes the data structures defined by the UBM.
* \{
*    \defgroup group_ubm_io_signals IO HFC/DFC signals
*    \brief
*    This section describes the Input and Output HFC/DFC signals.
*    \defgroup group_ubm_fru_data_structures FRU Data Structures
*    \brief
*    This section describes the data types defined by the UBM FRU.
*    \defgroup group_ubm_ifc_data_structures IFC Data Structures
*    \brief
*    This section describes the data types defined by the UBM IFC.
*    \defgroup group_ubm_ifc_callback_type IFC Callback Type
*    \brief
*    This section describes the callbacks types defined by the UBM IFC.
* \}
* \defgroup group_ubm_macros Macroses
* \brief
* This section describes the Macroses defined by the UBM.
* \{
*    \defgroup group_ubm_macros_fru FRU defines
*    \brief
*    This section describes the Macroses defined by the UBM for FRU options.
*    \defgroup group_ubm_macros_fru_oa FRU Overview Area defines
*    \brief
*    This section describes the Macroses defined by the UBM for FRU Overview Area options.
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
#define MTB_UBM_SPEC_VER            (0x14U)

/** @} group_ubm_macros */

/**
 * \addtogroup group_ubm_functions
 * @{
 */

mtb_en_ubm_status_t mtb_ubm_init(const mtb_stc_ubm_backplane_cfg_t* config,
                                 const mtb_stc_ubm_backplane_control_signals_t* signals,
                                 mtb_stc_ubm_context_t* ubm_context);

/** @} group_ubm_functions */

#ifdef __cplusplus
}
#endif

#endif /* MTB_UBM_H */
