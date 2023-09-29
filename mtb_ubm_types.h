/***************************************************************************//**
 * \file mtb_ubm_types.h
 * \version 1.0
 *
 * \brief
 * Provides declarations for the UBM middleware API types.
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


#ifndef MTB_UBM_TYPES_H
#define MTB_UBM_TYPES_H

#include <stdint.h>
#include <stdbool.h>

#include "cyhal.h"  /* cyhal_gpio_t, cyhal_gpio_t, cy_stc_sysint_t, and cyhal_uart_t */
#include "cy_result.h"
#include "cyhal_gpio.h"
#include "cy_em_eeprom.h"
#include "cyhal_flash.h"
#include "mtb_ubm_config.h"


/** The size of the I2C slave buffer to which the master writes data.*/
#define MTB_UBM_I2C_WRITE_BUFFER_SIZE                       (256U)

/** The size of the I2C slave buffer from which the master reads data.*/
#define MTB_UBM_I2C_READ_BUFFER_SIZE                        (256U)

/** The maximum number of the controllers supported */
#define MTB_UBM_CTRLS_MAX_NUM                               (MTB_UBM_DFC_MAX_NUM)

/** The number of the DFC domains */
#define MTB_UBM_DOMAINS_MAX_NUM                             (2U)

/** The size of the SES Array Device Slot Element field */
#define MTB_UBM_SES_ARRAY_DEVICE_SLOT_ELEMENT_SIZE          (4U)


/** UBM Controller commands */
/** Returns the operating state of the UBM Controller. */
#define MTB_UBM_CMD_OPERATIONAL_STATE                       (0x00U)

/** Returns the last command execution status of the UBM Controller. */
#define MTB_UBM_CMD_LAST_COMMAND_STATUS                     (0x01U)

/** Returns UBM Controller identification data. */
#define MTB_UBM_CMD_SILICON_IDENTITY_AND_VERSION            (0x02U)

/** Returns the Programmable Update Mode capabilities of the UBM Controller. */
#define MTB_UBM_CMD_PROGRAMMABLE_UPDATE_MODE_CAPABILITIES   (0x03U)

/** Indicates a sequence to unlock, and then transfer to Programmable Update Mode. */
#define MTB_UBM_CMD_ENTER_PROGRAMMABLE_UPDATE_MODE          (0x20U)

/** Indicates the method to exchange multiple bytes of a command, status, and data. */
#define MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER         (0x21U)

/** Indicates to transfer out of Programmable Update Mode. */
#define MTB_UBM_CMD_EXIT_PROGRAMMABLE_UPDATE_MODE           (0x22U)

/** Returns the Host Facing Connector information. */
#define MTB_UBM_CMD_HOST_FACING_CONNECTOR_INFO              (0x30U)

/** Returns the backplane number and type, which is unique in the chassis. */
#define MTB_UBM_CMD_BACKPLANE_INFO                          (0x31U)

/** Returns the Starting Slot applied to the Slot Offset
 * found in the UBM Port Route Information of the UBM FRU.
 */
#define MTB_UBM_CMD_STARTING_SLOT                           (0x32U)

/** Returns the backplane capabilities. */
#define MTB_UBM_CMD_CAPABILITIES                            (0x33U)

/** Indicates the UBM Controller features. */
#define MTB_UBM_CMD_FEATURES                                (0x34U)

/** The counter used to manage UBM Controller interrupts. */
#define MTB_UBM_CMD_CHANGE_COUNT                            (0x35U)

/** Controls the DFC Status and Control Descriptor to access. */
#define MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR_INDEX (0x36U)

/** Indicates the DFC Status and Control Descriptor data
 * for the current DFC Status and Control Descriptor Index.
 */
#define MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR       (0x40U)


/** The type for the UBM Controller commands */
typedef uint8_t mtb_ubm_cmd_t;


/** UBM Controller Programmable mode subcommands */
/** Not valid Programmable mode subcommand */
#define MTB_UBM_PM_CMD_INVALID                              (0x00U)

/** Returns the non-volatile structure and size of programmable segments. */
#define MTB_UBM_PM_CMD_GET_NON_VOLATILE_STORAGE_GEOMETRY    (0x01U)

/** Erases a segment of the non-volatile location to prepare for programming. */
#define MTB_UBM_PM_CMD_ERASE                                (0x02U)

/** Returns the status of an erase request. */
#define MTB_UBM_PM_CMD_ERASE_STATUS                         (0x03U)

/** Writes a segment of data into the non-volatile location. */
#define MTB_UBM_PM_CMD_PROGRAM                              (0x04U)

/** Returns the status of a program request. */
#define MTB_UBM_PM_CMD_PROGRAM_STATUS                       (0x05U)

/** Sets the Sector and Sector Index for a Verify Status request. */
#define MTB_UBM_PM_CMD_VERIFY                               (0x06U)

/** Returns the checksum for the non-volatile segment. */
#define MTB_UBM_PM_CMD_VERIFY_STATUS                        (0x07U)

/** Sets the Image Number for an Image Number Status request. */
#define MTB_UBM_PM_CMD_VERIFY_IMAGE                         (0x08U)

/** Returns information indicating the UBM Controller Image is valid. */
#define MTB_UBM_PM_CMD_VERIFY_IMAGE_STATUS                  (0x09U)

/** If multiple UBM Controller Images are supported, this command is used
 * to set the next image to use.
 */
#define MTB_UBM_PM_CMD_SET_ACTIVE_IMAGE                     (0x0AU)

/** Returns the status of a set active image request. */
#define MTB_UBM_PM_CMD_ACTIVE_IMAGE_STATUS                  (0x0BU)

/** UBM controller capabilities */
/** Clock Routing support */
#define MTB_UBM_CAP_CLOCK_ROUTING                           (0x01U)
/** Power control support */
#define MTB_UBM_CAP_SLOT_POWER_CONTROL                      (0x02U)
/** Reset control support */
#define MTB_UBM_CAP_PCIE_RESET_CONTROL                      (0x04U)
/** If Dual Port DFC connectors routes are supported */
#define MTB_UBM_CAP_DUAL_PORT                               (0x08U)

/** 2Wire Slave Reset is not supported */
#define MTB_UBM_CAP_2WIRE_RESET_OP_NOT_SUPPORTED            (0x00U)
/** 2Wire Slave Reset and 2Wire Mux is supported */
#define MTB_UBM_CAP_2WIRE_RESET_OP_2WIRE_AND_MUX            (0x01U)
/** UBM FRU and UBM Controller is supported */
#define MTB_UBM_CAP_2WIRE_RESET_OP_FRU_CONTROLLER           (0x02U)
/** 2Wire Slave Reset and UBM FRU and UBM Controller and
 * 2Wire Mux are supported */
#define MTB_UBM_CAP_2WIRE_RESET_OP_2W_FRU_CONTROLLER_MUX    (0x03U)

/** CHANGE_DETECT# Interrupt Operation support */
#define MTB_UBM_CAP_CHANGE_DET_INT_OP                       (0x40U)
/** DFC Change Count operation support */
#define MTB_UBM_CAP_CHANGE_COUNT                            (0x80U)
/** PRSNT# Reported */
#define MTB_UBM_CAP_PRSNT_REPORT                            (0x100U)
/** IFDET# Reported */
#define MTB_UBM_CAP_IFDET_REPORT                            (0x200U)
/** IFDET2# Reported */
#define MTB_UBM_CAP_IFDET2_REPORT                           (0x400U)
/** DFC PERST# Management Override Supported */
#define MTB_UBM_CAP_DFC_PERST                               (0x800U)
/** DFC SMBus Reset Control Supported */
#define MTB_UBM_CAP_DFC_SMBUS_RESET                         (0x1000U)


/** The type for UBM Controller programmable mode subcommands */
typedef uint8_t mtb_ubm_pm_cmd_t;


/**
 * \addtogroup group_ubm_enums
 * @{
 */


/** The status of initialization of the UBM middleware */
typedef enum
{
    /** Operation completed successfully */
    MTB_UBM_STATUS_SUCCESS = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                            CY_RSLT_MODULE_MIDDLEWARE_UBM, 0U),

    /** The number of HFCs configured is 0 or higher than MTB_UBM_HFC_MAX_NUM.
     * Check the \ref mtb_stc_ubm_backplane_cfg_t.num_of_hfc parameter setting. */
    MTB_UBM_STATUS_HFC_NUM_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                CY_RSLT_MODULE_MIDDLEWARE_UBM, 1U),

    /** The number of DFCs configured is 0 or higher than MTB_UBM_DFC_MAX_NUM.
     * Check the \ref mtb_stc_ubm_backplane_cfg_t.num_of_dfc parameter setting. */
    MTB_UBM_STATUS_DFC_NUM_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                CY_RSLT_MODULE_MIDDLEWARE_UBM, 2U),

    /** The number of routes configured is 0 or higher than MTB_UBM_ROUTES_MAX_NUM.
     * Check the \ref mtb_stc_ubm_backplane_cfg_t.num_of_routes parameter setting. */
    MTB_UBM_STATUS_ROUTES_NUM_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                   CY_RSLT_MODULE_MIDDLEWARE_UBM, 3U),

    /** The HFC IO configurational structure is not complete.
     * Check the \ref mtb_stc_ubm_backplane_control_signals_t.hfc_io parameter array
     * has enough initialization elements, \n as configured by
     * the \ref mtb_stc_ubm_backplane_cfg_t.num_of_hfc parameter. */
    MTB_UBM_STATUS_HFC_IO_CONF_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                    CY_RSLT_MODULE_MIDDLEWARE_UBM, 4U),

    /** The DFC IO configurational structure is not complete.
     * Check the \ref mtb_stc_ubm_backplane_control_signals_t.dfc_io parameter array
     * has enough initialization elements, \n as configured by
     * the \ref mtb_stc_ubm_backplane_cfg_t.num_of_dfc parameter. */
    MTB_UBM_STATUS_DFC_IO_CONF_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                    CY_RSLT_MODULE_MIDDLEWARE_UBM, 5U),

    /** The route information configurational structure is not complete.
     * Check the \ref mtb_stc_ubm_backplane_cfg_t.route_information parameter array
     * has enough initialization elements, \n as configured by
     * the \ref mtb_stc_ubm_backplane_cfg_t.num_of_routes parameter. */
    MTB_UBM_STATUS_ROUTES_CONF_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                    CY_RSLT_MODULE_MIDDLEWARE_UBM, 6U),

    /** PRSNT# The reported capability is enabled but the PRSNT pin is not configured.
     * Check the \ref mtb_stc_ubm_dfc_signals_t.prsnt pin settings,
     * or disable the \ref mtb_stc_ubm_capabilities_t.prsnt_reported setting. */
    MTB_UBM_STATUS_CAP_PRSNT_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                  CY_RSLT_MODULE_MIDDLEWARE_UBM, 7U),

    /** IFDET# The reported capability is enabled but the IFDET pin is not configured.
     * Check the \ref mtb_stc_ubm_dfc_signals_t.ifdet pin settings,
     * or disable the \ref mtb_stc_ubm_capabilities_t.ifdet_reported setting. */
    MTB_UBM_STATUS_CAP_IFDET_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                  CY_RSLT_MODULE_MIDDLEWARE_UBM, 8U),

    /** IFDET2# The reported capability is enabled but the IFDET2 pin is not configured.
     * Check the \ref mtb_stc_ubm_dfc_signals_t.ifdet2 pin settings,
     * or disable the \ref mtb_stc_ubm_capabilities_t.ifdet2_reported setting. */
    MTB_UBM_STATUS_CAP_IFDET2_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                   CY_RSLT_MODULE_MIDDLEWARE_UBM, 9U),

    /** 2Wire Reset timer initialization error */
    MTB_UBM_STATUS_2WIRE_RESET_TIMER_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                          CY_RSLT_MODULE_MIDDLEWARE_UBM, 10U),

    /** IO timer initialization error */
    MTB_UBM_STATUS_IO_TIMER_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                 CY_RSLT_MODULE_MIDDLEWARE_UBM, 11U),

    /** DFC Dual Port Enable IO pin initialization error
     * Check the \ref mtb_stc_ubm_dfc_signals_t.dualporten pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_DUALPORTEN_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                      CY_RSLT_MODULE_MIDDLEWARE_UBM, 12U),

    /** DFC PCIe Reset A IO pin initialization error
     * Check the \ref mtb_stc_ubm_dfc_signals_t.persta pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_DFC_PERSTA_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                      CY_RSLT_MODULE_MIDDLEWARE_UBM, 13U),

    /** DFC PCIe Reset B IO pin initialization error
     * Check the \ref mtb_stc_ubm_dfc_signals_t.perstb pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_DFC_PERSTB_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                      CY_RSLT_MODULE_MIDDLEWARE_UBM, 14U),

    /** DFC PCIe Clock Enable IO pin initialization error
     * Check the \ref mtb_stc_ubm_dfc_signals_t.refclken pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_REFCLKEN_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                    CY_RSLT_MODULE_MIDDLEWARE_UBM, 15U),

    /** DFC Power Disable IO pin initialization error
     * Check the \ref mtb_stc_ubm_dfc_signals_t.pwrdis pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_PWRDIS_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                  CY_RSLT_MODULE_MIDDLEWARE_UBM, 16U),

    /** DFC PRSNT IO pin initialization error
     * Check the \ref mtb_stc_ubm_dfc_signals_t.prsnt pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_PRSNT_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                 CY_RSLT_MODULE_MIDDLEWARE_UBM, 17U),

    /** DFC IFDET IO pin initialization error
     * Check the \ref mtb_stc_ubm_dfc_signals_t.ifdet pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_IFDET_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                 CY_RSLT_MODULE_MIDDLEWARE_UBM, 18U),

    /** DFC IFDET2 IO pin initialization error
     * Check the \ref mtb_stc_ubm_dfc_signals_t.ifdet2 pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_IFDET2_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                  CY_RSLT_MODULE_MIDDLEWARE_UBM, 19U),

    /** HFC PCIe Reset IO pin initialization error
     * Check the \ref mtb_stc_ubm_hfc_signals_t.perst pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_HFC_PERST_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                     CY_RSLT_MODULE_MIDDLEWARE_UBM, 20U),

    /** HFC 2Wire Reset IO pin initialization error
     * Check the \ref mtb_stc_ubm_hfc_signals_t.i2c_reset pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_2WIRE_RESET_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                       CY_RSLT_MODULE_MIDDLEWARE_UBM, 21U),

    /** HFC Change Detect IO pin initialization error
     * Check the \ref mtb_stc_ubm_hfc_signals_t.change_detect pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_CHANGE_DETECT_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                         CY_RSLT_MODULE_MIDDLEWARE_UBM, 22U),

    /** HFC BP TYPE pin initialization error
     * Check the \ref mtb_stc_ubm_hfc_signals_t.bp_type pin settings
     * and ensure that the selected pin is not used elsewhere in the application. */
    MTB_UBM_STATUS_IO_BP_TYPE_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                   CY_RSLT_MODULE_MIDDLEWARE_UBM, 23U),

    /** EmEEPROM error during FRU initialization
     * Check the mtb_stc_ubm_backplane_cfg_t.fru_config parameter settings.
     *
     * For more details, refer to the
     * <a href="https://github.com/Infineon/emeeprom">Emulated EEPROM Middleware Library</a> documentation. */
    MTB_UBM_STATUS_FRU_EEPROM_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                   CY_RSLT_MODULE_MIDDLEWARE_UBM, 24U),

    /** The start address of the upgrade area is invalid.
     * Check the address of the upgrade area is a valid address of the start of the flash row,
     * i.e. the adress is 512 bytes aligned. */
    MTB_UBM_STATUS_BOOTLOADER_START_ADDR_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                              CY_RSLT_MODULE_MIDDLEWARE_UBM, 25U),

    /** Bootloader flash area initialization error */
    MTB_UBM_STATUS_BOOTLOADER_FLASH_INIT_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                              CY_RSLT_MODULE_MIDDLEWARE_UBM, 26U),

    /** The 2Wire IO configuration is invalid.
     * Check the \ref mtb_stc_ubm_hfc_signals_t.sda and \ref mtb_stc_ubm_hfc_signals_t.scl pin settings. */
    MTB_UBM_STATUS_2WIRE_IO_CONFIG_ERR = CY_RSLT_CREATE(CY_RSLT_TYPE_ERROR,
                                                        CY_RSLT_MODULE_MIDDLEWARE_UBM, 27U)
} mtb_en_ubm_status_t;


/** UBM Controller operational states */
typedef enum
{
    /** Not valid state. */
    MTB_UBM_OP_STATE_INVALID = 0x00U,

    /** The state during the UBM Controller initialization before configuration has completed. */
    MTB_UBM_OP_STATE_INITIALIZING = 0x01U,

    /** The state indicates that the data in the UBM Controller is inconsistent. */
    MTB_UBM_OP_STATE_BUSY = 0x02U,

    /** The state indicates that the UBM Controller has been configured and data provided is consistent. */
    MTB_UBM_OP_STATE_READY = 0x03U,

    /** The state used to indicate the UBM Controller is currently operating in Reduced Functionality. */
    MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY = 0x04U
} mtb_en_ubm_op_state_t;


/** Last command status */
typedef enum
{
    /** The last command request has failed. */
    MTB_UBM_LC_STS_FAILED = 0x00U,

    /** The last command received was processed correctly. */
    MTB_UBM_LC_STS_SUCCESS = 0x01U,

    /** The invalid checksum is detected. */
    MTB_UBM_LC_STS_INVALID_CHECKSUM = 0x02U,

    /** The write transaction byte count is larger than expected. */
    MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN = 0x03U,

    /** The HFC is not allowed to perform the command request. */
    MTB_UBM_LC_STS_NO_ACCESS_ALLOWED = 0x04U,

    /** The Change Count command does not specify the current Change Count value. */
    MTB_UBM_LC_STS_CHANGE_COUNT_DOES_NOT_MATCH = 0x05U,

    /** Busy processing the last command request. */
    MTB_UBM_LC_STS_BUSY = 0x06U,

    /** The last command request is not supported. */
    MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED = 0x07U,

    /** The invalid descriptor index is detected. */
    MTB_UBM_LC_STS_INVALID_DESCRIPTOR_INDEX = 0x08U
} mtb_en_ubm_lc_sts_t;


#if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
/** Programmable mode status */
typedef enum
{
    /** Not valid Programmable mode status. */
    MTB_UBM_PM_STS_INVALID = 0x00U,

    /** The last command was successful and it contains returned data following this Status code. */
    MTB_UBM_PM_STS_SUCCESS = 0x01U,

    /** UBM Controller Image does not verify properly. */
    MTB_UBM_PM_STS_IMAGE_VERIFY_FAILED = 0x02U,

    /** The UBM Controller Image is not supported by the UBM Controller device. */
    MTB_UBM_PM_STS_UNSUPPORTED_DEVICE = 0x03U,

    /** The non-volatile location requested is invalid. */
    MTB_UBM_PM_STS_NON_VOLATILE_LOCATION_INVALID = 0x04U,

    /** An unknown programming error */
    MTB_UBM_PM_STS_UNKNOWN_ERROR = 0x05U,

    /** The last command is still busy executing. The host will retry the command. */
    MTB_UBM_PM_STS_BUSY = 0x06U
} mtb_en_ubm_pm_sts_t;
#endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */


/** The number of lanes in the port */
typedef enum
{
    /** 1 lane. */
    MTB_UBM_LINK_WIDTH_X1 = 0U,

    /** 2 lanes. */
    MTB_UBM_LINK_WIDTH_X2 = 1U,

    /** 4 lanes. */
    MTB_UBM_LINK_WIDTH_X4 = 2U,

    /** 8 lanes. */
    MTB_UBM_LINK_WIDTH_X8 = 3U,

    /** 16 lanes. */
    MTB_UBM_LINK_WIDTH_X16 = 4U
} mtb_en_ubm_link_width_t;


/** SAS link rate */
typedef enum
{
    /** Not supported */
    MTB_UBM_SAS_RATE_NS = 0U,

    /** SAS-1 (3 Gb/s) */
    MTB_UBM_SAS_1_RATE  = 1U,

    /** SAS-2 (6 Gb/s) */
    MTB_UBM_SAS_2_RATE  = 2U,

    /** SAS-3 (12 Gb/s) */
    MTB_UBM_SAS_3_RATE  = 3U,

    /** SAS-4 (22.5 Gb/s) */
    MTB_UBM_SAS_4_RATE  = 4U,

    /** SAS-5 (TBD) */
    MTB_UBM_SAS_5_RATE  = 5U,

    /** SAS-6 (TBD) */
    MTB_UBM_SAS_6_RATE  = 6U,

    /** No Limit */
    MTB_UBM_SAS_NO_RATE_LIMIT = 7U
} mtb_en_ubm_sas_link_rate_t;


/** PCIe link rate */
typedef enum
{
    /** Not supported */
    MTB_UBM_PCIE_RATE_NS = 0U,

    /** PCIe-1 (2.5 GT/s) */
    MTB_UBM_PCIE_1_RATE  = 1U,

    /** PCIe-2 (5 GT/s) */
    MTB_UBM_PCIE_2_RATE  = 2U,

    /** PCIe-3 (8 GT/s) */
    MTB_UBM_PCIE_3_RATE  = 3U,

    /** PCIe-4 (16 GT/s) */
    MTB_UBM_PCIE_4_RATE  = 4U,

    /** PCIe-5 (32 GT/s) */
    MTB_UBM_PCIE_5_RATE  = 5U,

    /** PCIe-6 (TBD) */
    MTB_UBM_PCIE_6_RATE  = 6U,

    /** No Limit */
    MTB_UBM_PCIE_NO_RATE_LIMIT = 7U
} mtb_en_ubm_pcie_link_rate_t;


/** The UBM Controller type */
typedef enum
{
    /** Defined by UBM specification */
    MTB_UBM_CONTROLLER_SPEC_DEFINED    = 0U,

    /** Vendor-specific */
    MTB_UBM_CONTROLLER_VENDOR_SPECIFIC = 1U
} mtb_en_ubm_controller_type_t;


/** SATA link rate */
typedef enum
{
    /** Not supported */
    MTB_UBM_SATA_RATE_NS = 0U,

    /** 3 Gb/s */
    MTB_UBM_SATA_3GBS_RATE  = 1U,

    /** 6 Gb/s */
    MTB_UBM_SATA_6GBS_RATE  = 2U,

    /** No Limit */
    MTB_UBM_SATA_NO_RATE_LIMIT = 3U
} mtb_en_ubm_sata_link_rate_t;


/** The connector port type */
typedef enum
{
    /** Supports PCIe and SAS/SATA via the same port. */
    MTB_UBM_PORT_TYPE_CONVERGED = 0U,

    /** Supports PCIe and does not support SAS/SATA via the same port. */
    MTB_UBM_PORT_TYPE_SEGREGATED = 1U
} mtb_en_ubm_port_type_t;


/** Connector port domain */
typedef enum
{
    /** Primary port. */
    MTB_UBM_PORT_DOMAIN_PRIMARY = 0U,

    /** Secondary port. */
    MTB_UBM_PORT_DOMAIN_SECONDARY = 1U
} mtb_en_ubm_port_domain_t;


/** Change Count increment source */
typedef enum
{
    /** Change source CPRSTN# Legacy Mode Change */
    MTB_UBM_CC_SOURCE_CPRSNT_LEGACY_MODE = 0U,

    /** Change source PCIe Reset Change */
    MTB_UBM_CC_SOURCE_PCIE_RESET = 1U,

    /** Change source Drive Type Installed Change */
    MTB_UBM_CC_SOURCE_DRIVE_TYPE = 2U,

    /** Change source Operational State Change */
    MTB_UBM_CC_SOURCE_OP_STATE = 3U,

    /** Change source Controller Reset */
    MTB_UBM_CC_SOURCE_CTRL_RESET = 4U
} mtb_en_ubm_change_count_source_t;

/** @} group_ubm_enums */


/**
 * \addtogroup group_ubm_fru_data_structures
 * @{
 */

/** Features configuration structure */
typedef struct
{
    /** Indicates if the UBM Controller generates a valid Read Checksum for the read phase of a 2Wire transaction. */
    bool read_checksum_creation;

    /** Indicates if the UBM Controller performs Checksum verification on the write phase of a 2Wire transaction. */
    bool write_checksum_checking;

    /** Indicates the behavior of the CPRSNT#/CHANGE_DETECT# signal. */
    bool cprsnt_legacy_mode;

    /** Indicates if a change to PCIe Reset field causes the Change Count field to increment. */
    bool pcie_reset_change_count_mask;

    /** Indicates if a change to Drive Type Installed field causes the Change Count field to increment. */
    bool drive_type_installed_change_count_mask;

    /** Indicatesif a change to Operational State field causes the Change Count field to increment. */
    bool operational_state_change_count_mask;

    /** Indicates the DFC PERST# behavior when a Drive has been installed. */
    uint8_t perst_management_override;

    /** Controls the DFC SMBRST# signal for all DFC's associated under the HFC. */
    bool smbus_reset_control;
} mtb_stc_ubm_features_t;


/** UBM FRU Overview Area configuration structure */
typedef struct
{
    /** Two wire device arrangement */
    uint8_t two_wire_device_arrangement;

    /** Two wire mux adress */
    uint8_t two_wire_mux_address;

    /** Two wire max byte count */
    uint8_t two_wire_max_byte_count;

    /** Device max time limit */
    uint8_t ubm_max_time_limit;

    /** Device features */
    mtb_stc_ubm_features_t ubm_controller_features;

    /** Maximum power per DFC */
    uint8_t maximum_power_per_dfc;

    /** Mux channel type */
    uint8_t mux_channel_count;

    /** Enable bit location */
    uint8_t enable_bit_location;

    /** Mux type */
    uint8_t mux_type;
} mtb_stc_ubm_fru_oa_config_t;

/** @} group_ubm_fru_data_structures */


/**
 * \addtogroup group_ubm_io_signals
 * @{
 */

/** DFC control signals configuration structure */
typedef struct
{
    /** Presence detect pin. */
    cyhal_gpio_t ifdet;

    /** Presence detect pin. */
    cyhal_gpio_t ifdet2;

    /** Presence detect pin. */
    cyhal_gpio_t prsnt;

    /** PCIe reset A pin. */
    cyhal_gpio_t persta;

    /** PCIe reset B pin. */
    cyhal_gpio_t perstb;

    /** Power disable pin. */
    cyhal_gpio_t pwrdis;

    /** Reference clock enable pin. */
    cyhal_gpio_t refclken;

    /** Dual-port enable pin. */
    cyhal_gpio_t dualporten;
} mtb_stc_ubm_dfc_signals_t;


/** HFC control signals configuration structure */
typedef struct
{
    /** Serial data pin. */
    cyhal_gpio_t sda;

    /** Serial clock pin. */
    cyhal_gpio_t scl;

    /** I2C interface reset pin. */
    cyhal_gpio_t i2c_reset;

    /** Change detected pin. */
    cyhal_gpio_t change_detect;

    /** Backplane type pin. */
    cyhal_gpio_t bp_type;

    /** PCIe reset pin. */
    cyhal_gpio_t perst;
} mtb_stc_ubm_hfc_signals_t;


/** @} group_ubm_io_signals */

/** Backplane control signals configuration structure */
typedef struct
{
    /** DFC control signals */
    mtb_stc_ubm_dfc_signals_t dfc_io[MTB_UBM_DFC_MAX_NUM];

    /** HFC control signals */
    mtb_stc_ubm_hfc_signals_t hfc_io[MTB_UBM_HFC_MAX_NUM];
} mtb_stc_ubm_backplane_control_signals_t;


/**
 * \addtogroup group_ubm_fru_data_structures
 * @{
 */

/** Drive Types Supported configuration structure */
typedef struct
{
    /** SFF-TA-1001 support */
    bool sff_ta_1001;

    /** Gen-Z support */
    bool gen_z;

    /** SAS/SATA support */
    bool sas_sata;

    /** Quad PCIe support */
    bool quad_pcie;

    /** DFC empty support */
    bool dfc_empty;
} mtb_stc_ubm_drive_types_t;


/** HFC routing configuration structure */
typedef struct
{
    /** UBM controller type */
    mtb_en_ubm_controller_type_t ubm_ctrl_type;

    /** I2C address for UBM controller */
    uint8_t ubm_ctrl_slave_addr;

    /** DFC status and control descriptor index */
    uint8_t drive_connector_idx;

    /** Drive types supported in DFC */
    mtb_stc_ubm_drive_types_t drive_types_supported;

    /** Number of lanes in the port */
    mtb_en_ubm_link_width_t drive_link_width;

    /** Connector port type */
    mtb_en_ubm_port_type_t port_type;

    /** Connector port domain */
    mtb_en_ubm_port_domain_t domain;

    /** Maximum SATA rate */
    mtb_en_ubm_sata_link_rate_t max_sata_line_rate;

    /** Maximum PCIe rate */
    mtb_en_ubm_pcie_link_rate_t max_pcie_line_rate;

    /** Maximum SAS rate */
    mtb_en_ubm_sas_link_rate_t max_sas_line_rate;

    /** Host lane index */
    uint8_t hfc_starting_phy_lane;

    /** Host connector identity */
    uint8_t hfc_identifier;

    /** Slot offset */
    uint8_t slot_offset;
} mtb_stc_ubm_routing_t;


/** @} group_ubm_fru_data_structures */

/** DFC_PERST signal structure */
typedef struct
{
    /** The counter to mesure a delay for ClkRef stabilization. */
    uint32_t delay_counter;

    /** Delay value */
    uint32_t delay_val;

    /** Detected HFC_PERST high */
    bool hfc_signal_state;

    /** Signa is de-asserted */
    bool signal_released;

    /** DFC SnCD last change */
    bool scd_last_change;
} mtb_stc_ubm_dfc_perst_status;


/** Silicon Identity and Version configuration structure */
typedef struct
{
    /** PCIe Vendor ID */
    uint16_t pcie_vendor_id;

    /** UBM Controller Device code */
    uint32_t device_code;

    /** UBM Controller Image Version Minor */
    uint8_t fw_version_major;

    /** UBM Controller Image Version Minor */
    uint8_t fw_version_minor;

    /** UBM Controller vendor-specific data */
    uint16_t vendor_specific;
} mtb_stc_ubm_siv_t;


/** Backplane Info configuration structure */
typedef struct
{
    /** Backplane type */
    uint8_t backplane_type;

    /** Backplane number */
    uint8_t backplane_number;
} mtb_stc_ubm_bp_info_t;


/** Capabilities configuration structure */
typedef struct
{
    /** Indicates the availability of high speed differential clock routing (i.e., RefClk) from the HFC to DFC. */
    bool clock_routing;

    /** Indicates if the DFCs support Power Disable (i.e., PwrDIS signal). */
    bool slot_power_control;

    /** Indicates if PCIe Reset Control is supported. */
    bool pcie_reset_control;

    /** Indicates if Dual Port DFC connectors are routed. */
    bool dual_port;

    /** Indicates the 2WIRE_RESET# signal support. */
    uint8_t i2c_reset_operation;

    /** Indicates if the CHANGE_DETECT# signal interrupt operation is supported. */
    bool change_detect_interrupt;

    /** Indicates if a change count is maintained per individual DFC Status and Control Command Descriptor. */
    bool dfc_change_count_supported;

    /** Indicates if the PRSNT# signal is reported. */
    bool prsnt_reported;

    /** Indicates if the IFDET# signal is reported. */
    bool ifdet_reported;

    /** Indicates if the IFDET2# signal is reported. */
    bool ifdet2_reported;

    /** Indicates if the UBM Controller supports the DFC PERST# Management Override field in the Features Command. */
    bool perst_override_supported;

    /** Indicates if the UBM Controller supports control over the DFC SMBRST# signals (e.g. See SFF-TA-1009). */
    bool smb_reset_supported;
} mtb_stc_ubm_capabilities_t;


/** Change Count command structure */
typedef struct
{
    /** Change count value */
    uint8_t change_count;

    /** Change source CPRSTN# Legacy Mode Change */
    bool source_cprsnt_legacy_mode_change;

    /** Change source PCIe Reset Change */
    bool source_pcie_reset_change;

    /** Change source Drive Type Installed Change */
    bool source_drive_type_installed_change;

    /** Change source Operational State Change */
    bool source_operational_state_change;

    /** Change source Controller Reset */
    bool source_controller_reset_change;
} mtb_stc_ubm_change_count_t;


/**
 * \addtogroup group_ubm_ifc_data_structures
 * @{
 */

/** SES-4 Array Device Slot control element */
typedef struct
{
    /** SES-4 Common Control Reset Swap bit */
    bool reset_swap;

    /** SES-4 Common Control Disable bit */
    bool disable;

    /** SES-4 Common Control Predicted Failure bit */
    bool predicted_failure;

    /** SES-4 Common Control Select bit */
    bool select;

    /** SES-4 Request Rebuild/Remap Aborted bit */
    bool request_rebuild_remap_aborted;

    /** SES-4 Request Rebuild/Remap bit */
    bool request_rebuild_remap;

    /** SES-4 Request In Failed Array bit */
    bool request_in_failed_array;

    /** SES-4 Request In Critical Array bit */
    bool request_in_critical_array;

    /** SES-4 Request Consistency Check bit */
    bool request_consistency_check;

    /** SES-4 Request Hot Spare bit */
    bool request_hot_spare;

    /** SES-4 Request Reserved Device bit */
    bool request_reserved_device;

    /** SES-4 Request OK bit */
    bool request_ok;

    /** SES-4 Request Identify bit */
    bool request_identify;

    /** SES-4 Request Removal bit */
    bool request_remove;

    /** SES-4 Request Insert bit */
    bool request_insert;

    /** SES-4 Request Device Missing Indication bit */
    bool request_missing;

    /** SES-4 Do Not Remove bit */
    bool do_not_remove;

    /** SES-4 Request Device Activity Indication bit */
    bool request_active;

    /** SES-4 Enable Bypass Port B bit */
    bool enable_bypass_b;

    /** SES-4 Enable Bypass Port A bit */
    bool enable_bypass_a;

    /** SES-4 Device OFF bit */
    bool device_off;

    /** SES-4 Request Fault Indication bit */
    bool request_fault;
} mtb_stc_ubm_ses_control_t;


/** SES-4 Array Device Slot status element */
typedef struct
{
    /** SES-4 Common Status Element Status Code bits */
    uint8_t element_status_code;

    /** SES-4 Common Status Swap bit */
    bool swap;

    /** SES-4 Common Status Disabled bit */
    bool disabled;

    /** SES-4 Common Status Predicted Failure bit */
    bool predicted_failure;

    /** SES-4 Rebuild/Remap Abort bit */
    bool rebuild_remap_abort;

    /** SES-4 Rebuild/Remap bit */
    bool rebuild_remap;

    /** SES-4 In Failed Array bit */
    bool in_failed_array;

    /** SES-4 In Critical Array bit */
    bool in_critical_array;

    /** SES-4 Consistency Check In Progress bit */
    bool consistency_check;

    /** SES-4 Hot Spare bit */
    bool hot_spare;

    /** SES-4 Reserved Device bit */
    bool reserved_device;

    /** SES-4 OK bit */
    bool ok;

    /** SES-4 Report bit */
    bool report;

    /** SES-4 Identify bit */
    bool identify;

    /** SES-4 Remove bit */
    bool rmv;

    /** SES-4 Insert bit */
    bool ready_to_insert;

    /** SES-4 Enclosure Bypassed Port B bit */
    bool enclosure_bypassed_b;

    /** SES-4 Enclosure Bypassed Port A bit */
    bool enclosure_bypassed_a;

    /** SES-4 Do Not Remove bit */
    bool do_not_remove;

    /** SES-4 Application Client Bypassed Port A bit */
    bool app_client_bypassed_a;

    /** SES-4 Device Bypassed Port B bit */
    bool device_bypassed_b;

    /** SES-4 Device Bypassed Port A bit */
    bool device_bypassed_a;

    /** SES-4 Bypassed Port B bit */
    bool bypassed_b;

    /** SES-4 Bypassed Port A bit */
    bool bypassed_a;

    /** SES-4 Device OFF bit */
    bool device_off;

    /** SES-4 Fault Requested bit */
    bool fault_requested;

    /** SES-4 Fault Sensed bit */
    bool fault_sensed;

    /** SES-4 Application Client Bypassed Port B bit */
    bool app_client_bypassed_b;
} mtb_stc_ubm_ses_status_t;


#if (MTB_UBM_SES_CB_ACTIVE || DOXYGEN)

/** The app callback context type */
typedef struct
{
    /** DFC index */
    uint8_t dfc_index;

    /** The pointer to the last received SES Array Device Slot Control Element. */
    const mtb_stc_ubm_ses_control_t* ses_control;

    /** The pointer to the current SES Array Device Slot Status Element. */
    mtb_stc_ubm_ses_status_t* ses_status;
} mtb_stc_ubm_ses_app_cb_context_t;


/** @} group_ubm_ifc_data_structures */

/**
 * \addtogroup group_ubm_ifc_callback_type
 * @{
 */
/** Application callback type */
typedef bool (* mtb_ubm_ses_app_cb_t)(mtb_stc_ubm_ses_app_cb_context_t* context);
/** @} group_ubm_ifc_callback_type */
#endif /* (MTB_UBM_SES_CB_ACTIVE || DOXYGEN) */


/** UBM Status and Control Descriptor structure */
typedef struct
{
    /** Indicates the type of device in DFC. */
    uint8_t* drive_type;

    /** Indicates if the DFC port link width will be bifurcated. */
    bool bifurcate_port;

    /** Specifies the port-specific DFC PERST# signal behavior. */
    uint8_t pcie_reset;

    /** Device OFF bit */
    bool device_off;

    /** Last received SES Array Device Slot Control Element */
    uint8_t ses_control[MTB_UBM_SES_ARRAY_DEVICE_SLOT_ELEMENT_SIZE];

    /** Current SES Array Device Slot Status Element */
    uint8_t ses_status[MTB_UBM_SES_ARRAY_DEVICE_SLOT_ELEMENT_SIZE];

    /** DFC Change Count indication */
    uint8_t dfc_change_count;

    /** Vendor-specific data byte 0 */
    uint8_t vendor_specific_byte_0;

    /** Vendor-specific data byte 1 */
    uint8_t vendor_specific_byte_1;
} mtb_stc_ubm_status_and_control_descriptor_t;


/** UBM backplane configuration structure */
typedef struct
{
    /** The number of the HFCs in the backplane. */
    uint8_t num_of_hfc;

    /** The number of the DFCs in the backplane. */
    uint8_t num_of_dfc;

    /** The number of the routes in the backplane. */
    uint8_t num_of_routes;

    /** UBM starting slot */
    uint8_t starting_slot;

    /** Overview area configuration */
    mtb_stc_ubm_fru_oa_config_t* overview_area;

    #if (MTB_UBM_SES_CB_ACTIVE)
    /** APP handler for SES Array Device Slot Control Element */
    mtb_ubm_ses_app_cb_t ses_event_handler;
    #endif /* MTB_UBM_SES_CB_ACTIVE */

    /** Backplane routing configuration */
    mtb_stc_ubm_routing_t route_information[MTB_UBM_ROUTES_MAX_NUM];

    /** Silicon Identity and Version parameters */
    mtb_stc_ubm_siv_t silicon_identity;

    /** Backplane Info parameters */
    mtb_stc_ubm_bp_info_t backplane_info;

    /** UBM comtroller capabilities */
    mtb_stc_ubm_capabilities_t capabilities;

    /** Storage configuration for FRU */
    cy_stc_eeprom_config_t* fru_config;

    /** Indicates whether the DFC port link width will be bifurcated. */
    bool bifurcate_port;
} mtb_stc_ubm_backplane_cfg_t;


/* UBM context structure forward declaration */
typedef struct mtb_stc_ubm_context_t mtb_stc_ubm_context_t;


/** I2C event callback parameter structure */
typedef struct
{
    /** The pointer to the UBM context structure. */
    mtb_stc_ubm_context_t* ubm_context;

    /** The index of the controller context structure. */
    uint8_t hfc_index;
} mtb_stc_ubm_i2c_callback_arg_t;


/** UBM I2C interface structure */
typedef struct
{
    /** HAL I2C structure  */
    cyhal_i2c_t scb_i2c_obj;

    /** The flag, which indicates a read request received. */
    bool read_request;

    /** The buffer to which the master writes data to. */
    uint8_t write_buffer[MTB_UBM_I2C_WRITE_BUFFER_SIZE];

    /** The buffer from which the master reads data from. */
    uint8_t read_buffer[MTB_UBM_I2C_READ_BUFFER_SIZE];

    /** The data length in the write buffer. */
    uint32_t write_data_length;

    /** The data length in the read buffer. */
    uint32_t read_data_length;

    /** Event callback parameter structure */
    mtb_stc_ubm_i2c_callback_arg_t cb_arg;
} mtb_stc_ubm_i2c_t;


#if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
/** UBM flash geometry structure */
typedef struct
{
    /** The value of the offset for the write row_buffer. */
    uint32_t      offset_row_buffer;

    /** The data to be programmed at the specified Sector Index. */
    uint8_t       row_buffer[CY_FLASH_SIZEOF_ROW];

    /** HAL Flash structure */
    cyhal_flash_t flash_obj;

    /** Indicates the sequence number of the Program Subcommand. */
    uint8_t       app_sequence_number;

    /** The Image Number field, which specifies the Image Number associated with
     *  the vendor-specific data set in the non-volatile storage. */
    uint8_t       image_number;

    /** The Sector Index field indicates the Sector Index in the Sector of the Non-Volatile Storage. */
    uint8_t       sector_index;

    /** The Sector Index Checksum field indicates the two's complement of
     * the summation of bytes located at the Sector Index. */
    uint8_t       checksum_sector_index;

    /** The status of the verification upgrade image. */
    bool          status_download_image;
} mtb_stc_ubm_flash_geometry_t;
#endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */


/** HFC GPIO event callback parameter structure */
typedef struct
{
    /** The pointer to the UBM context structure. */
    mtb_stc_ubm_context_t* ubm_context;

    /** The index of the HFC. */
    uint8_t hfc_index;
} mtb_stc_ubm_hfc_gpio_callback_arg_t;


/** HFC GPIO callbacks configuration structure */
typedef struct
{
    /** Callback configuration data for the PERST signal */
    cyhal_gpio_callback_data_t perst;

    /** Callback configuration data for the 2 Wire Reset event */
    cyhal_gpio_callback_data_t two_wire_reset;

    /** Callback argument */
    mtb_stc_ubm_hfc_gpio_callback_arg_t arg;
} mtb_stc_ubm_hfc_gpio_callbacks_t;


/** Input signal structure */
typedef struct
{
    /** Assigned PIN. */
    cyhal_gpio_t pin;

    /** Asserted signal timing. */
    uint32_t asserted_count;
} mtb_stc_ubm_input_t;


/** UBM controller structure */
typedef struct
{
    /** The index of the associated HFC. */
    uint8_t hfc_index;

    /** Th I2C address of the UBM controller. */
    uint8_t slave_address;

    /** The status of the last received UBM command. */
    mtb_en_ubm_lc_sts_t last_command_status;

    /** DFC Status and Control Descriptor Index */
    uint8_t status_n_control_descriptor_index;

    /** Legacy mode of CPRSNT/CHANGE_DETECT signal */
    bool legacy_mode;

    /** DFC Status and Control Descriptor */
    mtb_stc_ubm_status_and_control_descriptor_t scd[MTB_UBM_DFC_MAX_NUM];

    /** HFC port type */
    mtb_en_ubm_port_type_t port_type;

    /** The domain of the asociated DFC. */
    mtb_en_ubm_port_domain_t domain;

    /** UBM Controller Features */
    mtb_stc_ubm_features_t features;

    /** UBM Controller Change Count */
    mtb_stc_ubm_change_count_t change_count;

    /** The number of the DFC associated with the controller. */
    uint8_t dfc_count;

    /** Indexes of the DFC associated with the controller. */
    uint8_t dfc_list[MTB_UBM_DFC_MAX_NUM];
} mtb_stc_ubm_controller_t;


/** UBM HFC context structure */
typedef struct
{
    /** I2C interface structure */
    mtb_stc_ubm_i2c_t i2c;

    /** UBM HFC index */
    uint8_t index;

    /** 2 Wire/silicon reset signal */
    mtb_stc_ubm_input_t reset;

    /** HFCs control signals */
    mtb_stc_ubm_hfc_signals_t hfc_io;

    /** GPIO callbacks configuration structure */
    mtb_stc_ubm_hfc_gpio_callbacks_t cb_args;

    /** The number of the controllers associated with the HFC. */
    uint8_t ctrl_count;

    /** Indexes of the controllers associated with the HFC. */
    uint8_t ctrl_list[MTB_UBM_CTRLS_MAX_NUM];

    /** Controller I2C slave address */
    uint8_t ctrl_slave_address[MTB_UBM_CTRLS_MAX_NUM];

    /** The index of the addressed controller. */
    uint8_t selected_ctrl_index;

    /** The I2C slave address of the addressed controller. */
    uint8_t selected_slave_address;
} mtb_stc_ubm_hfc_t;


/** UBM DFC context structure */
typedef struct
{
    /** UBM DFC index */
    uint8_t index;

    /** DFCs control signals */
    mtb_stc_ubm_dfc_signals_t dfc_io;

    /** The array of structures for the status of the perst signal */
    mtb_stc_ubm_dfc_perst_status dfc_perst_a_b[MTB_UBM_DOMAINS_MAX_NUM];

    /** Drive type installed value */
    uint8_t drive_type_installed;

    /** The number of samples left for the drive type event. */
    uint8_t drive_type_counter;

    /** Indexes of the controllers associated with the DFC. */
    uint8_t ctrl_list[MTB_UBM_CTRLS_MAX_NUM];

    /** The number of the controllers associated with the DFC. */
    uint8_t ctrl_count;
} mtb_stc_ubm_dfc_t;


/** UBM context structure
 * All fields for the context structure are internal.
 * The firmware:
 *  - never reads or writes these values
 *  - allocates the structure
 *  - provides the address of the structure to the driver in function calls
 *  - ensures that the defined instance of this structure remains in scope
 *    while the drive is in use.
 */
struct mtb_stc_ubm_context_t
{
    /** \cond INTERNAL */

    /** UBM HFC context structure. */
    mtb_stc_ubm_hfc_t hfc[MTB_UBM_HFC_MAX_NUM];

    /** UBM controller context. */
    mtb_stc_ubm_controller_t ctrl[MTB_UBM_CTRLS_MAX_NUM];

    /** UBM controller context. */
    mtb_stc_ubm_dfc_t dfc[MTB_UBM_DFC_MAX_NUM];

    /** The number of the HFCs in the backplane. */
    uint8_t num_of_hfc;

    /** The number of the DFCs in the backplane. */
    uint8_t num_of_dfc;

    /** The number of the routes in the backplane. */
    uint8_t num_of_routes;

    /** The number of the controllers in the backplane. */
    uint8_t num_of_ctrls;

    #if (MTB_UBM_SES_CB_ACTIVE)
    /** The callback function of the control element applicationthe of SES Array Device Slot  */
    mtb_ubm_ses_app_cb_t ses_control_cb;
    #endif /* MTB_UBM_SES_CB_ACTIVE */

    #if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
    /** Programmable mode status */
    mtb_en_ubm_pm_sts_t update_subcmd_status;

    /** UBM flash geometry structure */
    mtb_stc_ubm_flash_geometry_t flash_layout;
    #endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */

    /** FRU cotext */
    cy_stc_eeprom_context_t fru_context;

    /** The operational state of the UBM controller. */
    mtb_en_ubm_op_state_t state;

    /** UBM Controller silicon identity and version */
    mtb_stc_ubm_siv_t silicon_identity;

    /** UBM Controller Backplane Info */
    mtb_stc_ubm_bp_info_t backplane_info;

    /** UBM Controller capabilities */
    mtb_stc_ubm_capabilities_t capabilities;

    /** UBM Starting slot */
    uint8_t starting_slot;

    /** The timer structure for the HFC 2Wire reset duration calculation. */
    cyhal_timer_t i2c_reset_timer;

    /** The timer structure for the periodic functionality. */
    cyhal_timer_t periodic_timer;

    /** \endcond */
};


#endif /* MTB_UBM_TYPES_H */
