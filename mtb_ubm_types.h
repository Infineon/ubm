/***************************************************************************//**
 * \file mtb_ubm_types.h
 * \version 0.5
 *
 * \brief
 * Provides the UBM middleware API types declarations.
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


/** Size of the I2C slave buffer to which master writes data.*/
#define MTB_UBM_I2C_WRITE_BUFFER_SIZE   (256U)

/** Size of the I2C slave buffer from which master reads data.*/
#define MTB_UBM_I2C_READ_BUFFER_SIZE    (256U)


/** UBM Controller commands */
/** Returns the operating state of the UBM Controller. */
#define MTB_UBM_CMD_OPERATIONAL_STATE                      (0x00U)

/** Returns the last command execution status of the UBM Controller. */
#define MTB_UBM_CMD_LAST_COMMAND_STATUS                     (0x01U)

/** Returns UBM Controller identification data. */
#define MTB_UBM_CMD_SILICON_IDENTITY_AND_VERSION            (0x02U)

/** Returns the Programmable Update Mode capabilities of the UBM Controller. */
#define MTB_UBM_CMD_PROGRAMMABLE_UPDATE_MODE_CAPABILITIES   (0x03U)

/** Indicates a sequence to unlock and transfer to Programmable Update Mode. */
#define MTB_UBM_CMD_ENTER_PROGRAMMABLE_UPDATE_MODE          (0x20U)

/** Indicates method to exchange multiple bytes of command, status and data. */
#define MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER         (0x21U)

/** Indicates to transfer out of Programmable Update Mode. */
#define MTB_UBM_CMD_EXIT_PROGRAMMABLE_UPDATE_MODE           (0x22U)

/** Returns the Host Facing Connector information. */
#define MTB_UBM_CMD_HOST_FACING_CONNECTOR_INFO              (0x30U)

/** Returns the backplane number and type that is unique in the chassis. */
#define MTB_UBM_CMD_BACKPLANE_INFO                          (0x31U)

/** Returns the Starting Slot which is applied to the Slot Offset
 * found in the UBM Port Route Information of the UBM FRU.
 */
#define MTB_UBM_CMD_STARTING_SLOT                           (0x32U)

/** Returns the backplane capabilities. */
#define MTB_UBM_CMD_CAPABILITIES                            (0x33U)

/** Indicates the UBM Controller features. */
#define MTB_UBM_CMD_FEATURES                                (0x34U)

/** Counter used to manage UBM Controller interrupts. */
#define MTB_UBM_CMD_CHANGE_COUNT                            (0x35U)

/** Controls the DFC Status and Control Descriptor to access. */
#define MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR_INDEX (0x36U)

/** Indicates the DFC Status and Control Descriptor data
 * for the current DFC Status and Control Descriptor Index.
 */
#define MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR       (0x40U)

/** Type for UBM Controller commands */
typedef uint8_t mtb_ubm_cmd_t;


/** UBM Controller programmable mode subcommands */
/** Not valid programmable mode subcommand. */
#define MTB_UBM_PM_CMD_INVALID                              (0x00U)

/** Returns the nonvolatile structure and size of the programmable segments. */
#define MTB_UBM_PM_CMD_GET_NON_VOLATILE_STORAGE_GEOMETRY    (0x01U)

/** Erases a segment of the nonvolatile location to prepare for programming. */
#define MTB_UBM_PM_CMD_ERASE                                (0x02U)

/** Returns the status of an erase request. */
#define MTB_UBM_PM_CMD_ERASE_STATUS                         (0x03U)

/** Writes a segment of data into the nonvolatile location. */
#define MTB_UBM_PM_CMD_PROGRAM                              (0x04U)

/** Returns the status of a program request. */
#define MTB_UBM_PM_CMD_PROGRAM_STATUS                       (0x05U)

/** Sets the Sector and Sector Index for a Verify Status request. */
#define MTB_UBM_PM_CMD_VERIFY                               (0x06U)

/** Returns the checksum for the nonvolatile segment. */
#define MTB_UBM_PM_CMD_VERIFY_STATUS                        (0x07U)

/** Sets the Image Number for an Image Number Status request. */
#define MTB_UBM_PM_CMD_VERIFY_IMAGE                         (0x08U)

/** Returns information indicating UBM Controller Image is valid. */
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

/** 2Wire Slave Reset and 2Wire Mux is supported */
#define MTB_UBM_CAP_2WIRE_RESET_OP_2WIRE_AND_MUX            (0x10U)
/** UBM FRU and UBM Controller is supported */
#define MTB_UBM_CAP_2WIRE_RESET_OP_FRU_CONTROLLER           (0x20U)
/** 2Wire Slave Reset and UBM FRU and UBM Controller and
 * 2Wire Mux are supported */
#define MTB_UBM_CAP_2WIRE_RESET_OP_2W_FRU_CONTROLLER_MUX    (0x30U)

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

/** Type for UBM Controller programmable mode subcommands */
typedef uint8_t mtb_ubm_pm_cmd_t;


/** UBM Controller operational states */
typedef enum
{
    /** Not valid state. */
    MTB_UBM_OP_STATE_INVALID = 0x00U,

    /** State during the UBM Controller initialization before configuration has completed. */
    MTB_UBM_OP_STATE_INITIALIZING = 0x01U,

    /** State indicates the data in UBM Controller is inconsistent. */
    MTB_UBM_OP_STATE_BUSY = 0x02U,

    /** State indicates UBM Controller has been configured and data provided is consistent. */
    MTB_UBM_OP_STATE_READY = 0x03U,

    /** State used to indicate UBM Controller is currently operating in Reduced Functionality. */
    MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY = 0x04U
} mtb_en_ubm_op_state_t;


/** Last command status */
typedef enum
{
    /** Last command request has failed. */
    MTB_UBM_LC_STS_FAILED = 0x00U,

    /** Last command received was processed correctly. */
    MTB_UBM_LC_STS_SUCCESS = 0x01U,

    /** Invalid Checksum detected. */
    MTB_UBM_LC_STS_INVALID_CHECKSUM = 0x02U,

    /** Write transaction byte count larger than expected. */
    MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN = 0x03U,

    /** Host facing connector is not allowed to perform command request. */
    MTB_UBM_LC_STS_NO_ACCESS_ALLOWED = 0x04U,

    /** The Change Count command did not specify the current Change Count value. */
    MTB_UBM_LC_STS_CHANGE_COUNT_DOES_NOT_MATCH = 0x05U,

    /** Busy processing the last command request. */
    MTB_UBM_LC_STS_BUSY = 0x06U,

    /** Last command request is not supported. */
    MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED = 0x07U,

    /** Invalid descriptor index was detected. */
    MTB_UBM_LC_STS_INVALID_DESCRIPTOR_INDEX = 0x08U
} mtb_en_ubm_lc_sts_t;


#if(MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
/** Programmable mode status */
typedef enum
{
    /** Not valid programmable mode status. */
    MTB_UBM_PM_STS_INVALID = 0x00U,

    /** Last Command was successful and contains returned data following this Status code. */
    MTB_UBM_PM_STS_SUCCESS = 0x01U,

    /** UBM Controller Image did not verify properly. */
    MTB_UBM_PM_STS_IMAGE_VERIFY_FAILED = 0x02U,

    /** UBM Controller Image is not supported by the UBM Controller device. */
    MTB_UBM_PM_STS_UNSUPPORTED_DEVICE = 0x03U,

    /** Non-Volatile Location requested is invalid. */
    MTB_UBM_PM_STS_NON_VOLATILE_LOCATION_INVALID = 0x04U,

    /** Unknown programming error has occurred. */
    MTB_UBM_PM_STS_UNKNOWN_ERROR = 0x05U,

    /** Last Command is still busy executing. Host should retry command. */
    MTB_UBM_PM_STS_BUSY = 0x06U
} mtb_en_ubm_pm_sts_t;
#endif /*MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED*/


/**
 * \addtogroup group_ubm_enums
 * @{
 */

/** The number of lanes in the port */
typedef enum
{
    MTB_UBM_LINK_WIDTH_X1 = 0U,     /**< 1 lane. */
    MTB_UBM_LINK_WIDTH_X2 = 1U,     /**< 2 lanes. */
    MTB_UBM_LINK_WIDTH_X4 = 2U,     /**< 4 lanes. */
    MTB_UBM_LINK_WIDTH_X8 = 3U,     /**< 8 lanes. */
    MTB_UBM_LINK_WIDTH_X16 = 4U     /**< 16 lanes. */
} mtb_en_ubm_link_width_t;

/** SAS link rate */
typedef enum
{
    MTB_UBM_SAS_RATE_NS = 0U,       /**< Not supported */
    MTB_UBM_SAS_1_RATE  = 1U,       /**< SAS-1 (3 Gb/s) */
    MTB_UBM_SAS_2_RATE  = 2U,       /**< SAS-2 (6 Gb/s) */
    MTB_UBM_SAS_3_RATE  = 3U,       /**< SAS-3 (12 Gb/s) */
    MTB_UBM_SAS_4_RATE  = 4U,       /**< SAS-4 (22.5 Gb/s) */
    MTB_UBM_SAS_5_RATE  = 5U,       /**< SAS-5 (TBD) */
    MTB_UBM_SAS_6_RATE  = 6U,       /**< SAS-6 (TBD) */
    MTB_UBM_SAS_NO_RATE_LIMIT = 7U  /**< No Limit */
} mtb_en_ubm_sas_link_rate_t;

/** PCIe link rate */
typedef enum
{
    MTB_UBM_PCIE_RATE_NS = 0U,       /**< Not supported */
    MTB_UBM_PCIE_1_RATE  = 1U,       /**< PCIe-1 (2.5 GT/s) */
    MTB_UBM_PCIE_2_RATE  = 2U,       /**< PCIe-2 (5 GT/s) */
    MTB_UBM_PCIE_3_RATE  = 3U,       /**< PCIe-3 (8 GT/s) */
    MTB_UBM_PCIE_4_RATE  = 4U,       /**< PCIe-4 (16 GT/s) */
    MTB_UBM_PCIE_5_RATE  = 5U,       /**< PCIe-5 (32 GT/s) */
    MTB_UBM_PCIE_6_RATE  = 6U,       /**< PCIe-6 (TBD) */
    MTB_UBM_PCIE_NO_RATE_LIMIT = 7U  /**< No Limit */
} mtb_en_ubm_pcie_link_rate_t;

/** The UBM Controller type */
typedef enum
{
    MTB_UBM_CONTROLLER_SPEC_DEFINED    = 0U, /**< Defined by UBM specification */
    MTB_UBM_CONTROLLER_VENDOR_SPECIFIC = 1U  /**< Vendor-specific */
} mtb_en_ubm_controller_type_t;

/** SATA link rate */
typedef enum
{
    MTB_UBM_SATA_RATE_NS = 0U,       /**< Not supported */
    MTB_UBM_SATA_3GBS_RATE  = 1U,    /**< 3 Gb/s */
    MTB_UBM_SATA_6GBS_RATE  = 2U,    /**< 6 Gb/s */
    MTB_UBM_SATA_NO_RATE_LIMIT = 3U  /**< No Limit */
} mtb_en_ubm_sata_link_rate_t;

/** The connector port type */
typedef enum
{
    MTB_UBM_PORT_TYPE_CONVERGED = 0U, /**< Supports PCIe and SAS/SATA via the same port. */
    MTB_UBM_PORT_TYPE_SEGREGATED = 1U /**< Supports PCIe and does not support SAS/SATA via the same port. */
} mtb_en_ubm_port_type_t;


/** The connector port domain */
typedef enum
{
    MTB_UBM_PORT_DOMAIN_PRIMARY = 0U,   /**< Primary port. */
    MTB_UBM_PORT_DOMAIN_SECONDARY = 1U  /**< Secondary port. */
} mtb_en_ubm_port_domain_t;

/** @} group_ubm_enums */

/**
 * \addtogroup group_ubm_fru_data_structures
 * @{
 */

 /** Features configuration structure */
typedef struct
{
    bool read_checksum_creation;                    /**< Indicates if the UBM Controller generates a valid Read Checksum for the read phase of a 2Wire transaction. */
    bool write_checksum_checking;                   /**< Indicates if the UBM Controller performs Checksum verification on the write phase of a 2Wire transaction. */
    bool cprsnt_legacy_mode;                        /**< Indicates the behavior of the CPRSNT#/CHANGE_DETECT# signal. */
    bool pcie_reset_change_count_mask;              /**< Indicates if a change to PCIe Reset field causes the Change Count field to increment. */
    bool drive_type_installed_change_count_mask;    /**< Indicates if a change to Drive Type Installed field causes the Change Count field to increment. */
    bool operational_state_change_count_mask;       /**< Indicatesif a change to Operational State field causes the Change Count field to increment. */
    uint8_t perst_management_override;              /**< Indicates the DFC PERST# behavior when a Drive has been installed. */
    bool smbus_reset_control;                       /**< Controls the DFC SMBRST# signal for all DFC’s associated under the HFC. */
} mtb_stc_ubm_features_t;

/** UBM FRU Overview Area configuration structure */
typedef struct
{
    uint8_t two_wire_device_arrangement;            /**< Two wire device arrangement. */
    uint8_t two_wire_mux_address;                   /**< Two wire mux adress. */
    uint8_t two_wire_max_byte_count;                /**< Two wire max byte count. */
    uint8_t ubm_max_time_limit;                     /**< Device max time limit. */
    mtb_stc_ubm_features_t ubm_controller_features; /**< Device features. */
    uint8_t maximum_power_per_dfc;                  /**< Maximum power per DFC. */
    uint8_t mux_channel_count;                      /**< Mux channel type. */
    uint8_t enable_bit_location;                    /**< Enable bit location. */
    uint8_t mux_type;                               /**< Mux type. */
} mtb_stc_ubm_fru_oa_config_t;

/** @} group_ubm_fru_data_structures */

/** DFC control signals configuration structure */
typedef struct
{
    cyhal_gpio_t ifdet;         /**< Presence detect pin. */
    cyhal_gpio_t ifdet2;        /**< Presence detect pin. */
    cyhal_gpio_t prsnt;         /**< Presence detect pin. */
    cyhal_gpio_t actdetect;     /**< Activity indicator pin. */
    cyhal_gpio_t persta;        /**< PCIe reset A pin. */
    cyhal_gpio_t perstb;        /**< PCIe reset B pin. */
    cyhal_gpio_t pwrdis;        /**< Power disable pin. */
    cyhal_gpio_t refclken;      /**< Reference clock enable pin. */
    cyhal_gpio_t dualporten;    /**< Dual-port enable pin. */
    cyhal_gpio_t hpt0;          /**< HPT0 pin. */
} mtb_stc_ubm_dfc_signals_t;


/** HFC control signals configuration structure */
typedef struct
{
    cyhal_gpio_t sda;           /**< Serial data pin. */
    cyhal_gpio_t scl;           /**< Serial clock pin. */
    cyhal_gpio_t i2c_reset;     /**< I2C interface reset pin. */
    cyhal_gpio_t change_detect; /**< Change detected pin. */
    cyhal_gpio_t bp_type;       /**< Backplane type pin. */
    cyhal_gpio_t perst;         /**< PCIe reset pin. */
} mtb_stc_ubm_hfc_signals_t;


/** Backplane control signals configuration structure */
typedef struct
{
    /** Device facing connectors control signals. */
    mtb_stc_ubm_dfc_signals_t dfc_io[MTB_UBM_DFC_NUM];

    /** Host facing connectors control signals. */
    mtb_stc_ubm_hfc_signals_t hfc_io[MTB_UBM_HFC_NUM];
} mtb_stc_ubm_backplane_control_signals_t;

/**
 * \addtogroup group_ubm_fru_data_structures
 * @{
 */

/** HFC routing configuration structure */
typedef struct
{
    mtb_en_ubm_link_width_t drive_link_width;       /**< Number of lanes in the port */
    uint8_t drive_connector_idx;                    /**< Device connector index */
    mtb_en_ubm_port_type_t port_type;               /**< Connector port type */
    mtb_en_ubm_port_domain_t domain;                /**< Connector port domain */
    uint8_t hfc_starting_phy_lane;                  /**< Host lane index */
    uint8_t hfc_identifier;                         /**< Host connector identity */
    uint8_t ubm_ctrl_slave_addr;                    /**< I2C address for UBM controller */
    mtb_en_ubm_controller_type_t ubm_ctrl_type;     /**< UBM controller type */
    uint8_t dfc_status_and_control;                 /**< DFC status and control descriptor index */
    uint8_t sff_ta_1001_support;                    /**< SFF-TA-1001 support */
    uint8_t gen_z_support;                          /**< Gen-Z support */
    uint8_t sas_sata_support;                       /**< SAS/SATA support */
    uint8_t quad_pcie_support;                      /**< Quad PCIe support */
    uint8_t dfc_empty;                              /**< DFC empty */
    mtb_en_ubm_sata_link_rate_t max_sata_rate;      /**< Maximum SATA rate */
    mtb_en_ubm_pcie_link_rate_t max_pcie_rate;      /**< Maximum PCIe rate */
    mtb_en_ubm_sas_link_rate_t max_sas_rate;        /**< Maximum SAS rate */
    uint8_t slot_offset;                            /**< Slot offset */
} mtb_stc_ubm_routing_t;

/** @} group_ubm_fru_data_structures */

/** Silicon Identity and Version configuration structure */
typedef struct
{
    uint16_t pcie_vendor_id;            /**< PCIe Vendor ID */
    uint32_t device_code;               /**< UBM Controller Device code */
    uint8_t fw_version_major;           /**< UBM Controller Image Version Minor */
    uint8_t fw_version_minor;           /**< UBM Controller Image Version Minor */
    uint16_t vendor_specific;           /**< UBM Controller vendor-specific data */
} mtb_stc_ubm_siv_t;


/** Backplane Info configuration structure */
typedef struct
{
    uint8_t backplane_type;             /**< Backplane type */
    uint8_t backplane_number;           /**< Backplane number */
} mtb_stc_ubm_bp_info_t;


/** Capabilities configuration structure */
typedef struct
{
    bool clock_routing;                 /**< Indicates availability of high speed differential clock routing (i.e., RefClk) 
                                             from the Host Facing Connector to the Drive Facing Connector */
    bool slot_power_control;            /**< Indicates if the Drive Facing Connectors support Power Disable (i.e., PwrDIS signal) */
    bool pcie_reset_control;            /**< Indicates if PCIe Reset Control is supported */
    bool dual_port;                     /**< Indicates if Dual Port DFC connectors are routed */
    uint8_t i2c_reset_operation;        /**< Indicates the 2WIRE_RESET# signal support */
    bool change_detect_interrupt;       /**< Indicates if the CHANGE_DETECT# signal interrupt operation is supported */
    bool dfc_change_count;              /**< Indicates if a change count is maintained per an individual DFC Status and Control Command Descriptor */
    bool prsnt_reported;                /**< Indicates if the PRSNT# signal is reported */
    bool ifdet_reported;                /**< Indicates if the IFDET# signal is reported */
    bool ifdet2_reported;               /**< Indicates if the IFDET2# signal is reported */
    bool perst_override_supported;      /**< Indicates if the UBM Controller supports the DFC 
                                             PERST# Management Override field in the Features Command */
    bool smb_reset_supported;           /**< Indicates if the UBM Controller supports control over the DFC 
                                             SMBRST# signals (e.g. See SFF-TA-1009) for all DFCs managed by the HFC */
} mtb_stc_ubm_capabilities_t;


/** UBM backplane configuration structure */
typedef struct
{
    bool hpt0_signal_support;                   /**< HPT0 signal support */
    cy_stc_eeprom_config_t* fru_config;         /**< Storage configuration for the FRU */
    mtb_stc_ubm_fru_oa_config_t *overview_area; /**< Overview area configuration */
    mtb_stc_ubm_routing_t hfc_routing[MTB_UBM_ROUTES_NUM]; /**< Backplane routing configuration */
    uint8_t starting_slot;                      /**< UBM starting slot */
    mtb_stc_ubm_siv_t silicon_identity;         /**< Silicon Identity and Version parameters */
    mtb_stc_ubm_bp_info_t backplane_info;       /**< Backplane Info parameters */
    mtb_stc_ubm_capabilities_t capabilities;    /**< UBM comtroller capabilities */
} mtb_stc_ubm_backplane_cfg_t;


/** UBM Status and Control Descriptor structure */
typedef struct
{
    uint8_t drive_type;                 /**< Indicates the type of device in the DFC */
    bool bifurcate_port;                /**< Indicates if the DFC port link width shall be bifurcated */
    uint8_t pcie_reset;                 /**< Specifies the port specific DFC PERST# signal behavior */
    uint8_t element_status_code;        /**< SES-4 Common Status Element Status Code bits */
    bool swap;                          /**< SES-4 Common Status Swap bit */
    bool disable;                       /**< SES-4 Common Status Disable bit */
    bool predicted_failure;             /**< SES-4 Common Status Predicted Failure bit */
    bool rebuild_remap_abort;           /**< SES-4 Rebuild/Remap Abort bit */
    bool rebuild_remap;                 /**< SES-4 Rebuild/Remap bit */
    bool in_failed_array;               /**< SES-4 In Failed Array bit */
    bool in_critical_array;             /**< SES-4 In Critical Array bit */
    bool consistency_check;             /**< SES-4 Consistency Check bit */
    bool hot_spare;                     /**< SES-4 Hot Spare bit */
    bool reserved_device;               /**< SES-4 Reserved Device bit */
    bool ok;                            /**< SES-4 OK bit */
    bool report;                        /**< SES-4 Report bit */
    bool identify;                      /**< SES-4 Identify bit */
    bool remove;                        /**< SES-4 Remove bit */
    bool insert;                        /**< SES-4 Insert bit */
    bool enclosure_bypassed_b;          /**< SES-4 Enclosure Bypassed Port B bit */
    bool enclosure_bypassed_a;          /**< SES-4 Enclosure Bypassed Port A bit */
    bool do_not_remove;                 /**< SES-4 Do Not Remove bit */
    bool app_client_bypassed_a;         /**< SES-4 Application Client Bypassed Port A bit */
    bool device_bypassed_b;             /**< SES-4 Device Bypassed Port B bit */
    bool device_bypassed_a;             /**< SES-4 Device Bypassed Port A bit */
    bool bypassed_b;                    /**< SES-4 Bypassed Port B bit */
    bool bypassed_a;                    /**< SES-4 Bypassed Port A bit */
    bool device_off;                    /**< SES-4 Device OFF bit */
    bool fault_requested;               /**< SES-4 Fault Requested bit */
    bool fault_sensed;                  /**< SES-4 Fault Sensed bit */
    bool app_client_bypassed_b;         /**< SES-4 Application Client Bypassed Port A bit */
    bool dfc_change_count;              /**< DFC Change Count indication */
} mtb_stc_ubm_status_and_control_descriptor_t;

/** UBM context structure forward declaration */
typedef struct mtb_stc_ubm_context mtb_stc_ubm_context_t;

/** UBM controller structure forward declaration */
typedef struct mtb_stc_ubm_controller mtb_stc_ubm_controller_t;

/** I2C event callback parameter structure */
typedef struct
{
    mtb_stc_ubm_context_t* ubm_context;        /**< Pointer to UBM context structure */
    mtb_stc_ubm_controller_t* ctrl_context;    /**< Pointer to controller context structure */
} mtb_stc_ubm_i2c_callback_arg_t;


/** UBM I2C interface structure */
typedef struct
{
    cyhal_i2c_t     scb_i2c_obj;                            /**< HAL I2C structure  */
    uint8_t         controller_address;                     /**< Slave address assigned to UBM Controller */
    uint8_t         received_address;                       /**< Slave address received from master */
    bool            read_request;                           /**< Flag indicating read request received */
    uint8_t write_buffer[MTB_UBM_I2C_WRITE_BUFFER_SIZE];    /**< Buffer to which master writes data to */
    uint8_t read_buffer[MTB_UBM_I2C_READ_BUFFER_SIZE];      /**< Buffer from which master reads data from */
    uint32_t write_data_length;                             /**< Data length in the write buffer */
    uint32_t read_data_length;                              /**< Data length in the read buffer */
    mtb_stc_ubm_i2c_callback_arg_t cb_arg;                  /**< Event callback parameter structure */
} mtb_stc_ubm_i2c_t;

#if(MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
/** UBM flash geometry structure */
typedef struct
{
    uint32_t      addr_start_upgrade_area;          /**< Address of start Upgrade area */
    uint32_t      size_upgrade_area;                /**< Size upgrade area. Size the application */
    uint32_t      offset_row_buffer;                /**< Value of the offset for write row_buffer */
    cyhal_flash_t flash_obj;                        /**< The HAL Flash structure */
    uint8_t       row_buffer[CY_FLASH_SIZEOF_ROW];  /**< The data is to be programmed at the specified Sector Index */
    uint8_t       app_sequence_number;              /**< Indicates the sequence number of the Program Subcommand */
    uint8_t       image_number;                     /**< The Image Number field specifies the Image Number associated
                                                         with the vendor-specific data set in the Non-Volatile Storage. */
    uint8_t       sector_index;                     /**< The Sector Index field indicates the Sector Index in the 
                                                         Sector of the Non-Volatile Storage. */
    uint8_t       checksum_sector_index;            /**< The Sector Index Checksum field indicates the two's
                                                         complement of the summation of bytes located at the Sector Index. */
    bool          status_download_image;            /**< The status of verefication upgrade image. */
} mtb_stc_ubm_flash_geometry_t;
#endif /*MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED*/

/** GPIO callbacks configuration structure */
typedef struct mtb_stc_ubm_gpio_callbacks
{
    cyhal_gpio_callback_data_t perst;               /**< Callback configuration data for the PERST signal */
    cyhal_gpio_callback_data_t device_det;          /**< Callback configuration data for the device detecton event */
    cyhal_gpio_callback_data_t activity_det;        /**< Callback configuration data for the activity detecton event */
    cyhal_gpio_callback_data_t two_wire_reset;      /**< Callback configuration data for the 2 Wire Reset event */
} mtb_stc_ubm_gpio_callbacks_t;

/** Input signal structure */
typedef struct
{
    cyhal_gpio_t pin;                       /**< Assigned PIN */
    uint32_t asserted_count;                /**< Timing of the asserted signal */
} cy_stc_ubm_input_t;

/** UBM controller structure */
struct mtb_stc_ubm_controller
{
    mtb_en_ubm_op_state_t state;                        /**< Operational state of the UBM controller */
    mtb_en_ubm_lc_sts_t last_command_status;            /**< Status of the last received UBM command */
    uint8_t status_n_control_descriptor_index;          /**< DFC Status and Control Descriptor Index */
    uint8_t status_n_control_descriptor_index_count;    /**< Number of DFC Status and Control Descriptor Index */
    mtb_stc_ubm_i2c_t i2c;                              /**< I2C interface structure */
    uint8_t index;                                      /**< UBM controller index */
    mtb_stc_ubm_dfc_signals_t dfc_io;                   /**< Device facing connectors control signals */
    mtb_stc_ubm_hfc_signals_t hfc_io;                   /**< Host facing connectors control signals */
    mtb_stc_ubm_gpio_callbacks_t cb_args;               /**< GPIO callbacks configuration structure */
    uint8_t detected_device;                            /**< Detected devices array */
    cy_stc_ubm_input_t reset;                           /**< 2 Wire/silicon reset signal */
    bool legacy_mode;                                   /**< Legacy mode of CPRSNT/CHANGE_DETECT signal */
    mtb_stc_ubm_status_and_control_descriptor_t scd[MTB_UBM_DFC_NUM]; /**< DFC Status and Control Descriptor */
    bool domain[MTB_UBM_DFC_NUM];                       /**< Domain settings for each DFC */
};


/** UBM context structure.
 * All fields for the context structure are internal. Firmware never reads or
 * writes these values. Firmware allocates the structure and provides the
 * address of the structure to the driver in function calls. Firmware must
 * ensure that the defined instance of this structure remains in scope
 * while the drive is in use.
 */
struct mtb_stc_ubm_context
{
    /** \cond INTERNAL */
    mtb_stc_ubm_controller_t ctrl[MTB_UBM_HFC_NUM];  /**< UBM controller context structure */

    #if(MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
    mtb_en_ubm_pm_sts_t update_subcmd_status;        /**< Programmable mode status */
    mtb_stc_ubm_flash_geometry_t flash_layout;       /**< UBM flash geometry structure */
    #endif /*MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED*/

    cy_stc_eeprom_context_t fru_context;             /**< FRU cotext */
    uint8_t starting_slot;                           /**< UBM Starting slot */
    mtb_stc_ubm_siv_t silicon_identity;              /**< UBM Controller silicon identity and version */
    mtb_stc_ubm_bp_info_t backplane_info;            /**< UBM Controller Backplane Info */
    mtb_stc_ubm_capabilities_t capabilities;         /**< UBM Controller capabilities */
    cyhal_timer_t timer_obj;                         /**< Timer structure */
    /** \endcond */
};


#endif /* MTB_UBM_TYPES_H */
