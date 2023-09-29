/***************************************************************************//**
 * \file mtb_ubm_controller.c
 * \version 1.0
 *
 * \brief
 * Provides common API implementation for the UBM controller.
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

#include "mtb_ubm_controller.h"
#include "mtb_ubm.h"
#include "mtb_ubm_config.h"
#include "mtb_ubm_io.h"
#include <string.h>

/* The response data length for the operational state command */
#define OP_STATE_CMD_RSP_LEN                (1U)
/* The response data length for the Last Command Status command */
#define LC_STATUS_CMD_RSP_LEN               (1U)
/* The response data length for the Update mode capabilities command */
#define UPDATE_MODE_CAP_CMD_RSP_LEN         (1U)
/* The response data length for the UBM Starting Slot Command */
#define STARTING_SLOT_CMD_RSP_LEN           (1U)
/* The response data length for UBM Backplane Info Command */
#define BACKPLANE_INFO_CMD_RSP_LEN          (1U)
/* The response data length for the Features Command */
#define FEATURES_CMD_RSP_LEN                (2U)
/* The response data length for the Capabilities Command */
#define CAPABILITIES_CMD_RSP_LEN            (2U)
/* The response data length for the HFC Info Command */
#define HFC_INFO_CMD_RSP_LEN                (1U)

#define DATA_BYTE_0_INDEX                   (0)
#define DATA_BYTE_1_INDEX                   (1)
#define DATA_BYTE_2_INDEX                   (2)
#define DATA_BYTE_3_INDEX                   (3)
#define DATA_BYTE_4_INDEX                   (4)
#define DATA_BYTE_5_INDEX                   (5)
#define DATA_BYTE_6_INDEX                   (6)
#define DATA_BYTE_7_INDEX                   (7)
#define DATA_BYTE_8_INDEX                   (8)
#define DATA_BYTE_9_INDEX                   (9)
#define DATA_BYTE_10_INDEX                  (10)
#define DATA_BYTE_11_INDEX                  (11)
#define DATA_BYTE_12_INDEX                  (12)
#define DATA_BYTE_13_INDEX                  (13)

#if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
/* The defult length value of the PMDT subcommand */
#define MTB_UBM_SUBCMD_DEFAULT_LEN          (4U)
/* The response data length for the Enter Programmable Update Mode commands */
#define ENTER_UPDATE_MODE_CMD_RSP_LEN       (4U)
/* The response data length for the Exit Programmable Update Mode commands */
#define EXIT_UPDATE_MODE_CMD_RSP_LEN        (4U)
/* The response data length for the Get Non-Volatile Storage Geometry commands */
#define GET_NVS_CMD_RSP_LEN                 (6U)
/* The response data length for the Erase Status Subcommands */
#define ERASE_STATUS_SUBCMD_RSP_LEN         (4U)
/* The response data length for the Program Status Subcommands */
#define PROGRAM_STATUS_SUBCMD_RSP_LEN       (3U)
/* The response data length for the Verify Status Subcommands */
#define VERIFY_STATUS_SUBCMD_RSP_LEN        (5U)
/* The response data length for the Active Image Status Subcommands */
#define ACTIVE_IMAGE_STATUS_SUBCMD_RSP_LEN  (3U)
/* The response data length for the Verify Image Status Subcommands */
#define VERIFY_IMAGE_STATUS_SUBCMD_RSP_LEN  (3U)
/* The specific sequence to unlock/lock Programmable Update Mode */
#define UPDATE_MODE_SPECIFIC_BYTE_0         (0x55U)
#define UPDATE_MODE_SPECIFIC_BYTE_1         (0x42U)
#define UPDATE_MODE_SPECIFIC_BYTE_2         (0x4DU)
/* The status of the transfer to Update mode */
#define TRANSFER_TO_UPDATE_MODE_Msk         (0x01U)
/* The structure of the write request of the UBM Command Enter/Exit Programmable Update Mode. */
#define UPDATE_MODE_SPECIFIC_BYTE_0_Pos     (1U)
#define UPDATE_MODE_SPECIFIC_BYTE_1_Pos     (2U)
#define UPDATE_MODE_SPECIFIC_BYTE_2_Pos     (3U)
#define TRANSFER_UPDATE_MODE_Pos            (4U)
/* The structure of the UBM Command Enter Programmable Update Mode read request. */
#define SLAVE_ADR_TO_UPDATE_MODE_Pos        (0U)
#define UNLOCK_BYTE_0_Pos                   (1U)
#define UNLOCK_BYTE_1_Pos                   (2U)
#define UNLOCK_BYTE_2_Pos                   (3U)
/* The structure of the UBM Command Exit Programmable Update Mode read request. */
#define LOCK_BYTE_0_Pos                     (0U)
#define LOCK_BYTE_1_Pos                     (1U)
#define LOCK_BYTE_2_Pos                     (2U)
#define OPERATIONAL_MODE_TRANSFER_Pos       (3U)
#define OPERATIONAL_MODE_TRANSFER           (0x01U)
/* The position subcommand byte into the UBM packet */
#define SUBCOMMAND_BYTE_Pos                 (1U)
/* The structure of the Get Non-Volatile Storage Geometry Subcommand. */
#define GNVSG_SECTORS_NUMBER                (1U)
#define GNVSG_FIRST_SECTOR_INDEX            (0U)
#define GNVSG_DATA_LENGTH                   (4U)
/* The structure of the UBM Program Subcommand. */
#define PS_DATA_LENGTH_Pos                  (2U)
#define PS_SECTOR_INDEX_Pos                 (4U)
#define PS_APP_SEQUANCE_NUMBER_Pos          (5U)
#define PS_FIRST_DATA_BYTE_Pos              (6U)
#define PS_NUMBER_SYSTEM_BYTE               (4U)
#define PS_NUMBER_UNFLASH_DATA              (3U)
/* The structure of the UBM Program Status Subcommand. */
#define PSS_SUBCOMMAND_STATUS_Pos           (0U)
#define PSS_DATA_LENGTH_Pos                 (1U)
#define PSS_APP_SEQUANCE_NUMBER_Pos         (2U)
#define PSS_DATA_LENGTH                     (1U)
/* The structure of the UBM Erase Subcommand. */
#define ES_DATA_LENGTH_Pos                  (2U)
#define ES_SECTOR_NUMBER_Pos                (3U)
#define ES_SECTOR_INDEX_Pos                 (4U)
#define ES_NUMBER_SYSTEM_BYTE               (4U)
/* The structure of the UBM Erase Status Subcommand. */
#define ESS_SUBCOMMAND_STATUS_Pos           (0U)
#define ESS_DATA_LENGTH_Pos                 (1U)
#define ESS_SECTOR_NUMBER_Pos               (2U)
#define ESS_SECTOR_INDEX_Pos                (3U)
#define ESS_DATA_LENGTH                     (2U)
/* The structure of the UBM Verify Subcommand */
#define VS_DATA_LENGTH_Pos                  (2U)
#define VS_SECTOR_NUMBER_Pos                (3U)
#define VS_SECTOR_INDEX_Pos                 (4U)
#define VS_NUMBER_SYSTEM_BYTE               (4U)
/* The structure of the UBM Verify Status Subcommand */
#define VSS_SUBCOMMAND_STATUS_Pos           (0U)
#define VSS_DATA_LENGTH_Pos                 (1U)
#define VSS_SECTOR_NUMBER_Pos               (2U)
#define VSS_SECTOR_INDEX_Pos                (3U)
#define VSS_CHECKSUM_SECTOR_INDEX_Pos       (4U)
#define VSS_DATA_LENGTH                     (3U)
/* The structure of the UBM Set Active Image Subcommand. */
#define SI_DATA_LENGTH_Pos                  (2U)
#define SI_IMAGE_NUMBER_Pos                 (3U)
#define SI_SUBCMD_OVERHEAD                  (4U)
/* The structure of the UBM Set Active Image Status Subcommand. */
#define SIS_SUBCOMMAND_STATUS_Pos           (0U)
#define SIS_DATA_LENGTH_Pos                 (1U)
#define SIS_IMAGE_NUMBER_Pos                (2U)
#define SiS_DATA_LENGTH                     (1U)
/* The structure of the UBM Verify Image Subcommand. */
#define VI_DATA_LENGTH_Pos                  (2U)
#define VI_IMAGE_NUMBER_Pos                 (3U)
#define VI_NUMBER_SYSTEM_BYTE               (4U)
/* The size of the verify image checksum CRC lookup table */
#define VI_CRC_TABLE_SIZE                   (16U)
/* The initial value for the verify image checksum */
#define VI_CRC_INIT_VALUE                   (0xFFFFFFFFU)
/* The structure of the UBM Verify Image Status Subcommand. */
#define VIS_SUBCOMMAND_STATUS_Pos           (0U)
#define VIS_DATA_LENGTH_Pos                 (1U)
#define VIS_IMAGE_NUMBER_Pos                (2U)
#define VIS_DATA_LENGTH                     (1U)
#endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */

/* Write commands length */
#define FEATURES_CMD_WRITE_LEN                      (4U)
#define DFC_STATUS_AND_CONTROL_DESCRIPTOR_INDEX_LEN (3U)
#define DFC_STATUS_AND_CONTROL_DESCRIPTOR_LEN       (10U)
#define ENTER_EXIT_PROGRAMMABLE_UPDATE_MODE_LEN     (6U)
#define CHANGE_COUNT_LEN                            (4U)

/* The structure of the HFC Info Command */
#define HFC_INFO_IDENTITY_Msk               (0x0FU)
#define HFC_INFO_IDENTITY_Pos               (0U)
#define HFC_INFO_PORT_TYPE_Msk              (0x80U)
#define HFC_INFO_PORT_TYPE_Pos              (7U)
/* The structure of the Backplane Info Command */
#define BACKPLANE_INFO_NUMBER_Msk           (0x0FU)
#define BACKPLANE_INFO_NUMBER_Pos           (0U)
#define BACKPLANE_INFO_TYPE_Msk             (0xE0U)
#define BACKPLANE_INFO_TYPE_Pos             (5U)
/* The structure of the Capabilities Command */
#define CAP_CLOCK_ROUTING_Pos               (0U)
#define CAP_CLOCK_ROUTING_Msk               (0x01U)
#define CAP_SLOT_POWER_CONTROL_Pos          (1U)
#define CAP_SLOT_POWER_CONTROL_Msk          (0x02U)
#define CAP_PCIE_RESET_CONTROL_Pos          (2U)
#define CAP_PCIE_RESET_CONTROL_Msk          (0x04U)
#define CAP_DUAL_PORT_Pos                   (3U)
#define CAP_DUAL_PORT_Msk                   (0x08U)
#define CAP_2WIRE_RESET_OP_Pos              (4U)
#define CAP_2WIRE_RESET_OP_Msk              (0x30U)
#define CAP_CHANGE_DET_INT_OP_Pos           (6U)
#define CAP_CHANGE_DET_INT_OP_Msk           (0x40U)
#define CAP_CHANGE_COUNT_Pos                (7U)
#define CAP_CHANGE_COUNT_Msk                (0x80U)
#define CAP_PRSNT_REPORT_Pos                (0U)
#define CAP_PRSNT_REPORT_Msk                (0x01U)
#define CAP_IFDET_REPORT_Pos                (1U)
#define CAP_IFDET_REPORT_Msk                (0x02U)
#define CAP_IFDET2_REPORT_Pos               (2U)
#define CAP_IFDET2_REPORT_Msk               (0x04U)
#define CAP_PERST_OVERRIDE_SUPPORT_Pos      (3U)
#define CAP_PERST_OVERRIDE_SUPPORT_Msk      (0x08U)
#define CAP_SMBUS_RESET_SUPPORT_Pos         (4U)
#define CAP_SMBUS_RESET_SUPPORT_Msk         (0x10U)
/* The structure of the Features Command */
#define FEATURES_READ_CHECKSUM_Pos          (0U)
#define FEATURES_READ_CHECKSUM_Msk          (0x01U)
#define FEATURES_WRITE_CHECKSUM_Pos         (1U)
#define FEATURES_WRITE_CHECKSUM_Msk         (0x02U)
#define FEATURES_CPRSNT_MODE_Pos            (2U)
#define FEATURES_CPRSNT_MODE_Msk            (0x04U)
#define FEATURES_PCIE_RESET_CC_Pos          (3U)
#define FEATURES_PCIE_RESET_CC_Msk          (0x08U)
#define FEATURES_DRIVE_TYPE_CC_Pos          (4U)
#define FEATURES_DRIVE_TYPE_CC_Msk          (0x10U)
#define FEATURES_OP_STATE_CC_Pos            (5U)
#define FEATURES_OP_STATE_CC_Msk            (0x20U)
#define FEATURES_PERST_OVRD_Pos             (6U)
#define FEATURES_PERST_OVRD_Msk             (0xC0U)
#define FEATURES_SMBUS_RESET_Pos            (0U)
#define FEATURES_SMBUS_RESET_Msk            (0x01U)

#define MTB_UBM_CTR_FIRST_BYTE_MASK         (0x00FFU)
#define MTB_UBM_CTR_SECOND_BYTE_MASK        (0xFF00U)
#define MTB_UBM_CTR_FIRST_BYTE_SHIFT        (0U)
#define MTB_UBM_CTR_SECOND_BYTE_SHIFT       (8U)

#define MTB_UBM_CMD_DEFAULT_LEN             (2U)
#define MTB_UBM_CMD_PROG_ENTER_EXIT_LEN     (6U)
#define MTB_UBM_CMD_STS_AND_CONTROL_LEN     (8U)

/* The structure of the Silicon Identity Command */
#define SI_IDENTITY_AND_VERSION_CMD_RSP_LEN (14U)
#define SI_IDENTITY_AND_VERSION_BYTE0_POS   (0U)
#define SI_IDENTITY_AND_VERSION_BYTE1_POS   (1U)
#define SI_IDENTITY_AND_VERSION_BYTE2_POS   (2U)
#define SI_IDENTITY_AND_VERSION_BYTE3_POS   (3U)
#define SI_IDENTITY_AND_VERSION_BYTE4_POS   (4U)
#define SI_IDENTITY_AND_VERSION_BYTE5_POS   (5U)
#define SI_IDENTITY_AND_VERSION_BYTE6_POS   (6U)
#define SI_IDENTITY_AND_VERSION_BYTE7_POS   (7U)
#define SI_IDENTITY_AND_VERSION_BYTE8_POS   (8U)
#define SI_IDENTITY_AND_VERSION_BYTE9_POS   (9U)
#define SI_IDENTITY_AND_VERSION_BYTE10_POS  (10U)
#define SI_IDENTITY_AND_VERSION_BYTE11_POS  (11U)
#define SI_IDENTITY_AND_VERSION_BYTE12_POS  (12U)
#define SI_IDENTITY_AND_VERSION_BYTE13_POS  (13U)
/* The structure of the Change Count Command */
#define CC_CMD_LEN                          (2U)
#define CC_CMD_BYTE0                        (0U)
#define CC_CMD_BYTE1                        (1U)
#define CC_CMD_CPRSNT_MODE_CHANGE_SRC_Pos   (0U)
#define CC_CMD_PCIE_RESET_CHANGE_SRC_Pos    (3U)
#define CC_CMD_DRIVE_TYPE_CHANGE_SRC_Pos    (4U)
#define CC_CMD_OP_STATE_CHANGE_SRC_Pos      (5U)
#define CC_CMD_CTRL_RESET_CHANGE_SRC_Pos    (7U)
/* The structure of the UBM DFC Status and Control Descriptor Index Command */
#define SCDI_CMD_INDEX_Pos                  (0U)
#define SCDI_CMD_LEN                        (1U)
/* The invalid value of the DFC Status and Control Descriptor Index */
#define SCD_INVALID_INDEX                   (0xFFU)
/* The structure of the UBM DFC Status and Control Descriptor Command */
#define SCD_CMD_LEN                         (8U)
#define SCD_CMD_RSP_LEN                     (8U)
#define SCD_CMD_BYTE0                       (0U)
#define SCD_CMD_BYTE1                       (1U)
#define SCD_CMD_BYTE2                       (2U)
#define SCD_CMD_BYTE3                       (3U)
#define SCD_CMD_BYTE4                       (4U)
#define SCD_CMD_BYTE5                       (5U)
#define SCD_CMD_BYTE6                       (6U)
#define SCD_CMD_BYTE7                       (7U)
#define SCD_CMD_BYTE8                       (8U)
#define SCD_SES_BYTE0                       (0U)
#define SCD_SES_BYTE1                       (1U)
#define SCD_SES_BYTE2                       (2U)
#define SCD_SES_BYTE3                       (3U)
#define SCD_CMD_DRIVE_TYPE_Pos              (0U)
#define SCD_CMD_DRIVE_TYPE_Msk              (0x07U)
#define SCD_CMD_BIFURCATE_PORT_Pos          (5U)
#define SCD_CMD_BIFURCATE_PORT_Msk          (0x20U)
#define SCD_CMD_PCIE_RESET_Pos              (6U)
#define SCD_CMD_PCIE_RESET_Msk              (0xC0U)
#define SCD_CMD_PCIE_RESET_NOP              (0x00U)
#define SCD_CMD_PCIE_RESET_INIT             (0x01U)
#define SCD_CMD_PCIE_RESET_HOLD             (0x02U)
#define SCD_CMD_PCIE_RESET_RESERVED         (0x03U)
#define SCD_CMD_SAS_SELECT_Pos              (7U)
#define SCD_CMD_SAS_SELECT_Msk              (0x80U)
#define SCD_CMD_SAS_DEVICE_OFF_Pos          (4U)
#define SCD_CMD_SAS_DEVICE_OFF_Msk          (0x10U)
/* The value of the delay release DFC_PERST signal */
#define MTB_UBM_PERST_LONG_DELAY            (91U) /* This value is not equal
                                                     to 100 because the overhead
                                                     of the HAL library leads to an
                                                     increase in the processing time. */
#define MTB_UBM_PERST_SHORT_DELAY           (1U)

#define DEVICE_TYPE_EMPTY                   (0x07U)

#define CHECKSUM_SEED_VALUE                 (0xA5U)

#define UBM_UPGRADE_IMAGE_CRC_OFFSET        (0x200U)


/*******************************************************************************
*            Internal API
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_operational_state(const mtb_stc_ubm_context_t* ubm_context,
                                                    mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_last_command_status(mtb_stc_ubm_hfc_t* hfc_context,
                                                      const mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_silicon_identity_and_version(const mtb_stc_ubm_context_t* ubm_context,
                                                               mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_programmable_update_mode_capabilities(mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_host_facing_connector_info(mtb_stc_ubm_hfc_t* hfc_context,
                                                             const mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_backplane_info(const mtb_stc_ubm_context_t* ubm_context,
                                                 mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_starting_slot(const mtb_stc_ubm_context_t* ubm_context,
                                                mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_capabilities(const mtb_stc_ubm_context_t* ubm_context,
                                               mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_features(const mtb_stc_ubm_context_t* ubm_context,
                                           mtb_stc_ubm_hfc_t* hfc_context,
                                           mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_change_count(const mtb_stc_ubm_context_t* ubm_context,
                                               mtb_stc_ubm_hfc_t* hfc_context,
                                               mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_dfc_status_and_control_descriptor_index(mtb_stc_ubm_hfc_t* hfc_context,
                                                                          mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_dfc_status_and_control_descriptor(mtb_stc_ubm_context_t* ubm_context,
                                                                    mtb_stc_ubm_hfc_t* hfc_context,
                                                                    mtb_stc_ubm_controller_t* ctrl_context);
static uint8_t get_checksum(const uint8_t* data_bytes, uint32_t data_length);
static uint8_t calculate_checksum(uint8_t address_byte, const uint8_t* data_bytes, uint32_t data_length);
static mtb_en_ubm_lc_sts_t validate_command_length(const mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t get_last_command_status(const mtb_stc_ubm_controller_t* ctrl_context);
static void set_last_command_status(mtb_stc_ubm_controller_t* ctrl_context, mtb_en_ubm_lc_sts_t status);
static void process_pwrdis_signal(mtb_stc_ubm_context_t* ubm_context,
                                  mtb_stc_ubm_dfc_t* dfc_context,
                                  bool new_device_off_state);


#if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
static mtb_en_ubm_lc_sts_t handle_enter_programmable_update_mode(mtb_stc_ubm_context_t* ubm_context,
                                                                 mtb_stc_ubm_hfc_t* hfc_context,
                                                                 const mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_programmable_mode_data_transfer(mtb_stc_ubm_context_t* ubm_context,
                                                                  mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_exit_programmable_update_mode(mtb_stc_ubm_context_t* ubm_context,
                                                                mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_get_non_volatile_storage_geometry_sub(mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_erase_sub(mtb_stc_ubm_context_t* ubm_context,
                                            const mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_erase_status_sub(const mtb_stc_ubm_context_t* ubm_context,
                                                   mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_program_sub(mtb_stc_ubm_context_t* ubm_context,
                                              const mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_program_status_sub(const mtb_stc_ubm_context_t* ubm_context,
                                                     mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_verify_sub(mtb_stc_ubm_context_t* ubm_context,
                                             const mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_verify_status_sub(const mtb_stc_ubm_context_t* ubm_context,
                                                    mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_verify_image_sub(mtb_stc_ubm_context_t* ubm_context,
                                                   const mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_verify_image_status_sub(const mtb_stc_ubm_context_t* ubm_context,
                                                          mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_set_active_image_sub(mtb_stc_ubm_context_t* ubm_context,
                                                       const mtb_stc_ubm_hfc_t* hfc_context);
static mtb_en_ubm_lc_sts_t handle_set_active_image_status_sub(const mtb_stc_ubm_context_t* ubm_context,
                                                              mtb_stc_ubm_hfc_t* hfc_context);
static uint8_t log_base_2(uint32_t argument);
static uint32_t get_address_from_row_flash(uint8_t row_num, uint32_t start_addr);
static uint8_t calculate_index_checksum(const uint8_t* data_bytes, uint32_t data_length);
static uint32_t verify_image_checksum(const uint8_t* image_start, uint32_t image_length);
static void switch_to_bootloader(const mtb_stc_ubm_context_t* ubm_context);
#endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */

/*******************************************************************************
* Function Name: mtb_ubm_get_packet_command
****************************************************************************//**
*
*  Retrieves the UBM controller command byte from the received packet.
*
* \param hfc_context
*  The pointer to the UBM controller context.
*
* \return
*  The received command byte.
*
*******************************************************************************/
mtb_ubm_cmd_t mtb_ubm_get_packet_command(const mtb_stc_ubm_hfc_t* hfc_context)
{
    /* The command byte is the first byte of the received packet. */
    mtb_ubm_cmd_t command = hfc_context->i2c.write_buffer[DATA_BYTE_0_INDEX];

    return command;
}


/*******************************************************************************
* Function Name: mtb_ubm_controller_handle_read_request
****************************************************************************//**
*
*  Handles requests from HFC master to the UBM Controller.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
*******************************************************************************/
void mtb_ubm_controller_handle_request(mtb_stc_ubm_context_t* ubm_context,
                                       mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_ubm_cmd_t command = MTB_UBM_PM_CMD_INVALID;
    mtb_en_ubm_op_state_t state = MTB_UBM_OP_STATE_INVALID;
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_SUCCESS;
    mtb_stc_ubm_controller_t* ctrl_context =
        &ubm_context->ctrl[hfc_context->ctrl_list[hfc_context->selected_ctrl_index]];

    state = mtb_ubm_get_op_state(ubm_context);

    if (MTB_UBM_OP_STATE_BUSY == state)
    {
        /* Busy processing the last command request */
        status = MTB_UBM_LC_STS_BUSY;
    }

    if ((MTB_UBM_LC_STS_SUCCESS == status) && ctrl_context->features.write_checksum_checking)
    {
        uint8_t received_checksum;
        uint8_t calculated_checksum;

        received_checksum = get_checksum(hfc_context->i2c.write_buffer,
                                         hfc_context->i2c.write_data_length);
        calculated_checksum = calculate_checksum(hfc_context->selected_slave_address << 1U,
                                                 hfc_context->i2c.write_buffer,
                                                 hfc_context->i2c.write_data_length - 1U);

        if (received_checksum != calculated_checksum)
        {
            /* Checksum is invalid */
            status = MTB_UBM_LC_STS_INVALID_CHECKSUM;
        }
    }

    if (MTB_UBM_LC_STS_SUCCESS == status)
    {
        status = validate_command_length(hfc_context);
    }

    command = mtb_ubm_get_packet_command(hfc_context);

    if (MTB_UBM_LC_STS_SUCCESS == status)
    {
        /* Handle command */
        switch (command)
        {
        case MTB_UBM_CMD_OPERATIONAL_STATE:
            status = handle_operational_state(ubm_context, hfc_context);
            break;
        case MTB_UBM_CMD_LAST_COMMAND_STATUS:
            status = handle_last_command_status(hfc_context, ctrl_context);
            break;
        case MTB_UBM_CMD_SILICON_IDENTITY_AND_VERSION:
            status = handle_silicon_identity_and_version(ubm_context, hfc_context);
            break;
        case MTB_UBM_CMD_PROGRAMMABLE_UPDATE_MODE_CAPABILITIES:
            status = handle_programmable_update_mode_capabilities(hfc_context);
            break;
        #if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
        case MTB_UBM_CMD_ENTER_PROGRAMMABLE_UPDATE_MODE:
            status = handle_enter_programmable_update_mode(ubm_context, hfc_context, ctrl_context);
            break;
        case MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER:
            status = handle_programmable_mode_data_transfer(ubm_context, hfc_context);
            break;
        case MTB_UBM_CMD_EXIT_PROGRAMMABLE_UPDATE_MODE:
            status = handle_exit_programmable_update_mode(ubm_context, hfc_context);
            switch_to_bootloader(ubm_context);
            break;
        #endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */
        case MTB_UBM_CMD_HOST_FACING_CONNECTOR_INFO:
            status = handle_host_facing_connector_info(hfc_context, ctrl_context);
            break;
        case MTB_UBM_CMD_BACKPLANE_INFO:
            status = handle_backplane_info(ubm_context, hfc_context);
            break;
        case MTB_UBM_CMD_STARTING_SLOT:
            status = handle_starting_slot(ubm_context, hfc_context);
            break;
        case MTB_UBM_CMD_CAPABILITIES:
            status = handle_capabilities(ubm_context, hfc_context);
            break;
        case MTB_UBM_CMD_FEATURES:
            status = handle_features(ubm_context, hfc_context, ctrl_context);
            break;
        case MTB_UBM_CMD_CHANGE_COUNT:
            status = handle_change_count(ubm_context, hfc_context, ctrl_context);
            break;
        case MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR_INDEX:
            status = handle_dfc_status_and_control_descriptor_index(hfc_context, ctrl_context);
            break;
        case MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR:
            status = handle_dfc_status_and_control_descriptor(ubm_context, hfc_context, ctrl_context);
            break;
        default:
            status = MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
            break;
        }
    }

    if ((MTB_UBM_LC_STS_SUCCESS == status) &&
        hfc_context->i2c.read_request &&
        ctrl_context->features.read_checksum_creation)
    {
        /* Calculate the checksum */
        if (hfc_context->i2c.read_data_length < MTB_UBM_I2C_READ_BUFFER_SIZE)
        {
            hfc_context->i2c.read_buffer[hfc_context->i2c.read_data_length] =
                calculate_checksum(0U, hfc_context->i2c.read_buffer, hfc_context->i2c.read_data_length);

            ++hfc_context->i2c.read_data_length;
        }
        else
        {
            status = MTB_UBM_LC_STS_FAILED;
        }
    }

    set_last_command_status(ctrl_context, status);
}


/*******************************************************************************
* Function Name: mtb_ubm_get_op_state
****************************************************************************//**
*
*  Returns the current operational state of the UBM Controller.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \return
*  See mtb_en_ubm_op_state_t.
*
*******************************************************************************/
mtb_en_ubm_op_state_t mtb_ubm_get_op_state(const mtb_stc_ubm_context_t* ubm_context)
{
    return ubm_context->state;
}


/*******************************************************************************
* Function Name: mtb_ubm_set_op_state
****************************************************************************//**
*
*  Saves the current operational state of the UBM Controller.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param state
*  The current state of the UBM Controller to be saved.
*
*******************************************************************************/
void mtb_ubm_set_op_state(mtb_stc_ubm_context_t* ubm_context,
                          mtb_en_ubm_op_state_t state)
{
    if (ubm_context->state != state)
    {
        ubm_context->state = state;
        for (uint32_t ctrl_index = 0U; ctrl_index < ubm_context->num_of_ctrls; ctrl_index++)
        {
            mtb_ubm_update_change_count(ubm_context,
                                        &ubm_context->ctrl[ctrl_index],
                                        NULL,
                                        MTB_UBM_CC_SOURCE_OP_STATE);
        }
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_update_change_count
****************************************************************************//**
*
*  Increments the Change Count and sets an appropriate change source.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \param dfc_context
*  The pointer to the DFC context structure.
*
* \param source
*  The source of the change.
*
*******************************************************************************/
void mtb_ubm_update_change_count(const mtb_stc_ubm_context_t* ubm_context,
                                 mtb_stc_ubm_controller_t* ctrl_context,
                                 const mtb_stc_ubm_dfc_t* dfc_context,
                                 mtb_en_ubm_change_count_source_t source)
{
    uint8_t change_count_before = ctrl_context->change_count.change_count;
    mtb_en_ubm_op_state_t state = mtb_ubm_get_op_state(ubm_context);
    bool change_count_updated = true;

    switch (source)
    {
    case MTB_UBM_CC_SOURCE_CPRSNT_LEGACY_MODE:
        if (MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY != state)
        {
            ctrl_context->change_count.source_cprsnt_legacy_mode_change = true;
            ctrl_context->change_count.change_count++;
        }
        break;

    case MTB_UBM_CC_SOURCE_PCIE_RESET:
        if (ctrl_context->features.pcie_reset_change_count_mask)
        {
            if (MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY != state)
            {
                ctrl_context->change_count.source_pcie_reset_change = true;
                ctrl_context->change_count.change_count++;
            }

            if (ubm_context->capabilities.dfc_change_count_supported && (NULL != dfc_context))
            {
                if (ctrl_context->scd[dfc_context->index].dfc_change_count == (uint8_t)UINT8_MAX)
                {
                    ctrl_context->scd[dfc_context->index].dfc_change_count = 1U;
                }
                else
                {
                    ctrl_context->scd[dfc_context->index].dfc_change_count++;
                }
            }
        }
        break;

    case MTB_UBM_CC_SOURCE_DRIVE_TYPE:
        if (ctrl_context->features.drive_type_installed_change_count_mask)
        {
            if (MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY != state)
            {
                ctrl_context->change_count.source_drive_type_installed_change = true;
                ctrl_context->change_count.change_count++;
            }

            if (ubm_context->capabilities.dfc_change_count_supported && (NULL != dfc_context))
            {
                if (ctrl_context->scd[dfc_context->index].dfc_change_count == (uint8_t)UINT8_MAX)
                {
                    ctrl_context->scd[dfc_context->index].dfc_change_count = 1U;
                }
                else
                {
                    ctrl_context->scd[dfc_context->index].dfc_change_count++;
                }
            }
        }
        break;

    case MTB_UBM_CC_SOURCE_OP_STATE:
        if (ctrl_context->features.operational_state_change_count_mask)
        {
            if (MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY != state)
            {
                ctrl_context->change_count.source_operational_state_change = true;
                ctrl_context->change_count.change_count++;
            }
        }
        break;

    case MTB_UBM_CC_SOURCE_CTRL_RESET:
        if (MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY != state)
        {
            ctrl_context->change_count.source_controller_reset_change = true;
            ctrl_context->change_count.change_count++;
        }
        break;

    default:
        change_count_updated = false; /* Unexpected source, no need to proceed further */
        break;
    }

    if (change_count_updated)
    {
        if (ctrl_context->features.cprsnt_legacy_mode)
        {
            /* CPRSNT# cable present operation */
            const mtb_stc_ubm_dfc_t* dfc;
            bool drive_installed = false;

            for (uint32_t dfc_index = 0U; dfc_index < ctrl_context->dfc_count; dfc_index++)
            {
                dfc = &ubm_context->dfc[ctrl_context->dfc_list[dfc_index]];

                if (DEVICE_TYPE_EMPTY != dfc->drive_type_installed)
                {
                    drive_installed = true;
                    break;
                }
            }

            if (drive_installed)
            {
                cyhal_gpio_write(ubm_context->hfc[ctrl_context->hfc_index].hfc_io.change_detect, false);
            }
            else
            {
                cyhal_gpio_write(ubm_context->hfc[ctrl_context->hfc_index].hfc_io.change_detect, true);
            }
        }
        else
        {
            /* CHANGE_DETECT# interrupt operation */
            if ((state != MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY) &&
                (ctrl_context->change_count.change_count != change_count_before) &&
                ubm_context->capabilities.change_detect_interrupt)
            {
                cyhal_gpio_write(ubm_context->hfc[ctrl_context->hfc_index].hfc_io.change_detect, false);
            }
        }
    }
}


/*******************************************************************************
* Function Name: handle_operational_state
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_OPERATIONAL_STATE command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_operational_state(const mtb_stc_ubm_context_t* ubm_context,
                                                    mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_op_state_t state;
    mtb_en_ubm_lc_sts_t status;

    if (hfc_context->i2c.read_request)
    {
        state = mtb_ubm_get_op_state(ubm_context);

        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = (uint8_t)state;
        hfc_context->i2c.read_data_length = OP_STATE_CMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request is not allowed */
        status = MTB_UBM_LC_STS_NO_ACCESS_ALLOWED;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_last_command_status
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_LAST_COMMAND_STATUS command.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_last_command_status(mtb_stc_ubm_hfc_t* hfc_context,
                                                      const mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t current_status;
    mtb_en_ubm_lc_sts_t previously_status;

    if (hfc_context->i2c.read_request)
    {
        previously_status = get_last_command_status(ctrl_context);

        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = (uint8_t)previously_status;
        hfc_context->i2c.read_data_length = LC_STATUS_CMD_RSP_LEN;

        current_status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request not allowed */
        current_status = MTB_UBM_LC_STS_NO_ACCESS_ALLOWED;
    }

    return current_status;
}


/*******************************************************************************
* Function Name: handle_silicon_identity_and_version
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_SILICON_IDENTITY_AND_VERSION command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_silicon_identity_and_version(const mtb_stc_ubm_context_t* ubm_context,
                                                               mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = MTB_UBM_SPEC_VER;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] =
            (uint8_t)(ubm_context->silicon_identity.pcie_vendor_id & 0x00FFU);
        hfc_context->i2c.read_buffer[DATA_BYTE_2_INDEX] =
            (uint8_t)((ubm_context->silicon_identity.pcie_vendor_id & 0xFF00U) >> 8U);
        hfc_context->i2c.read_buffer[DATA_BYTE_3_INDEX] = 0U;
        hfc_context->i2c.read_buffer[DATA_BYTE_4_INDEX] =
            (uint8_t)(ubm_context->silicon_identity.device_code & 0xFFU);
        hfc_context->i2c.read_buffer[DATA_BYTE_5_INDEX] =
            (uint8_t)((ubm_context->silicon_identity.device_code >> 8U) & 0xFFU);
        hfc_context->i2c.read_buffer[DATA_BYTE_6_INDEX] =
            (uint8_t)((ubm_context->silicon_identity.device_code >> 16U) & 0xFFU);
        hfc_context->i2c.read_buffer[DATA_BYTE_7_INDEX] =
            (uint8_t)((ubm_context->silicon_identity.device_code >> 24U) & 0xFFU);
        hfc_context->i2c.read_buffer[DATA_BYTE_8_INDEX] = 0U;
        hfc_context->i2c.read_buffer[DATA_BYTE_9_INDEX] = 0U;
        hfc_context->i2c.read_buffer[DATA_BYTE_10_INDEX] = ubm_context->silicon_identity.fw_version_minor;
        hfc_context->i2c.read_buffer[DATA_BYTE_11_INDEX] = ubm_context->silicon_identity.fw_version_major;
        hfc_context->i2c.read_buffer[DATA_BYTE_12_INDEX] =
            (uint8_t)(ubm_context->silicon_identity.vendor_specific & 0xFFU);
        hfc_context->i2c.read_buffer[DATA_BYTE_13_INDEX] =
            (uint8_t)((ubm_context->silicon_identity.vendor_specific & 0xFF00U) >> 8U);

        hfc_context->i2c.read_data_length = SI_IDENTITY_AND_VERSION_CMD_RSP_LEN;
        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request not allowed */
        status = MTB_UBM_LC_STS_NO_ACCESS_ALLOWED;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_programmable_update_mode_capabilities
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_PROGRAMMABLE_UPDATE_MODE_CAPABILITIES command.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_programmable_update_mode_capabilities(mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = MTB_UBM_UPDATE_MODE_CAPABILITIES;
        hfc_context->i2c.read_data_length = UPDATE_MODE_CAP_CMD_RSP_LEN;
        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request not allowed */
        status = MTB_UBM_LC_STS_NO_ACCESS_ALLOWED;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_host_facing_connector_info
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_HOST_FACING_CONNECTOR_INFO command.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_host_facing_connector_info(mtb_stc_ubm_hfc_t* hfc_context,
                                                             const mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] =
            (ctrl_context->hfc_index << HFC_INFO_IDENTITY_Pos) & HFC_INFO_IDENTITY_Msk;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            ((uint8_t)ctrl_context->port_type << HFC_INFO_PORT_TYPE_Pos) & HFC_INFO_PORT_TYPE_Msk;
        hfc_context->i2c.read_data_length = HFC_INFO_CMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request is not allowed */
        status = MTB_UBM_LC_STS_NO_ACCESS_ALLOWED;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_backplane_info
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_BACKPLANE_INFO command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_backplane_info(const mtb_stc_ubm_context_t* ubm_context,
                                                 mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] =
            (ubm_context->backplane_info.backplane_number << BACKPLANE_INFO_NUMBER_Pos) & BACKPLANE_INFO_NUMBER_Msk;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (ubm_context->backplane_info.backplane_type << BACKPLANE_INFO_TYPE_Pos) & BACKPLANE_INFO_TYPE_Msk;
        hfc_context->i2c.read_data_length = BACKPLANE_INFO_CMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request is not allowed */
        status = MTB_UBM_LC_STS_NO_ACCESS_ALLOWED;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_starting_slot
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_STARTING_SLOT command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_starting_slot(const mtb_stc_ubm_context_t* ubm_context,
                                                mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = ubm_context->starting_slot;
        hfc_context->i2c.read_data_length = STARTING_SLOT_CMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request indicates that more bytes are received than the Read-only command expects */
        status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_capabilities
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_CAPABILITIES command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_capabilities(const mtb_stc_ubm_context_t* ubm_context,
                                               mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] =
            (uint8_t)ubm_context->capabilities.clock_routing << CAP_CLOCK_ROUTING_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (uint8_t)ubm_context->capabilities.slot_power_control << CAP_SLOT_POWER_CONTROL_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (uint8_t)ubm_context->capabilities.pcie_reset_control << CAP_PCIE_RESET_CONTROL_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (uint8_t)ubm_context->capabilities.dual_port << CAP_DUAL_PORT_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (ubm_context->capabilities.i2c_reset_operation << CAP_2WIRE_RESET_OP_Pos) & CAP_2WIRE_RESET_OP_Msk;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (uint8_t)ubm_context->capabilities.change_detect_interrupt << CAP_CHANGE_DET_INT_OP_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (uint8_t)ubm_context->capabilities.dfc_change_count_supported << CAP_CHANGE_COUNT_Pos;

        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] =
            (uint8_t)ubm_context->capabilities.prsnt_reported << CAP_PRSNT_REPORT_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] |=
            (uint8_t)ubm_context->capabilities.ifdet_reported << CAP_IFDET_REPORT_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] |=
            (uint8_t)ubm_context->capabilities.ifdet2_reported << CAP_IFDET2_REPORT_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] |=
            (uint8_t)ubm_context->capabilities.perst_override_supported << CAP_PERST_OVERRIDE_SUPPORT_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] |=
            (uint8_t)ubm_context->capabilities.smb_reset_supported << CAP_SMBUS_RESET_SUPPORT_Pos;

        hfc_context->i2c.read_data_length = CAPABILITIES_CMD_RSP_LEN;
        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request not allowed */
        status = MTB_UBM_LC_STS_NO_ACCESS_ALLOWED;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_features
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_FEATURES command.
*
* \warning
*  The Features write command is not implemented.
*
* \param ubm_context
*  The pointer to the UMB context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_features(const mtb_stc_ubm_context_t* ubm_context,
                                           mtb_stc_ubm_hfc_t* hfc_context,
                                           mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] =
            (uint8_t)ctrl_context->features.read_checksum_creation << FEATURES_READ_CHECKSUM_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (uint8_t)ctrl_context->features.write_checksum_checking << FEATURES_WRITE_CHECKSUM_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (uint8_t)ctrl_context->features.cprsnt_legacy_mode << FEATURES_CPRSNT_MODE_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (uint8_t)ctrl_context->features.pcie_reset_change_count_mask << FEATURES_PCIE_RESET_CC_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (uint8_t)ctrl_context->features.drive_type_installed_change_count_mask << FEATURES_DRIVE_TYPE_CC_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (uint8_t)ctrl_context->features.operational_state_change_count_mask << FEATURES_OP_STATE_CC_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
            (ctrl_context->features.perst_management_override << FEATURES_PERST_OVRD_Pos) & FEATURES_PERST_OVRD_Msk;

        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] =
            (uint8_t)ctrl_context->features.smbus_reset_control << FEATURES_SMBUS_RESET_Pos;

        hfc_context->i2c.read_data_length = FEATURES_CMD_RSP_LEN;
        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        bool cprsnt_legacy_mode_before = ctrl_context->features.cprsnt_legacy_mode;
        uint8_t received_byte = hfc_context->i2c.write_buffer[DATA_BYTE_1_INDEX];

        ctrl_context->features.read_checksum_creation =
            (0U != (received_byte & FEATURES_READ_CHECKSUM_Msk));
        ctrl_context->features.write_checksum_checking =
            (0U != (received_byte & FEATURES_WRITE_CHECKSUM_Msk));
        ctrl_context->features.cprsnt_legacy_mode =
            (0U != (received_byte & FEATURES_CPRSNT_MODE_Msk));
        ctrl_context->features.pcie_reset_change_count_mask =
            (0U != (received_byte & FEATURES_PCIE_RESET_CC_Msk));
        ctrl_context->features.drive_type_installed_change_count_mask =
            (0U != (received_byte & FEATURES_DRIVE_TYPE_CC_Msk));
        ctrl_context->features.operational_state_change_count_mask =
            (0U != (received_byte & FEATURES_OP_STATE_CC_Msk));
        ctrl_context->features.perst_management_override =
            ((received_byte & FEATURES_PERST_OVRD_Msk) >> FEATURES_PERST_OVRD_Pos);

        received_byte = hfc_context->i2c.write_buffer[DATA_BYTE_2_INDEX];

        ctrl_context->features.smbus_reset_control =
            (0U != (received_byte & FEATURES_SMBUS_RESET_Msk));

        if (ctrl_context->features.cprsnt_legacy_mode != cprsnt_legacy_mode_before)
        {
            mtb_ubm_update_change_count(ubm_context, ctrl_context, NULL, MTB_UBM_CC_SOURCE_CPRSNT_LEGACY_MODE);
        }

        status = MTB_UBM_LC_STS_SUCCESS;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_change_count
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_CHANGE_COUNT command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_change_count(const mtb_stc_ubm_context_t* ubm_context,
                                               mtb_stc_ubm_hfc_t* hfc_context,
                                               mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (hfc_context->i2c.read_request)
    {
        /* Byte 0 */
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = ctrl_context->change_count.change_count;
        /* Byte 1 */
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] =
            (uint8_t)ctrl_context->change_count.source_cprsnt_legacy_mode_change << CC_CMD_CPRSNT_MODE_CHANGE_SRC_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] |=
            (uint8_t)ctrl_context->change_count.source_pcie_reset_change << CC_CMD_PCIE_RESET_CHANGE_SRC_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] |=
            (uint8_t)ctrl_context->change_count.source_drive_type_installed_change << CC_CMD_DRIVE_TYPE_CHANGE_SRC_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] |=
            (uint8_t)ctrl_context->change_count.source_operational_state_change << CC_CMD_OP_STATE_CHANGE_SRC_Pos;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] |=
            (uint8_t)ctrl_context->change_count.source_controller_reset_change << CC_CMD_CTRL_RESET_CHANGE_SRC_Pos;
        hfc_context->i2c.read_data_length = CC_CMD_LEN;
        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        uint8_t received_change_count =  hfc_context->i2c.write_buffer[DATA_BYTE_1_INDEX];

        if (received_change_count == ctrl_context->change_count.change_count)
        {
            ctrl_context->change_count.source_cprsnt_legacy_mode_change = false;
            ctrl_context->change_count.source_pcie_reset_change = false;
            ctrl_context->change_count.source_drive_type_installed_change = false;
            ctrl_context->change_count.source_operational_state_change = false;
            ctrl_context->change_count.source_controller_reset_change = false;

            if (ubm_context->capabilities.change_detect_interrupt &&
                !ctrl_context->features.cprsnt_legacy_mode)
            {
                cyhal_gpio_write(hfc_context->hfc_io.change_detect, true);
            }

            status = MTB_UBM_LC_STS_SUCCESS;
        }
        else
        {
            status = MTB_UBM_LC_STS_CHANGE_COUNT_DOES_NOT_MATCH;
        }
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_dfc_status_and_control_descriptor_index
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR_INDEX command.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_dfc_status_and_control_descriptor_index(mtb_stc_ubm_hfc_t* hfc_context,
                                                                          mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = ctrl_context->status_n_control_descriptor_index;
        hfc_context->i2c.read_data_length = SCDI_CMD_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        uint8_t received_index = hfc_context->i2c.write_buffer[DATA_BYTE_1_INDEX];
        bool index_valid = false;

        for (uint32_t dfc_index = 0U; dfc_index < ctrl_context->dfc_count; dfc_index++)
        {
            if (ctrl_context->dfc_list[dfc_index] == received_index)
            {
                index_valid = true;
                break;
            }
        }

        if (index_valid)
        {
            ctrl_context->status_n_control_descriptor_index = received_index;
            status = MTB_UBM_LC_STS_SUCCESS;
        }
        else
        {
            ctrl_context->status_n_control_descriptor_index = SCD_INVALID_INDEX;
            status = MTB_UBM_LC_STS_INVALID_DESCRIPTOR_INDEX;
        }
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_dfc_status_and_control_descriptor
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_dfc_status_and_control_descriptor(mtb_stc_ubm_context_t* ubm_context,
                                                                    mtb_stc_ubm_hfc_t* hfc_context,
                                                                    mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    uint8_t index = ctrl_context->status_n_control_descriptor_index;

    if (SCD_INVALID_INDEX != index)
    {
        if (hfc_context->i2c.read_request)
        {
            /* Byte 0 */
            hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] =
                (*ctrl_context->scd[index].drive_type << SCD_CMD_DRIVE_TYPE_Pos) & SCD_CMD_DRIVE_TYPE_Msk;
            hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
                (uint8_t)ctrl_context->scd[index].bifurcate_port << SCD_CMD_BIFURCATE_PORT_Pos;
            hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] |=
                (ctrl_context->scd[index].pcie_reset << SCD_CMD_PCIE_RESET_Pos) & SCD_CMD_PCIE_RESET_Msk;
            /* Bytes 1-4 */
            (void)memcpy(&hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX],
                         ctrl_context->scd[index].ses_status,
                         MTB_UBM_SES_ARRAY_DEVICE_SLOT_ELEMENT_SIZE);
            /* Byte 5 */
            hfc_context->i2c.read_buffer[DATA_BYTE_5_INDEX] = ctrl_context->scd[index].dfc_change_count;
            /* Byte 6 */
            hfc_context->i2c.read_buffer[DATA_BYTE_6_INDEX] = ctrl_context->scd[index].vendor_specific_byte_0;
            /* Byte 7 */
            hfc_context->i2c.read_buffer[DATA_BYTE_7_INDEX] = ctrl_context->scd[index].vendor_specific_byte_1;
            hfc_context->i2c.read_data_length = SCD_CMD_RSP_LEN;
            status = MTB_UBM_LC_STS_SUCCESS;
        }
        else
        {
            uint8_t pcie_reset_before_write = ctrl_context->scd[index].pcie_reset;
            bool device_off_before_write = ctrl_context->scd[index].device_off;
            /* Byte 0 */
            uint8_t byte = hfc_context->i2c.write_buffer[DATA_BYTE_1_INDEX];
            ctrl_context->scd[index].pcie_reset = (byte & SCD_CMD_PCIE_RESET_Msk) >> SCD_CMD_PCIE_RESET_Pos;
            /* Bytes 1-4 */
            bool select = (0U != (hfc_context->i2c.write_buffer[DATA_BYTE_2_INDEX] & SCD_CMD_SAS_SELECT_Msk));
            if (select)
            {
                (void)memcpy(ctrl_context->scd[index].ses_control,
                             &hfc_context->i2c.write_buffer[DATA_BYTE_2_INDEX],
                             MTB_UBM_SES_ARRAY_DEVICE_SLOT_ELEMENT_SIZE);
                ctrl_context->scd[index].device_off =
                    (0U != (ctrl_context->scd[index].ses_control[SCD_SES_BYTE3] & SCD_CMD_SAS_DEVICE_OFF_Msk));
            }
            /* Byte 6 */
            ctrl_context->scd[index].vendor_specific_byte_0 = hfc_context->i2c.write_buffer[DATA_BYTE_7_INDEX];
            /* Byte 7 */
            ctrl_context->scd[index].vendor_specific_byte_1 = hfc_context->i2c.write_buffer[DATA_BYTE_8_INDEX];

            #if (MTB_UBM_SES_CB_ACTIVE)
            /* Rise SES control application event */
            if (select)
            {
                mtb_ubm_ifc_ses_app_event(ubm_context->ses_control_cb,
                                          index,
                                          ctrl_context->scd[index].ses_control,
                                          ctrl_context->scd[index].ses_status);
            }
            #endif /* MTB_UBM_SES_CB_ACTIVE */

            /* Process the PCIe Reset Request */
            if (ubm_context->capabilities.pcie_reset_control &&
                (ctrl_context->scd[index].pcie_reset != pcie_reset_before_write))
            {
                status = mtb_ubm_process_pcie_reset_request(ubm_context,
                                                            hfc_context,
                                                            ctrl_context,
                                                            &ubm_context->dfc[index]);
            }
            else
            {
                status = MTB_UBM_LC_STS_SUCCESS;
            }

            if (select && (ctrl_context->scd[index].device_off != device_off_before_write))
            {
                /* Process Device Off */
                if (ubm_context->capabilities.slot_power_control)
                {
                    process_pwrdis_signal(ubm_context,
                                          &ubm_context->dfc[index],
                                          ctrl_context->scd[index].device_off);
                }
                ctrl_context->scd[index].ses_status[SCD_SES_BYTE3] &= ~(uint8_t)SCD_CMD_SAS_DEVICE_OFF_Msk;
                ctrl_context->scd[index].ses_status[SCD_SES_BYTE3] |=
                    (uint8_t)ctrl_context->scd[index].device_off << SCD_CMD_SAS_DEVICE_OFF_Pos;
            }
        }
    }
    else
    {
        status = MTB_UBM_LC_STS_INVALID_DESCRIPTOR_INDEX;
    }

    return status;
}


/*******************************************************************************
* Function Name: get_checksum
****************************************************************************//**
*
*  Retrieves the UBM I2C protocol checksum byte from the received packet.
*  The checksum is the last byte of the write transaction.
*
* \param data_bytes
*  The pointer to the received packet.
*
* \param data_length
*  The length of the received packet.
*
* \return
*  The received checksum.
*
*******************************************************************************/
static uint8_t get_checksum(const uint8_t* data_bytes, uint32_t data_length)
{
    return data_bytes[data_length - 1U];
}


/*******************************************************************************
* Function Name: calculate_checksum
****************************************************************************//**
*
*  Caulculates the UBM I2C protocol checksum. The checksum is computed by: first, summing
*  an initial checksum seed value of 0xA5 and all of the specified bytes
*  as unsigned 8-bit binary numbers, and then, discarding any overflow bits.
*  The two's complement of this summation is used as the checksum value.
*
* \param address_byte
*  The address byte received from the master. 0 in the case of the slave response.
*
* \param data_bytes
*  The pointer to the packet received or to be sent.
*
* \param data_length
*  The length of the packet.
*
* \return
*  The calculated checksum.
*
*******************************************************************************/
static uint8_t calculate_checksum(uint8_t address_byte, const uint8_t* data_bytes,
                                  uint32_t data_length)
{
    uint8_t checksum = CHECKSUM_SEED_VALUE;

    for (uint32_t i = 0U; i < data_length; i++)
    {
        checksum += data_bytes[i];
    }

    checksum += address_byte;
    checksum = (~checksum) + 1U;

    return checksum;
}


/*******************************************************************************
* Function Name: validate_command_length
****************************************************************************//**
*
*  Checks if the received command packet length is expected for the command received.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t validate_command_length(const mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_ubm_cmd_t command = mtb_ubm_get_packet_command(hfc_context);

    if (hfc_context->i2c.read_request)
    {
        if ((MTB_UBM_CMD_OPERATIONAL_STATE == command) ||
            (MTB_UBM_CMD_LAST_COMMAND_STATUS == command) ||
            (MTB_UBM_CMD_SILICON_IDENTITY_AND_VERSION == command) ||
            (MTB_UBM_CMD_PROGRAMMABLE_UPDATE_MODE_CAPABILITIES == command) ||
            (MTB_UBM_CMD_HOST_FACING_CONNECTOR_INFO == command) ||
            (MTB_UBM_CMD_BACKPLANE_INFO == command) ||
            (MTB_UBM_CMD_STARTING_SLOT == command) ||
            (MTB_UBM_CMD_CAPABILITIES == command) ||
            (MTB_UBM_CMD_FEATURES == command) ||
            (MTB_UBM_CMD_CHANGE_COUNT == command) ||
            (MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR_INDEX == command) ||
            (MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR == command))
        {
            if (MTB_UBM_CMD_DEFAULT_LEN == hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_SUCCESS;
            }
            else if (MTB_UBM_CMD_DEFAULT_LEN < hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
            }
            else
            {
                status = MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
            }
        }
        #if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
        else if (MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER == command)
        {
            if (MTB_UBM_SUBCMD_DEFAULT_LEN == hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_SUCCESS;
            }
            else if (MTB_UBM_SUBCMD_DEFAULT_LEN < hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
            }
            else
            {
                status = MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
            }
        }
        else if ((MTB_UBM_CMD_ENTER_PROGRAMMABLE_UPDATE_MODE == command) ||
                 (MTB_UBM_CMD_EXIT_PROGRAMMABLE_UPDATE_MODE == command))
        {
            if (MTB_UBM_CMD_DEFAULT_LEN == hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_SUCCESS;
            }
            else if (MTB_UBM_CMD_DEFAULT_LEN < hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
            }
            else
            {
                status = MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
            }
        }
        #endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */
        else
        {
            status = MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
        }
    }
    else /* Write request */
    {
        if (MTB_UBM_CMD_CHANGE_COUNT == command)
        {
            if (CHANGE_COUNT_LEN == hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_SUCCESS;
            }
            else if (CHANGE_COUNT_LEN < hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
            }
            else
            {
                status = MTB_UBM_LC_STS_FAILED;
            }
        }
        else if (MTB_UBM_CMD_FEATURES == command)
        {
            if (FEATURES_CMD_WRITE_LEN == hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_SUCCESS;
            }
            else if (FEATURES_CMD_WRITE_LEN < hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
            }
            else
            {
                status = MTB_UBM_LC_STS_FAILED;
            }
        }
        else if (MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR_INDEX == command)
        {
            if (DFC_STATUS_AND_CONTROL_DESCRIPTOR_INDEX_LEN == hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_SUCCESS;
            }
            else if (DFC_STATUS_AND_CONTROL_DESCRIPTOR_INDEX_LEN < hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
            }
            else
            {
                status = MTB_UBM_LC_STS_FAILED;
            }
        }
        else if (MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR == command)
        {
            if (DFC_STATUS_AND_CONTROL_DESCRIPTOR_LEN == hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_SUCCESS;
            }
            else if (DFC_STATUS_AND_CONTROL_DESCRIPTOR_LEN < hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
            }
            else
            {
                status = MTB_UBM_LC_STS_FAILED;
            }
        }
        #if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
        else if (MTB_UBM_CMD_ENTER_PROGRAMMABLE_UPDATE_MODE == command)
        {
            if (ENTER_EXIT_PROGRAMMABLE_UPDATE_MODE_LEN == hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_SUCCESS;
            }
            else if (ENTER_EXIT_PROGRAMMABLE_UPDATE_MODE_LEN < hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
            }
            else
            {
                status = MTB_UBM_LC_STS_FAILED;
            }
        }
        else if (MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER == command)
        {
            if (((uint32_t)hfc_context->i2c.write_buffer[PS_DATA_LENGTH_Pos] + PS_NUMBER_SYSTEM_BYTE) ==
                hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_SUCCESS;
            }
            else if (((uint32_t)hfc_context->i2c.write_buffer[PS_DATA_LENGTH_Pos] + PS_NUMBER_SYSTEM_BYTE) <
                     hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
            }
            else
            {
                status = MTB_UBM_LC_STS_FAILED;
            }
        }
        else if (MTB_UBM_CMD_EXIT_PROGRAMMABLE_UPDATE_MODE == command)
        {
            if (ENTER_EXIT_PROGRAMMABLE_UPDATE_MODE_LEN == hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_SUCCESS;
            }
            else if (ENTER_EXIT_PROGRAMMABLE_UPDATE_MODE_LEN < hfc_context->i2c.write_data_length)
            {
                status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
            }
            else
            {
                status = MTB_UBM_LC_STS_FAILED;
            }
        }
        #endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */
        else if ((MTB_UBM_CMD_OPERATIONAL_STATE == command) ||
                 (MTB_UBM_CMD_LAST_COMMAND_STATUS == command) ||
                 (MTB_UBM_CMD_SILICON_IDENTITY_AND_VERSION == command) ||
                 (MTB_UBM_CMD_PROGRAMMABLE_UPDATE_MODE_CAPABILITIES == command) ||
                 (MTB_UBM_CMD_HOST_FACING_CONNECTOR_INFO == command) ||
                 (MTB_UBM_CMD_BACKPLANE_INFO == command) ||
                 (MTB_UBM_CMD_STARTING_SLOT == command) ||
                 (MTB_UBM_CMD_CAPABILITIES == command))
        {
            status = MTB_UBM_LC_STS_NO_ACCESS_ALLOWED;
        }
        else
        {
            status = MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
        }
    }

    return status;
}


/*******************************************************************************
* Function Name: get_last_command_status
****************************************************************************//**
*
*  Returns the status of the last UBM Controller command.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t get_last_command_status(const mtb_stc_ubm_controller_t* ctrl_context)
{
    return ctrl_context->last_command_status;
}


/*******************************************************************************
* Function Name: set_last_command_status
****************************************************************************//**
*
*  Saves the status of the last UBM Controller command.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \param status
*  The state of the last UBM Controller command to be saved.
*
*******************************************************************************/
static void set_last_command_status(mtb_stc_ubm_controller_t* ctrl_context,
                                    mtb_en_ubm_lc_sts_t status)
{
    ctrl_context->last_command_status = status;
}


/*******************************************************************************
* Function Name: mtb_ubm_process_pcie_reset_request
****************************************************************************//**
*
*  Processes PCIe reset request from HFC.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \param dfc_context
*  The pointer to the DFC context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
mtb_en_ubm_lc_sts_t mtb_ubm_process_pcie_reset_request(const mtb_stc_ubm_context_t* ubm_context,
                                                       const mtb_stc_ubm_hfc_t* hfc_context,
                                                       const mtb_stc_ubm_controller_t* ctrl_context,
                                                       mtb_stc_ubm_dfc_t* dfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    uint8_t new_pcie_reset_field = ctrl_context->scd[dfc_context->index].pcie_reset;
    mtb_en_ubm_port_domain_t domain = ctrl_context->domain;

    if (cyhal_gpio_read(hfc_context->hfc_io.perst) && (!dfc_context->dfc_perst_a_b[(uint8_t)domain].scd_last_change))
    {
        switch (new_pcie_reset_field)
        {
        case SCD_CMD_PCIE_RESET_NOP:
            status = MTB_UBM_LC_STS_SUCCESS;
            break;
        case SCD_CMD_PCIE_RESET_INIT:
            /* Perform reset sequence */
            if (dfc_context->dfc_perst_a_b[(uint8_t)domain].signal_released)
            {
                if (MTB_UBM_PORT_DOMAIN_PRIMARY == domain)
                {
                    cyhal_gpio_write(dfc_context->dfc_io.persta, false);
                }
                else
                {
                    cyhal_gpio_write(dfc_context->dfc_io.perstb, false);
                }
                if (ubm_context->capabilities.clock_routing)
                {
                    cyhal_gpio_write(dfc_context->dfc_io.refclken, MTB_UBM_SIGNAL_TO_DISABLE_REFCLK_MUX);
                }

                dfc_context->dfc_perst_a_b[(uint8_t)domain].signal_released = false;
            }

            if (ubm_context->capabilities.clock_routing)
            {
                /* Enable REFCLKEN */
                cyhal_gpio_write(dfc_context->dfc_io.refclken, MTB_UBM_SIGNAL_TO_ENABLE_REFCLK_MUX);

                /* Enable DFC PERST with a delay to stabilization RefClk (100 ms)*/
                dfc_context->dfc_perst_a_b[(uint8_t)domain].delay_val = MTB_UBM_PERST_LONG_DELAY;
            }
            else
            {
                /* Enable DFC PERST with a short delay (1 ms)*/
                dfc_context->dfc_perst_a_b[(uint8_t)domain].delay_val = MTB_UBM_PERST_SHORT_DELAY;
            }

            dfc_context->dfc_perst_a_b[(uint8_t)domain].scd_last_change = true;

            status = MTB_UBM_LC_STS_SUCCESS;
            break;
        case SCD_CMD_PCIE_RESET_HOLD:
            /* Hold PERST signal asserted */
            if (MTB_UBM_PORT_DOMAIN_PRIMARY == domain)
            {
                cyhal_gpio_write(dfc_context->dfc_io.persta, false);
            }
            else
            {
                cyhal_gpio_write(dfc_context->dfc_io.perstb, false);
            }
            status = MTB_UBM_LC_STS_SUCCESS;
            break;
        default:
            status = MTB_UBM_LC_STS_FAILED;
            break;
        }
    }
    return status;
}


/*******************************************************************************
* Function Name: process_pwrdis_signal
****************************************************************************//**
*
*  The handler of the PWRDIS signal.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param dfc_context
*  The pointer to the DFC context structure.
*
* \param new_device_off_state
*  The value of the PCIe reset field.
*
*******************************************************************************/
static void process_pwrdis_signal(mtb_stc_ubm_context_t* ubm_context,
                                  mtb_stc_ubm_dfc_t* dfc_context,
                                  bool new_device_off_state)
{
    /* Update the output of the PWRDIS signal */
    cyhal_gpio_write(dfc_context->dfc_io.pwrdis, new_device_off_state);

    if (new_device_off_state)
    {
        cyhal_gpio_write(dfc_context->dfc_io.persta, false);

        if (ubm_context->capabilities.dual_port)
        {
            cyhal_gpio_write(dfc_context->dfc_io.perstb, false);
        }

        if (ubm_context->capabilities.clock_routing)
        {
            cyhal_gpio_write(dfc_context->dfc_io.refclken, MTB_UBM_SIGNAL_TO_DISABLE_REFCLK_MUX);
        }

        dfc_context->dfc_perst_a_b[MTB_UBM_PORT_DOMAIN_PRIMARY].signal_released = false;
        if (ubm_context->capabilities.dual_port)
        {
            dfc_context->dfc_perst_a_b[MTB_UBM_PORT_DOMAIN_SECONDARY].signal_released = false;
        }
    }
    else
    {
        if (!ubm_context->capabilities.clock_routing)
        {
            mtb_stc_ubm_controller_t* ctrl_context = &ubm_context->ctrl[dfc_context->ctrl_list[0]];

            ctrl_context->scd[dfc_context->index].pcie_reset = MTB_UBM_PCIE_RESET_FIELD_INIT;
            (void)mtb_ubm_process_pcie_reset_request(ubm_context,
                                                     &ubm_context->hfc[ctrl_context->hfc_index],
                                                     ctrl_context,
                                                     dfc_context);
            if (ubm_context->capabilities.dual_port)
            {
                (void)mtb_ubm_process_pcie_reset_request(ubm_context,
                                                         &ubm_context->hfc[ctrl_context->hfc_index],
                                                         ctrl_context,
                                                         dfc_context);
            }
        }
    }
}


#if (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
/*******************************************************************************
* Function Name: handle_enter_programmable_update_mode
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_ENTER_PROGRAMMABLE_UPDATE_MODE command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_enter_programmable_update_mode(mtb_stc_ubm_context_t* ubm_context,
                                                                 mtb_stc_ubm_hfc_t* hfc_context,
                                                                 const mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (hfc_context->i2c.read_request)
    {
        /* Handle the read request */
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = ctrl_context->slave_address;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] = UPDATE_MODE_SPECIFIC_BYTE_0;
        hfc_context->i2c.read_buffer[DATA_BYTE_2_INDEX] = UPDATE_MODE_SPECIFIC_BYTE_1;
        hfc_context->i2c.read_buffer[DATA_BYTE_3_INDEX] = UPDATE_MODE_SPECIFIC_BYTE_2;
        hfc_context->i2c.read_data_length = ENTER_UPDATE_MODE_CMD_RSP_LEN;
        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Check the unlocking sequence to transfer to Update mode. */
        if ((UPDATE_MODE_SPECIFIC_BYTE_0 == hfc_context->i2c.write_buffer[DATA_BYTE_1_INDEX]) &&
            (UPDATE_MODE_SPECIFIC_BYTE_1 == hfc_context->i2c.write_buffer[DATA_BYTE_2_INDEX]) &&
            (UPDATE_MODE_SPECIFIC_BYTE_2 == hfc_context->i2c.write_buffer[DATA_BYTE_3_INDEX]) &&
            (0U != (hfc_context->i2c.write_buffer[DATA_BYTE_4_INDEX] & TRANSFER_TO_UPDATE_MODE_Msk)))
        {
            mtb_ubm_set_op_state(ubm_context, MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY);
            /* Reset satus downloading upgrade image */
            ubm_context->flash_layout.status_download_image = false;
            status = MTB_UBM_LC_STS_SUCCESS;
        }
    }
    return status;
}


/*******************************************************************************
* Function Name: handle_programmable_mode_data_transfer
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_programmable_mode_data_transfer(mtb_stc_ubm_context_t* ubm_context,
                                                                  mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_op_state_t state = mtb_ubm_get_op_state(ubm_context);

    if (MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY == state)
    {
        /* The command is valid only in the reduced functionality */
        mtb_ubm_pm_cmd_t subcommand = hfc_context->i2c.write_buffer[DATA_BYTE_1_INDEX];

        switch (subcommand)
        {
        case MTB_UBM_PM_CMD_GET_NON_VOLATILE_STORAGE_GEOMETRY:
            status = handle_get_non_volatile_storage_geometry_sub(hfc_context);
            break;
        case MTB_UBM_PM_CMD_ERASE:
            status = handle_erase_sub(ubm_context, hfc_context);
            break;
        case MTB_UBM_PM_CMD_ERASE_STATUS:
            status = handle_erase_status_sub(ubm_context, hfc_context);
            break;
        case MTB_UBM_PM_CMD_PROGRAM:
            status = handle_program_sub(ubm_context, hfc_context);
            break;
        case MTB_UBM_PM_CMD_PROGRAM_STATUS:
            status = handle_program_status_sub(ubm_context, hfc_context);
            break;
        case MTB_UBM_PM_CMD_VERIFY:
            status = handle_verify_sub(ubm_context, hfc_context);
            break;
        case MTB_UBM_PM_CMD_VERIFY_STATUS:
            status = handle_verify_status_sub(ubm_context, hfc_context);
            break;
        case MTB_UBM_PM_CMD_VERIFY_IMAGE:
            status = handle_verify_image_sub(ubm_context, hfc_context);
            break;
        case MTB_UBM_PM_CMD_VERIFY_IMAGE_STATUS:
            status = handle_verify_image_status_sub(ubm_context, hfc_context);
            break;
        case MTB_UBM_PM_CMD_SET_ACTIVE_IMAGE:
            status = handle_set_active_image_sub(ubm_context, hfc_context);
            break;
        case MTB_UBM_PM_CMD_ACTIVE_IMAGE_STATUS:
            status = handle_set_active_image_status_sub(ubm_context, hfc_context);
            break;
        default: 
            status = MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
            break;
        }
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_exit_programmable_update_mode
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_EXIT_PROGRAMMABLE_UPDATE_MODE command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_exit_programmable_update_mode(mtb_stc_ubm_context_t* ubm_context,
                                                                mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_op_state_t state;
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    state = mtb_ubm_get_op_state(ubm_context);

    /* Command is valid only in the reduced functionality */
    if (MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY == state)
    {
        if (hfc_context->i2c.read_request)
        {
            /* Handle the read request */
            hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = UPDATE_MODE_SPECIFIC_BYTE_0;
            hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] = UPDATE_MODE_SPECIFIC_BYTE_1;
            hfc_context->i2c.read_buffer[DATA_BYTE_2_INDEX] = UPDATE_MODE_SPECIFIC_BYTE_2;
            hfc_context->i2c.read_buffer[DATA_BYTE_3_INDEX] = OPERATIONAL_MODE_TRANSFER;
            hfc_context->i2c.read_data_length = EXIT_UPDATE_MODE_CMD_RSP_LEN;
            status = MTB_UBM_LC_STS_SUCCESS;
        }
        else
        {
            /* Check the unlocking sequence to transfer to Update mode. */
            if ((UPDATE_MODE_SPECIFIC_BYTE_0 == hfc_context->i2c.write_buffer[DATA_BYTE_1_INDEX]) &&
                (UPDATE_MODE_SPECIFIC_BYTE_1 == hfc_context->i2c.write_buffer[DATA_BYTE_2_INDEX]) &&
                (UPDATE_MODE_SPECIFIC_BYTE_2 == hfc_context->i2c.write_buffer[DATA_BYTE_3_INDEX]) &&
                (0U != (hfc_context->i2c.write_buffer[DATA_BYTE_4_INDEX] & TRANSFER_TO_UPDATE_MODE_Msk)))
            {
                mtb_ubm_set_op_state(ubm_context, MTB_UBM_OP_STATE_READY);
                status = MTB_UBM_LC_STS_SUCCESS;
            }
        }
    }
    return status;
}


/*******************************************************************************
* Function Name: handle_get_non_volatile_storage_geometry_sub
****************************************************************************//**
*
*  Returns the non-volatile structure and the size of the programmable segments.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_get_non_volatile_storage_geometry_sub(mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    /* The allocation of memory in mcuboot is manual. This subcommand returns a
       hardcode definition that describes the flash layout. */
    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = (uint8_t)MTB_UBM_PM_STS_SUCCESS;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] = GNVSG_DATA_LENGTH;
        hfc_context->i2c.read_buffer[DATA_BYTE_2_INDEX] = GNVSG_SECTORS_NUMBER; /* MCUboot has one upgrade section */

        /* The size of the upgrade area transmits as log2 (size_slot) */
        hfc_context->i2c.read_buffer[DATA_BYTE_3_INDEX] = log_base_2(MTB_UBM_UPGRADE_AREA_SIZE);

        /* The address needs an offset because row_num is over 8-bit.
           It needs to add a transition from the shifted line numbers to real
           physical line numbers. */
        hfc_context->i2c.read_buffer[DATA_BYTE_4_INDEX] = GNVSG_FIRST_SECTOR_INDEX;
        hfc_context->i2c.read_buffer[DATA_BYTE_5_INDEX] = ((MTB_UBM_UPGRADE_AREA_SIZE / CY_FLASH_SIZEOF_ROW) - 1U);
        hfc_context->i2c.read_data_length = GET_NVS_CMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_erase_sub
****************************************************************************//**
*
*  Erases a segment of the non-volatile location to prepare for programming.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_erase_sub(mtb_stc_ubm_context_t* ubm_context,
                                            const mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_pm_sts_t status_subcmd = MTB_UBM_PM_STS_INVALID;
    uint32_t addr_row = 0U;
    uint8_t data_length = 0U;

    if (!hfc_context->i2c.read_request)
    {
        data_length = hfc_context->i2c.write_buffer[DATA_BYTE_2_INDEX];

        if (data_length != (hfc_context->i2c.write_data_length - ES_NUMBER_SYSTEM_BYTE))
        {
            status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
        }
        else
        {
            /* Save the parameters for a status subcommand */
            ubm_context->flash_layout.image_number = hfc_context->i2c.write_buffer[DATA_BYTE_3_INDEX];
            ubm_context->flash_layout.sector_index = hfc_context->i2c.write_buffer[DATA_BYTE_4_INDEX];

            addr_row = get_address_from_row_flash(hfc_context->i2c.write_buffer[DATA_BYTE_4_INDEX],
                                                  MTB_UBM_UPGRADE_IMAGE_START_ADDRESS);

            if ((addr_row < MTB_UBM_UPGRADE_IMAGE_START_ADDRESS) || \
                (addr_row >= (MTB_UBM_UPGRADE_IMAGE_START_ADDRESS + MTB_UBM_UPGRADE_AREA_SIZE)))
            {
                status_subcmd = MTB_UBM_PM_STS_NON_VOLATILE_LOCATION_INVALID;
            }
            else
            {
                (void)cyhal_flash_erase(&ubm_context->flash_layout.flash_obj, addr_row);

                status_subcmd = MTB_UBM_PM_STS_SUCCESS;
                status = MTB_UBM_LC_STS_SUCCESS;
            }
        }
    }

    ubm_context->update_subcmd_status = status_subcmd;
    return status;
}


/*******************************************************************************
* Function Name: handle_erase_status_sub
****************************************************************************//**
*
*  Returns the status of an erase request.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_erase_status_sub(const mtb_stc_ubm_context_t* ubm_context,
                                                   mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = (uint8_t)ubm_context->update_subcmd_status;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] = ESS_DATA_LENGTH;
        hfc_context->i2c.read_buffer[DATA_BYTE_2_INDEX] = ubm_context->flash_layout.image_number;
        hfc_context->i2c.read_buffer[DATA_BYTE_3_INDEX] = ubm_context->flash_layout.sector_index;
        hfc_context->i2c.read_data_length = ERASE_STATUS_SUBCMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    return status;
}


/*******************************************************************************
* Function Name: handle_program_sub
****************************************************************************//**
*
*  Writes a segment of data into the nonvolatile location.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_program_sub(mtb_stc_ubm_context_t* ubm_context,
                                              const mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_pm_sts_t status_subcmd = MTB_UBM_PM_STS_INVALID;
    uint8_t data_length = 0U;
    uint32_t addr_row = 0U;

    if (!hfc_context->i2c.read_request)
    {
        data_length = hfc_context->i2c.write_buffer[DATA_BYTE_2_INDEX];

        if (data_length != (hfc_context->i2c.write_data_length - PS_NUMBER_SYSTEM_BYTE))
        {
            status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
        }
        else
        {
            data_length = hfc_context->i2c.write_buffer[DATA_BYTE_2_INDEX] - PS_NUMBER_UNFLASH_DATA;
            addr_row = get_address_from_row_flash(hfc_context->i2c.write_buffer[DATA_BYTE_4_INDEX],
                                                  MTB_UBM_UPGRADE_IMAGE_START_ADDRESS);

            ubm_context->flash_layout.app_sequence_number = hfc_context->i2c.write_buffer[DATA_BYTE_5_INDEX];

            if ((addr_row < MTB_UBM_UPGRADE_IMAGE_START_ADDRESS) ||
                (addr_row >= (MTB_UBM_UPGRADE_IMAGE_START_ADDRESS + MTB_UBM_UPGRADE_AREA_SIZE)))
            {
                status_subcmd = MTB_UBM_PM_STS_NON_VOLATILE_LOCATION_INVALID;
            }
            else
            {
                (void)memcpy(&ubm_context->flash_layout.row_buffer[ubm_context->flash_layout.offset_row_buffer],
                             &hfc_context->i2c.write_buffer[DATA_BYTE_6_INDEX],
                             data_length);

                ubm_context->flash_layout.offset_row_buffer += data_length;

                if (CY_FLASH_SIZEOF_ROW == ubm_context->flash_layout.offset_row_buffer)
                {
                    CY_MISRA_DEVIATE_LINE('MISRA C-2012 Rule 11.3', 'Checked manually, row_buffer should be 4 bytes aligned.');
                    const uint32_t* data = (const uint32_t*)ubm_context->flash_layout.row_buffer;

                    (void)cyhal_flash_write(&ubm_context->flash_layout.flash_obj, addr_row, data);
                    ubm_context->flash_layout.offset_row_buffer = 0U;
                }

                status_subcmd = MTB_UBM_PM_STS_SUCCESS;
                status = MTB_UBM_LC_STS_SUCCESS;
            }
        }
    }

    ubm_context->update_subcmd_status = status_subcmd;
    return status;
}


/*******************************************************************************
* Function Name: handle_program_status_sub
****************************************************************************//**
*
*  Returns the status of a program request.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_program_status_sub(const mtb_stc_ubm_context_t* ubm_context,
                                                     mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = (uint8_t)ubm_context->update_subcmd_status;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] = PSS_DATA_LENGTH;
        hfc_context->i2c.read_buffer[DATA_BYTE_2_INDEX] = ubm_context->flash_layout.app_sequence_number;
        hfc_context->i2c.read_data_length = PROGRAM_STATUS_SUBCMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    return status;
}


/*******************************************************************************
* Function Name: handle_verify_sub
****************************************************************************//**
*
*  Sets the Sector and Sector Index for a Verify Status request.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_verify_sub(mtb_stc_ubm_context_t* ubm_context,
                                             const mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_pm_sts_t status_subcmd = MTB_UBM_PM_STS_INVALID;
    uint32_t addr_row = 0U;
    uint8_t data_length = 0U;
    uint8_t row_buffer[CY_FLASH_SIZEOF_ROW] = { 0U };

    if (!hfc_context->i2c.read_request)
    {
        data_length = hfc_context->i2c.write_buffer[DATA_BYTE_2_INDEX];

        if (data_length != (hfc_context->i2c.write_data_length - VS_NUMBER_SYSTEM_BYTE))
        {
            status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
        }
        else
        {
            /* Save the parameters for a status subcommand */
            ubm_context->flash_layout.image_number = hfc_context->i2c.write_buffer[DATA_BYTE_3_INDEX];
            ubm_context->flash_layout.sector_index = hfc_context->i2c.write_buffer[DATA_BYTE_4_INDEX];

            addr_row = get_address_from_row_flash(hfc_context->i2c.write_buffer[DATA_BYTE_4_INDEX],
                                                  MTB_UBM_UPGRADE_IMAGE_START_ADDRESS);

            if ((addr_row < MTB_UBM_UPGRADE_IMAGE_START_ADDRESS) ||
                (addr_row >= (MTB_UBM_UPGRADE_IMAGE_START_ADDRESS + MTB_UBM_UPGRADE_AREA_SIZE)))
            {
                status_subcmd = MTB_UBM_PM_STS_NON_VOLATILE_LOCATION_INVALID;
            }
            else
            {
                (void)cyhal_flash_read(&ubm_context->flash_layout.flash_obj,
                                       addr_row, row_buffer, CY_FLASH_SIZEOF_ROW);

                ubm_context->flash_layout.checksum_sector_index = calculate_index_checksum(row_buffer,
                                                                                           CY_FLASH_SIZEOF_ROW);
                status_subcmd = MTB_UBM_PM_STS_SUCCESS;
                status = MTB_UBM_LC_STS_SUCCESS;
            }
        }
    }

    ubm_context->update_subcmd_status = status_subcmd;
    return status;
}


/*******************************************************************************
* Function Name: handle_verify_status_sub
****************************************************************************//**
*
*  Returns the checksum for the nonvolatile segment.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_verify_status_sub(const mtb_stc_ubm_context_t* ubm_context,
                                                    mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = (uint8_t)ubm_context->update_subcmd_status;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] = VSS_DATA_LENGTH;
        hfc_context->i2c.read_buffer[DATA_BYTE_2_INDEX] = ubm_context->flash_layout.image_number;
        hfc_context->i2c.read_buffer[DATA_BYTE_3_INDEX] = ubm_context->flash_layout.sector_index;
        hfc_context->i2c.read_buffer[DATA_BYTE_4_INDEX] = ubm_context->flash_layout.checksum_sector_index;
        hfc_context->i2c.read_data_length = VERIFY_STATUS_SUBCMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    return status;
}


/*******************************************************************************
* Function Name: handle_verify_image_sub
****************************************************************************//**
*
*  Sets the Image Number for an Image Number Status request.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_verify_image_sub(mtb_stc_ubm_context_t* ubm_context,
                                                   const mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_pm_sts_t status_subcmd = MTB_UBM_PM_STS_IMAGE_VERIFY_FAILED;
    uint32_t imageCrc = 0U;

    if (!hfc_context->i2c.read_request)
    {
        uint8_t data_length = hfc_context->i2c.write_buffer[DATA_BYTE_2_INDEX];

        if (data_length != (hfc_context->i2c.write_data_length - VI_NUMBER_SYSTEM_BYTE))
        {
            status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
        }
        else
        {
            ubm_context->flash_layout.image_number = hfc_context->i2c.write_buffer[DATA_BYTE_3_INDEX];

            imageCrc = *((uint32_t*)(MTB_UBM_UPGRADE_IMAGE_START_ADDRESS + UBM_UPGRADE_IMAGE_CRC_OFFSET));
            /* Reset the CRC area */
            (void)cyhal_flash_erase(&ubm_context->flash_layout.flash_obj,
                                    (MTB_UBM_UPGRADE_IMAGE_START_ADDRESS + UBM_UPGRADE_IMAGE_CRC_OFFSET));

            if (imageCrc == verify_image_checksum((uint8_t*)MTB_UBM_UPGRADE_IMAGE_START_ADDRESS,
                                                  MTB_UBM_UPGRADE_AREA_SIZE))
            {
                ubm_context->flash_layout.status_download_image = true;
                status_subcmd = MTB_UBM_PM_STS_SUCCESS;
                status = MTB_UBM_LC_STS_SUCCESS;
            }
        }
    }

    ubm_context->update_subcmd_status = status_subcmd;
    return status;
}


/*******************************************************************************
* Function Name: handle_verify_image_status_sub
****************************************************************************//**
*
*  Returns information indicating UBM Controller Image is valid.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_verify_image_status_sub(const mtb_stc_ubm_context_t* ubm_context,
                                                          mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = (uint8_t)ubm_context->update_subcmd_status;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] = VIS_DATA_LENGTH;
        hfc_context->i2c.read_buffer[DATA_BYTE_2_INDEX] = ubm_context->flash_layout.image_number;
        hfc_context->i2c.read_data_length = VERIFY_IMAGE_STATUS_SUBCMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    return status;
}


/*******************************************************************************
* Function Name: handle_set_active_image_sub
****************************************************************************//**
*
*  If multiple UBM Controller Images are supported, this command is used
*  to set the next image to use.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_set_active_image_sub(mtb_stc_ubm_context_t* ubm_context,
                                                       const mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_pm_sts_t status_subcmd = MTB_UBM_PM_STS_INVALID;
    uint8_t image_number = 0U;
    uint8_t data_length = 0U;

    if (!hfc_context->i2c.read_request)
    {
        data_length = hfc_context->i2c.write_buffer[DATA_BYTE_2_INDEX];

        if (data_length < (hfc_context->i2c.write_data_length - SI_SUBCMD_OVERHEAD))
        {
            status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
        }
        else
        {
            image_number = hfc_context->i2c.write_buffer[DATA_BYTE_3_INDEX];
            /* Save the image number for a status subcommand */
            ubm_context->flash_layout.image_number = image_number;

            /* mcuboot as the bootloader uses one upgrade area. */
            if (image_number > 0U)
            {
                status_subcmd = MTB_UBM_PM_STS_IMAGE_VERIFY_FAILED;
            }
            else
            {
                status_subcmd = MTB_UBM_PM_STS_SUCCESS;
                status = MTB_UBM_LC_STS_SUCCESS;
            }
        }
    }

    ubm_context->update_subcmd_status = status_subcmd;
    return status;
}


/*******************************************************************************
* Function Name: handle_set_active_image_status_sub
****************************************************************************//**
*
*  Returns the status of a set active image request.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_set_active_image_status_sub(const mtb_stc_ubm_context_t* ubm_context,
                                                              mtb_stc_ubm_hfc_t* hfc_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (hfc_context->i2c.read_request)
    {
        hfc_context->i2c.read_buffer[DATA_BYTE_0_INDEX] = (uint8_t)ubm_context->update_subcmd_status;
        hfc_context->i2c.read_buffer[DATA_BYTE_1_INDEX] = SiS_DATA_LENGTH;
        hfc_context->i2c.read_buffer[DATA_BYTE_2_INDEX] = ubm_context->flash_layout.image_number;
        hfc_context->i2c.read_data_length = ACTIVE_IMAGE_STATUS_SUBCMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    return status;
}


/*******************************************************************************
* Function Name: log_base_2
****************************************************************************//**
*
*  Calculates the logarithm based on two.
*
* \param argument
*  The argument of the logarithm function based on two.
*
* \return log_value
*  The value of the logarithm function.
*
*******************************************************************************/
static uint8_t log_base_2(uint32_t argument)
{
    uint8_t log_value = 0U;
    uint32_t value = argument;

    while (0U != value)
    {
        log_value++;
        value >>= 1U;
    }
    return log_value - 1U;
}


/*******************************************************************************
* Function Name: get_address_from_row_flash
****************************************************************************//**
*
*  Calculates the address flash based on the row number.
*
* \param row_num
*  The row number.
*
* \param start_addr
*  The address of the start area.
*
* \return
*  The address of the start row.
*
*******************************************************************************/
static uint32_t get_address_from_row_flash(uint8_t row_num, uint32_t start_addr)
{
    uint32_t row_addr = start_addr + ((uint32_t)row_num * CY_FLASH_SIZEOF_ROW);
    return row_addr;
}


/*******************************************************************************
* Function Name: calculate_index_checksum
****************************************************************************//**
*
*  Caulculates the flash row checksum.
*
* \param data_bytes
*  The pointer to the packet received or to be sent.
*
* \param data_length
*  The length of the packet.
*
* \return
*  The calculated checksum.
*
*******************************************************************************/
static uint8_t calculate_index_checksum(const uint8_t* data_bytes, uint32_t data_length)
{
    uint32_t checksum = 0U;

    for (uint32_t i = 0U; i < data_length; i++)
    {
        checksum += data_bytes[i];
    }

    checksum = (~checksum) + 1U;

    return (uint8_t)checksum;
}


/*******************************************************************************
* Function Name: verify_image_checksum
****************************************************************************//**
*
*  Verifies the image checksum.
*
* \param image_start
*  The pointer to the packet received or to be sent.
*
* \param image_length
*  The length of the packet.
*
* \return
*  The calculated checksum.
*
*******************************************************************************/
static uint32_t verify_image_checksum(const uint8_t* image_start, uint32_t image_length)
{
    uint32_t crc = VI_CRC_INIT_VALUE;
    static const uint32_t crcTable[VI_CRC_TABLE_SIZE] =
    {
        0x00000000U, 0x105ec76fU, 0x20bd8edeU, 0x30e349b1U,
        0x417b1dbcU, 0x5125dad3U, 0x61c69362U, 0x7198540dU,
        0x82f63b78U, 0x92a8fc17U, 0xa24bb5a6U, 0xb21572c9U,
        0xc38d26c4U, 0xd3d3e1abU, 0xe330a81aU, 0xf36e6f75U,
    };
    const uint8_t* image = image_start;
    uint32_t length = image_length;

    if (length != 0U)
    {
        do
        {
            crc = crc ^ *image;
            crc = (crc >> 4U) ^ crcTable[crc & 0xFU];
            crc = (crc >> 4U) ^ crcTable[crc & 0xFU];
            --length;
            ++image;
        } while (length != 0U);
    }
    return (~crc);
}


/*******************************************************************************
* Function Name: switch_to_bootloader
****************************************************************************//**
*
*  This function transfers the control from the current application to the bootloader
*  application. The function performs switching via a software reset.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
*******************************************************************************/
static void switch_to_bootloader(const mtb_stc_ubm_context_t* ubm_context)
{
    if (ubm_context->flash_layout.status_download_image)
    {
        __NVIC_SystemReset();
    }
}


#endif /* (MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED) */
