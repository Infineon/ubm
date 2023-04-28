/***************************************************************************//**
 * \file mtb_ubm_controller.c
 * \version 0.5
 *
 * \brief
 * Provides the UBM controller common API implementation.
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
#include <string.h>

/* Response data length for operational state command */
#define OP_STATE_CMD_RSP_LEN                (1U)
/* Response data length for Last Command Status command */
#define LC_STATUS_CMD_RSP_LEN               (1U)
/* Response data length for an update mode capabilities command */
#define UPDATE_MODE_CAP_CMD_RSP_LEN         (1U)
/* Responce data length for UBM Starting Slot Command */
#define STARTING_SLOT_CMD_RSP_LEN           (1U)
/* Responce data length for UBM Backplane Info Command */
#define BACKPLANE_INFO_CMD_RSP_LEN          (1U)
/* Responce data length for features Command */
#define FEATURES_CMD_RSP_LEN                (2U)
/* Responce data length for capabilities Command */
#define CAPABILITIES_CMD_RSP_LEN            (2U)

#if(MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
/* Response data length for Enter Programmable Update Mode commands */
#define ENTER_UPDATE_MODE_CMD_RSP_LEN       (4U)
/* Response data length for Exit Programmable Update Mode commands */
#define EXIT_UPDATE_MODE_CMD_RSP_LEN        (4U)
/* Response data length for Erase Status Subcommands */
#define ERASE_STATUS_SUBCMD_RSP_LEN         (4U)
/* Response data length for Program Status Subcommands */
#define PROGRAM_STATUS_SUBCMD_RSP_LEN       (3U)
/* Response data length for Verify Status Subcommands */
#define VERIFY_STATUS_SUBCMD_RSP_LEN        (5U)
/* Response data length for Active Image Status Subcommands */
#define ACTIVE_IMAGE_STATUS_SUBCMD_RSP_LEN  (3U)
/* Response data length for Verify Image Status Subcommands */
#define VERIFY_IMAGE_STATUS_SUBCMD_RSP_LEN  (3U)
/* The specific sequence to unlock/lock Programmable Update Mode */
#define UPDATE_MODE_SPECIFIC_BYTE_0         (0x55U)
#define UPDATE_MODE_SPECIFIC_BYTE_1         (0x42U)
#define UPDATE_MODE_SPECIFIC_BYTE_2         (0x4DU)
/* Status of transfer to update mode */
#define TRANSFER_TO_UPDATE_MODE_Msk         (0x01U)
/* The structure of the UBM Command Enter/Exit Programmable Update Mode write request. */
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
/* The position subcommand byte into UBM packet */
#define SUBCOMMAND_BYTE_Pos                 (1U)
/* The structure of the UBM Program Subcommand. */
#define PS_DATA_LENGHT_Pos                  (2U)
#define PS_SECTOR_INDEX_Pos                 (4U)
#define PS_APP_SEQUANCE_NUMBER_Pos          (5U)
#define PS_FIRST_DATA_BYTE_Pos              (6U)
#define PS_NUMBER_SYSTEM_BYTE               (4U)
#define PS_NUMBER_UNFLASH_DATA              (3U)
/* The structure of the UBM Program Status Subcommand. */
#define PSS_SUBCOMMAND_STATUS_Pos           (0U)
#define PSS_DATA_LENGHT_Pos                 (1U)
#define PSS_APP_SEQUANCE_NUMBER_Pos         (2U)
#define PSS_DATA_LENGHT                     (1U)
/* The structure of the UBM Erase Subcommand. */
#define ES_DATA_LENGHT_Pos                  (2U)
#define ES_SECTOR_NUMBER_Pos                (3U)
#define ES_SECTOR_INDEX_Pos                 (4U)
#define ES_NUMBER_SYSTEM_BYTE               (4U)
/* The structure of the UBM Erase Status Subcommand. */
#define ESS_SUBCOMMAND_STATUS_Pos           (0U)
#define ESS_DATA_LENGHT_Pos                 (1U)
#define ESS_SECTOR_NUMBER_Pos               (2U)
#define ESS_SECTOR_INDEX_Pos                (3U)
#define ESS_DATA_LENGHT                     (2U)
/* The structure of the UBM Verify Subcommand */
#define VS_DATA_LENGHT_Pos                  (2U)
#define VS_SECTOR_NUMBER_Pos                (3U)
#define VS_SECTOR_INDEX_Pos                 (4U)
#define VS_NUMBER_SYSTEM_BYTE               (4U)
/* The structure of the UBM Verify Status Subcommand */
#define VSS_SUBCOMMAND_STATUS_Pos           (0U)
#define VSS_DATA_LENGHT_Pos                 (1U)
#define VSS_SECTOR_NUMBER_Pos               (2U)
#define VSS_SECTOR_INDEX_Pos                (3U)
#define VSS_CHECKSUM_SECTOR_INDEX_Pos       (4U)
#define VSS_DATA_LENGHT                     (3U)
/* The structure of the UBM Set Active Image Subcommand. */
#define SI_DATA_LENGHT_Pos                  (2U)
#define SI_IMAGE_NUMBER_Pos                 (3U)
/* The structure of the UBM Set Active Image Status Subcommand. */
#define SIS_SUBCOMMAND_STATUS_Pos           (0U)
#define SIS_DATA_LENGHT_Pos                 (1U)
#define SIS_IMAGE_NUMBER_Pos                (2U)
#define SiS_DATA_LENGHT                     (1U)
/* The structure of the UBM Verify Image Subcommand. */
#define VI_DATA_LENGHT_Pos                  (2U)
#define VI_IMAGE_NUMBER_Pos                 (3U)
#define VI_NUMBER_SYSTEM_BYTE               (4U)
/* The structure of the UBM Verify Image Status Subcommand. */
#define VIS_SUBCOMMAND_STATUS_Pos           (0U)
#define VIS_DATA_LENGHT_Pos                 (1U)
#define VIS_IMAGE_NUMBER_Pos                (2U)
#define VIS_DATA_LENGHT                     (1U)
/* Initial value for image checksum */
#define CRC_INIT  0xFFFFFFFFUL
#endif /*MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED*/

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

#define MTB_UBM_CTR_FIRST_BYTE_MASK         (0x00FFU)
#define MTB_UBM_CTR_SECOND_BYTE_MASK        (0xFF00U)
#define MTB_UBM_CTR_FIRST_BYTE_SHIFT        (0U)
#define MTB_UBM_CTR_SECOND_BYTE_SHIFT       (8U)

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
/* The structure of the UBM DFC Status and Control Descriptor Index Command */
#define SCDI_CMD_INDEX_Pos                  (0U)
#define SCDI_CMD_LEN                        (1U)
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
#define SCD_CMD_SAS_ELEM_STS_CODE_Pos       (0U)
#define SCD_CMD_SAS_ELEM_STS_CODE_Msk       (0x0FU)
#define SCD_CMD_SAS_RESET_SWAP_Pos          (4U)
#define SCD_CMD_SAS_RESET_SWAP_Msk          (0x10U)
#define SCD_CMD_SAS_DISABLE_Pos             (5U)
#define SCD_CMD_SAS_DISABLE_Msk             (0x20U)
#define SCD_CMD_SAS_PRDFAIL_Pos             (6U)
#define SCD_CMD_SAS_PRDFAIL_Msk             (0x40U)
#define SCD_CMD_SAS_SELECT_Pos              (7U)
#define SCD_CMD_SAS_SELECT_Msk              (0x80U)
#define SCD_CMD_SAS_REBUILD_REMAP_ABORT_Pos (0U)
#define SCD_CMD_SAS_REBUILD_REMAP_ABORT_Msk (0x01U)
#define SCD_CMD_SAS_REBUILD_REMAP_Pos       (1U)
#define SCD_CMD_SAS_REBUILD_REMAP_Msk       (0x02U)
#define SCD_CMD_SAS_IN_FAILED_ARRAY_Pos     (2U)
#define SCD_CMD_SAS_IN_FAILED_ARRAY_Msk     (0x04U)
#define SCD_CMD_SAS_IN_CRIT_ARRAY_Pos       (3U)
#define SCD_CMD_SAS_IN_CRIT_ARRAY_Msk       (0x08U)
#define SCD_CMD_SAS_CONS_CHECK_Pos          (4U)
#define SCD_CMD_SAS_CONS_CHECK_Msk          (0x10U)
#define SCD_CMD_SAS_HOT_SPARE_Pos           (5U)
#define SCD_CMD_SAS_HOT_SPARE_Msk           (0x20U)
#define SCD_CMD_SAS_RSVD_DEVICE_Pos         (6U)
#define SCD_CMD_SAS_RSVD_DEVICE_Msk         (0x40U)
#define SCD_CMD_SAS_OK_Pos                  (7U)
#define SCD_CMD_SAS_OK_Msk                  (0x80U)
#define SCD_CMD_SAS_REPORT_Pos              (0U)
#define SCD_CMD_SAS_REPORT_Msk              (0x01U)
#define SCD_CMD_SAS_IDENT_Pos               (1U)
#define SCD_CMD_SAS_IDENT_Msk               (0x02U)
#define SCD_CMD_SAS_REMOVE_Pos              (2U)
#define SCD_CMD_SAS_REMOVE_Msk              (0x04U)
#define SCD_CMD_SAS_INSERT_Pos              (3U)
#define SCD_CMD_SAS_INSERT_Msk              (0x08U)
#define SCD_CMD_SAS_MISSING_Pos             (4U)
#define SCD_CMD_SAS_MISSING_Msk             (0x10U)
#define SCD_CMD_SAS_ENCL_BYPASS_B_Pos       (4U)
#define SCD_CMD_SAS_ENCL_BYPASS_B_Msk       (0x10U)
#define SCD_CMD_SAS_ENCL_BYPASS_A_Pos       (5U)
#define SCD_CMD_SAS_ENCL_BYPASS_A_Msk       (0x20U)
#define SCD_CMD_SAS_DO_NOT_REMOVE_Pos       (6U)
#define SCD_CMD_SAS_DO_NOT_REMOVE_Msk       (0x40U)
#define SCD_CMD_SAS_ACTIVE_Pos              (7U)
#define SCD_CMD_SAS_ACTIVE_Msk              (0x80U)
#define SCD_CMD_SAS_APP_BYPASS_A_Pos        (7U)
#define SCD_CMD_SAS_APP_BYPASS_A_Msk        (0x80U)
#define SCD_CMD_SAS_DEV_BYPASS_B_Pos        (0U)
#define SCD_CMD_SAS_DEV_BYPASS_B_Msk        (0x01U)
#define SCD_CMD_SAS_DEV_BYPASS_A_Pos        (1U)
#define SCD_CMD_SAS_DEV_BYPASS_A_Msk        (0x02U)
#define SCD_CMD_SAS_BYPASS_B_Pos            (2U)
#define SCD_CMD_SAS_BYPASS_B_Msk            (0x04U)
#define SCD_CMD_SAS_BYPASS_A_Pos            (3U)
#define SCD_CMD_SAS_BYPASS_A_Msk            (0x08U)
#define SCD_CMD_SAS_DEVICE_OFF_Pos          (4U)
#define SCD_CMD_SAS_DEVICE_OFF_Msk          (0x10U)
#define SCD_CMD_SAS_FAULT_REQUEST_Pos       (5U)
#define SCD_CMD_SAS_FAULT_REQUEST_Msk       (0x20U)
#define SCD_CMD_SAS_FAULT_SENSED_Pos        (6U)
#define SCD_CMD_SAS_FAULT_SENSED_Msk        (0x40U)
#define SCD_CMD_SAS_APP_BYPASS_B_Pos        (7U)
#define SCD_CMD_SAS_APP_BYPASS_B_Msk        (0x80U)
#define SCD_CMD_DFC_CHANGE_COUNT_Pos        (0U)
#define SCD_CMD_DFC_CHANGE_COUNT_Msk        (0xFFU)


/*******************************************************************************
*            Internal API
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_operational_state(mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_last_command_status(mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_silicon_identity_and_version(mtb_stc_ubm_context_t* ubm_context,
                                                               mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_programmable_update_mode_capabilities(mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_host_facing_connector_info(mtb_stc_ubm_context_t* ubm_context,
                                                             mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_backplane_info(mtb_stc_ubm_context_t* ubm_context,
                                                 mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_starting_slot(mtb_stc_ubm_context_t* ubm_context,
                                                mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_capabilities(mtb_stc_ubm_context_t* ubm_context, 
                                               mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_features(mtb_stc_ubm_context_t* ubm_context, 
                                           mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_change_count(mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_dfc_status_and_control_descriptor_index(mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_dfc_status_and_control_descriptor(mtb_stc_ubm_context_t* ubm_context, 
                                                                    mtb_stc_ubm_controller_t* ctrl_context);
static uint8_t get_checksum(const uint8_t* data_bytes, uint32_t data_length);
static uint8_t calculate_checksum(uint8_t address_byte, const uint8_t* data_bytes, uint32_t data_length);
static mtb_en_ubm_lc_sts_t get_last_command_status(const mtb_stc_ubm_controller_t* ctrl_context);
static void set_last_command_status(mtb_stc_ubm_controller_t* ctrl_context, mtb_en_ubm_lc_sts_t status);

#if(MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
static mtb_en_ubm_lc_sts_t handle_enter_programmable_update_mode(mtb_stc_ubm_context_t* ubm_context,
                                                                 mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_programmable_mode_data_transfer(mtb_stc_ubm_context_t* ubm_context,
                                                                  mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_exit_programmable_update_mode(mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_get_non_volatile_storage_geometry_sub(mtb_stc_ubm_context_t* ubm_context, 
                                                                        mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_erase_sub(mtb_stc_ubm_context_t* ubm_context,
                                            mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_erase_status_sub(mtb_stc_ubm_context_t* ubm_context,
                                                   mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_program_sub(mtb_stc_ubm_context_t* ubm_context,
                                              mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_program_status_sub(mtb_stc_ubm_context_t* ubm_context,
                                                     mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_verify_sub(mtb_stc_ubm_context_t* ubm_context,
                                             mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_verify_status_sub(mtb_stc_ubm_context_t* ubm_context,
                                                    mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_verify_image_sub(mtb_stc_ubm_context_t* ubm_context,
                                                   mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_verify_image_status_sub(mtb_stc_ubm_context_t* ubm_context,
                                                          mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_set_active_image_sub(mtb_stc_ubm_context_t* ubm_context,
                                                       mtb_stc_ubm_controller_t* ctrl_context);
static mtb_en_ubm_lc_sts_t handle_set_active_image_status_sub(mtb_stc_ubm_context_t* ubm_context,
                                                              mtb_stc_ubm_controller_t* ctrl_context);
static uint8_t log_base_2(uint32_t argument);
static uint32_t get_address_from_row_flash(uint8_t row_num, uint32_t start_addr);
static uint8_t calculate_index_checksum(const uint8_t* data_bytes, uint32_t data_length);
static uint32_t verify_image_checksum(const uint8_t* image_start, uint32_t length);
static void switch_to_bootloader(mtb_stc_ubm_context_t* ubm_context);
#endif /* MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED */

/*******************************************************************************
* Function Name: mtb_ubm_get_packet_command
****************************************************************************//**
*
*  Retrieves UBM controller command byte from received packet.
*
* \param ctrl_context
*  The pointer to the UBM controller context.
*
* \return
*  Received command byte.
*
*******************************************************************************/
mtb_ubm_cmd_t mtb_ubm_get_packet_command(const mtb_stc_ubm_controller_t* ctrl_context)
{
    /* The command byte is the first byte of the received packet. */
    mtb_ubm_cmd_t command = ctrl_context->i2c.write_buffer[0];

    return command;
}


/*******************************************************************************
* Function Name: mtb_ubm_controller_handle_read_request
****************************************************************************//**
*
*  Handles requests from HFC master to UBM Controller.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
*******************************************************************************/
void mtb_ubm_controller_handle_request(mtb_stc_ubm_context_t* ubm_context,
                                       mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_ubm_cmd_t command;
    mtb_en_ubm_op_state_t state;
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_SUCCESS;

    state = mtb_ubm_get_op_state(ctrl_context);

    if (MTB_UBM_OP_STATE_BUSY == state)
    {
        /* Busy processing the last command request */
        status = MTB_UBM_LC_STS_BUSY;
    }

    if (MTB_UBM_LC_STS_SUCCESS == status)
    {
        uint8_t received_checksum;
        uint8_t calculated_checksum;

        received_checksum = get_checksum(ctrl_context->i2c.write_buffer,
                                         ctrl_context->i2c.write_data_length);
        calculated_checksum = calculate_checksum(ctrl_context->i2c.received_address << 1,
                                                 ctrl_context->i2c.write_buffer,
                                                 ctrl_context->i2c.write_data_length - 1);

        if (received_checksum != calculated_checksum)
        {
            /* Checksum is invalid */
            status = MTB_UBM_LC_STS_INVALID_CHECKSUM;
        }
    }

    if (MTB_UBM_LC_STS_SUCCESS == status)
    {
        command = mtb_ubm_get_packet_command(ctrl_context);

        /* Handle command */
        switch (command)
        {
        case MTB_UBM_CMD_OPERATIONAL_STATE:
            status = handle_operational_state(ctrl_context);
            break;
        case MTB_UBM_CMD_LAST_COMMAND_STATUS:
            status = handle_last_command_status(ctrl_context);
            break;
        case MTB_UBM_CMD_SILICON_IDENTITY_AND_VERSION:
            status = handle_silicon_identity_and_version(ubm_context, ctrl_context);
            break;
        case MTB_UBM_CMD_PROGRAMMABLE_UPDATE_MODE_CAPABILITIES:
            status = handle_programmable_update_mode_capabilities(ctrl_context);
            break;
        #if(MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
        case MTB_UBM_CMD_ENTER_PROGRAMMABLE_UPDATE_MODE:
            status = handle_enter_programmable_update_mode(ubm_context, ctrl_context);
            break;
        case MTB_UBM_CMD_PROGRAMMABLE_MODE_DATA_TRANSFER:
            status = handle_programmable_mode_data_transfer(ubm_context, ctrl_context);
            break;
        case MTB_UBM_CMD_EXIT_PROGRAMMABLE_UPDATE_MODE:
            status = handle_exit_programmable_update_mode(ctrl_context);
            switch_to_bootloader(ubm_context);
            break;
        #endif /*MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED*/
        case MTB_UBM_CMD_HOST_FACING_CONNECTOR_INFO:
            status = handle_host_facing_connector_info(ubm_context, ctrl_context);
            break;
        case MTB_UBM_CMD_BACKPLANE_INFO:
            status = handle_backplane_info(ubm_context, ctrl_context);
            break;
        case MTB_UBM_CMD_STARTING_SLOT:
            status = handle_starting_slot(ubm_context, ctrl_context);
            break;
        case MTB_UBM_CMD_CAPABILITIES:
            status = handle_capabilities(ubm_context, ctrl_context);
            break;
        case MTB_UBM_CMD_FEATURES:
            status = handle_features(ubm_context, ctrl_context);
            break;
        case MTB_UBM_CMD_CHANGE_COUNT:
            status = handle_change_count(ctrl_context);
            break;
        case MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR_INDEX:
            status = handle_dfc_status_and_control_descriptor_index(ctrl_context);
            break;
        case MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR:
            status = handle_dfc_status_and_control_descriptor(ubm_context, ctrl_context);
            break;
        default:
            status = MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
            break;
        }
    }

    if ((MTB_UBM_LC_STS_SUCCESS == status) && ctrl_context->i2c.read_request)
    {
        /* Calculate checksum */
        if (MTB_UBM_I2C_READ_BUFFER_SIZE > ctrl_context->i2c.read_data_length)
        {
            ctrl_context->i2c.read_buffer[ctrl_context->i2c.read_data_length] =
                calculate_checksum(0, ctrl_context->i2c.read_buffer, ctrl_context->i2c.read_data_length);

            ++ctrl_context->i2c.read_data_length;
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
*  Returns current operational state of the UBM Controller.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_op_state_t.
*
*******************************************************************************/
mtb_en_ubm_op_state_t mtb_ubm_get_op_state(const mtb_stc_ubm_controller_t* ctrl_context)
{
    return ctrl_context->state;
}


/*******************************************************************************
* Function Name: mtb_ubm_set_op_state
****************************************************************************//**
*
*  Saves the current operational state of the UBM Controller.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \param state
*  The current state of the UBM Controller to be saved.
*
*******************************************************************************/
void mtb_ubm_set_op_state(mtb_stc_ubm_controller_t* ctrl_context, mtb_en_ubm_op_state_t state)
{
    ctrl_context->state = state;
}


/*******************************************************************************
* Function Name: handle_operational_state
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_OPERATIONAL_STATE command.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_operational_state(mtb_stc_ubm_controller_t* ctrl_context)
{    
    mtb_en_ubm_op_state_t state;
    mtb_en_ubm_lc_sts_t status;
    
    if (ctrl_context->i2c.read_request)
    {
        state = mtb_ubm_get_op_state(ctrl_context);

        ctrl_context->i2c.read_buffer[0] = (uint8_t)state;
        ctrl_context->i2c.read_data_length = OP_STATE_CMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request indicates that more bytes received than read only command expects */
        status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_last_command_status
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_LAST_COMMAND_STATUS command.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_last_command_status(mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t current_status;
    mtb_en_ubm_lc_sts_t previously_status;

    if (ctrl_context->i2c.read_request)
    {
        previously_status = get_last_command_status(ctrl_context);

        ctrl_context->i2c.read_buffer[0] = (uint8_t)previously_status;
        ctrl_context->i2c.read_data_length = LC_STATUS_CMD_RSP_LEN;

        current_status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request indicates that more bytes received than read only command expects */
        current_status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_silicon_identity_and_version(mtb_stc_ubm_context_t* ubm_context,
                                                               mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status;
    
    if (ctrl_context->i2c.read_request)
    {
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE0_POS] = MTB_UBM_SPEC_VER;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE1_POS] = ubm_context->silicon_identity.pcie_vendor_id & 0x00FF;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE2_POS] = (ubm_context->silicon_identity.pcie_vendor_id & 0xFF00) >> 8;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE3_POS] = 0u;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE4_POS] = ubm_context->silicon_identity.device_code & 0xFF;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE5_POS] = (ubm_context->silicon_identity.device_code >> 8) & 0xFF;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE6_POS] = (ubm_context->silicon_identity.device_code >> 16) & 0xFF;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE7_POS] = (ubm_context->silicon_identity.device_code >> 24) & 0xFF;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE8_POS] = 0u;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE9_POS] = 0u;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE10_POS] = ubm_context->silicon_identity.fw_version_minor;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE11_POS] = ubm_context->silicon_identity.fw_version_major;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE12_POS] = ubm_context->silicon_identity.vendor_specific & 0xFF;
        ctrl_context->i2c.read_buffer[SI_IDENTITY_AND_VERSION_BYTE13_POS] = (ubm_context->silicon_identity.vendor_specific & 0xFF00) >> 8;

        ctrl_context->i2c.read_data_length = SI_IDENTITY_AND_VERSION_CMD_RSP_LEN;
        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request indicates that more bytes received than read only command expects */
        status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_programmable_update_mode_capabilities
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_PROGRAMMABLE_UPDATE_MODE_CAPABILITIES command.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_programmable_update_mode_capabilities(mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status;

    if (ctrl_context->i2c.read_request)
    {
        ctrl_context->i2c.read_buffer[0] = MTB_UBM_UPDATE_MODE_CAPABILITIES;
        ctrl_context->i2c.read_data_length = UPDATE_MODE_CAP_CMD_RSP_LEN;
        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request indicates that more bytes received than read only command expects */
        status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_host_facing_connector_info
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_HOST_FACING_CONNECTOR_INFO command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_host_facing_connector_info(mtb_stc_ubm_context_t* ubm_context,
                                                               mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (ctrl_context->i2c.read_request)
    {
        uint8_t port_type;
        uint8_t hfcIdentity;

        if ((mtb_ubm_fru_get_ri_port_type(ctrl_context->index, &port_type, ubm_context) == true) &&
        (mtb_ubm_fru_get_ri_hfc_identifier(ctrl_context->index, &hfcIdentity, ubm_context) == true))
        {
            ctrl_context->i2c.read_buffer[0u] = port_type << 7u | hfcIdentity;
            ctrl_context->i2c.read_data_length = STARTING_SLOT_CMD_RSP_LEN;

            status = MTB_UBM_LC_STS_SUCCESS;
        }
    }
    else
    {
        /* Write request indicates that more bytes received than read only command expects */
        status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_backplane_info(mtb_stc_ubm_context_t* ubm_context,
                                                mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status;

    if (ctrl_context->i2c.read_request)
    {
        ctrl_context->i2c.read_buffer[0u] = 
            (ubm_context->backplane_info.backplane_number << BACKPLANE_INFO_NUMBER_Pos) & BACKPLANE_INFO_NUMBER_Msk;
        ctrl_context->i2c.read_buffer[0u] |=
            (ubm_context->backplane_info.backplane_type << BACKPLANE_INFO_TYPE_Pos) & BACKPLANE_INFO_TYPE_Msk;
        ctrl_context->i2c.read_data_length = BACKPLANE_INFO_CMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request indicates that more bytes received than read only command expects */
        status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_starting_slot(mtb_stc_ubm_context_t* ubm_context,
                                                mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status;

    if (ctrl_context->i2c.read_request)
    {
        ctrl_context->i2c.read_buffer[0u] = ubm_context->starting_slot;
        ctrl_context->i2c.read_data_length = STARTING_SLOT_CMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request indicates that more bytes received than read only command expects */
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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_capabilities(mtb_stc_ubm_context_t* ubm_context, mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status;

    if (ctrl_context->i2c.read_request)
    {
        ctrl_context->i2c.read_buffer[0u] =
            ((uint8_t)ubm_context->capabilities.clock_routing << CAP_CLOCK_ROUTING_Pos) & CAP_CLOCK_ROUTING_Msk;
        ctrl_context->i2c.read_buffer[0u] |=
            ((uint8_t)ubm_context->capabilities.slot_power_control << CAP_SLOT_POWER_CONTROL_Pos) & CAP_SLOT_POWER_CONTROL_Msk;
        ctrl_context->i2c.read_buffer[0u] |=
            ((uint8_t)ubm_context->capabilities.pcie_reset_control << CAP_PCIE_RESET_CONTROL_Pos) & CAP_PCIE_RESET_CONTROL_Msk;
        ctrl_context->i2c.read_buffer[0u] |=
            ((uint8_t)ubm_context->capabilities.dual_port << CAP_DUAL_PORT_Pos) & CAP_DUAL_PORT_Msk;
        ctrl_context->i2c.read_buffer[0u] |=
            (ubm_context->capabilities.i2c_reset_operation << CAP_2WIRE_RESET_OP_Pos) & CAP_2WIRE_RESET_OP_Msk;
        ctrl_context->i2c.read_buffer[0u] |=
            ((uint8_t)ubm_context->capabilities.change_detect_interrupt << CAP_CHANGE_DET_INT_OP_Pos) & CAP_CHANGE_DET_INT_OP_Msk;
        ctrl_context->i2c.read_buffer[0u] |=
            ((uint8_t)ubm_context->capabilities.dfc_change_count << CAP_CHANGE_COUNT_Pos) & CAP_CHANGE_COUNT_Msk;

        ctrl_context->i2c.read_buffer[1u] =
            ((uint8_t)ubm_context->capabilities.prsnt_reported << CAP_PRSNT_REPORT_Pos) & CAP_PRSNT_REPORT_Msk;
        ctrl_context->i2c.read_buffer[1u] |=
            ((uint8_t)ubm_context->capabilities.ifdet_reported << CAP_IFDET_REPORT_Pos) & CAP_IFDET_REPORT_Msk;
        ctrl_context->i2c.read_buffer[1u] |=
            ((uint8_t)ubm_context->capabilities.ifdet2_reported << CAP_IFDET2_REPORT_Pos) & CAP_IFDET2_REPORT_Msk;
        ctrl_context->i2c.read_buffer[1u] |=
            ((uint8_t)ubm_context->capabilities.perst_override_supported << CAP_PERST_OVERRIDE_SUPPORT_Pos) & CAP_PERST_OVERRIDE_SUPPORT_Msk;
        ctrl_context->i2c.read_buffer[1u] |=
            ((uint8_t)ubm_context->capabilities.smb_reset_supported << CAP_SMBUS_RESET_SUPPORT_Pos) & CAP_SMBUS_RESET_SUPPORT_Msk;

        ctrl_context->i2c.read_data_length = CAPABILITIES_CMD_RSP_LEN;
        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Write request indicates that more bytes received than read only command expects */
        status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
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
*  Features write command is not implemented.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_features(mtb_stc_ubm_context_t* ubm_context, mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (ctrl_context->i2c.read_request)
    {
        uint16_t value = 0;

        if (mtb_ubm_fru_get_oa_data_controller_features(&value, ubm_context) == true)
        {
            ctrl_context->i2c.read_buffer[0u] = (value & MTB_UBM_CTR_FIRST_BYTE_MASK) >> MTB_UBM_CTR_FIRST_BYTE_SHIFT;
            ctrl_context->i2c.read_buffer[1u] = (value & MTB_UBM_CTR_SECOND_BYTE_MASK) >> MTB_UBM_CTR_SECOND_BYTE_SHIFT;
            ctrl_context->i2c.read_data_length = FEATURES_CMD_RSP_LEN;

            status = MTB_UBM_LC_STS_SUCCESS;
        }
    }
    else
    {
        /* Write request not implemented */
        status = MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_change_count
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_CHANGE_COUNT command.
*
* \warning
*  Change Count command is not implemented.
* 
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_change_count(mtb_stc_ubm_controller_t* ctrl_context)
{
    CY_UNUSED_PARAMETER(ctrl_context);
    return MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
}


/*******************************************************************************
* Function Name: handle_dfc_status_and_control_descriptor_index
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR_INDEX command.
*
* \warning
*  DFC Status and Control Descriptor Index command is not implemented.
* 
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_dfc_status_and_control_descriptor_index(mtb_stc_ubm_controller_t* ctrl_context)
{
    CY_UNUSED_PARAMETER(ctrl_context);
    return MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
}


/*******************************************************************************
* Function Name: handle_dfc_status_and_control_descriptor
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_DFC_STATUS_AND_CONTROL_DESCRIPTOR command.
*
* \warning
*  DFC Status and Control Descriptor command is not implemented.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_dfc_status_and_control_descriptor(mtb_stc_ubm_context_t* ubm_context,
                                                                    mtb_stc_ubm_controller_t* ctrl_context)
{
    CY_UNUSED_PARAMETER(ubm_context);
    CY_UNUSED_PARAMETER(ctrl_context);
    return MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
}


/*******************************************************************************
* Function Name: get_checksum
****************************************************************************//**
*
*  Retrieves UBM I2C protocol checksum byte from received packet.
*  The checksum is the last byte of the write transaction.
*
* \param data_bytes
*  The pointer to received packet.
*
* \param data_length
*  The length of the received packet.
*
* \return
*  Received checksum.
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
*  Caulculates UBM I2C protocol checksum. The checksum is computed by summing
*  an initial checksum seed value of 0xA5 and all of the specified bytes
*  as unsigned 8-bit binary numbers and discarding any overflow bits.
*  The two's complement of this summation is used as the checksum value.
*
* \param address_byte
*  The address byte received from master. 0 in case of the slave response.
*
* \param data_bytes
*  The pointer to the packet received or to be sent.
*
* \param data_length
*  The length of the packet.
*
* \return
*  Calculated checksum.
*
*******************************************************************************/
static uint8_t calculate_checksum(uint8_t address_byte, const uint8_t* data_bytes,
                                  uint32_t data_length)
{
    uint8_t checksum = 0xA5U;

    for (uint32_t i = 0U; i < data_length; ++i)
    {
        checksum += data_bytes[i];
    }

    checksum += address_byte;
    checksum = (~checksum) + 1U;

    return checksum;
}


/*******************************************************************************
* Function Name: get_last_command_status
****************************************************************************//**
*
*  Returns status of the last UBM Controller command.
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
static void set_last_command_status(mtb_stc_ubm_controller_t* ctrl_context, mtb_en_ubm_lc_sts_t status)
{
    ctrl_context->last_command_status = status;
}

#if(MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED)
/*******************************************************************************
* Function Name: handle_enter_programmable_update_mode
****************************************************************************//**
*
*  Handles the MTB_UBM_CMD_ENTER_PROGRAMMABLE_UPDATE_MODE command.
*
* \param ubm_context
*  The pointer to the UBM context structure.
* 
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_enter_programmable_update_mode(mtb_stc_ubm_context_t* ubm_context,
                                                                 mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (ctrl_context->i2c.read_request)
    {
        /* Handle read request */
        ctrl_context->i2c.read_buffer[SLAVE_ADR_TO_UPDATE_MODE_Pos] = ctrl_context->i2c.controller_address;
        ctrl_context->i2c.read_buffer[UNLOCK_BYTE_0_Pos] = UPDATE_MODE_SPECIFIC_BYTE_0;
        ctrl_context->i2c.read_buffer[UNLOCK_BYTE_1_Pos] = UPDATE_MODE_SPECIFIC_BYTE_1;
        ctrl_context->i2c.read_buffer[UNLOCK_BYTE_2_Pos] = UPDATE_MODE_SPECIFIC_BYTE_2;
        ctrl_context->i2c.read_data_length = ENTER_UPDATE_MODE_CMD_RSP_LEN;
        status = MTB_UBM_LC_STS_SUCCESS;
    }
    else
    {
        /* Check unlocking sequence to transfer to update mode. */
        if ((ctrl_context->i2c.write_buffer[UPDATE_MODE_SPECIFIC_BYTE_0_Pos] == UPDATE_MODE_SPECIFIC_BYTE_0) && \
            (ctrl_context->i2c.write_buffer[UPDATE_MODE_SPECIFIC_BYTE_1_Pos] == UPDATE_MODE_SPECIFIC_BYTE_1) && \
            (ctrl_context->i2c.write_buffer[UPDATE_MODE_SPECIFIC_BYTE_2_Pos] == UPDATE_MODE_SPECIFIC_BYTE_2))
        {
            if (ctrl_context->i2c.write_buffer[TRANSFER_UPDATE_MODE_Pos] & TRANSFER_TO_UPDATE_MODE_Msk)
            {
                mtb_ubm_set_op_state(ctrl_context, MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY);
                /* Reset satus downloading upgrade image */
                ubm_context->flash_layout.status_download_image = false;
                status = MTB_UBM_LC_STS_SUCCESS;
            }
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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_programmable_mode_data_transfer(mtb_stc_ubm_context_t* ubm_context, 
                                                                  mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_op_state_t state = mtb_ubm_get_op_state(ctrl_context);

    if (MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY == state)
    {
        /* Command is valid only in reduced functionality */
        status = MTB_UBM_LC_STS_NO_ACCESS_ALLOWED;
        mtb_ubm_pm_cmd_t subcommand = ctrl_context->i2c.write_buffer[SUBCOMMAND_BYTE_Pos];

        switch (subcommand)
        {
        case MTB_UBM_PM_CMD_GET_NON_VOLATILE_STORAGE_GEOMETRY:
            status = handle_get_non_volatile_storage_geometry_sub(ubm_context, ctrl_context);
            break;
        case MTB_UBM_PM_CMD_ERASE:
            status = handle_erase_sub(ubm_context, ctrl_context);
            break;
        case MTB_UBM_PM_CMD_ERASE_STATUS:
            status = handle_erase_status_sub(ubm_context, ctrl_context);
            break;
        case MTB_UBM_PM_CMD_PROGRAM:
            status = handle_program_sub(ubm_context, ctrl_context);
            break;
        case MTB_UBM_PM_CMD_PROGRAM_STATUS:
            status = handle_program_status_sub(ubm_context, ctrl_context);
            break;
        case MTB_UBM_PM_CMD_VERIFY:
            status = handle_verify_sub(ubm_context, ctrl_context);
            break;
        case MTB_UBM_PM_CMD_VERIFY_STATUS:
            status = handle_verify_status_sub(ubm_context, ctrl_context);
            break;
        case MTB_UBM_PM_CMD_VERIFY_IMAGE:
            status = handle_verify_image_sub(ubm_context, ctrl_context);
            break;
        case MTB_UBM_PM_CMD_VERIFY_IMAGE_STATUS:
            status = handle_verify_image_status_sub(ubm_context, ctrl_context);
            break;
        case MTB_UBM_PM_CMD_SET_ACTIVE_IMAGE:
            status = handle_set_active_image_sub(ubm_context, ctrl_context);
            break;
        case MTB_UBM_PM_CMD_ACTIVE_IMAGE_STATUS:
            status = handle_set_active_image_status_sub(ubm_context, ctrl_context);
            break;
        default: status = MTB_UBM_LC_STS_COMMAND_NOT_IMPLEMENTED;
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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_exit_programmable_update_mode(mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_op_state_t state;
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    state = mtb_ubm_get_op_state(ctrl_context);

    /* Command is valid only in reduced functionality */
    if (MTB_UBM_OP_STATE_REDUCED_FUNCTIONALITY == state)
    {
        if (ctrl_context->i2c.read_request)
        {
            /* Handle read request */
            ctrl_context->i2c.read_buffer[LOCK_BYTE_0_Pos] = UPDATE_MODE_SPECIFIC_BYTE_0;
            ctrl_context->i2c.read_buffer[LOCK_BYTE_1_Pos] = UPDATE_MODE_SPECIFIC_BYTE_1;
            ctrl_context->i2c.read_buffer[LOCK_BYTE_2_Pos] = UPDATE_MODE_SPECIFIC_BYTE_2;
            ctrl_context->i2c.read_buffer[OPERATIONAL_MODE_TRANSFER_Pos] = OPERATIONAL_MODE_TRANSFER;
            ctrl_context->i2c.read_data_length = EXIT_UPDATE_MODE_CMD_RSP_LEN;
            status = MTB_UBM_LC_STS_SUCCESS;
        }
        else
        {
            /* Check unlocking sequence to transfer to update mode. */
            if ((ctrl_context->i2c.write_buffer[UPDATE_MODE_SPECIFIC_BYTE_0_Pos] == UPDATE_MODE_SPECIFIC_BYTE_0) && \
                (ctrl_context->i2c.write_buffer[UPDATE_MODE_SPECIFIC_BYTE_1_Pos] == UPDATE_MODE_SPECIFIC_BYTE_1) && \
                (ctrl_context->i2c.write_buffer[UPDATE_MODE_SPECIFIC_BYTE_2_Pos] == UPDATE_MODE_SPECIFIC_BYTE_2))
            {
                if (ctrl_context->i2c.write_buffer[TRANSFER_UPDATE_MODE_Pos] & TRANSFER_TO_UPDATE_MODE_Msk)
                {
                    mtb_ubm_set_op_state(ctrl_context, MTB_UBM_OP_STATE_READY);
                    status = MTB_UBM_LC_STS_SUCCESS;
                }
            }
        }
    }
    return status;
}


/*******************************************************************************
* Function Name: handle_get_non_volatile_storage_geometry_sub
****************************************************************************//**
*
*  Returns the nonvolatile structure and size of the programmable segments.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_get_non_volatile_storage_geometry_sub(mtb_stc_ubm_context_t* ubm_context,
                                                                        mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    /* Allocation of memory in mcuboot is manual. This subcommand returns a
       hardcode definition that describes the flash layout. */
    if (ctrl_context->i2c.read_request)
    {
        ctrl_context->i2c.read_buffer[0] = MTB_UBM_PM_STS_SUCCESS;
        ctrl_context->i2c.read_buffer[1] = 0x04U;
        ctrl_context->i2c.read_buffer[2] = 0x01U; /* MCUboot has one upgrade section */

        /* The size of upgrade area transmit as log2 (size_slot) */
        ctrl_context->i2c.read_buffer[3] = log_base_2(ubm_context->flash_layout.size_upgrade_area);

        /* The address offset needs because row_num is over 8-bit. 
           It needs to add a transition from shifted line numbers to real
           physical ones. */
        ctrl_context->i2c.read_buffer[4] = 0x00U;
        ctrl_context->i2c.read_buffer[5] = ((ubm_context->flash_layout.size_upgrade_area \
                                             / CY_FLASH_SIZEOF_ROW) - 1);
        ctrl_context->i2c.read_data_length = 0x06U;

        status = MTB_UBM_LC_STS_SUCCESS;
    }

    return status;
}


/*******************************************************************************
* Function Name: handle_erase_sub
****************************************************************************//**
*
*  Erases a segment of the nonvolatile location to prepare for programming.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_erase_sub(mtb_stc_ubm_context_t* ubm_context,
                                            mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_pm_sts_t status_subcmd = MTB_UBM_PM_STS_INVALID;
    uint32_t addr_row = 0U;
    uint8_t data_length = 0U;

    if (!ctrl_context->i2c.read_request)
    {
        data_length = ctrl_context->i2c.write_buffer[ES_DATA_LENGHT_Pos];

        if (data_length != (ctrl_context->i2c.write_data_length - ES_NUMBER_SYSTEM_BYTE))
        {
            status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
        }
        else
        {
            /* Save parameters for status subcommand */
            ubm_context->flash_layout.image_number = ctrl_context->i2c.write_buffer[ES_SECTOR_NUMBER_Pos];
            ubm_context->flash_layout.sector_index = ctrl_context->i2c.write_buffer[ES_SECTOR_INDEX_Pos];

            addr_row = get_address_from_row_flash(ctrl_context->i2c.write_buffer[ES_SECTOR_INDEX_Pos], \
                                                ubm_context->flash_layout.addr_start_upgrade_area);

            if ((addr_row < ubm_context->flash_layout.addr_start_upgrade_area) || \
                (addr_row >= (ubm_context->flash_layout.addr_start_upgrade_area + \
                            ubm_context->flash_layout.size_upgrade_area)))
            {
                status_subcmd = MTB_UBM_PM_STS_NON_VOLATILE_LOCATION_INVALID;
            }
            else
            {
                cyhal_flash_erase(&ubm_context->flash_layout.flash_obj, addr_row);

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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_erase_status_sub(mtb_stc_ubm_context_t* ubm_context,
                                                   mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (ctrl_context->i2c.read_request)
    {
        ctrl_context->i2c.read_buffer[ESS_SUBCOMMAND_STATUS_Pos] = ubm_context->update_subcmd_status;
        ctrl_context->i2c.read_buffer[ESS_DATA_LENGHT_Pos] = ESS_DATA_LENGHT;
        ctrl_context->i2c.read_buffer[ESS_SECTOR_NUMBER_Pos] = ubm_context->flash_layout.image_number;
        ctrl_context->i2c.read_buffer[ESS_SECTOR_INDEX_Pos] = ubm_context->flash_layout.sector_index;
        ctrl_context->i2c.read_data_length = ERASE_STATUS_SUBCMD_RSP_LEN;

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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_program_sub(mtb_stc_ubm_context_t* ubm_context,
                                              mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_pm_sts_t status_subcmd = MTB_UBM_PM_STS_INVALID;
    uint8_t data_length = 0U;
    uint32_t addr_row = 0U;

    if (!ctrl_context->i2c.read_request)
    {
        data_length = ctrl_context->i2c.write_buffer[PS_DATA_LENGHT_Pos];

        if (data_length != (ctrl_context->i2c.write_data_length - PS_NUMBER_SYSTEM_BYTE))
        {
            status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
        }
        else
        {
            data_length = ctrl_context->i2c.write_buffer[PS_DATA_LENGHT_Pos] - PS_NUMBER_UNFLASH_DATA;
            addr_row = get_address_from_row_flash(ctrl_context->i2c.write_buffer[PS_SECTOR_INDEX_Pos], \
                                                ubm_context->flash_layout.addr_start_upgrade_area);

            ubm_context->flash_layout.app_sequence_number = ctrl_context->i2c.write_buffer[PS_APP_SEQUANCE_NUMBER_Pos];

            if ((addr_row < ubm_context->flash_layout.addr_start_upgrade_area) || \
                (addr_row >= (ubm_context->flash_layout.addr_start_upgrade_area + \
                            ubm_context->flash_layout.size_upgrade_area)))
            {
                status_subcmd = MTB_UBM_PM_STS_NON_VOLATILE_LOCATION_INVALID;
            }
            else
            {
                (void)memcpy((ubm_context->flash_layout.row_buffer + ubm_context->flash_layout.offset_row_buffer), \
                                                        &ctrl_context->i2c.write_buffer[PS_FIRST_DATA_BYTE_Pos], \
                                                        data_length);

                ubm_context->flash_layout.offset_row_buffer += data_length;

                if (ubm_context->flash_layout.offset_row_buffer == CY_FLASH_SIZEOF_ROW)
                {
                    cyhal_flash_write(&ubm_context->flash_layout.flash_obj, addr_row, \
                                                    (const uint32_t*)ubm_context->flash_layout.row_buffer);
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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_program_status_sub(mtb_stc_ubm_context_t* ubm_context,
                                                     mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (ctrl_context->i2c.read_request)
    {
        ctrl_context->i2c.read_buffer[PSS_SUBCOMMAND_STATUS_Pos] = ubm_context->update_subcmd_status;
        ctrl_context->i2c.read_buffer[PSS_DATA_LENGHT_Pos] = PSS_DATA_LENGHT;
        ctrl_context->i2c.read_buffer[PSS_APP_SEQUANCE_NUMBER_Pos] = ubm_context->flash_layout.app_sequence_number;
        ctrl_context->i2c.read_data_length = PROGRAM_STATUS_SUBCMD_RSP_LEN;

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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_verify_sub(mtb_stc_ubm_context_t* ubm_context,
                                             mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_pm_sts_t status_subcmd = MTB_UBM_PM_STS_INVALID;
    uint32_t addr_row = 0U;
    uint8_t data_length = 0U;
    uint8_t row_buffer[CY_FLASH_SIZEOF_ROW] = {0};

    if (!ctrl_context->i2c.read_request)
    {
        data_length = ctrl_context->i2c.write_buffer[VS_DATA_LENGHT_Pos];

        if (data_length != (ctrl_context->i2c.write_data_length - VS_NUMBER_SYSTEM_BYTE))
        {
            status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
        }
        else
        {
            /* Save parameters for status subcommand */
            ubm_context->flash_layout.image_number = ctrl_context->i2c.write_buffer[VS_SECTOR_NUMBER_Pos];
            ubm_context->flash_layout.sector_index = ctrl_context->i2c.write_buffer[VS_SECTOR_INDEX_Pos];

            addr_row = get_address_from_row_flash(ctrl_context->i2c.write_buffer[ES_SECTOR_INDEX_Pos], \
                                                ubm_context->flash_layout.addr_start_upgrade_area);

            if ((addr_row < ubm_context->flash_layout.addr_start_upgrade_area) || \
                (addr_row >= (ubm_context->flash_layout.addr_start_upgrade_area + \
                            ubm_context->flash_layout.size_upgrade_area)))
            {
                status_subcmd = MTB_UBM_PM_STS_NON_VOLATILE_LOCATION_INVALID;
            }
            else
            {
                cyhal_flash_read(&ubm_context->flash_layout.flash_obj, 
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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_verify_status_sub(mtb_stc_ubm_context_t* ubm_context,
                                                    mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (ctrl_context->i2c.read_request)
    {
        ctrl_context->i2c.read_buffer[VSS_SUBCOMMAND_STATUS_Pos] = ubm_context->update_subcmd_status;
        ctrl_context->i2c.read_buffer[VSS_DATA_LENGHT_Pos] = VSS_DATA_LENGHT;
        ctrl_context->i2c.read_buffer[VSS_SECTOR_NUMBER_Pos] = ubm_context->flash_layout.image_number;
        ctrl_context->i2c.read_buffer[VSS_SECTOR_INDEX_Pos] = ubm_context->flash_layout.sector_index;
        ctrl_context->i2c.read_buffer[VSS_CHECKSUM_SECTOR_INDEX_Pos] = ubm_context->flash_layout.checksum_sector_index;
        ctrl_context->i2c.read_data_length = VERIFY_STATUS_SUBCMD_RSP_LEN;

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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_verify_image_sub(mtb_stc_ubm_context_t* ubm_context,
                                                   mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_pm_sts_t status_subcmd = MTB_UBM_PM_STS_IMAGE_VERIFY_FAILED;
    uint32_t imageCrc = 0UL;

    if (!ctrl_context->i2c.read_request)
    {
        uint8_t data_length = ctrl_context->i2c.write_buffer[VI_DATA_LENGHT_Pos];

        if (data_length != (ctrl_context->i2c.write_data_length - VI_NUMBER_SYSTEM_BYTE))
        {
            status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
        }
        else
        {
            ubm_context->flash_layout.image_number = ctrl_context->i2c.write_buffer[VI_IMAGE_NUMBER_Pos];

            imageCrc = *((uint32_t*)(ubm_context->flash_layout.addr_start_upgrade_area + 0x200UL));
            /* Reset crc area */
            cyhal_flash_erase(&ubm_context->flash_layout.flash_obj, (ubm_context->flash_layout.addr_start_upgrade_area + 0x200UL));

            if (imageCrc == verify_image_checksum((uint8_t*)ubm_context->flash_layout.addr_start_upgrade_area,\
                                                  ubm_context->flash_layout.size_upgrade_area))
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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_verify_image_status_sub(mtb_stc_ubm_context_t* ubm_context,
                                                          mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (ctrl_context->i2c.read_request)
    {
        ctrl_context->i2c.read_buffer[VIS_SUBCOMMAND_STATUS_Pos] = ubm_context->update_subcmd_status;
        ctrl_context->i2c.read_buffer[VIS_DATA_LENGHT_Pos] = VIS_DATA_LENGHT;
        ctrl_context->i2c.read_buffer[VIS_IMAGE_NUMBER_Pos] = ubm_context->flash_layout.image_number;
        ctrl_context->i2c.read_data_length = VERIFY_IMAGE_STATUS_SUBCMD_RSP_LEN;

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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_set_active_image_sub(mtb_stc_ubm_context_t* ubm_context,
                                                       mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;
    mtb_en_ubm_pm_sts_t status_subcmd = MTB_UBM_PM_STS_INVALID;
    uint8_t image_number = 0U;
    uint8_t data_length = 0U;

    if (!ctrl_context->i2c.read_request)
    {
        data_length = ctrl_context->i2c.write_buffer[SI_DATA_LENGHT_Pos];

        if (data_length != (ctrl_context->i2c.write_data_length - 3U))
        {
            status = MTB_UBM_LC_STS_TOO_MANY_BYTES_WRITTEN;
        }
        else
        {
            image_number = ctrl_context->i2c.write_buffer[SI_IMAGE_NUMBER_Pos];
            /* Save image number for status subcommand */
            ubm_context->flash_layout.image_number = image_number;

            /* The mcuboot as bootloader use one upgrade area. */
            if (image_number > 0)
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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
* \return status
*  See mtb_en_ubm_lc_sts_t.
*
*******************************************************************************/
static mtb_en_ubm_lc_sts_t handle_set_active_image_status_sub(mtb_stc_ubm_context_t* ubm_context,
                                                              mtb_stc_ubm_controller_t* ctrl_context)
{
    mtb_en_ubm_lc_sts_t status = MTB_UBM_LC_STS_FAILED;

    if (ctrl_context->i2c.read_request)
    {
        ctrl_context->i2c.read_buffer[SIS_SUBCOMMAND_STATUS_Pos] = ubm_context->update_subcmd_status;
        ctrl_context->i2c.read_buffer[SIS_DATA_LENGHT_Pos] = ACTIVE_IMAGE_STATUS_SUBCMD_RSP_LEN;
        ctrl_context->i2c.read_buffer[SIS_IMAGE_NUMBER_Pos] = ubm_context->flash_layout.image_number;
        ctrl_context->i2c.read_data_length = ACTIVE_IMAGE_STATUS_SUBCMD_RSP_LEN;

        status = MTB_UBM_LC_STS_SUCCESS;
    }
    return status;
}


/*******************************************************************************
* Function Name: log_base_2
****************************************************************************//**
*
*  Calculate logarithm based on two.
*
* \param argument
*  Argument of the logarithm function based on two.
*
* \return log_value
*  Value of the logarithm function.
*
*******************************************************************************/
static uint8_t log_base_2(uint32_t argument)
{
    uint8_t log_value = 0U;

    while(argument)
    {
        log_value++;
        argument >>= 1;
    }
    return log_value - 1U;
}


/*******************************************************************************
* Function Name: get_address_from_row_flash
****************************************************************************//**
*
*  Calculate address flash based on the row number.
*
* \param row_num
*  Row number
*
* \param start_addr
*  Address of start area
*
* \return 
*  Address of start row
*
*******************************************************************************/
static uint32_t get_address_from_row_flash(uint8_t row_num, uint32_t start_addr)
{
    uint32_t row_addr = start_addr + (row_num * CY_FLASH_SIZEOF_ROW);
    return row_addr;
}


/*******************************************************************************
* Function Name: calculate_index_checksum
****************************************************************************//**
*
*  Caulculates flash row checksum.
*
* \param data_bytes
*  The pointer to the packet received or to be sent.
*
* \param data_length
*  The length of the packet.
*
* \return
*  Calculated checksum.
*
*******************************************************************************/
static uint8_t calculate_index_checksum(const uint8_t* data_bytes, uint32_t data_length)
{
    uint32_t checksum = 0U;

    for (uint32_t i = 0U; i < data_length; ++i)
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
*  Verifies image checksum.
*
* \param image_start
*  The pointer to the packet received or to be sent.
*
* \param length
*  The length of the packet.
*
* \return
*  Calculated checksum.
*
*******************************************************************************/
static uint32_t verify_image_checksum(const uint8_t* image_start, uint32_t length)
{
    uint32_t crc = CRC_INIT;
    static const uint32_t crcTable[0x16U] =
    {
        0x00000000U, 0x105ec76fU, 0x20bd8edeU, 0x30e349b1U,
        0x417b1dbcU, 0x5125dad3U, 0x61c69362U, 0x7198540dU,
        0x82f63b78U, 0x92a8fc17U, 0xa24bb5a6U, 0xb21572c9U,
        0xc38d26c4U, 0xd3d3e1abU, 0xe330a81aU, 0xf36e6f75U,
    };

    if (length != 0U)
    {
        do
        {
            crc = crc ^ *image_start;
            crc = (crc >> 4UL) ^ crcTable[crc & 0xFUL];
            crc = (crc >> 4UL) ^ crcTable[crc & 0xFUL];
            --length;
            ++image_start;
        } while (length != 0U);
    }
    return (~crc);
}

/*******************************************************************************
* Function Name: switch_to_bootloader
****************************************************************************//**
*
*  This function transfers control from the current application to bootloader
*  application. The function performs switching via software reset.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
*******************************************************************************/
static void switch_to_bootloader(mtb_stc_ubm_context_t* ubm_context)
{
    if (true == ubm_context->flash_layout.status_download_image)
    {
        __NVIC_SystemReset();
    }
}
#endif /*MTB_UBM_UPDATE_MODE_CAPABILITIES != MTB_UBM_UPDATE_NOT_SUPPORTED*/
