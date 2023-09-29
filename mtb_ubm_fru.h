/***************************************************************************//**
 * \file mtb_ubm_fru.h
 * \version 1.0
 *
 * \brief
 * Provides declarations for the UBM middleware FRU layout and configuration
 * structures.
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

#ifndef UBM_FRU_H
#define UBM_FRU_H

#include "mtb_ubm_types.h"
#include "cy_em_eeprom.h"

#ifdef __cplusplus
extern "C" {
#endif
/** \cond DO_NOT_DOCUMENT */

//------------------------------------------------------------------------------
/* Common header */
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_CH_ADDR                                  (0U)
#define MTB_UBM_FRU_CH_LEN                                   (8U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_CH_H0_ADDR                               (0U)
#define MTB_UBM_FRU_CH_H0_LEN                                (1U)

#define MTB_UBM_FORMAT_VERSION_Pos                           (0U)
#define MTB_UBM_FORMAT_VERSION_Msk                           (0x0FU)

#define MTB_UBM_FRU_CH_H0_RESERVED_Pos                       (4U)
#define MTB_UBM_FRU_CH_H0_RESERVED_Msk                       (0xF0U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_CH_H1_ADDR                               (1U)
#define MTB_UBM_FRU_CH_H1_LEN                                (1U)

#define MTB_UBM_FRU_CH_H1_INTENAL_OFFSET_Pos                 (0U)
#define MTB_UBM_FRU_CH_H1_INTENAL_OFFSET_Msk                 (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_CH_H2_ADDR                               (2U)
#define MTB_UBM_FRU_CH_H2_LEN                                (1U)

#define MTB_UBM_FRU_CH_H2_CHASIS_INFO_OFFSET_Pos             (0U)
#define MTB_UBM_FRU_CH_H2_CHASIS_INFO_OFFSET_Msk             (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_CH_H3_ADDR                               (3U)
#define MTB_UBM_FRU_CH_H3_LEN                                (1U)

#define MTB_UBM_FRU_CH_H3_BOARD_INFO_OFFSET_Pos              (0U)
#define MTB_UBM_FRU_CH_H3_BOARD_INFO_OFFSET_Msk              (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_CH_H4_ADDR                               (4U)
#define MTB_UBM_FRU_CH_H4_LEN                                (1U)

#define MTB_UBM_FRU_CH_H4_PRODUCT_INFO_OFFSET_Pos            (0U)
#define MTB_UBM_FRU_CH_H4_PRODUCT_INFO_OFFSET_Msk            (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_CH_H5_ADDR                               (5U)
#define MTB_UBM_FRU_CH_H5_LEN                                (1U)

#define MTB_UBM_FRU_CH_H5_MUTRECORD_OFFSET_Pos               (0U)
#define MTB_UBM_FRU_CH_H5_MUTRECORD_OFFSET_Msk               (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_CH_H6_ADDR                               (6U)
#define MTB_UBM_FRU_CH_H6_LEN                                (1U)

#define MTB_UBM_FRU_CH_H6_PAD_Pos                            (0U)
#define MTB_UBM_FRU_CH_H6_PAD_Msk                            (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_CH_H7_ADDR                               (7U)
#define MTB_UBM_FRU_CH_H7_LEN                                (1U)

#define MTB_UBM_FRU_CH_H7_CHECKSUM_Pos                       (0U)
#define MTB_UBM_FRU_CH_H7_CHECKSUM_Msk                       (0xFFU)
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/* Overview area header */
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_ADDR                                  (8U)
#define MTB_UBM_FRU_OA_LEN                                   (16U)

#define MTB_UBM_FRU_OA_H_LEN                                 (5U)
#define MTB_UBM_FRU_OA_CHEKSUM_LEN                           (1U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_H0_ADDR                               (0U)
#define MTB_UBM_FRU_OA_H0_LEN                                (1U)

#define MTB_UBM_FRU_OA_H0_REC_T_Pos                          (0U)
#define MTB_UBM_FRU_OA_H0_REC_T_Msk                          (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_H1_ADDR                               (1U)
#define MTB_UBM_FRU_OA_H1_LEN                                (1U)

#define MTB_UBM_FRU_OA_H1_FORM_V_Pos                         (0U)
#define MTB_UBM_FRU_OA_H1_FORM_V_Msk                         (0x7U)

#define MTB_UBM_FRU_OA_H1_EOLST_Pos                          (7U)
#define MTB_UBM_FRU_OA_H1_EOLST_Msk                          (0x80U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_H2_ADDR                               (2U)
#define MTB_UBM_FRU_OA_H2_LEN                                (1U)

#define MTB_UBM_FRU_OA_H2_REC_LEN_Pos                        (0U)
#define MTB_UBM_FRU_OA_H2_REC_LEN_Msk                        (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_H3_ADDR                               (3U)
#define MTB_UBM_FRU_OA_H3_LEN                                (1U)

#define MTB_UBM_FRU_OA_H3_REC_CHKS_Pos                       (0U)
#define MTB_UBM_FRU_OA_H3_REC_CHKS_Msk                       (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_H4_ADDR                               (4U)
#define MTB_UBM_FRU_OA_H4_LEN                                (1U)

#define MTB_UBM_FRU_OA_H4_HDR_CHKS_Pos                       (0U)
#define MTB_UBM_FRU_OA_H4_HDR_CHKS_Mks                       (0xFFU)
//------------------------------------------------------------------------------
/* Overview data */
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_D0_ADDR                               (5U)
#define MTB_UBM_FRU_OA_D0_LEN                                (1U)

#define MTB_UBM_FRU_OA_D0_SPEC_VER_Pos                       (0U)
#define MTB_UBM_FRU_OA_D0_SPEC_VER_Msk                       (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_D1_ADDR                               (6U)
#define MTB_UBM_FRU_OA_D1_LEN                                (1U)

#define MTB_UBM_FRU_OA_D1_2W_DEV_ARR_Pos                     (0U)
#define MTB_UBM_FRU_OA_D1_2W_DEV_ARR_Msk                     (0x03U)

#define MTB_UBM_FRU_OA_D1_2W_MUX_ADDR_Pos                    (2U)
#define MTB_UBM_FRU_OA_D1_2W_MUX_ADDR_Msk                    (0x1CU)

#define MTB_UBM_FRU_OA_D1_2W_MUX_BC_Pos                      (5U)
#define MTB_UBM_FRU_OA_D1_2W_MUX_BC_Msk                      (0xE0U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_D2_ADDR                               (7U)
#define MTB_UBM_FRU_OA_D2_LEN                                (1U)

#define MTB_UBM_FRU_OA_D2_FRU_INV_Pos                        (0U)
#define MTB_UBM_FRU_OA_D2_FRU_INV_Msk                        (0x01U)

#define MTB_UBM_FRU_OA_D2_C_MAX_T_Pos                        (1U)
#define MTB_UBM_FRU_OA_D2_C_MAX_T_Msk                        (0xFEU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_D3_4_ADDR                             (8U)
#define MTB_UBM_FRU_OA_D3_4_LEN                              (2U)

#define MTB_UBM_FRU_OA_D3_4_FEATURES_FIRST_BYTE_Pos          (0U)
#define MTB_UBM_FRU_OA_D3_4_FEATURES_FIRST_BYTE_Msk          (0x00FFU)

#define MTB_UBM_FRU_OA_D3_4_FEATURES_SECOND_BYTE_Pos         (8U)
#define MTB_UBM_FRU_OA_D3_4_FEATURES_SECOND_BYTE_Msk         (0x0100U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_D5_ADDR                               (10U)
#define MTB_UBM_FRU_OA_D5_LEN                                (1U)

#define MTB_UBM_FRU_OA_D5_STS_CNTRL_DESC_Pos                 (0U)
#define MTB_UBM_FRU_OA_D5_STS_CNTRL_DESC_Msk                 (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_D6_ADDR                               (11U)
#define MTB_UBM_FRU_OA_D6_LEN                                (1U)

#define MTB_UBM_FRU_OA_D6_ROUT_DESC_NUM_Pos                  (0U)
#define MTB_UBM_FRU_OA_D6_ROUT_DESC_NUM_Msk                  (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_D7_ADDR                               (12U)
#define MTB_UBM_FRU_OA_D7_LEN                                (1U)

#define MTB_UBM_FRU_OA_D7_BP_DFC_NUM_Pos                     (0U)
#define MTB_UBM_FRU_OA_D7_BP_DFC_NUM_Msk                     (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_D8_ADDR                               (13U)
#define MTB_UBM_FRU_OA_D8_LEN                                (1U)

#define MTB_UBM_FRU_OA_D8_POW_PER_DFC_Pos                    (0U)
#define MTB_UBM_FRU_OA_D8_POW_PER_DFC_Msk                    (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_OA_D9_ADDR                               (14U)
#define MTB_UBM_FRU_OA_D9_LEN                                (1U)

#define MTB_UBM_FRU_OA_D9_MUX_CH_CNT_Pos                     (0U)
#define MTB_UBM_FRU_OA_D9_MUX_CH_CNT_Msk                     (0x03U)

#define MTB_UBM_FRU_OA_D9_EN_LOC_Pos                         (2U)
#define MTB_UBM_FRU_OA_D9_EN_LOC_Msk                         (0x0CU)

#define MTB_UBM_FRU_OA_D9_MUX_TYPE_Pos                       (6U)
#define MTB_UBM_FRU_OA_D9_MUX_TYPE_Msk                       (0x40U)

#define MTB_UBM_FRU_OA_D9_VALID_Pos                          (7U)
#define MTB_UBM_FRU_OA_D9_VALID_Msk                          (0x80U)
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/* Route information header */
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_H_ADDR                                (24U)
#define MTB_UBM_FRU_RI_H_LEN                                 (5U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_H0_ADDR                               (0U)
#define MTB_UBM_FRU_RI_H0_LEN                                (1U)

#define MTB_UBM_FRU_RI_H0_REC_T_Pos                          (0U)
#define MTB_UBM_FRU_RI_H0_REC_T_Msk                          (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_H1_ADDR                               (1U)
#define MTB_UBM_FRU_RI_H1_LEN                                (1U)

#define MTB_UBM_FRU_RI_H1_FORM_V_Pos                         (0U)
#define MTB_UBM_FRU_RI_H1_FORM_V_Msk                         (0x7U)

#define MTB_UBM_FRU_RI_H1_EOLST_Pos                          (7U)
#define MTB_UBM_FRU_RI_H1_EOLST_Msk                          (0x80U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_H2_ADDR                               (2U)
#define MTB_UBM_FRU_RI_H2_LEN                                (1U)

#define MTB_UBM_FRU_RI_H2_REC_LEN_Pos                        (0U)
#define MTB_UBM_FRU_RI_H2_REC_LEN_Msk                        (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_H3_ADDR                               (3U)
#define MTB_UBM_FRU_RI_H3_LEN                                (1U)

#define MTB_UBM_FRU_RI_H3_REC_CHKS_Pos                       (0U)
#define MTB_UBM_FRU_RI_H3_REC_CHKS_Msk                       (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_H4_ADDR                               (4U)
#define MTB_UBM_FRU_RI_H4_LEN                                (1U)

#define MTB_UBM_FRU_RI_H4_HDR_CHKS_Pos                       (0U)
#define MTB_UBM_FRU_RI_H4_HDR_CHKS_Mks                       (0xFFU)
//------------------------------------------------------------------------------
/* Route information descriptor N */
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_D_ADDR                                (29U)
#define MTB_UBM_FRU_RI_D_LEN                                 (7U)
#define MTB_UBM_FRU_MAX_ROUTE_DESCRIPTORS                    (32U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_D0_ADDR                               (0U)
#define MTB_UBM_FRU_RI_D0_LEN                                (1U)

#define MTB_UBM_FRU_RI_D0_UBM_TYPE_Pos                       (0U)
#define MTB_UBM_FRU_RI_D0_UBM_TYPE_Msk                       (0x01U)

#define MTB_UBM_FRU_RI_D0_2W_S_ADDR_Pos                      (1U)
#define MTB_UBM_FRU_RI_D0_2W_S_ADDR_Msk                      (0xFEU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_D1_ADDR                               (1U)
#define MTB_UBM_FRU_RI_D1_LEN                                (1U)

#define MTB_UBM_FRU_RI_D1_STS_DI_Pos                         (0U)
#define MTB_UBM_FRU_RI_D1_STS_DI_Msk                         (0xFFU)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_D2_ADDR                               (2U)
#define MTB_UBM_FRU_RI_D2_LEN                                (1U)

#define MTB_UBM_FRU_RI_D2_SFF_TA_1001_Pos                    (1U)
#define MTB_UBM_FRU_RI_D2_SFF_TA_1001_Msk                    (0x02U)

#define MTB_UBM_FRU_RI_D2_GEN_Z_Pos                          (3U)
#define MTB_UBM_FRU_RI_D2_GEN_Z_Msk                          (0x08U)

#define MTB_UBM_FRU_RI_D2_SAS_SATA_Pos                       (4U)
#define MTB_UBM_FRU_RI_D2_SAS_SATA_Msk                       (0x10U)

#define MTB_UBM_FRU_RI_D2_QPCIE_Pos                          (5U)
#define MTB_UBM_FRU_RI_D2_QPCIE_Msk                          (0x20U)

#define MTB_UBM_FRU_RI_D2_DFC_EMPTY_Pos                      (7U)
#define MTB_UBM_FRU_RI_D2_DFC_EMPTY_Msk                      (0x80U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_D3_ADDR                               (3U)
#define MTB_UBM_FRU_RI_D3_LEN                                (1U)

#define MTB_UBM_FRU_RI_D3_LINK_WIDTH_Pos                     (0U)
#define MTB_UBM_FRU_RI_D3_LINK_WIDTH_Msk                     (0x0FU)

#define MTB_UBM_FRU_RI_D3_PORT_TYPE_Pos                      (6U)
#define MTB_UBM_FRU_RI_D3_PORT_TYPE_Msk                      (0x40U)

#define MTB_UBM_FRU_RI_D3_DOMAIN_Pos                         (7U)
#define MTB_UBM_FRU_RI_D3_DOMAIN_Msk                         (0x80U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_D4_ADDR                               (4U)
#define MTB_UBM_FRU_RI_D4_LEN                                (1U)

#define MTB_UBM_FRU_RI_D4_MAX_SATA_LINK_RATE_Pos             (0U)
#define MTB_UBM_FRU_RI_D4_MAX_SATA_LINK_RATE_Msk             (0x03U)

#define MTB_UBM_FRU_RI_D4_MAX_PCIE_LINK_RATE_Pos             (2U)
#define MTB_UBM_FRU_RI_D4_MAX_PCIE_LINK_RATE_Msk             (0x1CU)

#define MTB_UBM_FRU_RI_D4_MAX_SAS_LINK_RATE_Pos              (5U)
#define MTB_UBM_FRU_RI_D4_MAX_SAS_LINK_RATE_Msk              (0xE0U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_D5_ADDR                               (5U)
#define MTB_UBM_FRU_RI_D5_LEN                                (1U)

#define MTB_UBM_FRU_RI_D5_HFC_STARTING_LANE_Pos              (0U)
#define MTB_UBM_FRU_RI_D5_HFC_STARTING_LANE_Msk              (0x0FU)

#define MTB_UBM_FRU_RI_D5_HFC_IDENTITY_Pos                   (4U)
#define MTB_UBM_FRU_RI_D5_HFC_IDENTITY_Msk                   (0xF0U)
//------------------------------------------------------------------------------
#define MTB_UBM_FRU_RI_D6_ADDR                               (6U)
#define MTB_UBM_FRU_RI_D6_LEN                                (1U)

#define MTB_UBM_FRU_RI_D6_SLOT_OFFSET_Pos                    (0U)
#define MTB_UBM_FRU_RI_D6_SLOT_OFFSET_Msk                    (0xFFU)
//------------------------------------------------------------------------------

/** FRU Array size */
#define MTB_UBM_FRU_SIZE                                     (256U)
/** FRU Array address */
#define MTB_UBM_FRU_START_ADDRESS                            (0U)

/** FRU Common Header Format Version */
#define MTB_UBM_FRU_CH_FORMAT_VERSION                        (0x01U)
/** FRU Common Header Format Version */
#define MTB_UBM_FRU_CH_AREA_NOT_PRESENT                      (0x00U)
/** FRU Common Header Format Version */
#define MTB_UBM_FRU_CH_PADDING                               (0x00U)

/** FRU Overview Area record type */
#define MTB_UBM_FRU_OA_RECORD_TYPE                           (0xA0U)
/** FRU Overview Area record format */
#define MTB_UBM_FRU_OA_RECORD_FMT                            (0x02U)
/** FRU Overview Area record format */
#define MTB_UBM_FRU_OA_EOL_VALUE                             (0x00U)
/** FRU Overview Area record length */
#define MTB_UBM_FRU_OA_RECORD_LENGTH                         (0x0BU)
/** FRU Overview Area specification version */
#define MTB_UBM_FRU_SPEC_VERSION                             (MTB_UBM_SPEC_VER)

/** FRU Overview Area invalid flag */
#define MTB_UBM_FRU_OA_VALID                                 (0x00U)
/** FRU Overview Area valid flag */
#define MTB_UBM_FRU_OA_INVALID                               (0x01U)

/** FRU Overview Area record type */
#define MTB_UBM_FRU_RI_RECORD_TYPE                           (0xA1U)
/** FRU Overview Area record format */
#define MTB_UBM_FRU_RI_RECORD_FMT                            (0x02U)
/** FRU Overview Area last atribute flag */
#define MTB_UBM_FRU_RI_EOL_FLAG                              (0x01U)
/** @} group_ubm_macros_fru */
/** \endcond */
/**
 * \addtogroup group_ubm_macros_fru_oa
 * @{
 */

/** No mux present */
#define MTB_UBM_FRU_OA_2WIRE_ARRANGEMENT_NO_MUX               (0x00U)
/** DFC 2Wire interface behind Mux */
#define MTB_UBM_FRU_OA_2WIRE_ARRANGEMENT_DFC_MUX              (0x01U)
/** UBM Controller(s) and DFC 2Wire interface located behind Mux */
#define MTB_UBM_FRU_OA_2WIRE_ARRANGEMENT_DFC_CONTROLLER_MUX   (0x03U)

/** 2Wire Maximum byte count*/
/** No Limit */
#define MTB_UBM_FRU_OA_2WIRE_MUX_BYTE_CNT_NO_LIMIT           (0x00U)
/** 16 bytes */
#define MTB_UBM_FRU_OA_2WIRE_MUX_BYTE_CNT_16BYTES            (0x01U)
/** 32 bytes */
#define MTB_UBM_FRU_OA_2WIRE_MUX_BYTE_CNT_32BYTES            (0x02U)
/** 64 bytes */
#define MTB_UBM_FRU_OA_2WIRE_MUX_BYTE_CNT_64BYTES            (0x03U)
/** 128 bytes */
#define MTB_UBM_FRU_OA_2WIRE_MUX_BYTE_CNT_128BYTES           (0x04U)
/** 256 bytes */
#define MTB_UBM_FRU_OA_2WIRE_MUX_BYTE_CNT_256BYTES           (0x05U)

/** DFC PERST# Management Override */
/** No override */
#define MTB_UBM_FRU_OA_F_PERST_NO_OVERRIDE                   (0x00U)
/** DFC PERST# Managed upon install */
#define MTB_UBM_FRU_OA_F_PERST_MANAGED_UPON_INSTALL          (0x40U)
/** DFC PERST# Automatically released upon install */
#define MTB_UBM_FRU_OA_F_PERST_AUTO_UPON_INSTALL             (0x80U)

/** Operation State Count Mask insrement feature */
/** Operational State transitions do not cause the Change Count field to increment */
#define MTB_UBM_FRU_OA_F_OSCM_NO_INC                         (0x00U)
/** Operational State transitions cause the Change Count field to increment */
#define MTB_UBM_FRU_OA_F_OSCM_INC                            (0x20U)

/** Drive Type Installed Change Count Mask Increment feature */
/** Drive Type Installed field changes do not cause the Change Count field to increment */
#define MTB_UBM_FRU_OA_F_DTICC_NO_INC                        (0x00U)
/** Drive Type Installed field changes cause the Change Count field to increment */
#define MTB_UBM_FRU_OA_F_DTICC_INC                           (0x10U)

/** PCIe Reset Change Count Mask Increment feature */
/** PCIe Reset field changes do not cause the Change Count field to increment */
#define MTB_UBM_FRU_OA_F_PCIE_RES_NO_INC                     (0x00U)
/** PCIe Reset field changes cause the Change Count field to increment */
#define MTB_UBM_FRU_OA_F_PCIE_RES_INC                        (0x08U)

/** CPRSNT Legacy Mode - the behavior of the CPRSNT/CHANGE_DETECT# signal */
/** CHANGE_DETECT# interrupt operation */
#define MTB_UBM_FRU_OA_F_CHANGE_DETECT                       (0x00U)
/** CPRSNT# legacy operation */
#define MTB_UBM_FRU_OA_F_CPRSNT                              (0x04U)

/** Write Checksum Checking feature */
/** No Checksum Checking */
#define MTB_UBM_FRU_OA_F_WR_NO_CHECKS_CHECK                  (0x00U)
/** Checksum checking is enabled */
#define MTB_UBM_FRU_OA_F_WR_CHECKS_CHECK                     (0x02U)

/** Read Checksum Creation feature */
/** No Checksum Creation is performed */
#define MTB_UBM_FRU_OA_F_RD_NO_CHECKS_CHECK                  (0x00U)
/** Checksum Creation is enabled */
#define MTB_UBM_FRU_OA_F_RD_CHECKS_CREATION                  (0x01U)

/** DFC SMBRST signal for all DFCs */
/** NOP (e.g. No DFC SMBus Reset sequence outstanding) */
#define MTB_UBM_FRU_OA_F_SMBRST_NOP                          (0x00U)
/** Initiate DFC SMBus Reset sequence */
#define MTB_UBM_FRU_OA_F_SMBRST_INIT_SEQ                     (0x100U)

/** 2Wire Mux Enable Channel Selection Method */
/** Channels are selected using bit location */
#define MTB_UBM_FRU_OA_2W_MUX_CH_ENABLE_LOC                  (0x00U)
/** Channels are selected using enable bit and channel byte */
#define MTB_UBM_FRU_OA_2W_MUX_CH_ENABLE_LOC_CB               (0x01U)

/** 2Wire Mux Enable Bit Location */
/** Mux Enable is not applicable */
#define MTB_UBM_FRU_OA_2W_MUX_ENABLE_NA                      (0x00U)
/** Mux Enable located at Bit 2 of Channel Select Byte */
#define MTB_UBM_FRU_OA_2W_MUX_ENABLE_2CH_SEL                 (0x02U)
/** Mux Enable located at Bit 3 of Channel Select Byte */
#define MTB_UBM_FRU_OA_2W_MUX_ENABLE_3CH_SEL                 (0x03U)

/** 2Wire Mux Channel Count */
/** No Mux implemented */
#define MTB_UBM_FRU_OA_2W_MUX_NO_MUX                         (0x00U)
/** 2 Channel Mux implemented */
#define MTB_UBM_FRU_OA_2W_MUX_2CH                            (0x01U)
/** 4 Channel Mux implemented */
#define MTB_UBM_FRU_OA_2W_MUX_4CH                            (0x02U)
/** 8 Channel Mux implemented */
#define MTB_UBM_FRU_OA_2W_MUX_8CH                            (0x03U)

/** @} group_ubm_macros_fru */

/** \cond DO_NOT_DOCUMENT */


void mtb_ubm_fru_handle_request(mtb_stc_ubm_context_t* ubm_context, mtb_stc_ubm_hfc_t* hfc_context);
void mtb_ubm_fru_init_common_header(uint8_t* ram_array);
void mtb_ubm_fru_init_overview_area(const mtb_stc_ubm_backplane_cfg_t* config, uint8_t* ram_array);
void mtb_ubm_fru_init_ri_header(uint8_t routes_num, uint8_t* ram_array);
void mtb_ubm_fru_init_ri_descriptor(uint32_t num_of_routes,
                                    const mtb_stc_ubm_routing_t* routing_info,
                                    uint8_t* ram_array);

/** \endcond */

#ifdef __cplusplus
}
#endif

#endif /* UBM_FRU_H */
