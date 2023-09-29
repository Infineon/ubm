/***************************************************************************//**
 * \file mtb_ubm_fru.c
 * \version 1.0
 *
 * \brief
 * Provides common API implementation for the UBM FRU.
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

#include "mtb_ubm.h"
#include "mtb_ubm_fru.h"


#define MTB_UBM_FRU_FEATURES_READ_CHECKSUM_Pos               (0U)
#define MTB_UBM_FRU_FEATURES_READ_CHECKSUM_Msk               (0x01U)
#define MTB_UBM_FRU_FEATURES_WRITE_CHECKSUM_Pos              (1U)
#define MTB_UBM_FRU_FEATURES_WRITE_CHECKSUM_Msk              (0x02U)
#define MTB_UBM_FRU_FEATURES_CPRSNT_MODE_Pos                 (2U)
#define MTB_UBM_FRU_FEATURES_CPRSNT_MODE_Msk                 (0x04U)
#define MTB_UBM_FRU_FEATURES_PCIE_RESET_CC_Pos               (3U)
#define MTB_UBM_FRU_FEATURES_PCIE_RESET_CC_Msk               (0x08U)
#define MTB_UBM_FRU_FEATURES_DRIVE_TYPE_CC_Pos               (4U)
#define MTB_UBM_FRU_FEATURES_DRIVE_TYPE_CC_Msk               (0x10U)
#define MTB_UBM_FRU_FEATURES_OP_STATE_CC_Pos                 (5U)
#define MTB_UBM_FRU_FEATURES_OP_STATE_CC_Msk                 (0x20U)
#define MTB_UBM_FRU_FEATURES_PERST_OVRD_Pos                  (6U)
#define MTB_UBM_FRU_FEATURES_PERST_OVRD_Msk                  (0xC0U)
#define MTB_UBM_FRU_FEATURES_SMBUS_RESET_Pos                 (0U)
#define MTB_UBM_FRU_FEATURES_SMBUS_RESET_Msk                 (0x01U)
#define MTB_UBM_FRU_ADDRESS_BYTE_POS                         (0)


/*******************************************************************************
*            Internal API
*******************************************************************************/
static uint8_t mtb_ubm_fru_calculate_checksum(const uint8_t* data_bytes, uint32_t data_length);


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_format_version
****************************************************************************//**
*
*  Sets the FRU value of the format version of the common header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_format_version(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H0_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FORMAT_VERSION_Msk));
        bit_val |= ((*value << MTB_UBM_FORMAT_VERSION_Pos) & MTB_UBM_FORMAT_VERSION_Msk);

        array[MTB_UBM_FRU_CH_H0_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_reserved
****************************************************************************//**
*
*  Sets the FRU value of the reserved field of the common header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_reserved(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H0_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_CH_H0_RESERVED_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_CH_H0_RESERVED_Pos) & MTB_UBM_FRU_CH_H0_RESERVED_Msk);

        array[MTB_UBM_FRU_CH_H0_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_internal
****************************************************************************//**
*
*  Sets the FRU value of the internal area starting offset of the common header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_internal(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H1_ADDR];

        bit_val |= *value & MTB_UBM_FRU_CH_H1_INTENAL_OFFSET_Msk;

        array[MTB_UBM_FRU_CH_H1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_chasis_info
****************************************************************************//**
*
*  Sets the FRU value of the info starting offset of the common header chassis.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_chasis_info(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H2_ADDR];

        bit_val |= *value & MTB_UBM_FRU_CH_H2_CHASIS_INFO_OFFSET_Msk;

        array[MTB_UBM_FRU_CH_H2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_board_info
****************************************************************************//**
*
*  Sets the FRU value of the chassis info field of the common header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_board_info(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H3_ADDR];

        bit_val |= *value & MTB_UBM_FRU_CH_H3_BOARD_INFO_OFFSET_Msk;

        array[MTB_UBM_FRU_CH_H3_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_product_info
****************************************************************************//**
*
*  Sets the FRU value of the product info field of the common header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_product_info(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H4_ADDR];

        bit_val |= *value & MTB_UBM_FRU_CH_H4_PRODUCT_INFO_OFFSET_Msk;

        array[MTB_UBM_FRU_CH_H4_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_multirecord_offset
****************************************************************************//**
*
*  Sets the FRU value of the multirecord offset field of the common header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_multirecord_offset(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H5_ADDR];

        bit_val |= *value & MTB_UBM_FRU_CH_H5_MUTRECORD_OFFSET_Msk;

        array[MTB_UBM_FRU_CH_H5_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_padding
****************************************************************************//**
*
*  Sets the FRU value of the padding field of the common header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_padding(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H6_ADDR];

        bit_val |= *value & MTB_UBM_FRU_CH_H6_PAD_Msk;

        array[MTB_UBM_FRU_CH_H6_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_checksum
****************************************************************************//**
*
*  Sets the FRU value of the offset field of the common header multirecord.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_checksum(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H7_ADDR];

        bit_val |= *value & MTB_UBM_FRU_CH_H7_CHECKSUM_Msk;

        array[MTB_UBM_FRU_CH_H7_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_handle_request
****************************************************************************//**
*
*  Handles requests from HFC master to UBM FRU.
*
* \param ubm_context
*  The pointer to the UBM context structure.
*
* \param hfc_context
*  The pointer to the HFC context structure.
*
*******************************************************************************/
void mtb_ubm_fru_handle_request(mtb_stc_ubm_context_t* ubm_context, mtb_stc_ubm_hfc_t* hfc_context)
{
    uint16_t size = MTB_UBM_FRU_SIZE - hfc_context->i2c.write_buffer[MTB_UBM_FRU_ADDRESS_BYTE_POS];

    if (hfc_context->i2c.read_request)
    {
        (void)Cy_Em_EEPROM_Read(hfc_context->i2c.write_buffer[MTB_UBM_FRU_ADDRESS_BYTE_POS],
                                &hfc_context->i2c.read_buffer,
                                size,
                                &ubm_context->fru_context);
        hfc_context->i2c.read_data_length = size;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_init_common_header
****************************************************************************//**
*
*  Initializes the common header of the FRU.
*
* \param ram_array
*  The pointer to the FRU image section in the RAM.
*
*******************************************************************************/
void mtb_ubm_fru_init_common_header(uint8_t* ram_array)
{
    uint8_t constant_value;

    constant_value = MTB_UBM_FRU_CH_FORMAT_VERSION;
    mtb_ubm_fru_set_ch_format_version(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_CH_AREA_NOT_PRESENT;
    mtb_ubm_fru_set_ch_reserved(&constant_value, ram_array);
    mtb_ubm_fru_set_ch_internal(&constant_value, ram_array);
    mtb_ubm_fru_set_ch_chasis_info(&constant_value, ram_array);
    mtb_ubm_fru_set_ch_board_info(&constant_value, ram_array);
    mtb_ubm_fru_set_ch_product_info(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_CH_LEN / 8U; /* in multiples of 8 bytes */
    mtb_ubm_fru_set_ch_multirecord_offset(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_CH_PADDING;
    mtb_ubm_fru_set_ch_padding(&constant_value, ram_array);

    constant_value = mtb_ubm_fru_calculate_checksum(ram_array, (MTB_UBM_FRU_CH_LEN - 1U));
    mtb_ubm_fru_set_ch_checksum(&constant_value, ram_array);
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_record_type
****************************************************************************//**
*
*  Sets the FRU value of the multirecord offset field of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_record_type(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_OA_H0_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_record_fmt
****************************************************************************//**
*
*  Sets the FRU value of the record format of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_record_fmt(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_H1_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_H1_FORM_V_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_H1_FORM_V_Pos) & MTB_UBM_FRU_OA_H1_FORM_V_Msk);

        array[MTB_UBM_FRU_OA_H1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_eol_flag
****************************************************************************//**
*
*  Sets the FRU value of the overview area end of the list flag.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_eol_flag(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_H1_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_H1_EOLST_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_H1_EOLST_Pos) & MTB_UBM_FRU_OA_H1_EOLST_Msk);

        array[MTB_UBM_FRU_OA_H1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_record_length
****************************************************************************//**
*
*  Sets the FRU value of the record length of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_record_length(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_OA_H2_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_record_checksum
****************************************************************************//**
*
*  Sets the FRU value of the record checksum of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_record_checksum(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_OA_H3_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_header_checksum
****************************************************************************//**
*
*  Sets the FRU value of the header checksum of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_header_checksum(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_OA_H4_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_spec_version
****************************************************************************//**
*
*  Sets the FRU value of the spec version of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_spec_version(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_OA_D0_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_device_arrangement
****************************************************************************//**
*
*  Sets the FRU value of the ata mux device arrangement of the overview area .
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_device_arrangement(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D1_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_D1_2W_DEV_ARR_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_D1_2W_DEV_ARR_Pos) & MTB_UBM_FRU_OA_D1_2W_DEV_ARR_Msk);

        array[MTB_UBM_FRU_OA_D1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_mux_address
****************************************************************************//**
*
*  Sets the FRU value of the data mux device address of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_mux_address(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D1_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_D1_2W_MUX_ADDR_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_D1_2W_MUX_ADDR_Pos) & MTB_UBM_FRU_OA_D1_2W_MUX_ADDR_Msk);

        array[MTB_UBM_FRU_OA_D1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_mux_byte_count
****************************************************************************//**
*
*  Sets the FRU value of the mux byte count of the overview area data.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_mux_byte_count(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D1_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_D1_2W_MUX_BC_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_D1_2W_MUX_BC_Pos) & MTB_UBM_FRU_OA_D1_2W_MUX_BC_Msk);

        array[MTB_UBM_FRU_OA_D1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_fru_invalid
****************************************************************************//**
*
*  Sets the FRU value of the data FRU invalid flag of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_fru_invalid(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D2_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_D2_FRU_INV_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_D2_FRU_INV_Pos) & MTB_UBM_FRU_OA_D2_FRU_INV_Msk);

        array[MTB_UBM_FRU_OA_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_controller_max_time_limit
****************************************************************************//**
*
*  Sets the FRU value of the data controller maximum time limit of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_controller_max_time_limit(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D2_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_D2_C_MAX_T_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_D2_C_MAX_T_Pos) & MTB_UBM_FRU_OA_D2_C_MAX_T_Msk);

        array[MTB_UBM_FRU_OA_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_controller_features
****************************************************************************//**
*
*  Sets the FRU value of the data controller features of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_controller_features(const mtb_stc_ubm_features_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_OA_D3_4_ADDR] =
            ((uint8_t)value->read_checksum_creation << MTB_UBM_FRU_FEATURES_READ_CHECKSUM_Pos) &
            MTB_UBM_FRU_FEATURES_READ_CHECKSUM_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            ((uint8_t)value->write_checksum_checking << MTB_UBM_FRU_FEATURES_WRITE_CHECKSUM_Pos) &
            MTB_UBM_FRU_FEATURES_WRITE_CHECKSUM_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            ((uint8_t)value->cprsnt_legacy_mode << MTB_UBM_FRU_FEATURES_CPRSNT_MODE_Pos) &
            MTB_UBM_FRU_FEATURES_CPRSNT_MODE_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            ((uint8_t)value->pcie_reset_change_count_mask << MTB_UBM_FRU_FEATURES_PCIE_RESET_CC_Pos) &
            MTB_UBM_FRU_FEATURES_PCIE_RESET_CC_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            ((uint8_t)value->drive_type_installed_change_count_mask << MTB_UBM_FRU_FEATURES_DRIVE_TYPE_CC_Pos) &
            MTB_UBM_FRU_FEATURES_DRIVE_TYPE_CC_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            ((uint8_t)value->operational_state_change_count_mask << MTB_UBM_FRU_FEATURES_OP_STATE_CC_Pos) &
            MTB_UBM_FRU_FEATURES_OP_STATE_CC_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            (value->perst_management_override << MTB_UBM_FRU_FEATURES_PERST_OVRD_Pos) &
            MTB_UBM_FRU_FEATURES_PERST_OVRD_Msk;

        array[MTB_UBM_FRU_OA_D3_4_ADDR + 1U] =
            ((uint8_t)value->smbus_reset_control << MTB_UBM_FRU_FEATURES_SMBUS_RESET_Pos) &
            MTB_UBM_FRU_FEATURES_SMBUS_RESET_Msk;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_number_status_control_descriptors
****************************************************************************//**
*
*  Sets the FRU value of the data number of the control and status
*  descriptors of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_number_status_control_descriptors(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_OA_D5_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_number_port_route_descriptors
****************************************************************************//**
*
*  Sets the FRU value of the data number of the port route
*  descriptors of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_number_port_route_descriptors(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_OA_D6_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_number_backplane_dfc
****************************************************************************//**
*
*  Sets the FRU value of the data number of the backplane DFCs of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_number_backplane_dfc(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_OA_D7_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_maximum_power_per_dfc
****************************************************************************//**
*
*  Sets the FRU value of the data maximum power per DFCs of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_maximum_power_per_dfc(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_OA_D8_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_mux_channel_count
****************************************************************************//**
*
*  Sets the FRU value of the data mux channel count of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_mux_channel_count(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D9_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_D9_MUX_CH_CNT_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_D9_MUX_CH_CNT_Pos) & MTB_UBM_FRU_OA_D9_MUX_CH_CNT_Msk);

        array[MTB_UBM_FRU_OA_D9_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_enable_bit_location
****************************************************************************//**
*
*  Sets the FRU value of the data mux channel count of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_enable_bit_location(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D9_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_D9_EN_LOC_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_D9_EN_LOC_Pos) & MTB_UBM_FRU_OA_D9_EN_LOC_Msk);

        array[MTB_UBM_FRU_OA_D9_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_mux_type
****************************************************************************//**
*
*  Sets the FRU value of the data mux type of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_mux_type(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D9_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_D9_MUX_TYPE_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_D9_MUX_TYPE_Pos) & MTB_UBM_FRU_OA_D9_MUX_TYPE_Msk);

        array[MTB_UBM_FRU_OA_D9_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_valid
****************************************************************************//**
*
*  Sets the FRU value of the validity bit of the overview area.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_valid(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D9_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_D9_VALID_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_D9_VALID_Pos) & MTB_UBM_FRU_OA_D9_VALID_Msk);

        array[MTB_UBM_FRU_OA_D9_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_init_overview_area
****************************************************************************//**
*
*  Initializes the FRU overview area.
*
* \param config
*  The pointer to the UBM configuration structure.
*
* \param ram_array
*  The pointer to the FRU image section in the RAM.
*
*******************************************************************************/
void mtb_ubm_fru_init_overview_area(const mtb_stc_ubm_backplane_cfg_t* config, uint8_t* ram_array)
{
    uint8_t constant_value;

    constant_value = MTB_UBM_FRU_OA_RECORD_TYPE;
    mtb_ubm_fru_set_oa_record_type(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_OA_RECORD_FMT;
    mtb_ubm_fru_set_oa_record_fmt(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_OA_EOL_VALUE;
    mtb_ubm_fru_set_oa_eol_flag(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_OA_RECORD_LENGTH;
    mtb_ubm_fru_set_oa_record_length(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_SPEC_VERSION;
    mtb_ubm_fru_set_oa_data_spec_version(&constant_value, ram_array);

    mtb_ubm_fru_set_oa_data_device_arrangement(&config->overview_area->two_wire_device_arrangement, ram_array);
    mtb_ubm_fru_set_oa_data_mux_address(&config->overview_area->two_wire_mux_address, ram_array);
    mtb_ubm_fru_set_oa_data_mux_byte_count(&config->overview_area->two_wire_max_byte_count, ram_array);

    constant_value = MTB_UBM_FRU_OA_VALID;
    mtb_ubm_fru_set_oa_data_fru_invalid(&constant_value, ram_array);

    mtb_ubm_fru_set_oa_data_controller_max_time_limit(&config->overview_area->ubm_max_time_limit, ram_array);
    mtb_ubm_fru_set_oa_data_controller_features(&config->overview_area->ubm_controller_features, ram_array);

    constant_value = config->num_of_dfc;
    mtb_ubm_fru_set_oa_data_number_status_control_descriptors(&constant_value, ram_array);

    constant_value = config->num_of_routes;
    mtb_ubm_fru_set_oa_data_number_port_route_descriptors(&constant_value, ram_array);

    constant_value = config->num_of_dfc;
    mtb_ubm_fru_set_oa_data_number_backplane_dfc(&constant_value, ram_array);

    mtb_ubm_fru_set_oa_data_maximum_power_per_dfc(&config->overview_area->maximum_power_per_dfc, ram_array);
    mtb_ubm_fru_set_oa_data_mux_channel_count(&config->overview_area->mux_channel_count, ram_array);
    mtb_ubm_fru_set_oa_data_enable_bit_location(&config->overview_area->enable_bit_location, ram_array);
    mtb_ubm_fru_set_oa_data_mux_type(&config->overview_area->mux_type, ram_array);

    constant_value = MTB_UBM_FRU_OA_VALID;
    mtb_ubm_fru_set_oa_data_valid(&constant_value, ram_array);

    constant_value = mtb_ubm_fru_calculate_checksum(&ram_array[MTB_UBM_FRU_OA_H_LEN],
                                                    (MTB_UBM_FRU_OA_LEN - MTB_UBM_FRU_OA_H_LEN));
    mtb_ubm_fru_set_oa_record_checksum(&constant_value, ram_array);

    constant_value = mtb_ubm_fru_calculate_checksum(ram_array, (MTB_UBM_FRU_OA_H_LEN - MTB_UBM_FRU_OA_CHEKSUM_LEN));
    mtb_ubm_fru_set_oa_header_checksum(&constant_value, ram_array);
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_record_type
****************************************************************************//**
*
*  Sets the FRU value of the record type of the route information header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_record_type(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_RI_H0_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_record_fmt
****************************************************************************//**
*
*  Sets the FRU value of the record format of the route information header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_record_fmt(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_RI_H1_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_H1_FORM_V_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_RI_H1_FORM_V_Pos) & MTB_UBM_FRU_RI_H1_FORM_V_Msk);

        array[MTB_UBM_FRU_RI_H1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_eol_flag
****************************************************************************//**
*
*  Sets the FRU value of the end of the list flag of the route information header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_eol_flag(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_RI_H1_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_OA_H1_EOLST_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_OA_H1_EOLST_Pos) & MTB_UBM_FRU_OA_H1_EOLST_Msk);

        array[MTB_UBM_FRU_RI_H1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_record_length
****************************************************************************//**
*
*  Sets the FRU value of the record length of the route information header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_record_length(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_RI_H2_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_record_checksum
****************************************************************************//**
*
*  Sets the FRU value of the record checksum of the route information header.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_record_checksum(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_RI_H3_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_header_checksum
****************************************************************************//**
*
*  Sets the FRU value of the header checksum of the route information.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_header_checksum(const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[MTB_UBM_FRU_RI_H4_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_init_ri_header
****************************************************************************//**
*
*  Initializes the FRU overview area.
*
* \param routes_num
*  The total number of routes.
*
* \param ram_array
*  The pointer to the FRU image section in the RAM.
*
*******************************************************************************/
void mtb_ubm_fru_init_ri_header(uint8_t routes_num, uint8_t* ram_array)
{
    uint8_t constant_value;

    constant_value = MTB_UBM_FRU_RI_RECORD_TYPE;
    mtb_ubm_fru_set_ri_record_type(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_RI_RECORD_FMT;
    mtb_ubm_fru_set_ri_record_fmt(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_RI_EOL_FLAG;
    mtb_ubm_fru_set_ri_eol_flag(&constant_value, ram_array);

    constant_value = routes_num * MTB_UBM_FRU_RI_D_LEN;
    mtb_ubm_fru_set_ri_record_length(&constant_value, ram_array);

    constant_value = mtb_ubm_fru_calculate_checksum(&ram_array[MTB_UBM_FRU_RI_H_LEN],
                                                    ((uint32_t)routes_num * MTB_UBM_FRU_RI_D_LEN));
    mtb_ubm_fru_set_ri_record_checksum(&constant_value, ram_array);

    constant_value = mtb_ubm_fru_calculate_checksum(&ram_array[0U], (MTB_UBM_FRU_RI_H_LEN - 1U));
    mtb_ubm_fru_set_ri_header_checksum(&constant_value, ram_array);
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_ctrl_type
****************************************************************************//**
*
*  Sets the FRU value of the controller type of the route information UBM.
*
* \param route
* The route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_ctrl_type(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D0_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D0_UBM_TYPE_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_RI_D0_UBM_TYPE_Pos) & MTB_UBM_FRU_RI_D0_UBM_TYPE_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D0_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_two_wire_address
****************************************************************************//**
*
*  Sets the FRU value of the two-wire address of the route information UBM.
*
* \param route
* The route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_two_wire_address(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D0_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D0_2W_S_ADDR_Msk));
        bit_val |= ((*value) & MTB_UBM_FRU_RI_D0_2W_S_ADDR_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D0_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_sts_ctrl
****************************************************************************//**
*
*  Sets the FRU value of the UBM DFC Status and Control Descriptor Index of the route information .
*
* \param route
* The route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_sts_ctrl(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D1_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_sff_ta_1001
****************************************************************************//**
*
*  Sets the FRU value of the support bit of the route information UBM SFF-TA-1001.
*
* \param route
* The route number.
*
* \param bool_value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_sff_ta_1001(uint32_t route, const bool* bool_value, uint8_t* array)
{
    if ((NULL != array) && (NULL != bool_value))
    {
        uint8_t value = (*bool_value) ? 0x01U : 0x00U;
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D2_SFF_TA_1001_Msk));
        bit_val |= ((value << MTB_UBM_FRU_RI_D2_SFF_TA_1001_Pos) & MTB_UBM_FRU_RI_D2_SFF_TA_1001_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_gen_z
****************************************************************************//**
*
*  Sets the FRU value of the support bit of the route information UBM Gen-Z.
*
* \param route
* The route number.
*
* \param bool_value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_gen_z(uint32_t route, const bool* bool_value, uint8_t* array)
{
    if ((NULL != array) && (NULL != bool_value))
    {
        uint8_t value = (*bool_value) ? 0x01U : 0x00U;
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D2_GEN_Z_Msk));
        bit_val |= ((value << MTB_UBM_FRU_RI_D2_GEN_Z_Pos) & MTB_UBM_FRU_RI_D2_GEN_Z_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_sas_sata
****************************************************************************//**
*
*  Sets the FRU value of the support bit of the route information UBM SAS/SATA.
*
* \param route
* The route number.
*
* \param bool_value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_sas_sata(uint32_t route, const bool* bool_value, uint8_t* array)
{
    if ((NULL != array) && (NULL != bool_value))
    {
        uint8_t value = (*bool_value) ? 0x01U : 0x00U;
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D2_SAS_SATA_Msk));
        bit_val |= ((value << MTB_UBM_FRU_RI_D2_SAS_SATA_Pos) & MTB_UBM_FRU_RI_D2_SAS_SATA_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_quad_pcie
****************************************************************************//**
*
*  Sets the FRU value of the support bit of the route information UBM Quad PCIe.
*
* \param route
* The route number.
*
* \param bool_value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_quad_pcie(uint32_t route, const bool* bool_value, uint8_t* array)
{
    if ((NULL != array) && (NULL != bool_value))
    {
        uint8_t value = (*bool_value) ? 0x01U : 0x00U;
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D2_QPCIE_Msk));
        bit_val |= ((value << MTB_UBM_FRU_RI_D2_QPCIE_Pos) & MTB_UBM_FRU_RI_D2_QPCIE_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_dfc_empty
****************************************************************************//**
*
*  Sets the FRU value of the empty bit of the route information UBM DFC.
*
* \param route
* The route number.
*
* \param bool_value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_dfc_empty(uint32_t route, const bool* bool_value, uint8_t* array)
{
    if ((NULL != array) && (NULL != bool_value))
    {
        uint8_t value = (*bool_value) ? 0x01U : 0x00U;
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D2_DFC_EMPTY_Msk));
        bit_val |= ((value << MTB_UBM_FRU_RI_D2_DFC_EMPTY_Pos) & MTB_UBM_FRU_RI_D2_DFC_EMPTY_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_link_width
****************************************************************************//**
*
*  Sets the FRU value of the link width of the route information UBM.
*
* \param route
* The route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_link_width(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D3_LINK_WIDTH_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_RI_D3_LINK_WIDTH_Pos) & MTB_UBM_FRU_RI_D3_LINK_WIDTH_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_port_type
****************************************************************************//**
*
*  Sets the FRU value of the port type for the UBM of the route information.
*
* \param route
* Route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_port_type(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D3_PORT_TYPE_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_RI_D3_PORT_TYPE_Pos) & MTB_UBM_FRU_RI_D3_PORT_TYPE_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_domain
****************************************************************************//**
*
*  Sets the FRU value of the port domain for the UBM of the route information.
*
* \param route
* The route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_domain(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D3_DOMAIN_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_RI_D3_DOMAIN_Pos) & MTB_UBM_FRU_RI_D3_DOMAIN_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_max_sata_rate
****************************************************************************//**
*
*  Sets the FRU value of the port maximum SATA rate for the UBM of the route information.
*
* \param route
* The route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_max_sata_rate(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D4_MAX_SATA_LINK_RATE_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_RI_D4_MAX_SATA_LINK_RATE_Pos) & MTB_UBM_FRU_RI_D4_MAX_SATA_LINK_RATE_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_max_pcie_rate
****************************************************************************//**
*
*  Sets the FRU value of the port maximum PCIe rate for the UBM of the route information.
*
* \param route
* The route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_max_pcie_rate(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D4_MAX_PCIE_LINK_RATE_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_RI_D4_MAX_PCIE_LINK_RATE_Pos) & MTB_UBM_FRU_RI_D4_MAX_PCIE_LINK_RATE_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_max_sas_rate
****************************************************************************//**
*
*  Sets the FRU value of the port maximum SAS rate for the UBM of the route information.
*
* \param route
* The route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_max_sas_rate(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D4_MAX_SAS_LINK_RATE_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_RI_D4_MAX_SAS_LINK_RATE_Pos) & MTB_UBM_FRU_RI_D4_MAX_SAS_LINK_RATE_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_hfc_start_lane
****************************************************************************//**
*
*  Sets the FRU value of the port HFC starting lane for the UBM of the route information.
*
* \param route
* The route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_hfc_start_lane(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D5_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D5_HFC_STARTING_LANE_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_RI_D5_HFC_STARTING_LANE_Pos) & MTB_UBM_FRU_RI_D5_HFC_STARTING_LANE_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D5_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_hfc_identifier
****************************************************************************//**
*
*  Sets the FRU value of the port HFC identity for the UBM of the route information.
*
* \param route
* The route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_hfc_identifier(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D5_ADDR];

        bit_val = (uint8_t)(bit_val & (~MTB_UBM_FRU_RI_D5_HFC_IDENTITY_Msk));
        bit_val |= ((*value << MTB_UBM_FRU_RI_D5_HFC_IDENTITY_Pos) & MTB_UBM_FRU_RI_D5_HFC_IDENTITY_Msk);

        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D5_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_slot_offset
****************************************************************************//**
*
*  Sets the FRU value of the port slot offset for the UBM of the route information.
*
* \param route
* The route number.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to the RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_slot_offset(uint32_t route, const uint8_t* value, uint8_t* array)
{
    if ((NULL != array) && (NULL != value))
    {
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D6_ADDR] = *value;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_init_ri_descriptor
****************************************************************************//**
*
*  Initializes the route information data of the FRU.
*
* \param num_of_routes
*  The number of routes to initialize.
*
* \param routing_info
*  The pointer to the UBM context settings.
*
* \param ram_array
*  The pointer to the FRU image section in the RAM.
*
*******************************************************************************/
void mtb_ubm_fru_init_ri_descriptor(uint32_t num_of_routes,
                                    const mtb_stc_ubm_routing_t* routing_info,
                                    uint8_t* ram_array)
{
    if ((NULL != routing_info) && (NULL != ram_array))
    {
        for (int32_t i = (int32_t)num_of_routes - 1; i >= 0; i--)
        {
            mtb_ubm_fru_set_ri_ctrl_type((uint32_t)i, ((const uint8_t*)&routing_info[i].ubm_ctrl_type), ram_array);
            mtb_ubm_fru_set_ri_two_wire_address((uint32_t)i, &routing_info[i].ubm_ctrl_slave_addr, ram_array);
            mtb_ubm_fru_set_ri_sts_ctrl((uint32_t)i, &routing_info[i].drive_connector_idx, ram_array);
            mtb_ubm_fru_set_ri_sff_ta_1001((uint32_t)i, &routing_info[i].drive_types_supported.sff_ta_1001, ram_array);
            mtb_ubm_fru_set_ri_gen_z((uint32_t)i, &routing_info[i].drive_types_supported.gen_z, ram_array);
            mtb_ubm_fru_set_ri_sas_sata((uint32_t)i, &routing_info[i].drive_types_supported.sas_sata, ram_array);
            mtb_ubm_fru_set_ri_quad_pcie((uint32_t)i, &routing_info[i].drive_types_supported.quad_pcie, ram_array);
            mtb_ubm_fru_set_ri_dfc_empty((uint32_t)i, &routing_info[i].drive_types_supported.dfc_empty, ram_array);
            mtb_ubm_fru_set_ri_link_width((uint32_t)i, ((const uint8_t*)&routing_info[i].drive_link_width), ram_array);
            mtb_ubm_fru_set_ri_port_type((uint32_t)i, ((const uint8_t*)&routing_info[i].port_type), ram_array);
            mtb_ubm_fru_set_ri_domain((uint32_t)i, ((const uint8_t*)&routing_info[i].domain), ram_array);
            mtb_ubm_fru_set_ri_max_sata_rate((uint32_t)i, ((const uint8_t*)&routing_info[i].max_sata_line_rate), ram_array);
            mtb_ubm_fru_set_ri_max_pcie_rate((uint32_t)i, ((const uint8_t*)&routing_info[i].max_pcie_line_rate), ram_array);
            mtb_ubm_fru_set_ri_max_sas_rate((uint32_t)i, ((const uint8_t*)&routing_info[i].max_sas_line_rate), ram_array);
            mtb_ubm_fru_set_ri_hfc_start_lane((uint32_t)i, &routing_info[i].hfc_starting_phy_lane, ram_array);
            mtb_ubm_fru_set_ri_hfc_identifier((uint32_t)i, &routing_info[i].hfc_identifier, ram_array);
            mtb_ubm_fru_set_ri_slot_offset((uint32_t)i, &routing_info[i].slot_offset, ram_array);
        }
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_calculate_checksum
****************************************************************************//**
*
*  Calculates the checksum.
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
static uint8_t mtb_ubm_fru_calculate_checksum(const uint8_t* data_bytes, uint32_t data_length)
{
    uint8_t checksum = 0x00U;
    uint8_t cnt = 0x00U;

    for (uint32_t i = 0U; i < data_length; i++)
    {
        cnt = cnt + data_bytes[i];
    }

    checksum = ~(cnt) + 0x01U;

    return checksum;
}
