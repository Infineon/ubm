/***************************************************************************//**
 * \file mtb_ubm_fru.c
 * \version 0.5
 *
 * \brief
 * Provides the UBM FRU common API implementation.
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
#define MTB_UBM_FRU_FEATURES_PCIE_RESET_CHANGE_COUNT_Pos     (3U)
#define MTB_UBM_FRU_FEATURES_PCIE_RESET_CHANGE_COUNT_Msk     (0x08U)
#define MTB_UBM_FRU_FEATURES_DRIVE_TYPE_CHANGE_COUNT_Pos     (4U)
#define MTB_UBM_FRU_FEATURES_DRIVE_TYPE_CHANGE_COUNT_Msk     (0x10U)
#define MTB_UBM_FRU_FEATURES_OP_STATE_CHANGE_COUNT_Pos       (5U)
#define MTB_UBM_FRU_FEATURES_OP_STATE_CHANGE_COUNT_Msk       (0x20U)
#define MTB_UBM_FRU_FEATURES_PERST_OVERRIDE_Pos              (6U)
#define MTB_UBM_FRU_FEATURES_PERST_OVERRIDE_Msk              (0xC0U)
#define MTB_UBM_FRU_FEATURES_SMBUS_RESET_Pos                 (0U)
#define MTB_UBM_FRU_FEATURES_SMBUS_RESET_Msk                 (0x01U)


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_format_version
****************************************************************************//**
*
*  Sets the FRU value of the common header format version.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_format_version(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H0_ADDR];

        bit_val = bit_val & (~MTB_UBM_FORMAT_VERSION_Msk);
        bit_val |= ((*value << MTB_UBM_FORMAT_VERSION_Pos) & MTB_UBM_FORMAT_VERSION_Msk);
            
        array[MTB_UBM_FRU_CH_H0_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_reserved
****************************************************************************//**
*
*  Sets the FRU value of the common header reserved field.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_reserved(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H0_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_CH_H0_RESERVED_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_CH_H0_RESERVED_Pos) & MTB_UBM_FRU_CH_H0_RESERVED_Msk);

        array[MTB_UBM_FRU_CH_H0_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_internal
****************************************************************************//**
*
*  Sets the FRU value of the common header internal area starting offset.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_internal(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
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
*  Sets the FRU value of the common header chasis info starting offset.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_chasis_info(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
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
*  Sets the FRU value of the common header chasis info field.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_board_info(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
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
*  Sets the FRU value of the common header product info field.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_product_info(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
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
*  Sets the FRU value of the common header multirecord offset field.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_multirecord_offset(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_CH_H5_ADDR];

        bit_val |= *value & MTB_UBM_FRU_CH_H5_MUTRECORD_OFFSET_Msk;

        array[MTB_UBM_FRU_CH_H5_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_get_ch_multirecord_offset
****************************************************************************//**
*
*  Gets the FRU value of the common header multirecord offset field.
*
* \param value
* The pointer to the memory to be written with the value.
*
* \param ubm_context
* The pointer to the UBM context structure.
*
* \return
* Boolean value representing status of EEPROM operation.
*
*******************************************************************************/
bool mtb_ubm_fru_get_ch_multirecord_offset(uint8_t *value, mtb_stc_ubm_context_t* ubm_context)
{
    bool ret_val = false;

    if (ubm_context != NULL)
    {
        cy_en_em_eeprom_status_t status = CY_EM_EEPROM_WRITE_FAIL;
        uint8_t bit_val;
        
        status = Cy_Em_EEPROM_Read(MTB_UBM_FRU_CH_H5_ADDR, &bit_val,
            MTB_UBM_FRU_CH_H5_LEN, &ubm_context->fru_context);

        *value = bit_val & MTB_UBM_FRU_CH_H5_MUTRECORD_OFFSET_Msk;
        
        if (CY_EM_EEPROM_BAD_CHECKSUM != status)
        {
            ret_val = true;
        }
    }

    return ret_val;
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ch_padding
****************************************************************************//**
*
*  Sets the FRU value of the common header padding field.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_padding(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
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
*  Sets the FRU value of the common header multirecord offset field.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ch_checksum(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
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
* \param ctrl_context
*  The pointer to the UBM controller context structure.
*
*******************************************************************************/
void mtb_ubm_fru_handle_request(mtb_stc_ubm_context_t* ubm_context, mtb_stc_ubm_controller_t* ctrl_context)
{
    uint8_t size = (MTB_UBM_FRU_SIZE - 1U) - ctrl_context->i2c.write_buffer[0];
    
    if (ctrl_context->i2c.read_request)
    {
        Cy_Em_EEPROM_Read(ctrl_context->i2c.write_buffer[0], &ctrl_context->i2c.read_buffer, size, &ubm_context->fru_context);
        ctrl_context->i2c.read_data_length = size;
    }
   
    return;
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
void mtb_ubm_fru_init_common_header(uint8_t *ram_array)
{
    uint8_t constant_value;
    
    constant_value = 0x01U;
    mtb_ubm_fru_set_ch_format_version(&constant_value, ram_array);

    constant_value = 0x00U;
    mtb_ubm_fru_set_ch_reserved(&constant_value, ram_array);
    mtb_ubm_fru_set_ch_internal(&constant_value, ram_array);
    mtb_ubm_fru_set_ch_chasis_info(&constant_value, ram_array);
    mtb_ubm_fru_set_ch_board_info(&constant_value, ram_array);
    mtb_ubm_fru_set_ch_product_info(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_CH_LEN;
    mtb_ubm_fru_set_ch_multirecord_offset(&constant_value, ram_array);

    constant_value = 0x00U;
    mtb_ubm_fru_set_ch_padding(&constant_value, ram_array);

    constant_value = mtb_ubm_fru_calculate_checksum(ram_array, (MTB_UBM_FRU_CH_LEN - 1));
    mtb_ubm_fru_set_ch_checksum(&constant_value, ram_array);
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_record_type
****************************************************************************//**
*
*  Sets the FRU value of the overview area multirecord offset field.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_record_type(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = *value;
        bit_val = bit_val & (~MTB_UBM_FRU_OA_H0_REC_T_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_H0_REC_T_Pos) & MTB_UBM_FRU_OA_H0_REC_T_Msk);

        array[MTB_UBM_FRU_OA_H0_ADDR] = bit_val;
    }
}

/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_record_fmt
****************************************************************************//**
*
*  Sets the FRU value of the overview area record format.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_record_fmt(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_H1_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_H1_FORM_V_Msk);
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
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_eol_flag(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_H1_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_H1_EOLST_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_H1_EOLST_Pos) & MTB_UBM_FRU_OA_H1_EOLST_Msk);
            
        array[MTB_UBM_FRU_OA_H1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_record_length
****************************************************************************//**
*
*  Sets the FRU value of the overview area record length.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_record_length(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_H2_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_H2_REC_LEN_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_H2_REC_LEN_Pos) & MTB_UBM_FRU_OA_H2_REC_LEN_Msk);
            
        array[MTB_UBM_FRU_OA_H2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_record_checksum
****************************************************************************//**
*
*  Sets the FRU value of the overview area record checksum.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_record_checksum(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_H3_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_H3_REC_CHKS_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_H3_REC_CHKS_Pos) & MTB_UBM_FRU_OA_H3_REC_CHKS_Msk);
            
        array[MTB_UBM_FRU_OA_H3_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_header_checksum
****************************************************************************//**
*
*  Sets the FRU value of the overview area header checksum.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_header_checksum(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_H4_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_H4_HDR_CHKS_Mks);
        bit_val |= ((*value << MTB_UBM_FRU_OA_H4_HDR_CHKS_Pos) & MTB_UBM_FRU_OA_H4_HDR_CHKS_Mks);
            
        array[MTB_UBM_FRU_OA_H4_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_spec_version
****************************************************************************//**
*
*  Sets the FRU value of the overview area data spec version.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_spec_version(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D0_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D0_SPEC_VER_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D0_SPEC_VER_Pos) & MTB_UBM_FRU_OA_D0_SPEC_VER_Msk);
            
        array[MTB_UBM_FRU_OA_D0_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_device_arrangement
****************************************************************************//**
*
*  Sets the FRU value of the overview area data mux device arrangement.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_device_arrangement(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D1_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D1_2W_DEV_ARR_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D1_2W_DEV_ARR_Pos) & MTB_UBM_FRU_OA_D1_2W_DEV_ARR_Msk);
            
        array[MTB_UBM_FRU_OA_D1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_mux_address
****************************************************************************//**
*
*  Sets the FRU value of the overview area data mux device address.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_mux_address(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D1_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D1_2W_MUX_ADDR_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D1_2W_MUX_ADDR_Pos) & MTB_UBM_FRU_OA_D1_2W_MUX_ADDR_Msk);
            
        array[MTB_UBM_FRU_OA_D1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_mux_byte_count
****************************************************************************//**
*
*  Sets the FRU value of the overview area data mux byte count.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_mux_byte_count(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D1_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D1_2W_MUX_BC_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D1_2W_MUX_BC_Pos) & MTB_UBM_FRU_OA_D1_2W_MUX_BC_Msk);
            
        array[MTB_UBM_FRU_OA_D1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_fru_invalid
****************************************************************************//**
*
*  Sets the FRU value of the overview area data FRU invalid flag.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_fru_invalid(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D2_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D2_FRU_INV_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D2_FRU_INV_Pos) & MTB_UBM_FRU_OA_D2_FRU_INV_Msk);
            
        array[MTB_UBM_FRU_OA_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_controller_max_time_limit
****************************************************************************//**
*
*  Sets the FRU value of the overview area data controller maximum time limit.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_controller_max_time_limit(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D2_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D2_C_MAX_T_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D2_C_MAX_T_Pos) & MTB_UBM_FRU_OA_D2_C_MAX_T_Msk);
            
        array[MTB_UBM_FRU_OA_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_controller_features
****************************************************************************//**
*
*  Sets the FRU value of the overview area data controller features.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_controller_features(mtb_stc_ubm_features_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        array[MTB_UBM_FRU_OA_D3_4_ADDR] =
            ((uint8_t)value->read_checksum_creation << MTB_UBM_FRU_FEATURES_READ_CHECKSUM_Pos) & MTB_UBM_FRU_FEATURES_READ_CHECKSUM_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            ((uint8_t)value->write_checksum_checking << MTB_UBM_FRU_FEATURES_WRITE_CHECKSUM_Pos) & MTB_UBM_FRU_FEATURES_WRITE_CHECKSUM_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            ((uint8_t)value->cprsnt_legacy_mode << MTB_UBM_FRU_FEATURES_CPRSNT_MODE_Pos) & MTB_UBM_FRU_FEATURES_CPRSNT_MODE_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            ((uint8_t)value->pcie_reset_change_count_mask << MTB_UBM_FRU_FEATURES_PCIE_RESET_CHANGE_COUNT_Pos) & MTB_UBM_FRU_FEATURES_PCIE_RESET_CHANGE_COUNT_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            ((uint8_t)value->drive_type_installed_change_count_mask << MTB_UBM_FRU_FEATURES_DRIVE_TYPE_CHANGE_COUNT_Pos) & MTB_UBM_FRU_FEATURES_DRIVE_TYPE_CHANGE_COUNT_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            ((uint8_t)value->operational_state_change_count_mask << MTB_UBM_FRU_FEATURES_OP_STATE_CHANGE_COUNT_Pos) & MTB_UBM_FRU_FEATURES_OP_STATE_CHANGE_COUNT_Msk;
        array[MTB_UBM_FRU_OA_D3_4_ADDR] |=
            (value->perst_management_override << MTB_UBM_FRU_FEATURES_PERST_OVERRIDE_Pos) & MTB_UBM_FRU_FEATURES_PERST_OVERRIDE_Msk;

        array[MTB_UBM_FRU_OA_D3_4_ADDR + 1U] =
            (value->smbus_reset_control << MTB_UBM_FRU_FEATURES_SMBUS_RESET_Pos) & MTB_UBM_FRU_FEATURES_SMBUS_RESET_Msk;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_get_oa_data_controller_features
****************************************************************************//**
*
*  Gets the FRU value of the overview area data controller features.
*
* \param value
* The pointer to the value to be stored.
*
* \param ubm_context
* The pointer to UBM context
*
*******************************************************************************/
bool mtb_ubm_fru_get_oa_data_controller_features(uint16_t *value, mtb_stc_ubm_context_t* ubm_context)
{
    bool ret_val = false;

    if (ubm_context != NULL)
    {
        cy_en_em_eeprom_status_t status = CY_EM_EEPROM_WRITE_FAIL;
        uint16_t bit_val = 0U;

        status = Cy_Em_EEPROM_Read(MTB_UBM_FRU_OA_ADDR + MTB_UBM_FRU_OA_D3_4_ADDR, &bit_val,
                MTB_UBM_FRU_OA_D3_4_LEN, &ubm_context->fru_context);

        if ((CY_EM_EEPROM_SUCCESS == status) || (CY_EM_EEPROM_REDUNDANT_COPY_USED == status))
        {
            *value = bit_val;
            ret_val = true;
        }
    }

    return ret_val;
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_number_control_status_descriptors
****************************************************************************//**
*
*  Sets the FRU value of the overview area data number of the control and status
*  descriptors.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_number_control_status_descriptors(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D5_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D5_STS_CNTRL_DESC_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D5_STS_CNTRL_DESC_Pos) & MTB_UBM_FRU_OA_D5_STS_CNTRL_DESC_Msk);
            
        array[MTB_UBM_FRU_OA_D5_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_get_oa_data_number_control_status_descriptors
****************************************************************************//**
*
*  Gets the FRU value of the overview area data number of the control and status
*  descriptors.
*
* \param value
* The pointer to the area value to be stored.
*
* \param ubm_context
* The pointer to the UBM context structure.
*
* \return
* Boolean value representing status of EEPROM operation.
*
*******************************************************************************/
bool mtb_ubm_fru_get_oa_data_number_control_status_descriptors(uint8_t *value, mtb_stc_ubm_context_t* ubm_context)
{
    bool ret_val = false;

    if (ubm_context != NULL)
    {
        cy_en_em_eeprom_status_t status = CY_EM_EEPROM_WRITE_FAIL;
        uint8_t bit_val = 0U;
        uint8_t offset = 0U;

        if (true == mtb_ubm_fru_get_ch_multirecord_offset(&offset, ubm_context))
        {
            status = Cy_Em_EEPROM_Read((offset + MTB_UBM_FRU_OA_ADDR + MTB_UBM_FRU_OA_D5_ADDR), &bit_val,
                MTB_UBM_FRU_OA_D5_LEN, &ubm_context->fru_context);

            if ((CY_EM_EEPROM_SUCCESS == status) || (CY_EM_EEPROM_REDUNDANT_COPY_USED == status))
            {
                bit_val = bit_val & MTB_UBM_FRU_OA_D5_STS_CNTRL_DESC_Msk;
                *value = bit_val >> MTB_UBM_FRU_OA_D5_STS_CNTRL_DESC_Pos;
            }

            if (CY_EM_EEPROM_WRITE_FAIL != status)
            {
                ret_val = true;
            }
        }
    }

    return ret_val;
}



/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_number_port_route_descriptors
****************************************************************************//**
*
*  Sets the FRU value of the overview area data number of the port route
*  descriptors.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_number_port_route_descriptors(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D6_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D6_ROUT_DESC_NUM_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D6_ROUT_DESC_NUM_Pos) & MTB_UBM_FRU_OA_D6_ROUT_DESC_NUM_Msk);
            
        array[MTB_UBM_FRU_OA_D6_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_number_backplane_dfc
****************************************************************************//**
*
*  Sets the FRU value of the overview area data number of the backplane DFCs.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_number_backplane_dfc(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D7_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D7_BP_DFC_NUM_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D7_BP_DFC_NUM_Pos) & MTB_UBM_FRU_OA_D7_BP_DFC_NUM_Msk);
            
        array[MTB_UBM_FRU_OA_D7_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_maximum_power_per_dfc
****************************************************************************//**
*
*  Sets the FRU value of the overview area data maximum power per DFCs.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_maximum_power_per_dfc(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D8_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D8_POW_PER_DFC_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D8_POW_PER_DFC_Pos) & MTB_UBM_FRU_OA_D8_POW_PER_DFC_Msk);
            
        array[MTB_UBM_FRU_OA_D8_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_mux_channel_count
****************************************************************************//**
*
*  Sets the FRU value of the overview area data mux channel count.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_mux_channel_count(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D9_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D9_MUX_CH_CNT_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D9_MUX_CH_CNT_Pos) & MTB_UBM_FRU_OA_D9_MUX_CH_CNT_Msk);
            
        array[MTB_UBM_FRU_OA_D9_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_enable_bit_location
****************************************************************************//**
*
*  Sets the FRU value of the overview area data mux channel count.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_enable_bit_location(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D9_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D9_EN_LOC_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D9_EN_LOC_Pos) & MTB_UBM_FRU_OA_D9_EN_LOC_Msk);
            
        array[MTB_UBM_FRU_OA_D9_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_mux_type
****************************************************************************//**
*
*  Sets the FRU value of the overview area data mux type.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_mux_type(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D9_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D9_MUX_TYPE_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D9_MUX_TYPE_Pos) & MTB_UBM_FRU_OA_D9_MUX_TYPE_Msk);
            
        array[MTB_UBM_FRU_OA_D9_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_data_valid
****************************************************************************//**
*
*  Sets the FRU value of the overview area validity bit.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_oa_data_valid(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_OA_D9_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_D9_VALID_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_D9_VALID_Pos) & MTB_UBM_FRU_OA_D9_VALID_Msk);
            
        array[MTB_UBM_FRU_OA_D9_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_init_overview_area
****************************************************************************//**
*
*  Initializes overview area of the FRU.
*
* \param config
*  The pointer to the UBM configuration structure.
*
* \param ram_array
*  The pointer to the FRU image section in the RAM.
*
*******************************************************************************/
void mtb_ubm_fru_init_overview_area(const mtb_stc_ubm_backplane_cfg_t* config, uint8_t *ram_array)
{
    uint8_t constant_value;

    constant_value = MTB_UBM_FRU_OA_RECORD_TYPE;
    mtb_ubm_fru_set_oa_record_type(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_OA_RECORD_FMT;
    mtb_ubm_fru_set_oa_record_fmt(&constant_value, ram_array);

    constant_value = 0x00U;
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

    /* This Beta release does not support the DFC Status and Control Descriptors. The feature will be implemented in next releases. */
    constant_value = 0x04U;
    mtb_ubm_fru_set_oa_data_number_control_status_descriptors(&constant_value, ram_array);

    /* This Beta release does not support the Port Route Descriptors. The feature will be implemented in next releases. */
    constant_value = 0x02U;
    mtb_ubm_fru_set_oa_data_number_port_route_descriptors(&constant_value, ram_array);
    
    constant_value = MTB_UBM_DFC_NUM;
    mtb_ubm_fru_set_oa_data_number_backplane_dfc(&constant_value, ram_array);
    
    mtb_ubm_fru_set_oa_data_maximum_power_per_dfc(&config->overview_area->maximum_power_per_dfc, ram_array);
    mtb_ubm_fru_set_oa_data_mux_channel_count(&config->overview_area->mux_channel_count, ram_array);
    mtb_ubm_fru_set_oa_data_enable_bit_location(&config->overview_area->enable_bit_location, ram_array);
    mtb_ubm_fru_set_oa_data_mux_type(&config->overview_area->mux_type, ram_array);
    
    constant_value = MTB_UBM_FRU_OA_VALID;
    mtb_ubm_fru_set_oa_data_valid(&constant_value, ram_array);

    constant_value = mtb_ubm_fru_calculate_checksum(&ram_array[MTB_UBM_FRU_OA_H_LEN], (MTB_UBM_FRU_OA_LEN - MTB_UBM_FRU_OA_H_LEN));
    mtb_ubm_fru_set_oa_record_checksum(&constant_value, ram_array);

    constant_value = mtb_ubm_fru_calculate_checksum(ram_array, (MTB_UBM_FRU_OA_H_LEN - MTB_UBM_FRU_OA_CHEKSUM_LEN));
    mtb_ubm_fru_set_oa_header_checksum(&constant_value, ram_array);
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_record_type
****************************************************************************//**
*
*  Sets the FRU value of the route information header record type
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_record_type(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_RI_H0_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_H0_REC_T_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_H0_REC_T_Pos) & MTB_UBM_FRU_RI_H0_REC_T_Msk);
            
        array[MTB_UBM_FRU_RI_H0_ADDR] = bit_val;
            
    }
}

/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_record_fmt
****************************************************************************//**
*
*  Sets the FRU value of the route information header record format.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_record_fmt(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_RI_H1_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_H1_FORM_V_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_H1_FORM_V_Pos) & MTB_UBM_FRU_RI_H1_FORM_V_Msk);
            
        array[MTB_UBM_FRU_RI_H1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_oa_eol_flag
****************************************************************************//**
*
*  Sets the FRU value of the route information header end of the list flag.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_eol_flag(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_RI_H1_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_H1_EOLST_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_H1_EOLST_Pos) & MTB_UBM_FRU_OA_H1_EOLST_Msk);
            
        array[MTB_UBM_FRU_RI_H1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_record_length
****************************************************************************//**
*
*  Sets the FRU value of the route information header record length.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_record_length(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_RI_H2_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_H2_REC_LEN_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_H2_REC_LEN_Pos) & MTB_UBM_FRU_OA_H2_REC_LEN_Msk);
            
        array[MTB_UBM_FRU_RI_H2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_record_checksum
****************************************************************************//**
*
*  Sets the FRU value of the route information header record checksum.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_record_checksum(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_RI_H3_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_H3_REC_CHKS_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_OA_H3_REC_CHKS_Pos) & MTB_UBM_FRU_OA_H3_REC_CHKS_Msk);
            
        array[MTB_UBM_FRU_RI_H3_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_header_checksum
****************************************************************************//**
*
*  Sets the FRU value of the route information header checksum.
*
* \param value
* The pointer to the value to be set.
*
* \param array
* The pointer to RAM array.
*
*******************************************************************************/
static void mtb_ubm_fru_set_ri_header_checksum(uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[MTB_UBM_FRU_RI_H4_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_OA_H4_HDR_CHKS_Mks);
        bit_val |= ((*value << MTB_UBM_FRU_OA_H4_HDR_CHKS_Pos) & MTB_UBM_FRU_OA_H4_HDR_CHKS_Mks);
            
        array[MTB_UBM_FRU_RI_H4_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_init_route_information_header
****************************************************************************//**
*
*  Initializes overview area of the FRU.
*
* \param routeNumber
*  Total number of routes.
*
* \param ram_array
*  The pointer to the FRU image section in the RAM.
*
*******************************************************************************/
void mtb_ubm_fru_init_route_information_header(uint8_t routeNumber, uint8_t *ram_array)
{
    uint8_t constant_value;

    constant_value = MTB_UBM_FRU_RI_RECORD_TYPE;
    mtb_ubm_fru_set_ri_record_type(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_RI_RECORD_FMT;
    mtb_ubm_fru_set_ri_record_fmt(&constant_value, ram_array);

    constant_value = MTB_UBM_FRU_RI_EOL_FLAG;
    mtb_ubm_fru_set_ri_eol_flag(&constant_value, ram_array);

    constant_value = routeNumber * MTB_UBM_FRU_RI_D_LEN;
    mtb_ubm_fru_set_ri_record_length(&constant_value, ram_array);

    constant_value = mtb_ubm_fru_calculate_checksum(&ram_array[MTB_UBM_FRU_RI_H_LEN], (routeNumber * MTB_UBM_FRU_RI_D_LEN));
    mtb_ubm_fru_set_ri_record_checksum(&constant_value, ram_array);

    constant_value = mtb_ubm_fru_calculate_checksum(&ram_array[0U], (MTB_UBM_FRU_RI_H_LEN - 1U));
    mtb_ubm_fru_set_ri_header_checksum(&constant_value, ram_array);
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_ctrl_type
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM controller type.
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
static void mtb_ubm_fru_set_ri_ctrl_type(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D0_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D0_UBM_TYPE_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D0_UBM_TYPE_Pos) & MTB_UBM_FRU_RI_D0_UBM_TYPE_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D0_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_two_wire_address
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM two wire address.
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
static void mtb_ubm_fru_set_ri_two_wire_address(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D0_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D0_2W_S_ADDR_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D0_2W_S_ADDR_Pos) & MTB_UBM_FRU_RI_D0_2W_S_ADDR_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D0_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_get_ri_two_wire_address
****************************************************************************//**
*
*  Gets the FRU value of the route information UBM two wire address.
*
* \param route
* Route number.
*
* \param value
* The pointer to the area value to be storedt.
*
* \param ubm_context
* The pointer to the UBM context structure.
*
* \return
* Boolean value representing status of EEPROM operation.
*
*******************************************************************************/
bool mtb_ubm_fru_get_ri_two_wire_address(uint8_t route, uint8_t *value, mtb_stc_ubm_context_t* ubm_context)
{
    bool ret_val = false;

    if (ubm_context != NULL)
    {
        cy_en_em_eeprom_status_t status = CY_EM_EEPROM_WRITE_FAIL;
        uint8_t bit_val = 0U;
        uint8_t offset = 0U;

        if (true == mtb_ubm_fru_get_ch_multirecord_offset(&offset, ubm_context))
        {
            status = Cy_Em_EEPROM_Read((offset + route*6 + MTB_UBM_FRU_RI_D_ADDR + MTB_UBM_FRU_RI_D0_ADDR), &bit_val,
                MTB_UBM_FRU_RI_D0_LEN, &ubm_context->fru_context);

            if ((CY_EM_EEPROM_SUCCESS == status) || (CY_EM_EEPROM_REDUNDANT_COPY_USED == status))
            {
                bit_val = bit_val & MTB_UBM_FRU_RI_D0_2W_S_ADDR_Msk;
                *value = bit_val >> MTB_UBM_FRU_RI_D0_2W_S_ADDR_Pos;
                                
                status = Cy_Em_EEPROM_Write(((offset + route*6 + MTB_UBM_FRU_RI_D_ADDR + MTB_UBM_FRU_RI_D0_ADDR)), &bit_val,
                        MTB_UBM_FRU_RI_D0_LEN, &ubm_context->fru_context);
            }

            if (CY_EM_EEPROM_WRITE_FAIL != status)
            {
                ret_val = true;
            }
        }
    }

    return ret_val;
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_sts_ctrl
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM two wire address.
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
static void mtb_ubm_fru_set_ri_sts_ctrl(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D1_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D1_STS_DI_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D1_STS_DI_Pos) & MTB_UBM_FRU_RI_D1_STS_DI_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D1_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_sff_ta_1001
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM SFF-TA-1001 support bit.
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
static void mtb_ubm_fru_set_ri_sff_ta_1001(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D2_SFF_TA_1001_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D2_SFF_TA_1001_Pos) & MTB_UBM_FRU_RI_D2_SFF_TA_1001_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_gen_z
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM Gen-Z support bit.
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
static void mtb_ubm_fru_set_ri_gen_z(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D2_GEN_Z_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D2_GEN_Z_Pos) & MTB_UBM_FRU_RI_D2_GEN_Z_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_sas_sata
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM SAS/SATA support bit.
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
static void mtb_ubm_fru_set_ri_sas_sata(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D2_SAS_SATA_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D2_SAS_SATA_Pos) & MTB_UBM_FRU_RI_D2_SAS_SATA_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_quad_pcie
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM Quad PCIe support bit.
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
static void mtb_ubm_fru_set_ri_quad_pcie(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D2_QPCIE_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D2_QPCIE_Pos) & MTB_UBM_FRU_RI_D2_QPCIE_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_dfc_empty
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM DFC empty bit.
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
static void mtb_ubm_fru_set_ri_dfc_empty(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D2_DFC_EMPTY_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D2_DFC_EMPTY_Pos) & MTB_UBM_FRU_RI_D2_DFC_EMPTY_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D2_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_link_width
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM link width.
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
static void mtb_ubm_fru_set_ri_link_width(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D3_LINK_WIDTH_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D3_LINK_WIDTH_Pos) & MTB_UBM_FRU_RI_D3_LINK_WIDTH_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_port_type
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM port type.
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
static void mtb_ubm_fru_set_ri_port_type(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D3_PORT_TYPE_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D3_PORT_TYPE_Pos) & MTB_UBM_FRU_RI_D3_PORT_TYPE_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_get_ri_port_type
****************************************************************************//**
*
*  Gets the FRU value of the route information UBM port type.
*
* \param route
* Route number.
*
* \param value
* The pointer to the area value to be stored.
*
* \param ubm_context
* The pointer to the UBM context structure.
*
* \return
* Boolean value representing status of EEPROM operation.
*
*******************************************************************************/
bool mtb_ubm_fru_get_ri_port_type(uint8_t route, uint8_t *value, mtb_stc_ubm_context_t* ubm_context)
{
    bool ret_val = false;

    if (ubm_context != NULL)
    {
        cy_en_em_eeprom_status_t status = CY_EM_EEPROM_WRITE_FAIL;
        uint8_t bit_val = 0U;
        uint8_t offset = 0U;

        if (true == mtb_ubm_fru_get_ch_multirecord_offset(&offset, ubm_context))
        {
            status = Cy_Em_EEPROM_Read((offset + MTB_UBM_FRU_OA_LEN +
            (route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_H_LEN + MTB_UBM_FRU_RI_D3_ADDR), &bit_val,
                MTB_UBM_FRU_RI_D3_LEN, &ubm_context->fru_context);

            if ((CY_EM_EEPROM_SUCCESS == status) || (CY_EM_EEPROM_REDUNDANT_COPY_USED == status))
            {
                bit_val = bit_val & MTB_UBM_FRU_RI_D3_PORT_TYPE_Msk;
                *value = bit_val >> MTB_UBM_FRU_RI_D3_PORT_TYPE_Pos;
            }

            if (CY_EM_EEPROM_WRITE_FAIL != status)
            {
                ret_val = true;
            }
        }
    }

    return ret_val;
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_domain
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM port domain.
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
static void mtb_ubm_fru_set_ri_domain(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D3_DOMAIN_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D3_DOMAIN_Pos) & MTB_UBM_FRU_RI_D3_DOMAIN_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D3_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_get_ri_domain
****************************************************************************//**
*
*  Gets the FRU value of the route information UBM port domain.
*
* \param route
* Route number.
*
* \param value
* The pointer to the area value to be stored.
*
* \param ubm_context
* The pointer to the UBM context structure.
*
* \return
* Boolean value representing status of EEPROM operation.
*
*******************************************************************************/
bool mtb_ubm_fru_get_ri_domain(uint8_t route, uint8_t *value, mtb_stc_ubm_context_t* ubm_context)
{
    bool ret_val = false;

    if (ubm_context != NULL)
    {
        cy_en_em_eeprom_status_t status = CY_EM_EEPROM_WRITE_FAIL;
        uint8_t bit_val = 0U;
        uint8_t offset = 0U;

        if (true == mtb_ubm_fru_get_ch_multirecord_offset(&offset, ubm_context))
        {
            status = Cy_Em_EEPROM_Read((offset + MTB_UBM_FRU_OA_LEN +
            (route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_H_LEN + MTB_UBM_FRU_RI_D3_ADDR), &bit_val,
                MTB_UBM_FRU_RI_D3_LEN, &ubm_context->fru_context);

            if ((CY_EM_EEPROM_SUCCESS == status) || (CY_EM_EEPROM_REDUNDANT_COPY_USED == status))
            {
                bit_val = bit_val & MTB_UBM_FRU_RI_D3_DOMAIN_Msk;
                *value = bit_val >> MTB_UBM_FRU_RI_D3_DOMAIN_Pos;
            }

            if (CY_EM_EEPROM_WRITE_FAIL != status)
            {
                ret_val = true;
            }
        }
    }

    return ret_val;
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_max_sata_rate
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM port maximum SATA rate.
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
static void mtb_ubm_fru_set_ri_max_sata_rate(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D4_MAX_SATA_LINK_RATE_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D4_MAX_SATA_LINK_RATE_Pos) & MTB_UBM_FRU_RI_D4_MAX_SATA_LINK_RATE_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_max_pcie_rate
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM port maximum PCIe rate.
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
static void mtb_ubm_fru_set_ri_max_pcie_rate(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D4_MAX_PCIE_LINK_RATE_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D4_MAX_PCIE_LINK_RATE_Pos) & MTB_UBM_FRU_RI_D4_MAX_PCIE_LINK_RATE_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_max_sas_rate
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM port maximum SAS rate.
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
static void mtb_ubm_fru_set_ri_max_sas_rate(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D4_MAX_SAS_LINK_RATE_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D4_MAX_SAS_LINK_RATE_Pos) & MTB_UBM_FRU_RI_D4_MAX_SAS_LINK_RATE_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D4_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_hfc_start_lane
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM port maximum SAS rate.
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
static void mtb_ubm_fru_set_ri_hfc_start_lane(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D5_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D5_HFC_STARTING_LANE_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D5_HFC_STARTING_LANE_Pos) & MTB_UBM_FRU_RI_D5_HFC_STARTING_LANE_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D5_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_hfc_identifier
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM port maximum SAS rate.
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
static void mtb_ubm_fru_set_ri_hfc_identifier(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D5_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D5_HFC_IDENTITY_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D5_HFC_IDENTITY_Pos) & MTB_UBM_FRU_RI_D5_HFC_IDENTITY_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D5_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_get_ri_hfc_identifier
****************************************************************************//**
*
*  Gets the FRU value of the route information UBM port maximum SAS rate.
*
* \param route
* Route number.
*
* \param value
* The pointer to the area value to be stored.
*
* \param ubm_context
* The pointer to the UBM context structure.
*
* \return
* Boolean value representing status of EEPROM operation.
*
*******************************************************************************/
bool mtb_ubm_fru_get_ri_hfc_identifier(uint8_t route, uint8_t *value, mtb_stc_ubm_context_t* ubm_context)
{
    bool ret_val = false;

    if (ubm_context != NULL)
    {
        cy_en_em_eeprom_status_t status = CY_EM_EEPROM_WRITE_FAIL;
        uint8_t bit_val = 0U;
        uint8_t offset = 0U;

        if (true == mtb_ubm_fru_get_ch_multirecord_offset(&offset, ubm_context))
        {
            status = Cy_Em_EEPROM_Read((offset + MTB_UBM_FRU_OA_LEN +
            (route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_H_LEN + MTB_UBM_FRU_RI_D5_ADDR), &bit_val,
                MTB_UBM_FRU_RI_D5_LEN, &ubm_context->fru_context);

            if ((CY_EM_EEPROM_SUCCESS == status) || (CY_EM_EEPROM_REDUNDANT_COPY_USED == status))
            {
                bit_val = bit_val & MTB_UBM_FRU_RI_D5_HFC_IDENTITY_Msk;
                *value = bit_val >> MTB_UBM_FRU_RI_D5_HFC_IDENTITY_Pos;
            }

            if (CY_EM_EEPROM_WRITE_FAIL != status)
            {
                ret_val = true;
            }
        }
    }

    return ret_val;
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_set_ri_slot_offset
****************************************************************************//**
*
*  Sets the FRU value of the route information UBM port slot offset.
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
static void mtb_ubm_fru_set_ri_slot_offset(uint8_t route, const uint8_t *value, uint8_t *array)
{
    if ((array != NULL) && (value != NULL))
    {
        uint8_t bit_val = array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D6_ADDR];

        bit_val = bit_val & (~MTB_UBM_FRU_RI_D6_SLOT_OFFSET_Msk);
        bit_val |= ((*value << MTB_UBM_FRU_RI_D6_SLOT_OFFSET_Pos) & MTB_UBM_FRU_RI_D6_SLOT_OFFSET_Msk);
            
        array[(route * MTB_UBM_FRU_RI_D_LEN) + MTB_UBM_FRU_RI_D6_ADDR] = bit_val;
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_init_route_information_descriptor
****************************************************************************//**
*
*  Initializes the route information data of the FRU.
*
* \param num_of_routes
*  The number of routes to initialise.
*
* \param routing_info
*  The pointer to the UBM context settings.
*
* \param ram_array
*  The pointer to the FRU image section in the RAM.
*
*******************************************************************************/
void mtb_ubm_fru_init_route_information_descriptor(uint8_t num_of_routes, const mtb_stc_ubm_routing_t *routing_info, uint8_t *ram_array)
{
    for (int8_t i = num_of_routes - 1U; i >= 0; i--)
    {
        mtb_ubm_fru_set_ri_ctrl_type(i, ((uint8_t *)&routing_info[i].ubm_ctrl_type), ram_array);
        mtb_ubm_fru_set_ri_two_wire_address(i, &routing_info[i].ubm_ctrl_slave_addr, ram_array);
        mtb_ubm_fru_set_ri_sts_ctrl(i, &routing_info[i].dfc_status_and_control, ram_array);
        mtb_ubm_fru_set_ri_sff_ta_1001(i, &routing_info[i].sff_ta_1001_support, ram_array);
        mtb_ubm_fru_set_ri_gen_z(i, &routing_info[i].gen_z_support, ram_array);
        mtb_ubm_fru_set_ri_sas_sata(i, &routing_info[i].sas_sata_support, ram_array);
        mtb_ubm_fru_set_ri_quad_pcie(i, &routing_info[i].quad_pcie_support, ram_array);
        mtb_ubm_fru_set_ri_dfc_empty(i, &routing_info[i].dfc_empty, ram_array);
        mtb_ubm_fru_set_ri_link_width(i, ((uint8_t *)&routing_info[i].drive_link_width), ram_array);
        mtb_ubm_fru_set_ri_port_type(i, ((uint8_t *)&routing_info[i].port_type), ram_array);
        mtb_ubm_fru_set_ri_domain(i, ((uint8_t *)&routing_info[i].domain), ram_array);
        mtb_ubm_fru_set_ri_max_sata_rate(i, ((uint8_t *)&routing_info[i].max_sata_rate), ram_array);
        mtb_ubm_fru_set_ri_max_pcie_rate(i, ((uint8_t *)&routing_info[i].max_pcie_rate), ram_array);
        mtb_ubm_fru_set_ri_max_sas_rate(i, ((uint8_t *)&routing_info[i].max_sas_rate), ram_array);
        mtb_ubm_fru_set_ri_hfc_start_lane(i, &routing_info[i].hfc_starting_phy_lane, ram_array);
        mtb_ubm_fru_set_ri_hfc_identifier(i, &routing_info[i].hfc_identifier, ram_array);
        mtb_ubm_fru_set_ri_slot_offset(i,&routing_info[i].slot_offset, ram_array);
    }
}


/*******************************************************************************
* Function Name: mtb_ubm_fru_calculate_checksum
****************************************************************************//**
*
*  Calculates checksum
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
uint8_t mtb_ubm_fru_calculate_checksum(const uint8_t* data_bytes, uint32_t data_length)
{
    uint8_t checksum = 0x00U;
    uint8_t cnt = 0x00U;
    
    for(uint8_t i=0U; i < data_length; i++)
    {
        cnt = cnt + data_bytes[i];
    }

    checksum = ~(cnt) + 0x01U;

    return(checksum);
}

