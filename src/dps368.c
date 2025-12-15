#include "dps368.h"

#include <stdint.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

//******************************************************************************
//* Macros
//******************************************************************************
#ifdef DPS368_DEBUG
    #define ASSERT(statement, message) if (statement) {printk(message); return 1;}
#else
    #define ASSERT(statement, message) statement
#endif


//******************************************************************************
//* Constants
//******************************************************************************
extern const struct i2c_dt_spec dev_i2c_bps;
static dps368_coefficients_t g_coef;
static int32_t g_conv_time_ms;

const int32_t DPS368_SCALE_FACTOR[] = {
    524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960
};

//******************************************************************************
//* Function definitions
//******************************************************************************
int dps368_init(dps368_x_cfg_x_prc_t oversampling_ratio) {
    //TODO: Check if sensor is initialized
    //! Might not be necessary

    // Check if the device is present and works
    #ifdef DPS368_DEBUG
    uint8_t id;
    const uint8_t ID_ADDR = DPS368_ID;
    ASSERT(i2c_write_read_dt(&dev_i2c_bps, &ID_ADDR, 1, &id, 1),
        "dps368_init: I2C write/read failed. \r\n");
    ASSERT(id != DPS368_ID_DEFAULT,
        "dps368_init: ID mismatch. \r\n");
    #endif

    //TODO: Set conversion times
    switch (oversampling_ratio) {
    case X_PRC_1X:
        g_conv_time_ms = 4;
        break;
    case X_PRC_2X:
        g_conv_time_ms = 6;
        break;
    case X_PRC_4X:
        g_conv_time_ms = 9;
        break;
    case X_PRC_8X:
        g_conv_time_ms = 15;
        break;
    case X_PRC_16X:
        g_conv_time_ms = 28;
        break;
    case X_PRC_32X:
        g_conv_time_ms = 54;
        break;
    case X_PRC_64X:
        g_conv_time_ms = 105;
        break;
    case X_PRC_128X:
        g_conv_time_ms = 207;
        break;
    default:
        g_conv_time_ms = 4;
        break;
    }

    //TODO: Check if coefficients are ready in (MEAS_CFG/0x08)
    //! Might not be necessary

    // Set the pressure and temperature configuration registers (0x06 and 0x07)
    dps368_prs_cfg_t prs_cfg = DPS368_PRS_CFG_DEFAULT;
    prs_cfg.pm_prc = oversampling_ratio;
    dps368_tmp_cfg_t tmp_cfg = DPS368_TMP_CFG_DEFAULT;
    tmp_cfg.tmp_prc = oversampling_ratio;
    tmp_cfg.tmp_ext = TMP_EXT_EXTERNAL;


    // Set the configuration register (0x09)
    dps368_cfg_reg_t cfg_reg = DPS368_CFG_REG_DEFAULT;
    cfg_reg.tmp_int_en = false;
    cfg_reg.prs_int_en = false;
    cfg_reg.prs_shift_en = oversampling_ratio > X_PRC_8X;
    cfg_reg.tmp_shift_en = oversampling_ratio > X_PRC_8X;

    uint8_t config[6] = {
        DPS368_PRS_CFG, prs_cfg.raw,
        DPS368_TMP_CFG, tmp_cfg.raw,
        DPS368_CFG_REG, cfg_reg.raw
    };
    for (int i=0; i<3; i++){
        ASSERT(i2c_write_dt(&dev_i2c_bps, &config[i*2], 2),
        "dps368_init: I2C write failed. \r\n");
    }

    // Read the coefficient source register (0x28)
    const uint8_t COEF_SRC_ADDR = DPS368_COEF_SRCE;
    uint8_t coeff_src;
    ASSERT(i2c_write_read_dt(&dev_i2c_bps, &COEF_SRC_ADDR, 1, &coeff_src, 1),
        "dps368_check_status: I2C write/read failed. \r\n");

    // Read coefficients from 0x10 to 0x20
    uint8_t COEF_ADDR = DPS368_COEF_START;
    ASSERT(i2c_write_read_dt(&dev_i2c_bps, &COEF_ADDR, 1, &g_coef.raw, DPS368_COEF_SIZE),
        "dps368_init: I2C write/read failed. \r\n");

    return 0;
}

int dps368_check_status(dps368_meas_cfg_t *status_ptr) {
    // Read the measurement configuration register (0x08)
    const uint8_t MEAS_CFG_ADDR = DPS368_MEAS_CFG;
    ASSERT(i2c_write_read_dt(&dev_i2c_bps, &MEAS_CFG_ADDR, 1, &status_ptr->raw, 1),
        "dps368_check_status: I2C write/read failed. \r\n");
    return 0;
}

int dps368_start_tmp_conversion(void) {
    // Set the measurement configuration register (0x08)
    dps368_meas_cfg_t meas_cfg = DPS368_MEAS_CFG_DEFAULT;
    meas_cfg.meas_ctrl = MEAS_CTRL_TMP;
    uint8_t config[2] = {DPS368_MEAS_CFG, meas_cfg.raw};
    ASSERT(i2c_write_dt(&dev_i2c_bps, config, 2),
        "dps368_start_tmp_conversion: I2C write failed. \r\n");
    return 0;
}

int dps368_start_prs_conversion(void) {
    // Set the measurement configuration register (0x08)
    dps368_meas_cfg_t meas_cfg = DPS368_MEAS_CFG_DEFAULT;
    meas_cfg.meas_ctrl = MEAS_CTRL_PRS;
    uint8_t config[2] = {DPS368_MEAS_CFG, meas_cfg.raw};
    ASSERT(i2c_write_dt(&dev_i2c_bps, config, 2),
        "dps368_start_prs_conversion: I2C write failed. \r\n");
    return 0;
}

int dps368_read_tmp(dps368_data_t *data_ptr) {
    // Read the temperature data registers (0x03 to 0x05)
    const uint8_t TMP_ADDR = DPS368_TMP_B2;
    uint8_t TMP_Bytes;
    for (int i=0; i<3; i++){
        TMP_Bytes = TMP_ADDR + i;
        ASSERT(i2c_write_read_dt(&dev_i2c_bps, &TMP_Bytes, 1, &data_ptr->tmp.byte[2-i], 1),
        "dps368_read_tmp: I2C write/read failed. \r\n");
    }
    return 0;
}

int dps368_read_prs(dps368_data_t *data_ptr) {
    // Read the pressure data registers (0x00 to 0x02)
    const uint8_t PRS_ADDR = DPS368_PRS_B2;
    uint8_t PRS_Bytes;
    for (int i=0; i<3; i++){
        PRS_Bytes = PRS_ADDR + i;
        ASSERT(i2c_write_read_dt(&dev_i2c_bps, &PRS_Bytes, 1, &data_ptr->prs.byte[2-i], 1),
        "dps368_read_tmp: I2C write/read failed. \r\n");
    }
    return 0;
}

// Sign extension for calibration coefficients (used in post_process_data())
int32_t sign_extend(int32_t value, int bits) {
    if (value & (1 << (bits - 1))) {  // Check sign bit
        value -= (1 << bits);         // Sign extend
    }
    return value;
}

void dps368_post_process_data(dps368_data_t *data_ptr) {

    // Manually sign-extent the 2'c complement coefficients

    // Extracting c0 (12-bit) from 0x10[11:4] - 0x11[3:0]
    int32_t c0 = (g_coef.raw[0] << 4) | (g_coef.raw[1] >> 4);
    c0 = sign_extend(c0, 12);

    // Extracting c1 (12-bit) from 0x11[11:8] - 0x12[7:0]
    int32_t c1 = ((g_coef.raw[1] & 0x0F) << 8) | g_coef.raw[2];
    c1 = sign_extend(c1, 12);

    // Extracting c00 (20-bit) from 0x13[19:12] - 0x15[11:4] - 0x16[3:0]
    int32_t c00 = (g_coef.raw[3] << 12) | (g_coef.raw[4] << 4) | (g_coef.raw[5] >> 4);
    c00 = sign_extend(c00, 20);

    // Extracting c10 (20-bit) from 0x16[19:16] - 0x17[15:8] - 0x18[7:0]
    int32_t c10 = ((g_coef.raw[5] & 0x0F) << 16) | (g_coef.raw[6] << 8) | g_coef.raw[7];
    c10 = sign_extend(c10, 20);

    // Extracting c01 (16-bit) from 0x18[15:8] - 0x19[7:0]
    int32_t c01 = (g_coef.raw[8] << 8) | g_coef.raw[9];
    c01 = sign_extend(c01, 16);

    // Extracting c11 (16-bit) from 0x1A[15:8] - 0x1B[7:0]
    int32_t c11 = (g_coef.raw[10] << 8) | g_coef.raw[11];
    c11 = sign_extend(c11, 16);

    // Extracting c20 (16-bit) from 0x1C[15:8] - 0x1D[7:0]
    int32_t c20 = (g_coef.raw[12] << 8) | g_coef.raw[13];
    c20 = sign_extend(c20, 16);

    // Extracting c21 (16-bit) from 0x1E[15:8] - 0x1F[7:0]
    int32_t c21 = (g_coef.raw[14] << 8) | g_coef.raw[15];
    c21 = sign_extend(c21, 16);

    // Extracting c30 (16-bit) from 0x20[15:8] - 0x21[7:0]
    int32_t c30 = (g_coef.raw[16] << 8) | g_coef.raw[17];
    c30 = sign_extend(c30, 16);


    // Calculate the temperature
    float tmp_raw_sc = data_ptr->tmp.raw/(float)DPS368_SCALE_FACTOR[X_PRC_64X];
    data_ptr->tmp.comp = c0/2 + c1*tmp_raw_sc;

    // Calculate the pressure
    float prs_raw_sc = data_ptr->prs.raw/(float)DPS368_SCALE_FACTOR[X_PRC_64X];
    int32_t prs_comp = c00 
                     + prs_raw_sc*(c10 + prs_raw_sc*(c20 + prs_raw_sc*c30)) 
                     + tmp_raw_sc*c01 
                     + tmp_raw_sc*prs_raw_sc*(c11 + prs_raw_sc*c21);
    data_ptr->prs.comp = prs_comp;
}

int dps368_sample_single(dps368_data_t* data_ptr) {
    dps368_start_tmp_conversion();

    // Wait for the conversion to finish
    k_msleep(g_conv_time_ms);

    #ifdef DPS368_DEBUG
    dps368_meas_cfg_t status;
    dps368_check_status(&status);
    ASSERT(!status.tmp_ready, "dps368_sample_single: Insufficient tmp sleep time. \r\n");
    #endif

    dps368_read_tmp(data_ptr);
    dps368_start_prs_conversion();

    // Wait for the conversion to finish
    k_msleep(g_conv_time_ms);

    #ifdef DPS368_DEBUG
    dps368_check_status(&status);
    ASSERT(!status.prs_ready, 
        "dps368_sample_single: Insufficient prs sleep time. \r\n");
    #endif

    dps368_read_prs(data_ptr);
    dps368_post_process_data(data_ptr);
    return 0;
}