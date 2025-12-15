#include "opt4060.h"

#include <stdint.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>

//******************************************************************************
//* Macros
//******************************************************************************
#ifdef OPT4060_DEBUG
    #define ASSERT(statement, message) if (statement) {printk(message); return 1;}
#else
    #define ASSERT(statement, message) statement
#endif

//******************************************************************************
//* Constants
//******************************************************************************
extern const struct i2c_dt_spec dev_i2c_col;
static int32_t g_conv_time_us;
static opt4060_cfg_0_mode_t g_mode;


//******************************************************************************
//* Function definitions
//******************************************************************************
int opt4060_init(opt4060_cfg_0_conv_time_t conv_time, opt4060_cfg_0_mode_t mode) {
    g_mode = mode;
    #ifdef OPT4060_DEBUG
    opt4060_id_t id;
    const uint8_t ID_ADDR = OPT4060_ID;
    ASSERT(i2c_write_dt(&dev_i2c_col, &ID_ADDR, 1), 
        "opt4060_init: Failed I2C write to device. \n\r");
    ASSERT(i2c_read_dt(&dev_i2c_col, &id.raw, 2),
        "opt4060_init: Failed I2C read from device. \n\r");
    ASSERT(id.raw != OPT4060_ID_DEFAULT, 
        "opt4060_init: ID register mismatch. \n\r");
    #endif

    switch (conv_time) {
    case CONV_TIME_600US:
        g_conv_time_us = 600;
        break;
    case CONV_TIME_1MS:
        g_conv_time_us = 1000;
        break;
    case CONV_TIME_1M8S:
        g_conv_time_us = 1800;
        break;
    case CONV_TIME_3M4S:
        g_conv_time_us = 3400;
        break;
    case CONV_TIME_6M5S:
        g_conv_time_us = 6500;
        break;
    case CONV_TIME_12M7S:
        g_conv_time_us = 12700;
        break;
    case CONV_TIME_25MS:
        g_conv_time_us = 25000;
        break;
    case CONV_TIME_50MS:
        g_conv_time_us = 50000;
        break;
    case CONV_TIME_100MS:
        g_conv_time_us = 100000;
        break;
    case CONV_TIME_200MS:
        g_conv_time_us = 200000;
        break;
    case CONV_TIME_400MS:
        g_conv_time_us = 400000;
        break;
    case CONV_TIME_800MS:
        g_conv_time_us = 800000;
        break;
    default:
        g_conv_time_us = 100000;
        break;
    }
    g_conv_time_us *= 4; // 4 channel conversion
    if (mode == MODE_ONESHOT_AUTO) {
        g_conv_time_us += 500; // Additional margin
    }
    
    opt4060_cfg_1_t cfg_1 = OPT4060_CFG_1_DEFAULT;
    cfg_1.i2c_burst = true;
    cfg_1.int_cfg = INT_CFG_DATA_READY_ALL_CH;
    cfg_1.int_dir = INT_DIR_OUTPUT;

    uint8_t config[3];
    config[0] = OPT4060_CFG_1;
    config[1] = cfg_1.high_byte;
    config[2] = cfg_1.low_byte;
    ASSERT(i2c_write_dt(&dev_i2c_col, config, 3), 
        "opt4060_init: Failed I2C write to device. \n\r");
    return 0;
}

int opt4060_check_status(opt4060_status_t* status_ptr) {
    //const uint8_t STATUS_ADDR = OPT4060_STATUS;
    //ASSERT(i2c_write_dt(&dev_i2c_col, &STATUS_ADDR, 1), 
    //    "opt4060_check_status: Failed I2C write to device. \n\r");
    //ASSERT(i2c_read_dt(&dev_i2c_col, status_ptr, 2), 
    //    "opt4060_check_status: Failed I2C read from device. \n\r");
    return 0;
}

int opt4060_start_conversion(void) {
    // Trigger conversion by writing control register 0
    opt4060_cfg_0_t cfg_0 = OPT4060_CFG_0_DEFAULT;
    cfg_0.mode = g_mode;
    cfg_0.int_pol = INT_POL_ACTIVE_HIGH; // Must be active high to ensure no false triggers

    uint8_t config[3];
    config[0] = OPT4060_CFG_0;
    config[1] = cfg_0.high_byte;
    config[2] = cfg_0.low_byte;
    ASSERT(i2c_write_dt(&dev_i2c_col, config, 3), 
        "opt4060_start_conversion: Failed I2C write to device. \n\r");
    return 0;
}

int opt4060_read_channels(opt4060_data_t* data_ptr) {
    //! Might have to write the address first
    // i2c_write_dt(&dev_i2c_col, &OPT4060_CH0_0, 1);
    ASSERT(i2c_read_dt(&dev_i2c_col, data_ptr->byte, 16), 
        "OPT4060: Failed I2C read from device. \n\r");
    return 0;
}

void opt4060_post_process_data(opt4060_data_t* data_ptr) {
    uint32_t adc_codes_ch0 = data_ptr->ch0.mantissa << data_ptr->ch0.exponent;
    uint32_t adc_codes_ch1 = data_ptr->ch1.mantissa << data_ptr->ch1.exponent;
    uint32_t adc_codes_ch2 = data_ptr->ch2.mantissa << data_ptr->ch2.exponent;
    uint32_t adc_codes_ch3 = data_ptr->ch3.mantissa << data_ptr->ch3.exponent;

    float r = adc_codes_ch0*2.4;
    float g = adc_codes_ch1*1.0;
    float b = adc_codes_ch2*1.3;
    float w = adc_codes_ch3*1.0;

    float c = r + g + b; // For normalization

    data_ptr->r = r/c;
    data_ptr->g = g/c;
    data_ptr->b = b/c;
    data_ptr->w = w;

    data_ptr->lux = adc_codes_ch1*2.15e-3;
}

int opt4060_sample_oneshot(opt4060_data_t* data_ptr) {
    opt4060_start_conversion();    

    // Wait for conversion to finish
    k_usleep(g_conv_time_us);
    
    #ifdef OPT4060_DEBUG
    opt4060_status_t status;
    opt4060_check_status(&status);
    ASSERT(!status.ready, 
        "opt4060_sample_oneshot: Insufficient sleep time. \n\r");
    #endif

    opt4060_read_channels(data_ptr);
    opt4060_post_process_data(data_ptr);
    return 0;
}