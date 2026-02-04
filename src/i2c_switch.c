#include "i2c_switch.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>

//******************************************************************************
//* Global constants
//******************************************************************************

const struct i2c_dt_spec dev_i2c_i2cswitch = I2C_DT_SPEC_GET(I2C0_I2CSwitch);
const struct i2c_dt_spec dev_i2c_tmp = I2C_DT_SPEC_GET(I2C0_TMP);
const struct i2c_dt_spec dev_i2c_imu = I2C_DT_SPEC_GET(I2C0_IMU);
const struct i2c_dt_spec dev_i2c_pms = I2C_DT_SPEC_GET(I2C0_PMS);
const struct i2c_dt_spec dev_i2c_tof = I2C_DT_SPEC_GET(I2C0_TOF);
const struct i2c_dt_spec dev_i2c_bio = I2C_DT_SPEC_GET(I2C0_BIO);
const struct i2c_dt_spec dev_i2c_gps = I2C_DT_SPEC_GET(I2C0_GPS);
const struct i2c_dt_spec dev_i2c_col = I2C_DT_SPEC_GET(I2C0_COL);
const struct i2c_dt_spec dev_i2c_bps = I2C_DT_SPEC_GET(I2C0_BPS);
const struct i2c_dt_spec dev_i2c_air = I2C_DT_SPEC_GET(I2C0_AIR);
const struct i2c_dt_spec dev_i2c_generalcall = I2C_DT_SPEC_GET(I2C0_GeneralCall);

//******************************************************************************
//* Function definitions
//******************************************************************************

int i2c_switch_init(void) {
    int ret = 0;
    if (!device_is_ready(dev_i2c_i2cswitch.bus)) {
        //printf("Device %s is not ready\n", dev_i2c_i2cswitch.bus->name);
        return 1;
    }
    // Set all sensors I2C disabled
    uint8_t config = 0xFF;
    ret += i2c_write_dt(&dev_i2c_i2cswitch, &config, sizeof(config));
    return ret;
}

int i2c_switch_change_channel(uint8_t ch_mask, bool state) {
    int ret = 0;
    uint8_t config;
    ret += i2c_read_dt(&dev_i2c_i2cswitch, &config, sizeof(config));
    if (state)
        config = config | ch_mask;
    else
        config = config & (~ch_mask);
    ret += i2c_write_dt(&dev_i2c_i2cswitch, &config, sizeof(config));
    return ret;
}