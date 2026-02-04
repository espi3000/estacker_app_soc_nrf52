#ifndef _I2C_SWITCH_H_
#define _I2C_SWITCH_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

//******************************************************************************
//* Macros
//******************************************************************************

#define I2C0_I2CSwitch DT_NODELABEL(i2cswitch)
#define I2C0_TMP DT_NODELABEL(tmp)
#define I2C0_IMU DT_NODELABEL(imu)
#define I2C0_PMS DT_NODELABEL(pms)
#define I2C0_TOF DT_NODELABEL(tof)
#define I2C0_BIO DT_NODELABEL(bio)
#define I2C0_GPS DT_NODELABEL(gps)
#define I2C0_COL DT_NODELABEL(col)
#define I2C0_BPS DT_NODELABEL(bps)
#define I2C0_AIR DT_NODELABEL(air)
#define I2C0_GeneralCall DT_NODELABEL(generalcall)

#define I2C_SWITCH_TMP_MASK 0x04
#define I2C_SWITCH_IMU_MASK 0x10
#define I2C_SWITCH_PMS_MASK 0x08
#define I2C_SWITCH_TOF_MASK 0x02
#define I2C_SWITCH_BIO_MASK 0x01
#define I2C_SWITCH_GPS_MASK 0x20
#define I2C_SWITCH_COL_MASK 0x80
#define I2C_SWITCH_BPS_MASK 0x40
#define I2C_SWITCH_AIR_MASK 0x20

//******************************************************************************
//* Function prototypes
//******************************************************************************

/**
 * @brief Initialize the I2C switch
 * 
 * @return int 
 */
int i2c_switch_init(void);

/**
 * @brief Change the active channel(s) of the I2C switch
 * 
 * @param ch_mask Channel mask to select channels
 * @param state true to enable channels, false to disable channels
 * @return int 
 */
int i2c_switch_change_channel(uint8_t ch_mask, bool state);

#endif // _I2C_SWITCH_H_