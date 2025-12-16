
#ifndef _TOF_H_
#define _TOF_H_

#include <zephyr/drivers/i2c.h> // I2C

//******************************************************************************
//* Macros
//******************************************************************************

#define I2C0_IOExp DT_NODELABEL(ioexpander)
#define I2C0_I2CSwitch DT_NODELABEL(i2cswitch)
#define I2C0_TOF DT_NODELABEL(tof)
#define I2C0_GeneralCall DT_NODELABEL(generalcall)

//******************************************************************************
//* Global variables
//******************************************************************************

extern uint16_t sample_data[512];           // Sample data buffer
extern uint16_t sample_data_counter;        // Sample data counter
extern uint16_t sample_packets_counter;     // Number of sample packets. A sample packet can consist of the measurements of several sensors

//******************************************************************************
//* Types
//******************************************************************************

enum i2c_state {
    I2C_OFF = 0,
    I2C_ON = 1
};

enum power_state {
    POWER_OFF = 0,
    POWER_ON = 1
};

//******************************************************************************
//* Function prototypes
//******************************************************************************

/**
 * @brief Initialize I/O expander
 * 
 * @return int 
 */
int io_expander_init(void);

/**
 * @brief I/O expander interrupt handler
 * 
 * @return int 
 */
int io_expander_irq_handler(void);

/**
 * @brief Initialize sensor board
 * 
 * @return int 
 */
int sensor_board_init(void);

/**
 * @brief Power on/off specific sensors
 * 
 * @param sensor_mask The type of sensor to power on/off
 * @param state 1 to power on, 0 to power off
 * @return int 
 */
int power_sensors(uint16_t sensor_mask, enum power_state state);

/**
 * @brief Sample data from a specific sensor
 * 
 * @param sensor_mask The type of sensor to sample
 * @return int 
 */
int sensor_sample(uint16_t sensor_mask);

/**
 * @brief Process the sampled data
 * 
 * @return int 
 */
int process_samples(void);

#endif // _TOF_H_