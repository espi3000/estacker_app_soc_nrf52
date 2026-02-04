
#ifndef _TOF_H_
#define _TOF_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h> // I2C
#include <zephyr/sys/ring_buffer.h>

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
 * @brief Initialize the Time-of-Flight sensor
 * 
 * Powers on the sensor, enables I2C communication, and configures the sensor
 * for interrupt-driven distance measurements based on predefined parameters.
 * 
 * @retval 0 on success
 * @retval non-zero on failure
 */
int tof_init(void);

/**
 * @brief Powers on the Time-of-Flight sensor
 * 
 */
void tof_power_on(void);

/**
 * @brief Powers off the Time-of-Flight sensor
 * 
 */
void tof_power_off(void);

/**
 * @brief Enables I2C communication with the Time-of-Flight sensor
 * 
 */
void tof_i2c_enable(void);

/**
 * @brief Disables I2C communication with the Time-of-Flight sensor
 * 
 */
void tof_i2c_disable(void);

/**
 * @brief Clears the interrupt on the Time-of-Flight sensor
 * 
 * @retval 0 on success
 * @retval non-zero on failure
 */
int tof_clear_interrupt(void);

/**
 * @brief Starts continuous distance sampling on the Time-of-Flight sensor
 * 
 * @retval 0 on success
 * @retval non-zero on failure
 */
int tof_start_sampling(void);

/**
 * @brief Stops continuous distance sampling on the Time-of-Flight sensor
 * 
 * @retval 0 on success
 * @retval non-zero on failure
 */
int tof_stop_sampling(void);

/**
 * @brief Retrieves a distance sample from the Time-of-Flight sensor and stores 
 * it in the provided ring buffer
 * 
 * @param ringbuf Pointer to the ring buffer where the sample will be stored
 * @retval 0 on success
 * @retval non-zero on failure
 */
int tof_get_sample(struct ring_buf *ringbuf);

/**
 * @brief Gets the number of samples currently stored in the ring buffer
 * 
 * @param ringbuf Pointer to the ring buffer
 * @return Number of samples in the ring buffer
 */
int tof_get_num_samples(struct ring_buf *ringbuf);

/**
 * @brief Gets the total number of samples taken since initialization
 * 
 * @return Total number of samples taken
 */
uint64_t tof_get_tot_num_samples(void);

//int tof_sample_oneshot(void);
//int process_samples(void);

#endif // _TOF_H_