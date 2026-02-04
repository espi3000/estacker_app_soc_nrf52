#ifndef _BACKUP_H_
#define _BACKUP_H_

#include <zephyr/kernel.h>
#include <zephyr/sys/ring_buffer.h>

//******************************************************************************
//* Function prototypes
//******************************************************************************

/**
 * @brief Initialize the backup system
 * 
 * Configures flash memory (NVM) to store benchmark configurations, program 
 * states and sample data.
 * 
 * @param[in] ringbuf_ptr Pointer to the ring buffer which is used for sample data
 * @retval 0 on success
 * @retval 1 on failure
 */
int backup_init(struct ring_buf *ringbuf_ptr);

/**
 * @brief Load benchmark parameters from either host or NVM
 * 
 * If the host (Raspberry Pi) signals for a parameter update, the new parameters
 * are fetched via UART. Otherwise, the parameters are loaded from NVM. In the 
 * former case, the function never returns as the system power is expected to be
 * disconnected after receiving the new parameters.
 * 
 * @retval 0 on success
 * @retval 1 on failure
 */
int backup_load_params(void);

/**
 * @brief Read sample data from NVM into the ring buffer
 * 
 * @warning Will only work if backup_init() has been called before
 * 
 * @return Number of bytes read from NVM
 */
int backup_read_from_nvm(void);

/**
 * @brief Write sample data from the ring buffer into NVM
 * 
 * @warning Will only work if backup_init() has been called before
 * 
 * @return Number of bytes written to NVM
 */
int backup_write_to_nvm(void);

#endif // BACKUP_H