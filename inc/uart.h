#ifndef _UART_H_
#define _UART_H_

//#include <stdbool.h>

//******************************************************************************
//* Function prototypes
//******************************************************************************

/**
 * @brief Initialize the UART interface
 * 
 * @return 0 if successful, negative error code otherwise
 */
int uart_setup(void);

/**
 * @brief Notify the host (Raspberry Pi) that the SoC is ready to receive parameters
 * 
 * @retval true if parameters have been received
 * @retval false if parameters have not been received
 */
bool params_updated(void);

/**
 * @brief Notify the host (Raspberry Pi) that the SoC is ready to receive parameters
 */
void notify_host_ready_for_params(void);

#endif // UART_H