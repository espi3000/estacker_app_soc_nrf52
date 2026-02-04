#ifndef _STATE_LOGGER_H_
#define _STATE_LOGGER_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

//******************************************************************************
//* Types
//******************************************************************************

enum state_logger_states {
    OFF = 1000,                // Reserved for when DC/DC converter is off
    BOOTING = 0,            // Reserved for when the DC/DC converter has turned on but nRF has not reached the IDLE state yet
    SLEEP = 1,              // Unused
    IDLE = 2,
    SAMPLING = 3,
    PROCESSING = 4,
    COMMUNICATING = 5,
    BACKUP = 6,             // Backup & Restore
    ALWAYS_ON_IDLE = 99,    // Internal state; not reflected in state register
};

//******************************************************************************
//* Function prototypes
//******************************************************************************

/**
 * @brief Initialize the state logger GPIOs
 * 
 */
void state_logger_init(void);

/**
 * @brief Set the current state in the state logger
 * 
 * @param state State to set
 */
void state_logger_set(enum state_logger_states state);

/**
 * @brief Get the current state from the state logger
 * 
 * @return enum state_logger_states Current state
 */
enum state_logger_states state_logger_get(void);

#endif // _STATE_LOGGER_H_