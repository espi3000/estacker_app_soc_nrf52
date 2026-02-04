#include "state_logger.h"
#include "gpio_dev.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

//******************************************************************************
//* Global variables
//******************************************************************************

const struct gpio_dt_spec state0 = GPIO_DT_SPEC_GET(GPIO_NODE, state0_gpios);
const struct gpio_dt_spec state1 = GPIO_DT_SPEC_GET(GPIO_NODE, state1_gpios);
const struct gpio_dt_spec state2 = GPIO_DT_SPEC_GET(GPIO_NODE, state2_gpios);
const struct gpio_dt_spec stateSet = GPIO_DT_SPEC_GET(GPIO_NODE, stateset_gpios); 

//******************************************************************************
//* Static variables
//******************************************************************************

static enum state_logger_states s_state;

//******************************************************************************
//* Function definitions
//******************************************************************************

void state_logger_init(void) {
    s_state = 0;

    gpio_pin_configure_dt(&state0, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&state1, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&state2, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&stateSet, GPIO_OUTPUT_ACTIVE);

    gpio_pin_set_dt(&state0, 0);
    gpio_pin_set_dt(&state1, 0);
    gpio_pin_set_dt(&state2, 0);
    gpio_pin_set_dt(&stateSet, 0); 
}

// State Logging via State Register on Sensor Board
void state_logger_set(enum state_logger_states state) {
    s_state = state;

    // Set state bits via GPIOs and Pull !G (stateSet pin) to write to the latch
    gpio_pin_set_dt(&state0, (state >> 0) & 1);
    gpio_pin_set_dt(&state1, (state >> 1) & 1);
    gpio_pin_set_dt(&state2, (state >> 2) & 1);
    //k_msleep(0.1);
    gpio_pin_set_dt(&stateSet, 1); 
    k_msleep(0.1);
    gpio_pin_set_dt(&stateSet, 0);  
    // Reset State pins to reduce leakage
    gpio_pin_set_dt(&state0, 0);
    gpio_pin_set_dt(&state1, 0);
    gpio_pin_set_dt(&state2, 0);
}

enum state_logger_states state_logger_get(void) {
    return s_state;
}