#include "io_expander.h"
#include "gpio_dev.h"
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>

//******************************************************************************
//* Global constants
//******************************************************************************

const struct i2c_dt_spec dev_i2c_ioexp = I2C_DT_SPEC_GET(I2C0_IOExp);
const struct gpio_dt_spec IOExp_INTn = GPIO_DT_SPEC_GET(GPIO_NODE, ioexpintn_gpios);

//******************************************************************************
//* Function definitions
//******************************************************************************

// Configure IO Directions of I/O expander responsible for sensor power & interrupts
int io_expander_init(struct gpio_callback *ioexp_cb_data, gpio_callback_handler_t ioexp_handler) {
    // I/O Expander interrupt from sensor board
    gpio_pin_configure_dt(&IOExp_INTn, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&IOExp_INTn, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(ioexp_cb_data, ioexp_handler, BIT(IOExp_INTn.pin));
    gpio_add_callback(IOExp_INTn.port, ioexp_cb_data);

    //i2c_configure();

    int ret = 0;
    if (!i2c_is_ready_dt(&dev_i2c_ioexp)) { // device_is_ready(dev_i2c_ioexp.bus)) {
        //printf("Device %s is not ready\n", dev_i2c_ioexp.bus->name);
        return 1;
    }
    /**
     * Set IO outputs to low
     * IOs that are configured as inputs will have the following behavior:
     * - If port is 0, interrupt will assert when input is 1
     * - If port is 1, interrupt will assert when input is 0
     */
    const uint8_t output_port_0_reg = 0x02;
    const uint8_t set_io_low_cmd[3] = {output_port_0_reg, 0b01000000, 0b00000000}; 
    ret += i2c_write_dt(&dev_i2c_ioexp, set_io_low_cmd, sizeof(set_io_low_cmd));

    // Set IO Directions (1=input, 0=output)
    const uint8_t config_port_0_reg = 0x06;
    const uint8_t set_io_dir_cmd[3] = {config_port_0_reg, 0b01000000, 0b00000000}; 
    ret += i2c_write_dt(&dev_i2c_ioexp, set_io_dir_cmd, sizeof(set_io_dir_cmd));
    return ret;
}

int io_expander_clear_interrupt(void) {
    uint8_t input_port_0_reg = 0x00;
    uint8_t input_reg_data[2];
    return i2c_write_read_dt(&dev_i2c_ioexp, &input_port_0_reg, 1, &input_reg_data, 2);    
}

int io_expander_change_channel(uint16_t ch_mask, bool state) {
    int ret = 0;
    uint8_t output_reg_add = 0x02;
    uint16_t output_reg_data;
    ret += i2c_write_read_dt(&dev_i2c_ioexp, &output_reg_add, 1, &output_reg_data, 2);
    if (state) {
        output_reg_data = output_reg_data | ch_mask;
    } else {
        output_reg_data = output_reg_data & (~ch_mask);
    }
    uint8_t config[3] = {0x02, output_reg_data, output_reg_data >> 8}; // Set IO Directions (output reg, port 0, port 1)
    ret += i2c_write_dt(&dev_i2c_ioexp, config, sizeof(config));
    return ret;
}