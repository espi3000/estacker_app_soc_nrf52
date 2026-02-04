#ifndef _IO_EXPANDER_H_
#define _IO_EXPANDER_H_

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

//******************************************************************************
//* Macros
//******************************************************************************

#define I2C0_IOExp DT_NODELABEL(ioexpander)

#define IO_EXPANDER_TMP_MASK 0x0200
#define IO_EXPANDER_IMU_MASK 0x1000
#define IO_EXPANDER_PMS_MASK 0x0080
#define IO_EXPANDER_TOF_MASK 0x0030
#define IO_EXPANDER_BIO_MASK 0x0003
#define IO_EXPANDER_GPS_MASK 0x4000
#define IO_EXPANDER_MIC_MASK 0x0800
#define IO_EXPANDER_COL_MASK 0x0008
#define IO_EXPANDER_BPS_MASK 0x0004
#define IO_EXPANDER_AIR_MASK 0x4000

//******************************************************************************
//* Function prototypes
//******************************************************************************

/**
 * @brief Initialize I/O expander
 * 
 * @return int 
 */
int io_expander_init(struct gpio_callback *ioexp_cb_data, gpio_callback_handler_t ioexp_handler);

/**
 * @brief I/O expander interrupt handler
 * 
 * @return int 
 */
int io_expander_irq_handler(void);

/**
 * @brief Clear I/O expander interrupt
 * 
 * @return int 
 */
int io_expander_clear_interrupt(void);

/**
 * @brief Change the power channel(s) of the I/O expander
 * 
 * @param ch_mask Channel mask to select channels
 * @param state true to enable channels, false to disable channels
 * @return int 
 */
int io_expander_change_channel(uint16_t ch_mask, bool state);

#endif // _IO_EXPANDER_H_