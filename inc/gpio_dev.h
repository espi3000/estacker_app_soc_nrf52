#ifndef _GPIO_DEV_H_
#define _GPIO_DEV_H_

#include <zephyr/drivers/gpio.h>

#define GPIO_NODE DT_PATH(zephyr_user)

extern const struct gpio_dt_spec led_g;                 // Green board LED
extern const struct gpio_dt_spec led_r;                 // Red board LED

extern const struct gpio_dt_spec updateConfig;          // GPIO: Output from RPi to nRF to signal parameter update on startup
extern const struct gpio_dt_spec shutdown;              // Shutdown signal from external voltage supervisor
extern const struct gpio_dt_spec IOExp_INTn;            // I/O Expander interrupt from sensor board

extern const struct gpio_dt_spec EventGPIO;             // Reactive Signal from Event Trace (RPi)

extern const struct gpio_dt_spec low_v;                 // Low voltage signal from U14

extern const struct gpio_dt_spec rangeHigh;             // Used for output testing purposes. Not for range switching anymore.
extern const struct gpio_dt_spec ADC3Convstn;            // ADC 3 Conversion Start
extern const struct gpio_dt_spec ADC3Rstn;               // ADC 3 Reset 

extern const struct gpio_dt_spec MPPTBypassOff;         // Disable MPPT bypass
extern const struct gpio_dt_spec MPPTBypassOffn;        // Disable MPPT bypass

#endif // _GPIO_DEV_H_