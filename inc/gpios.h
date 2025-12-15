
#ifndef _GPIOS_H_
#define _GPIOS_H_

//#include <drivers/gpio.h> // GPIOs

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#define GPIO_NODE DT_PATH(zephyr_user)

extern const struct gpio_dt_spec updateConfig;          // GPIO: Output from RPi to nRF to signal parameter update on startup
extern const struct gpio_dt_spec shutdown;              // Shutdown signal from external voltage supervisor
extern const struct gpio_dt_spec led_g;                 // Green board LED
extern const struct gpio_dt_spec led_r;                 // Red board LED
extern const struct gpio_dt_spec IOExp_INTn;            // I/O Expander interrupt from sensor board

extern const struct gpio_dt_spec EventGPIO;             // Reactive Signal from Event Trace (RPi)

extern const struct gpio_dt_spec low_v;                 // Low voltage signal from U14

extern const struct gpio_dt_spec rangeHigh;             // Used for output testing purposes. Not for range switching anymore.
extern const struct gpio_dt_spec ADC3Convstn;            // ADC 3 Conversion Start
extern const struct gpio_dt_spec ADC3Rstn;               // ADC 3 Reset 

extern const struct gpio_dt_spec MPPTBypassOff;         // Disable MPPT bypass
extern const struct gpio_dt_spec MPPTBypassOffn;        // Disable MPPT bypass


int gpio_init(void);
void state_logger_set(uint8_t state);
void REQ_Range(int Range_High);

#endif // _GPIOS_H_