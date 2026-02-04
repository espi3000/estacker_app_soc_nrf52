
#include "gpios.h"
#include "gpio_dev.h"
#include <zephyr/drivers/gpio.h>

const struct gpio_dt_spec led_g = GPIO_DT_SPEC_GET(GPIO_NODE, ledg_gpios);
const struct gpio_dt_spec led_r = GPIO_DT_SPEC_GET(GPIO_NODE, ledr_gpios);

const struct gpio_dt_spec rangeHigh = GPIO_DT_SPEC_GET(GPIO_NODE, rangehigh_gpios);  

const struct gpio_dt_spec EventGPIO = GPIO_DT_SPEC_GET(GPIO_NODE, event_gpios);

const struct gpio_dt_spec low_v = GPIO_DT_SPEC_GET(GPIO_NODE, lowv_gpios);

const struct gpio_dt_spec ADC3Convstn = GPIO_DT_SPEC_GET(GPIO_NODE, adc3convstn_gpios);
const struct gpio_dt_spec ADC3Rstn = GPIO_DT_SPEC_GET(GPIO_NODE, adc3rstn_gpios);

const struct gpio_dt_spec MPPTBypassOff = GPIO_DT_SPEC_GET(GPIO_NODE, mpptbypassoff_gpios);
const struct gpio_dt_spec MPPTBypassOffn = GPIO_DT_SPEC_GET(GPIO_NODE, mpptbypassoffn_gpios);

int gpio_init(void) {

    // Board LEDs
    gpio_pin_configure_dt(&led_g, GPIO_OUTPUT_ACTIVE);
    gpio_pin_configure_dt(&led_r, GPIO_OUTPUT_ACTIVE);

    gpio_pin_set_dt(&led_g, 0);
    gpio_pin_set_dt(&led_r, 0);

    // Outputs to State Register
    //state_logger_init();          

    // Output to request higher current range from STM32
    gpio_pin_configure_dt(&rangeHigh, GPIO_OUTPUT_ACTIVE);    

    gpio_pin_set_dt(&rangeHigh, 0);          

    // Input set if Pi wants to update benchmark configuration
    //gpio_pin_configure_dt(&updateConfig, GPIO_INPUT | GPIO_PULL_DOWN);

    // Shutdown signal is high when voltage supervisor detects low energy storage voltage
    //gpio_pin_configure_dt(&shutdown, GPIO_INPUT);

    // I/O Expander interrupt from sensor board
    //gpio_pin_configure_dt(&IOExp_INTn, GPIO_INPUT);

    // Reactive Trace input from RPila
    gpio_pin_configure_dt(&EventGPIO, GPIO_INPUT);
    // ADC3 conversion start
    gpio_pin_configure_dt(&ADC3Convstn, GPIO_OUTPUT_ACTIVE);    
    gpio_pin_set_dt(&ADC3Convstn, 0);  
    // ADC3 Reset
    gpio_pin_configure_dt(&ADC3Rstn, GPIO_OUTPUT_ACTIVE);    
    gpio_pin_set_dt(&ADC3Rstn, 0);  


    // MPPT Bypass Disabling
    gpio_pin_configure_dt(&MPPTBypassOff, GPIO_OUTPUT_ACTIVE);    
    gpio_pin_configure_dt(&MPPTBypassOffn, GPIO_OUTPUT_ACTIVE);  
    gpio_pin_set_dt(&MPPTBypassOff, 0);  
    gpio_pin_set_dt(&MPPTBypassOffn, 0);  


    /*
    gpio_pin_toggle(led_g.port, led_g.pin);
    gpio_pin_toggle(led_r.port, led_r.pin);
    k_msleep(200);
    */
    return 0;
}

// Set/Clear GPIO for range change request
void REQ_Range(int Range_High) {
    if (Range_High == 1){
        gpio_pin_set_dt(&rangeHigh, 1);      
    } else {
        gpio_pin_set_dt(&rangeHigh, 0);   
    }
    
}
