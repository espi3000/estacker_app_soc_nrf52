
#include <zephyr/kernel.h>

// Power Management
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/ring_buffer.h>

//#include <logging/log.h>
//LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

// Custom peripherals
#include "main.h"
#include "gpios.h"
#include "tof.h"
#include "bluetooth.h"
#include "backup.h"

#include "io_expander.h"
#include "state_logger.h"
#include "gpio_dev.h"

//*******************************************************************************
//* Macros
//*******************************************************************************

//******************************************************************************
//* Global variables
//******************************************************************************

k_tid_t main_thread_id;
const struct device *cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
const struct gpio_dt_spec shutdown = GPIO_DT_SPEC_GET(GPIO_NODE, shutdown_gpios);

struct gpio_callback shutdown_cb_data;
 struct gpio_callback tof_sample_ready_cb_data;

#define RING_BUF_SIZE_POW2 10
#define RING_BUF_SIZE_BYTES (1 << RING_BUF_SIZE_POW2)
RING_BUF_ITEM_DECLARE_POW2(tof_ringbuf, RING_BUF_SIZE_POW2);

//******************************************************************************
//* Function prototypes
//******************************************************************************

//static void sampling_timer_callback(struct k_timer *timer_id);
void fsm_on_communicate(void);
void fsm_on_shutdown(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);
void fsm_on_sample_ready(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins);

void shutdown_init(struct gpio_callback *shutdown_cb_data, gpio_callback_handler_t shutdown_handler);

//******************************************************************************
//* Interrupt service routines
//******************************************************************************

// Timer Callback
//static void sampling_timer_callback(struct k_timer *timer_id) {
//    sample_now = true;

    //if (sample_packets_counter < SOC.num_samples) {
    //    state = SAMPLING;       // Sampling task only
    //}
    //if (sample_packets_counter >= SOC.num_samples - 1) {     //add: & sample_packets_counter > 0
    //    state = COMMUNICATING;  // Sampling + Communication back-to-back
    //    // -1 because the COMMUNICATING state will also sample once before sending the data
    //}
//}

// Timer instance
//K_TIMER_DEFINE(FSM_timer, sampling_timer_callback, NULL);

//******************************************************************************
//* Function definitions
//******************************************************************************

int main(void) {
    irq_lock(); // disable global interrupts
    //printk("Starting nRF52 Benchmark Application\n");

    NRF_POWER->DCDCEN = 1;
    // Delay to let the DC/DC converter stabilize 

    state_logger_init();
    state_logger_set(BACKUP);
    backup_init(&tof_ringbuf);
    state_logger_set(BOOTING);
    shutdown_init(&shutdown_cb_data, fsm_on_shutdown);
    io_expander_init(&tof_sample_ready_cb_data, fsm_on_sample_ready);

    gpio_init();
    k_msleep(1000);

    irq_unlock(0); // enable global interrupts
    
    state_logger_set(BACKUP);
    backup_load_params();
    state_logger_set(BOOTING);

    bt_init();
    tof_init();

    // Timer for sampling period
    //k_timer_start(&FSM_timer, K_SECONDS(0), K_MSEC(SOC.sample_time_ms));

    // Configure gpio interrupt:
    pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);

    main_thread_id = k_current_get();

    state_logger_set(IDLE);
    while (1) {   
        if (tof_get_num_samples(&tof_ringbuf) >= SOC.num_samples) {
            fsm_on_communicate();
        }
        // Optimize power consumption based on time until next event
        k_thread_suspend(main_thread_id);
    }
    return 0;
}

void shutdown_init(struct gpio_callback *shutdown_cb_data, gpio_callback_handler_t shutdown_handler) {
    gpio_pin_configure_dt(&shutdown, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&shutdown, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(shutdown_cb_data, shutdown_handler, BIT(shutdown.pin));
    gpio_add_callback(shutdown.port, shutdown_cb_data);
}

void fsm_on_communicate(void) {
    state_logger_set(COMMUNICATING);
    communicate_samples(&tof_ringbuf);
    state_logger_set(IDLE);
}

void fsm_on_shutdown(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins) {
    irq_lock(); // disable global interrupts
    int state_before_backup = state_logger_get();
    state_logger_set(BACKUP);
    backup_write_to_nvm();
    //tof_stop_sampling();
    state_logger_set(state_before_backup);
    irq_unlock(0); // enable global interrupts
}

void fsm_on_sample_ready(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins) {
    state_logger_set(SAMPLING);
    tof_get_sample(&tof_ringbuf);
    state_logger_set(IDLE);
    k_thread_resume(main_thread_id);
}