
#include <zephyr/kernel.h>

// Power Management
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>

// Flash (NVM)
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

//#include <logging/log.h>
//LOG_MODULE_REGISTER(app, CONFIG_LOG_DEFAULT_LEVEL);

// Custom peripherals
#include "main.h"
#include "gpios.h"
#include "uart.h"
#include "tof.h"
#include "bluetooth.h"

//*******************************************************************************
//* Macros
//*******************************************************************************

// Flash (NVM) definitions
#define STORAGE_NODE DT_NODE_BY_FIXED_PARTITION_LABEL(userstorage)
#define FLASH_NODE DT_MTD_FROM_FIXED_PARTITION(STORAGE_NODE)
// IDs of NVM entries
#define T_samples_ID 1
#define n_Samples_ID 2
#define Sensor_ID 3
#define Processing_ID 4
#define ENBackups_ID 5
#define sample_packets_counter_ID 6
#define sample_data_counter_ID 7
#define sample_data_buffer_ID 8
// Time-of-Flight (ToF) sensor parameters
// IDs of NVM entries
#define TOF_distance_mode_ID 9
#define TOF_intermeas_ms_ID 10
#define TOF_timing_budget_ms_ID 11
#define TOF_distance_threshold_low_ID 12
#define TOF_distance_threshold_high_ID 13
#define TOF_distance_threshold_window_ID 14

//******************************************************************************
//* Global variables
//******************************************************************************

enum states state;
enum states next_state;

struct Benchmark_params SOC;

struct TOF_params TOF;


volatile bool pin_state;        // This is a dummy variable only used to execute a periodic GPIO reading to emulate REACT.
volatile bool sample_now;

// Flash (NVM)
static struct nvs_fs fs;


const struct device *cons = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

//******************************************************************************
//* Function prototypes
//******************************************************************************

int init_flash(void);
static inline bool should_sample_now(void);
static inline bool should_communicate_now(void);
static inline bool should_backup_now(void);
int load_params(void);
static void sampling_timer_callback(struct k_timer *timer_id);
int nvm_read_benchmark_params(void);
void nvm_write_benchmark_params(void);
int nvm_read_backup(void);
void nvm_write_backup(void);
int nvm_read_tof_params(void);
void nvm_write_tof_params(void);
void fsm_default(void);
void fsm_default_processing(void);
void fsm_event_driven_communication(void);
void fsm_always_on_sensors(void);

//******************************************************************************
//* Interrupt service routines
//******************************************************************************

// Timer Callback
static void sampling_timer_callback(struct k_timer *timer_id) {
    sample_now = true;

    //if (sample_packets_counter < SOC.num_samples) {
    //    state = SAMPLING;       // Sampling task only
    //}
    //if (sample_packets_counter >= SOC.num_samples - 1) {     //add: & sample_packets_counter > 0
    //    state = COMMUNICATING;  // Sampling + Communication back-to-back
    //    // -1 because the COMMUNICATING state will also sample once before sending the data
    //}
}

// Timer instance
K_TIMER_DEFINE(FSM_timer, sampling_timer_callback, NULL);

//******************************************************************************
//* Function definitions
//******************************************************************************

int main(void) {
    printk("Starting nRF52 Benchmark Application\n");

    NRF_POWER->DCDCEN = 1;
    // Delay to let the DC/DC converter stabilize 
    k_msleep(1000);

    gpio_init();
    init_flash();
    bt_init();
    load_params();
    sensors_init();

    state = BOOTING;
    next_state = IDLE;

    // Timer for sampling period
    k_timer_start(&FSM_timer, K_SECONDS(0), K_MSEC(SOC.sample_time_ms));

    // Configure gpio interrupt:

    pm_device_action_run(cons, PM_DEVICE_ACTION_SUSPEND);

    // Choose state machine based on processing type
    void (*fsm)(void);
    if (SOC.processing == 99) {
        fsm = fsm_always_on_sensors;
    } else if (SOC.processing == 10) {
        fsm = fsm_event_driven_communication;
    } else if (SOC.processing > 0) {
        fsm = fsm_default_processing;
    } else {
        fsm = fsm_default;
    }

    while (1) {   
        if (state != next_state) {
            if (next_state != ALWAYS_ON_IDLE) {
                // Internal state -> not logged in state register
                state_logger_set(next_state);
                // Wait a bit to ensure the state is logged
                k_msleep(1);
            } else if (state != SAMPLING) {
                // Attribute ALWAYS_ON_IDLE to SAMPLING state
                // Don't need to log the same state again
                state_logger_set(SAMPLING);
                // Wait a bit to ensure the state is logged
                k_msleep(1);
            }
            state = next_state;
        }
        fsm();
    }
    return 0;
}

void fsm_default(void) {
    switch (state) {
    case IDLE:
        // Conserve power in IDLE state
        k_msleep(100);

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)) {
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else if (sample_packets_counter >= SOC.num_samples) {
            next_state = COMMUNICATING;
        } else {
            next_state = IDLE;
        }
        break;

    case SAMPLING:
        //printk("Sampling\n");
        sensors_sample(SOC.sensor_type);
        sample_now = false;
        //printk("Sampling done\n");

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)) {
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else if (sample_packets_counter >= SOC.num_samples) {
            next_state = COMMUNICATING;
        } else {
            next_state = IDLE;
        }
        break;

    case COMMUNICATING:
        //printk("Communicating\n");
        //printk("Sample Counter: %i\n", sample_data_counter);
        communicate_samples((uint16_t*)&sample_data, sample_data_counter);

        sample_packets_counter = 0;            // reset sample counter
        sample_data_counter = 0;                       

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)){
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else {
            next_state = IDLE;
        }
        break;

    case BACKUP:
        nvm_write_backup();
        next_state = IDLE;
        break;

    default:
        next_state = IDLE;
        break;
    }
}

void fsm_default_processing(void) {
    switch (state) {
    case IDLE:
        // Conserve power in IDLE state
        k_msleep(100);

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)) {
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else if (sample_packets_counter >= SOC.num_samples) {
            next_state = PROCESSING;
        } else {
            next_state = IDLE;
        }
        break;

    case SAMPLING:
        //printk("Sampling\n");
        sensors_sample(SOC.sensor_type);
        sample_now = false;
        //printk("Sampling done\n");

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)) {
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else if (sample_packets_counter >= SOC.num_samples) {
            next_state = PROCESSING;
        } else {
            next_state = IDLE;
        }
        break;

    case PROCESSING:
        process_samples();
        //printk("Processing done\n");

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)) {
            next_state = BACKUP;
        } else {
            next_state = COMMUNICATING;
        }
        break;

    case COMMUNICATING:
        //printk("Communicating\n");
        //printk("Sample Counter: %i\n", sample_data_counter);
        communicate_samples((uint16_t*)&sample_data, sample_data_counter);

        sample_packets_counter = 0;            // reset sample counter
        sample_data_counter = 0;                       

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)){
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else {
            next_state = IDLE;
        }
        break;

    case BACKUP:
        nvm_write_backup();
        next_state = IDLE;
        break;

    default:
        next_state = IDLE;
        break;
    }
}

void fsm_event_driven_communication(void) {
    switch (state) {
    case IDLE:
        // Conserve power in IDLE state
        k_msleep(100);

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)) {
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else if ((sample_packets_counter >= SOC.num_samples) || (sample_packets_counter && gpio_pin_get_dt(&EventGPIO))) {
            next_state = PROCESSING;
        } else {
            next_state = IDLE;
        }
        break;

    case SAMPLING:
        //printk("Sampling\n");
        sensors_sample(SOC.sensor_type);
        sample_now = false;
        //printk("Sampling done\n");

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)) {
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else if ((sample_packets_counter >= SOC.num_samples) || gpio_pin_get_dt(&EventGPIO)) {
            next_state = PROCESSING;
        } else {
            next_state = IDLE;
        }
        break;

    case PROCESSING:
        process_samples();
        //printk("Processing done\n");

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)) {
            next_state = BACKUP;
        } else {
            next_state = COMMUNICATING;
        }
        break;

    case COMMUNICATING:
        //printk("Communicating\n");
        //printk("Sample Counter: %i\n", sample_data_counter);
        communicate_samples((uint16_t*)&sample_data, sample_data_counter);

        sample_packets_counter = 0;            // reset sample counter
        sample_data_counter = 0;                       

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)){
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else {
            next_state = IDLE;
        }
        break;

    case BACKUP:
        nvm_write_backup();
        next_state = IDLE;
        break;

    default:
        next_state = IDLE;
        break;
    }
}

void fsm_always_on_sensors(void) {
    switch (state) {
    case IDLE:
        // Conserve power in IDLE state
        k_msleep(100);

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)) {
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else if (sample_packets_counter >= SOC.num_samples) {
            next_state = COMMUNICATING;
        } else {
            next_state = ALWAYS_ON_IDLE;
        }
        break;

    case ALWAYS_ON_IDLE:
        // Conserve power in IDLE state
        k_msleep(100);

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)) {
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else if (sample_packets_counter >= SOC.num_samples) {
            next_state = COMMUNICATING;
        } else {
            next_state = ALWAYS_ON_IDLE;
        }
        break;

    case SAMPLING:
        //printk("Sampling\n");
        sensors_sample(SOC.sensor_type);
        sample_now = false;
        //printk("Sampling done\n");

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)) {
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else if (sample_packets_counter >= SOC.num_samples) {
            next_state = COMMUNICATING;
        } else {
            next_state = ALWAYS_ON_IDLE;
        }
        break;

    case COMMUNICATING:
        //printk("Communicating\n");
        //printk("Sample Counter: %i\n", sample_data_counter);
        communicate_samples((uint16_t*)&sample_data, sample_data_counter);

        sample_packets_counter = 0;            // reset sample counter
        sample_data_counter = 0;                       

        if (SOC.backups_enabled && gpio_pin_get_dt(&shutdown)){
            next_state = BACKUP;
        } else if (sample_now && (sample_packets_counter < SOC.num_samples)) {
            next_state = SAMPLING;
        } else {
            next_state = ALWAYS_ON_IDLE;
        }
        break;

    case BACKUP:
        nvm_write_backup();
        next_state = IDLE;
        break;

    default:
        next_state = IDLE;
        break;
    }
}

static inline bool should_sample_now(void) {
    return sample_now && (sample_packets_counter < SOC.num_samples);
}

static inline bool should_communicate_now(void) {
    return sample_packets_counter >= SOC.num_samples;
}

static inline bool should_backup_now(void) {
    return SOC.backups_enabled && gpio_pin_get_dt(&shutdown);
}

// Initialize flash memory (NVM) to store benchmark configurations and program states
int init_flash(void){
    struct flash_pages_info info;
    int rc;
    fs.flash_device = DEVICE_DT_GET(FLASH_NODE);
    if (!device_is_ready(fs.flash_device)) {
	printk("Flash device %s is not ready\n", fs.flash_device->name);
	return 0;
    }
    fs.offset = DT_REG_ADDR(DT_NODE_BY_FIXED_PARTITION_LABEL(userstorage)); // equivalent to FLASH_AREA_OFFSET(userstorage);
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
	printk("Unable to get page info\n");
	return 0;
    }
    fs.sector_size = info.size;
    fs.sector_count = 2U;

    rc = nvs_mount(&fs);
    if (rc) {
    	printk("Flash Init failed\n");
    	return 0;
    }
    return 0;
}

// Either receive updated parameters via UART from RPi or restore configuration from NVM
int load_params(void) {
    if (gpio_pin_get_dt(&updateConfig)) {    // If paramter update is signaled by RPi
        uart_setup();       // activate UART
        notify_host_ready_for_params();             // Notify Raspberry Pi that SoC is ready to receive parameters
        while (!params_updated()) {    // Wait until parameters received
            gpio_pin_toggle_dt(&led_r);     
            k_msleep(100);
        }  
        gpio_pin_set_dt(&led_r, 0);  
        // Store parameters to NVM
        nvm_write_benchmark_params();

        if (SOC.sensor_type & 8) {
            nvm_write_tof_params();
        }

        sample_packets_counter = 0;     // Reset counter for stored sample packets
        sample_data_counter = 0;        // Reset counter for stored sample data
        nvm_write_backup();          // Reset backup data in NVM

        gpio_pin_set_dt(&led_g, 1);  
        while(1){};
        // Wait for RPi to disconnect constant power supply and connect to EH system (reboot)

    } else {        // If parameters will not be updated -> load from NVM
        int ret = 0;
        ret += nvm_read_benchmark_params();

        if (SOC.sensor_type & 8) {
            ret += nvm_read_tof_params();
        }

        if (SOC.backups_enabled) {
            state = BACKUP;
            state_logger_set(state);
            ret += nvm_read_backup();
            state = IDLE;
            state_logger_set(state);
        } else {
            sample_packets_counter = 0;     // Reset counter for stored sample packets
            sample_data_counter = 0;        // Reset counter for stored sample data
        }
        if (ret < 0) {
            printk("Failed to read backup data!\n");
        }
    }
    return 0;
}


int nvm_read_benchmark_params(void) {
    int ret = 0;
    ret += nvs_read(&fs, T_samples_ID, &SOC.sample_time_ms, sizeof(SOC.sample_time_ms));
    ret += nvs_read(&fs, n_Samples_ID, &SOC.num_samples, sizeof(SOC.num_samples));
    ret += nvs_read(&fs, Sensor_ID, &SOC.sensor_type, sizeof(SOC.sensor_type));
    ret += nvs_read(&fs, Processing_ID, &SOC.processing, sizeof(SOC.processing));
    ret += nvs_read(&fs, ENBackups_ID, &SOC.backups_enabled, sizeof(SOC.backups_enabled));
    return ret;
}

void nvm_write_benchmark_params(void) {
    (void)nvs_write(&fs, T_samples_ID, &SOC.sample_time_ms, sizeof(SOC.sample_time_ms));
    (void)nvs_write(&fs, n_Samples_ID, &SOC.num_samples, sizeof(SOC.num_samples));
    (void)nvs_write(&fs, Sensor_ID, &SOC.sensor_type, sizeof(SOC.sensor_type));
    (void)nvs_write(&fs, Processing_ID, &SOC.processing, sizeof(SOC.processing));
    (void)nvs_write(&fs, ENBackups_ID, &SOC.backups_enabled, sizeof(SOC.backups_enabled));
}

int nvm_read_backup(void) {
    int ret = 0;
    ret += nvs_read(&fs, sample_packets_counter_ID, &sample_packets_counter, sizeof(sample_packets_counter));
    ret += nvs_read(&fs, sample_data_counter_ID, &sample_data_counter, sizeof(sample_data_counter));
    ret += nvs_read(&fs, sample_data_buffer_ID, &sample_data, 2*sample_data_counter);
    return ret;
}

void nvm_write_backup(void) {
    nvs_write(&fs, sample_packets_counter_ID, &sample_packets_counter, sizeof(sample_packets_counter));
    nvs_write(&fs, sample_data_counter_ID, &sample_data_counter, sizeof(sample_data_counter));
    nvs_write(&fs, sample_data_buffer_ID, &sample_data, 2*sample_data_counter);
}

int nvm_read_tof_params(void) {
    int ret = 0;
    ret += nvs_read(&fs, TOF_distance_mode_ID, &TOF.distance_mode, sizeof(TOF.distance_mode));
    ret += nvs_read(&fs, TOF_intermeas_ms_ID, &TOF.intermeas_ms, sizeof(TOF.intermeas_ms));
    ret += nvs_read(&fs, TOF_timing_budget_ms_ID, &TOF.timing_budget_ms, sizeof(TOF.timing_budget_ms));
    ret += nvs_read(&fs, TOF_distance_threshold_low_ID, &TOF.distance_threshold_low, sizeof(TOF.distance_threshold_low));
    ret += nvs_read(&fs, TOF_distance_threshold_high_ID, &TOF.distance_threshold_high, sizeof(TOF.distance_threshold_high));
    ret += nvs_read(&fs, TOF_distance_threshold_window_ID, &TOF.distance_threshold_window, sizeof(TOF.distance_threshold_window));
    return ret;
}

void nvm_write_tof_params(void) {
    (void)nvs_write(&fs, TOF_distance_mode_ID, &TOF.distance_mode, sizeof(TOF.distance_mode));
    (void)nvs_write(&fs, TOF_intermeas_ms_ID, &TOF.intermeas_ms, sizeof(TOF.intermeas_ms));
    (void)nvs_write(&fs, TOF_timing_budget_ms_ID, &TOF.timing_budget_ms, sizeof(TOF.timing_budget_ms));
    (void)nvs_write(&fs, TOF_distance_threshold_low_ID, &TOF.distance_threshold_low, sizeof(TOF.distance_threshold_low));
    (void)nvs_write(&fs, TOF_distance_threshold_high_ID, &TOF.distance_threshold_high, sizeof(TOF.distance_threshold_high));
    (void)nvs_write(&fs, TOF_distance_threshold_window_ID, &TOF.distance_threshold_window, sizeof(TOF.distance_threshold_window));
}