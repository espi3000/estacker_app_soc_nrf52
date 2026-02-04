#include "backup.h"
#include "main.h"
#include "uart.h"
#include "tof.h"
#include "gpio_dev.h"
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/sys/ring_buffer.h>

//******************************************************************************
//* Macros
//******************************************************************************

// Flash (NVM) definitions
#define STORAGE_NODE DT_NODE_BY_FIXED_PARTITION_LABEL(userstorage)
#define FLASH_NODE DT_MTD_FROM_FIXED_PARTITION(STORAGE_NODE)
// IDs of NVM entries
#define SAMPLE_INTERVAL_ID 1
#define NUM_SAMPLES_ID 2
#define SENSOR_TYPE_ID 3
#define PROCESSING_ID 4
#define BACKUPS_ENABLED_ID 5
#define sample_packets_counter_ID 6
#define sample_data_counter_ID 7
#define sample_data_buffer_ID 8
// Time-of-Flight (ToF) sensor parameters
// IDs of NVM entries
#define TOF_DISTANCE_MODE_ID 9
#define TOF_INTERMEAS_MS_ID 10
#define TOF_TIMING_BUDGET_MS_ID 11
#define TOF_DISTANCE_THRESHOLD_LOW_ID 12
#define TOF_DISTANCE_THRESHOLD_HIGH_ID 13
#define TOF_DISTANCE_THRESHOLD_WINDOW_ID 14

#define tot_num_samples_ID 15
#define ring_buffer_size_ID 16
#define ring_buffer_data_ID 17

//******************************************************************************
//* Global variables
//******************************************************************************

struct Benchmark_params SOC;
struct TOF_params TOF;
const struct gpio_dt_spec updateConfig = GPIO_DT_SPEC_GET(GPIO_NODE, updateconfig_gpios);

//*******************************************************************************
//* Static variables
//*******************************************************************************

static struct nvs_fs fs;
static struct ring_buf *s_ringbuf_ptr;

//******************************************************************************
//* Private function prototypes
//******************************************************************************

static void _get_params_from_host(void);
static void _get_params_from_nvm(void);
static int _read_benchmark_params_from_nvm(void);
static void _write_benchmark_params_to_nvm(void);
static int _read_tof_params_from_nvm(void);
static void _write_tof_params_to_nvm(void);

//******************************************************************************
//* Function definitions
//******************************************************************************

int backup_init(struct ring_buf *ringbuf_ptr) {
    s_ringbuf_ptr = ringbuf_ptr;
    // Input set if Pi wants to update benchmark configuration
    gpio_pin_configure_dt(&updateConfig, GPIO_INPUT | GPIO_PULL_DOWN);

    struct flash_pages_info info;
    int rc;
    fs.flash_device = DEVICE_DT_GET(FLASH_NODE);
    if (!device_is_ready(fs.flash_device)) {
	    //printk("Flash device %s is not ready\n", fs.flash_device->name);
	    return 1;
    }
    fs.offset = DT_REG_ADDR(DT_NODE_BY_FIXED_PARTITION_LABEL(userstorage)); // equivalent to FLASH_AREA_OFFSET(userstorage);
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
	    //printk("Unable to get page info\n");
	    return 1;
    }
    fs.sector_size = info.size;
    fs.sector_count = 2U;

    rc = nvs_mount(&fs);
    if (rc) {
    	//printk("Flash Init failed\n");
    	return 1;
    }
    return 0;
}

// Either receive updated parameters via UART from RPi or restore configuration from NVM
int backup_load_params(void) {
    irq_lock(); // disable global interrupts
    if (gpio_pin_get_dt(&updateConfig)) {    // If parameter update is signaled by RPi
        _get_params_from_host();
    } else {        // If parameters will not be updated -> load from NVM
        _get_params_from_nvm();
    }
    irq_unlock(0); // enable global interrupts
    return 0;
}

int backup_read_from_nvm(void) {
    int ret = 0;
    uint8_t *data_ptr;
    uint32_t ring_buffer_size = 0;
    ring_buf_put_claim(s_ringbuf_ptr, &data_ptr, ring_buf_capacity_get(s_ringbuf_ptr));

    ret += nvs_read(&fs, tot_num_samples_ID, &g_tof_tot_num_samples, sizeof(g_tof_tot_num_samples));
    ret += nvs_read(&fs, ring_buffer_size_ID, &ring_buffer_size, sizeof(ring_buffer_size));
    ret += nvs_read(&fs, ring_buffer_data_ID, data_ptr, ring_buffer_size);

    ring_buf_put_finish(s_ringbuf_ptr, ring_buffer_size);
    return ret;
}

int backup_write_to_nvm(void) {
    int ret = 0;
    uint64_t tot_num_samples = tof_get_tot_num_samples();
    uint8_t *data_ptr;
    uint32_t ring_buffer_size = ring_buf_get_claim(s_ringbuf_ptr, &data_ptr, ring_buf_capacity_get(s_ringbuf_ptr));

    ret += nvs_write(&fs, tot_num_samples_ID, &tot_num_samples, sizeof(tot_num_samples));
    ret += nvs_write(&fs, ring_buffer_size_ID, &ring_buffer_size, sizeof(ring_buffer_size));
    ret += nvs_write(&fs, ring_buffer_data_ID, data_ptr, ring_buffer_size);
    // No need to call ring_buf_get_finish() as we are not consuming data here
    return ret;
}

static void _get_params_from_host(void) {
    uart_setup();       // activate UART
    notify_host_ready_for_params();             // Notify Raspberry Pi that SoC is ready to receive parameters
    while (!params_updated()) {    // Wait until parameters received
        gpio_pin_toggle_dt(&led_r);     
        k_msleep(100);
    }  
    gpio_pin_set_dt(&led_r, 0);  
    // Store parameters to NVM
    _write_benchmark_params_to_nvm();

    if (SOC.sensor_type & 8) {
        _write_tof_params_to_nvm();
    }

    //sample_packets_counter = 0;     // Reset counter for stored sample packets
    //sample_data_counter = 0;        // Reset counter for stored sample data
    backup_write_to_nvm();          // Reset backup data in NVM

    gpio_pin_set_dt(&led_g, 1);  
    while (1) {};
    // Wait for RPi to disconnect constant power supply and connect to EH system (reboot)
}

static void _get_params_from_nvm(void) {
    int ret = 0;
    ret += _read_benchmark_params_from_nvm();

    if (SOC.sensor_type & 8) {
        ret += _read_tof_params_from_nvm();
    }

    if (SOC.backups_enabled) {
        ret += backup_read_from_nvm();
    } else {
        // No backups -> reset sample data counters
        //sample_packets_counter = 0;     // Reset counter for stored sample packets
        //sample_data_counter = 0;        // Reset counter for stored sample data
    }
    if (ret < 0) {
        printk("Failed to read backup data!\n");
    }
}


static int _read_benchmark_params_from_nvm(void) {
    int ret = 0;
    ret += nvs_read(&fs, SAMPLE_INTERVAL_ID, &SOC.sample_time_ms, sizeof(SOC.sample_time_ms));
    ret += nvs_read(&fs, NUM_SAMPLES_ID, &SOC.num_samples, sizeof(SOC.num_samples));
    ret += nvs_read(&fs, SENSOR_TYPE_ID, &SOC.sensor_type, sizeof(SOC.sensor_type));
    ret += nvs_read(&fs, PROCESSING_ID, &SOC.processing, sizeof(SOC.processing));
    ret += nvs_read(&fs, BACKUPS_ENABLED_ID, &SOC.backups_enabled, sizeof(SOC.backups_enabled));
    return ret;
}

static void _write_benchmark_params_to_nvm(void) {
    int ret = 0;
    ret += nvs_write(&fs, SAMPLE_INTERVAL_ID, &SOC.sample_time_ms, sizeof(SOC.sample_time_ms));
    ret += nvs_write(&fs, NUM_SAMPLES_ID, &SOC.num_samples, sizeof(SOC.num_samples));
    ret += nvs_write(&fs, SENSOR_TYPE_ID, &SOC.sensor_type, sizeof(SOC.sensor_type));
    ret += nvs_write(&fs, PROCESSING_ID, &SOC.processing, sizeof(SOC.processing));
    ret += nvs_write(&fs, BACKUPS_ENABLED_ID, &SOC.backups_enabled, sizeof(SOC.backups_enabled));
}

static int _read_tof_params_from_nvm(void) {
    int ret = 0;
    ret += nvs_read(&fs, TOF_DISTANCE_MODE_ID, &TOF.distance_mode, sizeof(TOF.distance_mode));
    ret += nvs_read(&fs, TOF_INTERMEAS_MS_ID, &TOF.intermeas_ms, sizeof(TOF.intermeas_ms));
    ret += nvs_read(&fs, TOF_TIMING_BUDGET_MS_ID, &TOF.timing_budget_ms, sizeof(TOF.timing_budget_ms));
    ret += nvs_read(&fs, TOF_DISTANCE_THRESHOLD_LOW_ID, &TOF.distance_threshold_low, sizeof(TOF.distance_threshold_low));
    ret += nvs_read(&fs, TOF_DISTANCE_THRESHOLD_HIGH_ID, &TOF.distance_threshold_high, sizeof(TOF.distance_threshold_high));
    ret += nvs_read(&fs, TOF_DISTANCE_THRESHOLD_WINDOW_ID, &TOF.distance_threshold_window, sizeof(TOF.distance_threshold_window));
    return ret;
}

static void _write_tof_params_to_nvm(void) {
    int ret = 0;
    ret += nvs_write(&fs, TOF_DISTANCE_MODE_ID, &TOF.distance_mode, sizeof(TOF.distance_mode));
    ret += nvs_write(&fs, TOF_INTERMEAS_MS_ID, &TOF.intermeas_ms, sizeof(TOF.intermeas_ms));
    ret += nvs_write(&fs, TOF_TIMING_BUDGET_MS_ID, &TOF.timing_budget_ms, sizeof(TOF.timing_budget_ms));
    ret += nvs_write(&fs, TOF_DISTANCE_THRESHOLD_LOW_ID, &TOF.distance_threshold_low, sizeof(TOF.distance_threshold_low));
    ret += nvs_write(&fs, TOF_DISTANCE_THRESHOLD_HIGH_ID, &TOF.distance_threshold_high, sizeof(TOF.distance_threshold_high));
    ret += nvs_write(&fs, TOF_DISTANCE_THRESHOLD_WINDOW_ID, &TOF.distance_threshold_window, sizeof(TOF.distance_threshold_window));
}