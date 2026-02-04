#include "tof.h"
#include "main.h"
#include "VL53L1X_api.h"
#include "gpios.h"               // I2C TEST

#include "io_expander.h"
#include "i2c_switch.h"

#include <arm_math.h>
#include <arm_const_structs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/ring_buffer.h>

//******************************************************************************
//* Global variables
//******************************************************************************

uint64_t g_tof_tot_num_samples = 0;

//******************************************************************************
//* Static variables
//******************************************************************************

// Steps for EH OC sampling:
// Init ADC + Set ADC GPO1 High
// SoC GPIO: Disable MPPT bypass (3 GPIOs total)
// Sample ADC and read result(s), power down ADC
// Reset Bypass GPIOs

//******************************************************************************
//* Function definitions
//******************************************************************************

int tof_init(void) {
    g_tof_tot_num_samples = 0;

    tof_power_on();
    i2c_switch_init();
    tof_i2c_enable();

    int ret = 0;
    uint16_t dev = 0x0000;

    // Wait for device booted
    
    uint8_t booted_up;
    do {
        ret = VL53L1X_BootState(dev, &booted_up);
        //k_msleep(2);
    } while (!booted_up);

    // Initialize sensor to default values
    ret += VL53L1X_SensorInit(dev);
    ret += VL53L1X_SetDistanceMode(dev, TOF.distance_mode);        // Short distance mode
    ret += VL53L1X_SetInterMeasurementInMs(dev, TOF.intermeas_ms); // Interval between ranging operations (2000 ms)
    ret += VL53L1X_SetTimingBudgetInMs(dev, TOF.timing_budget_ms); // Minimum timing budget for all distances (33 ms)
    ret += VL53L1X_SetInterruptPolarity(dev, 0);                   // Interrupt active low
    ret += VL53L1X_SetDistanceThreshold(dev, 
        TOF.distance_threshold_low, 
        TOF.distance_threshold_high, 
        TOF.distance_threshold_window, 0);                  // Trigger interrupt if object closer than 300 mm detected

    // Always-on mode
    if (SOC.processing==99) {
        ret += VL53L1X_StartRanging(dev);                              // Starts continuous ranging
        ret += VL53L1X_ClearInterrupt(dev);
    } else {
        ret += VL53L1X_SetTimingBudgetInMs(dev, TOF.timing_budget_ms); // Minimum timing budget for all distances (200 ms)
    }

    return 0;
}

void tof_power_on(void) {
    io_expander_change_channel(IO_EXPANDER_TOF_MASK, true);
    //k_msleep(250); // Wait for sensor to boot up
    //! Clear interrupts?
}

void tof_power_off(void) {
    io_expander_change_channel(IO_EXPANDER_TOF_MASK, false);
    //k_msleep(250); //! Wait for sensor to power down?
}

void tof_i2c_enable(void) {
    i2c_switch_change_channel(I2C_SWITCH_TOF_MASK, true);
}

void tof_i2c_disable(void) {
    i2c_switch_change_channel(I2C_SWITCH_TOF_MASK, false);
}

int tof_clear_interrupt(void) {
    uint16_t dev = 0x0000;
    return VL53L1X_ClearInterrupt(dev);
}

int tof_start_sampling(void) {
    int ret = 0;
    uint16_t dev = 0x0000;

    // Wait for device booted
    //uint8_t state = 0;
    //do {
    //    ret = VL53L1X_BootState(dev, &state);
    //} while (state == 0);

    // Start sensing
    ret = ret + VL53L1X_StartRanging(dev);
    k_msleep(25);

    return ret;
}

int tof_stop_sampling(void) {
    int ret = 0;
    uint16_t dev = 0x0000;

    // Stop sensing
    ret += VL53L1X_StopRanging(dev);
    k_msleep(25);

    return ret;
}

int tof_get_sample(struct ring_buf *ringbuf) {
    int ret = 0;
    uint16_t dev = 0x0000;
    uint8_t tof_data_ready = 0;
    ret += VL53L1X_CheckForDataReady(dev, &tof_data_ready);
    if (!tof_data_ready) {
        return 1;   // No new data
    }
    uint64_t cycle_count = k_uptime_get(); //* Works, but large numbers (1.2e18 delta)
    uint8_t tof_range_status;
    uint16_t tof_distance;
    ret += VL53L1X_GetRangeStatus(dev, &tof_range_status);
    ret += VL53L1X_GetDistance(dev, &tof_distance);
    g_tof_tot_num_samples++;
    
    uint32_t data = ((uint32_t)tof_range_status << 16) | tof_distance;
    ring_buf_put(ringbuf, (uint8_t*)&cycle_count, sizeof(cycle_count));
    ring_buf_put(ringbuf, (uint8_t*)&data, sizeof(data));

    // Clearing interrupt starts next measurement in continuous mode
    ret += VL53L1X_ClearInterrupt(dev);
    io_expander_clear_interrupt();

    return ret;
}

int tof_get_num_samples(struct ring_buf *ringbuf) {
    return ring_buf_size_get(ringbuf)/(sizeof(uint64_t) + sizeof(uint32_t));
}

uint64_t tof_get_tot_num_samples(void) {
    return g_tof_tot_num_samples;
}

/*
int tof_sample_oneshot(void) {
    
    //power_sensors(SENS_TOF_MASK, 1);
    //Switch_SensI2C(SENS_TOF_MASK, 1);
    //tof_init();
    //tof_sample();
    //Switch_SensI2C(SENS_TOF_MASK, 0);
    //power_sensors(SENS_TOF_MASK, 0);
    //k_msleep(350);   // wait for possible interrupts to be cleared subsequently
    
    int ret = 0;
    uint16_t dev = 0x0000;

    if (SOC.processing != 99) {
        // Wait for device booted
        uint8_t state = 0;
        while (state == 0) {
            ret = VL53L1X_BootState(dev, &state);
            k_msleep(2);
        }

        // Load Sensor default config
        ret = ret + VL53L1X_DefaultConfig(dev);
        k_msleep(10);

        // Start sensing
        ret = ret + VL53L1X_StartRanging(dev);
        k_msleep(25);


        // Wait until new data is available
        uint8_t tof_data_ready = 0;
        while(tof_data_ready == 0) {
            ret = ret + VL53L1X_CheckForDataReady(dev, &tof_data_ready);
            k_msleep(5);
        }

        // Read TOF data

        // Check status (data integrity)
        // 0 = no error, 1 = sigma failure, 2 = signal failure, 4 = sensor out-of-bounds, and 7 = wraparound
        // SHOULD BE HANDLED
        uint8_t tof_rangeStatus;
        ret = ret + VL53L1X_GetRangeStatus(dev, &tof_rangeStatus);

        uint16_t tof_distance;
        ret = ret + VL53L1X_GetDistance(dev, &tof_distance);

        // Stop sensing
        ret = ret + VL53L1X_StopRanging(dev);
        k_msleep(25);

        // Clear interrupt 
        ret = ret + VL53L1X_ClearInterrupt(dev);

        if(ret != 0){
            printk("Failed to write to I2C device address %x. \n\r", dev_i2c_tof.addr);
            return 1;
        }

        //uint64_t cycle_count = k_cycle_get_64();
        //uint64_t *sample_data_64_ptr = (uint64_t*)&sample_data[sample_data_counter];
        // *sample_data_64_ptr = cycle_count;
        //sample_data_counter += sizeof(uint64_t)/sizeof(uint16_t);
        //sample_data[sample_data_counter] = tof_distance;            // Distance in mm
        //sample_data_counter += 1;

        //TODO: Use sample timer time
        //k_uptime_delta(&tof_timer);  // Reset and get elapsed time since last call
        // Store sensor data in sample buffer and increase data counter
        //uint64_t cycle_count = sys_clock_cycle_get_64();  //! Does not compile
        //uint64_t cycle_count = k_uptime_ticks();          // * Works, but large numbers (9.6e18 delta)
        uint64_t cycle_count = k_uptime_get();              // * Works, but large numbers (1.2e18 delta)
        //uint64_t cycle_count = sys_clock_tick_get();      // * Works, but large numbers (2.9e18 delta)
        //uint64_t cycle_count = k_cycle_get_64();          //! Only returns 0
        sample_data[sample_data_counter + 0] = (cycle_count >> 48); // Timestamp highest
        sample_data[sample_data_counter + 1] = (cycle_count >> 32) & 0xFFFF; // Timestamp high
        sample_data[sample_data_counter + 2] = (cycle_count >> 16) & 0xFFFF; // Timestamp mid
        sample_data[sample_data_counter + 3] = (cycle_count) & 0xFFFF;
        sample_data_counter = sample_data_counter + 4;
        sample_data[sample_data_counter] = tof_distance;            // Distance in mm
        sample_data_counter = sample_data_counter + 1;

        //// Store sensor data in sample buffer and increase data counter
        //uint32_t cycle_count = k_cycle_get_32();
        //sample_data[sample_data_counter] = cycle_count >> 16;            // Timestamp high
        //sample_data[sample_data_counter + 1] = cycle_count & 0xFFFF;   // Timestamp low
        //sample_data_counter = sample_data_counter + 2;
        //sample_data[sample_data_counter] = tof_distance;            // Distance in mm
        //sample_data_counter = sample_data_counter + 1;

    } else {

        if (IOExp_Int_Handling() == 8) {     // Check for interrupts. If TOF interrupt has triggered:
            sample_data[sample_data_counter] = 1;   // Register that event has been detected;
            printk("Interrupt detected!\n");
            //VL53L1X_StartRanging(dev);
            //VL53L1X_ClearInterrupt(dev);
        } else {
            sample_data[sample_data_counter] = 0;   // Register that no event has been detected;
            printk("No Interrupt detected.\n");
        }
        sample_data_counter = sample_data_counter + 1;
    }

    return 0;
}
*/

/*
int process_samples(void) {

    switch (SOC.processing) {
    case 1: { // Averaging
        float average_samples = 0;
        if (SOC.sensor_type & SENS_TMP_MASK) {
            for(int i = 0; i<sample_data_counter; i++) {
                average_samples = average_samples + (int16_t)sample_data[i];
            }
            average_samples = average_samples / sample_data_counter;
            sample_data[0] = (int16_t)average_samples;
            sample_data_counter = 1;
        }
        break;
    }
    default:
        break;
    }
    return 0;
}
*/