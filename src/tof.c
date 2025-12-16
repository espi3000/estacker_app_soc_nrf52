#include "tof.h"
#include "main.h"
#include "VL53L1X_api.h"
#include "gpios.h"               // I2C TEST
#include <arm_math.h>
#include <arm_const_structs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

// Connected to sensor board v1.1 or v1.2
#define BOARD_V1_2      1

uint16_t SENS_TMP_MASK = 1;
uint16_t SENS_IMU_MASK = 2;
uint16_t SENS_PMS_MASK = 4;
uint16_t SENS_TOF_MASK = 8;
uint16_t SENS_BIO_MASK = 16;
uint16_t SENS_GPS_MASK = 32;
uint16_t SENS_MIC_MASK = 64;
uint16_t SENS_COL_MASK = 128;
uint16_t SENS_BPS_MASK = 256;
uint16_t SENS_AIR_MASK = 512;

const struct i2c_dt_spec dev_i2c_ioexp = I2C_DT_SPEC_GET(I2C0_IOExp);
const struct i2c_dt_spec dev_i2c_i2cswitch = I2C_DT_SPEC_GET(I2C0_I2CSwitch);
const struct i2c_dt_spec dev_i2c_tof = I2C_DT_SPEC_GET(I2C0_TOF);
const struct i2c_dt_spec dev_i2c_generalcall = I2C_DT_SPEC_GET(I2C0_GeneralCall);

uint16_t sample_data[512];   // Sample data buffer
uint16_t sample_data_counter;     // Sample data counter
uint16_t sample_packets_counter;     // Number of sample packets. A sample packet can consist of the measurements of several sensors

uint16_t test_data;         // Send test_data if sensor type is 0


// Steps for EH OC sampling:
// Init ADC + Set ADC GPO1 High
// SoC GPIO: Disable MPPT bypass (3 GPIOs total)
// Sample ADC and read result(s), power down ADC
// Reset Bypass GPIOs


// Configure IO Directions of I/O expander responsible for sensor power & interrupts
int io_expander_init(void){
    int ret = 0;
    if (!device_is_ready(dev_i2c_ioexp.bus)) {
        printf("Device %s is not ready\n", dev_i2c_ioexp.bus->name);
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

int i2c_switch_init(void) {
    int ret = 0;
    if (!device_is_ready(dev_i2c_i2cswitch.bus)) {
        printf("Device %s is not ready\n", dev_i2c_i2cswitch.bus->name);
        return 1;
    }
    // Set all sensors I2C disabled
    uint8_t config = 0xFF;
    ret += i2c_write_dt(&dev_i2c_i2cswitch, &config, sizeof(config));
    return ret;
}

int IOExp_Int_Handling(void) {
    int ret = 0;    // Return
    // if interrupt detected, read input registers to clear pin
    if (gpio_pin_get_dt(&IOExp_INTn) == 0) {
        int err;
        uint8_t input_reg_add = 0x00;
        uint8_t input_reg_data[2];

        err = i2c_write_read_dt(&dev_i2c_ioexp, &input_reg_add, 1, &input_reg_data, 2); 
        
        uint16_t input_states = ((uint16_t)input_reg_data[1] << 8) | input_reg_data[0];  

        printk("Handler detected interrupt. States: %i\n", input_states);

        if ((input_states >> 13) & 1) {  // IMU Interrupt
            k_msleep(50);    // Debounce
            // Read interrupt register to clear all interrupt flags
            uint8_t config[2];
            config[0] = 0x3B; // Register
            config[1] = 0x00; // Data
            err = i2c_write_read_dt(&dev_i2c_imu, config, 1, &config[1], 1); 

            k_msleep(50);   // Wait for IMU interrupt output to clear

            ret = 2;       // IMU has triggered the interrupt

        } else if (((input_states >> 4) & 1) & (((input_states >> 6) & 1)==0)) { // TOF Interrupt (TOF sensor on & active low interrupt == low)
            printk("Detected TOF interrupt\n");

            uint16_t dev = 0x0000;      // TOF device dummy
            //VL53L1X_StopRanging(dev);
            VL53L1X_ClearInterrupt(dev);

            k_msleep(50);   // Wait for TOF interrupt output to clear
            ret = 8;       // TOF has triggered the interrupt

        } else {
            ret = 1;           // Other interrupt has triggered
        }

        // Reset I/O Expander interrupt
        err = i2c_write_read_dt(&dev_i2c_ioexp, &input_reg_add, 1, &input_reg_data, 2); 
         

    } else {
        ret = 0;           // No interrupt triggered
    }

    return ret;
    
}

int Switch_SensI2C(const uint16_t sensor_mask, enum i2c_state state) {
    int ret = 0;
    uint8_t config = 0x00;

    //                       TMP   IMU   PMS   TOF   BIO   GPS   COL   BPS   AIR
    uint8_t i2c_on_lut[]  = {0x04, 0x10, 0x08, 0x02, 0x01, 0x20, 0x80, 0x40, 0x20};
    uint8_t i2c_off_lut[] = {0xFB, 0xEF, 0xF7, 0xFD, 0xFE, 0xDF, 0x7F, 0xBF, 0xDF};
    size_t lut_size = sizeof(i2c_on_lut)/sizeof(i2c_on_lut[0]);
    uint8_t *lut;

    if (state == I2C_ON)
        lut = i2c_on_lut;
    else
        lut = i2c_off_lut;

    ret += i2c_read_dt(&dev_i2c_i2cswitch, &config, sizeof(config));

    // Enable/disable I2C for selected sensors
    for (int i = 0; i < lut_size; i++) {
        if (sensor_mask & (1 << i)) {
            config = config | lut[i];
        }
    }

    ret += i2c_write_dt(&dev_i2c_i2cswitch, &config, sizeof(config));

    if (ret != 0) {
        printk("Failed to write to I2C device address %x. \n\r", dev_i2c_i2cswitch.addr);
        return 1;
    }
    if (state == I2C_ON) {
        // Wait until sensor have booted up
        k_msleep(50);
    }
    return 0;
}

int tof_enable_i2c(void) {
    int ret = 0;
    uint8_t config;
    ret += i2c_read_dt(&dev_i2c_i2cswitch, &config, sizeof(config));
    config = config | 0x02;
    ret += i2c_write_dt(&dev_i2c_i2cswitch, &config, sizeof(config));
    return ret;
}

int tof_disable_i2c(void) {
    int ret = 0;
    uint8_t config;
    ret += i2c_read_dt(&dev_i2c_i2cswitch, &config, sizeof(config));
    config = config & 0xFD;
    ret += i2c_write_dt(&dev_i2c_i2cswitch, &config, sizeof(config));
    return ret;
}

int tof_enable_power(void) {
    int ret = 0;
    uint8_t output_reg_add = 0x02;
    uint16_t output_reg_data;
    ret += i2c_write_read_dt(&dev_i2c_ioexp, &output_reg_add, 1, &output_reg_data, 2);
    output_reg_data = output_reg_data & 0xFFCF;
    uint8_t config[3] = {0x02, output_reg_data, output_reg_data >> 8}; // Set IO Directions (output reg, port 0, port 1)
    ret += i2c_write_dt(&dev_i2c_ioexp, config, sizeof(config));
    k_msleep(250); // Wait for sensors to boot up
    return ret;
}

int tof_disable_power(void) {
    int ret = 0;
    uint8_t output_reg_add = 0x02;
    uint16_t output_reg_data;
    ret += i2c_write_read_dt(&dev_i2c_ioexp, &output_reg_add, 1, &output_reg_data, 2);
    output_reg_data = output_reg_data | 0x0030;
    uint8_t config[3] = {0x02, output_reg_data, output_reg_data >> 8}; // Set IO Directions (output reg, port 0, port 1)
    ret += i2c_write_dt(&dev_i2c_ioexp, config, sizeof(config));
    return ret;
}

int power_sensors(uint16_t sensor_mask, enum power_state state) {
    int ret;
    uint16_t output_reg_data;
    uint8_t output_reg_add = 0x02;

    //                          TMP     IMU     PMS     TOF     BIO     GPS     MIC     COL     BPS     AIR
    uint16_t power_on_lut[]  = {0x0200, 0x1000, 0x0080, 0x0030, 0x0003, 0x4000, 0x0800, 0x0008, 0x0004, 0x4000};
    uint16_t power_off_lut[] = {0xFDFF, 0xEFFF, 0xFF7F, 0xFFCF, 0xFFFC, 0xBFFF, 0xF7FF, 0xFFF7, 0xFFFB, 0xBFFF};
    size_t lut_size = sizeof(power_on_lut)/sizeof(power_on_lut[0]);
    uint16_t *lut;

    if (state == POWER_ON)
        lut = power_on_lut;
    else
        lut = power_off_lut;

    ret += i2c_write_read_dt(&dev_i2c_ioexp, &output_reg_add, 1, &output_reg_data, 2); 

    // Enable/disable power for selected sensors
    for (int i = 0; i < lut_size; i++) {
        if (sensor_mask & (1 << i)) {
            output_reg_data = output_reg_data & lut[i];
        }
    }

    if (state == POWER_OFF) {
        // Wait for I2C communications in progress to finish
        k_msleep(100);
    }
    
    uint8_t config[3] = {0x02, output_reg_data, output_reg_data >> 8}; // Set IO Directions (output reg, port 0, port 1)
    ret += i2c_write_dt(&dev_i2c_ioexp, config, sizeof(config));
    if (ret != 0) {
            printk("Failed to write to I2C device address %x. \n\r", dev_i2c_ioexp.addr);
            return 1;
    }
    if (state == POWER_ON) {
        // Wait and clear IO expander interrupts triggered by sensor power-on
        k_msleep(5);
        IOExp_Int_Handling();
        // Wait until sensors have booted up
        k_msleep(250);
    }
    return 0;
}

int tof_init(void) {
    printk("TOF Init Start\n");

    int ret = 0;
    uint16_t dev = 0x0000;

    /* Wait for device booted */
    uint8_t state = 0;
    while(state == 0) {
            ret = VL53L1X_BootState(dev, &state);
            k_msleep(2);
    }

    // Initialize sensor to default values
    ret = ret + VL53L1X_SensorInit(dev);

    if(ret != 0) {
        printk("Failed to write to I2C device address %x. \n\r", dev_i2c_tof.addr);
        return 1;
    }

    VL53L1X_SetDistanceMode(dev, TOF.distance_mode);        // Short distance mode
    VL53L1X_SetInterMeasurementInMs(dev, TOF.intermeas_ms); // Interval between ranging operations (2000 ms)
    VL53L1X_SetTimingBudgetInMs(dev, TOF.timing_budget_ms); // Minimum timing budget for all distances (33 ms)
    VL53L1X_SetInterruptPolarity(dev, 0);                   // Interrupt active low
    VL53L1X_SetDistanceThreshold(dev, 
        TOF.distance_threshold_low, 
        TOF.distance_threshold_high, 
        TOF.distance_threshold_window, 0);                  // Trigger interrupt if object closer than 300 mm detected

    // Always-on mode
    if (SOC.processing==99) {
        VL53L1X_StartRanging(dev);                              // Starts continuous ranging
        VL53L1X_ClearInterrupt(dev);
    } else {
        VL53L1X_SetTimingBudgetInMs(dev, TOF.timing_budget_ms); // Minimum timing budget for all distances (200 ms)
    }
    //if (SOC.processing==99){
    //    VL53L1X_SetDistanceMode(dev, 1);                        // Short distance mode
    //    VL53L1X_SetInterMeasurementInMs(dev, 2000);             // Interval between ranging operations (2000 ms)
    //    VL53L1X_SetTimingBudgetInMs(dev, 20);                   // Minimum timing budget for all distances (33 ms)
    //    VL53L1X_SetInterruptPolarity(dev, 0);                   // Interrupt active low
    //    VL53L1X_SetDistanceThreshold(dev, 150, 1000, 0, 0);     // Trigger interrupt if object closer than 300 mm detected
    //    VL53L1X_StartRanging(dev);                              // Starts continuous ranging
    //    VL53L1X_ClearInterrupt(dev);
    //} else {
    //    VL53L1X_SetDistanceMode(dev, 2);                        // Long distance mode
    //    VL53L1X_SetTimingBudgetInMs(dev, 200);                  // Minimum timing budget for all distances (200 ms)
    //}

    printk("TOF Init End\n");

    return 0;

}


int tof_sample(void) {
    int ret = 0;
    uint16_t dev = 0x0000;

    if (SOC.processing != 99) {
        /* Wait for device booted */
        uint8_t state = 0;
        while(state == 0){
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

        /*uint64_t cycle_count = k_cycle_get_64();
        uint64_t *sample_data_64_ptr = (uint64_t*)&sample_data[sample_data_counter];
        *sample_data_64_ptr = cycle_count;
        sample_data_counter += sizeof(uint64_t)/sizeof(uint16_t);
        sample_data[sample_data_counter] = tof_distance;            // Distance in mm
        sample_data_counter += 1;*/

        //TODO: Use sample timer time
        //k_uptime_delta(&tof_timer);  // Reset and get elapsed time since last call
        // Store sensor data in sample buffer and increase data counter
        //uint64_t cycle_count = sys_clock_cycle_get_64();  //! Does not compile
        //uint64_t cycle_count = k_uptime_ticks();          //* Works, but large numbers (9.6e18 delta)
        uint64_t cycle_count = k_uptime_get();              //* Works, but large numbers (1.2e18 delta)
        //uint64_t cycle_count = sys_clock_tick_get();      //* Works, but large numbers (2.9e18 delta)
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

int sensors_init(void){
    k_msleep(10);   // Wait for sensor board voltage to stabilize 

    io_expander_init();   // Initialize the IO Expander

    // Turn off all sensors
    power_sensors(
        SENS_TMP_MASK |
        SENS_IMU_MASK |
        SENS_PMS_MASK |
        SENS_TOF_MASK |
        SENS_BIO_MASK |
        SENS_GPS_MASK |
        SENS_COL_MASK |
        SENS_BPS_MASK |
        SENS_AIR_MASK, POWER_OFF
    );
    // Turn off all sensor I2C buses
    Switch_SensI2C(
        SENS_TMP_MASK |
        SENS_IMU_MASK |
        SENS_PMS_MASK |
        SENS_TOF_MASK |
        SENS_BIO_MASK |
        SENS_GPS_MASK |
        SENS_COL_MASK |
        SENS_BPS_MASK |
        SENS_AIR_MASK, I2C_OFF
    );

    // Time of Flight Sensor
    if (SOC.sensor_type & SENS_TOF_MASK) {
        // Init always-on sensor, otherwise (one-shot) init is handled in sensors_sample()
        if (SOC.processing == 99) {
            power_sensors(SENS_TOF_MASK, 1);
            Switch_SensI2C(SENS_TOF_MASK, 1);
            tof_init();
        }
    }

    // Check if I/O expander interrupt has triggered
    k_msleep(50);
    IOExp_Int_Handling();
    
    return 0;

}

int sensors_sample(uint16_t Sensor_Type) {
    if (Sensor_Type == 0) {
        for (sample_data_counter = 0; sample_data_counter < 100; sample_data_counter++) {
            sample_data[sample_data_counter] = test_data;
            sample_data_counter = sample_data_counter + 1;
            test_data ++;
        }
    }

    // Time of Flight Sensor
    if (Sensor_Type & SENS_TOF_MASK) {
        if (SOC.processing!=99) {        // One-shot sampling
            power_sensors(SENS_TOF_MASK, 1);
            Switch_SensI2C(SENS_TOF_MASK, 1);
            tof_init();
            tof_sample();
            Switch_SensI2C(SENS_TOF_MASK, 0);
            power_sensors(SENS_TOF_MASK, 0);
            k_msleep(350);   // wait for possible interrupts to be cleared subsequently
        } else {
            tof_sample();
        }
    }

    sample_packets_counter += 1;

    // Check if I/O expander interrupt has triggered
    k_msleep(200);
    IOExp_Int_Handling();

    return 0;

}

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