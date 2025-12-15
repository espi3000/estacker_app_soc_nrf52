
#include "sensors.h"
#include "main.h"
#include "VL53L1X_api.h"
#include "opt4060.h"
#include "dps368.h"
#include "gpios.h"               // I2C TEST
#define BME68X_USE_FPU
#include "bme68x.h"
#include "bme68x_common.h"

#include <arm_math.h>
#include <arm_const_structs.h>

#include <zephyr/kernel.h>
#include <zephyr/audio/dmic.h>

//#include "ai.h"


// Checking audio samples
#include <zephyr/drivers/uart.h>
#include "uart.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(Sensor, CONFIG_AUDIO_DMIC_LOG_LEVEL);

// PDM defines for the microphone
#define MAX_SAMPLE_RATE  16000
#define SAMPLE_BIT_WIDTH 16
#define BYTES_PER_SAMPLE sizeof(int16_t)
/* Milliseconds to wait for a block to be read. */
#define READ_TIMEOUT     1000

/* Size of a block for 100 ms of audio data. */
#define BLOCK_SIZE(_sample_rate, _number_of_channels) \
	(BYTES_PER_SAMPLE * (_sample_rate / 5) * _number_of_channels)

/* Driver will allocate blocks from this slab to receive audio data into them.
 * Application, after getting a given block from the driver and processing its
 * data, needs to free that block.
 */
#define MAX_BLOCK_SIZE   BLOCK_SIZE(MAX_SAMPLE_RATE, 1)
#define BLOCK_COUNT      3
//K_MEM_SLAB_DEFINE(mem_slab, MAX_BLOCK_SIZE, BLOCK_COUNT, 4);

struct dmic_cfg cfg;    // PDM driver configuration struct

// PDM defines end

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
const struct i2c_dt_spec dev_i2c_tmp = I2C_DT_SPEC_GET(I2C0_TMP);
const struct i2c_dt_spec dev_i2c_imu = I2C_DT_SPEC_GET(I2C0_IMU);
const struct i2c_dt_spec dev_i2c_pms = I2C_DT_SPEC_GET(I2C0_PMS);
const struct i2c_dt_spec dev_i2c_tof = I2C_DT_SPEC_GET(I2C0_TOF);
const struct i2c_dt_spec dev_i2c_bio = I2C_DT_SPEC_GET(I2C0_BIO);
const struct i2c_dt_spec dev_i2c_gps = I2C_DT_SPEC_GET(I2C0_GPS);
const struct i2c_dt_spec dev_i2c_col = I2C_DT_SPEC_GET(I2C0_COL);
const struct i2c_dt_spec dev_i2c_bps = I2C_DT_SPEC_GET(I2C0_BPS);
const struct i2c_dt_spec dev_i2c_air = I2C_DT_SPEC_GET(I2C0_AIR);
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//! AIR and BPS sensors both have 0x76 I2C address. Both have 0x77 as an 
//! alternative address, but this is the current I2C switch address.
//! Suggested solution: 
//!     Step 1: Change BPS I2C address to 0x77
//!     Step 2: Change the I2C address of the switch to one of the following:
//!
//! +----------+-------------+
//! | A2 A1 A0 | I2C Address |
//! +----------+-------------+
//! |  0  0  0 | 0x70        |
//! |  0  0  1 | 0x71        |
//! |  0  1  0 | 0x72        |
//! |  0  1  1 | 0x73        |
//! |  1  0  1 | 0x75        |
//! +----------+-------------+
//!
#warning "AIR sensor has the same I2C address as BPS"
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
const struct i2c_dt_spec dev_i2c_generalcall = I2C_DT_SPEC_GET(I2C0_GeneralCall);
//const struct device *const dmic_dev = DEVICE_DT_GET(DT_NODELABEL(dmic_dev));



// SPI

#define SPI_BUFSIZE  8   //SPI Communication buffer size
#define SPI_INSTANCE  0 //SPI Instance to be used

uint8_t   spi_tx_buf[SPI_BUFSIZE]; // spi tx buffer 
uint8_t   spi_rx_buf[SPI_BUFSIZE]; // spi rx buffer

//spi_buf and spi_buf_set structures to hold the buffer values for SPI
struct spi_buf tx_buf_arr; 
struct spi_buf_set tx;
struct spi_buf rx_buf_arr;
struct spi_buf_set rx ;

volatile  uint8_t   SPIReadLength, SPIWriteLength; // variables to hold read and write lengths


const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi2));
struct spi_config spi_cfg;
struct spi_cs_control chip;

// ---




uint16_t sample_data[512];   // Sample data buffer
uint16_t sample_data_counter;     // Sample data counter
uint16_t sample_packets_counter;     // Number of sample packets. A sample packet can consist of the measurements of several sensors

uint16_t test_data;         // Send test_data if sensor type is 0





void SPI_Init(void)
{
	if (!device_is_ready(spi_dev)) {
        /* Device is not ready to use */
		printk("\r\nStop device not ready\r\n");
	}
    
    spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA;
    spi_cfg.frequency = 4000000;
    spi_cfg.slave = 0;
    //spi_cfg.cs.gpio = SPI2_CS_DT_SPEC;
    spi_cfg.cs.gpio.port = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    spi_cfg.cs.gpio.pin = 11;  
    spi_cfg.cs.gpio.dt_flags = GPIO_ACTIVE_LOW;  
    spi_cfg.cs.delay = 0;
}

void AD7091_write_reg(uint8_t MSB, uint8_t LSB)
{
    SPIWriteLength = 2; // set the spi write length to 2 bytes
    SPIReadLength = 0; // set the read length
    
    spi_tx_buf[0] = MSB; // set the first byte which is a write command
    spi_tx_buf[1] = LSB; // A byte of data to be sent
	

   //specifying the tx and rx buffer specific to zephyr's SPI drivers
   	tx_buf_arr.buf = spi_tx_buf;
    tx_buf_arr.len = SPIWriteLength;
    tx.buffers = &tx_buf_arr;
    tx.count = 1;
	
    rx_buf_arr.buf = spi_rx_buf;
    rx_buf_arr.len = SPIReadLength;

    rx.buffers = &rx_buf_arr ;
    rx.count = 1;

	int error = spi_transceive(spi_dev, &spi_cfg, &tx,&rx);
	if(error != 0){
		printk("SPI transceive error: %i\n", error);
	}

    
}




int adc3_init() {
    
    return 0;
}

// Steps for EH OC sampling:
// Init ADC + Set ADC GPO1 High
// SoC GPIO: Disable MPPT bypass (3 GPIOs total)
// Sample ADC and read result(s), power down ADC
// Reset Bypass GPIOs




/*
int do_pdm_transfer(const struct device *dmic_dev, struct dmic_cfg *cfg, size_t block_count)
{
	int ret;

    printk("Starting PDM");

	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_START);
	if (ret < 0) {
		printk("START trigger failed: %d", ret);
		return ret;
	}

    printk("Triggered PDM\n");

    void *buffer[3];
    uint32_t size;
    uint16_t discarded_buffers = 3;
    uint8_t buffer_cnt = 0;

    for (int i=0; i<3+discarded_buffers; i++){

        if (buffer_cnt > 2){
            buffer_cnt = 0;
        }
        ret = dmic_read(dmic_dev, 0, &buffer[buffer_cnt], &size, READ_TIMEOUT);
        printk("%d - got buffer %p of %u bytes\n", i, buffer[buffer_cnt], size);
        k_mem_slab_free(&mem_slab, buffer[buffer_cnt]);
        buffer_cnt ++;
        
    }


	//int err = ei_wrapper_add_data(&input_data[cnt], ei_wrapper_get_window_size());
	//if (err) {
	//	printk("Cannot provide input data (err: %d)\n", err);
	//	printk("Increase CONFIG_EI_WRAPPER_DATA_BUF_SIZE\n");
	//	return;
	//}



	ret = dmic_trigger(dmic_dev, DMIC_TRIGGER_STOP);
	if (ret < 0) {
		printk("STOP trigger failed: %d", ret);
		return ret;
	}

    k_msleep(3000);

     run_classifier_custom((int16_t*)buffer[0]);


    // COMMENT OUT (TEST):

    const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
    activate_uart_callback();       // activate UART
    

    // Transmit samples via UART
    uint8_t tx_buf[2];    
    for (int i=0; i<3; i++){
        printk("Data: %i, address: %p\n", *(int16_t*)buffer[i], (int16_t*)buffer[i]);
        for (int j=0; j<MAX_BLOCK_SIZE/2; j++){
            memcpy(&tx_buf[0], (uint8_t*)buffer[i]+2*j, 1);
            memcpy(&tx_buf[1], (uint8_t*)buffer[i]+2*j+1, 1);
            uart_tx(uart, tx_buf, 2, SYS_FOREVER_MS);
            //printk("Buffer: %i, block index: %i\n", i, j);
            k_msleep(1);
        }
    }

    printk("UART TX Done\n");

    // COMMENT OUT (TEST) DONE



	return ret;
}




int mic_init(const struct device *dmic_dev){

    int ret;

	printk("DMIC sample\n");

	if (!device_is_ready(dmic_dev)) {
		printk("%s is not ready", dmic_dev->name);
		return 0;
	}

	struct pcm_stream_cfg stream = {
		.pcm_width = SAMPLE_BIT_WIDTH,
		.mem_slab  = &mem_slab,
	};

    cfg.io.min_pdm_clk_freq = 1000000;
    cfg.io.max_pdm_clk_freq = 3500000;
	cfg.io.min_pdm_clk_dc   = 40;
	cfg.io.max_pdm_clk_dc   = 60;

    cfg.channel.req_num_chan = 1;
    cfg.channel.req_num_streams = 1;
    cfg.channel.req_chan_map_lo = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);

    cfg.streams = &stream;
	cfg.streams[0].pcm_rate = MAX_SAMPLE_RATE;
	cfg.streams[0].block_size =	MAX_BLOCK_SIZE;  // BLOCK_SIZE(cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);


	printk("PCM output rate: %u, channels: %u",
		cfg.streams[0].pcm_rate, cfg.channel.req_num_chan);


	ret = dmic_configure(dmic_dev, &cfg);
	if (ret < 0) {
		printk("Failed to configure the driver: %d", ret);
		return 0;
	} else {
        printk("Configured PCM\n");
    }

    int err = ei_wrapper_init(result_ready_cb);

	if (err) {
		printk("Edge Impulse wrapper failed to initialize (err: %d)\n",
		       err);
		return;
	};

	printk("Machine learning model sampling frequency: %zu\n",
	       ei_wrapper_get_classifier_frequency());
	printk("Labels assigned by the model:\n");
	for (size_t i = 0; i < ei_wrapper_get_classifier_label_count(); i++) {
		printk("- %s\n", ei_wrapper_get_classifier_label(i));
	}

    printk("Window size: %d\n", ei_wrapper_get_window_size());
	printk("Wrapper init done\n");

    return 0;

}

*/


// Configure IO Directions of I/O expander responsible for sensor power & interrupts
int IOExp_init(void){
    int ret = 0;
    if (!device_is_ready(dev_i2c_ioexp.bus)) {
        printf("Device %s is not ready\n", dev_i2c_ioexp.bus->name);
        return 0;
    }
    uint8_t config1[3] = {0x02,0x00, 0x00}; // Set IO outputs to low
    ret =  ret + i2c_write_dt(&dev_i2c_ioexp, config1, sizeof(config1));

    // Set IO Directions
    if (BOARD_V1_2){
        config1[0] = 0x06; 
        config1[1] = 0x40;
        config1[2] = 0x25;
    } else {
        config1[0] = 0x06; 
        config1[1] = 0x4C;
        config1[2] = 0x25;
        // Set BIO_RSTn to input due to missing 3.3 V -> 1.8 V conversion
    }
    
    ret =  ret + i2c_write_dt(&dev_i2c_ioexp, config1, sizeof(config1));

    if(ret != 0){
            printk("Failed to write to I2C device address %x. \n\r", dev_i2c_ioexp.addr);
            return 1;
    }
    return 0;
}

int IOExp_Int_Handling(void){
    int ret = 0;    // Return
    // if interrupt detected, read input registers to clear pin
    if (gpio_pin_get_dt(&IOExp_INTn) == 0){
        int err;
        uint8_t input_reg_add = 0x00;
        uint8_t input_reg_data[2];

        err = i2c_write_read_dt(&dev_i2c_ioexp, &input_reg_add, 1, &input_reg_data, 2); 
        
        uint16_t input_states = ((uint16_t)input_reg_data[1] << 8) | input_reg_data[0];  

        printk("Handler detected interrupt. States: %i\n", input_states);

        if ((input_states >> 13) & 1){  // IMU Interrupt
            k_msleep(50);    // Debounce
            // Read interrupt register to clear all interrupt flags
            uint8_t config[2];
            config[0] = 0x3B; // Register
            config[1] = 0x00; // Data
            err = i2c_write_read_dt(&dev_i2c_imu, config, 1, &config[1], 1); 

            k_msleep(50);   // Wait for IMU interrupt output to clear

            ret = 2;       // IMU has triggered the interrupt

        } else if (((input_states >> 4) & 1) & (((input_states >> 6) & 1)==0)) // TOF Interrupt (TOF sensor on & active low interrupt == low)
        {
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

int Switch_SensI2C(const uint16_t Sensor_Type, const uint8_t I2C_On){
    int ret = 0;
    uint8_t config = 0x00;

    ret =  ret + i2c_read_dt(&dev_i2c_i2cswitch, &config, sizeof(config));

    // Enable I2C to sensor
    if (I2C_On){
        if (Sensor_Type & SENS_TMP_MASK) {
            config = config | 0x04;     // Turn on I2C to temperature sensor   
        }
        if (Sensor_Type & SENS_IMU_MASK) {
            config = config | 0x10; // Turn on I2C to intertial measurement unit
        }
        if (Sensor_Type & SENS_PMS_MASK) {
            config = config | 0x08; // Turn on I2C to presence and motion sensor
        }
        if (Sensor_Type & SENS_TOF_MASK) {
            config = config | 0x02; // Turn on I2C to time of flight sensor
        }
        if (Sensor_Type & SENS_BIO_MASK) {
            config = config | 0x01; // Turn on I2C to bio sensor + booster simultaneously
        }
        if ((Sensor_Type & SENS_GPS_MASK) && !BOARD_V1_2) {
            config = config | 0x20; // Turn on I2C to GPS
        } 
        if ((Sensor_Type & SENS_COL_MASK) && BOARD_V1_2) {
            config = config | 0x80; // Turn on I2C to color sensor
        }
        if ((Sensor_Type & SENS_BPS_MASK) && BOARD_V1_2) {
            config = config | 0x40; // Turn on I2C to pressure sensor
        }
        if ((Sensor_Type & SENS_AIR_MASK) && BOARD_V1_2) {
            config = config | 0x20; // Turn on I2C to air quality sensor
        }
    } else { 
        if (Sensor_Type & SENS_TMP_MASK) {
            config = config & 0xFB;     // Turn off I2C to temperature sensor
        }
        if (Sensor_Type & SENS_IMU_MASK) {
            config = config & 0xEF; // Turn off I2C to intertial measurement unit
        }
        if (Sensor_Type & SENS_PMS_MASK) {
            config = config & 0xF7; // Turn off I2C to presence and motion sensor  
        }
        if (Sensor_Type & SENS_TOF_MASK) {
            config = config & 0xFD; // Turn off I2C to time of flight sensor 
        }
        if (Sensor_Type & SENS_BIO_MASK) {
            config = config & 0xFE; // Turn off I2C to bio sensor + booster simultaneously
        }
        if ((Sensor_Type & SENS_GPS_MASK) && !BOARD_V1_2) {
            config = config & 0xDF; // Turn off I2C to GPS 
        } 
        if ((Sensor_Type & SENS_COL_MASK) && BOARD_V1_2) {
            config = config & 0x7F; // Turn off I2C to color sensor
        }
        if ((Sensor_Type & SENS_BPS_MASK) && BOARD_V1_2) {
            config = config & 0xBF; // Turn off I2C to pressure sensor
        }
        if ((Sensor_Type & SENS_AIR_MASK) && BOARD_V1_2) {
            config = config & 0xDF; // Turn off I2C to air quality sensor
        }
    }

    //printk(config);
    ret =  ret + i2c_write_dt(&dev_i2c_i2cswitch, &config, sizeof(config));

    if(ret != 0){
            printk("Failed to write to I2C device address %x. \n\r", dev_i2c_i2cswitch.addr);
            return 1;
    }
    if (I2C_On){
        // Wait until sensor have booted up
        k_msleep(50);
    }


    return 0;
}

int power_sensors(uint16_t Sensor_Type, uint8_t Power_On){
    int ret;
    uint16_t output_reg_data;
    uint8_t output_reg_add = 0x02;

    ret = i2c_write_read_dt(&dev_i2c_ioexp, &output_reg_add, 1, &output_reg_data, 2); 

    // Turn on sensor
    if (Power_On){
        if (Sensor_Type & SENS_TMP_MASK) {
            output_reg_data = output_reg_data | 0x0200; // Turn on temperature sensor
            
        }
        if (Sensor_Type & SENS_IMU_MASK) {
            output_reg_data = output_reg_data | 0x1000; // Turn on intertial measurement unit
            
        }
        if (Sensor_Type & SENS_PMS_MASK) {
            output_reg_data = output_reg_data | 0x0080; // Turn on presence and motion sensor
            
        }
        if (Sensor_Type & SENS_TOF_MASK) {
            output_reg_data = output_reg_data | 0x0030; // Turn on time of flight sensor + !TOF_SHUTDN
            
        }
        if (Sensor_Type & SENS_BIO_MASK) {
            output_reg_data = output_reg_data | 0x0003; // Turn on bio sensor + booster simultaneously
            
        }
        if ((Sensor_Type & SENS_GPS_MASK) && !BOARD_V1_2) {
            output_reg_data = output_reg_data | 0x4000; // Turn on GPS
            
        }
        if (Sensor_Type & SENS_MIC_MASK) {
            output_reg_data = output_reg_data | 0x0800; // Turn on microphone
            
        }
        if ((Sensor_Type & SENS_COL_MASK) && BOARD_V1_2) {
            output_reg_data = output_reg_data | 0x0008; // Turn on color sensor
        }

        if (Sensor_Type & SENS_BPS_MASK) {
            output_reg_data = output_reg_data | 0x0004; // Turn on pressure sensor    
        }
        if ((Sensor_Type & SENS_AIR_MASK) && BOARD_V1_2) {
            output_reg_data = output_reg_data | 0x4000; // Turn on air quality sensor    
        }
    } else { // Turn off sensor
        if (Sensor_Type & SENS_TMP_MASK) {
            output_reg_data = output_reg_data & 0xFDFF; // Turn off temperature sensor
        }
        if (Sensor_Type & SENS_IMU_MASK) {
            output_reg_data = output_reg_data & 0xEFFF; // Turn off intertial measurement unit
        }
        if (Sensor_Type & SENS_PMS_MASK) {
            output_reg_data = output_reg_data & 0xFF7F; // Turn off presence and motion sensor
        }
        if (Sensor_Type & SENS_TOF_MASK) {
            output_reg_data = output_reg_data & 0xFFCF; // Turn off time of flight sensor + !TOF_SHUTDN
        }
        if (Sensor_Type & SENS_BIO_MASK) {
            output_reg_data = output_reg_data & 0xFFFC; // Turn off bio sensor + booster simultaneously
        }
        if ((Sensor_Type & SENS_GPS_MASK) && !BOARD_V1_2) {
            output_reg_data = output_reg_data & 0xBFFF; // Turn off GPS
        }
        if (Sensor_Type & SENS_MIC_MASK) {
            output_reg_data = output_reg_data & 0xF7FF; // Turn off microphone
        }
        if ((Sensor_Type & SENS_COL_MASK) && BOARD_V1_2) {
            output_reg_data = output_reg_data & 0xFFF7; // Turn off color sensor
        }
        if (Sensor_Type & SENS_BPS_MASK) {
            output_reg_data = output_reg_data & 0xFFFB; // Turn off pressure sensor
        }
        if ((Sensor_Type & SENS_AIR_MASK) && BOARD_V1_2) {
            output_reg_data = output_reg_data & 0xBFFF; // Turn off air quality sensor
        }
    }

    if (Power_On == 0){
        // Wait for I2C communications in progress to finish
        k_msleep(100);
    }
    
    uint8_t config1[3] = {0x02, output_reg_data, output_reg_data >> 8}; // Set IO Directions (output reg, port 0, port 1)
    
    ret =  ret+ i2c_write_dt(&dev_i2c_ioexp, config1, sizeof(config1));
    if(ret != 0){
            printk("Failed to write to I2C device address %x. \n\r", dev_i2c_ioexp.addr);
            return 1;
    }
    if (Power_On){
        // Wait and clear IO expander interrupts triggered by sensor power-on
        k_msleep(5);
        IOExp_Int_Handling();
        // Wait until sensors have booted up
        k_msleep(250);
    }
    return 0;
}

int tmp_init(void){
    
    // This only has to be done once since data is stored in EEPROM!
    int ret;
    uint8_t config1[3] = {0x04,0x80, 0x00}; // Unlock EEPROM
    ret = i2c_write_dt(&dev_i2c_tmp, config1, sizeof(config1));
    
    uint8_t config2[3] = {0x01,0x04, 0x20}; // Mode: Shutdown, 8-bit avaraging
    ret = ret + i2c_write_dt(&dev_i2c_tmp, config2, sizeof(config2));

    uint8_t config3[1] = {0x06}; // General Call reset
    ret = ret + i2c_write_dt(&dev_i2c_tmp, config3, sizeof(config3));

    if(ret != 0){
            printk("Failed to write to I2C device address %x. \n\r", dev_i2c_tmp.addr);
            return 1;
    }
    return 0;
}

int tmp_sample(void){
    uint8_t config1[3] = {0x01, 0x0C, 0x20}; // Conversion mode: Single-shot, 8-bit averaging
    int ret;
    ret = i2c_write_dt(&dev_i2c_tmp, config1, sizeof(config1));
    k_msleep(1);

   /*
   // Not using Data_Ready polling (not reliable)
    // Wait until Data_Ready bit is set
    while (((config_val>>13) & 1) == 0)
    {
        ret = ret + i2c_write_read_dt(&dev_i2c_tmp, &config_add, 1, &config_val, 2); 
        config_val = (config_val>>8) | (config_val<<8);
        k_msleep(10);
    }
    */
    
    // Wait for max. conversion time with averaging == 8
    k_msleep(150);

    // Read temperature value
    uint16_t tmp_val;
    uint8_t tmp_add = 0x00;
    ret = ret + i2c_write_read_dt(&dev_i2c_tmp, &tmp_add, 1, &tmp_val, 2); 

    if(ret != 0){
        printk("Failed to write to I2C device address %x. \n\r", dev_i2c_tmp.addr);
        return 1;
    }

    // Store sensor data in sample buffer and increase data counter
    sample_data[sample_data_counter] = tmp_val; // (tmp_val>>8) | (tmp_val<<8);
    sample_data_counter = sample_data_counter + 1;

    return 0;
}


int imu_init(void){

    int ret = 0;
    uint8_t config[2] = {0x00, 0x00}; 

    // Soft reset device
    config[0] = 0x02; // Register
    config[1] = 0x10; // Data
    ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));
    k_msleep(1);

    // Gyro full scale: +/-250 dps, ODR: 25 Hz
    config[0] = 0x20; // Register
    config[1] = 0x6B; // Data
    ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

    // Accelerometer full scale: +/-2 g, ODR: 400 Hz
    config[0] = 0x21; // Register
    config[1] = 0x67; // Data
    ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

    k_msleep(50);

    if(ret != 0){
        printk("Failed to write to I2C device address %x. \n\r", dev_i2c_imu.addr);
        return 1;
    }

    if (SOC.processing == 2 || SOC.processing == 10){   // FFT 

        // Power Management Config (Idle mode (RC oscillator on))
        config[0] = 0x1F; // Register
        config[1] = 0x10; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // Accelerometer: 180 Hz filter, 32x averaging
        config[0] = 0x24; // Register
        config[1] = 0x41; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // MREG1 Address (SENSOR_CONFIG3)
        config[0] = 0x7A; // Register
        config[1] = 0x06; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // APEX_DISABLE (2.25 kbyte FIFO instead of 1k)
        config[0] = 0x7B; // Register
        config[1] = 0x40; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));
        k_msleep(1);

        // MREG1 Address (TMST_CONFIG1)
        config[0] = 0x7A; // Register
        config[1] = 0x00; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // Disable timestamps
        config[0] = 0x7B; // Register
        config[1] = 0x00; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));
        k_msleep(1);

        // MREG1 Address (FIFO_CONFIG5)
        config[0] = 0x7A; // Register
        config[1] = 0x01; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // FIFO_ACCEL_EN (Enable accelerometer data to be stored in FIFO (no gyro))
        config[0] = 0x7B; // Register
        config[1] = 0x01; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));
        k_msleep(1);

        // FIFO in stop-mode, don't bypass FIFO
        config[0] = 0x28; // Register
        config[1] = 0x02; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // Power Management Config (all off)
        config[0] = 0x1F; // Register
        config[1] = 0x00; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

    }

    // Always-on mode
    if (SOC.processing==99){

        // Power Management Config (Idle mode (RC oscillator on))
        config[0] = 0x1F; // Register
        config[1] = 0x10; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // Accelerometer: 180 Hz filter, 8x averaging
        config[0] = 0x24; // Register
        config[1] = 0x21; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // MREG1 Address (ACCEL_WOM_X_THR)
        config[0] = 0x7A; // Register
        config[1] = 0x4B; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // WOM X Threshold = 195 mg
        config[0] = 0x7B; // Register
        config[1] = 0x32; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));
        k_msleep(1);

        // MREG1 Address (ACCEL_WOM_Y_THR)
        config[0] = 0x7A; // Register
        config[1] = 0x4C; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // WOM Y Threshold = 195 mg
        config[0] = 0x7B; // Register
        config[1] = 0x32; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));
        k_msleep(1);
        
        // MREG1 Address (ACCEL_WOM_Z_THR)
        config[0] = 0x7A; // Register
        config[1] = 0x4D; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // WOM Z Threshold = 195 mg
        config[0] = 0x7B; // Register
        config[1] = 0x32; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));
        k_msleep(1);

        // Power Management Config (Accelerometer in duty-cycled low-power mode (mode 3))
        config[0] = 0x1F; // Register
        config[1] = 0x02; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // Configure interrupt 1 (Latched, push-pull, active high)
        config[0] = 0x06; // Register
        config[1] = 0x07; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // Configure Source for Interrupt 1 (XYZ WOM interrupts)
        config[0] = 0x2C; // Register
        config[1] = 0x07; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // Enable wake-on-motion (compare to initial sample, interrupt on any thresholds, interrupt on first event)
        config[0] = 0x27; // Register
        config[1] = 0x01; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        // Read interrupt register to clear all interrupt flags
        config[0] = 0x3B; // Register
        config[1] = 0x00; // Data
        ret = i2c_write_read_dt(&dev_i2c_imu, config, 1, &config[1], 1); 

    }

    return 0;
}

int imu_sample(void){

    int ret = 0;
    uint8_t config[2] = {0x00, 0x00}; 

    if (SOC.processing == 0 || SOC.processing == 1){   // IMU raw data

        // Turn on gyroscope and accelerometer in Low-Noise mode
        config[0] = 0x1F; // Register
        config[1] = 0x0F; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));
        // WAIT for >45 ms (according to register description datasheet)
        k_msleep(50);

        uint8_t data_ready_reg = 0x39;
        uint8_t data_ready_val = 0x00;

        // Wait until Data_Ready bit is set
        while ((data_ready_val & 1) == 0)
        {
            ret = ret + i2c_write_read_dt(&dev_i2c_imu, &data_ready_reg, 1, &data_ready_val, 1); 
            k_msleep(1);
        }

        // Read IMU data
        uint8_t imu_data_add = 0x09;    // Address of temp sensor register. Other sensor data follows
        volatile uint16_t imu_data[7];
        volatile uint16_t temp_data;
        volatile uint16_t accel_data[3];
        volatile uint16_t gyro_data[3];

        ret = ret + i2c_write_read_dt(&dev_i2c_imu, &imu_data_add, 1, (void*)imu_data, 14); 

        // store received data in separate variables
        temp_data = imu_data[0];
        for (int i = 0; i<3; i++){
                accel_data[i] = imu_data[i+1];
                gyro_data[i] = imu_data[i+4];
        }

        // Turn off gyroscope and accelerometer
        config[0] = 0x1F; // Register
        config[1] = 0x00; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        if(ret != 0){
            printk("Failed to write to I2C device address %x. \n\r", dev_i2c_imu.addr);
            return 1;
        }

        // Store sensor data in sample buffer and increase data counter
        sample_data[sample_data_counter] = temp_data;
        sample_data[sample_data_counter +1] = accel_data[3];
        sample_data[sample_data_counter +4] = gyro_data[3];

        sample_data_counter = sample_data_counter + 7;

    } else if(SOC.processing == 2 || SOC.processing == 10) {    // IMU with FFT processing (also with event traces (Proc==10))


        // Flush FIFO
        config[0] = 0x02; // Register
        config[1] = 0x04; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));
        // WAIT for >1.5us
        k_msleep(1);

        // Power Management Config (Accelerometer in duty-cycled low-power mode (mode 3))
        config[0] = 0x1F; // Register
        config[1] = 0x83; //0x02; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        k_msleep(1000);

        // Wait for FIFO full interrupt
        uint8_t INT_Status_reg = 0x3A;
        uint8_t INT_Status_val = 0x00;
        while (((INT_Status_val >> 1) & 1) == 0)
        {
            ret = ret + i2c_write_read_dt(&dev_i2c_imu, &INT_Status_reg, 1, &INT_Status_val, 1); 
            k_msleep(5);
        }

        // Read IMU data
        uint16_t num_imu_packets = 256;     // number of samples to be transferred
        uint8_t FIFO_add = 0x3F;    // FIFO address
        uint8_t imu_data[8];
        //uint8_t FIFO_header[num_imu_packets];
        float32_t accel_data[3][num_imu_packets];   // x,y,z accelerometer data
        //uint8_t temp_data[num_imu_packets];
        

        for (int i = 0; i<num_imu_packets; i++){
            ret = ret + i2c_write_read_dt(&dev_i2c_imu, &FIFO_add, 1, &imu_data, 8); 
            // store received data in separate variables
            //FIFO_header[i] = imu_data[0];
            accel_data[0][i] = (float32_t)((imu_data[1] << 8) | imu_data[2]);  
            accel_data[1][i] = (float32_t)((imu_data[3] << 8) | imu_data[4]);
            accel_data[2][i] = (float32_t)((imu_data[5] << 8) | imu_data[6]);
            //temp_data[i] = imu_data[7];         // Temperature = temp_data/128 + 25°C
        }

        // Turn off gyroscope and accelerometer
        config[0] = 0x1F; // Register
        config[1] = 0x00; // Data
        ret = ret + i2c_write_dt(&dev_i2c_imu, config, sizeof(config));

        if(ret != 0){
           printk("Failed to write to I2C device address %x. \n\r", dev_i2c_imu.addr);
        return 1;
    }

        // Calculate FFT -----

        state = PROCESSING;
        state_logger_set(state);
        k_msleep(1);

        float32_t FFT_output[128];
        float32_t FFT_max_mag;
        uint32_t FFT_max_index;

        arm_cfft_instance_f32 varInstCfftF32;       // FFT instance

        arm_status status;
        status = ARM_MATH_SUCCESS;

        status=arm_cfft_init_f32(&varInstCfftF32,128);  // initialize FFT instance

        // For each 3 axes (x,z,y)
        for (int i=0; i<3; i++){
            // Process the data through the CFFT/CIFFT module
            arm_cfft_f32(&varInstCfftF32, accel_data[i], 0, 0);

            // Process the data through the Complex Magnitude Module for
            //calculating the magnitude at each bin
            arm_cmplx_mag_f32(accel_data[i], FFT_output, 128);

            // Find 3 maxima in FFT
            for (int j=0; j<3; j++){

                // Calculates maxValue and returns corresponding BIN value
                arm_max_f32(FFT_output, 128, &FFT_max_mag, &FFT_max_index);

                // Store in sample_data
                // Store sensor data in sample buffer and increase data counter
                sample_data[sample_data_counter] = FFT_max_mag;
                sample_data[sample_data_counter +1] = FFT_max_index;
                sample_data_counter = sample_data_counter + 2;

                FFT_output[FFT_max_index] = 0;  // Set current maximum 0 to find next maximum
            }
            
        }
    } else if (SOC.processing == 99) {  // Always-on mode
        
        if (IOExp_Int_Handling() == 2){     // Check for interrupts. If IMU interrupt has triggered:
            sample_data[sample_data_counter] = 1;   // Register that event has been detected;
        } else {
            sample_data[sample_data_counter] = 0;   // Register that no event has been detected;
        }
        sample_data_counter = sample_data_counter + 1;
            
    }

    return 0;

}


int pms_sample(void){
    int ret = 0;
    uint8_t config[2] = {0x00, 0x00}; 

    // Enable block data update
    config[0] = 0x20; // Register
    config[1] = 0x10; // Data
    ret = ret + i2c_write_dt(&dev_i2c_pms, config, sizeof(config));
    k_msleep(1);

    // Trigger One-Shot Acquisition
    config[0] = 0x21; // Register
    config[1] = 0x01; // Data
    ret = ret + i2c_write_dt(&dev_i2c_pms, config, sizeof(config));
    k_msleep(25);

    uint8_t data_ready_reg = 0x23;
    uint8_t data_ready_val = 0x00;


    // Wait until Data_Ready bit is set
    while (((data_ready_val >> 2) & 1) == 0)
    {
        ret = ret + i2c_write_read_dt(&dev_i2c_pms, &data_ready_reg, 1, &data_ready_val, 1); 
        k_msleep(1);
    }


    // Read PMS data (object and ambient temperature)
    uint8_t pms_data_add = 0x26;    
    uint8_t pms_data;  
    volatile uint16_t tobj_L_data;
    volatile uint16_t tobj_H_data;
    volatile uint16_t tamb_L_data;
    volatile uint16_t tamb_H_data;

    ret = ret + i2c_write_read_dt(&dev_i2c_pms, &pms_data_add, 1, &pms_data, 1); 
    tobj_L_data = pms_data;     // store received data in separate variables

    pms_data_add = 0x27;    
    ret = ret + i2c_write_read_dt(&dev_i2c_pms, &pms_data_add, 1, &pms_data, 1); 
    tobj_H_data = pms_data;     // store received data in separate variables

    pms_data_add = 0x28;    
    ret = ret + i2c_write_read_dt(&dev_i2c_pms, &pms_data_add, 1, &pms_data, 1); 
    tamb_L_data = pms_data;     // store received data in separate variables

    pms_data_add = 0x29;    
    ret = ret + i2c_write_read_dt(&dev_i2c_pms, &pms_data_add, 1, &pms_data, 1); 
    tamb_H_data = pms_data;     // store received data in separate variables


    if(ret != 0){
        printk("Failed to write to I2C device address %x. \n\r", dev_i2c_pms.addr);
        return 1;
    }

    // Store sensor data in sample buffer and increase data counter
    sample_data[sample_data_counter] = ((tobj_H_data << 8) | tobj_L_data);          // Object temp:  tobj/2000 °C
    sample_data[sample_data_counter +1] = ((tamb_H_data << 8) | tamb_L_data);       // Ambient temp: tamb/100 °C
    sample_data_counter = sample_data_counter + 2;

    return 0;

}

int tof_init(void){

    printk("TOF Init Start\n");

    int ret = 0;
    uint16_t dev = 0x0000;

    /* Wait for device booted */
    uint8_t state = 0;
    while(state == 0){
            ret = VL53L1X_BootState(dev, &state);
            k_msleep(2);
    }

    // Initialize sensor to default values
    ret = ret + VL53L1X_SensorInit(dev);

    if(ret != 0){
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
    if (SOC.processing==99){
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


int tof_sample(void){

    int ret = 0;
    uint16_t dev = 0x0000;

    if (SOC.processing != 99){

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
        while(tof_data_ready == 0){
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

        if (IOExp_Int_Handling() == 8){     // Check for interrupts. If TOF interrupt has triggered:
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

int bio_sample(void){

    int ret = 0;
    int read_status = 0;
    uint8_t config[4]; 
    uint8_t status;
    uint8_t read[181] = {0x00, 0x00}; // 1 status + 15 samples * 12 bytes per sample
    #define CMD_DELAY 3     // default command delay in ms +1 ms

    // Sensor hub interrupt threshold = 15 samples
    config[0] = 0x10;   // Family Byte
    config[1] = 0x01;   // Index Byte
    config[2] = 0x0F;    // data
    ret = ret + i2c_write_dt(&dev_i2c_bio, config, 3);
    k_msleep(CMD_DELAY);

    // Sensor data output (Raw)
    config[0] = 0x10;   // Family Byte
    config[1] = 0x00;   // Index Byte
    config[2] = 0x01;    // data
    ret = ret + i2c_write_dt(&dev_i2c_bio, config, 3);
    k_msleep(CMD_DELAY);

    // Enable BPT algorithm in estimation mode
    config[0] = 0x52;   // Family Byte
    config[1] = 0x04;   // Index Byte
    config[2] = 0x02;   // data
    ret = ret + i2c_write_dt(&dev_i2c_bio, config, 3);
    k_msleep(20);

    // Disable automatic gain control (AGC)
    config[0] = 0x52;   // Family Byte
    config[1] = 0x00;   // Index Byte
    config[2] = 0x00;   // data
    ret = ret + i2c_write_dt(&dev_i2c_bio, config, 3);
    k_msleep(20);

    // Set LED 1 (red) to 1/4 full scale
    config[0] = 0x40;   // Family Byte
    config[1] = 0x03;   // Index Byte
    config[2] = 0x0C;   // data
    config[3] = 0x40;   // data
    ret = ret + i2c_write_dt(&dev_i2c_bio, config, 4);
    k_msleep(CMD_DELAY);

    // Set LED 2 (IR) to 1/4 full scale
    config[0] = 0x40;   // Family Byte
    config[1] = 0x03;   // Index Byte
    config[2] = 0x0D;   // data
    config[3] = 0x40;   // data
    ret = ret + i2c_write_dt(&dev_i2c_bio, config, 4);
    k_msleep(CMD_DELAY);

    // Set LED 3 (green) to 1/4 full scale
    config[0] = 0x40;   // Family Byte
    config[1] = 0x03;   // Index Byte
    config[2] = 0x0D;   // data
    config[3] = 0x40;   // data
    ret = ret + i2c_write_dt(&dev_i2c_bio, config, 4);
    k_msleep(CMD_DELAY);

    // Enable analog front end (AFE)
    config[0] = 0x44;   // Family Byte
    config[1] = 0x03;   // Index Byte
    config[2] = 0x01;   // data
    ret = ret + i2c_write_dt(&dev_i2c_bio, config, 3);
    k_msleep(40);
    ret = ret + i2c_read_dt(&dev_i2c_bio, &status, 1);
    read_status = read_status + status;
    k_msleep(CMD_DELAY);

    // Check status and wait for DataRdyInt (FIFO reached threshold)
    config[0] = 0x00;   // Family Byte
    config[1] = 0x00;   // Index Byte
    while((read[1] & 0x08) == 0){
        ret = ret + i2c_write_dt(&dev_i2c_bio, config, 2);
        k_msleep(CMD_DELAY);
        ret = ret + i2c_read_dt(&dev_i2c_bio, read, 2);
        read_status = read_status + read[0];
        k_msleep(CMD_DELAY);
    }

    // Skipping step of reading number of samples in FIFO since it is >= threshold

    // Read samples from FIFO
    config[0] = 0x12;   // Family Byte
    config[1] = 0x01;   // Index Byte
    ret = ret + i2c_write_dt(&dev_i2c_bio, config, 2);
    k_msleep(CMD_DELAY);
    ret = ret + i2c_read_dt(&dev_i2c_bio, read, 181);
    read_status = read_status + read[0];
    k_msleep(CMD_DELAY);

    // Store sensor data in sample buffer and increase data counter
    sample_data[sample_data_counter] = read[0];
    for (int i=1; i<90; i++){
        sample_data[sample_data_counter+i] = (read[i] << 8) | (read[i+1]);
    }
    sample_data_counter = sample_data_counter + 91;
    
    return 0;
}

int gps_init(void){
    return 0;
}

int gps_sample(){
    return 0;
}

int mic_sample(){
    
    /*
int ret;

	ret = do_pdm_transfer(dmic_dev, &cfg, 1 * BLOCK_COUNT);
    printk("PDM transfer done\n");
	if (ret < 0) {
		return 0;
	}
    */
    return 0;
}

int col_init(void){
    opt4060_init(CONV_TIME_100MS, MODE_ONESHOT_AUTO);
    return 0;
}

int col_sample(void){
    opt4060_data_t data;
    opt4060_sample_oneshot(&data);
    //! sample_data is 16 bit integer, data is 32 bit floating point
    // Store sensor data in sample buffer and increase data counter
    for (int i=0; i<OPT4060_DATA_SIZE_BYTES/2; i++){
        sample_data[sample_data_counter+i] = (data.byte[i] << 8) | (data.byte[i+1]);
    }
    sample_data_counter = sample_data_counter + OPT4060_DATA_SIZE_BYTES/2;
    return 0;
}

int bps_init(void) {
    dps368_init(X_PRC_64X);
    return 0;
}

int bps_sample() {
    printk("Sampling BPS Sensor");
    dps368_data_t data;
    dps368_sample_single(&data);

    uint32_t tmp_bits;
    uint32_t prs_bits;
    memcpy(&tmp_bits, &data.tmp.comp, sizeof(data.tmp.comp));
    memcpy(&prs_bits, &data.prs.comp, sizeof(data.prs.comp));

    // Store sensor data in sample buffer and increase data counter
    sample_data[sample_data_counter+1] = (uint16_t)(tmp_bits >> 16);
    sample_data[sample_data_counter] = (uint16_t)(tmp_bits);
    sample_data[sample_data_counter+3] = (uint16_t)(prs_bits >> 16);
    sample_data[sample_data_counter+2] = (uint16_t)(prs_bits);

    sample_data_counter = sample_data_counter + 4;
    
    return 0;
}

struct bme68x_dev bme;
struct bme68x_heatr_conf heatr_conf;
struct bme68x_conf conf;

int air_init(void) {
    int8_t rslt;

    rslt = bme68x_interface_init(&bme, BME68X_I2C_INTF);
    bme68x_check_rslt("bme68x_interface_init", rslt);

    rslt = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    conf.filter = BME68X_FILTER_OFF;
    conf.odr = BME68X_ODR_NONE;
    conf.os_hum = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_16X;
    conf.os_temp = BME68X_OS_16X;
    rslt = bme68x_set_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    heatr_conf.enable = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur = 300;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

    printk("Sample, TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%%), Gas resistance(ohm), Status\n");
    return 0;
}

int air_sample() {
    int8_t rslt;
    struct bme68x_data data;
    uint32_t del_period;
    uint8_t n_fields;

    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);

    /* Calculate delay period in microseconds */
    del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme) + (heatr_conf.heatr_dur * 1000);
    bme.delay_us(del_period, bme.intf_ptr);

    /* Check if rslt == BME68X_OK, report or handle if otherwise */
    rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
    bme68x_check_rslt("bme68x_get_data", rslt);

    // Store sensor data in sample buffer and increase data counter
    sample_data[sample_data_counter] = (uint16_t)data.temperature;
    sample_data[sample_data_counter+1] = (uint16_t)data.temperature >> 16;
    sample_data[sample_data_counter+2] = (uint16_t)data.pressure;
    sample_data[sample_data_counter+3] = (uint16_t)data.pressure >> 16;
    sample_data[sample_data_counter+4] = (uint16_t)data.humidity;
    sample_data[sample_data_counter+5] = (uint16_t)data.humidity >> 16;
    sample_data[sample_data_counter+6] = (uint16_t)data.gas_resistance;
    sample_data[sample_data_counter+7] = (uint16_t)data.gas_resistance >> 16;

    sample_data_counter = sample_data_counter + 8;

    return 0;
}


int sensors_init(void){

    k_msleep(10);   // Wait for sensor board voltage to stabilize 

    IOExp_init();   // Initialize the IO Expander

    // Turn off all sensors
    power_sensors(SENS_TMP_MASK, 0);
    power_sensors(SENS_IMU_MASK, 0);
    power_sensors(SENS_PMS_MASK, 0);
    power_sensors(SENS_TOF_MASK, 0);
    power_sensors(SENS_BIO_MASK, 0);
    power_sensors(SENS_GPS_MASK, 0);
    power_sensors(SENS_COL_MASK, 0);
    power_sensors(SENS_BPS_MASK, 0);
    power_sensors(SENS_AIR_MASK, 0);
    // Turn off all sensor I2C buses
    Switch_SensI2C(SENS_TMP_MASK, 0);
    Switch_SensI2C(SENS_IMU_MASK, 0);
    Switch_SensI2C(SENS_PMS_MASK, 0);
    Switch_SensI2C(SENS_TOF_MASK, 0);
    Switch_SensI2C(SENS_BIO_MASK, 0);
    Switch_SensI2C(SENS_GPS_MASK, 0);
    Switch_SensI2C(SENS_COL_MASK, 0);
    Switch_SensI2C(SENS_BPS_MASK, 0);
    Switch_SensI2C(SENS_AIR_MASK, 0);

    // Temperature Sensor
    if (SOC.sensor_type & SENS_TMP_MASK){ 
        power_sensors(SENS_TMP_MASK, 1);
        Switch_SensI2C(SENS_TMP_MASK, 1);
        tmp_init();
        Switch_SensI2C(SENS_TMP_MASK, 0);
        power_sensors(SENS_TMP_MASK, 0);
    } 

    // Inertial Measurement Unit
    if (SOC.sensor_type & SENS_IMU_MASK){
        power_sensors(SENS_IMU_MASK, 1);
        Switch_SensI2C(SENS_IMU_MASK, 1);
        imu_init();
        if (SOC.num_samples > 0){
            //Switch_SensI2C(SENS_IMU_MASK, 0);
            //power_sensors(SENS_IMU_MASK, 0);
        }
    }

    // Presence / Motion Sensor
    if (SOC.sensor_type & SENS_PMS_MASK){
        // Nothing to initialize
    }

    // Time of Flight Sensor
    if (SOC.sensor_type & SENS_TOF_MASK){
        // Init always-on sensor, otherwise (one-shot) init is handled in sensors_sample()
        if (SOC.processing == 99){
            power_sensors(SENS_TOF_MASK, 1);
            Switch_SensI2C(SENS_TOF_MASK, 1);
            tof_init();
        }
    }

    // Bio Sensor
    if (SOC.sensor_type & SENS_BIO_MASK){
        // Nothing to initialize
    } 

    // GNSS
    if (SOC.sensor_type & SENS_GPS_MASK & !BOARD_V1_2){
        power_sensors(SENS_GPS_MASK, 1);
        gps_init();
        power_sensors(SENS_GPS_MASK, 0);
    }

    // MIC
    if (SOC.sensor_type & SENS_MIC_MASK){
        //power_sensors(SENS_MIC_MASK, 1);
        //mic_init(dmic_dev);
    }

    // Color Sensor
    if (SOC.sensor_type & SENS_COL_MASK){
        // Nothing to initialize
    }

    // Pressure Sensor
    if (SOC.sensor_type & SENS_BPS_MASK){
        // Nothing to initialize
    }

    // Air Quality Sensor
    if (SOC.sensor_type & SENS_AIR_MASK){
        // Nothing to initialize
    }

    // Check if I/O expander interrupt has triggered
    k_msleep(50);
    IOExp_Int_Handling();
    
    return 0;

}

int sensors_sample(uint16_t Sensor_Type){

    if (Sensor_Type == 0){
        for (sample_data_counter = 0; sample_data_counter < 100; sample_data_counter++){
            sample_data[sample_data_counter] = test_data;
            sample_data_counter = sample_data_counter + 1;
            test_data ++;
        }
    }

    // Temperature Sensor
    if (Sensor_Type & SENS_TMP_MASK){
        power_sensors(SENS_TMP_MASK, 1);
        Switch_SensI2C(SENS_TMP_MASK, 1);
        tmp_sample();
        Switch_SensI2C(SENS_TMP_MASK, 0);
        power_sensors(SENS_TMP_MASK, 0);
        k_msleep(1000);   // wait for possible interrupts to be cleared subsequently
    } 

    // Inertial Measurement Unit
    if (Sensor_Type & SENS_IMU_MASK){
        if (SOC.processing!=99){            // One-shot sampling
            power_sensors(SENS_IMU_MASK, 1);
            Switch_SensI2C(SENS_IMU_MASK, 1);
            imu_init();
            imu_sample();
            Switch_SensI2C(SENS_IMU_MASK, 0);
            power_sensors(SENS_IMU_MASK, 0);
        } else {
            imu_sample();   // Sensor stays on when in always-on mode (SOC.processing==99)
        }
        

    }

    // Presence / Motion Sensor
    if (Sensor_Type & SENS_PMS_MASK){
        power_sensors(SENS_PMS_MASK, 1);
        Switch_SensI2C(SENS_PMS_MASK, 1);
        pms_sample();
        Switch_SensI2C(SENS_PMS_MASK, 0);
        power_sensors(SENS_PMS_MASK, 0);
    }

    // Time of Flight Sensor
    if (Sensor_Type & SENS_TOF_MASK){
        if (SOC.processing!=99){        // One-shot sampling
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

    // Bio Sensor
    if (Sensor_Type & SENS_BIO_MASK){
        power_sensors(SENS_BIO_MASK, 1);
        k_msleep(750);   // Wait 1000 ms
        Switch_SensI2C(SENS_BIO_MASK, 1);
        bio_sample();
        Switch_SensI2C(SENS_BIO_MASK, 0);
        power_sensors(SENS_BIO_MASK, 0);
        k_msleep(500);   // Wait before checking I/O exp. interrupt
    } 

    // GNSS
    if (Sensor_Type & SENS_GPS_MASK & !BOARD_V1_2){
        power_sensors(SENS_GPS_MASK, 1);
        Switch_SensI2C(SENS_GPS_MASK, 1);
        gps_sample();
        Switch_SensI2C(SENS_GPS_MASK, 0);
        power_sensors(SENS_GPS_MASK, 0);
    }

    // MIC
    if (Sensor_Type & SENS_MIC_MASK){
        power_sensors(SENS_MIC_MASK, 1);
        mic_sample();
        printk("Mic sampling done\n");
        power_sensors(SENS_MIC_MASK, 0);
    }

    // Color sensor
    if (Sensor_Type & SENS_COL_MASK){
        power_sensors(SENS_COL_MASK, 1);
        Switch_SensI2C(SENS_COL_MASK, 1);
        col_init();
        col_sample();
        Switch_SensI2C(SENS_COL_MASK, 0);
        power_sensors(SENS_COL_MASK, 0);
    }

    // Pressure Sensor
    if (Sensor_Type & SENS_BPS_MASK){
        power_sensors(SENS_BPS_MASK, 1);
        Switch_SensI2C(SENS_BPS_MASK, 1);
        //printk("BPS Sampling\n");
        bps_init();
        bps_sample();
        Switch_SensI2C(SENS_BPS_MASK, 0);
        //TODO: Check if sensor can be turned off or add sleep mode to driver
        power_sensors(SENS_BPS_MASK, 0);
    }

    // Air Quality Sensor
    if (Sensor_Type & SENS_AIR_MASK){
        power_sensors(SENS_AIR_MASK, 1);
        Switch_SensI2C(SENS_AIR_MASK, 1);
        air_init();
        air_sample();
        Switch_SensI2C(SENS_AIR_MASK, 0);
        power_sensors(SENS_AIR_MASK, 0);
        k_msleep(300);   // Wait before checking I/O exp. interrupt
    }

    sample_packets_counter += 1;

    // Check if I/O expander interrupt has triggered
    k_msleep(200);
    IOExp_Int_Handling();

    return 0;

}

int process_samples(void) {

    switch (SOC.processing)
        {
        case 1:     // Averaging
        {
            float average_samples = 0;
            if (SOC.sensor_type & SENS_TMP_MASK){
                for(int i = 0; i<sample_data_counter; i++){
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

/*

extern const struct gpio_dt_spec led_1;
extern const struct gpio_dt_spec led_2;
extern const struct gpio_dt_spec led_3;
extern const struct gpio_dt_spec led_4;
extern const struct gpio_dt_spec led_5;
extern const struct gpio_dt_spec led_6;
extern const struct gpio_dt_spec led_7;
extern const struct gpio_dt_spec led_8;
extern const struct gpio_dt_spec led_9;


int sensor_I2C_scanner(void){

    k_msleep(1000);   // Wait 1000 ms

    printk("Testing I/O Expander\n");

    // I/O Expander
    int ret_iox = 0;
    uint8_t config1[3] = {0x02,0x00, 0x00}; // Set IO outputs to low
    ret_iox = i2c_write_dt(&dev_i2c_ioexp, &config1, sizeof(config1));

    if (ret_iox == 0){
        printk("IOX: OK\n");
        gpio_pin_configure_dt(&led_1, (1U << 16));
    } else {
        printk("IOX: Not OK\n");
    }

    printk("Testing I2C Switch\n");

    // I2C Switch
    int ret_i2c = 0;
    ret_i2c = i2c_write_dt(&dev_i2c_i2cswitch, &config1[2], 1); // Deactivate Switches

    if (ret_i2c == 0){
        printk("I2C Switch: OK\n");
        gpio_pin_configure_dt(&led_2, (1U << 16));
    } else {
        printk("I2C Switch: Not OK\n");
    }

    // Init IOExpander
    IOExp_init();

    printk("Testing Temperature Sensor\n");

    // Temperature Sensor
    power_sensors(SENS_TMP_MASK, 1);
    Switch_SensI2C(SENS_TMP_MASK, 1);
    k_msleep(1000);   // Wait 1000 ms

    int ret_tmp;
    // Unlock EEPROM
    config1[0] = 0x04;
    config1[1] = 0x80;   
    config1[2] = 0x00;    
    ret_tmp = i2c_write_dt(&dev_i2c_tmp, &config1, sizeof(config1));

    Switch_SensI2C(SENS_TMP_MASK, 0);
    power_sensors(SENS_TMP_MASK, 0);

    if (ret_tmp == 0){
        printk("TMP: OK\n");
        gpio_pin_configure_dt(&led_3, (1U << 16));
    } else {
        printk("TMP: Not OK\n");
    }


  
    
    printk("Testing Inertial Measurement Unit\n");

    // Inertial Measurement Unit
    power_sensors(SENS_IMU_MASK, 1);
    Switch_SensI2C(SENS_IMU_MASK, 1);
    k_msleep(1000);   // Wait 1000 ms

    int ret_imu = 0;
    uint8_t config[2] = {0x00, 0x00}; 
    // Gyro full scale: +/-250 dps, ODR: 25 Hz
    config[0] = 0x20; // Register
    config[1] = 0x6B; // Data
    ret_imu = i2c_write_dt(&dev_i2c_imu, &config, sizeof(config));

    k_msleep(1000);   // Wait 1000 ms

    Switch_SensI2C(SENS_IMU_MASK, 0);
    power_sensors(SENS_IMU_MASK, 0);
    
    if (ret_imu == 0){
        printk("IMU: OK\n");
        gpio_pin_configure_dt(&led_4, (1U << 16));
    } else {
        printk("IMU: Not OK\n");
    }



    printk("Testing Presence / Motion Sensor\n");

    // Presence / Motion Sensor
    power_sensors(SENS_PMS_MASK, 1);
    Switch_SensI2C(SENS_PMS_MASK, 1);
    k_msleep(1000);   // Wait 1000 ms

    int ret_pms = 0;

    // Trigger One-Shot Acquisition
    config[0] = 0x21; // Register
    config[1] = 0x01; // Data
    ret_pms = i2c_write_dt(&dev_i2c_pms, &config, sizeof(config));

    Switch_SensI2C(SENS_PMS_MASK, 0);
    power_sensors(SENS_PMS_MASK, 0);

    if (ret_pms == 0){
        printk("PMS: OK\n");
        gpio_pin_configure_dt(&led_5, (1U << 16));
    } else {
        printk("PMS: Not OK\n");
    }



    printk("Testing Time of Flight Sensor\n");

    // Time of Flight Sensor
    power_sensors(SENS_TOF_MASK, 1);
    Switch_SensI2C(SENS_TOF_MASK, 1);
    k_msleep(1000);   // Wait 1000 ms

    int ret_tof = 0;
    uint16_t dev = 0x0000;
    // Wait for device booted
    uint8_t state = 0;
    ret_tof = VL53L1X_BootState(dev, &state);

    Switch_SensI2C(SENS_TOF_MASK, 0);
    power_sensors(SENS_TOF_MASK, 0);
    
    if (ret_tof == 0){
        printk("TOF: OK\n");
        gpio_pin_configure_dt(&led_6, (1U << 16));
    } else {
        printk("TOF: Not OK\n");
    }
 




    printk("Testing Bio Sensor\n");

    // Bio Sensor
    power_sensors(SENS_BIO_MASK, 1);
    k_msleep(1000);   // Wait 1000 ms
    Switch_SensI2C(SENS_BIO_MASK, 1);

    int ret_bio = 0;
    config1[0] = 0x10;   // Family Byte
    config1[1] = 0x00;   // Index Byte
    config1[2] = 0x01;    // data
    ret_bio = i2c_write_dt(&dev_i2c_bio, &config1, 3);

    Switch_SensI2C(SENS_BIO_MASK, 0);
    power_sensors(SENS_BIO_MASK, 0);
    
    if (ret_bio == 0){
        printk("BIO: OK\n");
        gpio_pin_configure_dt(&led_7, (1U << 16));
    } else {
        printk("BIO: Not OK\n");
    }

    printk("Testing GPS\n");

       

    // GNSS
    power_sensors(SENS_GPS_MASK, 1);
    k_msleep(3000);   // Wait 1000 ms
    Switch_SensI2C(SENS_GPS_MASK, 1);
    
    int ret_gps = 0;
    config[0] = 0x10;   
    ret_gps = i2c_write_dt(&dev_i2c_gps, &config, 1);

    k_msleep(100);   // Wait 1000 ms
    Switch_SensI2C(SENS_GPS_MASK, 0);
    power_sensors(SENS_GPS_MASK, 0);

    if (ret_gps == 0){
        printk("GPS: OK\n");
        gpio_pin_configure_dt(&led_8, (1U << 16));
    } else {
        printk("GPS: Not OK\n");
    }

    

    gpio_pin_configure_dt(&led_9, (1U << 16) | (1 << 4));

    return 0;

}

*/