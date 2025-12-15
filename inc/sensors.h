
#ifndef _SENSORS_H_
#define _SENSORS_H_

#include <zephyr/drivers/i2c.h> // I2C
#include <zephyr/drivers/spi.h> // SPI


#define I2C0_IOExp DT_NODELABEL(ioexpander)
#define I2C0_I2CSwitch DT_NODELABEL(i2cswitch)
#define I2C0_TMP DT_NODELABEL(tmp)
#define I2C0_IMU DT_NODELABEL(imu)
#define I2C0_PMS DT_NODELABEL(pms)
#define I2C0_TOF DT_NODELABEL(tof)
#define I2C0_BIO DT_NODELABEL(bio)
#define I2C0_GPS DT_NODELABEL(gps)
#define I2C0_COL DT_NODELABEL(col)
#define I2C0_BPS DT_NODELABEL(bps)
#define I2C0_AIR DT_NODELABEL(air)
#define I2C0_GeneralCall DT_NODELABEL(generalcall)

#define SPI_ADC3 DT_NODELABEL(adc3)

void AD7091_write_reg(uint8_t MSB, uint8_t LSB);
void SPI_Init(void);

int sensors_init(void);
int IOExp_init(void);
int tmp_init(void);
int col_init(void);
int col_sample(void);
int bps_init(void);
int bps_sample(void);
int air_init(void);
int air_sample(void);
int sensors_sample(uint16_t Sensor_Type);
int process_samples(void);

int power_sensors(uint16_t Sensor_Type, uint8_t Power_On);

int IOExp_Int_Handling(void);

// Testing
int sensor_I2C_scanner(void);


extern uint16_t sample_data[512];           // Sample data buffer
extern uint16_t sample_data_counter;        // Sample data counter
extern uint16_t sample_packets_counter;     // Number of sample packets. A sample packet can consist of the measurements of several sensors




#endif // _SENSORS_H_