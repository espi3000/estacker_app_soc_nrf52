#ifndef _OPT4060_H_
#define _OPT4060_H_

#include <stdint.h>

//******************************************************************************
//* I2C
//******************************************************************************
/**
 * ADDR pin configuration
 * ----------------------------
 * Pin connection | I2C address
 * ----------------------------
 * GND            | 0b1000100
 * VDD            | 0b1000101
 * SDA            | 0b1000110
 * SCL            | 0b1000111
 * ----------------------------
 */
#define OPT4060_I2C_ADDR_VDD 0b1000100
#define OPT4060_I2C_ADDR_GND 0b1000101
#define OPT4060_I2C_ADDR_SDA 0b1000110
#define OPT4060_I2C_ADDR_SCL 0b1000111

#ifndef OPT4060_DEVICE_ID
#define OPT4060_DEVICE_ID OPT4060_I2C_ADDR_GND
#endif

//******************************************************************************
//* Register addresses
//******************************************************************************
// Channel addresses
#define OPT4060_CH0_0       0x00
#define OPT4060_CH0_1       0x01
#define OPT4060_CH1_0       0x02
#define OPT4060_CH1_1       0x03
#define OPT4060_CH2_0       0x04
#define OPT4060_CH2_1       0x05
#define OPT4060_CH3_0       0x06
#define OPT4060_CH3_1       0x07

// Threshold addresses
#define OPT4060_THRESH_LOW  0x08
#define OPT4060_THRESH_HIGH 0x09

// Control and status addresses
#define OPT4060_CFG_0       0x0A
#define OPT4060_CFG_1       0x0B  
#define OPT4060_STATUS      0x0C
#define OPT4060_ID          0x11

//******************************************************************************
//* Default register values
//******************************************************************************
#define OPT4060_CHX_X_DEFAULT       {.raw = 0x0000}
#define OPT4060_THRESH_LOW_DEFAULT  {.raw = 0x0000}
#define OPT4060_THRESH_HIGH_DEFAULT {.raw = 0xBFFF}
#define OPT4060_CFG_0_DEFAULT       {.raw = 0x3208}
#define OPT4060_CFG_1_DEFAULT       {.raw = 0x8011}
#define OPT4060_STATUS_DEFAULT      {.raw = 0x0000}
#define OPT4060_ID_DEFAULT          {.raw = 0x0820}

//******************************************************************************
//* Control register 0 (0x0A) enums
//******************************************************************************
typedef enum {
    RANGE_2K2LUX = 0x00,
    RANGE_4K5LUX,
    RANGE_9KLUX,
    RANGE_18KLUX,
    RANGE_36KLUX,
    RANGE_72KLUX,
    RANGE_144KLUX,
    RANGE_AUTO = 0x0C
} opt4060_cfg_0_range_t;

typedef enum {
    CONV_TIME_600US = 0x00,
    CONV_TIME_1MS,
    CONV_TIME_1M8S,
    CONV_TIME_3M4S,
    CONV_TIME_6M5S,
    CONV_TIME_12M7S,
    CONV_TIME_25MS,
    CONV_TIME_50MS,
    CONV_TIME_100MS,
    CONV_TIME_200MS,
    CONV_TIME_400MS,
    CONV_TIME_800MS
} opt4060_cfg_0_conv_time_t;

typedef enum {
    MODE_POWER_DOWN = 0x00,
    MODE_ONESHOT_AUTO,
    MODE_ONESHOT,
    MODE_CONTINUOUS
} opt4060_cfg_0_mode_t;

typedef enum {
    INT_POL_ACTIVE_LOW = 0x00,
    INT_POL_ACTIVE_HIGH
} opt4060_cfg_0_int_pol_t;

//******************************************************************************
//* Control register 1 (0x0B) enums
//******************************************************************************
typedef enum {
    THRESH_CH_SEL_CH0 = 0x00,
    THRESH_CH_SEL_CH1,
    THRESH_CH_SEL_CH2,
    THRESH_CH_SEL_CH3
} opt4060_cfg_1_thresh_ch_sel_t;

typedef enum {
    INT_DIR_INPUT = 0x00,
    INT_DIR_OUTPUT
} opt4060_cfg_1_int_dir_t;

typedef enum {
    INT_CFG_SMBUS_ALERT = 0x00,
    INT_CFG_DATA_READY_NEXT_CH,
    INT_CFG_DATA_READY_ALL_CH
} opt4060_cfg_1_int_cfg_t;

//******************************************************************************
//* Data containers
//******************************************************************************
/**
 * @brief Combined container for chx_0 and chx_1 registers
 */
typedef union { 
    struct {
        uint32_t    crc         : 4;
        uint32_t    counter     : 4;
        uint32_t    mantissa    : 20;
        uint32_t    exponent    : 4;
    } __attribute__((packed));
    uint32_t raw;
} opt4060_chx_t;

#define OPT4060_DATA_SIZE_BYTES 20

/**
 * @brief Combined container for all channel data and lux
 */
typedef union {
    struct {
        union {
            opt4060_chx_t ch0;
            float r;
        };
        union {
            opt4060_chx_t ch1;
            float g;
        };
        union {
            opt4060_chx_t ch2;
            float b;
        };
        union {
            opt4060_chx_t ch3;
            float w;
        };
        float lux;
    };
    uint8_t byte[OPT4060_DATA_SIZE_BYTES];
} opt4060_data_t;



//******************************************************************************
//* Register bitfields
//******************************************************************************
/**
 * Channel data registers (even from 0x00 to 0x06)
 * Bit 0-11: Result MSB
 * Bit 12-15: Exponent
 */
typedef union {
    struct {
        uint16_t    result_msb      : 12;
        uint16_t    exponent        : 4;
    } __attribute__((packed));
    struct {
        uint8_t low_byte;
        uint8_t high_byte;
    };
    uint16_t raw;
} opt4060_chx_0_t;

/**
 * Channel data registers (odd from 0x01 to 0x07)
 * Bit 0-7: Result LSB
 * Bit 8-11: Counter
 * Bit 12-15: CRC
 */
typedef union {
    struct {
        uint16_t    crc             : 4;
        uint16_t    counter         : 4;
        uint16_t    result_lsb      : 8;
    } __attribute__((packed));
    struct {
        uint8_t low_byte;
        uint8_t high_byte;
    };
    uint16_t raw;
} opt4060_chx_1_t;

/**
 * Threshold register (0x08 to 0x09)
 * Bit 0-11: Result
 * Bit 12-15: Exponent
 */
typedef union {
    struct {
        uint16_t    result          : 12;
        uint16_t    exponent        : 4;
    } __attribute__((packed));
    struct {
        uint8_t low_byte;
        uint8_t high_byte;
    };
    uint16_t raw;
} opt4060_thresh_x_t;

/** 
 * Control register 0 (0x0A)
 * Bit 14-15: Fault Count
 * Bit 13: Interrupt Polarity
 * Bit 12: Latch
 * Bit 10-11: Mode
 * Bit 6-9: Conversion Time
 * Bit 2-5: Range
 * Bit 1: Reserved
 * Bit 0: Qwake
 */
typedef union {
    struct {
        uint16_t    fault_count     : 2;
        uint16_t    int_pol         : 1; // opt4060_cfg_0_int_pol_t
        uint16_t    latch           : 1;
        uint16_t    mode            : 2; // opt4060_cfg_0_mode_t
        uint16_t    conv_time       : 4; // opt4060_cfg_0_conv_time_t
        uint16_t    range           : 4; // opt4060_cfg_0_range_t
        uint16_t    reserved        : 1; // Always 0
        uint16_t    qwake           : 1;
    } __attribute__((packed));
    struct {
        uint8_t low_byte;
        uint8_t high_byte;
    };
    uint16_t raw;
} opt4060_cfg_0_t; 

/** 
 * Control register 1 (0x0B)
 * Bit 15: I2C Burst
 * Bit 14: Reserved
 * Bit 12-13: Interrupt Configuration
 * Bit 11: Interrupt Direction
 * Bit 9-10: Threshold Channel Select
 * Bit 0-8: Reserved
 */
typedef union {
    struct {
        uint16_t    i2c_burst       : 1;
        uint16_t    reserved_0      : 1; // Always 0
        uint16_t    int_cfg         : 2; // opt4060_cfg_1_int_cfg_t
        uint16_t    int_dir         : 1; // opt4060_cfg_1_int_dir_t
        uint16_t    thresh_ch_sel   : 2; // opt4060_cfg_1_thresh_ch_sel_t
        uint16_t    reserved_1      : 9; // Always 128
    } __attribute__((packed));
    struct {
        uint8_t low_byte;
        uint8_t high_byte;
    };
    uint16_t raw;
} opt4060_cfg_1_t;

/**
 * Status register (0x0C)
 * Bit 0: L
 * Bit 1: H
 * Bit 2: Ready
 * Bit 3: Overload
 * Bit 4-15: Reserved
 */
typedef union {
    struct {
        uint16_t    l               : 1;
        uint16_t    h               : 1;
        uint16_t    ready           : 1;
        uint16_t    overload        : 1;
        uint16_t    reserved        : 12; // Always 0
    } __attribute__((packed));
    struct {
        uint8_t low_byte;
        uint8_t high_byte;
    };
    uint16_t raw;
} opt4060_status_t;

//TODO: Combine DIDH and DIDL into one access
/**
 * ID register (0x11)
 * Bit 0-1: DIDL
 * Bit 2-13: DIDH
 * Bit 14-15: Reserved
 */
typedef union {
    struct {
        uint16_t    didh            : 12;
        uint16_t    didl            : 2;
        uint16_t    reserved        : 2; // Always 0
    } __attribute__((packed));
    struct {
        uint8_t low_byte;
        uint8_t high_byte;
    };
    uint16_t raw;
} opt4060_id_t;

//******************************************************************************
//* Function prototypes
//******************************************************************************
/**
 * @brief Initializes the OPT4060 sensor with I2C burst mode and data ready
 *        interrupt output.
 * 
 * @param conv_time Conversion time for the sensor.
 * @param mode Mode of operation for the sensor.
 * 
 * @retval 0 Success
 * @retval 1 Failure
 */
int opt4060_init(opt4060_cfg_0_conv_time_t conv_time, opt4060_cfg_0_mode_t mode);

/**
 * @brief Reads the status register of the OPT4060 sensor.
 * 
 * @param status_ptr Pointer to the variable where the status will be stored.
 * 
 * @retval 0 Success
 * @retval 1 Failure
 */
int opt4060_check_status(opt4060_status_t* status_ptr);

/**
 * @brief Starts all channel conversions on the OPT4060 sensor.
 * 
 * @retval 0 Success
 * @retval 1 Failure
 */
int opt4060_start_conversion(void);

/**
 * @brief Reads all channel data registers of the OPT4060 sensor.
 * 
 * @param data_ptr Pointer to the variable where the data will be stored.
 * 
 * @retval 0 Success
 * @retval 1 Failure
 */
int opt4060_read_channels(opt4060_data_t* data_ptr);

/**
 * @brief Transforms raw integer data into floating point lux and RGBW values.
 * 
 * @param data_ptr Pointer to the data to be post-processed.
 */
void opt4060_post_process_data(opt4060_data_t* data_ptr);

/**
 * @brief Samples all channels on the OPT4060 sensor a single time.
 * 
 * @param data_ptr Pointer to the variable where the data will be stored.
 * 
 * @retval 0 Success
 * @retval 1 Failure
 */
int opt4060_sample_oneshot(opt4060_data_t* data_ptr);

#endif // _OPT4060_H_