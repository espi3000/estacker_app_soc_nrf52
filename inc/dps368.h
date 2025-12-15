#ifndef _DPS368_H_
#define _DPS368_H_

#include <stdint.h>

//******************************************************************************
//* Constants
//******************************************************************************
extern const int32_t DPS368_SCALE_FACTOR[];

//******************************************************************************
//* I2C addresses
//******************************************************************************
#define DPS368_I2C_ADDR_DEFAULT 0x77
#define DPS368_I2C_ADDR_SDO     0x76

//******************************************************************************
//* Register addresses
//******************************************************************************
#define DPS368_PRS_B2           0x00
#define DPS368_PRS_B1           0x01
#define DPS368_PRS_B0           0x02
#define DPS368_TMP_B2           0x03
#define DPS368_TMP_B1           0x04
#define DPS368_TMP_B0           0x05
#define DPS368_PRS_CFG          0x06
#define DPS368_TMP_CFG          0x07
#define DPS368_MEAS_CFG         0x08
#define DPS368_CFG_REG          0x09
#define DPS368_INT_STS          0x0A
#define DPS368_FIFO_STS         0x0B
#define DPS368_RESET            0x0C
#define DPS368_ID               0x0D
#define DPS368_COEF_START       0x10
#define DPS368_COEF_END         0x21
#define DPS368_COEF_SRCE        0x28

//******************************************************************************
//* Default register values
//******************************************************************************
#define DPS368_PRS_BX_DEFAULT   {.raw = 0x00}
#define DPS368_TMP_BX_DEFAULT   {.raw = 0x00}
#define DPS368_PRS_CFG_DEFAULT  {.raw = 0x00}
#define DPS368_TMP_CFG_DEFAULT  {.raw = 0x00}
#define DPS368_MEAS_CFG_DEFAULT {.raw = 0xC0}
#define DPS368_CFG_REG_DEFAULT  {.raw = 0x00}
#define DPS368_INT_STS_DEFAULT  {.raw = 0x00}
#define DPS368_FIFO_STS_DEFAULT {.raw = 0x00}
#define DPS368_RESET_DEFAULT    {.raw = 0x00}
#define DPS368_ID_DEFAULT       {.raw = 0x10}
//#define DPS368_COEF_DEFAULT       //Undefined
//#define DPS368_COEF_SRCE_DEFAULT  //Undefined

//******************************************************************************
//* Macros
//******************************************************************************
#define DPS368_COEF_SIZE 18
#define DPS368_DATA_SIZE 3

//******************************************************************************
//* Enums
//******************************************************************************
/**
 * @brief DPS368 sampling rate configuration
 */
typedef enum {
    X_RATE_1HZ = 0,
    X_RATE_2HZ,
    X_RATE_4HZ,
    X_RATE_8HZ,
    X_RATE_16HZ,
    X_RATE_32HZ,
    X_RATE_64HZ,
    X_RATE_128HZ
} dps368_x_cfg_x_rate_t;

/**
 * @brief DPS368 oversampling ratio configuration
 */
typedef enum {
    X_PRC_1X = 0,   // Low precision
    X_PRC_2X,       // Low power
    X_PRC_4X,
    X_PRC_8X,
    X_PRC_16X,      // Standard
    X_PRC_32X,
    X_PRC_64X,      // High precision
    X_PRC_128X 
} dps368_x_cfg_x_prc_t;

/**
 * @brief DPS368 state configuration
 */
typedef enum {
    MEAS_CTRL_IDLE = 0,
    MEAS_CTRL_PRS = 1,
    MEAS_CTRL_TMP = 2,
    MEAS_CTRL_CONTINOUS_PRS = 5,
    MEAS_CTRL_CONTINOUS_TMP = 6,
    MEAS_CTRL_CONTINOUS_PRS_TMP = 7
} dps368_meas_cfg_meas_ctrl_t;

/**
 * @brief DPS368 interrupt polarity selection configuration
 */
typedef enum {
    INT_HL_ACTIVE_LOW = 0,
    INT_HL_ACTIVE_HIGH
} dps368_cfg_reg_int_hl_t;

/**
 * @brief DPS368 SPI configuration
 * 
 * 3 wire mode uses the same pin for MOSI and MISO
 * 4 wire mode uses separate pins for MOSI and MISO
 */
typedef enum {
    SPI_MODE_4WIRE = 0,
    SPI_MODE_3WIRE
} dps368_cfg_reg_spi_mode_t;

/**
 * @brief DPS368 soft reset configuration
 * 
 * The device will generate a soft reset when enabled
 */
typedef enum {
    SOFT_RST_DISABLE = 0b0000,
    SOFT_RST_ENABLE = 0b1001
} dps368_reset_soft_rst_t;

/**
 * @brief DPS368 temperature source configuration
 * 
 * The temperature can be sourced from either the internal temperature sensor on 
 * the ASIC or an external temperature sensor on the pressure sensor.
 */
typedef enum {
    TMP_EXT_INTERNAL = 0,
    TMP_EXT_EXTERNAL
} dps368_tmp_cfg_tmp_ext_t;

/**
 * @brief DPS368 temperature coefficient source configuration
 * 
 * The temperature coefficient can be sourced from either the internal
 * temperature sensor on the ASIC or an external temperature sensor on the
 * pressure sensor.
 */
typedef enum {
    TMP_COEF_SRCE_INTERNAL = 0,
    TMP_COEF_SRCE_EXTERNAL
} dps368_coef_srce_tmp_coef_srce_t;

//******************************************************************************
//* Data containers
//******************************************************************************
/**
 * @typedef dps368_data_t
 * @brief Data container for DPS368 sensor
 * 
 * This union is used to store the raw data from the sensor. The data is stored
 * in a 32-bit unsigned integer, but the data is represented as a 24-bit signed
 * integer. Thus the 24th bit is the sign bit.
 * 
 * @var dps368_data_t::data[]
 * Member 'data[]' is the 24-bit data.
 */
typedef struct {
    union {
        struct {
            int32_t raw : 24; // Ensures correct 2's complement representation
        };
        float comp;
        uint8_t byte[DPS368_DATA_SIZE];
    } tmp;
    union {
        struct {
            int32_t raw : 24; // Ensures correct 2's complement representation
        };
        float comp;
        uint8_t byte[DPS368_DATA_SIZE];
    } prs;
} dps368_data_t;

/**
 * @typedef dps368_coefficients_t
 * @brief Coefficients container for DPS368 sensor
 * 
 * This union is used to store the raw coefficients from the sensor. The registers
 * can be placed in the raw unsigned array and be accessed as signed coefficients.
 * For example: reg 0x10 should be placed in raw[0] and 0x30 be placed in raw[17].
 * 
 * @var dps368_coefficients_t::c0
 * Member 'c0' is the first signed coefficient.
 * @var dps368_coefficients_t::c1
 * Member 'c1' is the second signed coefficient.
 * @var dps368_coefficients_t::c00
 * Member 'c00' is the third signed coefficient.
 * @var dps368_coefficients_t::c10
 * Member 'c10' is the fourth signed coefficient.
 * @var dps368_coefficients_t::c01
 * Member 'c01' is the fifth signed coefficient.
 * @var dps368_coefficients_t::c11
 * Member 'c11' is the sixth signed coefficient.
 * @var dps368_coefficients_t::c20
 * Member 'c20' is the seventh signed coefficient.
 * @var dps368_coefficients_t::c21
 * Member 'c21' is the eighth signed coefficient.
 * @var dps368_coefficients_t::c30
 * Member 'c30' is the ninth signed coefficient.
 * @var dps368_coefficients_t::raw[]
 * Member 'raw[]' should be used for the COEF registers (0x10 to 0x21).
 */
typedef union {
    struct {
        int32_t c0  : 12; // 0x10[11:4] -0x11[3:0]
        int32_t c1  : 12; // 0x11[11:8] -0x12[7:0]
        int32_t c00 : 20; // 0x13[19:12]-0x15[11:4]-0x16[3:0]
        int32_t c10 : 20; // 0x16[19:16]-0x17[15:8]-0x18[7:0]
        int32_t c01 : 16; // 0x18[15:8] -0x19[7:0]
        int32_t c11 : 16; // 0x1A[15:8] -0x1B[7:0]
        int32_t c20 : 16; // 0x1C[15:8] -0x1D[7:0]
        int32_t c21 : 16; // 0x1E[15:8] -0x1F[7:0]
        int32_t c30 : 16; // 0x20[15:8] -0x21[7:0]
    } __attribute__((packed));
    uint8_t raw[DPS368_COEF_SIZE];
} dps368_coefficients_t;

//******************************************************************************
//* Register bitfields
//******************************************************************************
/**
 * @typedef dps368_prs_cfg_t
 * @brief DPS368 pressure configuration register (PRS_CFG/0x06)
 * 
 * @var dps368_prs_cfg_t::pm_prc
 * Member 'pm_prc' is the pressure oversampling ratio.
 * @var dps368_prs_cfg_t::pm_rate
 * Member 'pm_rate' is the pressure sampling rate.
 * @var dps368_prs_cfg_t::raw
 * Member 'raw' is the raw register value.
 */
typedef union {
    struct {
        uint8_t pm_prc          : 4; // [3:0] dps368_x_cfg_x_prc_t
        uint8_t pm_rate         : 3; // [6:4] dps368_x_cfg_x_rate_t 
        uint8_t reserved        : 1; // [7:7]
    } __attribute__((packed));
    uint8_t raw;
} dps368_prs_cfg_t;

/**
 * @typedef dps368_tmp_cfg_t
 * @brief DPS368 temperature configuration register (TMP_CFG/0x07)
 * 
 * @var dps368_tmp_cfg_t::tmp_prc
 * Member 'tmp_prc' is the temperature oversampling ratio.
 * @var dps368_tmp_cfg_t::tmp_rate
 * Member 'tmp_rate' is the temperature sampling rate.
 * @var dps368_tmp_cfg_t::tmp_ext
 * Member 'tmp_ext' is the temperature source selection.
 * @var dps368_tmp_cfg_t::raw
 * Member 'raw' is the raw register value.
 */
typedef union {
    struct {
        uint8_t tmp_prc         : 4; // [3:0] dps368_x_cfg_x_prc_t
        uint8_t tmp_rate        : 3; // [6:4] dps368_x_cfg_x_rate_t
        uint8_t tmp_ext         : 1; // [7:7] dps368_tmp_cfg_tmp_ext_t
    } __attribute__((packed));
    uint8_t raw;
} dps368_tmp_cfg_t;

/**
 * @typedef dps368_meas_cfg_t
 * @brief DPS368 measurement configuration register (MEAS_CFG/0x08)
 * 
 * @var dps368_meas_cfg_t::meas_ctrl
 * Member 'meas_ctrl' is the measurement control state.
 * @var dps368_meas_cfg_t::prs_ready
 * Member 'prs_ready' is the pressure data ready flag.
 * @var dps368_meas_cfg_t::tmp_ready
 * Member 'tmp_ready' is the temperature data ready flag.
 * @var dps368_meas_cfg_t::sensor_ready
 * Member 'sensor_ready' is the sensor initialization complete flag.
 * @var dps368_meas_cfg_t::coef_ready
 * Member 'coef_ready' is the coefficient ready flag.
 * @var dps368_meas_cfg_t::raw
 * Member 'raw' is the raw register value.
 */
typedef union {
    struct {
        uint8_t meas_ctrl       : 3; // [2:0] dps368_meas_cfg_meas_ctrl_t
        uint8_t reserved        : 1; // [3:3]
        uint8_t prs_ready       : 1; // [4:4]
        uint8_t tmp_ready       : 1; // [5:5]
        uint8_t sensor_ready    : 1; // [6:6]
        uint8_t coef_ready      : 1; // [7:7]
    } __attribute__((packed));
    uint8_t raw;
} dps368_meas_cfg_t;

/**
 * @typedef dps368_cfg_reg_t
 * @brief DPS368 interrupt and FIFO configuration register (CFG_REG/0x09)
 * 
 * @var dps368_cfg_reg_t::spi_mode
 * Member 'spi_mode' is the SPI mode selection.
 * @var dps368_cfg_reg_t::fifo_en
 * Member 'fifo_en' is the FIFO enable flag.
 * @var dps368_cfg_reg_t::prs_shift_en
 * Member 'prs_shift_en' enables the right shift of pressure measurement. Must 
 * be 1 when oversampling ratio is above 8.
 * @var dps368_cfg_reg_t::tmp_shift_en
 * Member 'tmp_shift_en' enables the right shift of temperature measurement. Must 
 * be 1 when oversampling ratio is above 8.
 * @var dps368_cfg_reg_t::prs_int_en
 * Member 'prs_int_en' enables the pressure measurement ready interrupt.
 * @var dps368_cfg_reg_t::tmp_int_en
 * Member 'tmp_int_en' enables the temperature measurement ready interrupt.
 * @var dps368_cfg_reg_t::fifo_int_en
 * Member 'fifo_int_en' enables the FIFO full interrupt.
 * @var dps368_cfg_reg_t::int_hl
 * Member 'int_hl' is the interrupt polarity selection.
 * @var dps368_cfg_reg_t::raw
 * Member 'raw' is the raw register value.
 */
typedef union {
    struct {
        uint8_t spi_mode        : 1; // [0:0] dps368_cfg_reg_spi_mode_t
        uint8_t fifo_en         : 1; // [1:1]
        uint8_t prs_shift_en    : 1; // [2:2]
        uint8_t tmp_shift_en    : 1; // [3:3]
        uint8_t prs_int_en      : 1; // [4:4]
        uint8_t tmp_int_en      : 1; // [5:5]
        uint8_t fifo_int_en     : 1; // [6:6]
        uint8_t int_hl          : 1; // [7:7] dps368_cfg_reg_int_hl_t
    } __attribute__((packed));
    uint8_t raw;
} dps368_cfg_reg_t;

/**
 * @typedef dps368_int_sts_t
 * @brief DPS368 interrupt status register (INT_STS/0x0A)
 * 
 * @var dps368_int_sts_t::int_prs
 * Member 'int_prs' is the pressure interrupt flag.
 * @var dps368_int_sts_t::int_tmp
 * Member 'int_tmp' is the temperature interrupt flag.
 * @var dps368_int_sts_t::int_fifo_full
 * Member 'int_fifo_full' is the FIFO full interrupt flag.
 * @var dps368_int_sts_t::raw
 * Member 'raw' is the raw register value.
 */
typedef union {
    struct {
        uint8_t  int_prs         : 1; // [0:0]
        uint8_t  int_tmp         : 1; // [1:1]
        uint8_t  int_fifo_full   : 1; // [2:2]
        uint8_t  reserved        : 5; // [7:3]
    } __attribute__((packed));
    uint8_t raw;
} dps368_int_sts_t;

/**
 * @typedef dps368_fifo_sts_t
 * @brief DPS368 FIFO status register (FIFO_STS/0x0B)
 * 
 * @var dps368_fifo_sts_t::fifo_empty
 * Member 'fifo_empty' is the FIFO empty flag.
 * @var dps368_fifo_sts_t::fifo_full
 * Member 'fifo_full' is the FIFO full flag.
 * @var dps368_fifo_sts_t::raw 
 * Member 'raw' is the raw register value.
 */
typedef union {
    struct {
        uint8_t fifo_empty      : 1; // [0:0]
        uint8_t fifo_full       : 1; // [1:1]
        uint8_t reserved        : 6; // [7:2]
    } __attribute__((packed));
    uint8_t raw;
} dps368_fifo_sts_t;

/**
 * @typedef dps368_reset_t
 * @brief DPS368 reset register (RESET/0x0C)
 * 
 * @var dps368_reset_t::soft_rst
 * Member 'soft_rst' is the soft reset configuration.
 * @var dps368_reset_t::fifo_flush
 * Member 'fifo_flush' clears the FIFO
 * @var dps368_reset_t::raw
 * Member 'raw' is the raw register value.
 */
typedef union {
    struct {
        uint8_t soft_rst        : 4; // [3:0] dps368_reset_soft_rst_t
        uint8_t reserved        : 3; // [6:4]
        uint8_t fifo_flush      : 1; // [7:7]
    } __attribute__((packed));
    uint8_t raw;
} dps368_reset_t;

/**
 * @typedef dps368_id_t
 * @brief DPS368 product and revision ID register (ID/0x0D)
 * 
 * @var dps368_id_t::prod_id
 * Member 'prod_id' is the product ID.
 * @var dps368_id_t::rev_id
 * Member 'rev_id' is the revision ID.
 * @var dps368_id_t::raw
 * Member 'raw' is the raw register value.
 */
typedef union {
    struct {
        uint8_t prod_id         : 4; // [3:0]
        uint8_t rev_id          : 4; // [7:4]
    } __attribute__((packed));
    uint8_t raw;
} dps368_id_t;

/**
 * @typedef dps368_coef_srce_t
 * @brief DPS368 coefficient source register (COEF_SRCE/0x28)
 * 
 * @var dps368_coef_srce_t::tmp_coef_srce
 * Member 'tmp_coef_srce' is the temperature coefficient source selection.
 * @var dps368_coef_srce_t::raw
 * Member 'raw' is the raw register value.
 */
typedef union {
    struct {
        uint8_t reserved        : 7; // [0:6]
        uint8_t tmp_coef_srce   : 1; // [7:7] dps368_coef_srce_tmp_coef_srce_t
    } __attribute__((packed));
    uint8_t raw;
} dps368_coef_srce_t;

//******************************************************************************
//* Function prototypes
//******************************************************************************
/**
 * @brief Initializes the DPS368 sensor
 * 
 * @param oversampling_ratio of pressure measurement
 * @retval 0 Success
 * @retval 1 Failure
 */
int dps368_init(dps368_x_cfg_x_prc_t oversampling_ratio);

/**
 * @brief Reads the status of the DPS368 sensor
 * 
 * @param status_ptr pointer to the variable where the status will be stored
 * @retval 0 Success
 * @retval 1 Failure
 */
int dps368_check_status(dps368_meas_cfg_t *status_ptr);

/**
 * @brief Starts a single temperature conversion
 * 
 * @retval 0 Success
 * @retval 1 Failure
 */
int dps368_start_tmp_conversion(void);

/**
 * @brief Starts a single pressure conversion
 * 
 * @retval 0 Success
 * @retval 1 Failure
 */
int dps368_start_prs_conversion(void);

/**
 * @brief Reads the temperature data from the DPS368 sensor
 * 
 * @param data_ptr pointer to the variable where the data will be stored
 * @retval 0 Success
 * @retval 1 Failure
 */
int dps368_read_tmp(dps368_data_t *data);

/**
 * @brief Reads the pressure data from the DPS368 sensor
 * 
 * @param data_ptr pointer to the variable where the data will be stored
 * @retval 0 Success
 * @retval 1 Failure
 */
int dps368_read_prs(dps368_data_t *data);

/**
 * @brief Post processes the raw data from the DPS368 sensor into compensated data
 * 
 * @param data_ptr pointer to the data to be post processed
 */
void dps368_post_process_data(dps368_data_t *data);

/**
 * @brief Samples pressure and temperature data from the DPS368 sensor a single time
 * 
 * @param coef_ptr pointer to the variable where the data will be stored
 * @retval 0 Success
 * @retval 1 Failure
 */
int dps368_sample_single(dps368_data_t *data);

#endif // _DPS368_H_