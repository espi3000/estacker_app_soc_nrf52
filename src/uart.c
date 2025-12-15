#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <cJSON.h>

#include "uart.h"
#include "main.h"

//******************************************************************************
//* Macros
//******************************************************************************

#define TX_BUF_SIZE 4 // Define the size of the transmit buffer
#define RX_BUF_SIZE 255 // Define the size of the receive buffer
#define RECEIVE_TIMEOUT 1000 // Define the receiving timeout period
#define SENSOR_TOF 8

//******************************************************************************
//* Private variables
//******************************************************************************

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static uint8_t uart_tx_buf[TX_BUF_SIZE];
static uint8_t uart_rx_buf[RX_BUF_SIZE*2] = {0}; // Max UART buffer size is 255 bytes,so we allocate twice that to handle UART_RX_BUF_REQUEST
static bool params_received = false; // flag if parameters were received via UART

//******************************************************************************
//* Private function prototypes
//******************************************************************************

/**
 * @brief UART RX callback function
 * 
 * @param dev 
 * @param evt 
 * @param user_data 
 */
static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data);

/**
 * @brief Parse system parameters from JSON
 * 
 * @param json 
 * @return int 
 */
static int system_params_json_parse(cJSON* json);

/**
 * @brief Parse Time-of-Flight (ToF) sensor parameters from JSON
 * 
 * @param json 
 * @return int 
 */
static int tof_sensor_params_json_parse(cJSON* json);

/**
 * @brief Write received parameters to global SOC and TOF structures
 */
static void write_params(void);

//******************************************************************************
//* Public function definitions
//******************************************************************************

int uart_setup(void){
    int status = 0;
    status = uart_callback_set(uart, uart_callback, NULL);
    status += uart_rx_enable(uart, uart_rx_buf, RX_BUF_SIZE, RECEIVE_TIMEOUT);
    return status;
} 

void notify_host_ready_for_params(void){
    strcpy(uart_tx_buf, "OK");  // Notify Raspberry that parameter transfer can be started
    uart_tx(uart, uart_tx_buf, 2, SYS_FOREVER_US);	
}

bool params_updated(void){
    if (params_received) {
        write_params();
        params_received = false; // Reset flag for next time
        return true;
    }
    return false;
}

void write_params(void) {
    int status = 0;

    cJSON* soc_params_json = cJSON_Parse(uart_rx_buf);
    if (soc_params_json == NULL) {
        // Error during parsing
        status = -EINVAL;
        goto UART_RX_RDY_json_undo;		
    }

    status = system_params_json_parse(soc_params_json);
    if (status) goto UART_RX_RDY_json_undo; 

    if (SOC.change_sensor_params) {
        if (SOC.sensor_type & SENSOR_TOF) {
            status = tof_sensor_params_json_parse(soc_params_json);
            if (status) goto UART_RX_RDY_json_undo;
        }
        //if (SOC.sensor_type & SENSOR_BIO) {
        //    status = bio_sensor_params_json_parse(soc_params_json);
        //    if (status) goto UART_RX_RDY_end;
        //}
        // ... other sensors
    }

    cJSON_Delete(soc_params_json);          
    //params_received = 1;     // Parameters updated
    strcpy(uart_tx_buf, "ACK");  // Acknowledge to RPi
    uart_tx(uart, uart_tx_buf, sizeof(uart_tx_buf), SYS_FOREVER_MS);	
    return;
UART_RX_RDY_json_undo:
    cJSON_Delete(soc_params_json);
    strcpy(uart_tx_buf, "NAK"); 
    uart_tx(uart, uart_tx_buf, sizeof(uart_tx_buf), SYS_FOREVER_MS);
}

//******************************************************************************
//* Private function definitions
//******************************************************************************

static void uart_callback(const struct device *dev, struct uart_event *evt, void *user_data) {
    int status = 0;
	switch (evt->type) {
    case UART_RX_RDY:
        if ((evt->data.rx.buf[0] == '{') && !params_received) {
            params_received = true; // Interlock to avoid race conditions
        }
        break;

    case UART_RX_BUF_REQUEST:
        // Ran out of buffer space. Append to the existing buffer
        uart_rx_buf_rsp(uart, uart_rx_buf + RX_BUF_SIZE, RX_BUF_SIZE);
        break;

	case UART_RX_DISABLED:
		uart_rx_enable(uart, uart_rx_buf, RX_BUF_SIZE, RECEIVE_TIMEOUT);
		break;
		
	default:
		break;
	}
}

static int system_params_json_parse(cJSON* json) {
    cJSON* sample_time_s_json = cJSON_GetObjectItem(json, "t_s");
    cJSON* num_samples_json = cJSON_GetObjectItem(json, "n");
    cJSON* sensor_type_json = cJSON_GetObjectItem(json, "sensor");
    cJSON* processing_json = cJSON_GetObjectItem(json, "processing");
    cJSON* backups_enabled_json = cJSON_GetObjectItem(json, "backup_en");
    cJSON* change_sensor_params_json = cJSON_GetObjectItem(json, "change_sensor_params");

    if (
        !cJSON_IsNumber(sample_time_s_json) || 
        !cJSON_IsNumber(num_samples_json) || 
        !cJSON_IsNumber(sensor_type_json) || 
        !cJSON_IsNumber(processing_json)
    ) {
        // Error: one of the required parameters is missing or not a number
        return -EINVAL;	
    }

    SOC.sample_time_ms = sample_time_s_json->valueint*1000; // Convert seconds to milliseconds
    SOC.num_samples = num_samples_json->valueint;
    SOC.sensor_type = sensor_type_json->valueint;
    SOC.processing = processing_json->valueint;
    SOC.backups_enabled = cJSON_IsTrue(backups_enabled_json);
    SOC.change_sensor_params = cJSON_IsTrue(change_sensor_params_json); // Should give false if NULL

    return 0;
}

static int tof_sensor_params_json_parse(cJSON* json) {
    cJSON* sensor_params_json = cJSON_GetObjectItem(json, "sensor_params");
    cJSON* distance_mode_json = cJSON_GetObjectItem(sensor_params_json, "distance_mode");
    cJSON* intermeas_ms_json = cJSON_GetObjectItem(sensor_params_json, "intermeas_ms");
    cJSON* timing_budget_ms_json = cJSON_GetObjectItem(sensor_params_json, "timing_budget_ms");
    cJSON* distance_threshold_low_json = cJSON_GetObjectItem(sensor_params_json, "distance_threshold_low");
    cJSON* distance_threshold_high_json = cJSON_GetObjectItem(sensor_params_json, "distance_threshold_high");
    cJSON* distance_threshold_window_json = cJSON_GetObjectItem(sensor_params_json, "distance_threshold_window");

    if (
        !cJSON_IsNumber(distance_mode_json) || 
        !cJSON_IsNumber(intermeas_ms_json) || 
        !cJSON_IsNumber(timing_budget_ms_json) || 
        !cJSON_IsNumber(distance_threshold_low_json) || 
        !cJSON_IsNumber(distance_threshold_high_json) || 
        !cJSON_IsNumber(distance_threshold_window_json)
    ) {
        // Error: one of the required parameters is missing or not a number
        return -EINVAL;	
    }

    TOF.distance_mode = distance_mode_json->valueint;
    TOF.intermeas_ms = intermeas_ms_json->valueint;
    TOF.timing_budget_ms = timing_budget_ms_json->valueint;
    TOF.distance_threshold_low = distance_threshold_low_json->valueint;
    TOF.distance_threshold_high = distance_threshold_high_json->valueint;
    TOF.distance_threshold_window = distance_threshold_window_json->valueint;

    return 0;
}