
#ifndef _MAIN_H_
#define _MAIN_H_

#include <stdint.h>

//extern uint32_t T_samples;      // time period between samples in ms (equals 1/fs)
//extern uint16_t n_samples;      // communication incidence: Number of samples per communication event
//extern uint8_t EN_Backups;      // Backup sampling progress and samples (if 1: Doesn't guarantee for a fixed T_samples within the n_samples taken)
//extern uint16_t Sensor_Type;          // MSB -> LSB: 0, MIC, GPS, BIO, TOF, PMS, IMU, TMP
//extern uint8_t Processing;      // Post-processing to be performed on the samples


enum states{
    OFF = 1000,                // Reserved for when DC/DC converter is off
    BOOTING = 0,            // Reserved for when the DC/DC converter has turned on but nRF has not reached the IDLE state yet
    SLEEP = 1,              // Unused
    IDLE = 2,
    SAMPLING = 3,
    PROCESSING = 4,
    COMMUNICATING = 5,
    BACKUP = 6,             // Backup & Restore
    ALWAYS_ON_IDLE = 99,    // Internal state; not reflected in state register
};

extern enum states state;   // SoC State

struct Benchmark_params {
    uint32_t sample_time_ms;    // time period between samples (equals 1/fs)
    uint16_t num_samples;       // communication incidence: Number of samples per communication event
    uint16_t sensor_type;       // MSB -> LSB: 0, MIC, GPS, BIO, TOF, PMS, IMU, TMP
    uint8_t processing;         // Post-processing to be performed on the samples
    bool backups_enabled;    // Backup sampling progress and samples (if 1: Doesn't guarantee for a fixed SOC.sample_time_ms within the n_samples taken)
    bool change_sensor_params; // Flag to indicate if sensor parameters should be changed
};

struct TOF_params {
    uint8_t distance_mode;
    uint16_t intermeas_ms;
    uint16_t timing_budget_ms;
    uint16_t distance_threshold_low;
    uint16_t distance_threshold_high;
    uint8_t distance_threshold_window;
};

extern struct Benchmark_params SOC;
extern struct TOF_params TOF;


#endif // _MAIN_H_