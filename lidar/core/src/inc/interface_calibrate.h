#include "vl53l8cx.h"
#include "vl53l8cx_plugin_motion_indicator.h"
#include "vl53l8cx_plugin_detection_thresholds.h"
#include "vl53l8cx_plugin_xtalk.h"

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <math.h>
#include <stddef.h>
#include <stdlib.h>
#include <wiringPi.h>

#define LPN 13
#define PWREN 15
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ABS(a) ((a) < 0 ? -(a) : (a))
#define tyles 64


typedef struct
{
    VL53L8CX_Configuration conf;
    VL53L8CX_Motion_Configuration motion_conf;
    VL53L8CX_DetectionThresholds detection_thresholds;
    VL53L8CX_ResultsData results;
    uint8_t calibrated;
    int ranging_frequency;
    int integration_time;
    uint8_t resolution;
    uint8_t nb_samples;
    uint16_t distance_mm;
    uint16_t reflectance_percent;
    uint8_t *data_is_ready;

} VL53L8CX_calibrate;

int calibrate(VL53L8CX_calibrate *calib);

int printInfoSingle(VL53L8CX_calibrate *calib);

int printInfoMultiple(VL53L8CX_calibrate *calib, int times);

int getZoneClosestDistance(VL53L8CX_calibrate *calib);

int getZoneStrongestReflectance(VL53L8CX_calibrate *calib);

int getDistance(VL53L8CX_calibrate *calib, int zone);

int getReflectance(VL53L8CX_calibrate *calib, int zone);

bool checkMaterial(VL53L8CX_calibrate *calib, int spad_threshold);

int getZoneMostSpads(VL53L8CX_calibrate *calib);

int getSpads(VL53L8CX_calibrate *calib, int zone);

int get_ranging_data(VL53L8CX_Configuration *p_dev, VL53L8CX_ResultsData *p_results);

void powerON(void);

int failure(int status, const char* message);

void sleep_ms(int ms);

size_t getSizeOfCalibrateStruct(void);