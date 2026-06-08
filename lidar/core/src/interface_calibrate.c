#include "inc/interface_calibrate.h"


#define _8x8 VL53L8CX_RESOLUTION_8X8
#define _4x4 VL53L8CX_RESOLUTION_4X4
#define CONTINUOUS VL53L8CX_RANGING_MODE_CONTINUOUS
#define AUTONOMOUS VL53L8CX_RANGING_MODE_AUTONOMOUS
#define CLOSEST VL53L8CX_TARGET_ORDER_CLOSEST
#define STRONGEST VL53L8CX_TARGET_ORDER_STRONGEST
#define FREQUENCE 10
#define INTEGRATION_TIME 20
#define MAX_UINT16 65535
#define SAMPLES 10


void sleep_ms(int ms)
{
    usleep(ms * 1000); // usleep takes microseconds
}

int failure(int status, const char* message)
{
    if (status != 0)
    {
        fprintf(stderr, "%s: %s\n", message, strerror(errno));
        return -1;
    }
    return 0;
}

void powerON(void)
{
    int status = wiringPiSetupPhys(); // Initialize wiringPi library
    failure(status, "Failed to initialize wiringPi");
    pinMode(SPI_I2C_N, OUTPUT);
    pinMode(LPn, OUTPUT);
    pinMode(PWR_EN, OUTPUT);
    digitalWrite(SPI_I2C_N, LOW);
    digitalWrite(LPn, LOW);
    digitalWrite(PWR_EN, LOW);
    sleep_ms(50);
    digitalWrite(SPI_I2C_N, HIGH);
    sleep_ms(100);
    digitalWrite(PWR_EN, HIGH);
    sleep_ms(50);
    digitalWrite(LPn, HIGH);
    sleep_ms(250);
}


int calibrate(VL53L8CX_calibrate *calib)
{

    int status;
    int i = 20; // Max wait time of 2 seconds for data ready

    powerON();
    calib->resolution = _8x8;
    calib->ranging_frequency = FREQUENCE;
    calib->integration_time = INTEGRATION_TIME;
    calib->data_is_ready = (uint8_t*)malloc(sizeof(uint8_t));

    status = vl53l8cx_init(&calib->conf);
    failure(status, "Failed to initialize VL53L8CX sensor");

    status = vl53l8cx_set_resolution(&calib->conf, calib->resolution);
    failure(status, "Failed to set resolution");
    
    status = vl53l8cx_set_ranging_frequency_hz(&calib->conf, calib->ranging_frequency);
    failure(status, "Failed to set ranging frequency");
    
    status = vl53l8cx_set_integration_time_ms(&calib->conf, calib->integration_time);
    failure(status, "Failed to set integration time");

    status = vl53l8cx_set_ranging_mode(&calib->conf, CONTINUOUS);
    failure(status, "Failed to set ranging mode");

    status = vl53l8cx_set_target_order(&calib->conf, STRONGEST);
    failure(status, "Failed to set target order");

    status = vl53l8cx_start_ranging(&calib->conf);
    failure(status, "Failed to start ranging");

    do{
        sleep_ms(100);
        status = vl53l8cx_check_data_ready(&calib->conf, calib->data_is_ready);
        i--;
    }while(status != 0 && i > 1);
    if(i < 1){
        failure(status, "Data not ready after waiting");
    }
    free(calib->data_is_ready);
    calib->calibrated = 1;
    return 0;
}

uint8_t calibrate_glass( VL53L8CX_calibrate *calib, uint16_t distance_mm, uint16_t reflectance_percent)
{
    uint8_t nb_samples = SAMPLES;
    vl53l8cx_calibrate_xtalk(&calib->conf, nb_samples, reflectance_percent, distance_mm);
    return 0;
}

int get_ranging_data(VL53L8CX_calibrate *calib)
{
    int status = 0;
    int min = 0;
    int value = 0;
    do{
        min = INT32_MAX;
        status = vl53l8cx_get_ranging_data(&calib->conf, &calib->results);
        for(int i = 0; i < 64; i++){
            value = calib->results.distance_mm[i];
            min = MIN(min, value);
        }
        sleep_ms(10);
    }while(min < 1);

    return status;
}

int printInfoSingle(VL53L8CX_calibrate *calib)
{
    int status = 0;
    if(calib->calibrated != 1){
        calibrate(calib);
    }
    int y = 0;
    status = get_ranging_data(calib);
    failure(status, "Failed to get ranging data");
    for(int i = 0; i < 8; i++){
        for(int j = 0; j < 8; j++){
            y = i*8 + j;
            printf("%d:%d mm, %d s, %d t\t|",
            y,
            calib->results.distance_mm[y],
            calib->results.signal_per_spad[y],
            calib->results.target_status[y]);
        }
        printf("\n");
    }
    return 0;
}

int printInfoMultiple(VL53L8CX_calibrate *calib, int times)
{
    for(int i = 0; i < times; i++){
        printInfoSingle(calib);
        printf("\n");
        sleep_ms(1000);
    }
    return 0;
}

int getZoneClosestDistance(VL53L8CX_calibrate *calib)
{
    int status = 0;
    if(calib->calibrated != 1){
        calibrate(calib);
    }
    status = get_ranging_data(calib);
    failure(status, "Failed to get ranging data");
    int closest_distance = MAX_UINT16; // Max value for uint16_t
    int zone = 0;
    for(int i = 0; i < 64; i++){
        if(calib->results.distance_mm[i] < closest_distance && calib->results.distance_mm[i] > 0){
            closest_distance = calib->results.distance_mm[i];
            zone = i;
        }
    }
    return zone;
}

int getZoneStrongestReflectance(VL53L8CX_calibrate *calib)
{
    int status = 0;
    if(calib->calibrated != 1){
        calibrate(calib);
    }
    status = get_ranging_data(calib);
    failure(status, "Failed to get ranging data");
    int strongest_reflectance = 0;
    int zone = 0;
    for(int i = 0; i < 64; i++){
        if(calib->results.reflectance[i] > strongest_reflectance){
            strongest_reflectance = calib->results.reflectance[i];
            zone = i;
        }
    }
    return zone;
}

int getZoneMostSpads(VL53L8CX_calibrate *calib)
{
    int status = 0;
    if(calib->calibrated != 1){
        calibrate(calib);
    }
    status = get_ranging_data(calib);
    failure(status, "Failed to get ranging data");
    uint32_t spads = 0;
    int zone = 0;
    for(int i = 0; i < 64; i++){
        if(calib->results.signal_per_spad[i] > spads){
            spads = calib->results.signal_per_spad[i];
            zone = i;
        }
    }
    return zone;
}

int getSpads(VL53L8CX_calibrate *calib, int zone)
{
    int status = 0;
    if(calib->calibrated != 1){
        calibrate(calib);
    }
    status = get_ranging_data(calib);
    failure(status, "Failed to get ranging data");
    return calib->results.signal_per_spad[zone];
}

int getDistance(VL53L8CX_calibrate *calib, int zone)
{
    int status = 0;
    if(calib->calibrated != 1){
        calibrate(calib);
    }
    status = get_ranging_data(calib);
    failure(status, "Failed to get ranging data");
    return calib->results.distance_mm[zone];
}

int getReflectance(VL53L8CX_calibrate *calib, int zone)
{
    int status = 0;
    if(calib->calibrated != 1){
        calibrate(calib);
    }
    status = get_ranging_data(calib);
    failure(status, "Failed to get ranging data");
    return calib->results.reflectance[zone];
}

size_t getSizeOfCalibrateStruct(void)
{
    return sizeof(VL53L8CX_calibrate);
}

bool checkMaterial(VL53L8CX_calibrate *calib, int spad_threshold){
    int zone = getZoneMostSpads(calib);
    int spads = getDistance(calib, zone);
    if(spads > spad_threshold){
        return true;
    }
    else{
        return false;
    }
}

int main(void){
    VL53L8CX_calibrate calib;
    calibrate(&calib);
    printInfoMultiple(&calib, 10);
    return 0;
}