#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <wiringPi.h>
#include "src/inc/vl53l8cx_api.h"

#define LPN 13
#define PWREN 15
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

int failure(int status, const char* message);
void powerON(void);


int main()
{ 
    VL53L8CX_ResultsData results;
    VL53L8CX_Configuration dev;
    uint8_t status;
    int nearest_distance = 100000000;
    int most_reflectance = 0;
    powerON();

    status = vl53l8cx_init(&dev);
    failure(status, "Failed to initialize VL53L8CX sensor");

    status = vl53l8cx_set_resolution(&dev, VL53L8CX_RESOLUTION_8X8);
    failure(status, "Failed to set resolution");
    
    status = vl53l8cx_set_ranging_frequency_hz(&dev, 10);
    failure(status, "Failed to set ranging frequency");
    
    status = vl53l8cx_set_integration_time_ms(&dev, 20);
    failure(status, "Failed to set integration time");

    status = vl53l8cx_start_ranging(&dev);
    failure(status, "Failed to start ranging");
    status = vl53l8cx_get_ranging_data(&dev, &results);
    failure(status, "Failed to get ranging data");
    usleep(100000); // usleep takes microseconds

    int y = 0;
    int increment = 0;
    do
    {
        status = vl53l8cx_get_ranging_data(&dev, &results);
        failure(status, "Failed to get ranging data");
        usleep(1000000);
        
        for(int i = 0; i < 8; i++)    {
            for(int j = 0; j < 8; j++){
                y = i*8 + j;
                printf("%d:%d mm, %d p\t|",
                y,
                results.distance_mm[y],
                results.reflectance[y]);
                nearest_distance = MIN(nearest_distance, results.distance_mm[y]);
                most_reflectance = MAX(most_reflectance, results.reflectance[y]);
            }
            printf("\n");
        }
        printf("\n");
        y = 0;
        increment++;
    }  while(((nearest_distance < 50) && (nearest_distance > 0)) || (increment < 30));
    printf("Nearest distance: %d mm\n", nearest_distance);
    printf("Most reflectance: %d p\n", most_reflectance);
    return 0;

}

void powerON(void){
    wiringPiSetupPhys();
    pinMode(LPN, OUTPUT);
    digitalWrite(LPN, LOW);
    pinMode(PWREN, OUTPUT);
    digitalWrite(PWREN, LOW);
    usleep(100 * 1000); // usleep takes microseconds
    digitalWrite(PWREN, HIGH);
    usleep(50 * 1000); // usleep takes microseconds
    digitalWrite(LPN, HIGH);
    usleep(250 * 1000); // usleep takes microseconds
}

int failure(int status, const char* message) {
    if (status != VL53L8CX_STATUS_OK) {
        fprintf(stderr, "%s\n", message);
        return -1;
    }
    return 0;
}



