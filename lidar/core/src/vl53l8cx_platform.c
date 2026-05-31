#include "inc/vl53l8cx_platform.h"


#define DATA_BLOCK_LEN	128
#define MIN(a, b) ((a > b) ? b : a)

int32_t VL53L8CX_InitSPI(VL53L8CX_Platform *p)
{
    p->defined = 1;
    p->speed = 3000000;
    p->channel = 0;
    p->mode = 3;
    p->fd = wiringPiSPISetupMode(p->channel, p->speed, p->mode);
    if (p->fd < 0) {
        fprintf(stderr, "Failed to initialize SPI device: %s\n", strerror(errno));
        return -1; // Return -1 for failure
    }
    

    return 0;
}
// Write one byte
int32_t VL53L8CX_WrByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t data)
{
    return VL53L8CX_WrMulti(p, reg, &data, 1); // Return 0 for success
}

// Read one byte
int32_t VL53L8CX_RdByte(VL53L8CX_Platform *p, uint16_t reg, uint8_t *data)
{
    return VL53L8CX_RdMulti(p, reg, data, 1);
}

// Write multiple bytes
int32_t VL53L8CX_WrMulti(VL53L8CX_Platform *p, uint16_t reg, uint8_t *pdata, uint32_t count)
{
	uint8_t buffer[DATA_BLOCK_LEN + 2];
	uint32_t len = MIN(count, DATA_BLOCK_LEN);
	uint16_t r = reg;
    if(p->defined != 1){
        VL53L8CX_InitSPI(p); // Initialize if not already done
    }

    if (p->fd < 0){
        fprintf(stderr, "SPI device not initialized\n");
        return -1; // Return -1 for failure
    }
    do {
        buffer[0] = ((r >> 8) & 0xFF) | (1 << 7); // High byte of register address
        buffer[1] = r & 0xFF;        // Low byte of register
        memcpy(&buffer[2], pdata, len); // Copy data to buffer for write operation
        if(wiringPiSPIDataRW(p->channel, buffer, len + 2) < 0) {
             perror("Failed to write SPI data");
             return -1; // Return -1 for failure
        }
        pdata+=len;
        count -= len;
        r += len;
        if (count < DATA_BLOCK_LEN) {
        	len = count;
        }
    } while (count > 0);

    return 0;
}

// Read multiple bytes
int32_t VL53L8CX_RdMulti(VL53L8CX_Platform *p, uint16_t reg, uint8_t *pdata, uint32_t count)
{
	uint8_t buffer[DATA_BLOCK_LEN + 2];
	uint32_t len = MIN(count, DATA_BLOCK_LEN);
	uint16_t r = reg;

    if(p->defined != 1){
        VL53L8CX_InitSPI(p); // Initialize if not already done
    }

    if (p->fd < 0){
        fprintf(stderr, "SPI device not initialized\n");
        return -1; // Return -1 for failure
    }

    do {
        buffer[0] = ((r >> 8) & 0xFF) & (~(1 << 7)); // High byte of register address
        buffer[1] = r & 0xFF;        // Low byte of register
        memset(&buffer[2], 0x00, DATA_BLOCK_LEN); // Clear buffer for read operation
        if(wiringPiSPIDataRW(p->channel, buffer, len + 2) < 0) {
             perror("Failed to read/write SPI data");
             return -1; // Return -1 for failure
        }
        memcpy(pdata, &buffer[2], len); // Copy data to buffer for write operation
        pdata+=len;
        count -= len;
        r += len;
        if (count < DATA_BLOCK_LEN) {
        	len = count;
        }
    } while (count > 0);
    return 0;
}

// Swap buffer (big-endian <-> little-endian, 4 bytes at a time)
void VL53L8CX_SwapBuffer(uint8_t *pbuffer, int size)
{
    for (int i = 0; i < size; i += 4) {
        uint8_t tmp;
        tmp = pbuffer[i];     pbuffer[i]   = pbuffer[i+3]; pbuffer[i+3] = tmp;
        tmp = pbuffer[i+1];   pbuffer[i+1] = pbuffer[i+2]; pbuffer[i+2] = tmp;
    }
}

// Delay in milliseconds
int32_t VL53L8CX_WaitMs(VL53L8CX_Platform *p, int wait_ms)
{
    (void)p;  // unused
    usleep(wait_ms*1000); // usleep takes microseconds
    return 0;
}

