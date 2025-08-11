#include <cmr_sd_benchmark.h>
#include <cmr_sd_lib.h>
#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include "main.h"


#define TEST_SIZE 512000*2 // 1M Test File
#define BUF_SIZE 8192


uint32_t sd_benchmark_write(const char *filename, uint32_t size_bytes) {
    FIL file;
    UINT written;
    uint8_t buffer[BUF_SIZE];
    memset(buffer, 0xAA, sizeof(buffer));

    FRESULT res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        printf("f_open failed: %d\r\n", res);
        return 0;
    }

    uint32_t start = HAL_GetTick();
    uint32_t remaining = size_bytes;

    while (remaining > 0) {
        UINT to_write = (remaining > sizeof(buffer)) ? sizeof(buffer) : remaining;
        res = f_write(&file, buffer, to_write, &written);
        if (res != FR_OK || written != to_write) {
            printf("f_write error\r\n");
            break;
        }
        remaining -= written;
    }

    f_close(&file);
    uint32_t elapsed = HAL_GetTick() - start;
    printf("Write %lu bytes in %lu ms\r\n", size_bytes, elapsed);
    return elapsed;
}

uint32_t sd_benchmark_read(const char *filename, uint32_t size_bytes) {
    FIL file;
    UINT read;
    uint8_t buffer[BUF_SIZE];

    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        printf("f_open failed: %d\r\n", res);
        return 0;
    }

    uint32_t start = HAL_GetTick();
    uint32_t remaining = size_bytes;

    while (remaining > 0) {
        UINT to_read = (remaining > sizeof(buffer)) ? sizeof(buffer) : remaining;
        res = f_read(&file, buffer, to_read, &read);
        if (res != FR_OK || read != to_read) {
            printf("f_read error\r\n");
            break;
        }
        remaining -= read;
    }

    f_close(&file);
    uint32_t elapsed = HAL_GetTick() - start;
    printf("Read %lu bytes in %lu ms\r\n", size_bytes, elapsed);
    return elapsed;
}

void sd_benchmark(void) {
    if (sd_mount() == FR_OK) {
        printf("Starting Benchmark Test\r\n");
        uint32_t w = sd_benchmark_write("bench.bin", TEST_SIZE);
        uint32_t r = sd_benchmark_read("bench.bin", TEST_SIZE);

        if (w > 0) printf("Write speed: %lu KB/s\r\n", (TEST_SIZE / 1024 * 1000) / w);
        if (r > 0) printf("Read  speed: %lu KB/s\r\n", (TEST_SIZE / 1024 * 1000) / r);

        sd_unmount();
    }
}
