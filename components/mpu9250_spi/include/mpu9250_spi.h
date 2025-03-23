#ifndef MPU9250_SPI_GUARD
#define MPU9250_SPI_GUARD

#include "driver/spi_master.h"

typedef struct {
    float x;
    float y;
    float z;
} vec3_t;

typedef enum {
    MPU9250_REG_TEMP = 65,
    MPU9250_REG_WHOAMI = 117,
} MPU9250_register_t;

typedef struct {
    int cs_pin;
    float temp_sensitivity;
    double room_temp_offset;
    spi_device_handle_t dev_handle;
} MPU9250_spi_device_t;

MPU9250_spi_device_t mpu9250_create_device(int cs_pin);

esp_err_t mpu9250_register_device(MPU9250_spi_device_t* dev, spi_host_device_t spi_host);

esp_err_t mpu9250_read_whoami(const MPU9250_spi_device_t* dev, uint8_t* out);

esp_err_t mpu9250_read_temp(const MPU9250_spi_device_t* dev, float* out);

#endif