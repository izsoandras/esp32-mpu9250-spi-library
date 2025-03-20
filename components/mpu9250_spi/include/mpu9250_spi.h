#ifndef MPU9250_SPI_GUARD
#define MPU9250_SPI_GUARD

#include "driver/spi_master.h"

typedef enum {
    MPU9250_REG_TEMP = 65,
    MPU9250_REG_WHOAMI = 117,
} MPU9250_register_t;

typedef struct {
    int cs_pin;
    spi_device_handle_t dev_handle;
} MPU9250_spi_device_t;

MPU9250_spi_device_t mpu9250_create_device(int cs_pin);

esp_err_t mpu9250_register_device(MPU9250_spi_device_t* dev, spi_host_device_t spi_host);

uint8_t mpu9250_read_whoami(const MPU9250_spi_device_t* dev);

#endif