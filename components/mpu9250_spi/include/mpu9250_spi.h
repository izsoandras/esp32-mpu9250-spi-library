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
    MPU9250_REG_GYRO_X = 67,
    MPU9250_REG_GYRO_Y = 69,
    MPU9250_REG_GYRO_Z = 71,
    MPU9250_REG_WHOAMI = 117,
} MPU9250_register_t;

typedef enum {
    MPU9250_GYRO_FS_250 = 0b00,
    MPU9250_GYRO_FS_500 = 0b01,
    MPU9250_GYRO_FS_1000 = 0b10,
    MPU9250_GYRO_FS_2000 = 0b11,
} MPU9250_gyro_fs_t;

extern const float MPU9250_GYRO_SENS[4];

typedef struct {
    float temp_sensitivity;
    double room_temp_offset;
    MPU9250_gyro_fs_t gyro_fs;
} MPU9250_config_t;

typedef struct {
    int cs_pin;
    MPU9250_config_t config;
    spi_device_handle_t dev_handle;
} MPU9250_spi_device_t;

MPU9250_spi_device_t mpu9250_create_device(int cs_pin);

esp_err_t mpu9250_register_device(MPU9250_spi_device_t* dev, spi_host_device_t spi_host);

esp_err_t mpu9250_read_whoami(const MPU9250_spi_device_t* dev, uint8_t* out);

esp_err_t mpu9250_read_temp(const MPU9250_spi_device_t* dev, float* out);

esp_err_t mpu9250_read_gyro(const MPU9250_spi_device_t* dev, vec3_t* out);

#endif