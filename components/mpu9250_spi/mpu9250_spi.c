#include "mpu9250_spi.h"

MPU9250_spi_device_t mpu9250_create_device(int cs_pin){
    // Create device configuration struct
    MPU9250_spi_device_t ret = {
        .cs_pin = cs_pin,
    };

    return ret;
}

esp_err_t mpu9250_register_device(MPU9250_spi_device_t* dev, spi_host_device_t spi_host){
    spi_device_interface_config_t devcfg = {
        .mode = 0, // SPI mode 0, alapértelmezetten 0 az órajel értéke, és felfutó élre történik mintavételezés
        .address_bits = 8,
        .clock_speed_hz = 250000, // 0,25 MHz, maximum 1 MHz lehet
        .spics_io_num = dev->cs_pin,       // CS Pin
        .queue_size = 1,
    };

    return spi_bus_add_device(spi_host, &devcfg, &(dev->dev_handle));
}

static uint8_t read_byte(const MPU9250_spi_device_t* dev, MPU9250_register_t reg){
    spi_transaction_t spi_tran = {
        .addr = reg | 0b10000000,
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->dev_handle, &spi_tran));

    return spi_tran.rx_data[0];
}

static uint16_t read_uint16(const MPU9250_spi_device_t* dev, MPU9250_register_t reg){
    spi_transaction_t spi_tran = {
        .addr = reg | 0b10000000,
        .length = 16,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->dev_handle, &spi_tran));
    return (((int16_t)spi_tran.rx_data[0]) << 8) | spi_tran.rx_data[1];
}

uint8_t mpu9250_read_whoami(const MPU9250_spi_device_t* dev){
    return read_byte(dev, MPU9250_REG_WHOAMI);
}
