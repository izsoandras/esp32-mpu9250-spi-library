#include "mpu9250_spi.h"

/**
 * Initialize a MPU9250_spi_device_t structure.a64l
 * Instance is created, but not added to any SPI bus.
 * 
 * @param cs_pin Chip select GPIO pin
 * 
 * @return Instance of sensor struct
*/
MPU9250_spi_device_t mpu9250_create_device(int cs_pin){
    // Create device configuration struct
    MPU9250_spi_device_t ret = {
        .cs_pin = cs_pin,
        .room_temp_offset = 0,
        .temp_sensitivity = 333.87,
    };

    return ret;
}

/**
 * Register the MPU9250_spi_device_t to an SPI peripherial of the ESP.
 * The MPU9250_spi_device_t is registered to the given SPI peripherial.
 * The device is added with it's default configuration.a64l
 * 
 * @param dev Pointer to the MPU9250_spi_device_t struct
 * @param spi_host the SPI peripherial identifier
 * 
 * @return Success flag of the device addition
 */
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

/**
 * Read a single byte from the given register of the given device.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t to read from
 * @param reg The register address which is read
 * 
 * @return Value read from the register
 */
static uint8_t read_byte(const MPU9250_spi_device_t* dev, MPU9250_register_t reg){
    spi_transaction_t spi_tran = {
        .addr = reg | 0b10000000,
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->dev_handle, &spi_tran));

    return spi_tran.rx_data[0];
}
 /**
  * Read a 16 bit unsigned integer from the given device, starting from the given register.
  * 
  * @param dev Pointer to the MPU9250_spi_device_t to read from
  * @param reg The start register address of the reading
  * 
  * @return The value of the registers interpreted as one 16 bit unsigned integer
  */
static uint16_t read_uint16(const MPU9250_spi_device_t* dev, MPU9250_register_t reg){
    spi_transaction_t spi_tran = {
        .addr = reg | 0b10000000,
        .length = 16,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->dev_handle, &spi_tran));
    return (((uint16_t)spi_tran.rx_data[0]) << 8) | spi_tran.rx_data[1];
}

 /**
  * Read a 16 bit signed integer value from the given device, starting from the given register.
  * 
  * @param dev Pointer to the MPU9250_spi_device_t to read from
  * @param reg The start register address of the reading
  * 
  * @return The value of the registers interpreted as one 16 bit signed integer
  */
 static int16_t read_int16(const MPU9250_spi_device_t* dev, MPU9250_register_t reg){
    spi_transaction_t spi_tran = {
        .addr = reg | 0b10000000,
        .length = 16,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    ESP_ERROR_CHECK(spi_device_polling_transmit(dev->dev_handle, &spi_tran));
    return (((int16_t)spi_tran.rx_data[0]) << 8) | spi_tran.rx_data[1];
}

/**
 * Read the 'Who am I?' register of the given device
 * 
 * @param dev Pointer to the MPU9250_spi_device_t to read from
 * @return WHOAMI value
 */
uint8_t mpu9250_read_whoami(const MPU9250_spi_device_t* dev){
    return read_byte(dev, MPU9250_REG_WHOAMI);
}

float mpu9250_read_temp(const MPU9250_spi_device_t* dev){
    int16_t raw_temp = read_int16(dev, MPU9250_REG_TEMP);
    return (raw_temp - dev->room_temp_offset) / dev->temp_sensitivity + 21;
}
