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
 * Copies the value of the register on the given address to the given destination.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t to read from
 * @param reg The register address which is read
 * @param dest Output destination
  * 
  * @return ESP error code
 */
static esp_err_t read_byte(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, uint8_t* dest){
    spi_transaction_t spi_tran = {
        .addr = reg | 0b10000000,
        .length = 8,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    esp_err_t err = spi_device_polling_transmit(dev->dev_handle, &spi_tran);

    *dest = spi_tran.rx_data[0];
    return err;
}
 /**
  * Read a 16 bit unsigned integer from the given device, starting from the given register.
  * 
  * Copies the value of 2 register, starting from the given address, 
  * interpreted as a 16 bit unsigned integer to the given destination.
  * 
  * @param dev Pointer to the MPU9250_spi_device_t to read from
  * @param reg The start register address of the reading
  * @param dest Output destination
  * 
  * @return ESP error code
  */
static esp_err_t read_uint16(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, uint16_t* dest){
    spi_transaction_t spi_tran = {
        .addr = reg | 0b10000000,
        .length = 16,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    esp_err_t err = spi_device_polling_transmit(dev->dev_handle, &spi_tran);
    *dest =  (((uint16_t)spi_tran.rx_data[0]) << 8) | spi_tran.rx_data[1];
    return err;
}

 /**
  * Read a 16 bit signed integer value from the given device, starting from the given register.
  * 
  * Copies the value of 2 register, starting from the given address, 
  * interpreted as a 16 bit signed integer to the given destination.
  * 
  * @param dev Pointer to the MPU9250_spi_device_t to read from
  * @param reg The start register address of the reading
  * @param dest Output destination
  * 
  * @return ESP error code
  */
 static esp_err_t read_int16(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, int16_t* dest){
    spi_transaction_t spi_tran = {
        .addr = reg | 0b10000000,
        .length = 16,
        .flags = SPI_TRANS_USE_RXDATA,
    };

    esp_err_t err = spi_device_polling_transmit(dev->dev_handle, &spi_tran);
    *dest = (((int16_t)spi_tran.rx_data[0]) << 8) | spi_tran.rx_data[1];
    return err;
}

/**
 * Read the 'Who am I?' register (117) of the given device
 * 
 * Reads the 'Who am I?' value from register 117 and copies the result
 * to @see out.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t to read from
 * @param out Output destination
 * 
 * @return ESP error code
 */
esp_err_t mpu9250_read_whoami(const MPU9250_spi_device_t* dev, uint8_t* out){
    return read_byte(dev, MPU9250_REG_WHOAMI, out);
}

/**
 * Read the tempearture registers (65, 66) of the given device
 * 
 * Reads the temperature measurement from registers 65-66 and
 * copies the result to @see out in °C.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t to read from
 * @param out Output destination
 * 
 * @return 
 */
esp_err_t mpu9250_read_temp(const MPU9250_spi_device_t* dev, float* out){
    int16_t raw_temp;
    esp_err_t err = read_int16(dev, MPU9250_REG_TEMP, &raw_temp);
    *out = (raw_temp - dev->room_temp_offset) / dev->temp_sensitivity + 21;
    return err;
}
