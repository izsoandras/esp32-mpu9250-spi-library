#include "mpu9250_spi.h"
#include <string.h>

/**
 * Sensitivity for different gyroscope fullscale setting [LSB/(°/s)]
 * = 2^16/FS
 */
const float MPU9250_GYRO_SENS[4] = {131.072, 65.536, 32.768, 16.384};

/**
 * Sensitivity for different accelerometer fullscale setting [LSB/g]
 * = 2^16/FS
 */
const float MPU9250_ACC_SENS[4] = {16384, 8192, 4096, 2048};

/**
 * Initialize an MPU9250_config_t with default values
 * 
 * @return Default configuration set for the sensor
 */
MPU9250_config_t MPU9250_get_default_config(){
    MPU9250_config_t ret = {
        .fifo_mode = MPU9250_FIFO_REPLACE,
        .ext_fsync = MPU9250_FSYNC_DIS,
        .gyro_fs = MPU9250_GYRO_FS_250,
        .gyro_fchoice = MPU9250_GYRO_FCHOICE_2,
        .gyro_dlpf_cfg = MPU9250_GYRO_DLPF_CFG_0,
        .acc_fs = MPU9250_ACC_FS_2G,
        .acc_default_x_offs = 0, //-3176,
        .acc_default_y_offs = 0, //-6996,
        .acc_default_z_offs = 0, //10774,
        .g = 9.8067,
        .room_temp_offset = 0,
        .temp_sensitivity = 333.87,
    };
    return ret;
}

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
        .config = MPU9250_get_default_config()
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
        .clock_speed_hz = 1000000, // 0,25 MHz, maximum 1 MHz lehet
        .spics_io_num = dev->cs_pin,       // CS Pin
        .queue_size = 1,
    };

    return spi_bus_add_device(spi_host, &devcfg, &(dev->dev_handle));
}

/**
 * @brief Construct content of configuration register based on the device struct
 * 
 * @param dev Pointer to the MPU9250_spi_device_t whose configuration register content is to be constructed
 * @return Register value
 */
static uint8_t build_config_reg(const MPU9250_spi_device_t* dev){
    return 0b01111111 & (dev->config.fifo_mode | dev->config.ext_fsync | dev->config.gyro_dlpf_cfg);
}

/**
 * @brief Construct content of gyroscope configuration register based on the device struct
 * 
 * @param dev Pointer to the MPU9250_spi_device_t whose gyroscope configuration register content is to be constructed
 * @return Register value 
 */
static uint8_t build_gyro_config_reg(const MPU9250_spi_device_t* dev){
    return 0b00011011 & ((dev->config.gyro_fs << 3) | (dev->config.gyro_fchoice^0b11));
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
esp_err_t read_byte(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, uint8_t* dest){
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
 esp_err_t read_int16(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, int16_t* dest){
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
 * Read the given number of bytes to the pointed destination starting from the given register of the given device.a64l
 * 
 * Copyes @see n number of bytes, starting from the register @see reg
 * to the memory location starting at @see out. No checking is done,
 * the plain representation is copied. interpreting it is up to the calling location.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t to read from
 * @param reg The start register address of the reading
 * @param dest Output destination
 * @param n Number of bytes to read
 * 
 * @return ESP error code
 */
esp_err_t read_n_bytes(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, void* dest, size_t n){
    spi_transaction_t spi_tran = {
        .addr = reg | 0b10000000,
        .length = n*8,
        .rx_buffer = dest,
    };

    return spi_device_polling_transmit(dev->dev_handle, &spi_tran);
}

/**
 * Send one byte to the sensor
 * 
 * @param dev Pointer to the MPU9250_spi_device_t to write
 * @param reg The register address to write
 * @param data Byte to write
 * 
 * @return ESP error code
 */
esp_err_t write_byte(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, uint8_t data){
    spi_transaction_t spi_tran = {
        .addr = reg & 0b01111111,
        .length = 8,
        .tx_data[0] = data,
        .flags = SPI_TRANS_USE_TXDATA,
    };

    return spi_device_polling_transmit(dev->dev_handle, &spi_tran);
}

/**
 * Send the bytes from @see buff consecutively starting at the address given in @see reg.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t to write to
 * @param reg The start register address of the writing
 * @param buff Bytes to be sent
 * @param n Number of bytes to send
 * 
 * @return ESP error code
 */
esp_err_t write_n_bytes(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, uint8_t* buff, size_t n){
    spi_transaction_t spi_tran = {
        .addr = reg & 0b01111111,
        .length = n*8,
    };
    if(n <= 4){
        for(uint8_t i = 0; i < n; i++)
            spi_tran.tx_data[i] = buff[i];
            
        spi_tran.flags = SPI_TRANS_USE_TXDATA;
    }else{
        spi_tran.tx_buffer = buff;
    }

    return spi_device_polling_transmit(dev->dev_handle, &spi_tran);
}

/**
 * Issues reset for the sensor
 * 
 * Reset command is sent by writing the value in the register.
 * The completion of the reset is not waited for. The sensor clears the
 * same bit when reset is done.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t to reset
 * 
 * @return ESP error code
 */
esp_err_t mpu9250_reset(const MPU9250_spi_device_t* dev){
    return write_byte(dev, MPU9250_REG_PWR_MGMT_1, 0b10000000);
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
 * @return ESP error code
 */
esp_err_t mpu9250_read_temp(const MPU9250_spi_device_t* dev, float* out){
    int16_t raw_temp;
    esp_err_t err = read_int16(dev, MPU9250_REG_TEMP, &raw_temp);
    *out = (raw_temp - dev->config.room_temp_offset) / dev->config.temp_sensitivity + 21;
    return err;
}

/**
 * Read the latest gyroscope measurements (67-72) of the given device
 * 
 * Reads the latest gyroscope measurement from registers 67-72 and
 * copies the result to @see out in deg/s.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t to read from
 * @param out Output destination
 * 
 * @return ESP error code
 */
esp_err_t mpu9250_read_gyro(const MPU9250_spi_device_t* dev, vec3_t* out){
    uint8_t gyro_data[6];
    esp_err_t err = read_n_bytes(dev, MPU9250_REG_GYRO_X, gyro_data, 6);
    
    out->x = (int16_t)(gyro_data[0]<<8 | gyro_data[1]) / MPU9250_GYRO_SENS[dev->config.gyro_fs];
    out->y = (int16_t)(gyro_data[2]<<8 | gyro_data[3]) / MPU9250_GYRO_SENS[dev->config.gyro_fs];
    out->z = (int16_t)(gyro_data[4]<<8 | gyro_data[5]) / MPU9250_GYRO_SENS[dev->config.gyro_fs];
    return err;
}

/**
 * Read the latest accelerometer measurements (59-64) of the given device
 * 
 * Reads the latest accelerometer measurement from registers 59-64 and
 * copies the result to @see out in m/s^2.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t to read from
 * @param out Output destination
 * 
 * @return ESP error code
 */
esp_err_t mpu9250_read_acc(const MPU9250_spi_device_t* dev, vec3_t* out){
    uint8_t acc_data[6];
    esp_err_t err = read_n_bytes(dev, MPU9250_REG_ACC_X, acc_data, 6);
    
    out->x = (int16_t)(acc_data[0]<<8 | acc_data[1]) / MPU9250_ACC_SENS[dev->config.acc_fs] * dev->config.g;
    out->y = (int16_t)(acc_data[2]<<8 | acc_data[3]) / MPU9250_ACC_SENS[dev->config.acc_fs] * dev->config.g;
    out->z = (int16_t)(acc_data[4]<<8 | acc_data[5]) / MPU9250_ACC_SENS[dev->config.acc_fs] * dev->config.g;
    return err;
}

/**
 * Set gyroscope full scale. Sends the update to the sensor immediately.
 * 
 * Changes the gyroscope fullscale setting. IMPORTANT: since the register contains
 * other options aswell, the MPU9250_spi_device_t structure is used to fill the data!
 * Pay attention to keep it consistent! (Values should only be change through functions)
 * 
 * @param dev Pointer to the MPU9250_spi_device_t whose gyroscope full scale is to be shifted
 * @param gyro_fs New full scale CONFIG_BOOTLOADER_COMPILER_OPTIMIZATION_SIZE
 * 
 * @return ESP error code
 */
esp_err_t mpu9250_set_gyro_fs(MPU9250_spi_device_t* dev, MPU9250_gyro_fs_t gyro_fs){
    dev->config.gyro_fs = gyro_fs;
    uint8_t byte = build_gyro_config_reg(dev);
    return write_byte(dev, MPU9250_REG_GYRO_CONF, byte);
}

/**
 * Set accelerometer full scale. Sends the update to the sensor immediately.
 * 
 * Changes the accelerometer fullscale setting. IMPORTANT: since the register contains
 * other options aswell, the MPU9250_spi_device_t structure is used to fill the data!
 * Pay attention to keep it consistent! (Values should only be change through functions)
 * 
 * @param dev Pointer to the MPU9250_spi_device_t whose accelerometer full scale is to be shifted
 * @param gyro_fs New full scale CONFIG_BOOTLOADER_COMPILER_OPTIMIZATION_SIZE
 * 
 * @return ESP error code
 */
esp_err_t mpu9250_set_acc_fs(MPU9250_spi_device_t* dev, MPU9250_acc_fs_t acc_fs){
    uint8_t byte = ((uint8_t)acc_fs << 3);
    dev->config.acc_fs = acc_fs;
    return write_byte(dev, MPU9250_REG_ACC_CONF, byte);
}

/**
 * Set the gyroscope offset values
 * 
 * Sets the gyroscope offset according to the given values in °/s.
 * The offset is automatically scaled by the sensor according to 
 * the full scale setting.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t whose gyroscope offset is to be changed
 * @param x_offs Offset for the X axis [°/s]
 * @param y_offs Offset for the Y axis [°/s]
 * @param z_offs Offset for the Z axis [°/s]
 * 
 * @return ESP error code
 */
esp_err_t mpu9250_set_gyro_offs(const MPU9250_spi_device_t* dev, float x_offs, float y_offs, float z_offs){
    float offs[] = {x_offs, y_offs, z_offs};
    uint8_t bytes[6];

    int16_t offs_temp;
    for(uint8_t i = 0; i < 3; i++){
        offs_temp = offs[i] * MPU9250_GYRO_SENS[0] / 4; // formula from the register map document reduces to this
        bytes[2*i] = (offs_temp >> 8) & 0xFF;
        bytes[2*i+1] = offs_temp & 0xFF;
    }

    return write_n_bytes(dev, MPU9250_REG_GYRO_OFFS_X, bytes, 6);
}

/**
 * Overwrites the default accelerometer offset values.
 * 
 * Reads the current accelerometer offset values from the sensor and updates the
 * stored values. This is necessary for @see mpu9250_set_acc_offs to function properly.
 * Issue this command once after reset, before changing the offset values!
 * 
 * @param dev Pointer to the MPU9250_spi_device_t whose accelerometer offset is to be read
 * 
 * @return ESP error code
 */
esp_err_t mpu9250_update_default_acc_offs(MPU9250_spi_device_t* dev){
    esp_err_t ret;
    int16_t offsets[3];

    for(uint8_t i = 0; i < 3; i++){
        ret = read_int16(dev, MPU9250_REG_ACC_OFFS_X+i*3, offsets+i); // register is stepped every 3 because there is 1 register between offset registers
        if(ret != ESP_OK)
            return ret;
    }

    dev->config.acc_default_x_offs = offsets[0];
    dev->config.acc_default_y_offs = offsets[1];
    dev->config.acc_default_z_offs = offsets[2];

    return ESP_OK;
}

/**
 * Set the accelerometer offset values
 * 
 * Sets the accelerometer offset according ot the given values in m/s^2.
 * The offset is automatically scaled by the sensor according to the full scale setting.
 * IMPORTANT! The offset registers contain facoty values by default, and the offset has to be
 * set relative to those! Don't forget to update those values according to your sensor!
 * This can be done either by hand or the @see mpu9250_update_default_acc_offs function after reset.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t whose accelerometer offset is to be changed
 * @param x_offs Offset for the X axis [m/s^2]
 * @param y_offs Offset for the Y axis [m/s^2]
 * @param z_offs Offset for the Z axis [m/s^2]
 * 
 * @return ESP error code
 */
esp_err_t mpu9250_set_acc_offs(const MPU9250_spi_device_t* dev, float x_offs, float y_offs, float z_offs){
    float offs[] = {x_offs, y_offs, z_offs};
    int16_t def_offs[] = {dev->config.acc_default_x_offs, dev->config.acc_default_y_offs, dev->config.acc_default_z_offs};
    uint8_t bytes[8];   // Memory addresses are not continously spaced
    bytes[2] = 0;
    bytes[5] = 0;

    int16_t offs_temp;
    for(uint8_t i = 0; i < 3; i++){
        offs_temp = def_offs[i] + offs[i] / dev->config.g * 2048; // available offset is +/-16g on each full scale setting, *2 because the 0th bit is reserved and [1:15] bits are used -> (2^15-1)/32 * 2
        bytes[3*i] = (offs_temp >> 8) & 0xFF;
        bytes[3*i+1] = offs_temp & 0xFE;
    }

    return write_n_bytes(dev, MPU9250_REG_ACC_OFFS_X, bytes, 8);
}

/**
 * @brief Change the DLPF setting for the gyroscope
 * 
 * Sets the FCHOICE_B and DLPF_CFG values of the sensor,
 * based on the required digital low pass filter setting.
 * 
 * @param dev Pointer to the MPU9250_spi_device_t whose DLPF setting is to be changed
 * @param dlpf_setting The cutoff-frequnecy/sample rate setting to set
 * 
 * @return ESP error code
 */
esp_err_t mpu9250_set_gyro_dlpf(MPU9250_spi_device_t* dev, MPU9250_gyro_dlpf_bw_fs_t dlpf_setting){
    switch(dlpf_setting){
        case MPU9250_GYRO_DLPF_8800Hz_32kHz:
            dev->config.gyro_fchoice = MPU9250_GYRO_FCHOICE_0;
            dev->config.gyro_dlpf_cfg = MPU9250_GYRO_DLPF_CFG_0;
            break;
        case MPU9250_GYRO_DLPF_3600Hz_32kHz:
            dev->config.gyro_fchoice = MPU9250_GYRO_FCHOICE_1;
            dev->config.gyro_dlpf_cfg = MPU9250_GYRO_DLPF_CFG_0;
            break;
        default:
            dev->config.gyro_fchoice = MPU9250_GYRO_FCHOICE_2;
            dev->config.gyro_dlpf_cfg = dlpf_setting;
            break;
    }

    uint8_t bytes[2] = {
        build_config_reg(dev),
        build_gyro_config_reg(dev)
    };

    return write_n_bytes(dev, MPU9250_REG_CONF, bytes, 2);
}