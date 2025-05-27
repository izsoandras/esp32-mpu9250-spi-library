#ifndef MPU9250_SPI_GUARD
#define MPU9250_SPI_GUARD

#include "driver/spi_master.h"
#include <stdbool.h>

#ifndef MPU9250_SLV0_TYPE
    #define MPU9250_SLV0_TYPE float
#endif
#ifndef MPU9250_SLV1_TYPE
    #define MPU9250_SLV1_TYPE float
#endif
#ifndef MPU9250_SLV2_TYPE
    #define MPU9250_SLV2_TYPE float
#endif

typedef struct {
    float x;
    float y;
    float z;
} vec3_t;

typedef enum {
    MPU9250_REG_GYRO_OFFS_X = 19,
    MPU9250_REG_GYRO_OFFS_Y = 21,
    MPU9250_REG_GYRO_OFFS_Z = 23,
    MPU9250_REG_CONF = 26,
    MPU9250_REG_GYRO_CONF = 27,
    MPU9250_REG_ACC_CONF2 = 29,
    MPU9250_REG_ACC_CONF = 28,
    MPU9250_REG_FIFO_EN = 35,
    MPU9250_REG_ACC_X = 59,
    MPU9250_REG_ACC_Y = 61,
    MPU9250_REG_ACC_Z = 63,
    MPU9250_REG_TEMP = 65,
    MPU9250_REG_GYRO_X = 67,
    MPU9250_REG_GYRO_Y = 69,
    MPU9250_REG_GYRO_Z = 71,
    MPU9250_REG_USR_CTRL = 106,
    MPU9250_REG_PWR_MGMT_1 = 107,
    MPU9250_REG_FIFO_CNT = 114,
    MPU9250_REG_FIFO_READ = 116,
    MPU9250_REG_WHOAMI = 117,
    MPU9250_REG_ACC_OFFS_X = 119,
    MPU9250_REG_ACC_OFFS_Y = 122,
    MPU9250_REG_ACC_OFFS_Z = 125,
} MPU9250_register_t;

typedef enum {
    MPU9250_GYRO_FS_250 = 0b00,
    MPU9250_GYRO_FS_500 = 0b01,
    MPU9250_GYRO_FS_1000 = 0b10,
    MPU9250_GYRO_FS_2000 = 0b11,
} MPU9250_gyro_fs_t;

typedef enum {
    MPU9250_ACC_FS_2G = 0b00,
    MPU9250_ACC_FS_4G = 0b01,
    MPU9250_ACC_FS_8G = 0b10,
    MPU9250_ACC_FS_16G = 0b11,
} MPU9250_acc_fs_t;

typedef enum {
    MPU9250_GYRO_FCHOICE_0 = 0b00,
    MPU9250_GYRO_FCHOICE_1 = 0b01,
    MPU9250_GYRO_FCHOICE_2 = 0b11,
} MPU9250_gyro_fchoice_t;

typedef enum {
    MPU9250_GYRO_DLPF_CFG_0 = 0,
    MPU9250_GYRO_DLPF_CFG_1 = 1,
    MPU9250_GYRO_DLPF_CFG_2 = 2,
    MPU9250_GYRO_DLPF_CFG_3 = 3,
    MPU9250_GYRO_DLPF_CFG_4 = 4,
    MPU9250_GYRO_DLPF_CFG_5 = 5,
    MPU9250_GYRO_DLPF_CFG_6 = 6,
    MPU9250_GYRO_DLPF_CFG_7 = 7,
} MPU9250_gyro_dlpf_cfg_t;

typedef enum {
    MPU9250_GYRO_DLPF_250Hz_8kHz = 0,
    MPU9250_GYRO_DLPF_184Hz_1kHz = 1,
    MPU9250_GYRO_DLPF_92Hz_1kHz = 2,
    MPU9250_GYRO_DLPF_41Hz_1kHz = 3,
    MPU9250_GYRO_DLPF_20Hz_1kHz = 4,
    MPU9250_GYRO_DLPF_10Hz_1kHz = 5,
    MPU9250_GYRO_DLPF_5Hz_1kHz = 6,
    MPU9250_GYRO_DLPF_3600Hz_8kHz = 7,
    MPU9250_GYRO_DLPF_8800Hz_32kHz = 8,
    MPU9250_GYRO_DLPF_3600Hz_32kHz = 9,
} MPU9250_gyro_dlpf_bw_fs_t;

typedef enum {
    MPU9250_ACC_FCHOICE_OFF = 0,
    MPU9250_ACC_FCHOICE_ON = 0b00001000,
} MPU9250_acc_fchoice_t;

typedef enum {
    MPU2950_ACC_DLPF_CFG_0 = 0,
    MPU2950_ACC_DLPF_CFG_1 = 1,
    MPU2950_ACC_DLPF_CFG_2 = 2,
    MPU2950_ACC_DLPF_CFG_3 = 3,
    MPU2950_ACC_DLPF_CFG_4 = 4,
    MPU2950_ACC_DLPF_CFG_5 = 5,
    MPU2950_ACC_DLPF_CFG_6 = 6,
    MPU2950_ACC_DLPF_CFG_7 = 7,
} MPU9250_acc_dlpf_cfg_t;

typedef enum {
    MPU9250_ACC_DLPF_218Hz_1kHz = 1,
    MPU9250_ACC_DLPF_99Hz_1kHz = 2,
    MPU9250_ACC_DLPF_45Hz_1kHz = 3,
    MPU9250_ACC_DLPF_21Hz_1kHz = 4,
    MPU9250_ACC_DLPF_10Hz_1kHz = 5,
    MPU9250_ACC_DLPF_5Hz_1kHz = 6,
    MPU9250_ACC_DLPF_420Hz_1kHz = 7,
    MPU9250_ACC_DLPF_1kHz_4kHz = 8,
} MPU9250_acc_dlpf_bw_fs_t;

typedef enum {
    MPU9250_FIFO_DIS = 0,
    MPU9250_FIFO_EN = 0b01000000,
} MPU9250_fifo_enable_t;

typedef enum {
    MPU9250_I2C_MASTER_DIS = 0,
    MPU9250_IC2_MASTER_EN = 0b00100000,
} MPU9250_i2c_master_enable_t;

typedef enum {
    MPU9250_I2C_IF_DIS =       0b00010000,
    MPU9250_FIFO_RST =         0b00000100,
    MPU9250_I2C_MASTER_RST =   0b00000010,
    MPU92550_SIGNAL_PATH_RST = 0b00000001
} MPU9250_rst_t;

typedef enum {
    MPU9250_FIFO_REPLACE = 0b00000000,
    MPU9250_FIFO_STOPWRITE = 0b01000000,
} MPU9250_fifo_mode_t;

typedef enum {
    MPU9250_FSYNC_DIS    = 0b00000000,
    MPU9250_FSYNC_TEMP   = 0b00001000,
    MPU9250_FSYNC_GYRO_X = 0b00010000,
    MPU9250_FSYNC_GYRO_Y = 0b00011000,
    MPU9250_FSYNC_GYRO_Z = 0b00100000,
    MPU9250_FSYNC_ACC_X  = 0b00101000,
    MPU9250_FSYNC_ACC_Y  = 0b00110000,
    MPU9250_FSYNC_ACC_Z  = 0b00111000,
} MPU9250_ext_fsync_set_t;

extern const float MPU9250_GYRO_SENS[4];

extern const float MPU9250_ACC_SENS[4];

typedef struct {
    // FIFO settings
    MPU9250_fifo_mode_t fifo_mode;
    bool fifo_sources[8];           // slave 3 support should be added here at index 8
    MPU9250_fifo_enable_t fifo_enabled;
    // External settings
    MPU9250_ext_fsync_set_t ext_fsync;
    // Gyro settings
    MPU9250_gyro_fs_t gyro_fs;
    MPU9250_gyro_fchoice_t gyro_fchoice;
    MPU9250_gyro_dlpf_cfg_t gyro_dlpf_cfg;
    // Acceleration settings
    MPU9250_acc_fs_t acc_fs;
    int16_t acc_default_x_offs;
    int16_t acc_default_y_offs;
    int16_t acc_default_z_offs;
    MPU9250_acc_fchoice_t acc_fchoice;
    MPU9250_acc_dlpf_cfg_t acc_dlpf_cfg;
    float g;
    // Temperature sensor settings
    float temp_sensitivity;
    double room_temp_offset;
    // I2C settings
    MPU9250_i2c_master_enable_t i2c_mst_en;
    uint8_t i2c_slave0_len;
    uint8_t i2c_slave1_len;
    uint8_t i2c_slave2_len;
} MPU9250_config_t;

typedef struct {
    int cs_pin;
    MPU9250_config_t config;
    spi_device_handle_t dev_handle;
} MPU9250_spi_device_t;

MPU9250_config_t MPU9250_get_default_config();

MPU9250_spi_device_t mpu9250_create_device(int cs_pin);

esp_err_t mpu9250_register_device(MPU9250_spi_device_t* dev, spi_host_device_t spi_host);

esp_err_t mpu9250_reset(const MPU9250_spi_device_t* dev);

esp_err_t mpu9250_read_whoami(const MPU9250_spi_device_t* dev, uint8_t* out);

esp_err_t mpu9250_read_temp(const MPU9250_spi_device_t* dev, float* out);

esp_err_t mpu9250_read_gyro(const MPU9250_spi_device_t* dev, vec3_t* out);

esp_err_t mpu9250_read_acc(const MPU9250_spi_device_t* dev, vec3_t* out);

esp_err_t mpu9250_set_gyro_fs(MPU9250_spi_device_t* dev, MPU9250_gyro_fs_t gyro_fs);

esp_err_t mpu9250_set_acc_fs(MPU9250_spi_device_t* dev, MPU9250_acc_fs_t acc_fs);

esp_err_t mpu9250_set_gyro_offs(const MPU9250_spi_device_t* dev, float x_offs, float y_offs, float z_offs);

esp_err_t mpu9250_set_gyro_dlpf(MPU9250_spi_device_t* dev, MPU9250_gyro_dlpf_bw_fs_t dlpf_setting);

esp_err_t mpu9250_update_default_acc_offs(MPU9250_spi_device_t* dev);

esp_err_t mpu9250_set_acc_offs(const MPU9250_spi_device_t* dev, float x_offs, float y_offs, float z_offs);

esp_err_t mpu9250_set_fifo_sources(MPU9250_spi_device_t* dev, const bool temp_en, const bool gyro_x_en, const bool gyro_y_en, const bool gyro_z_en, const bool acc_en, const bool slv2_en, const bool slv1_en, const bool slv0_en);

esp_err_t mpu9250_reset_fifo(const MPU9250_spi_device_t* dev);

esp_err_t mpu9250_set_fifo_enable(MPU9250_spi_device_t* dev, const MPU9250_fifo_enable_t fifo_en);

esp_err_t mpu9250_read_fifo_count(MPU9250_spi_device_t* dev, uint16_t* cnt);

esp_err_t mpu9250_read_fifo(const MPU9250_spi_device_t* dev, uint16_t sample_num, float* temp_buff, float* gyro_x_buff, float* gyro_y_buff, float* gyro_z_buff, vec3_t* acc_buff, MPU9250_SLV2_TYPE* slv2_buff, MPU9250_SLV2_TYPE (*slv2_conv)(uint8_t*), MPU9250_SLV1_TYPE* slv1_buff, MPU9250_SLV1_TYPE (*slv1_conv)(uint8_t*), MPU9250_SLV0_TYPE* slv0_buff, MPU9250_SLV0_TYPE (*slv0_conv)(uint8_t*));

esp_err_t read_int16(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, int16_t* dest);

esp_err_t read_n_bytes(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, void* dest, size_t n);

esp_err_t read_byte(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, uint8_t* dest);

esp_err_t write_byte(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, uint8_t data);

esp_err_t write_n_bytes(const MPU9250_spi_device_t* dev, MPU9250_register_t reg, uint8_t* buff, size_t n);

#endif