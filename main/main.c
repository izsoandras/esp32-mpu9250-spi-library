#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/timer.h"
#include "mpu9250_spi.h"

void app_main(){
    printf("Hello World\n");
    spi_bus_config_t buscfg = {
        .mosi_io_num = 25,
        .miso_io_num = 34,
        .sclk_io_num = 32,
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    MPU9250_spi_device_t mpu_dev = mpu9250_create_device(14);
    mpu9250_register_device(&mpu_dev, SPI2_HOST);

    uint8_t reg_val;
    read_byte(&mpu_dev, 117, &reg_val);
    printf("%d\n", reg_val);
}