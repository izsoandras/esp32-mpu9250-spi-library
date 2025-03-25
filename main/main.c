#include <stdio.h>
#include "driver/spi_master.h"
#include "mpu9250_spi.h"
#include <string.h>
#include "driver/uart.h"

void app_main(void)
{
    uart_set_baudrate(UART_NUM_0, 1000000);
    printf("Hello World\n");
    spi_bus_config_t buscfg = {
        .mosi_io_num = 25,
        .miso_io_num = 34,
        .sclk_io_num = 32,
        // .quadwp_io_num = -1,
        // .quadhd_io_num = -1,
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    MPU9250_spi_device_t mpu_dev = mpu9250_create_device(14);
    mpu9250_register_device(&mpu_dev, SPI2_HOST);

    uint8_t whoami;
    float temp;
    vec3_t gyro_data;
    vec3_t acc_data;
    uint8_t uart_buf[12];
    uint8_t i = 0;

    mpu9250_read_whoami(&mpu_dev, &whoami);
    printf("WHOAMI: %d\n", whoami);
    while(1){
        mpu9250_read_temp(&mpu_dev, &temp);
        mpu9250_read_gyro(&mpu_dev, &gyro_data);
        mpu9250_read_acc(&mpu_dev, &acc_data);
        // Human readable print
        printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\n", 
            temp, 
            gyro_data.x, gyro_data.y, gyro_data.z,
            acc_data.x, acc_data.y, acc_data.z
        );
        // Binary print
        // memcpy(uart_buf, &gyro_data.x, 4);
        // memcpy(uart_buf+4, &gyro_data.y, 4);
        // memcpy(uart_buf+8, &gyro_data.z, 4);
        // for(i = 0; i<12; i++)
        //     printf("%c",uart_buf[i]);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}