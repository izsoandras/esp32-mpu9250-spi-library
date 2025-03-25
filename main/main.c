#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/timer.h"
#include "mpu9250_spi.h"

#define SAMPLE_SIZE 4000

bool IRAM_ATTR timer_notify_isr_cb(void* args){
    TaskHandle_t* task_to_awake = (TaskHandle_t*) args;
    BaseType_t higher_priority_awoken = pdFALSE;
    xTaskNotifyFromISR(*task_to_awake, 0, eNoAction, &higher_priority_awoken);

    return higher_priority_awoken == pdTRUE;
}


// float data[SAMPLE_SIZE][6];

void app_main(void)
{
    // uart_set_baudrate(UART_NUM_0, 1000000);
    printf("Hello World\n");

    timer_config_t tcfg = {
        .divider = 2,
        .counter_en = TIMER_PAUSE,
        .counter_dir = TIMER_COUNT_UP,
        .alarm_en = TIMER_ALARM_EN,
        .auto_reload = TIMER_AUTORELOAD_EN
    };
    timer_init(TIMER_GROUP_0, TIMER_0, &tcfg);
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, ((uint64_t)0)-1);
    // timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    // TaskHandle_t* main_task_h = malloc(sizeof(TaskHandle_t));
    // *main_task_h = xTaskGetCurrentTaskHandle();
    // timer_isr_callback_add(TIMER_GROUP_0, TIMER_0, timer_notify_isr_cb, main_task_h, 0);

    spi_bus_config_t buscfg = {
        .mosi_io_num = 25,
        .miso_io_num = 34,
        .sclk_io_num = 32,
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));

    MPU9250_spi_device_t mpu_dev = mpu9250_create_device(14);
    mpu9250_register_device(&mpu_dev, SPI2_HOST);

    uint8_t whoami;
    float temp;
    vec3_t gyro_data;
    vec3_t acc_data;
    uint8_t i = 0;
    uint64_t tick;
    float tick_est = 0, sample_no = 0;

    mpu9250_read_whoami(&mpu_dev, &whoami);
    printf("WHOAMI: %d\n", whoami);

    uint16_t cntr = 0;
    while(1){
        // xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        
        timer_start(TIMER_GROUP_0, TIMER_0);
        mpu9250_read_temp(&mpu_dev, &temp);
        mpu9250_read_gyro(&mpu_dev, &gyro_data);
        mpu9250_read_acc(&mpu_dev, &acc_data);
        timer_pause(TIMER_GROUP_0, TIMER_0);
        timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &tick);
        sample_no++;
        tick_est = tick_est + 1/sample_no * (tick/(80000/2.0) - tick_est);
        printf("%f\t%f\n", tick_est, 1/tick_est);
        timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0);
        vTaskDelay(10/portTICK_PERIOD_MS);

        // mpu9250_read_gyro(&mpu_dev, &gyro_data);
        // mpu9250_read_acc(&mpu_dev, &acc_data);
        // data[cntr][0] = gyro_data.x;
        // data[cntr][1] = gyro_data.y;
        // data[cntr][2] = gyro_data.z;
        // data[cntr][3] = acc_data.x;
        // data[cntr][4] = acc_data.y;
        // data[cntr][5] = acc_data.z;
        // data[cntr][0] = temp;
        // cntr++;
        
        // if(cntr >= SAMPLE_SIZE){
        //     timer_pause(TIMER_GROUP_0, TIMER_0);
        //     for(i = 0; i < SAMPLE_SIZE; i++){    
        //         // printf("%f\t%f\t%f\t%f\t%f\t%f\n", 
        //         //     data[i][0],data[i][1],data[i][2],data[i][3],data[i][4],data[i][5]);
        //         printf("%f\n", data[i][0]);
        //         // Human readable print
        //         // printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\n", 
        //         //     temp, 
        //         //     gyro_data.x, gyro_data.y, gyro_data.z,
        //         //     acc_data.x, acc_data.y, acc_data.z
        //         // );
        //         // Binary print
        //         // memcpy(uart_buf, &gyro_data.x, 4);
        //         // memcpy(uart_buf+4, &gyro_data.y, 4);
        //         // memcpy(uart_buf+8, &gyro_data.z, 4);
        //         // for(i = 0; i<12; i++)
        //         //     printf("%c",uart_buf[i]);

        //         vTaskDelay(10 / portTICK_PERIOD_MS);
        //     }
        //     cntr = 0;
        //     timer_start(TIMER_GROUP_0, TIMER_0);
        // }
    }
}