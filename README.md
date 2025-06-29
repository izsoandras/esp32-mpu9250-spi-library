# MPU-9250 SPI library for ESP-IDF
## Next steps


- [ ] Clean up FIFO impelemntation
    - move slv3_fifo_en to the array
    - create enum for FIFO source indices with names
    - measurements are moved to the FIFO in increasing register order (acc, temp, gyro, slv0->3) -> fix in FIFO read implementation
- [ ] Separate different parts of the sensor to different source files and encapsulate parameterization in structs
  - Accelerometer
  - Gyroscope
  - I2C master
  - other?
- [ ] Magentometer
- [ ] Create reset procedure when I2C hangs (after esp reset or code upload)
- [ ] Check that DLPFs are working properly
  - Record movement -> FFT -> see that cutoff frequency is at given point
- [ ] Test FIFO with I2C slaves
     
## Test backlog

- I2C master delay \w scope

## Feature list

| Implemented | Feature | Remark |
| :-----------: | ------- | ------ |
| ✔️ | SPI communication | Always using polling transmit, ❗ read_n_bytes always uses buffer, but write switches between buffer and txdata |
| ✔️ | Reset sensor | |
| ✔️ | Read whoami | |
| ✔️ | Read temperature sensor | |
| ✔️ | Read gyroscope | |
| ✔️ | Read accelerometer | |
| ✔️ | Set gyroscope full scale | |
| ✔️ | Set accelerometer full scale | |
| ✔️ | Set gyroscope offset | |
| ✔️ | Set accelerometer offset | |
| ✔️ | Set gyroscope digital lowpass filter | |
| ✔️ | Set accelerometer digital lowpass filter | |
| ✔️ | Reset FIFO | |
| ✔️ | Set FIFO sources | |
| ✔️ | Read FIFO | ❗ Always uses memory buffer, instead of the fixed 4 byte registers |
| ⚠️ | I2C master handling | Default read/write always uses slave 4 |
| ✔️ | I2C slave control | |
| ✔️ | I2C sensor read | |
| | Magnetometer handling | |
| | Sample rate divider | |
| | Gyroscope self-test | |
| | Accelerometer self-test | |
| | Interrupt capabilities + I2C master bypass | |
| | Make use of slave 0-3 Data Out registers | |
| | Power management | |
| | Low power accelerometer ODR control | |
| | Wake-on motion | |
