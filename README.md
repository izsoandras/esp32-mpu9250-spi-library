# MPU-9250 SPI library for ESP-IDF
## Next steps

- [ ] Utilize I2C master to be able to read magnetometer
  - Test basic I2C functionality
  - I2C slave control
  - Add sensor reading functionality
- [ ] Separate different parts of the sensor to different source files and encapsulate parameterization in structs
  - Accelerometer
  - Gyroscope
  - Magnetometer
  - I2C master
  - other?
- [ ] Check that DLPFs are working properly
  - Record movement -> FFT -> see that cutoff frequency is at given point
     
## Test backlog

- I2C configuration register
- I2C write
- I2C master delay
- I2C status register

## Feature list

| Implemented | Feature | Remark |
| :-----------: | ------- | ------ |
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
| ⚠️ | I2C master handling | |
| | I2C slave control | |
| | I2C sensor read | |
| | Magnetometer handling | |
| | Sample rate divider | |
| | Gyroscope self-test | |
| | Accelerometer self-test | |
| | Interrupt capabilities + I2C master bypass | |
| | Power management | |
| | Low power accelerometer ODR control | |
| | Wake-on motion | |
