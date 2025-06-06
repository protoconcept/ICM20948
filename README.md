# ICM20948
**ICM20948 driver for the ESP-IDF platform (ESP32)**  
Developed by Kevin Assen - [Protoconcept](https://protoconcept.nl)  

## Overview

This repository contains an open-source driver for the **ICM-20948 9-axis IMU** sensor, including full support for the onboard **AK09916 magnetometer**.  
The driver is designed to run on **ESP32** platforms using the **ESP-IDF framework (version 5.3 or higher)**.

It provides access to:

- 3-axis accelerometer (±2/4/8/16g)
- 3-axis gyroscope (±250/500/1000/2000 dps)
- 3-axis magnetometer (via I2C master interface to AK09916)
- Onboard temperature sensor

This driver is based on the [SparkFun Qwiic ICM20948 Python Driver](https://github.com/sparkfun/qwiic_9dof_imu_icm20948_py).

---

## Features

- ✅ I²C communication with register bank switching
- ✅ Internal I²C master support for AK09916 magnetometer
- ✅ Raw sensor data reading for accel, gyro, magnetometer, and temperature
- ✅ Bit-level register access helpers
- ✅ I2C slave auto-poll configuration

---

## Requirements

- ESP32 development board  
- ESP-IDF version **5.3** or higher  
- ICM-20948 breakout module

Tested with:  
✔️ ESP32-WROOM  
✔️ ESP-IDF v5.3  
✔️ 400kHz I²C bus

---

## Usage

1. **Include the driver in your ESP-IDF project**
```c
#include "icm20948.h"
```

2. **Initialize the device**
```c
i2c_master_bus_handle_t i2c_bus = /* ... */;
icm20948_init(i2c_bus);
```

3. **Read sensor data**
```c
icm20948_sensordata_t data;
if (icm20948_read_raw(&data)) {
    printf("Accel X: %d, Gyro Z: %d, Temp: %.2f C\n", data.acc_x, data.gyr_z, data.temp);
}
```
---

## Author

**Kevin Assen**  
[Protoconcept](https://protoconcept.nl)  
info@protoconcept.nl

---

## Notes

- Make sure to calibrate magnetometer values for heading accuracy.  
- If using a different I2C address or SDA/SCL pins, adjust `icm20948_init()` accordingly.
