#ifndef A8F02D55_3261_42D1_9A5C_D7A372D01A48
#define A8F02D55_3261_42D1_9A5C_D7A372D01A48

#include "driver/i2c_master.h"
#include <stdint.h>
#include <stdbool.h>

#define WATCHDOG_ENABLE                        1                       // Will reset watchdog (esp-idf 5.3) when set

// Generalized
#define    REG_BANK_SEL                        0x7F

// Gyroscope and Accelerometer
// User Bank 0
#define    AGB0_REG_WHO_AM_I                   0x00
#define    AGB0_REG_USER_CTRL                  0x03
#define    AGB0_REG_LP_CONFIG                  0x05
#define    AGB0_REG_PWR_MGMT_1                 0x06
#define    AGB0_REG_PWR_MGMT_2                 0x07
#define    AGB0_REG_INT_PIN_CONFIG             0x0F
#define    AGB0_REG_INT_ENABLE                 0x10
#define    AGB0_REG_INT_ENABLE_1               0x11
#define    AGB0_REG_INT_ENABLE_2               0x12
#define    AGB0_REG_INT_ENABLE_3               0x13
#define    AGB0_REG_I2C_MST_STATUS             0x17
#define    AGB0_REG_INT_STATUS                 0x19
#define    AGB0_REG_INT_STATUS_1               0x1A
#define    AGB0_REG_INT_STATUS_2               0x1B
#define    AGB0_REG_INT_STATUS_3               0x1C
#define    AGB0_REG_DELAY_TIMEH                0x28
#define    AGB0_REG_DELAY_TIMEL                0x29
#define    AGB0_REG_ACCEL_XOUT_H               0x2D
#define    AGB0_REG_ACCEL_XOUT_L               0x2E
#define    AGB0_REG_ACCEL_YOUT_H               0x2F
#define    AGB0_REG_ACCEL_YOUT_L               0x30
#define    AGB0_REG_ACCEL_ZOUT_H               0x31
#define    AGB0_REG_ACCEL_ZOUT_L               0x32
#define    AGB0_REG_GYRO_XOUT_H                0x33
#define    AGB0_REG_GYRO_XOUT_L                0x34
#define    AGB0_REG_GYRO_YOUT_H                0x35
#define    AGB0_REG_GYRO_YOUT_L                0x36
#define    AGB0_REG_GYRO_ZOUT_H                0x37
#define    AGB0_REG_GYRO_ZOUT_L                0x38
#define    AGB0_REG_TEMP_OUT_H                 0x39
#define    AGB0_REG_TEMP_OUT_L                 0x3A
#define    AGB0_REG_EXT_SLV_SENS_DATA_00       0x3B
#define    AGB0_REG_EXT_SLV_SENS_DATA_01       0x3C
#define    AGB0_REG_EXT_SLV_SENS_DATA_02       0x3D
#define    AGB0_REG_EXT_SLV_SENS_DATA_03       0x3E
#define    AGB0_REG_EXT_SLV_SENS_DATA_04       0x3F
#define    AGB0_REG_EXT_SLV_SENS_DATA_05       0x40
#define    AGB0_REG_EXT_SLV_SENS_DATA_06       0x41
#define    AGB0_REG_EXT_SLV_SENS_DATA_07       0x42
#define    AGB0_REG_EXT_SLV_SENS_DATA_08       0x43
#define    AGB0_REG_EXT_SLV_SENS_DATA_09       0x44
#define    AGB0_REG_EXT_SLV_SENS_DATA_10       0x45
#define    AGB0_REG_EXT_SLV_SENS_DATA_11       0x46
#define    AGB0_REG_EXT_SLV_SENS_DATA_12       0x47
#define    AGB0_REG_EXT_SLV_SENS_DATA_13       0x48
#define    AGB0_REG_EXT_SLV_SENS_DATA_14       0x49
#define    AGB0_REG_EXT_SLV_SENS_DATA_15       0x4A
#define    AGB0_REG_EXT_SLV_SENS_DATA_16       0x4B
#define    AGB0_REG_EXT_SLV_SENS_DATA_17       0x4C
#define    AGB0_REG_EXT_SLV_SENS_DATA_18       0x4D
#define    AGB0_REG_EXT_SLV_SENS_DATA_19       0x4E
#define    AGB0_REG_EXT_SLV_SENS_DATA_20       0x4F
#define    AGB0_REG_EXT_SLV_SENS_DATA_21       0x50
#define    AGB0_REG_EXT_SLV_SENS_DATA_22       0x51
#define    AGB0_REG_EXT_SLV_SENS_DATA_23       0x52
#define    AGB0_REG_FIFO_EN_1                  0x66
#define    AGB0_REG_FIFO_EN_2                  0x67
#define    AGB0_REG_FIFO_MODE                  0x69
#define    AGB0_REG_FIFO_COUNT_H               0x70
#define    AGB0_REG_FIFO_COUNT_L               0x71
#define    AGB0_REG_FIFO_R_W                   0x72
#define    AGB0_REG_DATA_RDY_STATUS            0x74
#define    AGB0_REG_FIFO_CFG                   0x76
#define    AGB0_REG_MEM_START_ADDR             0x7C
#define    AGB0_REG_MEM_R_W                    0x7D 
#define    AGB0_REG_MEM_BANK_SEL               0x7E
#define    AGB0_REG_REG_BANK_SEL               0x7F
// Bank 1
#define    AGB1_REG_SELF_TEST_X_GYRO           0x02
#define    AGB1_REG_SELF_TEST_Y_GYRO           0x03
#define    AGB1_REG_SELF_TEST_Z_GYRO           0x04
#define    AGB1_REG_SELF_TEST_X_ACCEL          0x0E
#define    AGB1_REG_SELF_TEST_Y_ACCEL          0x0F
#define    AGB1_REG_SELF_TEST_Z_ACCEL          0x10
#define    AGB1_REG_XA_OFFS_H                  0x14
#define    AGB1_REG_XA_OFFS_L                  0x15
#define    AGB1_REG_YA_OFFS_H                  0x17
#define    AGB1_REG_YA_OFFS_L                  0x18
#define    AGB1_REG_ZA_OFFS_H                  0x1A
#define    AGB1_REG_ZA_OFFS_L                  0x1B
#define    AGB1_REG_TIMEBASE_CORRECTION_PLL    0x28
#define    AGB1_REG_REG_BANK_SEL               0x7F
// Bank 2
#define    AGB2_REG_GYRO_SMPLRT_DIV            0x00
#define    AGB2_REG_GYRO_CONFIG_1              0x01
#define    AGB2_REG_GYRO_CONFIG_2              0x02
#define    AGB2_REG_XG_OFFS_USRH               0x03
#define    AGB2_REG_XG_OFFS_USRL               0x04
#define    AGB2_REG_YG_OFFS_USRH               0x05
#define    AGB2_REG_YG_OFFS_USRL               0x06
#define    AGB2_REG_ZG_OFFS_USRH               0x07
#define    AGB2_REG_ZG_OFFS_USRL               0x08
#define    AGB2_REG_ODR_ALIGN_EN               0x09
#define    AGB2_REG_ACCEL_SMPLRT_DIV_1         0x10
#define    AGB2_REG_ACCEL_SMPLRT_DIV_2         0x11
#define    AGB2_REG_ACCEL_INTEL_CTRL           0x12
#define    AGB2_REG_ACCEL_WOM_THR              0x13
#define    AGB2_REG_ACCEL_CONFIG_1             0x14
#define    AGB2_REG_ACCEL_CONFIG_2             0x15
#define    AGB2_REG_FSYNC_CONFIG               0x52
#define    AGB2_REG_TEMP_CONFIG                0x53
#define    AGB2_REG_MOD_CTRL_USR               0x54
#define    AGB2_REG_REG_BANK_SEL               0x7F
// Bank 3
#define    AGB3_REG_I2C_MST_ODR_CONFIG         0x00
#define    AGB3_REG_I2C_MST_CTRL               0x01
#define    AGB3_REG_I2C_MST_DELAY_CTRL         0x02
#define    AGB3_REG_I2C_SLV0_ADDR              0x03
#define    AGB3_REG_I2C_SLV0_REG               0x04
#define    AGB3_REG_I2C_SLV0_CTRL              0x05
#define    AGB3_REG_I2C_SLV0_DO                0x06
#define    AGB3_REG_I2C_SLV1_ADDR              0x07
#define    AGB3_REG_I2C_SLV1_REG               0x08
#define    AGB3_REG_I2C_SLV1_CTRL              0x09
#define    AGB3_REG_I2C_SLV1_DO                0x0A
#define    AGB3_REG_I2C_SLV2_ADDR              0x0B
#define    AGB3_REG_I2C_SLV2_REG               0x0C
#define    AGB3_REG_I2C_SLV2_CTRL              0x0D
#define    AGB3_REG_I2C_SLV2_DO                0x0E
#define    AGB3_REG_I2C_SLV3_ADDR              0x0F
#define    AGB3_REG_I2C_SLV3_REG               0x10
#define    AGB3_REG_I2C_SLV3_CTRL              0x11
#define    AGB3_REG_I2C_SLV3_DO                0x12
#define    AGB3_REG_I2C_SLV4_ADDR              0x13
#define    AGB3_REG_I2C_SLV4_REG               0x14
#define    AGB3_REG_I2C_SLV4_CTRL              0x15
#define    AGB3_REG_I2C_SLV4_DO                0x16
#define    AGB3_REG_I2C_SLV4_DI                0x17
#define    AGB3_REG_REG_BANK_SEL               0x7F

// ICM20948 Accelerometer full scale range options
#define ICM20948_GPM2       0x00  // ±2g
#define ICM20948_GPM4       0x01  // ±4g
#define ICM20948_GPM8       0x02  // ±8g
#define ICM20948_GPM16      0x03  // ±16g

// ICM20948 Gyroscope full scale range options
#define ICM20948_DPS250     0x00  // ±250°/s
#define ICM20948_DPS500     0x01  // ±500°/s
#define ICM20948_DPS1000    0x02  // ±1000°/s
#define ICM20948_DPS2000    0x03  // ±2000°/s

// ICM20948 Accelerometer Digital Low-Pass Filter (DLPF) configs
#define ICM20948_ACC_D246BW_N265BW     0x00
#define ICM20948_ACC_D246BW_N265BW_1   0x01
#define ICM20948_ACC_D111BW4_N136BW    0x02
#define ICM20948_ACC_D50BW4_N68BW8     0x03
#define ICM20948_ACC_D23BW9_N34BW4     0x04
#define ICM20948_ACC_D11BW5_N17BW      0x05
#define ICM20948_ACC_D5BW7_N8BW3       0x06
#define ICM20948_ACC_D473BW_N499BW     0x07

// ICM20948 Gyroscope Digital Low-Pass Filter (DLPF) configs
#define ICM20948_GYR_D196BW6_N229BW8   0x00
#define ICM20948_GYR_D151BW8_N187BW6   0x01
#define ICM20948_GYR_D119BW5_N154BW3   0x02
#define ICM20948_GYR_D51BW2_N73BW3     0x03
#define ICM20948_GYR_D23BW9_N35BW9     0x04
#define ICM20948_GYR_D11BW6_N17BW8     0x05
#define ICM20948_GYR_D5BW7_N8BW9       0x06
#define ICM20948_GYR_D361BW4_N376BW5   0x07

// Magnetometer specific stuff
#define MAG_AK09916_I2C_ADDR     0x0C
#define MAG_AK09916_WHO_AM_I     0x4809
#define MAG_REG_WHO_AM_I         0x00

// Magnetometer operation modes
#define AK09916_MODE_POWER_DOWN      0x00
#define AK09916_MODE_SINGLE          (0x01 << 0)
#define AK09916_MODE_CONT_10HZ       (0x01 << 1)
#define AK09916_MODE_CONT_20HZ       (0x02 << 1)
#define AK09916_MODE_CONT_50HZ       (0x03 << 1)
#define AK09916_MODE_CONT_100HZ      (0x04 << 1)
#define AK09916_MODE_SELF_TEST       (0x01 << 4)

// Magnetometer register addresses
#define AK09916_REG_WIA1     0x00
#define AK09916_REG_WIA2     0x01
#define AK09916_REG_ST1      0x10
#define AK09916_REG_HXL      0x11
#define AK09916_REG_HXH      0x12
#define AK09916_REG_HYL      0x13
#define AK09916_REG_HYH      0x14
#define AK09916_REG_HZL      0x15
#define AK09916_REG_HZH      0x16
#define AK09916_REG_ST2      0x18
#define AK09916_REG_CNTL2    0x31
#define AK09916_REG_CNTL3    0x32

typedef struct {
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
    float temp;
    uint8_t mag_stat1;
    uint8_t mag_stat2;
} icm20948_sensordata_t;

bool icm20948_init(i2c_master_bus_handle_t bus);
bool icm20948_read_raw(icm20948_sensordata_t *pData);
void icm20948_scan_bus(i2c_master_bus_handle_t bus);

#endif /* A8F02D55_3261_42D1_9A5C_D7A372D01A48 */
