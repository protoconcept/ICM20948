/**
 * @file icm20948.c
 * @author Kevin Assen (kevin@protoconcept.nl)
 * @brief Driver for the ICM-20948
 * @version 0.1
 * @date 13-05-2025
 * 
 * @company Protoconcept
 * 
 * @license MIT
 * 
 * @details
 * This open-source driver is based on the SparkFun Qwiic ICM20948 Python driver:
 * https://github.com/sparkfun/qwiic_9dof_imu_icm20948_py
 *  
 * 
 * @copyright Copyright (c) 2025 Kevin Assen, Protoconcept
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "icm20948.h"

#include <string.h>
#include "esp_log.h"
#include "driver/i2c_master.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_timer.h"
#if WATCHDOG_ENABLE == 1
#include "esp_task_wdt.h"
#endif

#define TAG "ICM20948"

#define I2C_FREQ_HZ     400000

static i2c_master_dev_handle_t icm_handle = NULL;

static uint8_t icm20948_read_cmd(uint8_t bank, uint8_t reg);
static void icm20948_send_cmd(uint8_t bank, uint8_t reg, uint8_t val);

/**
 * @brief Writes a value to a register
 * 
 * @param reg 
 * @param val 
 */
static void icm20948_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    i2c_master_transmit(icm_handle, buf, 2, 100);
}

/**
 * @brief Sets the current bank for register
 * 
 * @param bank 
 */
static void icm20948_write_bank(uint8_t bank)
{
    bank = ((bank << 4) & 0x30); // bits 5:4 of REG_BANK_SEL
    icm20948_write_reg(REG_BANK_SEL, bank);
}

/**
 * @brief Sets/resets a bit of a register
 * 
 * @param bank 0-3
 * @param reg register
 * @param bit bit to set
 * @param state true = set / false = reset
 */
static void icm20948_write_bit(uint8_t bank, uint8_t reg, uint8_t bit, bool state)
{
    uint8_t val = icm20948_read_cmd(bank, reg);
    if (state) {
        val |= (1 << bit);
    } else {
        val &= ~(1 << bit);
    }
    icm20948_send_cmd(bank, reg, val);
}

/**
 * @brief Reads and returns a register
 * 
 * @param reg 
 * @return uint8_t 
 */
static uint8_t icm20948_read_reg(uint8_t reg)
{
    uint8_t val = 0;
    if (i2c_master_transmit_receive(icm_handle, &reg, 1, &val, 1, 100) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read register");
    }
    
    return val;
}

/**
 * @brief Reads a bit of a register
 * 
 * @param bank 0-3
 * @param reg register
 * @param bit bit to read
 * @return true = set
 * @return false = reset
 */
static bool icm20948_read_bit(uint8_t bank, uint8_t reg, uint8_t bit)
{
    uint8_t val = icm20948_read_cmd(bank, reg);
    return (val >> bit) & 0x01;
}

/**
 * @brief Reads a block of data
 * 
 * @param bank 0-3
 * @param start_reg base address to read 
 * @param buffer buffer to write data to
 * @param length length of data block to read in bytes
 * @return true 
 * @return false 
 */
static bool icm20948_read_bytes(uint8_t bank, uint8_t start_reg, uint8_t* buffer, size_t length)
{
    icm20948_write_bank(bank);

    // Send register address, then read `length` bytes into buffer
    if (i2c_master_transmit_receive(icm_handle, &start_reg, 1, buffer, length, 100) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read %u bytes from reg 0x%02X", length, start_reg);
        return false;
    }

    return true;
}

/**
 * @brief Reads a command (Wrapper function so you don't have to set bank first)
 * 
 * @param bank 0-3
 * @param reg register
 * @return uint8_t 
 */
static uint8_t icm20948_read_cmd(uint8_t bank, uint8_t reg)
{
    icm20948_write_bank(bank);
    return icm20948_read_reg(reg);
}

/**
 * @brief Sends a command (Wrapper function so you don't have to set bank first)
 * 
 * @param bank 0-3
 * @param reg register
 * @param val value
 */
static void icm20948_send_cmd(uint8_t bank, uint8_t reg, uint8_t val)
{
    icm20948_write_bank(bank);
    icm20948_write_reg(reg, val);
}

/**
 * @brief Resets icm20948
 * 
 */
static void icm20948_reset()
{
    icm20948_write_bit(0, AGB0_REG_PWR_MGMT_1, 7, true);
}

/**
 * @brief Enables/disables sleep mode
 * 
 * @param state 
 */
static void icm20948_sleep(bool state)
{
    icm20948_write_bit(0, AGB0_REG_PWR_MGMT_1, 6, state);
}

/**
 * @brief Enables/disables low power mode
 * 
 * @param state 
 */
static void icm20948_low_power(bool state)
{
    icm20948_write_bit(0, AGB0_REG_PWR_MGMT_1, 5, state);
}

/**
 * @brief Writes sample mode of sensors
 * 
 * @param internal_acc 
 * @param internal_gyr 
 * @param internal_mst 
 * @param mode_cycled true = cycled. false = continuous
 */
static void icm20948_sample_mode(bool internal_acc, bool internal_gyr, bool internal_mst, bool mode_cycled)
{
    if (internal_acc)
    {
        icm20948_write_bit(0, AGB0_REG_LP_CONFIG, 5, mode_cycled);
    }
    
    if (internal_gyr)
    {
        icm20948_write_bit(0, AGB0_REG_LP_CONFIG, 4, mode_cycled);
    }

    if (internal_mst)
    {
        icm20948_write_bit(0, AGB0_REG_LP_CONFIG, 6, mode_cycled);
    }
}

/**
 * @brief Sets full scale range of accel
 * 
 * @param mode 
 */
static void icm20948_full_scale_range_acc(uint8_t mode)
{
    uint8_t acc_cfg_reg = icm20948_read_cmd(2, AGB2_REG_ACCEL_CONFIG_1);

    acc_cfg_reg &= ~(0b00000110);   // Clear bits 2:1 (0b0000.0XX0)
    acc_cfg_reg |= (mode << 1);     // place mode select into bits 2:1 of AGB2_REG_ACCEL_CONFIG	

    icm20948_send_cmd(2, AGB2_REG_ACCEL_CONFIG_1, acc_cfg_reg);
}

/**
 * @brief Sets full scale range of gyro
 * 
 * @param mode 
 */
static void icm20948_full_scale_range_gyro(uint8_t mode)
{
    uint8_t gyro_cfg_reg = icm20948_read_cmd(2, AGB2_REG_GYRO_CONFIG_1);

    gyro_cfg_reg &= ~(0b00000110);   // Clear bits 2:1 (0b0000.0XX0)
    gyro_cfg_reg |= (mode << 1);     // place mode select into bits 2:1 of AGB2_REG_GYRO_CONFIG_1	

    icm20948_send_cmd(2, AGB2_REG_GYRO_CONFIG_1, gyro_cfg_reg);
}

/**
 * @brief Sets dlpf config of accelerometer
 * 
 * @param dlpcfg 
 */
static void icm20948_dlpf_acc(uint8_t dlpcfg)
{
    uint8_t acc_cfg_reg = icm20948_read_cmd(2, AGB2_REG_ACCEL_CONFIG_1);

    acc_cfg_reg &= ~(0b00111000);       // clear bits 5:3 (0b00XX.X000)
    acc_cfg_reg |= (dlpcfg << 3);       // place dlpcfg select into bits 5:3 of AGB2_REG_ACCEL_CONFIG_1		

    icm20948_send_cmd(2, AGB2_REG_ACCEL_CONFIG_1, acc_cfg_reg);
}

/**
 * @brief Sets dlpf config of gyro
 * 
 * @param dlpcfg 
 */
static void icm20948_dlpf_gyro(uint8_t dlpcfg)
{
    uint8_t gyro_cfg_reg = icm20948_read_cmd(2, AGB2_REG_GYRO_CONFIG_1);

    gyro_cfg_reg &= ~(0b00111000);       // clear bits 5:3 (0b00XX.X000)
    gyro_cfg_reg |= (dlpcfg << 3);       // place dlpcfg select into bits 5:3 of AGB2_REG_GYRO_CONFIG_1		

    icm20948_send_cmd(2, AGB2_REG_GYRO_CONFIG_1, gyro_cfg_reg);
}

/**
 * @brief Enables DLPF of accelerometer
 * 
 * @param state 
 */
static void icm20948_dlpf_acc_enable(bool state)
{
    icm20948_write_bit(2, AGB2_REG_ACCEL_CONFIG_1, 0, state);
}

/**
 * @brief Enables DLPF of gyro
 * 
 * @param state 
 */
static void icm20948_dlpf_gyro_enable(bool state)
{
    icm20948_write_bit(2, AGB2_REG_GYRO_CONFIG_1, 0, state);
}

/**
 * @brief Enables/disables I2C master passthrough
 * 
 * @param state 
 */
static void icm20948_master_passthrough(bool state)
{
    icm20948_write_bit(0, AGB0_REG_INT_PIN_CONFIG, 1, state);
}

/**
 * @brief Enables/disables I2C master
 * 
 * @param state 
 */
static void icm20948_master_enable(bool state)
{
    icm20948_master_passthrough(false);

    uint8_t mst_reg = icm20948_read_cmd(3, AGB3_REG_I2C_MST_CTRL);

    mst_reg &= ~(0x0F);     // clear bits for master clock [3:0]
    mst_reg |= (0x07);  	// set bits for master clock [3:0], 0x07 corresponds to 345.6 kHz, good for up to 400 kHz
    mst_reg |= (1<<4);      // set bit [4] for NSR (next slave read). 0 = restart between reads. 1 = stop between reads.

    icm20948_send_cmd(3, AGB3_REG_I2C_MST_CTRL, mst_reg);

    // Set/clear the I2C_MST_EN bit [5] as needed
    icm20948_write_bit(0, AGB0_REG_USER_CTRL, 5, state);
}

/**
 * @brief Transact between ICM20948 and I2C slave 4
 * 
 * @param addr Slave address
 * @param reg register
 * @param data data to send. 0 when reading
 * @param send_reg_addr 
 * @param rw 
 * @return uint8_t 
 */
static uint8_t icm20948_master_slv4_txn(uint8_t addr, uint8_t reg, uint8_t data, bool send_reg_addr, bool rw)
{
    uint8_t ctlr_reg_slv4 = 0x00;
    uint8_t result = 0x00;
    if (rw)
    {
        addr |= 0x80;
    }

    icm20948_send_cmd(3, AGB3_REG_I2C_SLV4_ADDR, addr);
    icm20948_send_cmd(3, AGB3_REG_I2C_SLV4_REG, reg);

    ctlr_reg_slv4 |= (1<<7);         // EN bit [7] (set)
    ctlr_reg_slv4 &= ~(1<<6);        // INT_EN bit [6] (cleared)
    ctlr_reg_slv4 &= ~(0x0F);        // DLY bits [4:0] (cleared = 0)

    if (send_reg_addr)
    {
        ctlr_reg_slv4 &= ~(1<<5);    // REG_DIS bit [5] (cleared)
    }
    else
    {
        ctlr_reg_slv4 |= (1<<5);     // REG_DIS bit [5] (set)
    }

    if (!rw)
    {
        icm20948_send_cmd(3, AGB3_REG_I2C_SLV4_DO, data);
    }

    icm20948_send_cmd(3, AGB3_REG_I2C_SLV4_CTRL, ctlr_reg_slv4);

    int64_t start_time = esp_timer_get_time();  // microseconds

    while(true)
    {
        if (icm20948_read_bit(0, AGB0_REG_I2C_MST_STATUS, 6)) {
            break;
        }

        if ((esp_timer_get_time() - start_time) > 1 * 1000 * 1000) {
            ESP_LOGD(TAG, "Timeout waiting for ready");
            break;
        }
#if WATCHDOG_ENABLE == 1
        esp_task_wdt_reset();
#endif

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    if (icm20948_read_bit(0, AGB0_REG_I2C_MST_STATUS, 4))
    {
        ESP_LOGW(TAG, "Failed to send data to slave4");
    }

    if (rw)
    {
        result = icm20948_read_cmd(3, AGB3_REG_I2C_SLV4_DI);
    }

    return result;
}

/**
 * @brief Sends data to slave 4
 * 
 * @param addr 
 * @param reg 
 * @param data 
 */
static void icm20948_master_slv4_single_write(uint8_t addr, uint8_t reg, uint8_t data)
{
    icm20948_master_slv4_txn(addr, reg, data, true, false);
}

/**
 * @brief Sends data to AK09916
 * 
 * @param reg 
 * @param data 
 */
static void icm20948_write_mag(uint8_t reg, uint8_t data)
{
    icm20948_master_slv4_single_write(MAG_AK09916_I2C_ADDR, reg, data);
}

/**
 * @brief Reads data from slave 4
 * 
 * @param addr 
 * @param reg 
 * @return uint8_t 
 */
static uint8_t icm20948_master_slv4_single_read(uint8_t addr, uint8_t reg)
{
    return icm20948_master_slv4_txn(addr, reg, 0, true, true);
}

/**
 * @brief Reads data from AK09916
 * 
 * @param reg 
 * @return uint8_t 
 */
static uint8_t icm20948_read_mag(uint8_t reg)
{
    return icm20948_master_slv4_single_read(MAG_AK09916_I2C_ADDR, reg);
}

/**
 * @brief Resets I2C master module
 * 
 */
static void icm20948_i2c_master_reset()
{
    icm20948_write_bit(0, AGB0_REG_USER_CTRL, 1, true);
}

/**
 * @brief Configures I2C master module
 * 
 * @param slave 0-3
 * @param addr 
 * @param reg 
 * @param len 
 * @param rw 
 * @param enable 
 * @param data_only 
 * @param grp 
 * @param swap 
 */
static void icm20948_i2c_master_configure(uint8_t slave, uint8_t addr, uint8_t reg, uint8_t len, bool rw, bool enable, bool data_only, bool grp, bool swap)
{
    uint8_t slave_addr_reg = 0x00;
    uint8_t slave_reg_reg = 0x00;
    uint8_t slave_ctrl_reg = 0x00;
    uint8_t ctrl_reg_slvX = 0x00;

    switch (slave)
    {
        case 0:
            slave_addr_reg = AGB3_REG_I2C_SLV0_ADDR;
			slave_reg_reg = AGB3_REG_I2C_SLV0_REG;
			slave_ctrl_reg = AGB3_REG_I2C_SLV0_CTRL;
            break;
        case 1:
            slave_addr_reg = AGB3_REG_I2C_SLV1_ADDR;
			slave_reg_reg = AGB3_REG_I2C_SLV1_REG;
			slave_ctrl_reg = AGB3_REG_I2C_SLV1_CTRL;
            break;
        case 2:
            slave_addr_reg = AGB3_REG_I2C_SLV2_ADDR;
			slave_reg_reg = AGB3_REG_I2C_SLV2_REG;
			slave_ctrl_reg = AGB3_REG_I2C_SLV2_CTRL;
            break;
        case 3:
            slave_addr_reg = AGB3_REG_I2C_SLV3_ADDR;
			slave_reg_reg = AGB3_REG_I2C_SLV3_REG;
			slave_ctrl_reg = AGB3_REG_I2C_SLV3_CTRL;
            break;
        default:
            ESP_LOGW(TAG, "Unknown slave id %d", slave);
        break;
    }

    if (rw)
    {
        addr |= (1<<7);  // Set bit# set RNW bit [7]
    }

    icm20948_send_cmd(3, slave_addr_reg, addr);
    icm20948_send_cmd(3, slave_reg_reg, reg);

    //Set up the control info
    ctrl_reg_slvX = 0x00;
    ctrl_reg_slvX |= len;
    ctrl_reg_slvX |= (enable << 7);
    ctrl_reg_slvX |= (swap << 6);
    ctrl_reg_slvX |= (data_only << 5);
    ctrl_reg_slvX |= (grp << 4);

    icm20948_send_cmd(3, slave_ctrl_reg, ctrl_reg_slvX);
}

/**
 * @brief Returns true when expected WHO_AM_I value returned for AK09916
 * 
 * @return true 
 * @return false 
 */
static bool icm20948_mag_who_am_I()
{
    uint8_t whoamI1 = icm20948_read_mag(AK09916_REG_WIA1);
    uint8_t whoamI2 = icm20948_read_mag(AK09916_REG_WIA2);

    if ((whoamI1 == (MAG_AK09916_WHO_AM_I >> 8)) && (whoamI2 == (MAG_AK09916_WHO_AM_I & 0xFF)))
    {
        return true;
    }
    ESP_LOGE(TAG, "Who am I failed. Received IDs: %d %d ", whoamI1, whoamI2);
    return false;
}

/**
 * @brief True when data ready
 * 
 * @return true 
 * @return false 
 */
static bool icm20948_data_is_ready()
{
    return icm20948_read_bit(0, AGB0_REG_INT_STATUS_1, 0);
}

/**
 * @brief Startup routine for magnetometer
 * 
 */
static void icm20948_magnetometer_startup()
{
    uint8_t mag_reg_ctrl2 = 0x00;
    ESP_LOGI(TAG, "Starting magnetometer...");

    icm20948_master_passthrough(false);
    icm20948_master_enable(true);

    // Retry WHO_AM_I
    int tries = 0, maxTries = 5;
    while (tries < maxTries)
    {
        if (icm20948_mag_who_am_I()) {
            ESP_LOGI(TAG, "AK09916 magnetometer found.");
            break;
        }

        ESP_LOGW(TAG, "AK09916 WHO_AM_I failed. Retrying...");
        icm20948_i2c_master_reset();
        tries++;
    }

    if (tries == maxTries) {
        ESP_LOGE(TAG, "Magnetometer init failed: WHO_AM_I mismatch");
        return;
    }

    //Set up magnetometer
    mag_reg_ctrl2 |= AK09916_MODE_CONT_100HZ;
    icm20948_write_mag(AK09916_REG_CNTL2, mag_reg_ctrl2);

    ESP_LOGI(TAG, "Magnetometer configured for continuous read.");
    icm20948_i2c_master_configure(0, MAG_AK09916_I2C_ADDR, AK09916_REG_ST1, 9, true, true, false, false, false);
}

/**
 * @brief Scans bus for I2C devices. 
 * 
 * @param bus 
 */
void icm20948_scan_bus(i2c_master_bus_handle_t bus)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("i2c.master", ESP_LOG_NONE);  // Disable errors to ignore nack spam temporarily
    ESP_LOGI(TAG, "Scanning full I2C address space...");

    for (uint8_t addr = 0x03; addr <= 0x77; addr++) {
        i2c_device_config_t dev_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = addr,
            .scl_speed_hz = I2C_FREQ_HZ
        };

        i2c_master_dev_handle_t temp_handle;
        if (i2c_master_bus_add_device(bus, &dev_cfg, &temp_handle) == ESP_OK) {
            uint8_t reg = 0x00, val = 0;
            if (i2c_master_transmit_receive(temp_handle, &reg, 1, &val, 1, 100) == ESP_OK) {
                ESP_LOGI(TAG, "Device responded at 0x%02X (reg 0x00 = 0x%02X)", addr, val);
            }
            i2c_master_bus_rm_device(temp_handle);
        }
#if WATCHDOG_ENABLE == 1
        esp_task_wdt_reset();
#endif
    }

    ESP_LOGI(TAG, "I2C scan complete.");
    esp_log_level_set("i2c.master", ESP_LOG_ERROR);
}

/**
 * @brief Reads raw data of ICM20948
 * 
 * @param pData 
 * @return true 
 * @return false 
 */
bool icm20948_read_raw(icm20948_sensordata_t *pData)
{
    uint8_t buff[23];

    if (!icm20948_data_is_ready())
    {
        ESP_LOGW(TAG, "Magnetometer data not ready");
        return false;
    }

    icm20948_read_bytes(0, AGB0_REG_ACCEL_XOUT_H, buff, sizeof(buff));

    pData->acc_x = (int16_t)((buff[0] << 8) | buff[1]);
    pData->acc_y = (int16_t)((buff[2] << 8) | buff[3]);
    pData->acc_z = (int16_t)((buff[4] << 8) | buff[5]);

    pData->gyr_x = (int16_t)((buff[6] << 8) | buff[7]);
    pData->gyr_y = (int16_t)((buff[8] << 8) | buff[9]);
    pData->gyr_z = (int16_t)((buff[10] << 8) | buff[11]);

    int16_t raw_temp = (int16_t)((buff[12] << 8) | buff[13]);
    pData->temp = ((float)raw_temp) / 333.87f + 21.0f;

    pData->mag_stat1 = buff[14];

    pData->mag_x = (int16_t)((buff[16] << 8) | buff[15]);
    pData->mag_y = (int16_t)((buff[18] << 8) | buff[17]);
    pData->mag_z = (int16_t)((buff[20] << 8) | buff[19]);

    pData->mag_stat2 = buff[21];

    return true;
}

/**
 * @brief ICM20948 setup
 * 
 * @param bus 
 * @return true 
 * @return false 
 */
bool icm20948_init(i2c_master_bus_handle_t bus)
{
    esp_log_level_set(TAG, ESP_LOG_INFO);
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x68,
        .scl_speed_hz = 400000
    };

    if (i2c_master_bus_add_device(bus, &dev_cfg, &icm_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add ICM20948");
        return false;
    }

    uint8_t whoami = icm20948_read_cmd(0, AGB0_REG_WHO_AM_I);
    if (whoami != 0xEA) {
        ESP_LOGE(TAG, "Wrong WHO_AM_I: 0x%02X", whoami);
        return false;
    }

    ESP_LOGI(TAG, "ICM20948 found. Starting init sequence");

    icm20948_reset();
    vTaskDelay(pdMS_TO_TICKS(50));

    // Disable sleep mode
    icm20948_sleep(false);
    
    // Disable low power
    icm20948_low_power(false);

    // Set sample mode to continuous for both accel and gyro
    icm20948_sample_mode(true, true, false, false);

    // Set full scale range for both accel and gyro
    icm20948_full_scale_range_acc(ICM20948_GPM2);
    icm20948_full_scale_range_gyro(ICM20948_GPM2);

    // Set low pass filter for both accel and gyro
    icm20948_dlpf_acc(ICM20948_ACC_D473BW_N499BW);
    icm20948_dlpf_gyro(ICM20948_GYR_D361BW4_N376BW5);

    // Disable digital low pass filters on both accel and gyro
    icm20948_dlpf_acc_enable(false);
    icm20948_dlpf_gyro_enable(false);

    icm20948_magnetometer_startup();

    return true;
}

