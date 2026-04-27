#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "mpu6050.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_event.h"


/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(MPU6050_t *self)
{
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;
    self->conf.mode = I2C_MODE_MASTER;
    self->conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;
    self->conf.sda_pullup_en = 1;
    self->conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;
    self->conf.scl_pullup_en = 1;
    self->conf.clk_stretch_tick = 300; // 300 ticks, Clock stretch is about 210us, you can make changes according to the actual situation.
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, self->conf.mode));
    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &self->conf));
    return ESP_OK;
}

/**
 * @brief test code to write mpu6050
 *
 * 1. send data
 * ___________________________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | write data_len byte + ack  | stop |
 * --------|---------------------------|-------------------------|----------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to send
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t mpu6050_i2c_write(MPU6050_t *self, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_write(cmd, data, data_len, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(self->i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

/**
 * @brief test code to read mpu6050
 *
 * 1. send reg address
 * ______________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write reg_address + ack | stop |
 * --------|---------------------------|-------------------------|------|
 *
 * 2. read data
 * ___________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | read data_len byte + ack(last nack)  | stop |
 * --------|---------------------------|--------------------------------------|------|
 *
 * @param i2c_num I2C port number
 * @param reg_address slave reg address
 * @param data data to read
 * @param data_len data length
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_ERR_INVALID_ARG Parameter error
 *     - ESP_FAIL Sending command error, slave doesn't ACK the transfer.
 *     - ESP_ERR_INVALID_STATE I2C driver not installed or not in master mode.
 *     - ESP_ERR_TIMEOUT Operation timeout because the bus is busy.
 */
esp_err_t mpu6050_i2c_read(MPU6050_t *self, uint8_t reg_address, uint8_t *data, size_t data_len)
{
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_address, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(self->i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MPU6050_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, data, data_len, LAST_NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(self->i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

esp_err_t mpu6050_i2c_init(MPU6050_t *self, i2c_port_t i2c_num)
{
    uint8_t cmd_data;
    vTaskDelay(100 / portTICK_RATE_MS);
    i2c_master_init(self);
    self->i2c_num = i2c_num;

    cmd_data = 0x00;    // reset mpu6050
    ESP_ERROR_CHECK(mpu6050_i2c_write(self, PWR_MGMT_1, &cmd_data, 1));
    cmd_data = 0x07;    // Set the SMPRT_DIV
    ESP_ERROR_CHECK(mpu6050_i2c_write(self, SMPLRT_DIV, &cmd_data, 1));
    cmd_data = 0x06;    // Set the Low Pass Filter
    ESP_ERROR_CHECK(mpu6050_i2c_write(self, CONFIG, &cmd_data, 1));
    cmd_data = 0x18;    // Set the GYRO range
    ESP_ERROR_CHECK(mpu6050_i2c_write(self, GYRO_CONFIG, &cmd_data, 1));
    cmd_data = 0x01;    // Set the ACCEL range
    ESP_ERROR_CHECK(mpu6050_i2c_write(self, ACCEL_CONFIG, &cmd_data, 1));
    return ESP_OK;
}