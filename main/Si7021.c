/*
 *
 * Written by Olof Astrand <olof.astrand@gmail.com>
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 *
*/

/** 
    @brief esp-idf code to read temperature and pressure from Si7021 device
    You must set up inc devices pins e.t.c before using this

*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "Si7021.h"

#define WRITE_BIT  I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT   I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */

#define  SI7021_ADDRESS    0x40
#define I2C_MASTER_NUM   I2C_NUM_0   /*!< I2C port number for master dev */


/**
 *    https://cdn-learn.adafruit.com/assets/assets/000/035/931/original/Support_Documents_TechnicalDocs_Si7021-A20.pdf
 * @brief  read relative humidity from Si7021 device
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  ... read 1 byte + nack | stop |
 * --------|---------------------------|---------------------|--------------------|------|
 */
esp_err_t i2c_7021_read_raw_rh(i2c_port_t i2c_num, uint8_t* data_h,uint8_t* data_l)
{
    uint8_t checksum;
    int times_tried=0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SI7021_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd,  0xE5, ACK_CHECK_EN);
    i2c_master_write_byte(cmd,  0xF5, ACK_CHECK_EN);

    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ret;
    }
    vTaskDelay(200 / portTICK_RATE_MS);

    // Device will NAK until measurement is complete
    ret = ESP_FAIL;
    while ((ret!=ESP_OK) && (times_tried++<10) ) 
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, SI7021_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, data_h, ACK_VAL);
        i2c_master_read_byte(cmd, data_l, ACK_VAL);
        i2c_master_read_byte(cmd, &checksum, NACK_VAL);

        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
    }
    if (ret == ESP_FAIL) {
        return ESP_FAIL;
    }
    return ESP_OK;
}


/**
 *    https://cdn-learn.adafruit.com/assets/assets/000/035/931/original/Support_Documents_TechnicalDocs_Si7021-A20.pdf
 * @brief  read raw temp data from Si7021 device
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  ... read 1 byte + nack | stop |
 * --------|---------------------------|---------------------|--------------------|------|
 */
esp_err_t i2c_7021_read_raw_temp(i2c_port_t i2c_num, uint8_t* data_h,uint8_t* data_l)
{
    uint8_t checksum;
    int times_tried=0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SI7021_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd,  0xE3, ACK_CHECK_EN);
    i2c_master_write_byte(cmd,  0xF3, ACK_CHECK_EN);

    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ret;
    }
    vTaskDelay(200 / portTICK_RATE_MS);

    // Device will NAK until measurement is complete
    ret = ESP_FAIL;
    while ((ret!=ESP_OK) && (times_tried++<10) ) 
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, SI7021_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, data_h, ACK_VAL);
        i2c_master_read_byte(cmd, data_l, ACK_VAL);
        i2c_master_read_byte(cmd, &checksum, NACK_VAL);

        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
    }
    if (ret == ESP_FAIL) {
        return ESP_FAIL;
    }
    return ESP_OK;
}


/**
 * @brief  read relative humidity from Si7021 device
*/
float i2c_7021_read_rh() {
    uint8_t dataH;
    uint8_t dataL;

   esp_err_t result=i2c_7021_read_raw_rh(I2C_MASTER_NUM, &dataH,&dataL);
   if (result!=ESP_OK) {
       printf("Failed reding relateive humidity\n");
   }

   float val=dataH*256+dataL;

   float ret=(125.0*(val)/65536.0)-6.0;

   return(ret);
}

/**
 * @brief  read temp from Si7021 device
*/
float i2c_7021_read_temp() {
    uint8_t dataH;
    uint8_t dataL;

   esp_err_t result=i2c_7021_read_raw_temp(I2C_MASTER_NUM, &dataH,&dataL);
   if (result!=ESP_OK) {
       printf("Failed reading temp\n");
   }

   float val=dataH*256+dataL;
   float ret=(175.72*(val)/65536.0)-46.85;

   return(ret);
}
#if 0

#define  SI7021_ADDRESS    0x40
#define  SN_CMD_START      0xFA
#define  SN_CMD2           0x0F


/**
 *    https://cdn-learn.adafruit.com/assets/assets/000/035/931/original/Support_Documents_TechnicalDocs_Si7021-A20.pdf
 * @brief test code to read serial number from Si7021 device
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  ... read 1 byte + nack | stop |
 * --------|---------------------------|---------------------|--------------------|------|
 */
esp_err_t i2c_sn7021_test(i2c_port_t i2c_num, uint8_t* sna_3,uint8_t* sna_2,uint8_t* sna_1,uint8_t* sna_0)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SI7021_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, SN_CMD_START, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, SN_CMD2, ACK_CHECK_EN);

    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SI7021_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, sna_3, ACK_VAL);
    i2c_master_read_byte(cmd, sna_2, ACK_VAL);
    i2c_master_read_byte(cmd, sna_1, ACK_VAL);
    i2c_master_read_byte(cmd, sna_0, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t i2c_sn7021_test2(i2c_port_t i2c_num, uint8_t* sna_3,uint8_t* sna_2,uint8_t* sna_1,uint8_t* sna_0)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SI7021_ADDRESS << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xFC, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0xC9, ACK_CHECK_EN);

    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, SI7021_ADDRESS << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, sna_3, ACK_VAL);
    i2c_master_read_byte(cmd, sna_2, ACK_VAL);
    i2c_master_read_byte(cmd, sna_1, ACK_VAL);
    i2c_master_read_byte(cmd, sna_0, NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_FAIL) {
        return ESP_FAIL;
    }
    return ESP_OK;
}


    ret = i2c_sn7021_test( I2C_MASTER_NUM, &sna3, &sna2,&sna1,&sna0);

    if (ret == ESP_OK) {
        printf("s/n: %02x %02x %02x %02x\n", sna3,sna2,sna1,sna0);
    } else {
        printf("No ack, sensor not connected...skip...\n");
    }
    ret = i2c_sn7021_test2( I2C_MASTER_NUM, &sna3, &sna2,&sna1,&sna0);
    if (ret == ESP_OK) {
        printf("s/nb: %02x %02x %02x %02x\n", sna3,sna2,sna1,sna0);
    } else {
        printf("No ack, sensor not connected...skip...\n");
    }
    if (sna3==0x0D) {
        printf("Si7013\n");
    }
    if (sna3==0x14) {
        printf("Si7020\n");
    }
    if (sna3==0x15) {
        printf("Si7021\n");
    }


#endif