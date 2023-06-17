/*
 *
 *  This program provides a simple polling mode driver and example
 *  for the ADXL345 Accelerometer from Analog Devices.
 * 
 *  It is designed for ESP-IDF and tailored to the ESP32S2 on the FSPI
 *  interface (if you don't modify the .h file)
 *
 *  It is adapted from the original no OS driver from Analog Devices
 *  (license below).
 * 
 *  It is very basic and stripped down in functionality.
 * 
 *  Some things to improve:
 *   -- Get rid of need for global adxl handle
 *   -- Develop a device structure to hold the handle, and the device settings for easy reference/checking
 *   -- De-initialize to unallocate memory, etc. if used above.
 *   -- Add interrupt support
 *   -- Add feature support (activity, tap, inactivity, freefall, etc.)
 * 
 *  Mark Roberts (mdroberts1243@gmail.com)
 *  Modifications are Copyright (c) 2023 Mark Roberts
 */

/***************************************************************************//**
 *   @file   adxl345.c
 *   @brief  Implementation of ADXL345 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#include "sdkconfig.h"
#include "esp_log.h"

#include "adxl345.h"

static const char TAG[] = "adxl345";

extern spi_device_handle_t adxl;

/***************************************************************************//**
 * @brief Reads the value of a register.
 *
 * @param handle           - The SPI device handle.
 * @param register_address - Address of the register.
 *
 * @return register_value  - Value of the register.
*******************************************************************************/
uint8_t adxl345_get_register_value(spi_device_handle_t handle, uint8_t register_address)
{
	uint8_t data_buffer[2] = {0, 0};
	uint8_t register_value = 0;

    data_buffer[0] = ADXL345_SPI_READ | register_address;
	data_buffer[1] = 0;
    spi_transaction_t t = {
            .flags = SPI_TRANS_USE_RXDATA,
            .cmd = data_buffer[0]<<8 | data_buffer[1],
            .rxlength = 8,
    };
    esp_err_t ret = spi_device_polling_transmit(handle,&t);
    if  (ret != ESP_OK) {
        ESP_LOGE(TAG, "Got an error reading a register: %d, err: %d", register_address, ret);
    }
    register_value = t.rx_data[0];

    return register_value;
}

/***************************************************************************//**
 * @brief Writes data into a register.
 *
 * @param handle           - The SPI device handle.
 * @param register_address - Address of the register.
 * @param register_value   - Data value to write.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_register_value(spi_device_handle_t handle,
				uint8_t register_address,
				uint8_t register_value)
{
	uint8_t data_buffer[2] = {0, 0};

    data_buffer[0] = ADXL345_SPI_WRITE | register_address;
    data_buffer[1] = register_value;
    spi_transaction_t t = {
            .cmd = data_buffer[0]<<8 | data_buffer[1],
    };
    esp_err_t ret = spi_device_polling_transmit(handle,&t);
    if  (ret != ESP_OK) {
        ESP_LOGE(TAG, "Got an error setting a register: %d, err: %d", register_address, ret);
    }
}

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the ADXL345
 *        part is present.
 *
 * @param handle             - SPI device handle.
 * @param *buscfg            - Pointer to the bus configuration
 * @param *devcfg            - Pointer to the device configuration
 *
 * @return status    - Result of the initialization procedure.
 *                     Example: -1 - SPI peripheral was not initialized or
 *                                   ADXL345 part is not present.
 *                               0 - SPI peripheral is initialized and
 *                                   ADXL345 part is present.
*******************************************************************************/
int32_t adxl345_init(spi_device_handle_t handle, spi_bus_config_t *buscfg, 
                        spi_device_interface_config_t *devcfg)
{
	esp_err_t ret;

    ESP_LOGI(TAG, "Initializing bus SPI%d...", ADXL_SPI+1);
    buscfg->miso_io_num = ADXL_MISO;
    buscfg->mosi_io_num = ADXL_MOSI;
    buscfg->sclk_io_num = ADXL_SCK;
    buscfg->quadwp_io_num = -1;
    buscfg->quadhd_io_num = -1;
    buscfg->flags = SPICOMMON_BUSFLAG_MASTER;
    
    //Initialize the SPI bus
//    ret = spi_bus_initialize(ADXL_SPI, buscfg, SPI_DMA_CH_AUTO);
    ret = spi_bus_initialize(ADXL_SPI, buscfg, 0);  // zero disables DMA (no need for it)

    if  (ret != ESP_OK) {
        ESP_LOGE(TAG, "Got an error configuring bus: %d", ret);
        return -1;
    }

    ESP_LOGI(TAG, "Adding ADXL345 device to SPI bus...");
    devcfg->command_bits = 16;              // two bytes for a command (write a register)
    devcfg->address_bits = 0;               // no address phase for ADXL345
    devcfg->clock_speed_hz = ADXL_CLK_FREQ; // 2MHz should be conservative 
    devcfg->mode = 3;                       // SPI mode 3 CPOL=1 and CPHA=1
    devcfg->spics_io_num = ADXL_CS;
    devcfg->queue_size = 1;
    devcfg->flags = SPI_DEVICE_HALFDUPLEX;
    devcfg->input_delay_ns = ADXL_INPUT_DELAY_NS;  // should be zero delay until we figure it out
    //devcfg->cs_ena_pretrans = 1; // https://github.com/espressif/esp-idf/issues/7825 (WAG)
   
    //Attach the ADXL345 to the SPI bus
    // I'm using the global variable adxl handle because I couldn't get it to work
    // passing by reference.
    ret = spi_bus_add_device(ADXL_SPI, devcfg, &adxl);
    if  (ret != ESP_OK) {
        ESP_LOGE(TAG, "Got an error adding device: %d", ret);
        return -1;
    }

    ESP_LOGI(TAG, "Going out of measurement mode");
    adxl345_set_power_mode(adxl, 0);

    ESP_LOGI(TAG, "Setting measurement range to full range");
    // I believe full range (ADXL345_FULL_RES) overrides G range setting
    adxl345_set_range_resolution(adxl, ADXL345_RANGE_PM_2G, ADXL345_FULL_RES); 

    ESP_LOGI(TAG, "Setting x,y, and z offsets to zero");
    adxl345_set_offset(adxl, 0, 0, 0);

    ESP_LOGI(TAG, "Setting BW_RATE to 4 (1.56 Hz samples)");
    // rate 7 would be 12.5 Hz which gives us more than 5Hz signal discrimination
    adxl345_set_register_value(adxl, ADXL345_BW_RATE, ADXL345_RATE(4));

    // ESP_LOGI(TAG, "Setting FIFO_CTL to 0b10010000 (stream mode, 16 sample watermark interrupt level)");
    // adxl345_set_register_value(adxl, ADXL345_FIFO_CTL, 0b10010000);
    ESP_LOGI(TAG, "Setting FIFO_CTL to zero (bypassed)");
    adxl345_set_register_value(adxl, ADXL345_FIFO_CTL, 0);

    ESP_LOGI(TAG, "Reading the device ID");     // verify there is a device!
    // Again using the global handle
	if (adxl345_get_register_value(adxl, ADXL345_DEVID) != ADXL345_ID)
		return -1;

	return 0;
}

/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param handle           - The SPI device handle.
 * @param pwr_mode         - Power mode.
 *                              Example: 0x0 - standby mode.
 *                                       0x1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_power_mode(spi_device_handle_t handle,
			    uint8_t pwr_mode)
{
	uint8_t old_power_ctl = 0;
	uint8_t new_power_ctl = 0;

	old_power_ctl = adxl345_get_register_value(handle,
			ADXL345_POWER_CTL);
	new_power_ctl = old_power_ctl & ~ADXL345_PCTL_MEASURE;
	new_power_ctl = new_power_ctl | (pwr_mode * ADXL345_PCTL_MEASURE);
	adxl345_set_register_value(handle,
			ADXL345_POWER_CTL,
			new_power_ctl);
}


/***************************************************************************//**
 * @brief Reads the raw output data of each axis.
 *
 * @param handle    - The SPI device handle.
 * @param x         - X-axis's output data.
 * @param y         - Y-axis's output data.
 * @param z         - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void adxl345_get_xyz(spi_device_handle_t handle,
		     int16_t* x,
		     int16_t* y,
		     int16_t* z)
{
    uint8_t mb_read_datax0 = ADXL345_SPI_READ | 
                        ADXL345_SPI_MB | ADXL345_DATAX0;
    uint8_t read_buffer[8] = {0,0,0,0,0,0,0,0};

    spi_transaction_ext_t t; // supports variable command bits
    memset(&t,0, sizeof(t));

    t.base.flags = SPI_TRANS_VARIABLE_CMD;
    t.command_bits = 8;
    t.base.cmd = mb_read_datax0;
    t.base.rxlength = 6*8;
    t.base.rx_buffer = read_buffer;

    esp_err_t ret = spi_device_polling_transmit(handle, (spi_transaction_t *)&t);
    if  (ret != ESP_OK) {
        ESP_LOGE(TAG, "Got an error reading xyz raw: err: %d", ret);
    }

    /* x = ((ADXL345_DATAX1) << 8) + ADXL345_DATAX0 */
    *x = ((int16_t)read_buffer[1] << 8) + read_buffer[0];
    /* y = ((ADXL345_DATAY1) << 8) + ADXL345_DATAY0 */
    *y = ((int16_t)read_buffer[3] << 8) + read_buffer[2];
    /* z = ((ADXL345_DATAZ1) << 8) + ADXL345_DATAZ0 */
    *z = ((int16_t)read_buffer[5] << 8) + read_buffer[4];

}

/***************************************************************************//**
 * @brief Reads the raw output data of each axis and converts it to g.
 *
 * @param handle           - The SPI device handle.
 * @param x                - X-axis's output data.
 * @param y                - Y-axis's output data.
 * @param z                - Z-axis's output data.
 *
 * @return None.
*******************************************************************************/
void adxl345_get_g_xyz(spi_device_handle_t handle,
		       float* x,
		       float* y,
		       float* z)
{
	int16_t x_data = 0;  // X-axis's output data.
	int16_t y_data = 0;  // Y-axis's output data.
	int16_t z_data = 0;  // Z-axis's output data.
    uint8_t data_format = adxl345_get_register_value(handle, ADXL345_DATA_FORMAT);
    bool full_resolution_set = (((data_format & 0b00001000) == 0b00001000) ? true : false);
    uint16_t range_multiplier = ((data_format & 0b00000011)<<1) + 1;

	adxl345_get_xyz(handle, &x_data, &y_data, &z_data);
	*x = (float)(full_resolution_set ? (x_data * ADXL345_SCALE_FACTOR) :
		     (x_data * ADXL345_SCALE_FACTOR * range_multiplier));
	*y = (float)(full_resolution_set ? (y_data * ADXL345_SCALE_FACTOR) :
		     (y_data * ADXL345_SCALE_FACTOR * range_multiplier));
	*z = (float)(full_resolution_set ? (z_data * ADXL345_SCALE_FACTOR) :
		     (z_data * ADXL345_SCALE_FACTOR * range_multiplier));
}

/***************************************************************************//**
 * @brief Sets an offset value for each axis (Offset Calibration).
 *
 * @param handle      - The SPI device handle.
 * @param x_offset - X-axis's offset.
 * @param y_offset - Y-axis's offset.
 * @param z_offset - Z-axis's offset.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_offset(spi_device_handle_t handle,
			uint8_t x_offset,
			uint8_t y_offset,
			uint8_t z_offset)
{
	adxl345_set_register_value(handle,
				   ADXL345_OFSX,
				   x_offset);
	adxl345_set_register_value(handle,
				   ADXL345_OFSY,
				   y_offset);
	adxl345_set_register_value(handle,
				   ADXL345_OFSZ,
				   z_offset);
}

/***************************************************************************//**
 * @brief Selects the measurement range.
 *
 * @param handle      - The SPI device handle.
 * @param g_range  - Range option.
 *                   Example: ADXL345_RANGE_PM_2G  - +-2 g
 *                            ADXL345_RANGE_PM_4G  - +-4 g
 *                            ADXL345_RANGE_PM_8G  - +-8 g
 *                            ADXL345_RANGE_PM_16G - +-16 g
 * @param full_res - Full resolution option.
 *                   Example: 0x0 - Disables full resolution.
 *                            ADXL345_FULL_RES - Enables full resolution.
 *
 * @return None.
*******************************************************************************/
void adxl345_set_range_resolution(spi_device_handle_t handle,
				  uint8_t g_range,
				  uint8_t full_res)
{
	uint8_t old_data_format = 0;
	uint8_t new_data_format = 0;

	old_data_format = adxl345_get_register_value(handle,
			  ADXL345_DATA_FORMAT);
	new_data_format = old_data_format &
			  ~(ADXL345_RANGE(0x3) | ADXL345_FULL_RES);
	new_data_format =  new_data_format | ADXL345_RANGE(g_range) | full_res;
	adxl345_set_register_value(handle,
				   ADXL345_DATA_FORMAT,
				   new_data_format);
//	dev->selected_range = (1 << (g_range + 1)); // would be nice to have device structure
//	dev->full_resolution_set = full_res ? 1 : 0;
}
