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
 *  Mark Roberts (mdroberts1243@gmail.com)
 *  Modifications are Copyright (c) 2023 Mark Roberts
 */

/***************************************************************************//**
 *   @file   main.c
 *   @brief  Example for ADXL345 Driver.
 *   @author Mark Roberts (mdroberts1243@gmail.com)
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

static const char TAG[] = "main";

/* The (initially blank) structures we need for the 
 * ESP-IDF API calls.  These are retained.
 */
spi_device_handle_t adxl;    // this adxl is used by the API calls, it's global for convenience
static spi_bus_config_t buscfg; 
static spi_device_interface_config_t devcfg;

void app_main(void)
{
    /* Get pointers to the configuration data structures */
    spi_bus_config_t *buscfg_ptr = &buscfg;
    spi_device_interface_config_t *devcfg_ptr = &devcfg;

    /* Initialize the ADXL345 and check it is there */
    if (adxl345_init(adxl, buscfg_ptr, devcfg_ptr) != 0) {
        ESP_LOGE(TAG, "Initialization of the device failed!");
        abort();
    };
    /* Enable the device to produce data */
    ESP_LOGI(TAG, "Going into measurement mode");
    adxl345_set_power_mode(adxl, 1);

    ESP_LOGI(TAG, "Loop, reading raw xyz");
    while(1)
    {
    //    int16_t x, y, z;
        float xf, yf, zf;

        // try directly reading the x, y and z values from individual 8-bit registers
        // x = ((int16_t)adxl345_get_register_value(adxl,ADXL345_DATAX1) << 8) + adxl345_get_register_value(adxl,ADXL345_DATAX0);
        // y = ((int16_t)adxl345_get_register_value(adxl,ADXL345_DATAY1) << 8) + adxl345_get_register_value(adxl,ADXL345_DATAY0);
        // z = ((int16_t)adxl345_get_register_value(adxl,ADXL345_DATAZ1) << 8) + adxl345_get_register_value(adxl,ADXL345_DATAZ0);
        // ESP_LOGI(TAG, "Got Register Values x:%04x, y:%04x, z:%04x.",x,y,z);

        // using the 'raw' read function:
    //    adxl345_get_xyz(adxl, &x, &y, &z);
    //    ESP_LOGI(TAG, "Got x:%d, y:%d, z:%d.",x,y,z);
    
        // floating point in g:
        adxl345_get_g_xyz(adxl, &xf, &yf, &zf);
        ESP_LOGI(TAG, "Got x:%f, y:%f, z:%f.",xf,yf,zf);
        
        vTaskDelay(50);
    };
}