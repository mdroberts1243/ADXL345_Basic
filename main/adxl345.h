/*
 *
 * From Analog Devices NO OS version
 *
 */

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
 *   @file   adxl345.h
 *   @brief  Include file for ADXL345 Driver.
 *   @author modified by Mark Roberts (mdroberts1243@gmail.com)
********************************************************************************
 * Parts originally Copyright 2012(c) Analog Devices, Inc.
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

#ifndef __ADXL345_H__
#define __ADXL345_H__

/* I2C address of the device */
#define ADXL345_ADDRESS		0x1D	// not used here

/* SPI commands */
#define ADXL345_SPI_READ        (1 << 7)
#define ADXL345_SPI_WRITE       (0 << 7)
#define ADXL345_SPI_MB          (1 << 6)

/* ADXL345 Register Map */
#define	ADXL345_DEVID           0x00 // R   Device ID.
#define ADXL345_THRESH_TAP      0x1D // R/W Tap threshold.
#define ADXL345_OFSX            0x1E // R/W X-axis offset.
#define ADXL345_OFSY            0x1F // R/W Y-axis offset.
#define ADXL345_OFSZ            0x20 // R/W Z-axis offset.
#define ADXL345_DUR             0x21 // R/W Tap duration.
#define ADXL345_LATENT          0x22 // R/W Tap latency.
#define ADXL345_WINDOW          0x23 // R/W Tap window.
#define ADXL345_THRESH_ACT      0x24 // R/W Activity threshold.
#define ADXL345_THRESH_INACT    0x25 // R/W Inactivity threshold.
#define ADXL345_TIME_INACT      0x26 // R/W Inactivity time.
#define ADXL345_ACT_INACT_CTL   0x27 // R/W Axis enable control for activity
// and inactivity detection.
#define ADXL345_THRESH_FF       0x28 // R/W Free-fall threshold.
#define ADXL345_TIME_FF         0x29 // R/W Free-fall time.
#define ADXL345_TAP_AXES        0x2A // R/W Axis control for tap/double tap.
#define ADXL345_ACT_TAP_STATUS  0x2B // R   Source of tap/double tap.
#define ADXL345_BW_RATE         0x2C // R/W Data rate and power mode control.
#define ADXL345_POWER_CTL       0x2D // R/W Power saving features control.
#define ADXL345_INT_ENABLE      0x2E // R/W Interrupt enable control.
#define ADXL345_INT_MAP         0x2F // R/W Interrupt mapping control.
#define ADXL345_INT_SOURCE      0x30 // R   Source of interrupts.
#define ADXL345_DATA_FORMAT     0x31 // R/W Data format control.
#define ADXL345_DATAX0          0x32 // R   X-Axis Data 0.
#define ADXL345_DATAX1          0x33 // R   X-Axis Data 1.
#define ADXL345_DATAY0          0x34 // R   Y-Axis Data 0.
#define ADXL345_DATAY1          0x35 // R   Y-Axis Data 1.
#define ADXL345_DATAZ0          0x36 // R   Z-Axis Data 0.
#define ADXL345_DATAZ1          0x37 // R   Z-Axis Data 1.
#define ADXL345_FIFO_CTL        0x38 // R/W FIFO control.
#define ADXL345_FIFO_STATUS     0x39 // R   FIFO status.

/* ADXL345_ACT_INACT_CTL definition */
#define ADXL345_ACT_ACDC        (1 << 7)
#define ADXL345_ACT_X_EN        (1 << 6)
#define ADXL345_ACT_Y_EN        (1 << 5)
#define ADXL345_ACT_Z_EN        (1 << 4)
#define ADXL345_INACT_ACDC      (1 << 3)
#define ADXL345_INACT_X_EN      (1 << 2)
#define ADXL345_INACT_Y_EN      (1 << 1)
#define ADXL345_INACT_Z_EN      (1 << 0)

/* ADXL345_TAP_AXES definition */
#define ADXL345_SUPPRESS        (1 << 3)
#define ADXL345_TAP_X_EN        (1 << 2)
#define ADXL345_TAP_Y_EN        (1 << 1)
#define ADXL345_TAP_Z_EN        (1 << 0)

/* ADXL345_ACT_TAP_STATUS definition */
#define ADXL345_ACT_X_SRC       (1 << 6)
#define ADXL345_ACT_Y_SRC       (1 << 5)
#define ADXL345_ACT_Z_SRC       (1 << 4)
#define ADXL345_ASLEEP          (1 << 3)
#define ADXL345_TAP_X_SRC       (1 << 2)
#define ADXL345_TAP_Y_SRC       (1 << 1)
#define ADXL345_TAP_Z_SRC       (1 << 0)

/* ADXL345_BW_RATE definition */
#define ADXL345_LOW_POWER       (1 << 4)
#define ADXL345_RATE(x)         ((x) & 0xF)

/* ADXL345_POWER_CTL definition */
#define ADXL345_PCTL_LINK       (1 << 5)
#define ADXL345_PCTL_AUTO_SLEEP (1 << 4)
#define ADXL345_PCTL_MEASURE    (1 << 3)
#define ADXL345_PCTL_SLEEP      (1 << 2)
#define ADXL345_PCTL_WAKEUP(x)  ((x) & 0x3)

/* ADXL345_INT_ENABLE / ADXL345_INT_MAP / ADXL345_INT_SOURCE definition */
#define ADXL345_DATA_READY      (1 << 7)
#define ADXL345_SINGLE_TAP      (1 << 6)
#define ADXL345_DOUBLE_TAP      (1 << 5)
#define ADXL345_ACTIVITY        (1 << 4)
#define ADXL345_INACTIVITY      (1 << 3)
#define ADXL345_FREE_FALL       (1 << 2)
#define ADXL345_WATERMARK       (1 << 1)
#define ADXL345_OVERRUN         (1 << 0)

/* ADXL345_DATA_FORMAT definition */
#define ADXL345_SELF_TEST       (1 << 7)
#define ADXL345_SPI             (1 << 6)
#define ADXL345_INT_INVERT      (1 << 5)
#define ADXL345_FULL_RES        (1 << 3)
#define ADXL345_JUSTIFY         (1 << 2)
#define ADXL345_RANGE(x)        ((x) & 0x3)

/* ADXL345_RANGE(x) options */
#define ADXL345_RANGE_PM_2G     0
#define ADXL345_RANGE_PM_4G     1
#define ADXL345_RANGE_PM_8G     2
#define ADXL345_RANGE_PM_16G    3

/* ADXL345_FIFO_CTL definition */
#define ADXL345_FIFO_MODE(x)    (((x) & 0x3) << 6)
#define ADXL345_TRIGGER         (1 << 5)
#define ADXL345_SAMPLES(x)      ((x) & 0x1F)

/* ADXL345_FIFO_MODE(x) options */
#define ADXL345_FIFO_BYPASS     0
#define ADXL345_FIFO_FIFO       1
#define ADXL345_FIFO_STREAM     2
#define ADXL345_FIFO_TRIGGER    3

/* ADXL345_FIFO_STATUS definition */
#define ADXL345_FIFO_TRIG       (1 << 7)
#define ADXL345_ENTRIES(x)      ((x) & 0x3F)

/* ADXL345 ID */
#define ADXL345_ID              0xE5

/* ADXL345 Full Resolution Scale Factor */
#define ADXL345_SCALE_FACTOR    0.0039


/* 
 * My attempt at an ESP-IDF polling version loosely based on Analog Device's API
 */ 
/*****************************************************************************
 * @brief Reads the value of a register.
 *
 * @param handle           - The SPI device handle.
 * @param register_address - Address of the register.
 *
 * @return register_value  - Value of the register.
*******************************************************************************/
uint8_t adxl345_get_register_value(spi_device_handle_t handle, uint8_t register_address);

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
				uint8_t register_value);

/***************************************************************************//**
 * @brief Initializes the communication peripheral and checks if the ADXL345
 *        part is present.
 *
 * @param handle            - The SPI device handle.
 * @param init_param    - The structure that contains the device initial
 * 		       parameters.
 *
 * @return status    - Result of the initialization procedure.
 *                     Example: -1 - SPI peripheral was not initialized or
 *                                   ADXL345 part is not present.
 *                               0 - SPI peripheral is initialized and
 *                                   ADXL345 part is present.
*******************************************************************************/
int32_t adxl345_init(spi_device_handle_t handle, spi_bus_config_t *buscfg, 
                        spi_device_interface_config_t *devcfg);

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
			    uint8_t pwr_mode);

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
		     int16_t* z);

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
		       float* z);

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
			uint8_t z_offset);

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
				  uint8_t full_res);

/******************************************************************************/				  			

/******************************************************************************/				  			

/******************************************************************************/				  			

/*
 * Specific to the board...  ESP32S2 Solo Module based.
 */
#define ADXL_SPI FSPI_HOST
#define ADXL_CS GPIO_NUM_10
#define ADXL_MOSI GPIO_NUM_11
#define ADXL_SCK GPIO_NUM_12
#define ADXL_MISO GPIO_NUM_13
#define ADXL_INT GPIO_NUM_14

#define ADXL_CLK_FREQ         (2000000)   //When powered by 3.3V, ADXL is 2MHz
#define ADXL_INPUT_DELAY_NS   0  // zero until we know if we need it ((1000*1000*1000/EEPROM_CLK_FREQ)/2+20)

/* THE END */
#endif