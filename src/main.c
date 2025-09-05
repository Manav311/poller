/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

 #include "rfal_nfc.h"
 
 LOG_MODULE_REGISTER(Lesson7_Exercise1, LOG_LEVEL_INF);
 
 /* STEP 3 - Retrieve the API-device structure */
#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB
struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(nfc0), SPIOP, 0);

const struct gpio_dt_spec ledspec = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);


 int main(void)
 {
     int err;

     //ReturnCode err1 = rfalNfcInitialize();

     err = gpio_is_ready_dt(&ledspec);
	if (!err) {
		LOG_ERR("Error: GPIO device is not ready, err: %d", err);
		return 0;
	}

     err = spi_is_ready_dt(&spispec);
	if (!err) {
		LOG_ERR("Error: SPI device is not ready, err: %d", err);
		return 0;
	}
    gpio_pin_configure_dt(&ledspec, GPIO_OUTPUT_ACTIVE);

     LOG_INF("Lesson 7 - Exercise 1 started");
 
     /* STEP 9.2 - Define variables to store the temperature, pressure and humidity */
    
 
     while (1) {
         /* STEP 9.3 - Continuously read out sensor data using the sensor API calls */
        gpio_pin_toggle_dt(&ledspec);
         LOG_INF("Lesson 7 - Exercise 1 started");
         k_sleep(K_MSEC(1000));
     }
     
     return 0;
 }