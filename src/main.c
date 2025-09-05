/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

 #include "rfal_nfc.h"
 
 LOG_MODULE_REGISTER(Lesson7_Exercise1, LOG_LEVEL_INF);
 
 
 int main(void)
 {
     int err;

     //ReturnCode err1 = rfalNfcInitialize();

     err = st25r_spi_init();
 
     LOG_INF("Lesson 7 - Exercise 1 started");
 
     /* STEP 9.2 - Define variables to store the temperature, pressure and humidity */
    
 
     while (1) {
         /* STEP 9.3 - Continuously read out sensor data using the sensor API calls */
        
         
         k_sleep(K_MSEC(1000));
     }
     
     return 0;
 }