#ifndef OPA323_ADC_CONTROL_H
#define OPA323_ADC_CONTROL_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize SAADC to read photodiode connected to P0.03 (AIN1).
 */
int saadc_init(void);

/**
 * @brief Initialize PWM for LED brightness control.
 */
int pwm_led_init(void);

/**
 * @brief Perform an ADC conversion and update PWM duty cycle based on ambient light.
 */
void update_led_brightness_from_adc(void);

/**
 * @brief Initialize LEDs (Red = P0.17, Green = P0.19, Blue = P0.20)
 */
int leds_init(void);

/**
 * @brief Controls LED state for charging indication
 * @param charging true = Green LED ON, others OFF; false = Red LED ON, others OFF
 */
void set_led_charging(bool charging);

#endif // OPA323_ADC_CONTROL_H