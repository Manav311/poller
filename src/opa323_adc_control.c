#include "opa323_adc_control.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(opa323_adc, LOG_LEVEL_INF);

// Define the ADC configuration
#define ADC_DEVICE_NODE DT_NODELABEL(adc)
#define ADC_RESOLUTION 10
#define ADC_GAIN ADC_GAIN_1
#define ADC_REFERENCE ADC_REF_VDD_1_4
#define ADC_ACQUISITION_TIME ADC_ACQ_TIME_DEFAULT
#define ADC_1ST_CHANNEL_ID 1
#define ADC_1ST_CHANNEL_INPUT NRF_SAADC_AIN1

// Define PWM devices
#define PWM_LED_R_NODE DT_ALIAS(pwm_led0)
#define PWM_LED_G_NODE DT_ALIAS(pwm_led1)
#define PWM_LED_B_NODE DT_ALIAS(pwm_led2)

// GPIO pins for LEDs
#define LED_R_PIN 17
#define LED_G_PIN 19
#define LED_B_PIN 20

// Averaging and thresholding constants
#define ADC_AVERAGE_COUNT 20
#define DARK_ADC_THRESHOLD 25
#define ADC_REFERENCE_MV 825.0f

// Global variables
static const struct device *adc_dev;
static const struct device *pwm_led_r;
static const struct device *pwm_led_g;
static const struct device *pwm_led_b;
static const struct device *gpio_dev;

static int16_t adc_sample_buffer[1];
static int16_t adc_history[ADC_AVERAGE_COUNT];
static uint8_t adc_history_idx = 0;
static bool adc_history_full = false;
static bool is_charging = false;

static struct adc_channel_cfg m_1st_channel_cfg = {
    .gain             = ADC_GAIN,
    .reference        = ADC_REFERENCE,
    .acquisition_time = ADC_ACQUISITION_TIME,
    .channel_id       = ADC_1ST_CHANNEL_ID,
#if defined(CONFIG_ADC_CONFIGURABLE_INPUTS)
    .input_positive   = ADC_1ST_CHANNEL_INPUT,
#endif
};

static struct adc_sequence sequence = {
    .buffer      = adc_sample_buffer,
    .buffer_size = sizeof(adc_sample_buffer),
    .resolution  = ADC_RESOLUTION,
};

int saadc_init(void)
{
    int ret;

    adc_dev = DEVICE_DT_GET(ADC_DEVICE_NODE);
    if (!device_is_ready(adc_dev)) {
        LOG_ERR("ADC device not ready");
        return -ENODEV;
    }

    ret = adc_channel_setup(adc_dev, &m_1st_channel_cfg);
    if (ret) {
        LOG_ERR("Setting up the ADC channel failed with code %d", ret);
        return ret;
    }

    (void)adc_sequence_init_dt(&m_1st_channel_cfg, &sequence);

    // Initialize ADC history
    memset(adc_history, 0, sizeof(adc_history));
    adc_history_idx = 0;
    adc_history_full = false;

    LOG_INF("ADC initialized successfully");
    return 0;
}

int pwm_led_init(void)
{
    pwm_led_r = DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(pwm_led0)));
    pwm_led_g = DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(pwm_led1)));
    pwm_led_b = DEVICE_DT_GET(DT_PWMS_CTLR(DT_ALIAS(pwm_led2)));

    if (!device_is_ready(pwm_led_r) || !device_is_ready(pwm_led_g) || !device_is_ready(pwm_led_b)) {
        LOG_ERR("PWM LED devices not ready");
        return -ENODEV;
    }

    // Initialize all PWM channels to 0 (LEDs off)
    pwm_set_dt(&(struct pwm_dt_spec){pwm_led_r, DT_PWMS_CHANNEL(DT_ALIAS(pwm_led0)), DT_PWMS_PERIOD(DT_ALIAS(pwm_led0)), DT_PWMS_FLAGS(DT_ALIAS(pwm_led0))}, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_led_g, DT_PWMS_CHANNEL(DT_ALIAS(pwm_led1)), DT_PWMS_PERIOD(DT_ALIAS(pwm_led1)), DT_PWMS_FLAGS(DT_ALIAS(pwm_led1))}, 0);
    pwm_set_dt(&(struct pwm_dt_spec){pwm_led_b, DT_PWMS_CHANNEL(DT_ALIAS(pwm_led2)), DT_PWMS_PERIOD(DT_ALIAS(pwm_led2)), DT_PWMS_FLAGS(DT_ALIAS(pwm_led2))}, 0);

    LOG_INF("PWM LEDs initialized successfully");
    return 0;
}

int leds_init(void)
{
    gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(gpio_dev)) {
        LOG_ERR("GPIO device not ready");
        return -ENODEV;
    }

    // Configure LED pins as outputs
    gpio_pin_configure(gpio_dev, LED_R_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio_dev, LED_G_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(gpio_dev, LED_B_PIN, GPIO_OUTPUT_INACTIVE);

    LOG_INF("LEDs initialized successfully");
    return 0;
}

void update_led_brightness_from_adc(void)
{
    int ret;
    int16_t raw_adc_value;

    // Perform ADC conversion
    ret = adc_read(adc_dev, &sequence);
    if (ret) {
        LOG_ERR("ADC read failed with code %d", ret);
        return;
    }

    raw_adc_value = adc_sample_buffer[0];

    // Store in averaging buffer
    adc_history[adc_history_idx] = raw_adc_value;
    adc_history_idx = (adc_history_idx + 1) % ADC_AVERAGE_COUNT;

    if (adc_history_idx == 0) {
        adc_history_full = true;
    }

    // Calculate average
    int32_t sum = 0;
    uint8_t count = adc_history_full ? ADC_AVERAGE_COUNT : adc_history_idx;
    
    for (int i = 0; i < count; i++) {
        sum += adc_history[i];
    }
    
    int16_t averaged_adc_value = (count == 0) ? 0 : (sum / count);

    // Clamp to valid range
    if (averaged_adc_value < 0) averaged_adc_value = 0;
    if (averaged_adc_value > 1023) averaged_adc_value = 1023;

    // Calculate voltage for logging
    float voltage_mv = ((float)averaged_adc_value / 1023.0f) * ADC_REFERENCE_MV;

    // Calculate brightness
    uint32_t final_brightness_pwm;
    
    if (averaged_adc_value <= DARK_ADC_THRESHOLD) {
        final_brightness_pwm = PWM_USEC(10); // Minimum brightness
    } else {
        float normalized_for_brightness = ((float)(averaged_adc_value - DARK_ADC_THRESHOLD)) / (1023 - DARK_ADC_THRESHOLD);
        if (normalized_for_brightness < 0.0f) normalized_for_brightness = 0.0f;
        if (normalized_for_brightness > 1.0f) normalized_for_brightness = 1.0f;

        // Apply logarithmic scaling
        float brightness_f = logf(1.0f + 9.0f * normalized_for_brightness) / logf(10.0f);
        final_brightness_pwm = PWM_USEC((uint32_t)(brightness_f * 1000.0f));
    }

    // Set PWM based on charging status
    struct pwm_dt_spec led_r_spec = {pwm_led_r, DT_PWMS_CHANNEL(DT_ALIAS(pwm_led0)), DT_PWMS_PERIOD(DT_ALIAS(pwm_led0)), DT_PWMS_FLAGS(DT_ALIAS(pwm_led0))};
    struct pwm_dt_spec led_g_spec = {pwm_led_g, DT_PWMS_CHANNEL(DT_ALIAS(pwm_led1)), DT_PWMS_PERIOD(DT_ALIAS(pwm_led1)), DT_PWMS_FLAGS(DT_ALIAS(pwm_led1))};
    struct pwm_dt_spec led_b_spec = {pwm_led_b, DT_PWMS_CHANNEL(DT_ALIAS(pwm_led2)), DT_PWMS_PERIOD(DT_ALIAS(pwm_led2)), DT_PWMS_FLAGS(DT_ALIAS(pwm_led2))};

    if (is_charging) {
        pwm_set_dt(&led_r_spec, 0);                    // Red off
        pwm_set_dt(&led_g_spec, final_brightness_pwm); // Green with brightness
        pwm_set_dt(&led_b_spec, 0);                    // Blue off
    } else {
        pwm_set_dt(&led_r_spec, final_brightness_pwm); // Red with brightness
        pwm_set_dt(&led_g_spec, 0);                    // Green off
        pwm_set_dt(&led_b_spec, 0);                    // Blue off
    }
}

void set_led_charging(bool charging)
{
    is_charging = charging;
}