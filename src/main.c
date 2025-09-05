#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/dis.h>

#include "st25r3916b_driver.h"
#include "opa323_adc_control.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define DEVICE_NAME         CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN     (sizeof(DEVICE_NAME) - 1)

// BLE advertising data
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL,
                  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
                  BT_UUID_16_ENCODE(BT_UUID_DIS_VAL)),
};

// BLE scan response data
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

// RF field state tracking
static bool rf_field_active = false;
static uint8_t initial_no_coil_amplitude = 0;
static bool baseline_calibrated = false;

// Coil detection thresholds - adjust based on calibration
#define RF_AMP_BASELINE_THRESHOLD_LOW  0x10
#define RF_AMP_COIL_DETECT_DROP        0x08

// Battery simulation
static uint8_t battery_level = 100;

static void connected(struct bt_conn *conn, uint8_t err)
{
    char addr[BT_ADDR_LE_STR_LEN];

    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Connected %s", addr);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    char addr[BT_ADDR_LE_STR_LEN];

    bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
    LOG_INF("Disconnected from %s (reason %u)", addr, reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }

    LOG_INF("Bluetooth initialized");

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }

    LOG_INF("Advertising successfully started");
}

static void bas_notify(void)
{
    battery_level--;
    if (battery_level == 0) {
        battery_level = 100;
    }

    bt_bas_set_battery_level(battery_level);
}

static void configure_system_settings(void)
{
    uint8_t io_config_reg2_val;
    
    // Read current IO Configuration Register 2 (0x01)
    if (st25r391x_read_register(0x01, &io_config_reg2_val) != 0) {
        LOG_ERR("Failed to read IO config register");
        return;
    }
    
    LOG_INF("ST25R: IO_CONFIG_REG_2 (0x01) before config: 0x%02X", io_config_reg2_val);

    // Set sup3V bit (bit 7) to 1 for 3.3V supply mode
    io_config_reg2_val |= (1U << 7); 
    // Set io_drv_lvl bit (bit 2) to 1 to increase IO driving level
    io_config_reg2_val |= (1U << 2);

    if (st25r391x_write_register(0x01, io_config_reg2_val) == 0) {
        LOG_INF("ST25R: IO_CONFIG_REG_2 (0x01) after config (sup3V=1, io_drv_lvl=1): 0x%02X", io_config_reg2_val);
    }
}

// Work queue for periodic tasks
static void battery_work_handler(struct k_work *work);
static void led_update_work_handler(struct k_work *work);
static void rf_detection_work_handler(struct k_work *work);

K_WORK_DEFINE(battery_work, battery_work_handler);
K_WORK_DEFINE(led_update_work, led_update_work_handler);
K_WORK_DEFINE(rf_detection_work, rf_detection_work_handler);

// Timers
static void battery_timer_handler(struct k_timer *timer);
static void led_timer_handler(struct k_timer *timer);
static void rf_timer_handler(struct k_timer *timer);

K_TIMER_DEFINE(battery_timer, battery_timer_handler, NULL);
K_TIMER_DEFINE(led_timer, led_timer_handler, NULL);
K_TIMER_DEFINE(rf_timer, rf_timer_handler, NULL);

static void battery_work_handler(struct k_work *work)
{
    bas_notify();
}

static void led_update_work_handler(struct k_work *work)
{
    update_led_brightness_from_adc();
}

static void rf_detection_work_handler(struct k_work *work)
{
    if (!baseline_calibrated) {
        // Calibrate baseline RF amplitude without coil
        LOG_INF("Calibrating RF amplitude baseline without coil...");
        
        if (enable_charging() == 0) {
            k_msleep(100); // Give field time to stabilize
            initial_no_coil_amplitude = st25r3916b_measure_rf_amplitude();
            disable_charging(); // Turn OFF RF field immediately after measurement
            
            LOG_INF("Baseline RF Amp (no coil): 0x%02X", initial_no_coil_amplitude);
            
            if (initial_no_coil_amplitude > 0) {
                baseline_calibrated = true;
                LOG_INF("RF amplitude baseline calibrated. Ready for coil detection.");
            } else {
                LOG_WRN("Failed to get a valid RF amplitude baseline. Retrying...");
            }
        }
        return;
    }

    // Detect coil using amplitude measurement
    uint8_t current_rf_amplitude;
    bool coil_detected = false;

    // Turn ON RF field momentarily to "ping" for coil presence
    if (enable_charging() == 0) {
        k_msleep(50); // Short delay for field stability
        current_rf_amplitude = st25r3916b_measure_rf_amplitude();
        disable_charging(); // Turn OFF RF field immediately after measurement

        LOG_INF("Pinged RF Amp: 0x%02X", current_rf_amplitude);

        // Logic to detect coil: amplitude drops significantly when coil is present
        if (initial_no_coil_amplitude > RF_AMP_BASELINE_THRESHOLD_LOW &&
            (initial_no_coil_amplitude - current_rf_amplitude) >= RF_AMP_COIL_DETECT_DROP) {
            coil_detected = true;
        }

        // Decision to enable/disable continuous RF field
        if (coil_detected) {
            if (!rf_field_active) {
                if (enable_charging() == 0) {
                    set_led_charging(true);
                    rf_field_active = true;
                    LOG_INF("NFC Coil Detected. Charging Enabled (Continuous RF Field).");
                }
            }
        } else {
            if (rf_field_active) {
                if (disable_charging() == 0) {
                    set_led_charging(false);
                    rf_field_active = false;
                    LOG_INF("No NFC Coil Detected. Charging Disabled (RF Field OFF).");
                }
            }
        }
    }
}

static void battery_timer_handler(struct k_timer *timer)
{
    k_work_submit(&battery_work);
}

static void led_timer_handler(struct k_timer *timer)
{
    k_work_submit(&led_update_work);
}

static void rf_timer_handler(struct k_timer *timer)
{
    k_work_submit(&rf_detection_work);
}

int main(void)
{
    int err;

    LOG_INF("Slimiot NFC Charging App Started");

    // Initialize Bluetooth
    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return 0;
    }

    // Initialize ST25R3916B NFC/RFID components
    err = st25r_spi_init();
    if (err) {
        LOG_ERR("ST25R SPI init failed (err %d)", err);
        return 0;
    }

    err = st25r_irq_init();
    if (err) {
        LOG_ERR("ST25R IRQ init failed (err %d)", err);
        return 0;
    }

    err = st25r3916b_initial_setup();
    if (err) {
        LOG_ERR("ST25R initial setup failed (err %d)", err);
        return 0;
    }

    configure_system_settings();

    err = st25r3916b_prepare_for_rf_field();
    if (err) {
        LOG_ERR("ST25R RF field preparation failed (err %d)", err);
        return 0;
    }

    LOG_INF("ST25R3916B prepared for RF field operation");

    // Initialize ambient light sensing and LED control
    err = pwm_led_init();
    if (err) {
        LOG_ERR("PWM LED init failed (err %d)", err);
        return 0;
    }

    err = saadc_init();
    if (err) {
        LOG_ERR("SAADC init failed (err %d)", err);
        return 0;
    }

    err = leds_init();
    if (err) {
        LOG_ERR("LEDs init failed (err %d)", err);
        return 0;
    }

    LOG_INF("All peripherals initialized successfully");

    // Start periodic timers
    k_timer_start(&battery_timer, K_SECONDS(2), K_SECONDS(2));
    k_timer_start(&led_timer, K_MSEC(100), K_MSEC(100));
    k_timer_start(&rf_timer, K_MSEC(500), K_MSEC(500));

    LOG_INF("Timers started, entering main loop");

    // Main loop - just handle IRQs and sleep
    while (1) {
        // Handle ST25R IRQ if pending
        if (g_st25r_irq_pending) {
            g_st25r_irq_pending = false;
            LOG_INF("Processing ST25R IRQ");
            // Additional IRQ processing can be added here if needed
        }

        // Sleep to allow work queue and interrupts to run
        k_msleep(10);
    }

    return 0;
}