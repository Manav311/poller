#include "st25r3916b_driver.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(st25r3916b, LOG_LEVEL_INF);

// --- ST25R3916B Register Addresses ---
#define ST25R_REG_OP_CONTROL         0x02
#define ST25R_REG_MODE               0x03
#define ST25R_REG_AUX                0x0A
#define ST25R_REG_REGULATOR          0x2C
#define ST25R_REG_IRQ_MAIN           0x1A
#define ST25R_REG_IRQ_MASK_MAIN      0x16
#define ST25R_REG_IRQ_TIMER_NFC_MASK 0x17
#define ST25R_REG_IRQ_TIMER_NFC      0x1B
#define ST25R_REG_FIELD_ACT          0x2A
#define ST25R_REG_FIELD_DEACT        0x2B
#define ST25R_REG_AUX_DISPLAY        0x31
#define ST25R_REG_TX_DRIVER_REGISTER 0x28
#define ST25R_REG_AD_CONV_OUTPUT     0x25

// --- Bit definitions from datasheet ---
#define ST25R_IRQ_OSC                (1U << 7)
#define ST25R_IRQ_EXTERNAL_FIELD_DETECTED (1U << 4)
#define ST25R_AUX_DISPLAY_EXTERNAL_FIELD_BIT (1U << 6)
#define ST25R_AUX_DISPLAY_FDO_FIELD_ON_BIT (1U << 7)

// ST25R3916B Commands
#define ST25R_CMD_SW_RESET           0xC0

// Device handles
static const struct device *spi_dev;
static struct spi_config spi_cfg;
static const struct device *irq_gpio_dev;
static struct gpio_callback irq_cb_data;

// Global flag to signal IRQ to the main application logic
volatile bool g_st25r_irq_pending = false;

// IRQ handler for ST25R3916B
static void st25r_irq_handler(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    // Set a global flag to be processed in the main loop
    g_st25r_irq_pending = true;
    LOG_INF("ST25R IRQ occurred");

    // Read and clear IRQ registers on the ST25R3916B
    uint8_t irq_status_main;
    st25r391x_read_register(ST25R_REG_IRQ_MAIN, &irq_status_main);
    LOG_INF("IRQ Main Status: 0x%02X", irq_status_main);

    uint8_t irq_status_timer_nfc;
    st25r391x_read_register(ST25R_REG_IRQ_TIMER_NFC, &irq_status_timer_nfc);
    LOG_INF("IRQ Timer & NFC Status: 0x%02X", irq_status_timer_nfc);
}

int st25r_spi_init(void)
{
    spi_dev = DEVICE_DT_GET(DT_ALIAS(st25_spi));
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("ST25R SPI device not ready");
        return -ENODEV;
    }

    spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_OP_MODE_MASTER;
    spi_cfg.frequency = 4000000; // 4 MHz
    spi_cfg.slave = 0;

    LOG_INF("ST25R SPI Initialized with Mode 1");
    return 0;
}

int st25r391x_write_register(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2] = { (uint8_t)(reg | 0x80), value }; // MSB=1 for write
    
    struct spi_buf tx_spi_buf = {
        .buf = tx_buf,
        .len = sizeof(tx_buf)
    };
    
    struct spi_buf_set tx_spi_buf_set = {
        .buffers = &tx_spi_buf,
        .count = 1
    };

    int ret = spi_transceive(spi_dev, &spi_cfg, &tx_spi_buf_set, NULL);
    if (ret < 0) {
        LOG_ERR("SPI write failed: %d", ret);
        return ret;
    }

    LOG_DBG("SPI Write: Reg 0x%02X, Val 0x%02X", reg, value);
    return 0;
}

int st25r391x_read_register(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buf[2] = { (uint8_t)(reg & 0x7F), 0x00 }; // MSB=0 for read
    uint8_t rx_buf[2];

    struct spi_buf tx_spi_buf = {
        .buf = tx_buf,
        .len = sizeof(tx_buf)
    };
    
    struct spi_buf rx_spi_buf = {
        .buf = rx_buf,
        .len = sizeof(rx_buf)
    };
    
    struct spi_buf_set tx_spi_buf_set = {
        .buffers = &tx_spi_buf,
        .count = 1
    };
    
    struct spi_buf_set rx_spi_buf_set = {
        .buffers = &rx_spi_buf,
        .count = 1
    };

    int ret = spi_transceive(spi_dev, &spi_cfg, &tx_spi_buf_set, &rx_spi_buf_set);
    if (ret < 0) {
        LOG_ERR("SPI read failed: %d", ret);
        return ret;
    }

    *value = rx_buf[1]; // The actual data is in the second byte
    LOG_DBG("SPI Read: Reg 0x%02X, Val 0x%02X", reg, *value);
    return 0;
}

int st25r391x_send_command(uint8_t command)
{
    uint8_t tx_buf[1] = { command };
    
    struct spi_buf tx_spi_buf = {
        .buf = tx_buf,
        .len = sizeof(tx_buf)
    };
    
    struct spi_buf_set tx_spi_buf_set = {
        .buffers = &tx_spi_buf,
        .count = 1
    };

    int ret = spi_transceive(spi_dev, &spi_cfg, &tx_spi_buf_set, NULL);
    if (ret < 0) {
        LOG_ERR("SPI command failed: %d", ret);
        return ret;
    }

    LOG_DBG("SPI Command: 0x%02X", command);
    return 0;
}

int st25r_irq_init(void)
{
    irq_gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(irq_gpio_dev)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure(irq_gpio_dev, 31, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure IRQ pin: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure(irq_gpio_dev, 31, GPIO_INT_EDGE_FALLING);
    if (ret < 0) {
        LOG_ERR("Failed to configure IRQ interrupt: %d", ret);
        return ret;
    }

    gpio_init_callback(&irq_cb_data, st25r_irq_handler, BIT(31));
    ret = gpio_add_callback(irq_gpio_dev, &irq_cb_data);
    if (ret < 0) {
        LOG_ERR("Failed to add IRQ callback: %d", ret);
        return ret;
    }

    LOG_INF("ST25R IRQ Pin Initialized");
    return 0;
}

int st25r3916b_initial_setup(void)
{
    LOG_INF("Performing ST25R3916B Soft Reset...");
    int ret = st25r391x_send_command(ST25R_CMD_SW_RESET);
    if (ret < 0) return ret;
    
    k_msleep(10); // Wait after reset

    // Mask all main interrupts initially
    ret = st25r391x_write_register(ST25R_REG_IRQ_MASK_MAIN, 0xFF);
    if (ret < 0) return ret;
    
    // Clear any pending IRQs by reading the registers
    uint8_t dummy_irq_read;
    st25r391x_read_register(ST25R_REG_IRQ_MAIN, &dummy_irq_read);
    st25r391x_read_register(ST25R_REG_IRQ_TIMER_NFC, &dummy_irq_read);
    LOG_INF("All ST25R IRQs masked and cleared");

    // Wait for oscillator to stabilize
    ret = st25r391x_write_register(ST25R_REG_IRQ_MASK_MAIN, (uint8_t)(~ST25R_IRQ_OSC));
    if (ret < 0) return ret;
    
    LOG_INF("Waiting for ST25R3916B oscillator to stabilize...");
    uint8_t irq_status = 0;
    bool osc_stable = false;
    
    for (int i = 0; i < 200; i++) {
        ret = st25r391x_read_register(ST25R_REG_IRQ_MAIN, &irq_status);
        if (ret < 0) return ret;
        
        if (irq_status & ST25R_IRQ_OSC) {
            osc_stable = true;
            LOG_INF("ST25R3916B Oscillator stable (IRQ: 0x%02X)", irq_status);
            break;
        }
        k_msleep(1);
    }
    
    if (!osc_stable) {
        LOG_ERR("ST25R3916B Oscillator failed to stabilize within timeout");
        return -ETIMEDOUT;
    }

    // Re-mask all IRQs
    ret = st25r391x_write_register(ST25R_REG_IRQ_MASK_MAIN, 0xFF);
    if (ret < 0) return ret;
    
    st25r391x_read_register(ST25R_REG_IRQ_MAIN, &dummy_irq_read);
    st25r391x_read_register(ST25R_REG_IRQ_TIMER_NFC, &dummy_irq_read);

    // Set Mode Register for General Purpose operation
    ret = st25r391x_write_register(ST25R_REG_MODE, 0x00);
    if (ret < 0) return ret;
    
    LOG_INF("ST25R3916B Mode Register set to 0x00 (General Purpose)");
    LOG_INF("ST25R3916B Initial setup complete");
    
    return 0;
}

int st25r3916b_prepare_for_rf_field(void)
{
    LOG_INF("Preparing ST25R3916B for RF field generation...");

    uint8_t op_control_val;
    int ret;

    // Ensure chip is in Ready mode
    ret = st25r391x_read_register(ST25R_REG_OP_CONTROL, &op_control_val);
    if (ret < 0) return ret;
    
    op_control_val |= (1U << 7); // Set 'en' bit
    ret = st25r391x_write_register(ST25R_REG_OP_CONTROL, op_control_val);
    if (ret < 0) return ret;
    
    k_msleep(5);
    LOG_INF("OP_CONTROL after enabling 'en' bit: 0x%02X", op_control_val);

    // Set regulated voltage
    ret = st25r391x_write_register(ST25R_REG_REGULATOR, 0x07);
    if (ret < 0) return ret;
    
    LOG_INF("Regulator set to 0x07");

    // Set Field On/Off Thresholds
    ret = st25r391x_write_register(ST25R_REG_FIELD_ACT, 0x1F);
    if (ret < 0) return ret;
    
    ret = st25r391x_write_register(ST25R_REG_FIELD_DEACT, 0x1E);
    if (ret < 0) return ret;
    
    LOG_INF("External Field Detect Thresholds set: Act=0x1F, Deact=0x1E");

    // Disable the External Field Detector
    ret = st25r391x_read_register(ST25R_REG_OP_CONTROL, &op_control_val);
    if (ret < 0) return ret;
    
    op_control_val &= ~((1U << 1) | (1U << 0)); // Clear en_fd_c bits
    ret = st25r391x_write_register(ST25R_REG_OP_CONTROL, op_control_val);
    if (ret < 0) return ret;
    
    LOG_INF("OP_CONTROL External Field Detector disabled: 0x%02X", op_control_val);

    // Mask the External Field Detected interrupt
    uint8_t current_timer_nfc_irq_mask;
    ret = st25r391x_read_register(ST25R_REG_IRQ_TIMER_NFC_MASK, &current_timer_nfc_irq_mask);
    if (ret < 0) return ret;
    
    ret = st25r391x_write_register(ST25R_REG_IRQ_TIMER_NFC_MASK, 
                                   current_timer_nfc_irq_mask | ST25R_IRQ_EXTERNAL_FIELD_DETECTED);
    if (ret < 0) return ret;
    
    LOG_INF("IRQ Timer & NFC Mask after masking I_eon: 0x%02X", 
            (current_timer_nfc_irq_mask | ST25R_IRQ_EXTERNAL_FIELD_DETECTED));

    // Clear any pending IRQs
    uint8_t dummy_irq_read;
    st25r391x_read_register(ST25R_REG_IRQ_TIMER_NFC, &dummy_irq_read);
    LOG_INF("Cleared Timer & NFC IRQs (0x%02X)", dummy_irq_read);

    LOG_INF("ST25R3916B prepared for RF field generation");
    k_msleep(5);
    
    return 0;
}

uint8_t st25r3916b_measure_rf_amplitude(void)
{
    uint8_t op_control_val;
    int ret = st25r391x_read_register(ST25R_REG_OP_CONTROL, &op_control_val);
    if (ret < 0 || !(op_control_val & (1U << 7))) {
        LOG_WRN("ST25R: Chip not in Ready mode for amplitude measurement");
        return 0;
    }

    // Send Measure Amplitude command
    ret = st25r391x_send_command(0xD3);
    if (ret < 0) {
        LOG_ERR("Failed to send measure amplitude command");
        return 0;
    }

    // Wait for command completion
    k_usleep(50);

    uint8_t amp_value = 0;
    ret = st25r391x_read_register(ST25R_REG_AD_CONV_OUTPUT, &amp_value);
    if (ret < 0) {
        LOG_ERR("Failed to read amplitude value");
        return 0;
    }

    LOG_DBG("RF Amplitude Measurement: 0x%02X", amp_value);
    return amp_value;
}

int enable_charging(void)
{
    LOG_INF("Attempting to enable charging RF field...");

    uint8_t op_control_val;
    int ret;

    // Ensure chip is in Ready mode
    ret = st25r391x_read_register(ST25R_REG_OP_CONTROL, &op_control_val);
    if (ret < 0) return ret;
    
    op_control_val |= (1U << 7); // Set 'en' bit
    ret = st25r391x_write_register(ST25R_REG_OP_CONTROL, op_control_val);
    if (ret < 0) return ret;
    
    k_msleep(5);
    LOG_INF("OP_CONTROL after enabling 'en' bit: 0x%02X", op_control_val);

    // Set regulated voltage
    ret = st25r391x_write_register(ST25R_REG_REGULATOR, 0x07);
    if (ret < 0) return ret;
    
    LOG_INF("Regulator set to 0x07");

    // Configure TX Driver for output power
    ret = st25r391x_write_register(ST25R_REG_TX_DRIVER_REGISTER, 0x00);
    if (ret < 0) return ret;
    
    LOG_INF("TX_DRIVER_REGISTER set to 0x00 (Max Drive)");

    // Enable Transmitter (TX)
    ret = st25r391x_read_register(ST25R_REG_OP_CONTROL, &op_control_val);
    if (ret < 0) return ret;
    
    op_control_val |= (1U << 3); // Set 'tx_en' bit
    ret = st25r391x_write_register(ST25R_REG_OP_CONTROL, op_control_val);
    if (ret < 0) return ret;
    
    LOG_INF("OP_CONTROL after setting 'tx_en' bit: 0x%02X", op_control_val);

    k_msleep(10); // Wait for stabilization

    // Verify chip state and RF field presence
    uint8_t actual_op_control;
    ret = st25r391x_read_register(ST25R_REG_OP_CONTROL, &actual_op_control);
    if (ret < 0) return ret;
    
    LOG_INF("Final OP_CONTROL: 0x%02X", actual_op_control);

    uint8_t actual_regulator_val;
    st25r391x_read_register(ST25R_REG_REGULATOR, &actual_regulator_val);
    LOG_INF("Final REGULATOR: 0x%02X", actual_regulator_val);

    uint8_t actual_tx_driver_val;
    st25r391x_read_register(ST25R_REG_TX_DRIVER_REGISTER, &actual_tx_driver_val);
    LOG_INF("Final TX_DRIVER_REGISTER: 0x%02X", actual_tx_driver_val);

    uint8_t aux_display_status = 0;
    ret = st25r391x_read_register(ST25R_REG_AUX_DISPLAY, &aux_display_status);
    if (ret < 0) return ret;
    
    LOG_INF("AUX_DISPLAY after TX enable: 0x%02X", aux_display_status);

    if (aux_display_status & ST25R_AUX_DISPLAY_FDO_FIELD_ON_BIT) {
        LOG_INF("Internal RF field is active (fdo_field_on set, bit 7)");
    } else {
        LOG_WRN("Internal RF field is NOT active (fdo_field_on NOT set, bit 7)");
    }
    
    if (aux_display_status & ST25R_AUX_DISPLAY_EXTERNAL_FIELD_BIT) {
        LOG_INF("External field detected (efd_o set, bit 6)");
    } else {
        LOG_INF("External field NOT detected (efd_o NOT set, bit 6)");
    }

    if ((actual_op_control & (1U << 3)) && (aux_display_status & ST25R_AUX_DISPLAY_FDO_FIELD_ON_BIT)) {
        LOG_INF("RF field is successfully active (charging possible)");
    } else {
        LOG_ERR("RF field is NOT active! Check configuration and chip state");
        return -EIO;
    }
    
    return 0;
}

int disable_charging(void)
{
    LOG_INF("Disabling charging RF field...");

    uint8_t op_control_val;
    int ret = st25r391x_read_register(ST25R_REG_OP_CONTROL, &op_control_val);
    if (ret < 0) return ret;
    
    // Clear 'tx_en' bit while ensuring 'en' bit remains set
    op_control_val &= ~(1U << 3); // Clear 'tx_en' bit
    op_control_val |= (1U << 7);  // Ensure 'en' bit is high
    ret = st25r391x_write_register(ST25R_REG_OP_CONTROL, op_control_val);
    if (ret < 0) return ret;

    LOG_INF("RF field disabled (chip still in Ready mode: 0x%02X)", op_control_val);
    return 0;
}
