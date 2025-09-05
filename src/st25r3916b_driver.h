#ifndef ST25R3916B_H
#define ST25R3916B_H

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdbool.h>

// --- Driver Core Functions ---

/**
 * @brief Initializes the SPI interface for communication with the ST25R3916B.
 */
int st25r_spi_init(void);

/**
 * @brief Writes a single byte to an ST25R3916B register.
 *
 * @param[in] reg The register address to write to.
 * @param[in] value The byte value to write.
 */
int st25r391x_write_register(uint8_t reg, uint8_t value);

/**
 * @brief Reads a single byte from an ST25R3916B register.
 *
 * @param[in] reg The register address to read from.
 * @param[out] value Pointer to store the read byte value.
 */
int st25r391x_read_register(uint8_t reg, uint8_t *value);

/**
 * @brief Sends a command to the ST25R3916B.
 * Commands are special single-byte operations.
 *
 * @param[in] command The command byte to send.
 */
int st25r391x_send_command(uint8_t command);

/**
 * @brief Configures the GPIO interrupt pin for the ST25R3916B IRQ.
 * The interrupt handler will signal pending events.
 */
int st25r_irq_init(void);

/**
 * @brief Global flag indicating if an ST25R3916B IRQ is pending.
 * This flag is set in the IRQ handler and should be cleared by the main application
 * after processing the interrupt.
 */
extern volatile bool g_st25r_irq_pending;

// --- NFC/WPT Specific Functions ---

/**
 * @brief Performs a soft reset of the ST25R3916B and waits for the oscillator to stabilize.
 * This should be called once at system startup.
 */
int st25r3916b_initial_setup(void);

/**
 * @brief Enables the ST25R3916B for external RF field detection mode.
 * This configures the necessary registers to sense an incoming RF field.
 */
int st25r3916b_prepare_for_rf_field(void);

/**
 * @brief Measures the amplitude of the RF signal on RFI inputs.
 * @return Raw 8-bit ADC value from the A/D converter output register (0x25).
 */
uint8_t st25r3916b_measure_rf_amplitude(void);

/**
 * @brief Commands the ST25R3916B to enable its RF field for potential charging.
 * This function requires further implementation to follow WLC protocol.
 */
int enable_charging(void);

/**
 * @brief Commands the ST25R3916B to disable its RF field, stopping charging.
 */
int disable_charging(void);

#endif // ST25R3916B_H