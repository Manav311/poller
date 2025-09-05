# NFC Charger - nRF Connect SDK Migration

This project has been migrated from nRF5 SDK to nRF Connect SDK 3.0.0 for the nRF52832 microcontroller.

## Project Structure

```
nfc-charger/
├── CMakeLists.txt                          # Main build configuration
├── prj.conf                                # Zephyr project configuration
├── Kconfig                                 # Custom Kconfig options
├── west.yml                                # West manifest for dependencies
├── boards/
│   └── nrf52832dk_nrf52832.overlay        # Device tree overlay
├── src/
│   ├── main.c                             # Main application (migrated)
│   ├── st25r3916b_driver.c               # ST25R3916B driver (migrated)
│   ├── st25r3916b_driver.h               # ST25R3916B header (migrated)
│   ├── opa323_adc_control.c              # ADC/LED control (migrated)
│   └── opa323_adc_control.h              # ADC/LED header (migrated)
└── README.md                              # This file
```

## Key Migration Changes

### 1. Build System
- **nRF5 SDK**: Used Segger Embedded Studio with proprietary build system
- **nRF Connect SDK**: Uses CMake and Zephyr's build system

### 2. Hardware Abstraction
- **nRF5 SDK**: Direct register access and nRF5 HAL drivers
- **nRF Connect SDK**: Zephyr's device driver model with device tree configuration

### 3. RTOS Integration
- **nRF5 SDK**: No built-in RTOS (bare metal with SoftDevice)
- **nRF Connect SDK**: Zephyr RTOS with work queues and timers

### 4. Bluetooth Stack
- **nRF5 SDK**: SoftDevice S132/S140
- **nRF Connect SDK**: Zephyr Bluetooth stack

## Hardware Configuration

The device tree overlay (`boards/nrf52832dk_nrf52832.overlay`) defines:

- **SPI Interface**: For ST25R3916B communication
  - SCK: P0.29
  - MOSI: P0.28  
  - MISO: P0.27
  - CS: P0.16

- **RGB LEDs with PWM**: For ambient light adaptation
  - Red: P0.17
  - Green: P0.19  
  - Blue: P0.20

- **ADC Channel**: For photodiode ambient light sensing
  - Input: P0.03 (AIN1)

- **IRQ Pin**: For ST25R3916B interrupt
  - Pin: P0.31

## Building the Project

### Prerequisites
1. Install nRF Connect SDK 3.0.0
2. Install west tool
3. Set up toolchain (GCC ARM Embedded)

### Build Steps
```bash
# Initialize west workspace
west init -m https://github.com/nrfconnect/sdk-nrf --mr v2.3.0 ncs
cd ncs

# Clone this project
git clone <your-repo> nfc-charger
cd nfc-charger

# Update dependencies
west update

# Build the project
west build -b nrf52832dk_nrf52832

# Flash to device
west flash
```

## Key Features Preserved

### 1. NFC Wireless Charging
- ST25R3916B RF field generation
- Coil detection via amplitude measurement
- Automatic charging enable/disable

### 2. Ambient Light Adaptation
- Photodiode-based light sensing
- Logarithmic LED brightness scaling
- Averaging filter for stability

### 3. Bluetooth Low Energy
- Battery service (BAS)
- Device information service (DIS)
- Advertising with device name

### 4. Visual Feedback
- Red LED: Normal operation (brightness varies with ambient light)
- Green LED: Charging active (brightness varies with ambient light)

## Configuration Options

Key configurations in `prj.conf`:

```ini
# Bluetooth
CONFIG_BT=y
CONFIG_BT_PERIPHERAL=y
CONFIG_BT_BAS=y
CONFIG_BT_DIS=y

# Hardware drivers
CONFIG_SPI=y
CONFIG_ADC=y
CONFIG_PWM=y
CONFIG_GPIO=y

# Logging
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
```

## Migration Notes

### Challenges Addressed

1. **Timer Management**: Converted from app_timer to Zephyr k_timer
2. **Work Queues**: Replaced interrupt-driven code with work queue handlers
3. **Device Tree**: Hardware configuration moved from C code to device tree
4. **Error Handling**: Updated to use Zephyr error codes
5. **Logging**: Migrated from NRF_LOG to Zephyr LOG

### Code Changes

1. **Header Changes**:
   ```c
   // Old (nRF5 SDK)
   #include "nrf_drv_spi.h"
   #include "app_error.h"
   
   // New (nRF Connect SDK)
   #include <zephyr/drivers/spi.h>
   #include <zephyr/kernel.h>
   ```

2. **Initialization Pattern**:
   ```c
   // Old: Direct device initialization
   nrf_drv_spi_init(&spi, &config, handler, NULL);
   
   // New: Device tree based initialization
   spi_dev = DEVICE_DT_GET(DT_ALIAS(st25_spi));
   ```

3. **Work Scheduling**:
   ```c
   // Old: Direct function calls in main loop
   update_led_brightness_from_adc();
   
   // New: Work queue based
   k_work_submit(&led_update_work);
   ```

## Troubleshooting

### Common Issues

1. **SPI Communication Fails**
   - Check device tree pin assignments
   - Verify SPI mode configuration (Mode 1 for ST25R3916B)
   - Ensure CS pin is correctly configured

2. **ADC Readings Invalid**
   - Verify ADC channel configuration in device tree
   - Check reference voltage settings
   - Ensure pin multiplexing is correct

3. **PWM LEDs Not Working**
   - Check PWM device tree configuration
   - Verify pin assignments match hardware
   - Ensure PWM frequency and polarity are correct

4. **Bluetooth Not Advertising**
   - Check that CONFIG_BT=y in prj.conf
   - Verify device name configuration
   - Ensure sufficient heap/stack sizes

### Debug Tips

- Enable verbose logging: `CONFIG_LOG_DEFAULT_LEVEL=4`
- Use RTT for real-time logging during development
- Check Zephyr documentation for driver-specific configurations
- Use device tree overlays for board-specific customizations

## Testing

The migrated application maintains the same functionality:

1. **RF Field Testing**: Place NFC coil near antenna to trigger charging mode
2. **Light Adaptation**: Cover/uncover photodiode to see LED brightness change
3. **Bluetooth**: Connect with nRF Connect app to see battery service
4. **Visual Feedback**: LED color indicates charging state (red/green)

## Future Enhancements

Potential improvements enabled by nRF Connect SDK:

1. **Power Management**: Utilize Zephyr's power management for better battery life
2. **OTA Updates**: Implement MCUboot for over-the-air firmware updates  
3. **Networking**: Add Thread/Matter support for IoT connectivity
4. **Security**: Leverage PSA crypto APIs for enhanced security
5. **Multi-threading**: Use Zephyr threads for more responsive operation

## Support

For issues with the migration:
1. Check Zephyr and nRF Connect SDK documentation
2. Review device tree bindings for your hardware
3. Use nRF Connect SDK forums for community support
4. Check Nordic's DevZone for nRF-specific questions