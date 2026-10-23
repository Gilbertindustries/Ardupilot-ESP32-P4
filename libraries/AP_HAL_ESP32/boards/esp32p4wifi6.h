/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Waveshare ESP32-P4-WIFI6 dev board (Pico form factor)
 *
 *   - ESP32-P4: dual-core RISC-V HP @ 400MHz + LP core, 32MB in-package
 *     octal PSRAM, 32MB NOR flash
 *   - ESP32-C6-MINI-1: WiFi 6 / BLE 5 co-processor on SDIO (the P4 itself
 *     has no radio)
 *   - Type-C port: power + CH343 USB-serial wired to UART0 (console)
 *   - 40-pin (2x20) Pico-style header, TF card slot (SDIO)
 *
 * No flight sensors are onboard, so none are configured yet: this target
 * brings the board up with MAVLink on the Type-C console, and sensors can
 * be wired to the header later (see the commented probe lists below).
 *
 * PINS MARKED TODO MUST BE CHECKED AGAINST THE WAVESHARE SCHEMATIC before
 * attaching hardware. The console UART0 pins (37/38) are the ESP32-P4
 * chip defaults and are safe (ROM download mode depends on them).
 */

#pragma once

#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_ESP32_P4WIFI6

// P4 bring-up marker: shared-library fixes in this fork are scoped to this
// board only. The ESP32-S3 CANBus port and any other ESP32 targets must
// keep stock behaviour, so all such fixes are gated on this define.
#define HAL_ESP32_P4WIFI6_BRINGUP 1

#define HAL_ESP32_BOARD_NAME "esp32p4wifi6"

// make sensor selection clearer (for the probe lists below)
#define PROBE_IMU_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,GET_I2C_DEVICE(bus, addr),##args))
#define PROBE_IMU_SPI(driver, devname, args ...) ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this,hal.spi->get_device(devname),##args))
#define PROBE_BARO_I2C(driver, bus, addr, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(GET_I2C_DEVICE(bus, addr)),##args))
#define PROBE_BARO_SPI(driver, devname, args ...) ADD_BACKEND(AP_Baro_ ## driver::probe(*this,std::move(hal.spi->get_device(devname)),##args))

// No default internal driver; sensors come in via the probe list below.
// (HAL_INS_PROBE_LIST runs independently of this setting.)
#define HAL_INS_DEFAULT HAL_INS_NONE
//#define HAL_INS_DEFAULT HAL_INS_ICM20XXX_I2C
//#define HAL_INS_ICM20XXX_I2C_BUS 0
//#define HAL_INS_ICM20XXX_I2C_ADDR (0x68)

/*
 * Example IMU: BMI160 on I2C bus 0. All knobs are overridable — these are
 * defaults for the current wiring, not hardcodes:
 *   - address: SDO/GND -> 0x68, SDO/VDDIO -> 0x69
 *   - rotation: set to your physical mounting
 *   - the driver polls the FIFO at 1kHz, so no INT wire is required
 *   - the local port forces 400Hz ODR (BMI160_ODR_400HZ in the driver) so
 *     the FIFO never overflows at the 1kHz poll rate on slow I2C
 * Set HAL_INS_BMI160_ENABLED to 0 to go back to a sensor-less board.
 */
#ifndef HAL_INS_BMI160_ENABLED
#define HAL_INS_BMI160_ENABLED 1
#endif
// primary address (SA0/SDO high) and fallback (SA0/SDO low). The sensor on
// this board answers at 0x69, so that is probed first; putting the wrong
// address first just costs 5 softreset attempts and prints a misleading
// "BMI160: failed to init" for the miss before the second probe succeeds.
#ifndef HAL_INS_BMI160_I2C_ADDR
#define HAL_INS_BMI160_I2C_ADDR 0x69
#endif
#ifndef HAL_INS_BMI160_I2C_ADDR_ALT
#define HAL_INS_BMI160_I2C_ADDR_ALT 0x68
#endif
#ifndef HAL_INS_BMI160_ROTATION
#define HAL_INS_BMI160_ROTATION ROTATION_NONE
#endif
#if HAL_INS_BMI160_ENABLED
#define HAL_INS_PROBE_LIST \
	PROBE_IMU_I2C(BMI160, 0, HAL_INS_BMI160_I2C_ADDR, HAL_INS_BMI160_ROTATION); \
	PROBE_IMU_I2C(BMI160, 0, HAL_INS_BMI160_I2C_ADDR_ALT, HAL_INS_BMI160_ROTATION)
#endif
/*
 * Barometer: BMP280 (GY-BMP280 breakout) on I2C bus 0, sharing the bus
 * with the BMI160 - no extra pins needed.
 *   - SCL -> GPIO8, SDA -> GPIO7
 *   - VCC -> 3V3. NOT 5V: the P4 is not 5V tolerant, and on these
 *     breakouts the module's own pull-ups sit on its VCC rail, so a 5V
 *     supply would pull SDA/SCL to 5V.
 *   - CSB -> 3V3 (selects I2C mode; with CSB low the part comes up in SPI
 *     mode, the WHOAMI read at 0xD0 fails and the probe is rejected)
 *   - SDO -> GND gives 0x76, SDO -> VCC gives 0x77. SDO is often left
 *     floating on GY-BMP280, so both addresses are probed; the miss costs
 *     nothing because AP_Baro's GET_I2C_DEVICE guards against re-probing.
 * The driver polls at 50Hz and averages internally, so no INT pin is used.
 * HAL_BARO_ALLOW_INIT_NO_BARO stays set for now so the board still boots
 * while the baro is being brought up.
 */
#ifndef HAL_BARO_BMP280_I2C_ADDR
#define HAL_BARO_BMP280_I2C_ADDR 0x76
#endif
#ifndef HAL_BARO_BMP280_I2C_ADDR2
#define HAL_BARO_BMP280_I2C_ADDR2 0x77
#endif
#define HAL_BARO_PROBE_LIST \
	PROBE_BARO_I2C(BMP280, 0, HAL_BARO_BMP280_I2C_ADDR); \
	PROBE_BARO_I2C(BMP280, 0, HAL_BARO_BMP280_I2C_ADDR2)

// I2C: one hardware-controller bus. The pins below are EXAMPLES (defaults
// for the current wiring: SCL=GPIO8, SDA=GPIO7 on the 40-pin header) —
// override them for your setup, nothing here is hardcoded to one board.
// Note: software (bit-banged) I2C is not available on the P4 port
// (.soft=false always).
//
// GPIO8 caveat: it is the ESP32-P4 SPI-boot strapping pin. The I2C
// pull-ups hold it high at reset = normal flash boot (desired); only a
// RESET pressed while SCL happens to be low would land in download mode.
#ifndef HAL_ESP32_I2C0_SCL_GPIO
#define HAL_ESP32_I2C0_SCL_GPIO GPIO_NUM_8
#endif
#ifndef HAL_ESP32_I2C0_SDA_GPIO
#define HAL_ESP32_I2C0_SDA_GPIO GPIO_NUM_7
#endif

#define HAL_ESP32_I2C_BUSES \
	{.port=I2C_NUM_0, .sda=HAL_ESP32_I2C0_SDA_GPIO, .scl=HAL_ESP32_I2C0_SCL_GPIO, .speed=400*KHZ, .internal=true}

// no SPI devices yet (TODO: define buses/devices when wiring IMU/baro)
#define HAL_ESP32_SPI_BUSES {}
#define HAL_ESP32_SPI_DEVICES {}

// RMT-based RC input (TODO: pick the header pin for your receiver)
#define HAL_ESP32_RMT_RX_PIN_NUMBER 4
//#define HAL_ESP32_RCIN GPIO_NUM_5

// PWM output pins (TODO: verify against schematic; driven via MCPWM + GPIO
// matrix, any output-capable header pin will do).
// NOTE: never place an I2C pin here — RCOutput::init() claims these as
// MCPWM outputs and would hijack the line (GPIO8 is SCL on this board!).
#define HAL_ESP32_RCOUT {GPIO_NUM_2, GPIO_NUM_9, GPIO_NUM_10, GPIO_NUM_11}

// no airspeed sensor
#define AP_AIRSPEED_ENABLED 0
#define AP_AIRSPEED_ANALOG_ENABLED 0
#define AP_AIRSPEED_BACKEND_DEFAULT_ENABLED 0

// allow booting with no barometer
#define HAL_BARO_ALLOW_INIT_NO_BARO 1

// no compass
#define AP_COMPASS_ENABLE_DEFAULT 0
#define ALLOW_ARM_NO_COMPASS

// see boards.py
#ifndef ENABLE_HEAP
#define ENABLE_HEAP 1
#endif

/*
 * WiFi: OPTIONAL and disabled by default.
 * The P4 has no radio; WiFi/BLE is served by the onboard ESP32-C6 over
 * SDIO via Espressif's esp_wifi_remote layer. That component needs
 * ESP-IDF >= 5.4 (this tree now pins v5.5.4 - see
 * Tools/scripts/esp32_get_idf.sh), and the C6 must run matching
 * esp-hosted slave firmware. To try WiFi later: flash the C6 slave
 * firmware, then uncomment below.
 * 1 = TCP (connect to 192.168.4.1 port 5760), 2 = UDP
 */
//#define HAL_ESP32_WIFI 1
#define WIFI_SSID "ardupilot-esp32"
#define WIFI_PWD "ardupilot-esp32"

// serial ports: UART0 is the console/MAVLink on the Type-C CH343
// (chip-default pins). UART1/2 mappings are placeholders on header pins
// (TODO: verify against schematic) for future GPS/telemetry use.
#define HAL_ESP32_UART_DEVICES \
	{.port=UART_NUM_0, .rx=GPIO_NUM_38, .tx=GPIO_NUM_37}, \
	{.port=UART_NUM_1, .rx=GPIO_NUM_3, .tx=GPIO_NUM_4}, \
	{.port=UART_NUM_2, .rx=GPIO_NUM_5, .tx=GPIO_NUM_6}

// only the console (SERIAL0) is used for now; enable more via params
#define DEFAULT_SERIAL2_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL3_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL4_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL5_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL6_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL7_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL8_PROTOCOL SerialProtocol_None
#define DEFAULT_SERIAL9_PROTOCOL SerialProtocol_None

// ADC disabled until the battery-sense pin mapping is verified
#define HAL_DISABLE_ADC_DRIVER 1
#define HAL_USE_ADC 0

// no notification LED configured
#define DEFAULT_NTF_LED_TYPES Notify_LED_None

// TF card slot is on SDMMC (TODO: define SDMMC pins from schematic, then
// uncomment to get microSD logging/params)
//#define HAL_ESP32_SDCARD
//#define HAL_ESP32_SDMMC

// no logging backends until storage is sorted (MAVLink console only)
#define HAL_LOGGING_FILESYSTEM_ENABLED 0
#define HAL_LOGGING_DATAFLASH_ENABLED 0
#define HAL_LOGGING_MAVLINK_ENABLED 0

#define HAL_BOARD_LOG_DIRECTORY "/SDCARD/APM/LOGS"
#define HAL_BOARD_STORAGE_DIRECTORY "/SDCARD/APM/STORAGE"
#define HAL_BOARD_TERRAIN_DIRECTORY "/SDCARD/APM/TERRAIN"

#define HAL_LOGGING_BACKENDS_DEFAULT 1

#define AP_RCPROTOCOL_ENABLED 0
