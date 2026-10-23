/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_Baro_BMP280.h"

#if AP_BARO_BMP280_ENABLED

#include <AP_Math/definitions.h>
#include <utility>

extern const AP_HAL::HAL &hal;

#define BMP280_MODE_SLEEP  0
#define BMP280_MODE_FORCED 1
#define BMP280_MODE_NORMAL 3
#define BMP280_MODE BMP280_MODE_NORMAL

#define BMP280_OVERSAMPLING_1  1
#define BMP280_OVERSAMPLING_2  2
#define BMP280_OVERSAMPLING_4  3
#define BMP280_OVERSAMPLING_8  4
#define BMP280_OVERSAMPLING_16 5
#define BMP280_OVERSAMPLING_P BMP280_OVERSAMPLING_16
#define BMP280_OVERSAMPLING_T BMP280_OVERSAMPLING_2

#define BMP280_FILTER_COEFFICIENT 2

#define BMP280_ID            0x58
#define BME280_ID            0x60

#define BMP280_REG_CALIB     0x88
#define BMP280_REG_ID        0xD0
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_STATUS    0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_DATA      0xF7

#ifdef HAL_ESP32_P4WIFI6_BRINGUP
// ESP32-P4 bring-up diagnostic (local fork): this baro shares I2C bus 0 with
// the BMI160, which polls its FIFO at 1kHz, so register reads can
// occasionally fail. Count them and report once a second from update() (the
// main loop) - that thread is known to print reliably on this port, unlike
// the bus thread whose output gets shredded by MAVLink on the shared UART0.
static uint32_t bmp280_read_errors;
static uint32_t bmp280_samples;
static float    bmp280_last_press;
static uint32_t bmp280_last_errors;
static uint32_t bmp280_last_samples;
static uint32_t bmp280_last_ms;
#endif

AP_Baro_BMP280::AP_Baro_BMP280(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
{
}

AP_Baro_Backend *AP_Baro_BMP280::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }

    AP_Baro_BMP280 *sensor = NEW_NOTHROW AP_Baro_BMP280(baro, std::move(dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_BMP280::_init()
{
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    uint8_t whoami;
    if (!_dev->read_registers(BMP280_REG_ID, &whoami, 1)  ||
        (whoami != BME280_ID && whoami != BMP280_ID)) {
        // not a BMP280 or BME280
        return false;
    }

    // read the calibration data
    uint8_t buf[24];
#ifdef HAL_ESP32_P4WIFI6_BRINGUP
    // Same hazard as in _timer(): the stock code ignores this return value.
    // On a failed read buf[] is uninitialised stack memory, and the garbage
    // coefficients would then persist for the whole session and corrupt
    // EVERY pressure reading. Retry briefly first (the BMI160 is hammering
    // the same bus, so one transient failure at boot is plausible) and only
    // then fail the probe - so an unreliable sensor is reported absent rather
    // than reporting plausible looking rubbish.
    bool calib_ok = false;
    for (uint8_t i = 0; i < 5; i++) {
        if (_dev->read_registers(BMP280_REG_CALIB, buf, sizeof(buf))) {
            calib_ok = true;
            break;
        }
        hal.scheduler->delay(2);
    }
    if (!calib_ok) {
        return false;
    }
#else
    _dev->read_registers(BMP280_REG_CALIB, buf, sizeof(buf));
#endif

    _t1 = ((int16_t)buf[1] << 8) | buf[0];
    _t2 = ((int16_t)buf[3] << 8) | buf[2];
    _t3 = ((int16_t)buf[5] << 8) | buf[4];
    _p1 = ((int16_t)buf[7] << 8) | buf[6];
    _p2 = ((int16_t)buf[9] << 8) | buf[8];
    _p3 = ((int16_t)buf[11] << 8) | buf[10];
    _p4 = ((int16_t)buf[13] << 8) | buf[12];
    _p5 = ((int16_t)buf[15] << 8) | buf[14];
    _p6 = ((int16_t)buf[17] << 8) | buf[16];
    _p7 = ((int16_t)buf[19] << 8) | buf[18];
    _p8 = ((int16_t)buf[21] << 8) | buf[20];
    _p9 = ((int16_t)buf[23] << 8) | buf[22];

    // SPI write needs bit mask
    uint8_t mask = 0xFF;
    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        mask = 0x7F;
    }

    _dev->setup_checked_registers(2, 20);
    
    _dev->write_register((BMP280_REG_CTRL_MEAS & mask), (BMP280_OVERSAMPLING_T << 5) |
                         (BMP280_OVERSAMPLING_P << 2) | BMP280_MODE, true);

    _dev->write_register((BMP280_REG_CONFIG & mask), BMP280_FILTER_COEFFICIENT << 2, true);

    _instance = _frontend.register_sensor();

    _dev->set_device_type(DEVTYPE_BARO_BMP280);
    set_bus_id(_instance, _dev->get_bus_id());
    
    // request 50Hz update
    _dev->register_periodic_callback(20 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Baro_BMP280::_timer, void));

    return true;
}



//  accumulate a new sensor reading
void AP_Baro_BMP280::_timer(void)
{
    uint8_t buf[6];

#ifdef HAL_ESP32_P4WIFI6_BRINGUP
    // ESP32-P4 bring-up (local fork): the stock code ignores this return
    // value and then feeds buf[] straight into the pressure/temperature
    // maths. On a failed read buf[] still holds uninitialised stack memory.
    // That is not contained further down: HAL_BARO_FILTER_DEFAULT is 0, so
    // pressure_ok() has its range check disabled and accepts anything, and
    // update() runs far faster than this 50Hz timer so _pressure_count is
    // usually 1 - meaning ONE bad sample is published directly with no
    // averaging or rejection. Observed symptom: pressure/altitude jumping to
    // nonsense (e.g. 0 <-> 3680m, ~642hPa) for a single loop then snapping
    // back. Skip the sample instead.
    if (!_dev->read_registers(BMP280_REG_DATA, buf, sizeof(buf))) {
        bmp280_read_errors++;
        return;
    }
    bmp280_samples++;
#else
    _dev->read_registers(BMP280_REG_DATA, buf, sizeof(buf));
#endif

    _update_temperature((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
    _update_pressure((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));

    _dev->check_next_register();
}

// transfer data to the frontend
void AP_Baro_BMP280::update(void)
{
    WITH_SEMAPHORE(_sem);

#ifdef HAL_ESP32_P4WIFI6_BRINGUP
    // Report read health once a second. Deliberately placed BEFORE the
    // early-out below, so it still reports when every read is failing - which
    // is precisely the case we need to detect.
    const uint32_t diag_now_ms = AP_HAL::millis();
    if (bmp280_last_ms == 0) {
        // first call: start the window now so the first report covers a full
        // second of real data instead of printing immediately
        bmp280_last_ms = diag_now_ms;
    } else if (diag_now_ms - bmp280_last_ms >= 1000) {
        DEV_PRINTF("BMP280 diag: %u ok/s, %u err/s, press=%.1f hPa, temp=%.1f C\n",
                   (unsigned)(bmp280_samples - bmp280_last_samples),
                   (unsigned)(bmp280_read_errors - bmp280_last_errors),
                   (double)bmp280_last_press,
                   (double)_temperature);
        bmp280_last_samples = bmp280_samples;
        bmp280_last_errors = bmp280_read_errors;
        bmp280_last_ms = diag_now_ms;
    }
#endif

    if (_pressure_count == 0) {
        return;
    }

    const float press = _pressure_sum/_pressure_count;
    _copy_to_frontend(_instance, press, _temperature);
#ifdef HAL_ESP32_P4WIFI6_BRINGUP
    bmp280_last_press = press;
#endif
    _pressure_count = 0;
    _pressure_sum = 0;
}

// calculate temperature
void AP_Baro_BMP280::_update_temperature(int32_t temp_raw)
{
    int32_t var1, var2, t;

    // according to datasheet page 22
    var1 = ((((temp_raw >> 3) - ((int32_t)_t1 << 1))) * ((int32_t)_t2)) >> 11;
    var2 = (((((temp_raw >> 4) - ((int32_t)_t1)) * ((temp_raw >> 4) - ((int32_t)_t1))) >> 12) * ((int32_t)_t3)) >> 14;
    _t_fine = var1 + var2;
    t = (_t_fine * 5 + 128) >> 8;

    const float temp = ((float)t) * 0.01f;

    WITH_SEMAPHORE(_sem);
    
    _temperature = temp;
}

// calculate pressure
void AP_Baro_BMP280::_update_pressure(int32_t press_raw)
{
    int64_t var1, var2, p;

    // according to datasheet page 22
    var1 = ((int64_t)_t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)_p6;
    var2 = var2 + ((var1 * (int64_t)_p5) << 17);
    var2 = var2 + (((int64_t)_p4) << 35);
    var1 = ((var1 * var1 * (int64_t)_p3) >> 8) + ((var1 * (int64_t)_p2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_p1) >> 33;

    if (var1 == 0) {
        return;
    }

    p = 1048576 - press_raw;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)_p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)_p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)_p7) << 4);


    const float press = (float)p / 256.0f;
    if (!pressure_ok(press)) {
        return;
    }
    
    WITH_SEMAPHORE(_sem);
    
    _pressure_sum += press;
    _pressure_count++;
}

#endif  // AP_BARO_BMP280_ENABLED
