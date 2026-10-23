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

#include "CANIface.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Common/ExpandingString.h>
#include <driver/twai.h>
#include <cstring>

using namespace ESP32;

// queue depths for the TWAI driver
#ifndef HAL_ESP32_CAN_TX_QUEUE_LEN
#define HAL_ESP32_CAN_TX_QUEUE_LEN 16
#endif
#define TWAI_RX_QUEUE_LEN 16

// task used to turn TWAI alerts into event semaphore wakeups
#define ALERT_TASK_STACK 4096
#define ALERT_TASK_PRIO  12

// TWAI alerts we act on
#define TWAI_ALERTS_ENABLED (TWAI_ALERT_RX_DATA |         \
                             TWAI_ALERT_TX_SUCCESS |      \
                             TWAI_ALERT_TX_IDLE |         \
                             TWAI_ALERT_RX_QUEUE_FULL |   \
                             TWAI_ALERT_BUS_ERROR |       \
                             TWAI_ALERT_ERR_PASS |        \
                             TWAI_ALERT_ABOVE_ERR_WARN |  \
                             TWAI_ALERT_BELOW_ERR_WARN |  \
                             TWAI_ALERT_BUS_OFF |         \
                             TWAI_ALERT_BUS_RECOVERED)

/*
  map a CAN bitrate to a TWAI timing configuration. Returns false for
  bitrates the TWAI controller cannot produce
 */
static bool bitrate_to_timing(const uint32_t bitrate, twai_timing_config_t &timing)
{
    switch (bitrate) {
    case 1000000:
        timing = TWAI_TIMING_CONFIG_1MBITS();
        return true;
    case 800000:
        timing = TWAI_TIMING_CONFIG_800KBITS();
        return true;
    case 500000:
        timing = TWAI_TIMING_CONFIG_500KBITS();
        return true;
    case 250000:
        timing = TWAI_TIMING_CONFIG_250KBITS();
        return true;
    case 125000:
        timing = TWAI_TIMING_CONFIG_125KBITS();
        return true;
    case 100000:
        timing = TWAI_TIMING_CONFIG_100KBITS();
        return true;
    case 50000:
        timing = TWAI_TIMING_CONFIG_50KBITS();
        return true;
    case 25000:
        timing = TWAI_TIMING_CONFIG_25KBITS();
        return true;
    default:
        return false;
    }
}

CANIface::CANIface(uint8_t self_index) :
    self_index_(self_index)
{
}

bool CANIface::init(const uint32_t bitrate, const OperatingMode mode)
{
    if (initialized_) {
        return true;
    }

    twai_timing_config_t timing_config;
    if (!bitrate_to_timing(bitrate, timing_config)) {
        return false;
    }

    twai_mode_t twai_mode = TWAI_MODE_NORMAL;
    if (mode == SilentMode) {
        twai_mode = TWAI_MODE_LISTEN_ONLY;
    }

    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(
        HAL_ESP32_CAN_TX_PIN, HAL_ESP32_CAN_RX_PIN, twai_mode);
    general_config.tx_queue_len = HAL_ESP32_CAN_TX_QUEUE_LEN;
    general_config.rx_queue_len = TWAI_RX_QUEUE_LEN;
    general_config.alerts_enabled = TWAI_ALERTS_ENABLED;

    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&general_config, &timing_config, &filter_config) != ESP_OK) {
        return false;
    }
    if (twai_start() != ESP_OK) {
        twai_driver_uninstall();
        return false;
    }

    bitrate_ = bitrate;
    mode_ = mode;
    bus_off_ = false;
    initialized_ = true;

    // a task converts TWAI alerts into wakeups of the driver event semaphore
    if (alert_task_handle == nullptr &&
        xTaskCreate(alert_task_trampoline, "can_twai", ALERT_TASK_STACK,
                    this, ALERT_TASK_PRIO, &alert_task_handle) != pdPASS) {
        alert_task_handle = nullptr;
    }

    return true;
}

int16_t CANIface::send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags)
{
    if (!initialized_ || frame.isErrorFrame() ||
        frame.dlc > AP_HAL::CANFrame::NonFDCANMaxDataLen) {
        return -1;
    }

    {
        WITH_SEMAPHORE(sem);
        stats.tx_requests++;
    }

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    if (frame.isExtended()) {
        msg.extd = 1;
        msg.identifier = frame.id & AP_HAL::CANFrame::MaskExtID;
    } else {
        msg.identifier = frame.id & AP_HAL::CANFrame::MaskStdID;
    }
    if (frame.isRemoteTransmissionRequest()) {
        msg.rtr = 1;
    }
    msg.data_length_code = frame.dlc;
    if (!msg.rtr) {
        memcpy(msg.data, frame.data, frame.dlc);
    }

    const esp_err_t ret = twai_transmit(&msg, 0);
    if (ret == ESP_ERR_TIMEOUT) {
        // the hardware tx queue is full; the caller retries and is woken
        // by a TX alert as the queue drains. We deliberately do not cache
        // the full state as it can race with a concurrent queue drain
        WITH_SEMAPHORE(sem);
        stats.tx_overflow++;
        return 0;
    }
    if (ret != ESP_OK) {
        // driver not running, bus-off or bad argument
        WITH_SEMAPHORE(sem);
        stats.tx_rejected++;
        return -1;
    }

    {
        WITH_SEMAPHORE(sem);
        stats.tx_success++;
        stats.last_transmit_us = AP_HAL::micros64();
    }

    // notify registered callbacks (MAVLink frame forwarding/logging)
    AP_HAL::CANIface::send(frame, tx_deadline, flags);

    return 1;
}

int16_t CANIface::receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us, CanIOFlags& out_flags)
{
    poll_read();
    {
        WITH_SEMAPHORE(sem);
        if (rx_queue.is_empty()) {
            return 0;
        }
        const CanRxItem &rx = *rx_queue[0];
        out_frame = rx.frame;
        out_timestamp_us = rx.timestamp_us;
        out_flags = rx.flags;
        IGNORE_RETURN(rx_queue.pop());
    }

    return AP_HAL::CANIface::receive(out_frame, out_timestamp_us, out_flags);
}

bool CANIface::select(bool &read_select, bool &write_select,
                      const AP_HAL::CANFrame* const pending_tx, uint64_t blocking_deadline)
{
    if (!initialized_) {
        read_select = false;
        write_select = false;
        return false;
    }

    // move any frames waiting in the hardware into our queue
    poll_read();

    const uint64_t now_us = AP_HAL::micros64();
    const bool need_block =
        (read_select && rx_queue.is_empty()) ||
        (write_select && !tx_space());

    if (need_block && event_sem != nullptr && blocking_deadline > now_us) {
        IGNORE_RETURN(event_sem->wait(uint32_t(blocking_deadline - now_us)));
        poll_read();
    }

    read_select = !rx_queue.is_empty();
    write_select = tx_space();
    return true;
}

bool CANIface::set_event_handle(AP_HAL::BinarySemaphore *sem_handle)
{
    event_sem = sem_handle;
    return true;
}

bool CANIface::add_to_rx_queue(const CanRxItem &rx_item)
{
    WITH_SEMAPHORE(sem);
    return rx_queue.push(rx_item);
}

void CANIface::signal_event(void)
{
    AP_HAL::BinarySemaphore *sem_ptr = event_sem;
    if (sem_ptr != nullptr) {
        sem_ptr->signal();
    }
}

void CANIface::poll_read(void)
{
    for (;;) {
        twai_message_t msg;
        memset(&msg, 0, sizeof(msg));
        if (twai_receive(&msg, 0) != ESP_OK) {
            break;
        }
        CanRxItem item {};
        item.timestamp_us = AP_HAL::micros64();
        item.flags = 0;
        uint32_t id = msg.identifier;
        if (msg.extd) {
            id |= AP_HAL::CANFrame::FlagEFF;
        }
        if (msg.rtr) {
            id |= AP_HAL::CANFrame::FlagRTR;
        }
        item.frame.id = id;
        item.frame.dlc = msg.data_length_code > AP_HAL::CANFrame::NonFDCANMaxDataLen ?
                         AP_HAL::CANFrame::NonFDCANMaxDataLen : msg.data_length_code;
        if (!msg.rtr) {
            memcpy(item.frame.data, msg.data, item.frame.dlc);
        }
        WITH_SEMAPHORE(sem);
        if (rx_queue.push(item)) {
            stats.rx_received++;
        } else {
            stats.rx_overflow++;
        }
    }
}

void CANIface::alert_task_trampoline(void *arg)
{
    static_cast<CANIface *>(arg)->alert_task();
}

void CANIface::alert_task(void)
{
    for (;;) {
        uint32_t alerts = 0;
        if (twai_read_alerts(&alerts, portMAX_DELAY) != ESP_OK) {
            // driver not installed (yet)
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        bool wake = false;

        if (alerts & TWAI_ALERT_RX_DATA) {
            poll_read();
            wake = true;
        }
        if (alerts & (TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_IDLE)) {
            // a frame left the tx queue, wake anyone retrying a send
            wake = true;
        }
        if (alerts & (TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS)) {
            error_count_++;
        }
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
            WITH_SEMAPHORE(sem);
            stats.rx_overflow++;
        }
        if (alerts & TWAI_ALERT_BUS_OFF) {
            bus_off_ = true;
            {
                WITH_SEMAPHORE(sem);
                stats.num_busoff_err++;
            }
            // start the bus-off recovery sequence, completion is
            // reported by TWAI_ALERT_BUS_RECOVERED
            twai_initiate_recovery();
            wake = true;
        }
        if (alerts & TWAI_ALERT_BUS_RECOVERED) {
            bus_off_ = false;
            if (twai_start() == ESP_OK) {
                wake = true;
            }
        }

        if (wake) {
            signal_event();
        }
    }
}

void CANIface::get_stats(ExpandingString &str)
{
    WITH_SEMAPHORE(sem);
    str.printf("tx_requests:    %u\n"
               "tx_rejected:    %u\n"
               "tx_overflow:    %u\n"
               "tx_success:     %u\n"
               "rx_received:    %u\n"
               "rx_overflow:    %u\n"
               "bus_off_err:    %u\n"
               "errors:         %u\n",
               unsigned(stats.tx_requests),
               unsigned(stats.tx_rejected),
               unsigned(stats.tx_overflow),
               unsigned(stats.tx_success),
               unsigned(stats.rx_received),
               unsigned(stats.rx_overflow),
               unsigned(stats.num_busoff_err),
               unsigned(error_count_));
}

void CANIface::flush_tx(void)
{
    if (initialized_) {
        (void)twai_clear_transmit_queue();
    }
}

void CANIface::clear_rx(void)
{
    if (initialized_) {
        (void)twai_clear_receive_queue();
    }
    WITH_SEMAPHORE(sem);
    rx_queue.clear();
}

#endif // HAL_NUM_CAN_IFACES