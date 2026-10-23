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
#pragma once

#include <AP_HAL/AP_HAL.h>
#include "HAL_ESP32_Namespace.h"

#if HAL_NUM_CAN_IFACES > 0

#include <AP_HAL/CANIface.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_HAL_ESP32/Semaphores.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// the board config must tell us which GPIOs the CAN transceiver is on
#ifndef HAL_ESP32_CAN_TX_PIN
#error "HAL_NUM_CAN_IFACES > 0 requires HAL_ESP32_CAN_TX_PIN in the board header"
#endif
#ifndef HAL_ESP32_CAN_RX_PIN
#error "HAL_NUM_CAN_IFACES > 0 requires HAL_ESP32_CAN_RX_PIN in the board header"
#endif

// depth of the software rx queue drained from the TWAI driver
#ifndef HAL_ESP32_CAN_RX_QUEUE_LEN
#define HAL_ESP32_CAN_RX_QUEUE_LEN 64
#endif

/*
  CAN interface for the ESP32 family using the TWAI (Two-Wire Automotive
  Interface) controller. The ESP32/S2/S3 have a single TWAI controller,
  therefore only one CAN interface is supported.
 */
class ESP32::CANIface : public AP_HAL::CANIface
{
public:
    CANIface() : CANIface(0) {}
    explicit CANIface(uint8_t self_index);

    // initialise the TWAI peripheral. Only classic CAN is supported by
    // TWAI so fdbitrate is ignored
    bool init(const uint32_t bitrate, const OperatingMode mode) override;
    bool init(const uint32_t bitrate, const uint32_t fdbitrate, const OperatingMode mode) override
    {
        return init(bitrate, mode);
    }

    // queue a frame for transmission, negative on error, 0 if no space, 1 on success
    int16_t send(const AP_HAL::CANFrame& frame, uint64_t tx_deadline, CanIOFlags flags) override;

    // pop a received frame, negative on error, 0 if queue empty, 1 on success
    int16_t receive(AP_HAL::CANFrame& out_frame, uint64_t& out_timestamp_us, CanIOFlags& out_flags) override;

    // update read_select/write_select to show readiness. Blocks on the
    // event semaphore until blocking_deadline if an event is being waited on
    bool select(bool &read_select, bool &write_select,
                const AP_HAL::CANFrame* const pending_tx, uint64_t blocking_deadline) override;

    bool set_event_handle(AP_HAL::BinarySemaphore *sem_handle) override;

    bool is_initialized() const override
    {
        return initialized_;
    }
    bool is_busoff() const override
    {
        return bus_off_;
    }
    uint32_t getErrorCount() const override
    {
        return error_count_;
    }

    void get_stats(ExpandingString &str) override;
    const bus_stats_t *get_statistics() const override
    {
        return &stats;
    }

    void flush_tx() override;
    void clear_rx() override;

protected:
    int8_t get_iface_num() const override
    {
        return int8_t(self_index_);
    }
    bool add_to_rx_queue(const CanRxItem &rx_item) override;

private:
    void alert_task(void);
    static void alert_task_trampoline(void *arg);
    // move frames from the TWAI driver rx queue into our software queue
    void poll_read(void);
    void signal_event(void);
    // note we don't try to cache tx queue fullness here; twai_transmit()
    // is the ground truth for space, which avoids a stale-flag race between
    // a full-queue timeout and a concurrent queue drain
    bool tx_space(void) const
    {
        return initialized_ && !bus_off_;
    }

    const uint8_t self_index_;
    bool initialized_ = false;

    // state shared between threads
    volatile bool bus_off_ = false;
    volatile uint32_t error_count_ = 0;

    AP_HAL::BinarySemaphore *event_sem = nullptr;

    ObjectArray<CanRxItem> rx_queue{HAL_ESP32_CAN_RX_QUEUE_LEN};
    HAL_Semaphore sem;              // protects rx_queue and stats
    bus_stats_t stats{};

    TaskHandle_t alert_task_handle = nullptr;
};

// the ESP32 family has a single TWAI (CAN) controller
static_assert(HAL_NUM_CAN_IFACES == 1, "ESP32 supports exactly one CAN interface");

// used by AP_CANManager to allocate an interface if the HAL didn't provide one
typedef ESP32::CANIface HAL_CANIface;

#endif // HAL_NUM_CAN_IFACES