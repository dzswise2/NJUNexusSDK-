/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/src/adapters/device_adapter.cpp
 * @Description: 统一设备适配器基类实现，提供设备初始化、通信、状态监控等基础功能
 *
 * Copyright (c) 2025 by Pegasus, All Rights Reserved.
 */

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "teleop_adapter/adapters/device_adapter.hpp"
#include <queue>
#include <deque>
#include <unordered_set>
#include <iostream>
#include "nexus_log.hpp"

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

/*******************************************************************************
 * MonitorImpl — hidden monitoring state
 ******************************************************************************/

struct DeviceAdapter::MonitorImpl {
    std::thread monitor_thread;
    std::atomic<bool> monitor_thread_running{false};

    mutable std::mutex seq_queue_mutex;
    std::queue<uint32_t> seq_queue;

    int consecutive_received_count{0};
    int consecutive_missing_count{0};

    uint64_t first_ext_sequence{0};
    uint64_t last_ext_sequence{0};
    bool has_first_ext_sequence{false};
    uint64_t unique_received_frame_count{0};
    int startup_grace_count{0};

    std::unordered_set<uint32_t> seq_seen_set;
    std::deque<uint32_t> seq_seen_deque;
    std::atomic<bool> stats_external{false};
};

/*******************************************************************************
 * Constructor & Destructor
 ******************************************************************************/

DeviceAdapter::DeviceAdapter(const DeviceAdapterConfig& config,
                             std::unique_ptr<CommunicationInterface> comm_interface)
        : config_(config),
            comm_interface_(std::move(comm_interface)),
                thread_running_(false),
                monitor_impl_(std::make_unique<MonitorImpl>()){
    
    device_status_.connected = 0;
    device_status_.status = 4;
    device_status_.initialized = false;
    device_status_.total_packet_count = 0;
    device_status_.total_packet_loss_count = 0;
    device_status_.overall_packet_loss_rate = 0.0f;
    device_status_.motors_enabled = false;
    device_status_.error_code = 0;
    device_status_.error_message = "";

    monitor_impl_->monitor_thread_running = true;
    monitor_impl_->monitor_thread = std::thread(&DeviceAdapter::monitorThreadFunc, this);
}

DeviceAdapter::~DeviceAdapter() {
    shutdown();
}

/*******************************************************************************
 * Public Methods
 ******************************************************************************/

bool DeviceAdapter::initialize() {
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        if (device_status_.initialized == true) {
            return true;
        }
    }
    
    if (comm_interface_) {
        if (!comm_interface_->isOpen()) {
            if (!comm_interface_->open()) {
                std::lock_guard<std::mutex> lock(status_mutex_);
                setError(2, "Failed to open communication: " + comm_interface_->getLastError());
                return false;
            }
        }
    }

    thread_running_ = true;
    read_thread_ = std::thread(&DeviceAdapter::readThreadFunc, this);
    write_thread_ = std::thread(&DeviceAdapter::writeThreadFunc, this);

    if (!monitor_impl_->monitor_thread_running) {
        monitor_impl_->monitor_thread_running = true;
        monitor_impl_->monitor_thread = std::thread(&DeviceAdapter::monitorThreadFunc, this);
    }
    
    if (!deviceHandshake()) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        setError(3, "Device handshake failed");
        if (comm_interface_) {
            comm_interface_->close();
        }
        NEXUS_COUT << "Handshake failed" << std::endl;
        return false;
    }
    
    if (!configureDevice()) {
        std::lock_guard<std::mutex> lock(status_mutex_);
        setError(4, "Device configuration failed");
        if (comm_interface_) {
            comm_interface_->close();
        }
        return false;
    }
    
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        device_status_.initialized = true;
        device_status_.status = 0;
        clearError();
    }
    
    if (config_.auto_enable) {
        if (!enableMotors()) {
            std::lock_guard<std::mutex> lock(status_mutex_);
            setError(5, "Failed to auto-enable motors");
        }
    }
    
    return true;
}

void DeviceAdapter::shutdown() {
    if (thread_running_) {
        thread_running_ = false;
        
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        
        if (write_thread_.joinable()) {
            write_thread_.join();
        }
    }
    if (monitor_impl_ && monitor_impl_->monitor_thread_running) {
        monitor_impl_->monitor_thread_running = false;
        if (monitor_impl_->monitor_thread.joinable()) {
            monitor_impl_->monitor_thread.join();
        }
    }
    
    if (device_status_.motors_enabled) {
        disableMotors();
    }
    
    if (comm_interface_ && comm_interface_->isOpen()) {
        comm_interface_->close();
    }
    
    std::lock_guard<std::mutex> lock(status_mutex_);
    device_status_.connected = false;
    device_status_.initialized = false;
    device_status_.motors_enabled = false;
    device_status_.status = 3;
}

bool DeviceAdapter::isConnected() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (!comm_interface_) {
        return device_status_.connected;
    }
    return device_status_.connected && comm_interface_->isOpen();
}

bool DeviceAdapter::readSensorData(SensorData& sensor_data) {
    (void)sensor_data;
    return false;
}

bool DeviceAdapter::readExternalWrench(std::array<double, 6>& wrench) {
    (void)wrench;
    return false;
}

DeviceStatus DeviceAdapter::getDeviceStatus() const {
    std::lock_guard<std::mutex> lock(status_mutex_);
    return device_status_;
}

DeviceAdapterConfig DeviceAdapter::getConfig() const {
    return config_;
}

bool DeviceAdapter::clearErrors() {
    std::lock_guard<std::mutex> lock(status_mutex_);
    clearError();
    return true;
}

int DeviceAdapter::getNumDofs() const {
    return config_.num_of_dofs;
}

std::string DeviceAdapter::getDeviceName() const {
    return config_.device_name;
}

/*******************************************************************************
 * Protected Methods
 ******************************************************************************/

void DeviceAdapter::setDeviceStatus(const DeviceStatus& status) {
    std::lock_guard<std::mutex> lock(status_mutex_);
    device_status_ = status;
}

void DeviceAdapter::setError(int error_code, const std::string& error_message) {
    device_status_.error_code = error_code;
    device_status_.error_message = error_message;
    device_status_.status = 2;
}

void DeviceAdapter::clearError() {
    device_status_.error_code = 0;
    device_status_.error_message = "";
}

void DeviceAdapter::notifyReceivedFrame(uint32_t frame_seq) {
    {
        std::lock_guard<std::mutex> lock(monitor_impl_->seq_queue_mutex);
        monitor_impl_->seq_queue.push(frame_seq);
    }
}

void DeviceAdapter::enableExternalStats(bool enabled) {
    monitor_impl_->stats_external = enabled;
}

/*******************************************************************************
 * Thread Functions
 ******************************************************************************/

void DeviceAdapter::monitorThreadFunc() {
    auto& m = *monitor_impl_;
    m.monitor_thread_running = true;
    while (m.monitor_thread_running) {
        int rate = config_.feedback_rate > 0 ? config_.feedback_rate : 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / rate));

        int received_count = 0;
        uint32_t seq = 0;
        uint32_t max_seq_in_period = 0;
        uint32_t min_seq_in_period = 0;
        bool have_seq = false;
        {
            std::lock_guard<std::mutex> lock(m.seq_queue_mutex);
            while (!m.seq_queue.empty()) {
                seq = m.seq_queue.front();
                m.seq_queue.pop();

                if (!have_seq) {
                    have_seq = true;
                    min_seq_in_period = max_seq_in_period = seq;
                } else {
                    if (seq > max_seq_in_period) max_seq_in_period = seq;
                    if (seq < min_seq_in_period) min_seq_in_period = seq;
                }

                bool is_new = false;
                if (m.seq_seen_set.find(seq) == m.seq_seen_set.end()) {
                    is_new = true;
                    m.seq_seen_set.insert(seq);
                    m.seq_seen_deque.push_back(seq);
                    while ((int)m.seq_seen_deque.size() > config_.seq_window_size) {
                        uint32_t old = m.seq_seen_deque.front();
                        m.seq_seen_deque.pop_front();
                        m.seq_seen_set.erase(old);
                    }
                }

                if (!is_new) {
                    continue;
                }

                ++received_count;
                ++m.unique_received_frame_count;

                if (!m.has_first_ext_sequence) {
                    m.has_first_ext_sequence = true;
                    m.first_ext_sequence = static_cast<uint64_t>(seq);
                    m.last_ext_sequence = static_cast<uint64_t>(seq);
                } else {
                    int bits = config_.seq_num_bits;
                    if (bits <= 0) bits = 32;
                    if (bits > 32) bits = 32;
                    uint64_t mod = (1ULL << bits);
                    uint64_t last_mod = m.last_ext_sequence & (mod - 1);
                    uint64_t cur_mod = static_cast<uint64_t>(seq) & (mod - 1);
                    uint64_t delta = (cur_mod + mod - last_mod) % mod;

                    uint64_t candidate_ext = 0;
                    if (delta == 0) {
                        candidate_ext = m.last_ext_sequence;
                    } else if (delta <= (mod / 2)) {
                        candidate_ext = m.last_ext_sequence + delta;
                    } else {
                        candidate_ext = m.last_ext_sequence + 1;
                    }

                    if (candidate_ext > m.last_ext_sequence) {
                        m.last_ext_sequence = candidate_ext;
                    }
                }
            }
        }

        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            if (received_count > 0) {
                m.consecutive_received_count++;
                m.consecutive_missing_count = 0;
            } else {
                m.consecutive_received_count = 0;
                m.consecutive_missing_count++;
            }

            if (!m.stats_external) {
                uint64_t total_sent = 0;
                uint64_t total_received = m.unique_received_frame_count;
                if (m.has_first_ext_sequence) {
                    total_sent = (m.last_ext_sequence - m.first_ext_sequence) + 1;
                }
                uint64_t total_loss = 0;
                if (total_sent > total_received) {
                    total_loss = total_sent - total_received;
                } else {
                    total_loss = 0;
                }

                device_status_.total_packet_count = total_sent;
                device_status_.total_packet_loss_count = total_loss;

                if (total_sent > 0) {
                    device_status_.overall_packet_loss_rate = static_cast<float>(device_status_.total_packet_loss_count) /
                        static_cast<float>(device_status_.total_packet_count);
                } else {
                    device_status_.overall_packet_loss_rate = 0.0f;
                }
            }

            if (m.consecutive_received_count >= 10) {
                if (!device_status_.connected) {
                    device_status_.connected = 1;
                    clearError();
                    if (device_status_.initialized == true) {
                        device_status_.status = 0;
                    }
                }
            }

            if (m.consecutive_missing_count >= 10) {
                if (device_status_.connected) {
                    device_status_.connected = 0;
                    setError(101, "Connection lost: no frames for 10 cycles");
                    device_status_.status = 3; // OFFLINE
                }
            }
        }
    }
}

} // namespace teleop_adapter
