/*
 * @Description: Internal implementation details for TeleopArmAdapter (not for distribution).
 */

#ifndef TELEOP_ADAPTER_ADAPTERS_DETAIL_TELEOP_ARM_ADAPTER_IMPL_HPP
#define TELEOP_ADAPTER_ADAPTERS_DETAIL_TELEOP_ARM_ADAPTER_IMPL_HPP

#include "teleop_adapter/adapters/teleop_arm_adapter.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <queue>
#include <vector>

namespace teleop_adapter {

static constexpr uint8_t PROTOCOL_HEADER = 0xA5;
static constexpr uint8_t CMD_MIT_CONTROL = 0x4A;
static constexpr uint8_t CMD_MIT_FEEDBACK = 0x4D;
static constexpr uint8_t CMD_ZERO_SET = 0x5A;
static constexpr uint8_t CMD_ZERO_RESPONSE = 0x5D;
static constexpr uint8_t CMD_FREQ_SET = 0x6A;
static constexpr uint8_t CMD_FREQ_RESPONSE = 0x6D;
static constexpr uint8_t CMD_ERROR_CLEAR = 0x7A;
static constexpr uint8_t CMD_ERROR_RESPONSE = 0x7D;
static constexpr uint8_t CMD_PACKET_QUERY = 0x8A;
static constexpr uint8_t CMD_PACKET_RESPONSE = 0x8D;
static constexpr uint8_t CMD_PERIPHERAL_SET = 0x9A;
static constexpr uint8_t CMD_PERIPHERAL_RESPONSE = 0x9D;

static constexpr uint8_t MOTOR_STATUS_DISABLE = 0;
static constexpr uint8_t MOTOR_STATUS_ENABLE = 1;
static constexpr uint8_t MOTOR_STATUS_OFFLINE = 2;

static constexpr float POS_MIN = -12.5f;
static constexpr float POS_MAX = 12.5f;
static constexpr float VEL_MIN = -45.0f;
static constexpr float VEL_MAX = 45.0f;
static constexpr float TORQUE_MIN = -30.0f;
static constexpr float TORQUE_MAX = 30.0f;
static constexpr float KP_MIN = 0.0f;
static constexpr float KP_MAX = 500.0f;
static constexpr float KD_MIN = 0.0f;
static constexpr float KD_MAX = 5.0f;

struct MotorControlData {
    uint8_t   mode;
    uint16_t  target_pos;
    uint16_t  target_vel;
    uint16_t  kp;
    uint16_t  kd;
    uint16_t  target_torque;
} __attribute__((packed));

struct ProtocolHeader {
    uint8_t head;
    uint16_t seq_num;
    uint8_t cmd;
    uint8_t length;
} __attribute__((packed));

enum class SignalFrameType {
    ZERO_SET,
    FREQ_SET,
    ERROR_CLEAR,
    PACKET_QUERY,
    PERIPHERAL_SET,
};

struct SignalFrame {
    SignalFrameType type;
    std::vector<uint8_t> data;
    std::chrono::steady_clock::time_point send_time;
    int retry_count;
};

struct TeleopArmAdapter::Impl {
    explicit Impl(TeleopArmAdapter* owner);

    TeleopArmAdapter* owner_;

    uint16_t floatToUint(float value, float min_val, float max_val, int bits) const;
    float uintToFloat(uint16_t value, float min_val, float max_val, int bits) const;
    uint8_t calculateChecksum(const uint8_t* data, size_t length) const;
    std::vector<uint8_t> encodeMitControlFrame(const std::vector<MotorCommand>& commands, uint8_t mode);
    bool decodeMitFeedbackFrame(const uint8_t* data, size_t length, std::vector<MotorState>& states);
    std::vector<uint8_t> encodeSignalFrame(uint8_t cmd, const std::vector<uint8_t>& payload);
    std::vector<uint8_t> encodeSignalFrame(SignalFrameType type, const std::vector<uint8_t>& payload);
    bool decodeSignalResponse(uint8_t expected_cmd, const uint8_t* data, size_t length, std::vector<uint8_t>& payload);
    bool sendSignalFrameAndWaitResponse(SignalFrameType type, const std::vector<uint8_t>& payload,
                                      std::vector<uint8_t>& response, int timeout_ms);
    void processReceivedFrame(const uint8_t* data, size_t length);
    bool validateFrame(const uint8_t* data, size_t length);
    bool canSendControlFrame() const;
    void updateCommCycle();
    void processSignalQueue();

    bool deviceHandshake();
    bool configureDevice();
    bool enableMotors();
    bool disableMotors();
    bool sendMotorCommands(const std::vector<MotorCommand>& commands, uint8_t mode);
    bool readMotorStates(std::vector<MotorState>& states);
    void readThreadFunc();
    void writeThreadFunc();
    bool setMotorZero(int timeout_ms);
    bool clearDeviceErrors(int timeout_ms);
    bool queryPacketLoss(std::vector<float>& packet_loss_rates, int timeout_ms);
    bool setCommFrequency(int frequency_hz, int timeout_ms);
    bool setPeripheralState(uint8_t led_control, int timeout_ms);
    bool getPeripheralState(PeripheralState& peripheral_state) const;
    std::vector<MotorFeedbackData> getMotorFeedback() const;
    bool hasErrors() const;

    std::atomic<uint16_t> sequence_number_;
    std::vector<MotorState> latest_motor_states_;
    mutable std::mutex motor_states_mutex_;
    std::chrono::steady_clock::time_point last_feedback_time_;
    std::vector<MotorFeedbackData> motor_feedback_;
    PeripheralState latest_peripheral_state_;
    mutable std::mutex peripheral_state_mutex_;
    std::atomic<uint32_t> device_errors_;
    std::vector<uint8_t> last_control_frame_;
    mutable std::mutex control_frame_mutex_;
    uint8_t current_motor_mode_;
    std::queue<SignalFrame> signal_queue_;
    mutable std::mutex signal_queue_mutex_;
    std::condition_variable signal_cv_;
    mutable std::mutex response_mutex_;
    std::condition_variable response_cv_;
    std::vector<uint8_t> pending_response_;
    uint8_t expected_response_cmd_;
    bool response_received_;
    std::chrono::steady_clock::time_point last_control_send_time_;
    std::chrono::steady_clock::time_point cycle_start_time_;
    std::atomic<uint32_t> checksum_error_count_;
    std::atomic<uint32_t> timeout_count_;
};

} // namespace teleop_adapter

#endif
