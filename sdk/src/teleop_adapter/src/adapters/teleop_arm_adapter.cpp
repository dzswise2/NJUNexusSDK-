/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/src/adapters/teleop_arm_adapter.cpp
 * @Description: Teleop Arm 机械臂设备适配器实现，包括MIT控制、信令帧处理、协议编解码
 *
 * Copyright (c) 2025 by Pegasus, All Rights Reserved.
 */

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "teleop_adapter/adapters/teleop_arm_adapter.hpp"
#include "teleop_adapter/adapters/detail/teleop_arm_adapter_impl.hpp"
#include "teleop_adapter/utils/endian_utils.hpp"
#include <algorithm>
#include <cstring>
#include <thread>
#include <chrono>
#include <cstdint>
#include <iostream>
#include "nexus_log.hpp"
#include <iomanip>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

/*******************************************************************************
 * Constructor & Destructor
 ******************************************************************************/

/**
 * @brief  构造函数
 * @param  config 设备适配器配置
 * @param  comm_interface 通信接口
 * @retval null
 */
TeleopArmAdapter::TeleopArmAdapter(const DeviceAdapterConfig& config,
                                 std::unique_ptr<CommunicationInterface> comm_interface)
    : DeviceAdapter(config, std::move(comm_interface)),
      impl_(std::make_unique<Impl>(this))
{
}

TeleopArmAdapter::Impl::Impl(TeleopArmAdapter* owner)
    : owner_(owner),
      sequence_number_(0),
      last_feedback_time_(std::chrono::steady_clock::now()),
      expected_response_cmd_(0),
      response_received_(false),
      last_control_send_time_(std::chrono::steady_clock::now()),
      cycle_start_time_(std::chrono::steady_clock::now()),
      current_motor_mode_(MOTOR_MODE_ENABLE),
      checksum_error_count_(0),
      timeout_count_(0)
{
    latest_motor_states_.resize(owner_->getNumDofs());
    for (auto& state : latest_motor_states_) {
        state.position = 0.0;
        state.velocity = 0.0;
        state.effort = 0.0;
        state.enabled = false;
        state.error_code = 0;
    }

    latest_peripheral_state_.led_status = 0;
    latest_peripheral_state_.button_status = 0;
    latest_peripheral_state_.joystick_x = 2048;
    latest_peripheral_state_.joystick_y = 2048;
    latest_peripheral_state_.timestamp = std::chrono::steady_clock::time_point();
}

/**
 * @brief  析构函数
 * @param  null
 * @retval null
 */
TeleopArmAdapter::~TeleopArmAdapter() {
}

bool TeleopArmAdapter::deviceHandshake() {
    return impl_->deviceHandshake();
}

bool TeleopArmAdapter::configureDevice() {
    return impl_->configureDevice();
}

bool TeleopArmAdapter::enableMotors() {
    return impl_->enableMotors();
}

bool TeleopArmAdapter::disableMotors() {
    return impl_->disableMotors();
}

bool TeleopArmAdapter::sendMotorCommands(const std::vector<MotorCommand>& commands, uint8_t mode) {
    return impl_->sendMotorCommands(commands, mode);
}

bool TeleopArmAdapter::sendMotorCommands(const std::vector<MotorCommand>& commands) {
    return sendMotorCommands(commands, impl_->current_motor_mode_);
}

bool TeleopArmAdapter::readMotorStates(std::vector<MotorState>& states) {
    return impl_->readMotorStates(states);
}

void TeleopArmAdapter::readThreadFunc() {
    impl_->readThreadFunc();
}

void TeleopArmAdapter::writeThreadFunc() {
    impl_->writeThreadFunc();
}

bool TeleopArmAdapter::setMotorZero(int timeout_ms) {
    return impl_->setMotorZero(timeout_ms);
}

bool TeleopArmAdapter::clearDeviceErrors(int timeout_ms) {
    return impl_->clearDeviceErrors(timeout_ms);
}

bool TeleopArmAdapter::queryPacketLoss(std::vector<float>& packet_loss_rates, int timeout_ms) {
    return impl_->queryPacketLoss(packet_loss_rates, timeout_ms);
}

bool TeleopArmAdapter::setCommFrequency(int frequency_hz, int timeout_ms) {
    return impl_->setCommFrequency(frequency_hz, timeout_ms);
}

bool TeleopArmAdapter::setPeripheralState(uint8_t led_control, int timeout_ms) {
    return impl_->setPeripheralState(led_control, timeout_ms);
}

bool TeleopArmAdapter::getPeripheralState(PeripheralState& peripheral_state) const {
    return impl_->getPeripheralState(peripheral_state);
}

std::vector<MotorFeedbackData> TeleopArmAdapter::getMotorFeedback() const {
    return impl_->getMotorFeedback();
}

bool TeleopArmAdapter::hasErrors() const {
    return impl_->hasErrors();
}

/*******************************************************************************
 * Public Methods - DeviceAdapter Interface
 ******************************************************************************/

/**
 * @brief  设备握手
 * @param  null
 * @retval 握手成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::deviceHandshake() {   
    // 检查是否能收到反馈数据
    auto start_time = std::chrono::steady_clock::now();
    auto timeout = std::chrono::milliseconds(2000);
    
    while (std::chrono::steady_clock::now() - start_time < timeout) {
        {
            std::lock_guard<std::mutex> lock(owner_->status_mutex_);
            if (owner_->device_status_.connected == 1) {
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    return false;  // 握手超时
}

/**
 * @brief  配置设备
 * @param  null
 * @retval 配置成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::configureDevice() {
    // 1. 设置通讯频率
    if (!setCommFrequency(owner_->getConfig().feedback_rate, 1000)) {
        return false;
    }
    
    // 2. 清除设备错误
    // if (!clearDeviceErrors()) {
    //     return false;
    // }
    
    // 3. 如果需要，设置零位（这里跳过，因为通常在上层应用中手动调用
    
    return true;
}

/**
 * @brief  使能电机
 * @param  null
 * @retval 成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::enableMotors() {
    // Teleop Arm的电机使能是通过MIT控制帧中的mode字段控制
    std::vector<MotorCommand> enable_commands(owner_->getNumDofs());
    
    for (int i = 0; i < owner_->getNumDofs(); ++i) {
        enable_commands[i].kp = 0.0f;       // 使能时刚度设为0
        enable_commands[i].position = 0.0f;  // 当前位置
        enable_commands[i].kd = 0.0f;       // 阻尼设为0
        enable_commands[i].velocity = 0.0f;  // 速度为0
        enable_commands[i].torque = 0.0f;   // 力矩为0
    }
    
    // 通过sendMotorCommands发送使能命令
    if (!sendMotorCommands(enable_commands, MOTOR_MODE_ENABLE)) {
        return false;
    }
    
    // 更新设备状态
    {
        std::lock_guard<std::mutex> lock(owner_->status_mutex_);
        owner_->device_status_.motors_enabled = true;
        owner_->clearError();
    }
    
    return true;
}

/**
 * @brief  失能电机
 * @param  null
 * @retval 成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::disableMotors() {
    // 发送失能命令
    std::vector<MotorCommand> disable_commands(owner_->getNumDofs());
    
    for (int i = 0; i < owner_->getNumDofs(); ++i) {
        disable_commands[i].kp = 0.0f;
        disable_commands[i].position = 0.0f;
        disable_commands[i].kd = 0.0f;
        disable_commands[i].velocity = 0.0f;
        disable_commands[i].torque = 0.0f;
    }
    
    // 通过sendMotorCommands发送失能命令
    if (!sendMotorCommands(disable_commands, MOTOR_MODE_DISABLE)) {
        return false;
    }
    
    // 更新设备状态
    {
        std::lock_guard<std::mutex> lock(owner_->status_mutex_);
        owner_->device_status_.motors_enabled = false;
        owner_->clearError();
    }
    
    return true;
}

/**
 * @brief  发送电机命令
 * @param  commands 电机命令数组
 * @param  mode 电机模式
 * @retval 成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::sendMotorCommands(const std::vector<MotorCommand>& commands, uint8_t mode) {
    if (commands.size() != static_cast<size_t>(owner_->getNumDofs())) {
        return false;
    }
    
    // 记录当前模式
    current_motor_mode_ = mode;
    
    // 编码控制帧，使用指定的mode
    auto frame_data = encodeMitControlFrame(commands, mode);
    if (frame_data.empty()) {
        return false;
    }
    
    // 将控制帧保存到队列，由writeThreadFunc定频发送
    {
        std::lock_guard<std::mutex> lock(control_frame_mutex_);
        last_control_frame_ = std::move(frame_data);
    }
    
    return true;
}

/**
 * @brief  读取电机状态
 * @param  states 电机状态输出数组
 * @retval 成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::readMotorStates(std::vector<MotorState>& states) {
    std::lock_guard<std::mutex> lock(motor_states_mutex_);
    states = latest_motor_states_;
    return true;
}

/*******************************************************************************
 * Thread Functions
 ******************************************************************************/

/**
 * @brief  读线程函数
 * @param  null
 * @retval null
 */
void TeleopArmAdapter::Impl::readThreadFunc() {
    std::vector<uint8_t> buffer(1024);
    std::vector<uint8_t> temp_buffer(1024);
    const size_t min_frame_size = sizeof(ProtocolHeader) + 1;  // 最小帧长度：帧头 + 校验和
    
    owner_->comm_interface_->clearBuffer();

    while (owner_->thread_running_) {
        try {
            // 读取数据
            int bytes_read = owner_->comm_interface_->read(buffer.data(), buffer.size(), 10000);
            
            // 检查是否满足最小帧长度
            if (bytes_read < static_cast<int>(min_frame_size)) {
                NEXUS_COUT << "Insufficient data: " << bytes_read << " bytes read" << std::endl;
                continue;  // 数据不足，丢弃
            }
            
            // 检查帧头是否正确
            if (buffer[0] != PROTOCOL_HEADER) {
                NEXUS_COUT << "Invalid frame header" << std::endl;
                continue;  // 帧头不匹配，丢弃
            }
            
            // 验证帧完整性（零拷贝）
            if (validateFrame(buffer.data(), bytes_read)) {
                processReceivedFrame(buffer.data(), bytes_read);
            } else {
                checksum_error_count_++;
            }
            
        } catch (const std::exception& e) {
            // 记录错误但继续运行
            owner_->setError(100, "Read thread error: " + std::string(e.what()));
        }
    }
}

/**
 * @brief  写线程函数
 * @param  null
 * @retval null
 */
void TeleopArmAdapter::Impl::writeThreadFunc() {
    // 根据配置的反馈率计算通讯周期
    auto comm_period = std::chrono::milliseconds(1000 / owner_->getConfig().feedback_rate);
    
    while (owner_->thread_running_) {
        auto cycle_start = std::chrono::steady_clock::now();
        
        try {
            // 优先发送信令帧
            bool signal_frame_sent = false;
            {
                std::lock_guard<std::mutex> lock(signal_queue_mutex_);
                if (!signal_queue_.empty()) {
                    SignalFrame& frame = signal_queue_.front();
                    
                    // 编码信令帧
                    auto frame_data = encodeSignalFrame(frame.type, frame.data);
                    
                    // 发送前更新帧序号，确保序号连续递增
                    uint16_t current_seq = sequence_number_++;
                    // 使用小端序写入16位序号（偏移1字节，跳过帧头）
                    EndianUtils::writeLittle16(&frame_data[1], current_seq);
                    
                    // 重新计算校验和（因为帧序号改变了）
                    uint8_t checksum = calculateChecksum(frame_data.data(), frame_data.size() - 1);
                    frame_data[frame_data.size() - 1] = checksum;
                    
                    // 发送信令帧
                    int bytes_sent = owner_->comm_interface_->write(frame_data.data(), frame_data.size());
                    
                    if (bytes_sent == static_cast<int>(frame_data.size())) {
                        signal_frame_sent = true;
                    } else {
                        NEXUS_COUT << "Signal frame send FAILED!" << std::endl;
                    }
                    // 无论发送成功与否，都立即移除（避免重复发送）
                    signal_queue_.pop();
                }
            }
            
            // 如果没有信令帧，发送MIT控制帧
            if (!signal_frame_sent) {
                std::lock_guard<std::mutex> lock(control_frame_mutex_);
                if (!last_control_frame_.empty()) {
                    // 每次发送前更新帧序号，确保MCU收到的序号单调递增
                    uint16_t current_seq = sequence_number_++;
                    // 使用小端序写入16位序号（偏移1字节，跳过帧头）
                    EndianUtils::writeLittle16(&last_control_frame_[1], current_seq);
                    
                    // 重新计算校验和（因为帧序号改变了）
                    uint8_t checksum = calculateChecksum(last_control_frame_.data(), last_control_frame_.size() - 1);
                    last_control_frame_[last_control_frame_.size() - 1] = checksum;
                    
                    int bytes_sent = owner_->comm_interface_->write(last_control_frame_.data(), 
                                                           last_control_frame_.size());
                    if (bytes_sent == static_cast<int>(last_control_frame_.size())) {
                        // 发送成功
                    }
                }
            }
            
        } catch (const std::exception& e) {
            owner_->setError(101, "Write thread error: " + std::string(e.what()));
        }
        
        // 计算剩余时间并等待到下一个周期
        auto cycle_end = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(cycle_end - cycle_start);
        auto remaining = comm_period - elapsed;
        
        if (remaining.count() > 0) {
            std::this_thread::sleep_for(remaining);
        }
    }
}

/*******************************************************************************
 * Teleop Arm Specific Functions
 ******************************************************************************/

/**
 * @brief  设置电机零位
 * @param  timeout_ms 超时时间（毫秒）
 * @retval 成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::setMotorZero(int timeout_ms) {
    std::vector<uint8_t> payload = {0x01};  // 设置零位命令
    std::vector<uint8_t> response;
    
    if (!sendSignalFrameAndWaitResponse(SignalFrameType::ZERO_SET, payload, response, timeout_ms)) {
        return false;
    }
    
    // 检查应答结果
    return !response.empty() && response[0] == 0x01;  // 0x01表示成功
}

/**
 * @brief  清除设备错误
 * @param  timeout_ms 超时时间（毫秒）
 * @retval 成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::clearDeviceErrors(int timeout_ms) {
    std::vector<uint8_t> payload = {0x01};  // 清除错误命令
    std::vector<uint8_t> response;
    
    if (!sendSignalFrameAndWaitResponse(SignalFrameType::ERROR_CLEAR, payload, response, timeout_ms)) {
        return false;
    }
    
    // 检查应答结果：0x01表示清除成功，0x00表示清除失败
    return !response.empty() && response[0] == 0x01;
}

/**
 * @brief  查询丢包率
 * @param  packet_loss_rates 丢包率输出数组
 * @param  timeout_ms 超时时间（毫秒）
 * @retval 成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::queryPacketLoss(std::vector<float>& packet_loss_rates, int timeout_ms) {
    std::vector<uint8_t> payload = {0x01};  // 查询命令
    std::vector<uint8_t> response;
    
    if (!sendSignalFrameAndWaitResponse(SignalFrameType::PACKET_QUERY, payload, response, timeout_ms)) {
        return false;
    }
    
    // 解析丢包率数据
    if (response.size() < static_cast<size_t>(owner_->getNumDofs() * 2)) {
        return false;
    }
    
    packet_loss_rates.clear();
    for (int i = 0; i < owner_->getNumDofs(); ++i) {
        // 使用小端序读取16位丢包率数据
        uint16_t loss_rate_raw = EndianUtils::readLittle16(&response[i * 2]);
        float loss_rate = static_cast<float>(loss_rate_raw) * 0.1f;  // 单位0.1%
        packet_loss_rates.push_back(loss_rate);
    }
    
    return true;
}

/**
 * @brief  设置通讯频率
 * @param  frequency_hz 频率（Hz）
 * @param  timeout_ms 超时时间（毫秒）
 * @retval 成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::setCommFrequency(int frequency_hz, int timeout_ms) {
    if (frequency_hz < 10 || frequency_hz > 500) {
        return false;
    }
    
    uint8_t freq_param = static_cast<uint8_t>(frequency_hz / 10);  // 单位10Hz
    std::vector<uint8_t> payload = {freq_param};
    std::vector<uint8_t> response;
    
    if (!sendSignalFrameAndWaitResponse(SignalFrameType::FREQ_SET, payload, response, timeout_ms)) {
        return false;
    }
    
    // 检查应答结果
    if (response.empty() || response[0] != 0x01) {
        return false;
    }
    
    // 注意：实际的通讯周期由 owner_->getConfig().feedback_rate 控制
    // 这里只是向设备发送频率设置命令
    
    return true;
}

/**
 * @brief  设置外设状态
 * @param  led_control LED控制字节
 * @param  timeout_ms 超时时间（毫秒）
 * @retval 成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::setPeripheralState(uint8_t led_control, int timeout_ms) {
    std::vector<uint8_t> payload = {led_control};
    std::vector<uint8_t> response;
    
    if (!sendSignalFrameAndWaitResponse(SignalFrameType::PERIPHERAL_SET, payload, response, timeout_ms)) {
        return false;
    }
    
    // 检查应答内容：0x9D响应应至少包含LED状态字节(BYTE1)
    // BYTE1: LED状态 (与命令格式相同)
    // BYTE2: 按钮状态
    // BYTE3-4: 摇杆X坐标 (小端序)
    // BYTE5-6: 摇杆Y坐标 (小端序)
    if (response.empty() || response.size() < 1 || response[0] != led_control) {
        return false;
    }
    
    return true;
}

/**
 * @brief  获取外设状态
 * @param  peripheral_state 外设状态输出
 * @retval 成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::getPeripheralState(PeripheralState& peripheral_state) const {
    std::lock_guard<std::mutex> lock(peripheral_state_mutex_);
    
    // 检查是否有有效的外设状态数据
    // 如果时间戳为默认值，说明还没有收到任何状态
    if (latest_peripheral_state_.timestamp.time_since_epoch().count() == 0) {
        return false;
    }
    
    peripheral_state = latest_peripheral_state_;
    return true;
}

/*******************************************************************************
 * Protocol Processing Functions
 ******************************************************************************/

/**
 * @brief  浮点数转无符号整数
 * @param  value 浮点值
 * @param  min_val 最小值
 * @param  max_val 最大值
 * @param  bits 位数
 * @retval 转换后的无符号整数
 */
uint16_t TeleopArmAdapter::Impl::floatToUint(float value, float min_val, float max_val, int bits) const {
    float span = max_val - min_val;
    float offset = min_val;
    uint16_t max_int = (1 << bits) - 1;
    
    // 限制范围
    value = std::max(min_val, std::min(max_val, value));
    
    return static_cast<uint16_t>((value - offset) * max_int / span);
}

/**
 * @brief  无符号整数转浮点数
 * @param  value 无符号整数值
 * @param  min_val 最小值
 * @param  max_val 最大值
 * @param  bits 位数
 * @retval 转换后的浮点数
 */
float TeleopArmAdapter::Impl::uintToFloat(uint16_t value, float min_val, float max_val, int bits) const {
    float span = max_val - min_val;
    float offset = min_val;
    uint16_t max_int = (1 << bits) - 1;
    
    return static_cast<float>(value) * span / max_int + offset;
}

/**
 * @brief  计算校验和
 * @param  data 数据指针
 * @param  length 数据长度
 * @retval 校验和
 */
uint8_t TeleopArmAdapter::Impl::calculateChecksum(const uint8_t* data, size_t length) const {
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    return sum;
}

/**
 * @brief  编码MIT控制帧
 * @param  commands 电机命令数组
 * @param  mode 电机模式
 * @retval 编码后的帧数据
 */
std::vector<uint8_t> TeleopArmAdapter::Impl::encodeMitControlFrame(const std::vector<MotorCommand>& commands, 
                                                           uint8_t mode) {
    if (commands.size() != static_cast<size_t>(owner_->getNumDofs())) {
        return {};
    }
    
    size_t frame_size = sizeof(ProtocolHeader) + commands.size() * sizeof(MotorControlData) + 1;
    std::vector<uint8_t> frame_data(frame_size);
    
    // 填充帧头
    frame_data[0] = PROTOCOL_HEADER;  // 帧头
    EndianUtils::writeLittle16(&frame_data[1], 0);  // 序号初始化为0，将在发送时更新
    frame_data[3] = CMD_MIT_CONTROL;  // 功能码
    frame_data[4] = static_cast<uint8_t>(commands.size() * sizeof(MotorControlData));  // 数据长度
    
    // 填充电机控制数据（使用字节序转换）
    uint8_t* data_ptr = frame_data.data() + sizeof(ProtocolHeader);
    
    for (size_t i = 0; i < commands.size(); ++i) {
        size_t offset = i * sizeof(MotorControlData);
        
        // mode字段（1字节，无需转换）
        data_ptr[offset] = mode;
        
        // target_pos字段（2字节，转换为小端序）
        uint16_t target_pos = floatToUint(commands[i].position, POS_MIN, POS_MAX, 16);
        EndianUtils::writeLittle16(&data_ptr[offset + 1], target_pos);
        
        // target_vel字段（2字节，转换为小端序）
        uint16_t target_vel = floatToUint(commands[i].velocity, VEL_MIN, VEL_MAX, 12);
        EndianUtils::writeLittle16(&data_ptr[offset + 3], target_vel);
        
        // kp字段（2字节，转换为小端序）
        uint16_t kp = floatToUint(commands[i].kp, KP_MIN, KP_MAX, 12);
        EndianUtils::writeLittle16(&data_ptr[offset + 5], kp);
        
        // kd字段（2字节，转换为小端序）
        uint16_t kd = floatToUint(commands[i].kd, KD_MIN, KD_MAX, 12);
        EndianUtils::writeLittle16(&data_ptr[offset + 7], kd);
        
        // target_torque字段（2字节，转换为小端序）
        uint16_t target_torque = floatToUint(commands[i].torque, TORQUE_MIN, TORQUE_MAX, 12);
        EndianUtils::writeLittle16(&data_ptr[offset + 9], target_torque);
    }
    
    // 计算校验和
    uint8_t checksum = calculateChecksum(frame_data.data(), frame_data.size() - 1);
    frame_data[frame_data.size() - 1] = checksum;
    
    return frame_data;
}

/**
 * @brief  解码MIT反馈帧
 * @param  data 数据指针
 * @param  length 数据长度
 * @param  states 电机状态输出
 * @retval 解码成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::decodeMitFeedbackFrame(const uint8_t* data, size_t length, std::vector<MotorState>& states) {
    if (length < sizeof(ProtocolHeader) + 1) {
        return false;
    }
    
    const ProtocolHeader* header = reinterpret_cast<const ProtocolHeader*>(data);
    if (header->head != PROTOCOL_HEADER || header->cmd != CMD_MIT_FEEDBACK) {
        return false;
    }
    
    size_t expected_motors = header->length / sizeof(MotorFeedbackData);
    if (expected_motors != static_cast<size_t>(owner_->getNumDofs())) {
        return false;
    }
    
    // 使用字节序转换解析反馈数据
    const uint8_t* motor_data_ptr = data + sizeof(ProtocolHeader);
    
    states.resize(owner_->getNumDofs());
    for (int i = 0; i < owner_->getNumDofs(); ++i) {
        size_t offset = i * sizeof(MotorFeedbackData);
        
        // work_status字段（1字节，无需转换）
        uint8_t work_status = motor_data_ptr[offset];
        
        // current_pos字段（2字节，从小端序读取）
        uint16_t current_pos = EndianUtils::readLittle16(&motor_data_ptr[offset + 1]);
        states[i].position = uintToFloat(current_pos, POS_MIN, POS_MAX, 16);
        
        // current_vel字段（2字节，从小端序读取）
        uint16_t current_vel = EndianUtils::readLittle16(&motor_data_ptr[offset + 3]);
        states[i].velocity = uintToFloat(current_vel, VEL_MIN, VEL_MAX, 12);
        
        // current_torque字段（2字节，从小端序读取）
        uint16_t current_torque = EndianUtils::readLittle16(&motor_data_ptr[offset + 5]);
        states[i].effort = uintToFloat(current_torque, TORQUE_MIN, TORQUE_MAX, 12);
        
        // temperature字段（1字节，无需转换）
        uint8_t temperature = motor_data_ptr[offset + 7];
        states[i].temperature = static_cast<double>(temperature);
        
        // error_code字段（1字节，无需转换）
        uint8_t error_code = motor_data_ptr[offset + 8];
        states[i].error_code = error_code;
        
        // 设置使能状态
        states[i].enabled = (work_status == MOTOR_STATUS_ENABLE);
    }
    
    return true;
}

/**
 * @brief  编码信令帧（按命令码）
 * @param  cmd 命令码
 * @param  payload 载荷数据
 * @retval 编码后的帧数据
 */
std::vector<uint8_t> TeleopArmAdapter::Impl::encodeSignalFrame(uint8_t cmd, const std::vector<uint8_t>& payload) {
    size_t frame_size = sizeof(ProtocolHeader) + payload.size() + 1;
    std::vector<uint8_t> frame_data(frame_size);
    
    // 填充帧头
    frame_data[0] = PROTOCOL_HEADER;  // 帧头
    EndianUtils::writeLittle16(&frame_data[1], 0);  // 序号初始化为0，将在发送时更新
    frame_data[3] = cmd;  // 功能码
    frame_data[4] = static_cast<uint8_t>(payload.size());  // 数据长度
    
    // 填充载荷
    if (!payload.empty()) {
        std::memcpy(frame_data.data() + sizeof(ProtocolHeader), payload.data(), payload.size());
    }
    
    // 计算校验和
    uint8_t checksum = calculateChecksum(frame_data.data(), frame_data.size() - 1);
    frame_data[frame_data.size() - 1] = checksum;
    
    return frame_data;
}

/**
 * @brief  编码信令帧（按类型）
 * @param  type 信令帧类型
 * @param  payload 载荷数据
 * @retval 编码后的帧数据
 */
std::vector<uint8_t> TeleopArmAdapter::Impl::encodeSignalFrame(SignalFrameType type, const std::vector<uint8_t>& payload) {
    uint8_t cmd = 0;
    
    switch (type) {
        case SignalFrameType::ZERO_SET:
            cmd = CMD_ZERO_SET;
            break;
        case SignalFrameType::FREQ_SET:
            cmd = CMD_FREQ_SET;
            break;
        case SignalFrameType::ERROR_CLEAR:
            cmd = CMD_ERROR_CLEAR;
            break;
        case SignalFrameType::PACKET_QUERY:
            cmd = CMD_PACKET_QUERY;
            break;
        case SignalFrameType::PERIPHERAL_SET:
            cmd = CMD_PERIPHERAL_SET;
            break;
        default:
            return {};
    }
    
    return encodeSignalFrame(cmd, payload);
}

/**
 * @brief  解码信令响应
 * @param  expected_cmd 期望的命令码
 * @param  data 数据指针
 * @param  length 数据长度
 * @param  payload 载荷输出
 * @retval 解码成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::decodeSignalResponse(uint8_t expected_cmd, const uint8_t* data, size_t length, std::vector<uint8_t>& payload) {
    if (length < sizeof(ProtocolHeader) + 1) {
        return false;
    }
    
    const ProtocolHeader* header = reinterpret_cast<const ProtocolHeader*>(data);
    if (header->head != PROTOCOL_HEADER || header->cmd != expected_cmd) {
        return false;
    }
    
    payload.clear();
    if (header->length > 0) {
        const uint8_t* payload_start = data + sizeof(ProtocolHeader);
        payload.assign(payload_start, payload_start + header->length);
    }
    
    return true;
}

/**
 * @brief  发送信令帧并等待响应
 * @param  type 信令帧类型
 * @param  payload 载荷数据
 * @param  response 响应输出
 * @param  timeout_ms 超时时间（毫秒）
 * @retval 成功返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::sendSignalFrameAndWaitResponse(SignalFrameType type, const std::vector<uint8_t>& payload,
                                                   std::vector<uint8_t>& response, int timeout_ms) {
    uint8_t expected_response_cmd = 0;
    
    switch (type) {
        case SignalFrameType::ZERO_SET:
            expected_response_cmd = CMD_ZERO_RESPONSE;
            break;
        case SignalFrameType::FREQ_SET:
            expected_response_cmd = CMD_FREQ_RESPONSE;
            break;
        case SignalFrameType::ERROR_CLEAR:
            expected_response_cmd = CMD_ERROR_RESPONSE;  // 错误清除有应答
            break;
        case SignalFrameType::PACKET_QUERY:
            expected_response_cmd = CMD_PACKET_RESPONSE;
            break;
        case SignalFrameType::PERIPHERAL_SET:
            expected_response_cmd = CMD_PERIPHERAL_RESPONSE;
            break;
        default:
            return false;
    }
    
    // 准备等待应答
    {
        std::lock_guard<std::mutex> lock(response_mutex_);
        expected_response_cmd_ = expected_response_cmd;
        response_received_ = false;
        pending_response_.clear();
    }
    
    // 将信令帧加入队列，由writeThreadFunc发送
    {
        std::lock_guard<std::mutex> lock(signal_queue_mutex_);
        SignalFrame frame;
        frame.type = type;
        frame.data = payload;
        frame.send_time = std::chrono::steady_clock::now();
        frame.retry_count = 0;
        signal_queue_.push(frame);
    }
    
    // 等待应答
    std::unique_lock<std::mutex> lock(response_mutex_);
    bool received = response_cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
                                        [this] { return response_received_; });
    
    if (received) {
        // 信令帧已在writeThreadFunc中发送后立即移除，这里只需要返回应答数据
        response = pending_response_;
        return true;
    } else {
        // 超时，信令帧已在writeThreadFunc中发送后立即移除
        timeout_count_++;
        return false;
    }
}

/*******************************************************************************
 * Frame Processing Functions
 ******************************************************************************/

/**
 * @brief  处理接收到的帧
 * @param  data 数据指针
 * @param  length 数据长度
 * @retval null
 */
void TeleopArmAdapter::Impl::processReceivedFrame(const uint8_t* data, size_t length) {
    if (length < sizeof(ProtocolHeader)) {
        return;
    }
    
    const ProtocolHeader* header = reinterpret_cast<const ProtocolHeader*>(data);

    // 打印帧序号 (使用小端序读取)，设备上实际序号为 16 位，但内部统计使用 32/64 位扩展以便处理回绕与累计
    uint32_t seq_num = static_cast<uint32_t>(EndianUtils::readLittle16(reinterpret_cast<const uint8_t*>(&header->seq_num)));

    // 强制转换为 32 位后传递给监控/统计逻辑（notifyReceivedFrame 接受 uint32_t）
    owner_->notifyReceivedFrame(seq_num);
    
    switch (header->cmd) {
        case CMD_MIT_FEEDBACK: {
            std::lock_guard<std::mutex> lock(motor_states_mutex_);
            if (decodeMitFeedbackFrame(data, length, latest_motor_states_)) {
                last_feedback_time_ = std::chrono::steady_clock::now();
            }
            break;
        }
        
        case CMD_PERIPHERAL_RESPONSE: {
            // 同时更新外设状态和处理信令响应
            // 解析外设状态数据 (载荷6字节: LED + 按键 + 摇杆X(2B) + 摇杆Y(2B))
            if (length >= sizeof(ProtocolHeader) + 6) {
                std::lock_guard<std::mutex> lock(peripheral_state_mutex_);
                const uint8_t* payload = data + sizeof(ProtocolHeader);
                latest_peripheral_state_.led_status = payload[0];      // BYTE1: LED状态
                latest_peripheral_state_.button_status = payload[1];   // BYTE2: 按钮状态
                latest_peripheral_state_.joystick_x =                  // BYTE3-4: 摇杆X (大端序)
                    (static_cast<uint16_t>(payload[2]) << 8) | static_cast<uint16_t>(payload[3]);
                latest_peripheral_state_.joystick_y =                  // BYTE5-6: 摇杆Y (大端序)
                    (static_cast<uint16_t>(payload[4]) << 8) | static_cast<uint16_t>(payload[5]);
                latest_peripheral_state_.timestamp = std::chrono::steady_clock::now();
            } else if (length >= sizeof(ProtocolHeader) + 2) {
                // 兼容旧版协议（仅LED+按键，无摇杆数据）
                std::lock_guard<std::mutex> lock(peripheral_state_mutex_);
                const uint8_t* payload = data + sizeof(ProtocolHeader);
                latest_peripheral_state_.led_status = payload[0];
                latest_peripheral_state_.button_status = payload[1];
                latest_peripheral_state_.joystick_x = 2048;  // 默认中间值
                latest_peripheral_state_.joystick_y = 2048;
                latest_peripheral_state_.timestamp = std::chrono::steady_clock::now();
            }
            
            // 同时处理信令响应（用于setPeripheralState的应答）
            {
                std::lock_guard<std::mutex> lock(response_mutex_);
                if (decodeSignalResponse(expected_response_cmd_, data, length, pending_response_)) {
                    response_received_ = true;
                    response_cv_.notify_one();
                }
            }
            break;
        }
        
        case CMD_ZERO_RESPONSE:
        case CMD_FREQ_RESPONSE:
        case CMD_ERROR_RESPONSE:
        case CMD_PACKET_RESPONSE: {
            std::lock_guard<std::mutex> lock(response_mutex_);
            if (decodeSignalResponse(expected_response_cmd_, data, length, pending_response_)) {
                response_received_ = true;
                response_cv_.notify_one();
            }
            break;
        }
        
        default:
            // 未知命令，忽略
            break;
    }
}

/**
 * @brief  验证帧完整性
 * @param  data 数据指针
 * @param  length 数据长度
 * @retval 验证通过返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::validateFrame(const uint8_t* data, size_t length) {
    if (length < sizeof(ProtocolHeader) + 1) {
        return false;
    }
    
    // 验证校验和
    uint8_t calculated_checksum = calculateChecksum(data, length - 1);
    uint8_t frame_checksum = data[length - 1];
    
    return calculated_checksum == frame_checksum;
}

/*******************************************************************************
 * Timing Management Functions
 ******************************************************************************/

/**
 * @brief  检查是否可以发送控制帧
 * @param  null
 * @retval 可以发送返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::canSendControlFrame() const {
    // 根据配置的反馈率计算通讯周期
    auto comm_period = std::chrono::milliseconds(1000 / owner_->getConfig().feedback_rate);
    // 检查距离上次发送是否超过一个周期
    auto elapsed = std::chrono::steady_clock::now() - last_control_send_time_;
    return elapsed >= comm_period;
}

/**
 * @brief  更新通讯周期
 * @param  null
 * @retval null
 */
void TeleopArmAdapter::Impl::updateCommCycle() {
    cycle_start_time_ = std::chrono::steady_clock::now();
}

/**
 * @brief  处理信令队列
 * @param  null
 * @retval null
 */
void TeleopArmAdapter::Impl::processSignalQueue() {
    std::lock_guard<std::mutex> lock(signal_queue_mutex_);
    
    // 简化实现：在这个版本中信令帧通过同步方式处理
    // 实际使用中可以实现更复杂的队列处理逻辑
}

/**
 * @brief  获取电机反馈数据
 * @param  null
 * @retval 电机反馈数据数组
 */
std::vector<MotorFeedbackData> TeleopArmAdapter::Impl::getMotorFeedback() const {
    std::lock_guard<std::mutex> lock(owner_->status_mutex_);
    return motor_feedback_;
}

/**
 * @brief  检查是否有错误
 * @param  null
 * @retval 有错误返回true，否则返回false
 */
bool TeleopArmAdapter::Impl::hasErrors() const {
    std::lock_guard<std::mutex> lock(owner_->status_mutex_);
    
    // Check for device errors
    if (device_errors_.load() > 0) {
        return true;
    }
    
    // Check motor status for errors
    for (const auto& feedback : motor_feedback_) {
        if (feedback.error_code != 0) {
            return true;
        }
    }
    
    return false;
}

} // namespace teleop_adapter
