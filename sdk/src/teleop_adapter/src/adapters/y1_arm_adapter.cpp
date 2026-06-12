/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/src/adapters/y1_arm_adapter.cpp
 * @Description: Y1机械臂适配器实现，封装Y1 SDK接口，实现设备通信与控制
 *
 * Copyright (c) 2025 by Pegasus, All Rights Reserved.
 */

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "teleop_adapter/adapters/y1_arm_adapter.hpp"
#include "y1_sdk_interface.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include "nexus_log.hpp"
#include <fstream>
#include <sys/stat.h>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <mutex>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

/*******************************************************************************
 * Impl — hidden Y1 SDK state
 ******************************************************************************/

struct Y1ArmAdapter::Impl {
    std::unique_ptr<imeta::y1_controller::Y1SDKInterface> y1;
    std::mutex y1_mutex;
    std::string can_id;
    std::string urdf_path;
    int arm_end_type{0};
    bool enable_arm_flag{true};

    std::vector<MotorCommand> last_commands;
    std::mutex commands_mutex;

    std::vector<MotorState> y1_latest_states;
    std::mutex y1_states_mutex;
};

/*******************************************************************************
 * Constructor & Destructor
 ******************************************************************************/

Y1ArmAdapter::Y1ArmAdapter(const DeviceAdapterConfig& config,
                           const std::string& can_id)
  : DeviceAdapter(config, nullptr),
    impl_(std::make_unique<Impl>()) {
  auto& m = *impl_;
  m.can_id = can_id;
  std::string package_share_dir = ament_index_cpp::get_package_share_directory("teleop_adapter");
  m.urdf_path = package_share_dir + "/urdf/y1_with_gripper.urdf";
  m.arm_end_type = 1;
  m.enable_arm_flag = true;

  try {
    m.y1 = std::make_unique<imeta::y1_controller::Y1SDKInterface>(m.can_id, m.urdf_path, m.arm_end_type, m.enable_arm_flag);
  } catch (const std::exception& e) {
    NEXUS_CERR << "[Y1ArmAdapter] Failed to create Y1SDKInterface: " << e.what() << std::endl;
    m.y1.reset();
  }

  {
    std::lock_guard<std::mutex> sl(m.y1_states_mutex);
    m.y1_latest_states.resize(config_.num_of_dofs);
    for (auto &state : m.y1_latest_states) {
      state.position = 0.0f;
      state.velocity = 0.0f;
      state.effort = 0.0f;
      state.enabled = false;
      state.error_code = 0;
      state.temperature = 0.0;
    }
  }

  {
    std::lock_guard<std::mutex> sl(status_mutex_);
    device_status_.status = 0; // NORMAL
  }

  enableExternalStats(true);
}

/**
 * @brief  析构函数
 * @param  null
 * @retval null
 */
Y1ArmAdapter::~Y1ArmAdapter() { 
  shutdown(); 
}

/*******************************************************************************
 * Public Methods - DeviceAdapter Interface
 ******************************************************************************/

/**
 * @brief  关闭适配器
 * @param  null
 * @retval null
 */
void Y1ArmAdapter::shutdown() {
  // 调用基类的 shutdown（会停止读写线程）
  DeviceAdapter::shutdown();
}

/**
 * @brief  设备握手
 * @param  null
 * @retval 握手成功返回true，否则返回false
 */
bool Y1ArmAdapter::deviceHandshake() {
  auto& m = *impl_;
  std::lock_guard<std::mutex> lock(m.y1_mutex);
  if (!m.y1) return false;
  bool ok = m.y1->Init();
  if (ok) {
    std::lock_guard<std::mutex> sl(status_mutex_);
    device_status_.connected = 1;
    device_status_.initialized = true;
    device_status_.status = 0;
    device_status_.error_code = 0;
    device_status_.error_message.clear();
  }
  return ok;
}

/**
 * @brief  配置设备
 * @param  null
 * @retval 配置成功返回true，否则返回false
 */
bool Y1ArmAdapter::configureDevice() {
  auto& m = *impl_;
  std::lock_guard<std::mutex> lock(m.y1_mutex);
  if (!m.y1) return false;
  m.y1->SetArmControlMode(imeta::y1_controller::Y1SDKInterface::MIT_CONTROL);
  return true;
}

/**
 * @brief  使能电机
 * @param  null
 * @retval 成功返回true，否则返回false
 */
bool Y1ArmAdapter::enableMotors() {
  auto& m = *impl_;
  std::lock_guard<std::mutex> lock(m.y1_mutex);
  if (!m.y1) return false;
  m.y1->SetEnableArm(true);
  {
    std::lock_guard<std::mutex> sl(status_mutex_);
    device_status_.motors_enabled = true;
  }
  {
    std::lock_guard<std::mutex> lk(m.commands_mutex);
    m.last_commands.assign(static_cast<size_t>(config_.num_of_dofs), MotorCommand{});
  }
  return true;
}

/**
 * @brief  失能电机
 * @param  null
 * @retval 成功返回true，否则返回false
 */
bool Y1ArmAdapter::disableMotors() {
  auto& m = *impl_;
  std::lock_guard<std::mutex> lock(m.y1_mutex);
  if (!m.y1) return false;
  m.y1->SetEnableArm(false);
  {
    std::lock_guard<std::mutex> sl(status_mutex_);
    device_status_.motors_enabled = false;
  }
  return true;
}

/**
 * @brief  发送电机命令
 * @param  commands 电机命令数组
 * @retval 成功返回true，否则返回false
 */
bool Y1ArmAdapter::sendMotorCommands(const std::vector<MotorCommand>& commands) {
  std::lock_guard<std::mutex> lk(impl_->commands_mutex);
  impl_->last_commands = commands;
  return true;
}

bool Y1ArmAdapter::readMotorStates(std::vector<MotorState>& states) {
  std::lock_guard<std::mutex> lock(impl_->y1_states_mutex);
  states = impl_->y1_latest_states;
  return true;
}

/**
 * @brief  获取设备状态
 * @param  null
 * @retval 设备状态结构体
 */
DeviceStatus Y1ArmAdapter::getDeviceStatus() const {
  std::lock_guard<std::mutex> sl(status_mutex_);
  return device_status_;
}

/*******************************************************************************
 * Thread Functions
 ******************************************************************************/

/**
 * @brief  读线程函数
 * @param  null
 * @retval null
 */
void Y1ArmAdapter::readThreadFunc() {
  auto& mi = *impl_;
  auto comm_period = std::chrono::milliseconds(1000 / std::max(1, config_.feedback_rate));
  auto next_read_time = std::chrono::steady_clock::now() + comm_period;

  while (thread_running_) {
    try {
      bool sdk_valid = false;
      {
        std::lock_guard<std::mutex> lock(mi.y1_mutex);
        sdk_valid = (mi.y1 != nullptr);
      }

      if (!sdk_valid) {
        // SDK 指针失效，停止读线程
        break;
      }

      std::vector<double> pos, vel, eff, temp;
      std::vector<int> joint_errors;
      bool read_success = false;
      {
        std::lock_guard<std::mutex> lock(mi.y1_mutex);
        if (mi.y1) {
          try {
            pos = mi.y1->GetJointPosition();
            vel = mi.y1->GetJointVelocity();
            eff = mi.y1->GetJointEffort();
            temp = mi.y1->GetRotorTemperature();
            joint_errors = mi.y1->GetJointErrorCode();
            read_success = true;
          } catch (const std::exception& e) {
            NEXUS_CERR << "[Y1ArmAdapter] readThreadFunc exception: " << e.what() << std::endl;
            // 即使异常，也要通知监控线程我们在尝试读取（避免断连）
            read_success = false;
            // CAN 超时异常：使用 setError 设置错误码
            if (std::string(e.what()).find("Timeout waiting for CAN frame") != std::string::npos) {
              setError(101, "Timeout waiting for CAN frame");
            }
          }
        }
      }

      // 处理遥测数据（检查是否为空）
      bool has_valid_telemetry = read_success && !(pos.empty() && vel.empty() && eff.empty());
      
      // 更新设备丢包统计（每适配器独立统计）：成功读取计为收包，失败/超时计为丢包
      {
        std::lock_guard<std::mutex> sl(status_mutex_);
        if (read_success) {
          device_status_.total_packet_count += 1;
        } else {
          device_status_.total_packet_loss_count += 1;
        }
        const uint64_t total = device_status_.total_packet_count + device_status_.total_packet_loss_count;
        device_status_.overall_packet_loss_rate = total ? static_cast<float>(device_status_.total_packet_loss_count) / static_cast<float>(total) : 0.0f;
      }

      // 成功读取时通知监控线程（用于连接状态判断），使用自增序号
      static uint32_t read_seq_counter = 0;
      if (read_success) {
        notifyReceivedFrame(++read_seq_counter);
      }
      
      // 注意：所有状态管理由监控线程和心跳线程负责

      double grip_pos = 0.0;
      {
        std::lock_guard<std::mutex> lock(mi.y1_mutex);
        if (mi.y1) {
          try {
            grip_pos = mi.y1->GetGripperJointPosition();
          } catch (const std::exception& e) {
            NEXUS_CERR << "[Y1ArmAdapter] GetGripperJointPosition exception: " << e.what() << std::endl;
          }
        }
      }

      {
        std::lock_guard<std::mutex> sl(mi.y1_states_mutex);
        if (mi.y1_latest_states.size() != static_cast<size_t>(config_.num_of_dofs)) {
          mi.y1_latest_states.resize(config_.num_of_dofs);
        }
        
        for (size_t i = 0; i < 6 && i < config_.num_of_dofs; ++i) {
          mi.y1_latest_states[i].position = static_cast<float>((i < pos.size()) ? pos[i] : 0.0);
          mi.y1_latest_states[i].velocity = static_cast<float>((i < vel.size()) ? vel[i] : 0.0);
          mi.y1_latest_states[i].effort = static_cast<float>((i < eff.size()) ? eff[i] : 0.0);
          mi.y1_latest_states[i].temperature = static_cast<double>((i < temp.size()) ? temp[i] : 0.0);
          mi.y1_latest_states[i].enabled = has_valid_telemetry;
          
          int error_code = (i < joint_errors.size()) ? joint_errors[i] : 0;
          mi.y1_latest_states[i].error_code = error_code;
          
          if (error_code >= 2) {
            setError(error_code, "Joint " + std::to_string(i) + " error code: " + std::to_string(error_code));
          }
        }
        
        if (config_.num_of_dofs > 6) {
          mi.y1_latest_states[6].position = static_cast<float>(grip_pos);
          mi.y1_latest_states[6].velocity = static_cast<float>((6 < vel.size()) ? vel[6] : 0.0);
          mi.y1_latest_states[6].effort = static_cast<float>((6 < eff.size()) ? eff[6] : 0.0);
          mi.y1_latest_states[6].temperature = static_cast<double>((6 < temp.size()) ? temp[6] : 0.0);
          mi.y1_latest_states[6].enabled = has_valid_telemetry;
          
          int gripper_error = (6 < joint_errors.size()) ? joint_errors[6] : 0;
          mi.y1_latest_states[6].error_code = gripper_error;
          
          if (gripper_error >= 2) {
            setError(gripper_error, "Gripper error code: " + std::to_string(gripper_error));
          }
        }
      }

    } catch (const std::exception& e) {
      NEXUS_CERR << "[Y1ArmAdapter] readThreadFunc outer exception: " << e.what() << std::endl;
    }

    // 精确等待到下一个周期（使用 sleep_until 而不是 sleep_for）
    std::this_thread::sleep_until(next_read_time);
    next_read_time += comm_period;
  }
}

/**
 * @brief  写线程函数
 * @param  null
 * @retval null
 */
void Y1ArmAdapter::writeThreadFunc() {
  auto& mi = *impl_;
  auto comm_period = std::chrono::milliseconds(1000 / std::max(1, config_.feedback_rate));
  auto next_write_time = std::chrono::steady_clock::now() + comm_period;

  while (thread_running_) {
    try {
      std::vector<MotorCommand> cmds;
      {
        std::lock_guard<std::mutex> lk(mi.commands_mutex);
        cmds = mi.last_commands;
      }

      if (!cmds.empty()) {
        std::lock_guard<std::mutex> lock(mi.y1_mutex);
        if (mi.y1) {
          std::array<imeta::y1_controller::MitControlCommand, 6> arr{};
          for (size_t i = 0; i < 6 && i < cmds.size(); ++i) {
            arr[i].kp = cmds[i].kp;
            arr[i].joint_position = cmds[i].position;
            arr[i].kd = cmds[i].kd;
            arr[i].joint_velocity = cmds[i].velocity;
            arr[i].torque = cmds[i].torque;
          }
          mi.y1->MitControlArm(arr);

          if (cmds.size() > 6) {
            imeta::y1_controller::MitControlCommand g{};
            g.kp = cmds[6].kp;
            g.joint_position = cmds[6].position;
            g.kd = cmds[6].kd;
            g.joint_velocity = cmds[6].velocity;
            g.torque = cmds[6].torque;
            mi.y1->MitControlGripper(g);
          }
        }
      }

    } catch (const std::exception& e) {
      NEXUS_CERR << "[Y1ArmAdapter] writeThreadFunc error: " << e.what() << std::endl;
    }

    std::this_thread::sleep_until(next_write_time);
    next_write_time += comm_period;
  }
}

} // namespace teleop_adapter
