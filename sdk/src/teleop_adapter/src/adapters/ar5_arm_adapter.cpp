/*
 * @Author: Infra Embedded
 * @Date: 2025-03-31 10:00:00
 * @FilePath: teleop_adapter/src/adapters/ar5_arm_adapter.cpp
 * @Description: AR5机械臂适配器实现，封装Rokae SDK实时控制接口
 *
 *               支持两种控制模式（通过宏 AR5_USE_POSITION_CONTROL 切换）:
 *               - 位置控制模式(默认): 使用 JointPosition 回调，直接取MIT命令中的position字段下发
 *               - 力矩控制模式: 使用 Torque 回调，通过MIT公式计算期望力矩
 *                 MIT公式: tau = kp * (q_des - q_cur) + kd * (dq_des - dq_cur) + tau_ff
 *
 *               三线程架构:
 *               1) SDK回调线程(1ms): 原子地读状态+计算命令+返回控制命令
 *               2) writeThreadFunc: 按yaml频率将上层MIT命令写入共享缓存
 *               3) readThreadFunc: 按yaml频率从共享缓存读取关节状态更新电机状态
 *
 * Copyright (c) 2025 by Infra Embedded, All Rights Reserved.
 */

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "teleop_adapter/adapters/ar5_arm_adapter.hpp"
#include "rokae/robot.h"
#include "rokae/motion_control_rt.h"
#include "rokae/data_types.h"
#include <iostream>
#include "nexus_log.hpp"
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <cmath>

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

/*******************************************************************************
 * Impl — hidden Rokae SDK state
 ******************************************************************************/

struct Ar5ArmAdapter::Impl {
    std::unique_ptr<rokae::ArRobot> robot;
    std::shared_ptr<rokae::RtMotionControlCobot<AR5_DOF>> rt_controller;
    std::mutex robot_mutex;

    std::string robot_ip;
    std::string local_ip;
    std::array<double, AR5_DOF> init_joint_positions;
    rokae::Load load{};
    bool has_load{false};

    std::vector<MotorCommand> cmd_buffer;
    std::mutex cmd_buffer_mutex;

    std::array<double, AR5_DOF> cb_joint_pos{};
    std::array<double, AR5_DOF> cb_joint_vel{};
    std::array<double, AR5_DOF> cb_joint_torque{};
    std::array<double, 6> cb_ext_wrench{};

    std::vector<MotorState> ar5_latest_states;
    std::array<double, 6> ar5_latest_ext_wrench{};
    std::mutex ar5_states_mutex;
    std::mutex cb_state_mutex;

    std::atomic<bool> rt_control_started{false};
    std::atomic<uint64_t> cb_loop_count{0};

    std::array<double, AR5_DOF> prev_tau_desired{};

    std::thread rt_loop_thread;
};

/*******************************************************************************
 * Constructor & Destructor
 ******************************************************************************/

Ar5ArmAdapter::Ar5ArmAdapter(const DeviceAdapterConfig& config,
                             const std::string& robot_ip,
                             const std::string& local_ip,
                             const std::array<double, AR5_DOF>& init_joint_positions,
                             double load_mass,
                             const std::array<double, 3>& load_cog,
                             const std::array<double, 3>& load_inertia)
  : DeviceAdapter(config, nullptr),
    impl_(std::make_unique<Impl>()) {

  impl_->robot_ip = robot_ip;
  impl_->local_ip = local_ip;
  impl_->init_joint_positions = init_joint_positions;

  if (load_mass > 0 || load_cog[0] != 0 || load_cog[1] != 0 || load_cog[2] != 0 ||
      load_inertia[0] != 0 || load_inertia[1] != 0 || load_inertia[2] != 0) {
      impl_->load = rokae::Load(load_mass, load_cog, load_inertia);
      impl_->has_load = true;
  }

  impl_->cb_joint_pos.fill(0.0);
  impl_->cb_joint_vel.fill(0.0);
  impl_->cb_joint_torque.fill(0.0);

  {
    std::lock_guard<std::mutex> sl(impl_->ar5_states_mutex);
    impl_->ar5_latest_states.resize(config_.num_of_dofs);
    for (auto& state : impl_->ar5_latest_states) {
      state.position = 0.0;
      state.velocity = 0.0;
      state.effort = 0.0;
      state.enabled = false;
      state.error_code = 0;
      state.temperature = 0.0;
    }
  }

  {
    std::lock_guard<std::mutex> sl(status_mutex_);
    device_status_.status = 4; // INITIALIZING
  }

  enableExternalStats(true);
}

/**
 * @brief  析构函数
 */
Ar5ArmAdapter::~Ar5ArmAdapter() {
  shutdown();
}

/*******************************************************************************
 * Public Methods - DeviceAdapter Interface
 ******************************************************************************/

/**
 * @brief  关闭适配器
 *         停止SDK回调循环、停止读写线程、下电、断开连接
 */
void Ar5ArmAdapter::shutdown() {
  DeviceAdapter::shutdown();

  auto& m = *impl_;
  std::lock_guard<std::mutex> lock(m.robot_mutex);
  if (m.robot) {
    error_code ec;
    try {
      if (m.rt_controller && m.rt_control_started) {
        try {
          m.rt_controller->stopLoop();
        } catch (const std::exception& e) {
          NEXUS_CERR << "[Ar5ArmAdapter] stopLoop exception: " << e.what() << std::endl;
        }
        if (m.rt_loop_thread.joinable()) {
          m.rt_loop_thread.join();
        }
        try {
          m.rt_controller->stopMove();
        } catch (const std::exception& e) {
          NEXUS_CERR << "[Ar5ArmAdapter] stopMove exception: " << e.what() << std::endl;
        }
        m.rt_control_started = false;
      }

      m.robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
      if (ec) {
        NEXUS_CERR << "[Ar5ArmAdapter] setMotionControlMode(NrtCommand) failed: " << ec.message() << std::endl;
      }

      m.robot->setOperateMode(rokae::OperateMode::manual, ec);
      m.robot->setPowerState(false, ec);
      m.robot->stopReceiveRobotState();
    } catch (const std::exception& e) {
      NEXUS_CERR << "[Ar5ArmAdapter] shutdown exception: " << e.what() << std::endl;
    }

    m.rt_controller.reset();
    m.robot.reset();
  }
}

/**
 * @brief  设备握手
 *         连接AR5、设置操作模式、上电、MoveJ到初始位姿
 * @retval 成功返回true
 */
bool Ar5ArmAdapter::deviceHandshake() {
  auto& m = *impl_;
  std::lock_guard<std::mutex> lock(m.robot_mutex);

  try {
    NEXUS_COUT << "[Ar5ArmAdapter] Connecting to AR5 at " << m.robot_ip
              << " (local: " << m.local_ip << ")..." << std::endl;
    m.robot = std::make_unique<rokae::ArRobot>(m.robot_ip, m.local_ip);
    NEXUS_COUT << "[Ar5ArmAdapter] Connected to AR5 successfully." << std::endl;

    error_code ec;

    m.robot->recoverState(1, ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] recoverState(estop) failed: " << ec.message() << std::endl;
    } else {
      NEXUS_COUT << "[Ar5ArmAdapter] E-stop recovered." << std::endl;
    }

    m.robot->clearServoAlarm(ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] clearServoAlarm failed: " << ec.message() << std::endl;
    } else {
      NEXUS_COUT << "[Ar5ArmAdapter] Servo alarm cleared." << std::endl;
    }

    m.robot->moveReset(ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] moveReset failed: " << ec.message() << std::endl;
    } else {
      NEXUS_COUT << "[Ar5ArmAdapter] Move reset done." << std::endl;
    }

    m.robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] setMotionControlMode(NrtCommand) failed: " << ec.message() << std::endl;
      return false;
    }

    m.robot->setOperateMode(rokae::OperateMode::automatic, ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] setOperateMode(automatic) failed: " << ec.message() << std::endl;
      return false;
    }

    m.robot->calibrateForceSensor(true, 0, ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] calibrateForceSensor failed: " << ec.message() << std::endl;
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    NEXUS_COUT << "[Ar5ArmAdapter] Force sensor calibration (all axes) done." << std::endl;

    m.robot->setPowerState(true, ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] setPowerState(true) failed: " << ec.message() << std::endl;
      return false;
    }

    NEXUS_COUT << "[Ar5ArmAdapter] Moving to initial joint position..." << std::endl;
    rokae::JointPosition target_pos({
      m.init_joint_positions[0], m.init_joint_positions[1], m.init_joint_positions[2],
      m.init_joint_positions[3], m.init_joint_positions[4], m.init_joint_positions[5],
      m.init_joint_positions[6]
    });
    rokae::MoveAbsJCommand move_cmd(target_pos, 600, 0);
    std::string cmd_id;
    m.robot->moveAppend(move_cmd, cmd_id, ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] moveAppend failed: " << ec.message() << std::endl;
      return false;
    }

    m.robot->moveStart(ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] moveStart failed: " << ec.message() << std::endl;
      return false;
    }

    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      auto state = m.robot->operationState(ec);
      if (state == rokae::OperationState::idle || state == rokae::OperationState::unknown) {
        break;
      }
    }
    NEXUS_COUT << "[Ar5ArmAdapter] Reached initial position." << std::endl;

    // 更新设备状态
    {
      std::lock_guard<std::mutex> sl(status_mutex_);
      device_status_.connected = 1;
      device_status_.initialized = true;
      device_status_.status = 0; // NORMAL
      device_status_.error_code = 0;
      device_status_.error_message.clear();
    }

    return true;

  } catch (const std::exception& e) {
    NEXUS_CERR << "[Ar5ArmAdapter] deviceHandshake exception: " << e.what() << std::endl;
    m.robot.reset();
    return false;
  }
}

/**
 * @brief  配置设备
 *         切换到实时模式、启动状态数据接收、设置SDK回调循环（位置/力矩由宏选择）、非阻塞启动
 * @retval 成功返回true
 */
bool Ar5ArmAdapter::configureDevice() {
  auto& m = *impl_;
  std::lock_guard<std::mutex> lock(m.robot_mutex);
  if (!m.robot) return false;

  try {
    error_code ec;

    m.robot->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] setMotionControlMode(RtCommand) failed: " << ec.message() << std::endl;
      return false;
    }

    m.robot->setOperateMode(rokae::OperateMode::automatic, ec);
    m.robot->setPowerState(true, ec);

    m.robot->stopReceiveRobotState();

    using namespace rokae::RtSupportedFields;
    m.robot->startReceiveRobotState(
      std::chrono::milliseconds(1),
      {jointPos_m, jointVel_m, tau_m, tauExt_inBase}
    );
    NEXUS_COUT << "[Ar5ArmAdapter] Started receiving robot state at 1ms interval." << std::endl;

    auto rt_weak = m.robot->getRtMotionController();
    m.rt_controller = rt_weak.lock();
    if (!m.rt_controller) {
      NEXUS_CERR << "[Ar5ArmAdapter] Failed to get RT motion controller." << std::endl;
      return false;
    }

    {
      std::lock_guard<std::mutex> lk(m.cmd_buffer_mutex);
      m.cmd_buffer.assign(static_cast<size_t>(config_.num_of_dofs), MotorCommand{});
      for (auto& cmd : m.cmd_buffer) {
        cmd.kd = 0.1;
      }
    }

    // ---- 设置SDK回调函数 ----
    // 回调在SDK内部1ms线程中执行，需保证快速完成
    // 捕获this指针，通过共享缓存与读写线程交互
    // 注：robot_ptr 裸指针在回调生命周期内有效（shutdown先stopLoop再reset robot_）
    rokae::ArRobot* robot_ptr = m.robot.get();

// #ifdef AR5_USE_POSITION_CONTROL
//     // ======================== 位置控制模式 ========================
//     // 取MIT命令中的position字段作为关节位置目标下发给SDK

//     // 读取当前关节位置作为位置命令的初始值
//     {
//       error_code pos_ec;
//       auto cur_pos = robot_->jointPos(pos_ec);
//       if (!pos_ec) {
//         std::lock_guard<std::mutex> lk(cb_state_mutex_);
//         cb_joint_pos_ = cur_pos;
//       }
//     }

//     rt_controller_->startMove(rokae::RtControllerMode::jointPosition);
//     rt_controller_->setFilterLimit(true, 10);

//     std::function<rokae::JointPosition(void)> callback = [this, robot_ptr]() -> rokae::JointPosition {
//       using namespace rokae::RtSupportedFields;

//       // 1. 读取关节状态（SDK已在回调前自动updateRobotState）
//       std::array<double, AR5_DOF> q{}, dq{}, tau{};
//       robot_ptr->getStateData(jointPos_m, q);
//       robot_ptr->getStateData(jointVel_m, dq);
//       robot_ptr->getStateData(tau_m, tau);

//       // 2. 将关节状态写入共享缓存（供readThread读取）
//       {
//         std::lock_guard<std::mutex> lk(cb_state_mutex_);
//         cb_joint_pos_ = q;
//         cb_joint_vel_ = dq;
//         cb_joint_torque_ = tau;
//       }

//       // 3. 从共享缓存读取最新MIT命令，提取position字段
//       std::array<double, AR5_DOF> target_pos = q;  // 默认保持当前位置
//       {
//         std::lock_guard<std::mutex> lk(cmd_buffer_mutex_);
//         for (size_t i = 0; i < AR5_DOF && i < cmd_buffer_.size(); ++i) {
//           if (cmd_buffer_[i].kp > 0.0 || cmd_buffer_[i].position != 0.0) {
//             target_pos[i] = cmd_buffer_[i].position;
//           }
//         }
//       }

//       // 4. 构造JointPosition命令返回给SDK
//       rokae::JointPosition jp_cmd({
//         target_pos[0], target_pos[1], target_pos[2], target_pos[3],
//         target_pos[4], target_pos[5], target_pos[6]
//       });

//       // 5. 更新回调计数
//       cb_loop_count_.fetch_add(1, std::memory_order_relaxed);

//       return jp_cmd;
//     };

// #else
    // ======================== 力矩控制模式 ========================
    // 通过MIT公式计算期望力矩下发给SDK

    if (m.has_load) {
        error_code load_ec;
        m.rt_controller->setLoad(m.load, load_ec);
        if (load_ec) {
            NEXUS_CERR << "[Ar5ArmAdapter] setLoad failed: " << load_ec.message() << std::endl;
        } else {
            NEXUS_COUT << "[Ar5ArmAdapter] Load set: mass=" << m.load.mass
                       << "kg, cog=[" << m.load.cog[0] << "," << m.load.cog[1] << "," << m.load.cog[2]
                       << "], inertia=[" << m.load.inertia[0] << "," << m.load.inertia[1] << "," << m.load.inertia[2] << "]"
                       << std::endl;
        }
    }

    m.rt_controller->startMove(rokae::RtControllerMode::torque);

    std::function<rokae::Torque(void)> callback = [this, robot_ptr]() -> rokae::Torque {
      using namespace rokae::RtSupportedFields;

      // 1. 读取关节状态（SDK已在回调前自动updateRobotState）
      std::array<double, AR5_DOF> q{}, dq{}, tau{};
      std::array<double, 6> ext_wrench{};
      robot_ptr->getStateData(jointPos_m, q);
      robot_ptr->getStateData(jointVel_m, dq);
      robot_ptr->getStateData(tau_m, tau);
      robot_ptr->getStateData(tauExt_inBase, ext_wrench);

      // 2. 将关节状态写入共享缓存（供readThread读取）
      {
        std::lock_guard<std::mutex> lk(impl_->cb_state_mutex);
        impl_->cb_joint_pos = q;
        impl_->cb_joint_vel = dq;
        impl_->cb_joint_torque = tau;
        impl_->cb_ext_wrench = ext_wrench;
      }

      // 3. 从共享缓存读取最新MIT命令
      std::vector<MotorCommand> cmds;
      {
        std::lock_guard<std::mutex> lk(impl_->cmd_buffer_mutex);
        cmds = impl_->cmd_buffer;
      }

      // 4. 计算MIT力矩，限制步长和绝对值
      auto tau_desired = computeMitTorque(cmds, q, dq);
      // constexpr std::array<double, AR5_DOF> kMaxTorqueStep = {2.36, 2.36, 0.8, 0.8, 0.36, 0.36, 0.36};
      // constexpr std::array<double, AR5_DOF> kMaxTorqueStep = {54, 54, 33, 33, 9.5, 9.5, 9.5};
      constexpr std::array<double, AR5_DOF> kMaxTorqueStep = {5.4, 5.4, 3.3, 3.3, 0.95, 0.95, 0.95};
      constexpr std::array<double, AR5_DOF> kMaxTorqueAbs  = {108, 108, 66, 66, 19, 19, 19};
      for (size_t i = 0; i < AR5_DOF; ++i) {
        double diff = tau_desired[i] - impl_->prev_tau_desired[i];
        if (diff > kMaxTorqueStep[i]) {
          tau_desired[i] = impl_->prev_tau_desired[i] + kMaxTorqueStep[i];
        } else if (diff < -kMaxTorqueStep[i]) {
          tau_desired[i] = impl_->prev_tau_desired[i] - kMaxTorqueStep[i];
        }
        if (tau_desired[i] > kMaxTorqueAbs[i]) {
          tau_desired[i] = kMaxTorqueAbs[i];
        } else if (tau_desired[i] < -kMaxTorqueAbs[i]) {
          tau_desired[i] = -kMaxTorqueAbs[i];
        }
      }
      impl_->prev_tau_desired = tau_desired;

      // 5. 构造Torque命令返回给SDK
      rokae::Torque torque_cmd(AR5_DOF, 0.0);
      for (size_t i = 0; i < AR5_DOF; ++i) {
        torque_cmd.tau[i] = tau_desired[i];
      }

      // 6. 更新回调计数
      impl_->cb_loop_count.fetch_add(1, std::memory_order_relaxed);

      return torque_cmd;
    };
// #endif  // AR5_USE_POSITION_CONTROL

    // 设置回调：useStateDataInLoop=true，SDK在每次回调前自动更新状态数据
    m.rt_controller->setControlLoop(callback, 0, true);

    m.rt_loop_thread = std::thread([this]() {
      try {
        impl_->rt_controller->startLoop(true);
      } catch (const std::exception& e) {
        NEXUS_CERR << "[Ar5ArmAdapter] startLoop(blocking) exception: " << e.what() << std::endl;
        std::lock_guard<std::mutex> sl(status_mutex_);
        device_status_.status = 2; // ERROR
        device_status_.error_message = "SDK RT loop exception: " + std::string(e.what());
      }
    });
    m.rt_control_started = true;
    NEXUS_COUT << "[Ar5ArmAdapter] SDK callback loop started (blocking in dedicated thread)." << std::endl;

    return true;

  } catch (const std::exception& e) {
    NEXUS_CERR << "[Ar5ArmAdapter] configureDevice exception: " << e.what() << std::endl;
    return false;
  }
}

/**
 * @brief  使能电机（AR5通过上电实现）
 * @retval 成功返回true
 */
bool Ar5ArmAdapter::enableMotors() {
  auto& m = *impl_;
  std::lock_guard<std::mutex> lock(m.robot_mutex);
  if (!m.robot) return false;

  try {
    error_code ec;
    m.robot->setPowerState(true, ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] enableMotors failed: " << ec.message() << std::endl;
      return false;
    }

    {
      std::lock_guard<std::mutex> sl(status_mutex_);
      device_status_.motors_enabled = true;
    }

    return true;

  } catch (const std::exception& e) {
    NEXUS_CERR << "[Ar5ArmAdapter] enableMotors exception: " << e.what() << std::endl;
    return false;
  }
}

/**
 * @brief  失能电机（AR5通过下电实现）
 * @retval 成功返回true
 */
bool Ar5ArmAdapter::disableMotors() {
  auto& m = *impl_;
  std::lock_guard<std::mutex> lock(m.robot_mutex);
  if (!m.robot) return false;

  try {
    error_code ec;
    m.robot->setPowerState(false, ec);
    if (ec) {
      NEXUS_CERR << "[Ar5ArmAdapter] disableMotors failed: " << ec.message() << std::endl;
      return false;
    }

    {
      std::lock_guard<std::mutex> sl(status_mutex_);
      device_status_.motors_enabled = false;
    }

    return true;

  } catch (const std::exception& e) {
    NEXUS_CERR << "[Ar5ArmAdapter] disableMotors exception: " << e.what() << std::endl;
    return false;
  }
}

/**
 * @brief  发送电机命令（MIT模式）
 *         将MIT命令写入共享缓存，供SDK回调线程读取
 * @param  commands MIT命令数组
 * @retval 成功返回true
 */
bool Ar5ArmAdapter::sendMotorCommands(const std::vector<MotorCommand>& commands) {
  std::lock_guard<std::mutex> lk(impl_->cmd_buffer_mutex);
  impl_->cmd_buffer = commands;
  return true;
}

bool Ar5ArmAdapter::readMotorStates(std::vector<MotorState>& states) {
  std::lock_guard<std::mutex> lock(impl_->ar5_states_mutex);
  states = impl_->ar5_latest_states;
  return true;
}

bool Ar5ArmAdapter::readExternalWrench(std::array<double, 6>& wrench) {
  std::lock_guard<std::mutex> lock(impl_->ar5_states_mutex);
  wrench = impl_->ar5_latest_ext_wrench;
  return true;
}

/**
 * @brief  获取设备状态
 * @retval 设备状态结构体
 */
DeviceStatus Ar5ArmAdapter::getDeviceStatus() const {
  std::lock_guard<std::mutex> sl(status_mutex_);
  return device_status_;
}

/*******************************************************************************
 * MIT Control Formula
 ******************************************************************************/

/**
 * @brief  根据MIT命令和当前关节状态计算期望力矩
 *         MIT公式: tau = kp * (q_des - q_cur) + kd * (dq_des - dq_cur) + tau_ff
 * @param  commands MIT命令数组
 * @param  current_pos 当前关节位置 [rad]
 * @param  current_vel 当前关节速度 [rad/s]
 * @return 期望力矩数组 [Nm]
 */
std::array<double, AR5_DOF> Ar5ArmAdapter::computeMitTorque(
    const std::vector<MotorCommand>& commands,
    const std::array<double, AR5_DOF>& current_pos,
    const std::array<double, AR5_DOF>& current_vel) const {

  std::array<double, AR5_DOF> tau_desired;
  tau_desired.fill(0.0);

  for (size_t i = 0; i < AR5_DOF && i < commands.size(); ++i) {
    double kp = commands[i].kp;
    double q_des = commands[i].position;
    double kd = commands[i].kd;
    double dq_des = commands[i].velocity;
    double tau_ff = commands[i].torque;

    double q_cur = current_pos[i];
    double dq_cur = current_vel[i];

    // MIT控制公式
    tau_desired[i] = kp * (q_des - q_cur) + kd * (dq_des - dq_cur) + tau_ff;
  }

  return tau_desired;
}

/*******************************************************************************
 * Thread Functions
 ******************************************************************************/

/**
 * @brief  读线程函数
 *         按yaml配置频率从共享缓存读取关节状态，更新电机状态供上层ROS2发布
 *         关节状态由SDK回调线程以1ms周期写入共享缓存
 */
void Ar5ArmAdapter::readThreadFunc() {
  auto& mi = *impl_;
  auto comm_period = std::chrono::milliseconds(1000 / std::max(1, config_.feedback_rate));
  auto next_read_time = std::chrono::steady_clock::now() + comm_period;

  uint64_t last_cb_count = 0;

  while (thread_running_) {
    try {
      std::array<double, AR5_DOF> pos{}, vel{}, torque{};
      std::array<double, 6> ext_wrench{};
      {
        std::lock_guard<std::mutex> lk(mi.cb_state_mutex);
        pos = mi.cb_joint_pos;
        vel = mi.cb_joint_vel;
        torque = mi.cb_joint_torque;
        ext_wrench = mi.cb_ext_wrench;
      }

      uint64_t current_cb_count = mi.cb_loop_count.load(std::memory_order_relaxed);
      bool cb_active = (current_cb_count > last_cb_count);
      last_cb_count = current_cb_count;

      {
        std::lock_guard<std::mutex> sl(status_mutex_);
        if (cb_active) {
          device_status_.total_packet_count += 1;
        } else if (mi.rt_control_started) {
          device_status_.total_packet_loss_count += 1;
        }
        const uint64_t total = device_status_.total_packet_count + device_status_.total_packet_loss_count;
        device_status_.overall_packet_loss_rate = total ?
          static_cast<float>(device_status_.total_packet_loss_count) / static_cast<float>(total) : 0.0f;
      }

      static uint32_t read_seq_counter = 0;
      if (cb_active) {
        notifyReceivedFrame(++read_seq_counter);
      }

      {
        std::lock_guard<std::mutex> sl(mi.ar5_states_mutex);
        if (mi.ar5_latest_states.size() != static_cast<size_t>(config_.num_of_dofs)) {
          mi.ar5_latest_states.resize(config_.num_of_dofs);
        }

        for (size_t i = 0; i < AR5_DOF && i < static_cast<size_t>(config_.num_of_dofs); ++i) {
          mi.ar5_latest_states[i].position = pos[i];
          mi.ar5_latest_states[i].velocity = vel[i];
          mi.ar5_latest_states[i].effort = torque[i];
          mi.ar5_latest_states[i].temperature = 0.0;
          mi.ar5_latest_states[i].enabled = cb_active;
          mi.ar5_latest_states[i].error_code = 0;
        }

        mi.ar5_latest_ext_wrench = ext_wrench;
      }

    } catch (const std::exception& e) {
      NEXUS_CERR << "[Ar5ArmAdapter] readThreadFunc exception: " << e.what() << std::endl;
    }

    std::this_thread::sleep_until(next_read_time);
    next_read_time += comm_period;
  }
}

/**
 * @brief  写线程函数
 *         按yaml配置频率运行，MIT命令已由sendMotorCommands()直接写入共享缓存
 *         此线程主要用于监控SDK回调循环状态和错误恢复
 */
void Ar5ArmAdapter::writeThreadFunc() {
  auto& mi = *impl_;
  auto comm_period = std::chrono::milliseconds(1000 / std::max(1, config_.feedback_rate));
  auto next_write_time = std::chrono::steady_clock::now() + comm_period;

  while (thread_running_) {
    try {
      if (mi.rt_control_started && mi.rt_controller) {
        if (mi.rt_controller->hasMotionError()) {
          NEXUS_CERR << "[Ar5ArmAdapter] SDK motion error detected!" << std::endl;
          std::lock_guard<std::mutex> sl(status_mutex_);
          device_status_.status = 2; // ERROR
          device_status_.error_code = 100;
          device_status_.error_message = "SDK RT motion error";
        }
      }

    } catch (const std::exception& e) {
      NEXUS_CERR << "[Ar5ArmAdapter] writeThreadFunc exception: " << e.what() << std::endl;
    }

    std::this_thread::sleep_until(next_write_time);
    next_write_time += comm_period;
  }
}

} // namespace teleop_adapter
