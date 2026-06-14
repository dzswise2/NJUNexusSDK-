/*
 * @FilePath: teleop_adapter/src/adapters/ar5_gripper_adapter.cpp
 * @Description: AR5机械臂适配器（带夹爪）实现
 *               8自由度 = 7 AR5关节 + 1 夹爪关节
 *               夹爪关节位置值直接透传给 setGripperPosition(),
 *               闭环控制：周期性查询 getGripperStatus().position 作为状态上报。
 *
 *               四线程架构:
 *               1) SDK回调线程(1ms): 原子地读状态+计算MIT扭矩+返回控制命令
 *               2) writeThreadFunc: 按yaml频率检测SDK错误
 *               3) readThreadFunc: 按yaml频率从共享缓存读取状态（7臂+1夹爪）
 *               4) gripperControlLoop: 异步执行 setGripperPosition/getGripperPosition(HTTP)
 *
 * Copyright (c) 2025 by Infra Embedded, All Rights Reserved.
 */

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "teleop_adapter/adapters/ar5_gripper_adapter.hpp"
#include "rokae/robot.h"
#include "rokae/motion_control_rt.h"
#include "rokae/data_types.h"
#include "teleop_adapter/peripherals/remote_client.h"
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
 * Impl — hidden state
 ******************************************************************************/

struct Ar5GripperAdapter::Impl {
    // ---- Rokae SDK ----
    std::unique_ptr<rokae::ArRobot> robot;
    std::shared_ptr<rokae::RtMotionControlCobot<AR5_ARM_DOF>> rt_controller;
    std::mutex robot_mutex;

    std::string robot_ip;
    std::string local_ip;
    std::string peripheral_server_url;
    std::array<double, AR5_ARM_DOF> init_joint_positions;
    rokae::Load load{};
    bool has_load{false};

    // ---- 命令缓存 (MIT) ----
    std::vector<MotorCommand> cmd_buffer;
    std::mutex cmd_buffer_mutex;

    // ---- SDK 回调共享缓存 (7 arm joints) ----
    std::array<double, AR5_ARM_DOF> cb_joint_pos{};
    std::array<double, AR5_ARM_DOF> cb_joint_vel{};
    std::array<double, AR5_ARM_DOF> cb_joint_torque{};
    std::array<double, 6> cb_ext_wrench{};
    std::mutex cb_state_mutex;

    // ---- 夹爪异步控制 ----
    std::atomic<int> gripper_pos_cmd{0};
    std::atomic<bool> gripper_cmd_pending{true};  // 首次循环即写入初始角度
    int gripper_feedback_pos{0};
    std::mutex gripper_mutex;
    std::thread gripper_loop_thread;
    std::atomic<bool> gripper_loop_running{false};

    // ---- 最终输出状态 ----
    std::vector<MotorState> latest_states;
    std::array<double, 6> latest_ext_wrench{};
    std::mutex states_mutex;

    // ---- SDK 控制状态 ----
    std::atomic<bool> rt_control_started{false};
    std::atomic<uint64_t> cb_loop_count{0};

    std::array<double, AR5_ARM_DOF> prev_tau_desired{};

    std::thread rt_loop_thread;

    // ---- 外设客户端 ----
    std::unique_ptr<peripherals_remote::PeripheralsClient> peripheral_client;
};

/*******************************************************************************
 * Constructor & Destructor
 ******************************************************************************/

Ar5GripperAdapter::Ar5GripperAdapter(
    const DeviceAdapterConfig& config,
    const std::string& robot_ip,
    const std::string& local_ip,
    const std::string& peripheral_server_url,
    const std::array<double, AR5_ARM_DOF>& init_joint_positions,
    double load_mass,
    const std::array<double, 3>& load_cog,
    const std::array<double, 3>& load_inertia)
  : DeviceAdapter(config, nullptr),
    impl_(std::make_unique<Impl>()) {

    impl_->robot_ip = robot_ip;
    impl_->local_ip = local_ip;
    impl_->peripheral_server_url = peripheral_server_url;
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
        std::lock_guard<std::mutex> sl(impl_->states_mutex);
        impl_->latest_states.resize(config_.num_of_dofs);
        for (auto& state : impl_->latest_states) {
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

Ar5GripperAdapter::~Ar5GripperAdapter() {
    shutdown();
}

/*******************************************************************************
 * Public Methods - DeviceAdapter Interface
 ******************************************************************************/

void Ar5GripperAdapter::shutdown() {
    DeviceAdapter::shutdown();

    auto& m = *impl_;

    // 先停止夹爪异步控制线程（使用 peripheral_client）
    m.gripper_loop_running = false;
    if (m.gripper_loop_thread.joinable()) {
        m.gripper_loop_thread.join();
    }

    std::lock_guard<std::mutex> lock(m.robot_mutex);
    if (m.robot) {
        error_code ec;
        try {
            if (m.rt_controller && m.rt_control_started) {
                try {
                    m.rt_controller->stopLoop();
                } catch (const std::exception& e) {
                    NEXUS_CERR << "[Ar5GripperAdapter] stopLoop exception: " << e.what() << std::endl;
                }
                if (m.rt_loop_thread.joinable()) {
                    m.rt_loop_thread.join();
                }
                try {
                    m.rt_controller->stopMove();
                } catch (const std::exception& e) {
                    NEXUS_CERR << "[Ar5GripperAdapter] stopMove exception: " << e.what() << std::endl;
                }
                m.rt_control_started = false;
            }

            m.robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
            if (ec) {
                NEXUS_CERR << "[Ar5GripperAdapter] setMotionControlMode(NrtCommand) failed: " << ec.message() << std::endl;
            }

            m.robot->setOperateMode(rokae::OperateMode::manual, ec);
            m.robot->setPowerState(false, ec);
            m.robot->stopReceiveRobotState();
        } catch (const std::exception& e) {
            NEXUS_CERR << "[Ar5GripperAdapter] shutdown exception: " << e.what() << std::endl;
        }

        m.rt_controller.reset();
        m.robot.reset();
    }

    if (m.peripheral_client) {
        m.peripheral_client->disconnect();
        m.peripheral_client.reset();
    }
}

bool Ar5GripperAdapter::deviceHandshake() {
    auto& m = *impl_;
    std::lock_guard<std::mutex> lock(m.robot_mutex);

    try {
        // ---- 连接 AR5 ----
        NEXUS_COUT << "[Ar5GripperAdapter] Connecting to AR5 at " << m.robot_ip
                   << " (local: " << m.local_ip << ")..." << std::endl;
        m.robot = std::make_unique<rokae::ArRobot>(m.robot_ip, m.local_ip);
        NEXUS_COUT << "[Ar5GripperAdapter] Connected to AR5 successfully." << std::endl;

        error_code ec;

        m.robot->recoverState(1, ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] recoverState(estop) failed: " << ec.message() << std::endl;
        } else {
            NEXUS_COUT << "[Ar5GripperAdapter] E-stop recovered." << std::endl;
        }

        m.robot->clearServoAlarm(ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] clearServoAlarm failed: " << ec.message() << std::endl;
        } else {
            NEXUS_COUT << "[Ar5GripperAdapter] Servo alarm cleared." << std::endl;
        }

        m.robot->moveReset(ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] moveReset failed: " << ec.message() << std::endl;
        } else {
            NEXUS_COUT << "[Ar5GripperAdapter] Move reset done." << std::endl;
        }

        m.robot->setMotionControlMode(rokae::MotionControlMode::NrtCommand, ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] setMotionControlMode(NrtCommand) failed: " << ec.message() << std::endl;
            return false;
        }

        m.robot->setOperateMode(rokae::OperateMode::automatic, ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] setOperateMode(automatic) failed: " << ec.message() << std::endl;
            return false;
        }

        m.robot->calibrateForceSensor(true, 0, ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] calibrateForceSensor failed: " << ec.message() << std::endl;
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        NEXUS_COUT << "[Ar5GripperAdapter] Force sensor calibration done." << std::endl;

        m.robot->setPowerState(true, ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] setPowerState(true) failed: " << ec.message() << std::endl;
            return false;
        }

        NEXUS_COUT << "[Ar5GripperAdapter] Moving to initial joint position..." << std::endl;
        rokae::JointPosition target_pos({
            m.init_joint_positions[0], m.init_joint_positions[1], m.init_joint_positions[2],
            m.init_joint_positions[3], m.init_joint_positions[4], m.init_joint_positions[5],
            m.init_joint_positions[6]
        });
        rokae::MoveAbsJCommand move_cmd(target_pos, 600, 0);
        std::string cmd_id;
        m.robot->moveAppend(move_cmd, cmd_id, ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] moveAppend failed: " << ec.message() << std::endl;
            return false;
        }

        m.robot->moveStart(ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] moveStart failed: " << ec.message() << std::endl;
            return false;
        }

        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            auto state = m.robot->operationState(ec);
            if (state == rokae::OperationState::idle || state == rokae::OperationState::unknown) {
                break;
            }
        }
        NEXUS_COUT << "[Ar5GripperAdapter] Reached initial position." << std::endl;

        // ---- 连接外设服务器 ----
        if (!m.peripheral_server_url.empty()) {
            NEXUS_COUT << "[Ar5GripperAdapter] Connecting to peripheral server at "
                       << m.peripheral_server_url << "..." << std::endl;
            m.peripheral_client = std::make_unique<peripherals_remote::PeripheralsClient>();
            if (!m.peripheral_client->connect(m.peripheral_server_url)) {
                NEXUS_CERR << "[Ar5GripperAdapter] Failed to connect to peripheral server." << std::endl;
                return false;
            }
            NEXUS_COUT << "[Ar5GripperAdapter] Peripheral server connected." << std::endl;
        }

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
        NEXUS_CERR << "[Ar5GripperAdapter] deviceHandshake exception: " << e.what() << std::endl;
        m.robot.reset();
        return false;
    }
}

bool Ar5GripperAdapter::configureDevice() {
    auto& m = *impl_;
    std::lock_guard<std::mutex> lock(m.robot_mutex);
    if (!m.robot) return false;

    try {
        error_code ec;

        m.robot->setMotionControlMode(rokae::MotionControlMode::RtCommand, ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] setMotionControlMode(RtCommand) failed: " << ec.message() << std::endl;
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
        NEXUS_COUT << "[Ar5GripperAdapter] Started receiving robot state at 1ms interval." << std::endl;

        auto rt_weak = m.robot->getRtMotionController();
        m.rt_controller = rt_weak.lock();
        if (!m.rt_controller) {
            NEXUS_CERR << "[Ar5GripperAdapter] Failed to get RT motion controller." << std::endl;
            return false;
        }

        {
            std::lock_guard<std::mutex> lk(m.cmd_buffer_mutex);
            m.cmd_buffer.assign(static_cast<size_t>(config_.num_of_dofs), MotorCommand{});
            for (auto& cmd : m.cmd_buffer) {
                cmd.kd = 0.1;
            }
        }

        // ---- 力矩控制模式 (7 arm joints only) ----
        rokae::ArRobot* robot_ptr = m.robot.get();

        if (m.has_load) {
            error_code load_ec;
            m.rt_controller->setLoad(m.load, load_ec);
            if (load_ec) {
                NEXUS_CERR << "[Ar5GripperAdapter] setLoad failed: " << load_ec.message() << std::endl;
            } else {
                NEXUS_COUT << "[Ar5GripperAdapter] Load set: mass=" << m.load.mass
                           << "kg, cog=[" << m.load.cog[0] << "," << m.load.cog[1] << "," << m.load.cog[2]
                           << "], inertia=[" << m.load.inertia[0] << "," << m.load.inertia[1] << "," << m.load.inertia[2] << "]"
                           << std::endl;
            }
        }

        m.rt_controller->startMove(rokae::RtControllerMode::torque);

        std::function<rokae::Torque(void)> callback = [this, robot_ptr]() -> rokae::Torque {
            using namespace rokae::RtSupportedFields;

            std::array<double, AR5_ARM_DOF> q{}, dq{}, tau{};
            std::array<double, 6> ext_wrench{};
            robot_ptr->getStateData(jointPos_m, q);
            robot_ptr->getStateData(jointVel_m, dq);
            robot_ptr->getStateData(tau_m, tau);
            robot_ptr->getStateData(tauExt_inBase, ext_wrench);

            {
                std::lock_guard<std::mutex> lk(impl_->cb_state_mutex);
                impl_->cb_joint_pos = q;
                impl_->cb_joint_vel = dq;
                impl_->cb_joint_torque = tau;
                impl_->cb_ext_wrench = ext_wrench;
            }

            std::vector<MotorCommand> cmds;
            {
                std::lock_guard<std::mutex> lk(impl_->cmd_buffer_mutex);
                cmds = impl_->cmd_buffer;
            }

            auto tau_desired = computeMitTorque(cmds, q, dq);

            constexpr std::array<double, AR5_ARM_DOF> kMaxTorqueStep = {5.4, 5.4, 3.3, 3.3, 0.95, 0.95, 0.95};
            constexpr std::array<double, AR5_ARM_DOF> kMaxTorqueAbs  = {108, 108, 66, 66, 19, 19, 19};
            for (size_t i = 0; i < AR5_ARM_DOF; ++i) {
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

            rokae::Torque torque_cmd(AR5_ARM_DOF, 0.0);
            for (size_t i = 0; i < AR5_ARM_DOF; ++i) {
                torque_cmd.tau[i] = tau_desired[i];
            }

            impl_->cb_loop_count.fetch_add(1, std::memory_order_relaxed);

            return torque_cmd;
        };

        m.rt_controller->setControlLoop(callback, 0, true);

        m.rt_loop_thread = std::thread([this]() {
            try {
                impl_->rt_controller->startLoop(true);
            } catch (const std::exception& e) {
                NEXUS_CERR << "[Ar5GripperAdapter] startLoop(blocking) exception: " << e.what() << std::endl;
                std::lock_guard<std::mutex> sl(status_mutex_);
                device_status_.status = 2; // ERROR
                device_status_.error_message = "SDK RT loop exception: " + std::string(e.what());
            }
        });
        m.rt_control_started = true;
        NEXUS_COUT << "[Ar5GripperAdapter] SDK callback loop started (blocking in dedicated thread)." << std::endl;

        // 启动夹爪异步控制线程
        m.gripper_loop_running = true;
        m.gripper_loop_thread = std::thread(&Ar5GripperAdapter::gripperControlLoop, this);
        NEXUS_COUT << "[Ar5GripperAdapter] Gripper control loop started." << std::endl;

        return true;

    } catch (const std::exception& e) {
        NEXUS_CERR << "[Ar5GripperAdapter] configureDevice exception: " << e.what() << std::endl;
        return false;
    }
}

bool Ar5GripperAdapter::enableMotors() {
    auto& m = *impl_;
    std::lock_guard<std::mutex> lock(m.robot_mutex);
    if (!m.robot) return false;

    try {
        error_code ec;
        m.robot->setPowerState(true, ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] enableMotors failed: " << ec.message() << std::endl;
            return false;
        }

        {
            std::lock_guard<std::mutex> sl(status_mutex_);
            device_status_.motors_enabled = true;
        }

        return true;

    } catch (const std::exception& e) {
        NEXUS_CERR << "[Ar5GripperAdapter] enableMotors exception: " << e.what() << std::endl;
        return false;
    }
}

bool Ar5GripperAdapter::disableMotors() {
    auto& m = *impl_;
    std::lock_guard<std::mutex> lock(m.robot_mutex);
    if (!m.robot) return false;

    try {
        error_code ec;
        m.robot->setPowerState(false, ec);
        if (ec) {
            NEXUS_CERR << "[Ar5GripperAdapter] disableMotors failed: " << ec.message() << std::endl;
            return false;
        }

        {
            std::lock_guard<std::mutex> sl(status_mutex_);
            device_status_.motors_enabled = false;
        }

        return true;

    } catch (const std::exception& e) {
        NEXUS_CERR << "[Ar5GripperAdapter] disableMotors exception: " << e.what() << std::endl;
        return false;
    }
}

bool Ar5GripperAdapter::sendMotorCommands(const std::vector<MotorCommand>& commands) {
    auto& m = *impl_;

    // 写入 MIT 命令缓存 (关节 0-6)
    {
        std::lock_guard<std::mutex> lk(m.cmd_buffer_mutex);
        m.cmd_buffer = commands;
    }

    // 写入夹爪异步命令 (关节 7)，fire-and-forget，开合度直接透传
    if (commands.size() > IDX_GRIPPER) {
        int pos = static_cast<int>(commands[IDX_GRIPPER].position);
        m.gripper_pos_cmd.store(pos, std::memory_order_relaxed);
        m.gripper_cmd_pending.store(true, std::memory_order_relaxed);
    }

    return true;
}

bool Ar5GripperAdapter::readMotorStates(std::vector<MotorState>& states) {
    std::lock_guard<std::mutex> lock(impl_->states_mutex);
    states = impl_->latest_states;
    return true;
}

bool Ar5GripperAdapter::readExternalWrench(std::array<double, 6>& wrench) {
    std::lock_guard<std::mutex> lock(impl_->states_mutex);
    wrench = impl_->latest_ext_wrench;
    return true;
}

DeviceStatus Ar5GripperAdapter::getDeviceStatus() const {
    std::lock_guard<std::mutex> sl(status_mutex_);
    return device_status_;
}

/*******************************************************************************
 * MIT Control Formula (7 arm joints only)
 ******************************************************************************/

std::array<double, AR5_ARM_DOF> Ar5GripperAdapter::computeMitTorque(
    const std::vector<MotorCommand>& commands,
    const std::array<double, AR5_ARM_DOF>& current_pos,
    const std::array<double, AR5_ARM_DOF>& current_vel) const {

    std::array<double, AR5_ARM_DOF> tau_desired;
    tau_desired.fill(0.0);

    for (size_t i = 0; i < AR5_ARM_DOF && i < commands.size(); ++i) {
        double kp = commands[i].kp;
        double q_des = commands[i].position;
        double kd = commands[i].kd;
        double dq_des = commands[i].velocity;
        double tau_ff = commands[i].torque;

        double q_cur = current_pos[i];
        double dq_cur = current_vel[i];

        tau_desired[i] = kp * (q_des - q_cur) + kd * (dq_des - dq_cur) + tau_ff;
    }

    return tau_desired;
}

/*******************************************************************************
 * Thread Functions
 ******************************************************************************/

void Ar5GripperAdapter::readThreadFunc() {
    auto& mi = *impl_;
    auto comm_period = std::chrono::milliseconds(1000 / std::max(1, config_.feedback_rate));
    auto next_read_time = std::chrono::steady_clock::now() + comm_period;

    uint64_t last_cb_count = 0;

    while (thread_running_) {
        try {
            // 读取 SDK 回调共享缓存 (7 arm joints)
            std::array<double, AR5_ARM_DOF> pos{}, vel{}, torque{};
            std::array<double, 6> ext_wrench{};
            {
                std::lock_guard<std::mutex> lk(mi.cb_state_mutex);
                pos = mi.cb_joint_pos;
                vel = mi.cb_joint_vel;
                torque = mi.cb_joint_torque;
                ext_wrench = mi.cb_ext_wrench;
            }

            // 读取夹爪反馈（由 gripperControlLoop 异步更新）
            int gripper_feedback;
            {
                std::lock_guard<std::mutex> lk(mi.gripper_mutex);
                gripper_feedback = mi.gripper_feedback_pos;
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
                std::lock_guard<std::mutex> sl(mi.states_mutex);
                if (mi.latest_states.size() != static_cast<size_t>(config_.num_of_dofs)) {
                    mi.latest_states.resize(config_.num_of_dofs);
                }

                // 关节 0-6: 从 SDK 读取
                for (size_t i = 0; i < AR5_ARM_DOF && i < static_cast<size_t>(config_.num_of_dofs); ++i) {
                    mi.latest_states[i].position = pos[i];
                    mi.latest_states[i].velocity = vel[i];
                    mi.latest_states[i].effort = torque[i];
                    mi.latest_states[i].temperature = 0.0;
                    mi.latest_states[i].enabled = cb_active;
                    mi.latest_states[i].error_code = 0;
                }

                // 关节 7: 夹爪 (闭环，上报真实位置)
                if (IDX_GRIPPER < static_cast<size_t>(config_.num_of_dofs)) {
                    mi.latest_states[IDX_GRIPPER].position = static_cast<double>(gripper_feedback);
                    mi.latest_states[IDX_GRIPPER].velocity = 0.0;
                    mi.latest_states[IDX_GRIPPER].effort = 0.0;
                    mi.latest_states[IDX_GRIPPER].temperature = 0.0;
                    mi.latest_states[IDX_GRIPPER].enabled = mi.gripper_loop_running.load(std::memory_order_relaxed);
                    mi.latest_states[IDX_GRIPPER].error_code = 0;
                }

                mi.latest_ext_wrench = ext_wrench;
            }

        } catch (const std::exception& e) {
            NEXUS_CERR << "[Ar5GripperAdapter] readThreadFunc exception: " << e.what() << std::endl;
        }

        std::this_thread::sleep_until(next_read_time);
        next_read_time += comm_period;
    }
}

void Ar5GripperAdapter::writeThreadFunc() {
    auto& mi = *impl_;
    auto comm_period = std::chrono::milliseconds(1000 / std::max(1, config_.feedback_rate));
    auto next_write_time = std::chrono::steady_clock::now() + comm_period;

    while (thread_running_) {
        try {
            if (mi.rt_control_started && mi.rt_controller) {
                if (mi.rt_controller->hasMotionError()) {
                    NEXUS_CERR << "[Ar5GripperAdapter] SDK motion error detected!" << std::endl;
                    std::lock_guard<std::mutex> sl(status_mutex_);
                    device_status_.status = 2; // ERROR
                    device_status_.error_code = 100;
                    device_status_.error_message = "SDK RT motion error";
                }
            }

        } catch (const std::exception& e) {
            NEXUS_CERR << "[Ar5GripperAdapter] writeThreadFunc exception: " << e.what() << std::endl;
        }

        std::this_thread::sleep_until(next_write_time);
        next_write_time += comm_period;
    }
}

void Ar5GripperAdapter::gripperControlLoop() {
    auto& m = *impl_;
    int query_counter = 0;
    constexpr int kQueryInterval = 3;  // 50Hz / 3 ≈ 17Hz gripper feedback query

    while (m.gripper_loop_running.load(std::memory_order_relaxed)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // 写入：仅当有新命令时执行
        if (m.gripper_cmd_pending.exchange(false, std::memory_order_relaxed)) {
            int pos = m.gripper_pos_cmd.load(std::memory_order_relaxed);
            if (m.peripheral_client && m.peripheral_client->isConnected()) {
                m.peripheral_client->setGripperPosition(pos);
            }
        }

        // 读取：周期性查询真实位置（闭环反馈）
        if (++query_counter >= kQueryInterval) {
            query_counter = 0;
            if (m.peripheral_client && m.peripheral_client->isConnected()) {
                int gripper_pos = 0;
                if (m.peripheral_client->getGripperPosition(gripper_pos)) {
                    std::lock_guard<std::mutex> lk(m.gripper_mutex);
                    m.gripper_feedback_pos = gripper_pos;
                }
            }
        }
    }
}

} // namespace teleop_adapter
