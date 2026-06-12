/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: teleop_adapter/src/nodes/teleop_arm_driver_node.cpp
 * @Description: 远程操控机械臂驱动节点实现，管理多个机械臂适配器，实现ROS2与硬件之间的双向通信
 *
 * Copyright (c) 2025 by Pegasus, All Rights Reserved.
 */

/*******************************************************************************
 * Include
 ******************************************************************************/
#include "teleop_adapter/nodes/detail/teleop_arm_driver_node_impl.hpp"
#ifdef BUILD_Y1_ADAPTER
#include "teleop_adapter/adapters/y1_arm_adapter.hpp"
#endif
#include "teleop_adapter/adapters/ar5_arm_adapter.hpp"
#include "teleop_adapter/adapters/ar5_suction_cup_adapter.hpp"
#include "teleop_adapter/adapters/ar5_gripper_adapter.hpp"
#include <chrono>
#include <algorithm>
#include <set>
#include <cmath>

using namespace std::chrono_literals;

/*******************************************************************************
 * Namespace
 ******************************************************************************/
namespace teleop_adapter {

TeleopArmDriverNode::Impl::Impl(TeleopArmDriverNode* owner) : owner_(owner) {}

/*******************************************************************************
 * Constructor & Destructor
 ******************************************************************************/

/**
 * @brief  构造函数
 * @param  options ROS2节点选项
 * @retval null
 */
TeleopArmDriverNode::TeleopArmDriverNode(const rclcpp::NodeOptions& options)
    : Node("teleop_arm_adapter",
           rclcpp::NodeOptions(options)
               .allow_undeclared_parameters(true)
               .automatically_declare_parameters_from_overrides(true)),
        impl_(std::make_unique<Impl>(this))
{
    RCLCPP_INFO(this->get_logger(), "Initializing Teleop Arm Driver Node...");
}

/**
 * @brief  析构函数
 * @param  null
 * @retval null
 */
TeleopArmDriverNode::~TeleopArmDriverNode()
{
    impl_->shutdown();
}

/*******************************************************************************
 * Public Methods
 ******************************************************************************/

/**
 * @brief  初始化节点
 * @param  null
 * @retval 初始化成功返回true，否则返回false
 */
bool TeleopArmDriverNode::initialize()
{
    return impl_->initialize();
}

bool TeleopArmDriverNode::Impl::initialize()
{
    if (is_initialized_.load()) {
        RCLCPP_WARN(owner_->get_logger(), "Node already initialized");
        return true;
    }

    // 参数已经在构造函数中自动声明，无需手动声明

    if (!loadGlobalConfig()) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed to load global configuration");
        return false;
    }

    if (!discoverAndLoadArmConfigs()) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed to discover arm configurations");
        return false;
    }

    if (!validateConfiguration()) {
        RCLCPP_ERROR(owner_->get_logger(), "Configuration validation failed");
        return false;
    }

    if (!initializeAdapters()) {
        RCLCPP_ERROR(owner_->get_logger(), "Failed to initialize adapters");
        return false;
    }

    initializeRosInterface();

    // 只有 teleop 适配器才需要外设接口，Y1 不启用
    if (adapter_type_ == "teleop") {
        initializePeripheralInterface();  // 初始化外设接口
    } else {
        RCLCPP_INFO(owner_->get_logger(),
                    "Peripheral interface disabled for adapter_type = %s",
                    adapter_type_.c_str());
    }

    is_initialized_.store(true);
    is_running_.store(true);

    RCLCPP_INFO(owner_->get_logger(), "Teleop Arm Driver Node initialized successfully");
    RCLCPP_INFO(owner_->get_logger(), "Managing %zu arm(s) with %d total DOFs",
                adapters_.size(), total_num_of_dofs_);

    return true;
}

/**
 * @brief  关闭节点
 * @param  null
 * @retval null
 */
void TeleopArmDriverNode::shutdown()
{
    impl_->shutdown();
}

void TeleopArmDriverNode::Impl::shutdown()
{
    if (!is_running_.load()) {
        return;
    }

    RCLCPP_INFO(owner_->get_logger(), "Shutting down Teleop Arm Driver Node...");

    is_running_.store(false);

    // 取消定时器
    if (publish_timer_) {
        publish_timer_->cancel();
    }
    if (status_timer_) {
        status_timer_->cancel();
    }
    if (peripheral_timer_) {
        peripheral_timer_->cancel();
    }

    // 关闭所有适配器
    std::lock_guard<std::mutex> lock(adapters_mutex_);
    for (auto& [name, adapter] : adapters_) {
        if (adapter) {
            RCLCPP_INFO(owner_->get_logger(), "Shutting down adapter: %s", name.c_str());
            adapter->disableMotors();
            adapter->shutdown();
        }
    }
    adapters_.clear();

    is_initialized_.store(false);
    RCLCPP_INFO(owner_->get_logger(), "Teleop Arm Driver Node shutdown complete");
}

/*******************************************************************************
 * Parameter Functions
 ******************************************************************************/

/**
 * @brief  声明ROS2参数
 * @param  null
 * @retval null
 */
void TeleopArmDriverNode::Impl::declareParameters()
{
    owner_->declare_parameter<bool>("auto_enable", true);
    owner_->declare_parameter<int>("num_of_dofs", 6);
    owner_->declare_parameter<std::vector<std::string>>("joint_names", std::vector<std::string>());
    owner_->declare_parameter<std::vector<int64_t>>("joint_mapping", std::vector<int64_t>());
    
    owner_->declare_parameter<std::string>("joint_mit_control_topic", "rt/teleop/table_arm/joint_cmd");
    owner_->declare_parameter<std::string>("joint_state_topic", "rt/teleop/table_arm/joint_state");
    owner_->declare_parameter<std::string>("status_topic", "rt/teleop/table_arm/status");

    owner_->declare_parameter<std::string>("robot_name", "");
    // 新增：适配器类型参数，默认 teleop 或 y1
    owner_->declare_parameter<std::string>("adapter_type", "teleop");

    // 新增：外设相关topic参数
    owner_->declare_parameter<std::string>("peripheral_command_topic", "/rt/teleop/peripheral/command");
    owner_->declare_parameter<std::string>("gripper_key_state_topic", "/rt/teleop/gripper_key_state");

    RCLCPP_INFO(owner_->get_logger(), "Parameters declared");
}

/**
 * @brief  加载全局配置参数
 * @param  null
 * @retval 加载成功返回true，否则返回false
 */
bool TeleopArmDriverNode::Impl::loadGlobalConfig()
{
    try {
        // 读取基本配置
        auto_enable_ = owner_->get_parameter("auto_enable").as_bool();
        total_num_of_dofs_ = owner_->get_parameter("num_of_dofs").as_int();
        joint_names_ = owner_->get_parameter("joint_names").as_string_array();
        
        // 转换关节映射（int64 -> int）
        auto mapping_int64 = owner_->get_parameter("joint_mapping").as_integer_array();
        joint_mapping_.clear();
        for (auto val : mapping_int64) {
            joint_mapping_.push_back(static_cast<int>(val));
        }
        
        // 读取主题名称
        // 先读取 robot_name 和基础 topic 名
        robot_name_ = owner_->get_parameter("robot_name").as_string();
        auto base_joint_cmd_topic    = owner_->get_parameter("joint_mit_control_topic").as_string();
        auto base_joint_state_topic  = owner_->get_parameter("joint_state_topic").as_string();
        auto base_status_topic       = owner_->get_parameter("status_topic").as_string();

        // 统一处理前缀，robot_name_ 为空就不加
        auto prefix = std::string{};
        if (!robot_name_.empty()) {
            prefix = "/" + robot_name_;
        }

        joint_cmd_topic_   = prefix + "/" + base_joint_cmd_topic;
        joint_state_topic_ = prefix + "/" + base_joint_state_topic;
        status_topic_      = prefix + "/" + base_status_topic;

        if (owner_->has_parameter("external_wrench_topic")) {
            auto base_ext_wrench_topic = owner_->get_parameter("external_wrench_topic").as_string();
            external_wrench_topic_ = prefix + "/" + base_ext_wrench_topic;
        }
        // 新增：读取适配器类型
        adapter_type_ = owner_->get_parameter("adapter_type").as_string();

        RCLCPP_INFO(owner_->get_logger(), "  Adapter Type: %s", adapter_type_.c_str());
        RCLCPP_INFO(owner_->get_logger(), "Global configuration loaded:");
        RCLCPP_INFO(owner_->get_logger(), "  Total DOFs: %d", total_num_of_dofs_);
        RCLCPP_INFO(owner_->get_logger(), "  Auto Enable: %s", auto_enable_ ? "true" : "false");
        RCLCPP_INFO(owner_->get_logger(), "  Robot Name: %s", robot_name_.c_str());
        RCLCPP_INFO(owner_->get_logger(), "  Joint cmd topic: %s", joint_cmd_topic_.c_str());
        RCLCPP_INFO(owner_->get_logger(), "  Joint state topic: %s", joint_state_topic_.c_str());
        RCLCPP_INFO(owner_->get_logger(), "  Status topic: %s", status_topic_.c_str());
        
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(owner_->get_logger(), "Error loading global config: %s", e.what());
        return false;
    }
}

/**
 * @brief  发现并加载所有机械臂配置
 * @param  null
 * @retval 加载成功返回true，否则返回false
 */
bool TeleopArmDriverNode::Impl::discoverAndLoadArmConfigs()
{
    try {
        auto param_names = owner_->list_parameters({}, 0).names;
        
        // 使用 vector 保持参数定义顺序，用 set 去重
        std::vector<std::string> arm_names_vec;
        std::set<std::string> arm_names_set;
        
        for (const auto& param_name : param_names) {
            size_t pos = param_name.find("_arm_");
            if (pos != std::string::npos) {
                std::string arm_name = param_name.substr(0, pos + 4);
                // 只有在 set 中不存在时才添加到 vector，保持首次出现的顺序
                if (arm_names_set.insert(arm_name).second) {
                    arm_names_vec.push_back(arm_name);
                }
            }
        }
        
        if (arm_names_vec.empty()) {
            RCLCPP_ERROR(owner_->get_logger(), "No arm configurations found");
            return false;
        }
        
        RCLCPP_INFO(owner_->get_logger(), "Discovered %zu arm(s) (in parameter definition order):", arm_names_vec.size());
        
        for (const auto& arm_name : arm_names_vec) {
            std::string port_param = arm_name + "_port";
            std::string rate_param = arm_name + "_feedback_rate";
            std::string dofs_param = arm_name + "_num_of_dofs";
            
            // 检查三个必需参数是否都存在
            if (!owner_->has_parameter(port_param)) {
                RCLCPP_ERROR(owner_->get_logger(), "  [%s] Error: missing parameter '%s'", 
                           arm_name.c_str(), port_param.c_str());
                return false;
            }
            if (!owner_->has_parameter(rate_param)) {
                RCLCPP_ERROR(owner_->get_logger(), "  [%s] Error: missing parameter '%s'", 
                           arm_name.c_str(), rate_param.c_str());
                return false;
            }
            if (!owner_->has_parameter(dofs_param)) {
                RCLCPP_ERROR(owner_->get_logger(), "  [%s] Error: missing parameter '%s'", 
                           arm_name.c_str(), dofs_param.c_str());
                return false;
            }
            
            ArmConfig config;
            config.name = arm_name.substr(0, arm_name.size() - 4);
            config.port = owner_->get_parameter(port_param).as_string();
            config.feedback_rate = owner_->get_parameter(rate_param).as_int();
            config.num_of_dofs = owner_->get_parameter(dofs_param).as_int();

            if (config.port.empty()) {
                RCLCPP_ERROR(owner_->get_logger(), "  [%s] Error: port is empty", config.name.c_str());
                return false;
            }
            
            // AR5 专用参数（可选），ar5 和 ar5_suction_cup 都需要
            if (adapter_type_ == "ar5" || adapter_type_ == "ar5_suction_cup" || adapter_type_ == "ar5_gripper") {
                std::string local_ip_param = arm_name + "_local_ip";
                if (owner_->has_parameter(local_ip_param)) {
                    config.local_ip = owner_->get_parameter(local_ip_param).as_string();
                }
                std::string init_pos_param = arm_name + "_init_joint_positions";
                if (owner_->has_parameter(init_pos_param)) {
                    auto init_pos_values = owner_->get_parameter(init_pos_param).as_double_array();
                    config.init_joint_positions = init_pos_values;
                }
                // 负载参数（可选）
                std::string load_mass_param = arm_name + "_load_mass";
                if (owner_->has_parameter(load_mass_param)) {
                    config.load_mass = owner_->get_parameter(load_mass_param).as_double();
                }
                std::string load_cog_param = arm_name + "_load_cog";
                if (owner_->has_parameter(load_cog_param)) {
                    auto cog_values = owner_->get_parameter(load_cog_param).as_double_array();
                    for (size_t i = 0; i < 3 && i < cog_values.size(); ++i) {
                        config.load_cog[i] = cog_values[i];
                    }
                }
                std::string load_inertia_param = arm_name + "_load_inertia";
                if (owner_->has_parameter(load_inertia_param)) {
                    auto inertia_values = owner_->get_parameter(load_inertia_param).as_double_array();
                    for (size_t i = 0; i < 3 && i < inertia_values.size(); ++i) {
                        config.load_inertia[i] = inertia_values[i];
                    }
                }
            }

            arm_configs_.push_back(config);
            
            RCLCPP_INFO(owner_->get_logger(), "  [%s]: port=%s, rate=%dHz, dofs=%d",
                       config.name.c_str(), config.port.c_str(), 
                       config.feedback_rate, config.num_of_dofs);
        }
        
        return true;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(owner_->get_logger(), "Error discovering arm configs: %s", e.what());
        return false;
    }
}

/**
 * @brief  验证配置的一致性和完整性
 * @param  null
 * @retval 验证通过返回true，否则返回false
 */
bool TeleopArmDriverNode::Impl::validateConfiguration()
{
    if (arm_configs_.empty()) {
        RCLCPP_ERROR(owner_->get_logger(), "Validation failed: No arm configurations found");
        return false;
    }
    
    int calculated_total_dofs = 0;
    for (const auto& config : arm_configs_) {
        calculated_total_dofs += config.num_of_dofs;
    }
    
    if (calculated_total_dofs != total_num_of_dofs_) {
        RCLCPP_ERROR(owner_->get_logger(), 
                    "Validation failed: Total DOFs mismatch - configured: %d, calculated: %d",
                    total_num_of_dofs_, calculated_total_dofs);
        return false;
    }
    
    if (joint_names_.size() != static_cast<size_t>(total_num_of_dofs_)) {
        RCLCPP_ERROR(owner_->get_logger(), 
                    "Validation failed: joint_names count (%zu) != num_of_dofs (%d)",
                    joint_names_.size(), total_num_of_dofs_);
        return false;
    }
    
    if (joint_mapping_.size() != static_cast<size_t>(total_num_of_dofs_)) {
        RCLCPP_ERROR(owner_->get_logger(), 
                    "Validation failed: joint_mapping count (%zu) != num_of_dofs (%d)",
                    joint_mapping_.size(), total_num_of_dofs_);
        return false;
    }
    
    std::set<int> mapping_set(joint_mapping_.begin(), joint_mapping_.end());
    if (mapping_set.size() != joint_mapping_.size()) {
        RCLCPP_ERROR(owner_->get_logger(), "Validation failed: joint_mapping has duplicate indices");
        return false;
    }
    
    for (int idx : joint_mapping_) {
        if (idx < 0 || idx >= total_num_of_dofs_) {
            RCLCPP_ERROR(owner_->get_logger(), 
                        "Validation failed: joint_mapping index %d out of range [0, %d)",
                        idx, total_num_of_dofs_);
            return false;
        }
    }
    
    std::set<std::string> name_set(joint_names_.begin(), joint_names_.end());
    if (name_set.size() != joint_names_.size()) {
        RCLCPP_ERROR(owner_->get_logger(), "Validation failed: joint_names has duplicate names");
        return false;
    }
    
    RCLCPP_INFO(owner_->get_logger(), "Configuration validation passed");
    return true;
}

/*******************************************************************************
 * Adapter Functions
 ******************************************************************************/

/**
 * @brief  初始化所有机械臂适配器
 * @param  null
 * @retval 初始化成功返回true，否则返回false
 */
bool TeleopArmDriverNode::Impl::initializeAdapters()
{
    int joint_offset = 0;  // 关节索引偏移量，用于分配关节范围
    
    for (const auto& config : arm_configs_) {
        RCLCPP_INFO(owner_->get_logger(), "Initializing adapter: %s", config.name.c_str());
        
        // 创建适配器实例
        auto adapter = createAdapter(config);
        if (!adapter) {
            RCLCPP_ERROR(owner_->get_logger(), "Failed to create adapter for %s", config.name.c_str());
            return false;
        }
        
        // 为该机械臂分配关节索引范围
        adapter_joint_ranges_[config.name] = {joint_offset, joint_offset + config.num_of_dofs - 1};
        joint_offset += config.num_of_dofs;
        
        // 保存适配器实例
        adapters_[config.name] = std::move(adapter);
        
        RCLCPP_INFO(owner_->get_logger(), "Adapter %s initialized successfully (joints %d-%d)",
                   config.name.c_str(), 
                   adapter_joint_ranges_[config.name].first,
                   adapter_joint_ranges_[config.name].second);
    }
    
    return true;
}

/**
 * @brief  创建单个机械臂适配器
 * @param  config 机械臂配置信息
 * @retval 适配器智能指针，失败返回nullptr
 */
std::unique_ptr<DeviceAdapter> TeleopArmDriverNode::Impl::createAdapter(const ArmConfig& config)
{
    try {
        // 配置设备适配器参数
        DeviceAdapterConfig device_config;
        device_config.device_name = config.name;
        device_config.num_of_dofs = config.num_of_dofs;
        device_config.feedback_rate = config.feedback_rate;
        device_config.auto_enable = auto_enable_;
        device_config.seq_num_bits = 16;  // 设备使用16位序列号
        device_config.seq_window_size = 1024;

        std::unique_ptr<DeviceAdapter> adapter;

        if (adapter_type_ == "teleop") {
            // 创建并配置串口
            auto serial_port = std::make_unique<SerialPort>();
            SerialPort::Config serial_config(config.port, 2000000);  // 固定2Mbps波特率
            
            if (!serial_port->open(serial_config)) {
                RCLCPP_ERROR(owner_->get_logger(), "Failed to open serial port: %s", config.port.c_str());
                return nullptr;
            }
            // Teleop 适配器：使用串口对象
            adapter = std::make_unique<TeleopArmAdapter>(device_config, std::move(serial_port));
#ifdef BUILD_Y1_ADAPTER
        } else if (adapter_type_ == "y1") {
            // Y1 适配器：按 y1_driver_node 的用法直接传串口名
            adapter = std::make_unique<Y1ArmAdapter>(device_config, config.port);
#endif
        } else if (adapter_type_ == "ar5") {
            // AR5 适配器：使用 Rokae SDK 力矩控制接口，需要 robot_ip 和 local_ip
            std::string robot_ip = config.port;  // port 字段复用为 robot_ip
            std::string local_ip = config.local_ip;
            if (robot_ip.empty()) {
                RCLCPP_ERROR(owner_->get_logger(), "AR5 adapter requires robot_ip (via right_arm_port parameter)");
                return nullptr;
            }
            if (local_ip.empty()) {
                RCLCPP_ERROR(owner_->get_logger(), "AR5 adapter requires local_ip parameter (right_arm_local_ip)");
                return nullptr;
            }

            // 构造初始关节位姿
            std::array<double, AR5_DOF> init_positions = {0, M_PI/6, 0, M_PI/3, 0, 0, 0};
            if (!config.init_joint_positions.empty()) {
                for (size_t i = 0; i < AR5_DOF && i < config.init_joint_positions.size(); ++i) {
                    init_positions[i] = config.init_joint_positions[i];
                }
            }

            adapter = std::make_unique<Ar5ArmAdapter>(device_config, robot_ip, local_ip, init_positions,
                                                      config.load_mass, config.load_cog, config.load_inertia);
        } else if (adapter_type_ == "ar5_suction_cup") {
            // AR5 Suction Cup 适配器：AR5 SDK + 外设HTTP API（旋转吸盘）
            std::string robot_ip = config.port;
            std::string local_ip = config.local_ip;
            if (robot_ip.empty()) {
                RCLCPP_ERROR(owner_->get_logger(), "AR5 suction cup adapter requires robot_ip (via right_arm_port parameter)");
                return nullptr;
            }
            if (local_ip.empty()) {
                RCLCPP_ERROR(owner_->get_logger(), "AR5 suction cup adapter requires local_ip parameter (right_arm_local_ip)");
                return nullptr;
            }

            // 读取外设服务器URL
            std::string peripheral_server_url;
            std::string peripheral_url_param = config.name + "_arm_peripheral_url";
            if (owner_->has_parameter(peripheral_url_param)) {
                peripheral_server_url = owner_->get_parameter(peripheral_url_param).as_string();
            }
            if (peripheral_server_url.empty()) {
                RCLCPP_WARN(owner_->get_logger(), "AR5 suction cup adapter: peripheral_server_url not set, suction cup control disabled");
            }

            // 构造初始关节位姿
            std::array<double, AR5_ARM_DOF> init_positions = {0, M_PI/6, 0, M_PI/3, 0, 0, 0};
            if (!config.init_joint_positions.empty()) {
                for (size_t i = 0; i < AR5_ARM_DOF && i < config.init_joint_positions.size(); ++i) {
                    init_positions[i] = config.init_joint_positions[i];
                }
            }

            adapter = std::make_unique<Ar5SuctionCupAdapter>(
                device_config, robot_ip, local_ip, peripheral_server_url, init_positions,
                config.load_mass, config.load_cog, config.load_inertia);
        } else if (adapter_type_ == "ar5_gripper") {
            // AR5 Gripper 适配器：AR5 SDK + 外设HTTP API（夹爪）
            std::string robot_ip = config.port;
            std::string local_ip = config.local_ip;
            if (robot_ip.empty()) {
                RCLCPP_ERROR(owner_->get_logger(), "AR5 gripper adapter requires robot_ip (via right_arm_port parameter)");
                return nullptr;
            }
            if (local_ip.empty()) {
                RCLCPP_ERROR(owner_->get_logger(), "AR5 gripper adapter requires local_ip parameter (right_arm_local_ip)");
                return nullptr;
            }

            // 读取外设服务器URL
            std::string peripheral_server_url;
            std::string peripheral_url_param = config.name + "_arm_peripheral_url";
            if (owner_->has_parameter(peripheral_url_param)) {
                peripheral_server_url = owner_->get_parameter(peripheral_url_param).as_string();
            }
            if (peripheral_server_url.empty()) {
                RCLCPP_WARN(owner_->get_logger(), "AR5 gripper adapter: peripheral_server_url not set, gripper control disabled");
            }

            // 构造初始关节位姿
            std::array<double, AR5_ARM_DOF> init_positions = {0, M_PI/6, 0, M_PI/3, 0, 0, 0};
            if (!config.init_joint_positions.empty()) {
                for (size_t i = 0; i < AR5_ARM_DOF && i < config.init_joint_positions.size(); ++i) {
                    init_positions[i] = config.init_joint_positions[i];
                }
            }

            adapter = std::make_unique<Ar5GripperAdapter>(
                device_config, robot_ip, local_ip, peripheral_server_url, init_positions,
                config.load_mass, config.load_cog, config.load_inertia);
        } else {
            RCLCPP_ERROR(owner_->get_logger(), "Unsupported adapter_type for teleop_arm_driver_node: %s", adapter_type_.c_str());
            return nullptr;
        }

        // 初始化适配器（包括握手和配置设备）
        if (!adapter->initialize()) {
            RCLCPP_ERROR(owner_->get_logger(), "Failed to initialize adapter for %s", config.name.c_str());
            return nullptr;
        }
        
        return adapter;
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(owner_->get_logger(), "Exception creating adapter for %s: %s", 
                    config.name.c_str(), e.what());
        return nullptr;
    }
}

/*******************************************************************************
 * ROS Interface Functions
 ******************************************************************************/

/**
 * @brief  初始化ROS2接口
 * @param  null
 * @retval null
 */
void TeleopArmDriverNode::Impl::initializeRosInterface()
{
    joint_state_publisher_ = owner_->create_publisher<sensor_msgs::msg::JointState>(
        joint_state_topic_, rclcpp::QoS(10));

    if (!external_wrench_topic_.empty()) {
        external_wrench_publisher_ = owner_->create_publisher<geometry_msgs::msg::WrenchStamped>(
            external_wrench_topic_, rclcpp::QoS(10));
        RCLCPP_INFO(owner_->get_logger(), "  External wrench topic: %s", external_wrench_topic_.c_str());
    }

    status_publisher_ = owner_->create_publisher<infra_msg::msg::TeleOpArmStatus>(
        status_topic_, rclcpp::QoS(10));
    
    joint_cmd_subscriber_ = owner_->create_subscription<infra_msg::msg::JointMitControl>(
        joint_cmd_topic_, rclcpp::QoS(10),
        std::bind(&TeleopArmDriverNode::Impl::jointCommandCallback, this, std::placeholders::_1));
    
    // 创建关节状态发布定时器（根据最高反馈频率）
    int max_rate = 0;
    for (const auto& config : arm_configs_) {
        max_rate = std::max(max_rate, config.feedback_rate);
    }
    
    auto timer_period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / max_rate));
    publish_timer_ = owner_->create_wall_timer(
        timer_period,
        std::bind(&TeleopArmDriverNode::Impl::publishTimerCallback, this));
    
    // 创建机械臂状态发布定时器（固定10Hz）
    auto status_period = std::chrono::milliseconds(100);  // 10Hz = 100ms
    status_timer_ = owner_->create_wall_timer(
        status_period,
        std::bind(&TeleopArmDriverNode::Impl::publishArmStatus, this));
    
    RCLCPP_INFO(owner_->get_logger(), "ROS interface initialized");
    RCLCPP_INFO(owner_->get_logger(), "  Publishing to: %s, %s", 
               joint_state_topic_.c_str(), status_topic_.c_str());
    RCLCPP_INFO(owner_->get_logger(), "  Subscribing to: %s", joint_cmd_topic_.c_str());
    RCLCPP_INFO(owner_->get_logger(), "  Joint state publish rate: %d Hz", max_rate);
    RCLCPP_INFO(owner_->get_logger(), "  Arm status publish rate: 10 Hz");
}

/*******************************************************************************
 * Callback Functions
 ******************************************************************************/

/**
 * @brief  关节命令回调函数
 * @param  msg 接收到的MIT控制命令消息
 * @retval null
 */
void TeleopArmDriverNode::Impl::jointCommandCallback(const infra_msg::msg::JointMitControl::SharedPtr msg)
{
    if (!is_running_.load()) {
        return;
    }
    
    std::lock_guard<std::mutex> lock(command_mutex_);
    
    try {
        // 检查所有命令数组尺寸是否一致
        size_t expected_size = static_cast<size_t>(total_num_of_dofs_);
        if (msg->joint_position.size() != expected_size ||
            msg->joint_velocity.size() != expected_size ||
            msg->torque.size() != expected_size ||
            msg->kp.size() != expected_size ||
            msg->kd.size() != expected_size) {
            RCLCPP_WARN_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 1000,
                               "Joint command size mismatch: expected %zu for all fields, got pos=%zu, vel=%zu, torque=%zu, kp=%zu, kd=%zu",
                               expected_size, msg->joint_position.size(), msg->joint_velocity.size(),
                               msg->torque.size(), msg->kp.size(), msg->kd.size());
            return;
        }
        
        // 构建电机命令数组，应用关节映射（ROS顺序 -> 设备顺序）
        std::vector<MotorCommand> commands(total_num_of_dofs_);
        for (int i = 0; i < total_num_of_dofs_; ++i) {
            int device_idx = joint_mapping_[i];  // ROS索引i对应的设备索引
            commands[device_idx].position = static_cast<float>(msg->joint_position[i]);
            commands[device_idx].velocity = static_cast<float>(msg->joint_velocity[i]);
            commands[device_idx].torque = static_cast<float>(msg->torque[i]);
            commands[device_idx].kp = static_cast<float>(msg->kp[i]);
            commands[device_idx].kd = static_cast<float>(msg->kd[i]);
        }
        
        // 将命令分发给各个适配器
        if (!distributeCommands(commands)) {
            RCLCPP_WARN_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 1000,
                               "Failed to distribute commands to adapters");
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(owner_->get_logger(), "Error in joint command callback: %s", e.what());
    }
}

/**
 * @brief  关节状态发布定时器回调函数
 * @param  null
 * @retval null
 */
void TeleopArmDriverNode::Impl::publishTimerCallback()
{
    if (is_running_.load()) {
        publishJointStates();
        if (external_wrench_publisher_) {
            publishExternalWrench();
        }
    }
}

/*******************************************************************************
 * Publish Functions
 ******************************************************************************/

/**
 * @brief  发布关节状态
 * @param  null
 * @retval null
 */
void TeleopArmDriverNode::Impl::publishJointStates()
{
    try {
        // 从所有适配器聚合关节状态
        std::vector<MotorState> device_states;
        if (!aggregateJointStates(device_states)) {
            return;
        }
        
        // 应用逆向关节映射（设备顺序 -> ROS 顺序）
        std::vector<MotorState> ros_states;
        reverseJointMapping(device_states, ros_states);
        
        // 构建 ROS 消息
        auto msg = std::make_unique<sensor_msgs::msg::JointState>();
        msg->header.stamp = owner_->get_clock()->now();
        msg->name = joint_names_;
        
        msg->position.resize(total_num_of_dofs_);
        msg->velocity.resize(total_num_of_dofs_);
        msg->effort.resize(total_num_of_dofs_);
        
        for (int i = 0; i < total_num_of_dofs_; ++i) {
            msg->position[i] = static_cast<double>(ros_states[i].position);
            msg->velocity[i] = static_cast<double>(ros_states[i].velocity);
            msg->effort[i] = static_cast<double>(ros_states[i].effort);
        }
        
        joint_state_publisher_->publish(std::move(msg));
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 1000,
                            "Error publishing joint states: %s", e.what());
    }
}

/**
 * @brief  发布基坐标系中外部力/力矩 [Fx,Fy,Fz,Tx,Ty,Tz]
 */
void TeleopArmDriverNode::Impl::publishExternalWrench()
{
    try {
        std::lock_guard<std::mutex> lock(adapters_mutex_);

        for (const auto& [name, adapter] : adapters_) {
            if (!adapter) continue;

            std::array<double, 6> wrench{};
            if (!adapter->readExternalWrench(wrench)) continue;

            auto msg = std::make_unique<geometry_msgs::msg::WrenchStamped>();
            msg->header.stamp = owner_->get_clock()->now();
            msg->header.frame_id = robot_name_ + "_base_link";
            msg->wrench.force.x  = wrench[0];
            msg->wrench.force.y  = wrench[1];
            msg->wrench.force.z  = wrench[2];
            msg->wrench.torque.x = wrench[3];
            msg->wrench.torque.y = wrench[4];
            msg->wrench.torque.z = wrench[5];

            external_wrench_publisher_->publish(std::move(msg));
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 1000,
                            "Error publishing external wrench: %s", e.what());
    }
}

/**
 * @brief  发布机械臂状态
 * @param  null
 * @retval null
 */
void TeleopArmDriverNode::Impl::publishArmStatus()
{
    if (!is_running_.load()) {
        return;
    }
    
    try {
        std::lock_guard<std::mutex> lock(adapters_mutex_);
        
        // 为每个适配器发布状态
        for (const auto& [name, adapter] : adapters_) {
            if (adapter) {
                auto msg = std::make_unique<infra_msg::msg::TeleOpArmStatus>();
                msg->header.stamp = owner_->get_clock()->now();
                msg->arm_name = name;
                msg->arm_model = adapter_type_;
                
                // 获取适配器设备状态
                auto status = adapter->getDeviceStatus();
                msg->connection_status = status.connected;
                msg->device_status = status.status;
                msg->error_code = status.error_code;
                msg->error_message = status.error_message;
                
                // 初始化通讯统计信息
                msg->total_packet_loss_count = status.total_packet_loss_count;
                msg->total_packet_count = status.total_packet_count;
                msg->overall_packet_loss_rate = status.overall_packet_loss_rate;
                
                // 读取各电机状态并填充到消息中
                std::vector<MotorState> motor_states;
                if (adapter->readMotorStates(motor_states)) {
                    msg->motor_status_array.resize(motor_states.size());
                    
                    for (size_t i = 0; i < motor_states.size(); ++i) {
                        auto& motor_msg = msg->motor_status_array[i];
                        const auto& motor_state = motor_states[i];
                        
                        // 填充电机状态信息
                        motor_msg.motor_id = static_cast<uint8_t>(i);
                        motor_msg.temperature = static_cast<float>(motor_state.temperature);
                        motor_msg.is_online = motor_state.enabled;
                        motor_msg.current_mode = motor_state.enabled ? 
                            infra_msg::msg::MotorStatus::MODE_MIT_CONTROL : 
                            infra_msg::msg::MotorStatus::MODE_IDLE;
                        
                        // 使用整体丢包率作为每电机的丢包率展示
                        motor_msg.packet_loss_rate = msg->overall_packet_loss_rate;
                        
                        // 可选：设置电机名称（基于关节索引范围）
                        auto range_it = adapter_joint_ranges_.find(name);
                        if (range_it != adapter_joint_ranges_.end()) {
                            int global_joint_idx = range_it->second.first + i;
                            if (global_joint_idx < static_cast<int>(joint_names_.size())) {
                                motor_msg.motor_name = joint_names_[global_joint_idx];
                            }
                        }
                    }
                }
                
                status_publisher_->publish(std::move(msg));
            }
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 1000,
                            "Error publishing arm status: %s", e.what());
    }
}

/*******************************************************************************
 * Joint State Functions
 ******************************************************************************/

/**
 * @brief  聚合所有机械臂的关节状态
 * @param  joint_states 输出参数，聚合后的关节状态数组
 * @retval 成功返回true
 */
bool TeleopArmDriverNode::Impl::aggregateJointStates(std::vector<MotorState>& joint_states)
{
    joint_states.clear();
    joint_states.resize(total_num_of_dofs_);
    
    std::lock_guard<std::mutex> lock(adapters_mutex_);
    
    // 遍历所有适配器
    for (const auto& [name, adapter] : adapters_) {
        if (!adapter) {
            continue;
        }
        
        // 获取该适配器的关节索引范围
        auto range_it = adapter_joint_ranges_.find(name);
        if (range_it == adapter_joint_ranges_.end()) {
            continue;
        }
        
        int start_idx = range_it->second.first;
        int end_idx = range_it->second.second;
        
        // 读取该适配器的关节状态
        std::vector<MotorState> adapter_states;
        if (!adapter->readMotorStates(adapter_states)) {
            continue;
        }
        
        // 将状态填充到对应的索引位置
        for (size_t i = 0; i < adapter_states.size() && (start_idx + i) <= end_idx; ++i) {
            joint_states[start_idx + i] = adapter_states[i];
        }
    }
    
    return true;
}

/**
 * @brief  将关节命令分发给各个机械臂适配器
 * @param  commands 完整的关节命令数组
 * @retval 分发成功返回true
 */
bool TeleopArmDriverNode::Impl::distributeCommands(const std::vector<MotorCommand>& commands)
{
    // 检查命令数组大小
    if (commands.size() != static_cast<size_t>(total_num_of_dofs_)) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(adapters_mutex_);
    
    // 遍历所有适配器
    for (const auto& [name, adapter] : adapters_) {
        if (!adapter) {
            continue;
        }
        
        // 获取该适配器的关节索引范围
        auto range_it = adapter_joint_ranges_.find(name);
        if (range_it == adapter_joint_ranges_.end()) {
            continue;
        }
        
        int start_idx = range_it->second.first;
        int end_idx = range_it->second.second;
        int num_joints = end_idx - start_idx + 1;
        
        // 提取该适配器对应的命令子集
        std::vector<MotorCommand> adapter_commands(num_joints);
        for (int i = 0; i < num_joints; ++i) {
            adapter_commands[i] = commands[start_idx + i];
        }
        
        // 发送命令到适配器
        if (!adapter->sendMotorCommands(adapter_commands)) {
            RCLCPP_WARN_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 1000,
                               "Failed to send commands to adapter: %s", name.c_str());
        }
    }
    
    return true;
}

/*******************************************************************************
 * Joint Mapping Functions
 ******************************************************************************/

/**
 * @brief  应用关节映射（ROS顺序 -> 设备顺序）
 * @param  ros_order ROS顺序的关节值
 * @param  device_order 输出参数，设备顺序的关节值
 * @retval null
 */
void TeleopArmDriverNode::Impl::applyJointMapping(const std::vector<double>& ros_order,
                                                  std::vector<double>& device_order)
{
    device_order.resize(ros_order.size());
    for (size_t i = 0; i < ros_order.size(); ++i) {
        device_order[joint_mapping_[i]] = ros_order[i];
    }
}

/**
 * @brief  应用逆向关节映射（设备顺序 -> ROS顺序）
 * @param  device_order 设备顺序的电机状态
 * @param  ros_order 输出参数，ROS顺序的电机状态
 * @retval null
 */
void TeleopArmDriverNode::Impl::reverseJointMapping(const std::vector<MotorState>& device_order,
                                                  std::vector<MotorState>& ros_order)
{
    ros_order.resize(device_order.size());
    for (size_t i = 0; i < device_order.size(); ++i) {
        ros_order[i] = device_order[joint_mapping_[i]];
    }
}

/*******************************************************************************
 * Peripheral Functions
 ******************************************************************************/

/**
 * @brief  初始化外设接口
 * @param  null
 * @retval null
 */
void TeleopArmDriverNode::Impl::initializePeripheralInterface()
{
    // 构造带 robot 前缀的外设相关 topic
    std::string prefix;
    if (!robot_name_.empty()) {
        prefix = "/" + robot_name_;
    }

    // 获取topic参数
    std::string peripheral_command_topic = owner_->get_parameter("peripheral_command_topic").as_string();
    std::string gripper_key_state_topic = owner_->get_parameter("gripper_key_state_topic").as_string();

    // 发布者：夹爪按键状态
    gripper_key_state_publisher_ = owner_->create_publisher<infra_msg::msg::TeleopGripperKeyState>(
        prefix + "/" + gripper_key_state_topic, rclcpp::QoS(10));
    
    // 订阅者：外设命令
    peripheral_cmd_subscriber_ = owner_->create_subscription<infra_msg::msg::PeripheralCommand>(
        prefix + "/" + peripheral_command_topic, rclcpp::QoS(10),
        std::bind(&TeleopArmDriverNode::Impl::peripheralCommandCallback, this,
                 std::placeholders::_1));
     
    // 独立定时器：100Hz 检测按钮事件
    peripheral_timer_ = owner_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&TeleopArmDriverNode::Impl::peripheralTimerCallback, this));
    
    RCLCPP_INFO(owner_->get_logger(), "Peripheral interface initialized");
    RCLCPP_INFO(owner_->get_logger(), "  Publishing gripper key states to: %s", (prefix + gripper_key_state_topic).c_str());
    RCLCPP_INFO(owner_->get_logger(), "  Subscribing peripheral commands from: %s", (prefix + peripheral_command_topic).c_str());
}

/**
 * @brief  外设定时器回调函数
 * @param  null
 * @retval null
 */
void TeleopArmDriverNode::Impl::peripheralTimerCallback()
{
    // 只有 teleop 适配器才有外设
    if (!is_running_.load() || adapter_type_ != "teleop") {
        return;
    }
    
    try {
        std::lock_guard<std::mutex> lock(adapters_mutex_);
        
        bool should_publish = false;
        
        // 检查是否有状态变化（按键或摇杆）
        for (const auto& [name, adapter] : adapters_) {
            if (!adapter) {
                continue;
            }
            
            PeripheralState state;
            auto teleop_adapter = dynamic_cast<TeleopArmAdapter*>(adapter.get());
            if (teleop_adapter && teleop_adapter->getPeripheralState(state)) {
                // 检查按键状态变化
                auto it = last_button_status_.find(name);
                if (it == last_button_status_.end() || it->second != state.button_status) {
                    should_publish = true;
                    last_button_status_[name] = state.button_status;
                }
                // 检查摇杆值变化
                auto itx = last_joystick_x_.find(name);
                auto ity = last_joystick_y_.find(name);
                if (itx == last_joystick_x_.end() || itx->second != state.joystick_x ||
                    ity == last_joystick_y_.end() || ity->second != state.joystick_y) {
                    should_publish = true;
                    last_joystick_x_[name] = state.joystick_x;
                    last_joystick_y_[name] = state.joystick_y;
                }
            }
        }
        
        // 每秒发布一次按钮状态（100次调用 = 1秒）
        peripheral_publish_counter_++;
        if (peripheral_publish_counter_ >= 100) {
            peripheral_publish_counter_ = 0;
            should_publish = true;
        }
        
        if (should_publish) {
            // 发布当前夹爪按键状态
            auto msg = std::make_unique<infra_msg::msg::TeleopGripperKeyState>();
            msg->header.stamp = owner_->get_clock()->now();
            
            for (const auto& [name, adapter] : adapters_) {
                if (!adapter) {
                    continue;
                }
                
                PeripheralState state;
                auto teleop_adapter = dynamic_cast<TeleopArmAdapter*>(adapter.get());
                if (teleop_adapter) {
                    infra_msg::msg::GripperKeyState key_state;
                    key_state.gripper_name = name;
                    if (!teleop_adapter->getPeripheralState(state)) {
                        state.button_status = 0;
                        state.joystick_x = 2048;
                        state.joystick_y = 2048;
                    }
                    // 按键映射 (按协议 Bit 定义)
                    key_state.sw_key           = (state.button_status & 0x01) != 0;  // Bit 0
                    key_state.teleop_key       = (state.button_status & 0x02) != 0;  // Bit 1
                    key_state.marker_key       = (state.button_status & 0x04) != 0;  // Bit 2
                    key_state.data_collect_key = (state.button_status & 0x08) != 0;  // Bit 3
                    key_state.take_over_key    = (state.button_status & 0x10) != 0;  // Bit 4
                    key_state.safety_key       = (state.button_status & 0x20) != 0;  // Bit 5
                    key_state.rocker_key       = (state.button_status & 0x40) != 0;  // Bit 6
                    
                    // 摇杆值映射 (0~4096 → -100~100)
                    key_state.rocker_x = state.getJoystickXNormalized();
                    key_state.rocker_y = state.getJoystickYNormalized();
                    
                    msg->key.push_back(key_state);
                    
                    RCLCPP_DEBUG(owner_->get_logger(),
                                "[%s] Publishing gripper key state: sw=%d, teleop=%d, marker=%d, "
                                "data=%d, take_over=%d, safety=%d, rocker=%d, joy_x=%d, joy_y=%d",
                                name.c_str(), key_state.sw_key, key_state.teleop_key,
                                key_state.marker_key, key_state.data_collect_key,
                                key_state.take_over_key, key_state.safety_key,
                                key_state.rocker_key, key_state.rocker_x, key_state.rocker_y);
                }
            }
            
            if (!msg->key.empty()) {
                gripper_key_state_publisher_->publish(std::move(msg));
            }
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(owner_->get_logger(), *owner_->get_clock(), 1000,
                            "Error in peripheral timer: %s", e.what());
    }
}

/**
 * @brief  外设命令回调函数
 * @param  msg 接收到的外设命令消息
 * @retval null
 */
void TeleopArmDriverNode::Impl::peripheralCommandCallback(
    const infra_msg::msg::PeripheralCommand::SharedPtr msg)
{
    // 只有 teleop 适配器才有外设
    if (!is_running_.load() || adapter_type_ != "teleop") {
        return;
    }
    
    try {
        std::lock_guard<std::mutex> lock(adapters_mutex_);
        
        // 查找对应的适配器
        auto it = adapters_.find(msg->arm_name);
        if (it == adapters_.end()) {
            RCLCPP_WARN(owner_->get_logger(), 
                       "Received command for unknown arm: %s", msg->arm_name.c_str());
            return;
        }
        
        auto& adapter = it->second;
        
        // 构造LED控制字节
        uint8_t led_control = 0;
        led_control |= (msg->red_mode & 0x03) << 0;      // Bit[1:0]
        led_control |= (msg->green_mode & 0x03) << 2;    // Bit[3:2]
        led_control |= (msg->blue_mode & 0x03) << 4;     // Bit[5:4]
        
        // 发送到设备
        auto teleop_adapter = dynamic_cast<TeleopArmAdapter*>(adapter.get());
        if (teleop_adapter && teleop_adapter->setPeripheralState(led_control)) {
            RCLCPP_DEBUG(owner_->get_logger(),
                       "[%s] LED set: R=%d G=%d B=%d",
                       msg->arm_name.c_str(),
                       msg->red_mode, msg->green_mode, msg->blue_mode);
        } else {
            RCLCPP_WARN(owner_->get_logger(),
                       "[%s] Failed to set LED (timeout)",
                       msg->arm_name.c_str());
        }
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(owner_->get_logger(),
                    "Error in peripheral command callback: %s", e.what());
    }
}

} // namespace teleop_adapter
