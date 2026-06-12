/*
 * @Description: Internal implementation details for TeleopArmDriverNode (not for distribution).
 */

#ifndef TELEOP_ADAPTER_NODES_DETAIL_TELEOP_ARM_DRIVER_NODE_IMPL_HPP
#define TELEOP_ADAPTER_NODES_DETAIL_TELEOP_ARM_DRIVER_NODE_IMPL_HPP

#include "teleop_adapter/nodes/teleop_arm_driver_node.hpp"

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <infra_msg/msg/tele_op_arm_status.hpp>
#include <infra_msg/msg/joint_mit_control.hpp>
#include <infra_msg/msg/peripheral_command.hpp>
#include <infra_msg/msg/teleop_gripper_key_state.hpp>

#include "teleop_adapter/adapters/device_adapter.hpp"
#include "teleop_adapter/adapters/teleop_arm_adapter.hpp"
#include "teleop_adapter/communication/serial_port.hpp"

#include <array>
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace teleop_adapter {

struct ArmConfig {
    std::string name;
    std::string port;
    int feedback_rate;
    int num_of_dofs;
    std::string local_ip;
    std::vector<double> init_joint_positions;
    double load_mass{0};
    std::array<double, 3> load_cog{};
    std::array<double, 3> load_inertia{};
};

struct TeleopArmDriverNode::Impl {
    explicit Impl(TeleopArmDriverNode* owner);

    TeleopArmDriverNode* owner_;

    void declareParameters();
    bool loadGlobalConfig();
    bool discoverAndLoadArmConfigs();
    bool validateConfiguration();
    bool initializeAdapters();
    std::unique_ptr<DeviceAdapter> createAdapter(const ArmConfig& config);
    void initializeRosInterface();
    void jointCommandCallback(const infra_msg::msg::JointMitControl::SharedPtr msg);
    void publishTimerCallback();
    void publishJointStates();
    void publishExternalWrench();
    void publishArmStatus();
    void initializePeripheralInterface();
    void peripheralTimerCallback();
    void peripheralCommandCallback(const infra_msg::msg::PeripheralCommand::SharedPtr msg);
    bool aggregateJointStates(std::vector<MotorState>& joint_states);
    bool distributeCommands(const std::vector<MotorCommand>& commands);
    void applyJointMapping(const std::vector<double>& ros_order, std::vector<double>& device_order);
    void reverseJointMapping(const std::vector<MotorState>& device_order, std::vector<MotorState>& ros_order);

    bool initialize();
    void shutdown();

    int total_num_of_dofs_{0};
    bool auto_enable_{true};
    std::string adapter_type_;
    std::vector<std::string> joint_names_;
    std::vector<int> joint_mapping_;
    std::string robot_name_;

    std::string joint_cmd_topic_;
    std::string joint_state_topic_;
    std::string status_topic_;
    std::string external_wrench_topic_;

    std::vector<ArmConfig> arm_configs_;

    std::map<std::string, std::unique_ptr<DeviceAdapter>> adapters_;
    std::map<std::string, std::pair<int, int>> adapter_joint_ranges_;
    std::mutex adapters_mutex_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr external_wrench_publisher_;
    rclcpp::Publisher<infra_msg::msg::TeleOpArmStatus>::SharedPtr status_publisher_;
    rclcpp::Subscription<infra_msg::msg::JointMitControl>::SharedPtr joint_cmd_subscriber_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;

    rclcpp::Publisher<infra_msg::msg::TeleopGripperKeyState>::SharedPtr gripper_key_state_publisher_;
    rclcpp::Subscription<infra_msg::msg::PeripheralCommand>::SharedPtr peripheral_cmd_subscriber_;
    rclcpp::TimerBase::SharedPtr peripheral_timer_;

    int peripheral_publish_counter_{99};
    std::map<std::string, uint8_t> last_button_status_;
    std::map<std::string, uint16_t> last_joystick_x_;
    std::map<std::string, uint16_t> last_joystick_y_;

    std::mutex command_mutex_;

    std::atomic<bool> is_initialized_{false};
    std::atomic<bool> is_running_{false};
};

} // namespace teleop_adapter

#endif
