/*
 * @Author: Infra Embedded
 * @Date: 2025-01-07 10:00:00
 * @LastEditors: JiangHu JiangHu@pegasus-ai.cn
 * @LastEditTime: 2025-01-07 10:00:00
 * @FilePath: gripper_keyboard/src/qwer_keyboard_node.cpp
 * @Description: QWER键盘节点，监听键盘按键事件并发布夹爪按键状态
 *
 * Copyright (c) 2025 by Pegasus, All Rights Reserved.
 */

/*******************************************************************************
 * Include
 ******************************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <infra_msg/msg/teleop_gripper_key_state.hpp>
#include <infra_msg/msg/gripper_key_state.hpp>
#include <libevdev/libevdev.h>
#include <linux/input.h>
#include <filesystem>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <vector>
#include <string>
#include <map>

/*******************************************************************************
 * Struct Definition
 ******************************************************************************/

/**
 * @brief  夹爪按键配置结构体
 */
struct GripperKeyConfig {
  std::string name;
  bool state_teleop {false};
  bool state_collect {false};
  bool state_marker {false};
  bool state_safety {false};
  bool state_take_over {false};
  bool state_rocker {false};
  
  bool last_teleop {false};
  bool last_collect {false};
  bool last_marker {false};
  bool last_safety {false};
  bool last_take_over {false};
  bool last_rocker {false};
};

/*******************************************************************************
 * Class Definition
 ******************************************************************************/

/**
 * @brief  QWER键盘节点类
 */
class QWERKeyboardNode : public rclcpp::Node {
public:
  /*******************************************************************************
   * Constructor & Destructor
   ******************************************************************************/

  /**
   * @brief  构造函数
   * @param  null
   * @retval null
   */
  QWERKeyboardNode()
    : Node(
        "qwer_keyboard_node",
        rclcpp::NodeOptions()
          .allow_undeclared_parameters(true)
          .automatically_declare_parameters_from_overrides(true))
  {
    if (this->has_parameter("master_robot_cfg.robot_name")) {
      robot_name_ = this->get_parameter("master_robot_cfg.robot_name").as_string();
    } else if (this->has_parameter("robot_name")) {
      robot_name_ = this->get_parameter("robot_name").as_string();
    }

    // Topic parameter (optional)
    if (this->has_parameter("gripper_key_topic")) {
      topic_name_ = this->get_parameter("gripper_key_topic").as_string();
    }

    // Enable grippers parameter (optional)
    std::vector<std::string> enabled_grippers;
    if (this->has_parameter("enabled_grippers")) {
      enabled_grippers = this->get_parameter("enabled_grippers").as_string_array();
    } else {
      // Default: both grippers
      enabled_grippers = {"left_gripper", "right_gripper"};
    }

    // Initialize gripper configs
    for (const auto& name : enabled_grippers) {
      GripperKeyConfig config;
      config.name = name;
      grippers_.push_back(config);
    }
    
    if (grippers_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No grippers enabled!");
      return;
    }

    pub_ = this->create_publisher<infra_msg::msg::TeleopGripperKeyState>(resolveTopic(topic_name_), 10);
    RCLCPP_INFO(this->get_logger(), "QWERKeyboardNode publishing on %s", resolveTopic(topic_name_).c_str());

    enumerateInputDevices();

    timer_ = this->create_wall_timer(std::chrono::milliseconds(20), [this](){ this->pollOnce(); });
  }

  /**
   * @brief  析构函数
   * @param  null
   * @retval null
   */
  ~QWERKeyboardNode() override {
    cleanupDevices();
  }

private:
  /*******************************************************************************
   * Private Methods
   ******************************************************************************/

  /**
   * @brief  解析话题名称
   * @param  t 话题名称
   * @retval 解析后的完整话题名称
   */
  std::string resolveTopic(const std::string &t) const {
    if (!t.empty() && t[0] == '/') return t;
    std::string base = t.empty() ? std::string("rt/teleop/gripper_key_state") : t;
    if (robot_name_.empty()) return std::string("/") + base;
    return std::string("/") + robot_name_ + "/" + base;
  }

  /**
   * @brief  枚举输入设备
   * @param  null
   * @retval null
   */
  void enumerateInputDevices() {
    cleanupDevices();
    try {
      for (const auto &entry : std::filesystem::directory_iterator("/dev/input/")) {
        const auto path = entry.path().string();
        if (path.find("event") == std::string::npos) continue;
        int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) {
          RCLCPP_WARN(this->get_logger(), "Cannot open %s: %s", path.c_str(), strerror(errno));
          continue;
        }
        struct libevdev *dev = nullptr;
        if (libevdev_new_from_fd(fd, &dev) < 0) {
          RCLCPP_WARN(this->get_logger(), "libevdev init failed: %s", path.c_str());
          ::close(fd);
          continue;
        }
        if (libevdev_has_event_type(dev, EV_KEY)) {
          devices_.emplace_back(fd, dev);
          RCLCPP_INFO(this->get_logger(), "Listening: %s", path.c_str());
        } else {
          libevdev_free(dev);
          ::close(fd);
        }
      }
      if (devices_.empty()) {
        RCLCPP_ERROR(this->get_logger(), "No keyboard input devices found under /dev/input");
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Enumerate devices exception: %s", e.what());
    }
  }

  /**
   * @brief  清理设备资源
   * @param  null
   * @retval null
   */
  void cleanupDevices() {
    for (auto &p : devices_) {
      libevdev_free(p.second);
      ::close(p.first);
    }
    devices_.clear();
  }

  /**
   * @brief  轮询一次按键事件
   * @param  null
   * @retval null
   */
  void pollOnce() {
    // poll all devices
    for (auto &p : devices_) {
      struct libevdev *dev = p.second;
      struct input_event ev;
      while (libevdev_next_event(dev, LIBEVDEV_READ_FLAG_NORMAL, &ev) == 0) {
        if (ev.type != EV_KEY) continue;
        const bool pressed = (ev.value == 1) || (ev.value == 2);
        
        // Update all grippers with same QWERTY keys
        switch (ev.code) {
          case KEY_Q:
            for (auto& gripper : grippers_) gripper.state_teleop = pressed;
            break;
          case KEY_W:
            for (auto& gripper : grippers_) gripper.state_collect = pressed;
            break;
          case KEY_E:
            for (auto& gripper : grippers_) gripper.state_marker = pressed;
            break;
          case KEY_R:
            for (auto& gripper : grippers_) gripper.state_safety = pressed;
            break;
          case KEY_T:
            for (auto& gripper : grippers_) gripper.state_take_over = pressed;
            break;
          case KEY_Y:
            for (auto& gripper : grippers_) gripper.state_rocker = pressed;
            break;
          default: break;
        }
      }
    }

    maybePublish();
  }

  /**
   * @brief  发布按键状态（如果需要）
   * @param  null
   * @retval null
   */
  void maybePublish() {
    if (!pub_) return;

    infra_msg::msg::TeleopGripperKeyState msg;
    msg.header.stamp = this->now();
    msg.key.resize(grippers_.size());
    
    bool changed = false;
    for (size_t i = 0; i < grippers_.size(); ++i) {
      auto& gripper = grippers_[i];
      
      msg.key[i].gripper_name = gripper.name;
      msg.key[i].teleop_key = gripper.state_teleop;
      msg.key[i].data_collect_key = gripper.state_collect;
      msg.key[i].marker_key = gripper.state_marker;
      msg.key[i].safety_key = gripper.state_safety;
      msg.key[i].take_over_key = gripper.state_take_over;
      msg.key[i].rocker_key = gripper.state_rocker;
      
      // Check if any state changed for this gripper
      if (gripper.state_teleop != gripper.last_teleop ||
          gripper.state_collect != gripper.last_collect ||
          gripper.state_marker != gripper.last_marker ||
          gripper.state_safety != gripper.last_safety ||
          gripper.state_take_over != gripper.last_take_over ||
          gripper.state_rocker != gripper.last_rocker) {
        changed = true;
      }
    }

    counter_++;
    const bool periodic = (counter_ >= 50); // ~1Hz at 20ms tick
    if (!changed && !periodic) return;
    if (periodic) counter_ = 0;

    // Update last states
    for (auto& gripper : grippers_) {
      gripper.last_teleop = gripper.state_teleop;
      gripper.last_collect = gripper.state_collect;
      gripper.last_marker = gripper.state_marker;
      gripper.last_safety = gripper.state_safety;
      gripper.last_take_over = gripper.state_take_over;
      gripper.last_rocker = gripper.state_rocker;
    }

    pub_->publish(msg);
  }

  /*******************************************************************************
   * Private Members
   ******************************************************************************/
  std::string robot_name_ {"y1_master"};
  std::string topic_name_ {"infra/teleop_gripper_key_state"};

  rclcpp::Publisher<infra_msg::msg::TeleopGripperKeyState>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<std::pair<int, libevdev*>> devices_;
  std::vector<GripperKeyConfig> grippers_;
  
  int counter_ {0};
};

/*******************************************************************************
 * Main Function
 ******************************************************************************/

/**
 * @brief  主函数：ROS 2 程序的入口
 * @param  argc  命令行参数计数
 * @param  argv  命令行参数数组
 * @retval 0 正常退出，非0 异常退出
 */
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<QWERKeyboardNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
