teleop_adapter
================

This package provides C++ adapter nodes that map device-specific teleoperation messages to a unified `teleop_msg::JointMitControl` topic.

How to build
------------

Make sure your workspace has the `teleop_msg` package built (contains message definitions). Then from the workspace root:

```bash
colcon build --packages-select teleop_adapter
```

Run the example node:

```bash
source install/setup.bash
ros2 run teleop_adapter teleop_adapter_node
```
# Teleop Adapter 目录结构说明

## 概览

本项目采用分层模块化的目录结构，清晰地划分了适配器、通信层、节点和工具类。

## 源码目录结构 (src/)

```
src/
├── adapters/              # 设备适配器层
│   ├── device_adapter.cpp        # 设备适配器基类实现
│   └── table_arm_adapter.cpp     # Table Arm 机械臂适配器实现
│
├── communication/         # 通信接口层
│   └── serial_port.cpp           # 串口通信实现
│
└── nodes/                # ROS2 节点层
    ├── table_arm_driver_node.cpp     # Table Arm 驱动节点实现
    ├── table_arm_driver_main.cpp     # Table Arm 驱动节点入口
    └── teleop_adapter_node.cpp       # 示例节点
```

## 头文件目录结构 (include/teleop_adapter/)

```
include/teleop_adapter/
├── adapters/              # 设备适配器头文件
│   ├── device_adapter.hpp        # 设备适配器基类
│   └── table_arm_adapter.hpp     # Table Arm 机械臂适配器
│
├── communication/         # 通信接口头文件
│   ├── communication_interface.hpp   # 通信接口抽象基类
│   └── serial_port.hpp              # 串口通信类
│
├── nodes/                # ROS2 节点头文件
│   └── table_arm_driver_node.hpp    # Table Arm 驱动节点
│
└── utils/                # 工具类
    └── endian_utils.hpp            # 字节序转换工具
```

## 模块说明

### 1. 适配器层 (adapters/)

**职责**: 实现具体设备的协议解析、命令编码、状态解码等核心业务逻辑

- **device_adapter.hpp/cpp**: 设备适配器基类
  - 提供统一的设备控制接口
  - 管理设备连接状态
  - 实现通用的读写线程框架
  - 提供错误处理机制

- **table_arm_adapter.hpp/cpp**: Table Arm 机械臂适配器
  - 实现 MIT 控制协议
  - 处理电机状态反馈
  - 管理信令帧队列
  - 支持电机使能/失能、零位设置、错误清除等功能

### 2. 通信层 (communication/)

**职责**: 提供底层硬件通信能力，屏蔽不同通信方式的差异

- **communication_interface.hpp**: 通信接口抽象基类
  - 定义统一的 read/write/open/close 接口
  - 支持多种通信方式（串口、网络等）

- **serial_port.hpp/cpp**: 串口通信实现
  - 线程安全的串口读写
  - 支持多种波特率、数据位、停止位配置
  - 非阻塞 I/O 实现

### 3. 节点层 (nodes/)

**职责**: 封装 ROS2 节点，连接适配器和 ROS2 生态系统

- **table_arm_driver_node.hpp/cpp**: Table Arm 驱动节点
  - 管理适配器生命周期
  - 发布关节状态 (JointState)
  - 发布设备状态 (TeleOpArmStatus)
  - 订阅控制命令
  - 提供参数配置

- **table_arm_driver_main.cpp**: 节点入口程序
  - ROS2 节点初始化和启动

### 4. 工具类 (utils/)

**职责**: 提供通用的辅助功能

- **endian_utils.hpp**: 字节序转换工具
  - 自动检测系统字节序
  - 提供大小端转换函数
  - 支持跨平台字节序处理

## 依赖关系

```
nodes (ROS2层)
  ↓ 依赖
adapters (业务逻辑层)
  ↓ 依赖
communication (通信层)
  ↓ 依赖
utils (工具层)
```

## Include 路径规范

使用新的目录结构后，include 路径应遵循以下规范：

```cpp
// 适配器
#include "teleop_adapter/adapters/device_adapter.hpp"
#include "teleop_adapter/adapters/table_arm_adapter.hpp"

// 通信层
#include "teleop_adapter/communication/communication_interface.hpp"
#include "teleop_adapter/communication/serial_port.hpp"

// 节点
#include "teleop_adapter/nodes/table_arm_driver_node.hpp"

// 工具
#include "teleop_adapter/utils/endian_utils.hpp"
```

## 扩展新设备

添加新设备适配器时，请遵循以下步骤：

1. **创建适配器类**
   - 在 `src/adapters/` 创建 `xxx_adapter.cpp`
   - 在 `include/teleop_adapter/adapters/` 创建 `xxx_adapter.hpp`
   - 继承 `DeviceAdapter` 基类
   - 实现必要的虚函数

2. **创建 ROS2 节点** (可选)
   - 在 `src/nodes/` 创建节点实现
   - 在 `include/teleop_adapter/nodes/` 创建节点头文件

3. **更新 CMakeLists.txt**
   - 添加新的库目标
   - 配置依赖关系
   - 添加安装规则

## 编译

```bash
cd /path/to/workspace
colcon build --packages-select teleop_adapter
```

## 维护说明

- **添加新功能**: 根据功能类型放入相应目录
- **公共工具**: 统一放在 `utils/` 目录
- **通信方式**: 实现 `CommunicationInterface` 接口，放在 `communication/` 目录
- **协议实现**: 继承 `DeviceAdapter`，放在 `adapters/` 目录
- **ROS2 集成**: 放在 `nodes/` 目录

---

**最后更新**: 2025-11-10
**维护者**: Teleop Team
