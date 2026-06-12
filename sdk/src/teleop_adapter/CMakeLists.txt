cmake_minimum_required(VERSION 3.8)
project(teleop_adapter)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 默认使用 Release 编译模式（-O3 -DNDEBUG）
# 可通过命令行覆盖：colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(infra_msg REQUIRED)

# ============================================
# Console Log Control
#   ENABLE_CONSOLE_LOG=ON  -> 打印所有日志 (RCLCPP + std::cout/cerr)
#   ENABLE_CONSOLE_LOG=OFF -> 关闭所有日志输出 (默认)
# 用法: colcon build --cmake-args -DENABLE_CONSOLE_LOG=ON
# ============================================
option(ENABLE_CONSOLE_LOG "Enable all console log output (RCLCPP + std::cout/cerr)" ON)
if(NOT ENABLE_CONSOLE_LOG)
  add_compile_definitions(RCLCPP_LOG_MIN_SEVERITY=RCLCPP_LOG_MIN_SEVERITY_NONE)
else()
  add_compile_definitions(NEXUS_ENABLE_CONSOLE_LOG)
endif()

# ============================================
# Platform Architecture Detection
# ============================================
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_CXX_COMPILER MATCHES "aarch64")
  set(NEXUS_ARCH "aarch64")
else()
  set(NEXUS_ARCH "x86_64")
endif()
message(STATUS "Target architecture: ${NEXUS_ARCH}")

# ============================================
# Y1 SDK availability — auto-detect, disable on aarch64 if library missing
# ============================================
set(Y1_SDK_LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/imeta_y1/lib/${NEXUS_ARCH}")
set(Y1_SDK_LIB "${Y1_SDK_LIB_DIR}/liby1_sdk.so")
option(BUILD_Y1_ADAPTER "Build Y1 arm adapter (requires Y1 SDK)" ON)
if(BUILD_Y1_ADAPTER AND NOT EXISTS "${Y1_SDK_LIB}")
  message(WARNING "Y1 SDK library not found at ${Y1_SDK_LIB}. Disabling Y1 adapter.")
  set(BUILD_Y1_ADAPTER OFF)
endif()
if(NOT BUILD_Y1_ADAPTER)
  message(STATUS "Y1 adapter: DISABLED")
endif()

# Include directories
include_directories(include)

# ============================================
# Serial Port Library (Communication Layer)
# ============================================
add_library(serial_port src/communication/serial_port.cpp)
target_include_directories(serial_port PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ============================================
# Device Adapter (Unified Device Adapter Base Class)
# ============================================
add_library(device_adapter src/adapters/device_adapter.cpp)
target_include_directories(device_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(device_adapter serial_port)

# ============================================
# Teleop Arm Adapter (Specific Device Implementation)
# ============================================
add_library(teleop_arm_adapter src/adapters/teleop_arm_adapter.cpp)
target_include_directories(teleop_arm_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(teleop_arm_adapter device_adapter)

# ============================================
# Y1 Arm Adapter (CAN-driven, using Y1 SDK)
# ============================================
if(BUILD_Y1_ADAPTER)
  add_library(y1_arm_adapter src/adapters/y1_arm_adapter.cpp)
  target_include_directories(y1_arm_adapter PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_SOURCE_DIR}/external/imeta_y1
  )
  target_link_libraries(y1_arm_adapter device_adapter teleop_arm_adapter)
  ament_target_dependencies(y1_arm_adapter rclcpp std_msgs sensor_msgs infra_msg ament_index_cpp)

  # Link external Y1 SDK if available
  message(STATUS "Linking Y1 SDK library (${NEXUS_ARCH}): ${Y1_SDK_LIB}")
  target_link_libraries(y1_arm_adapter ${Y1_SDK_LIB})
  find_library(GLOG_LIB glog)
  if(GLOG_LIB)
    message(STATUS "Linking glog: ${GLOG_LIB}")
    target_link_libraries(y1_arm_adapter ${GLOG_LIB})
  else()
    message(WARNING "glog library not found. Install libglog-dev or provide libglog.so to satisfy SDK logging symbols.")
  endif()
endif()

# 移除 y1_adapter 目标：不编译 src/adapters/y1_adapter.cpp

# ============================================
# AR5 Arm Adapter (Rokae SDK torque control, MIT formula)
# ============================================
set(AR5_SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/rokae_ar5")
set(AR5_SDK_INCLUDE "${AR5_SDK_DIR}/include")
set(AR5_SDK_LIB_DIR "${AR5_SDK_DIR}/lib/${NEXUS_ARCH}")

# Rokae SDK headers include Eigen via "Eigen/Core", need Eigen3 include path
find_package(Eigen3 REQUIRED)

add_library(ar5_arm_adapter src/adapters/ar5_arm_adapter.cpp)
target_include_directories(ar5_arm_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${AR5_SDK_INCLUDE}
  ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(ar5_arm_adapter device_adapter)

# Link Rokae AR5 SDK libraries
set(AR5_CORE_SO "${AR5_SDK_LIB_DIR}/libxCoreSDK.so.0.7.1")
set(AR5_MODEL_A "${AR5_SDK_LIB_DIR}/libxMateModel.a")
if(EXISTS "${AR5_CORE_SO}" AND EXISTS "${AR5_MODEL_A}")
  message(STATUS "Linking Rokae AR5 SDK libraries into ar5_arm_adapter")
  # libxMateModel.a depends on g3log and orocos-kdl (SDK bundled static libs)
  set(AR5_G3LOG_A "${AR5_SDK_LIB_DIR}/libg3log.a")
  set(AR5_KDL_A "${AR5_SDK_LIB_DIR}/liborocos-kdl.a")
  target_link_libraries(ar5_arm_adapter ${AR5_CORE_SO} ${AR5_MODEL_A} ${AR5_G3LOG_A} ${AR5_KDL_A})
else()
  message(WARNING "Rokae AR5 SDK libraries not found at ${AR5_SDK_LIB_DIR}. "
    "Please place libxCoreSDK.so.0.7.1 and libxMateModel.a there.")
endif()

# ============================================
# AR5 Suction Cup Adapter (Rokae SDK + peripherals remote HTTP API)
# ============================================
add_library(ar5_suction_cup_adapter
  src/adapters/ar5_suction_cup_adapter.cpp
  src/peripherals/remote_client.cpp
)
target_include_directories(ar5_suction_cup_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${AR5_SDK_INCLUDE}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann
)
target_link_libraries(ar5_suction_cup_adapter
  device_adapter
  pthread
)

# Link Rokae AR5 SDK
if(EXISTS "${AR5_CORE_SO}" AND EXISTS "${AR5_MODEL_A}")
  message(STATUS "Linking Rokae AR5 SDK into ar5_suction_cup_adapter")
  target_link_libraries(ar5_suction_cup_adapter
    ${AR5_CORE_SO} ${AR5_MODEL_A} ${AR5_G3LOG_A} ${AR5_KDL_A}
  )
else()
  message(WARNING "Rokae AR5 SDK libraries not found — ar5_suction_cup_adapter will not link")
endif()

# ============================================
# AR5 Gripper Adapter (Rokae SDK + peripherals remote HTTP API)
# ============================================
add_library(ar5_gripper_adapter
  src/adapters/ar5_gripper_adapter.cpp
  src/peripherals/remote_client.cpp
)
target_include_directories(ar5_gripper_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${AR5_SDK_INCLUDE}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/nlohmann
)
target_link_libraries(ar5_gripper_adapter
  device_adapter
  pthread
)

# Link Rokae AR5 SDK
if(EXISTS "${AR5_CORE_SO}" AND EXISTS "${AR5_MODEL_A}")
  message(STATUS "Linking Rokae AR5 SDK into ar5_gripper_adapter")
  target_link_libraries(ar5_gripper_adapter
    ${AR5_CORE_SO} ${AR5_MODEL_A} ${AR5_G3LOG_A} ${AR5_KDL_A}
  )
else()
  message(WARNING "Rokae AR5 SDK libraries not found — ar5_gripper_adapter will not link")
endif()

# ============================================
# Teleop Arm Driver Node (table arm only)
# ============================================
add_library(teleop_arm_driver_node src/nodes/teleop_arm_driver_node.cpp)
target_include_directories(teleop_arm_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/external/imeta_y1
  ${AR5_SDK_INCLUDE}
)
set(TELEOP_DRIVER_LIBS teleop_arm_adapter ar5_arm_adapter ar5_suction_cup_adapter ar5_gripper_adapter)
if(BUILD_Y1_ADAPTER)
  list(APPEND TELEOP_DRIVER_LIBS y1_arm_adapter)
  target_compile_definitions(teleop_arm_driver_node PUBLIC BUILD_Y1_ADAPTER)
endif()
target_link_libraries(teleop_arm_driver_node ${TELEOP_DRIVER_LIBS})
ament_target_dependencies(teleop_arm_driver_node rclcpp std_msgs sensor_msgs geometry_msgs infra_msg)

add_executable(teleop_arm_driver_main src/nodes/teleop_arm_driver_main.cpp)
target_link_libraries(teleop_arm_driver_main teleop_arm_driver_node)
ament_target_dependencies(teleop_arm_driver_main rclcpp)

# Set RPATH so the executable can find libxCoreSDK.so.0 at runtime in the install lib dir
set_target_properties(teleop_arm_driver_main PROPERTIES
  INSTALL_RPATH "$ORIGIN/../lib"
  BUILD_WITH_INSTALL_RPATH FALSE
  BUILD_RPATH "${AR5_SDK_LIB_DIR}"
)

## 独立 Y1 driver_node / y1_driver_main 已被二合一节点取代，如需恢复可从历史版本还原

# ============================================
# Install
# ============================================
set(INSTALL_TARGETS
  serial_port
  device_adapter
  teleop_arm_adapter
  ar5_arm_adapter
  ar5_suction_cup_adapter
  ar5_gripper_adapter
  teleop_arm_driver_node
)
if(BUILD_Y1_ADAPTER)
  list(APPEND INSTALL_TARGETS y1_arm_adapter)
endif()
install(TARGETS ${INSTALL_TARGETS}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装 Y1 SDK 头文件与动态库（若存在）
if(BUILD_Y1_ADAPTER)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/imeta_y1/y1_sdk_interface.h")
    install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/external/imeta_y1/y1_sdk_interface.h
      DESTINATION include/teleop_adapter/external/imeta_y1)
  endif()
  if(EXISTS "${Y1_SDK_LIB}")
    install(FILES ${Y1_SDK_LIB} DESTINATION lib)
    # Install SONAME symlink (liby1_sdk_x64.so) so the runtime linker can find it
    set(Y1_SDK_SONAME_LINK "${Y1_SDK_LIB_DIR}/liby1_sdk_x64.so")
    if(EXISTS "${Y1_SDK_SONAME_LINK}")
      install(FILES ${Y1_SDK_SONAME_LINK} DESTINATION lib)
    endif()
  endif()
endif()

# 安装 Rokae AR5 SDK 头文件与动态库（若存在）
if(EXISTS "${AR5_SDK_INCLUDE}/rokae")
  install(DIRECTORY ${AR5_SDK_INCLUDE}/rokae
    DESTINATION include/teleop_adapter/external/rokae_ar5)
endif()
# Install Rokae AR5 shared library + SONAME symlink
if(EXISTS "${AR5_CORE_SO}")
  install(FILES ${AR5_CORE_SO} DESTINATION lib)
  # Create SONAME symlink (libxCoreSDK.so.0) in install dir
  install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink
    libxCoreSDK.so.0.7.1 \$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/lib/libxCoreSDK.so.0)")
endif()

install(TARGETS
  teleop_arm_driver_main
  DESTINATION lib/${PROJECT_NAME}
)

# Omit detail/ implementation headers from the install tree (closed-source / customer SDK).
install(DIRECTORY include/
  DESTINATION include/
  PATTERN "*/detail/*" EXCLUDE
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

ament_package()
