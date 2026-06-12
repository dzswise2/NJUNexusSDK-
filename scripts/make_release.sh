#!/usr/bin/env bash
#
# make_release.sh — 从源码包构建闭源客户发布包
#
# 用法:
#   ./scripts/make_release.sh [--skip-build] [--output-dir DIR] [--arch ARCH]
#
# 前提:
#   - 当前目录为 nexus-sdk 工作空间根目录
#   - 已安装 colcon, ROS 2 以及所有构建依赖
#
set -euo pipefail

###############################################################################
# 配置
###############################################################################

RELEASE_PACKAGES=(
    teleop_adapter
    robot_controller
    nexus_manage
    human_data
)

MSG_PACKAGES=(
    infra_msg
)

WORKSPACE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
INSTALL_DIR="${WORKSPACE_ROOT}/install"
OUTPUT_DIR="${WORKSPACE_ROOT}/release"
SKIP_BUILD=false
ARCH="$(uname -m)"  # x86_64 or aarch64

declare -A PKG_LIBRARIES
PKG_LIBRARIES[teleop_adapter]="serial_port device_adapter teleop_arm_adapter y1_arm_adapter ar5_arm_adapter teleop_arm_driver_node"
PKG_LIBRARIES[robot_controller]="common_fsm ar5_analytical_ik robot_model gripper_controller oscbf_torque_cbf oscbf_vel_cbf arm_controller kinematics_solver gripper_solver arm_control_node"
PKG_LIBRARIES[nexus_manage]="state_machine_engine config_parser_component key_detector_component controller_config_component inference_config_component rotation_orthogonalizer_component rotation_mapping_component command_calculator_component teleop_manager_node"
PKG_LIBRARIES[human_data]="common_fsm kinematics_solver gripper_solver human_data_solver_node"

declare -A PKG_EXECUTABLES
PKG_EXECUTABLES[teleop_adapter]="teleop_arm_driver_main"
PKG_EXECUTABLES[robot_controller]="arm_control_node"
PKG_EXECUTABLES[nexus_manage]="teleop_manager_node"
PKG_EXECUTABLES[human_data]="human_data_solver_node"

declare -A PKG_EXEC_DEPS
PKG_EXEC_DEPS[teleop_adapter]="rclcpp std_msgs sensor_msgs infra_msg ament_index_cpp"
PKG_EXEC_DEPS[robot_controller]="rclcpp std_msgs sensor_msgs infra_msg geometry_msgs ament_index_cpp"
PKG_EXEC_DEPS[nexus_manage]="rclcpp std_msgs infra_msg geometry_msgs"
PKG_EXEC_DEPS[human_data]="rclcpp std_msgs sensor_msgs infra_msg ament_index_cpp"

declare -A PKG_EXTRA_EXEC_DEPS
PKG_EXTRA_EXEC_DEPS[teleop_adapter]="launch launch_ros"
PKG_EXTRA_EXEC_DEPS[robot_controller]="eigen pinocchio launch launch_ros"
PKG_EXTRA_EXEC_DEPS[nexus_manage]="launch launch_ros"
PKG_EXTRA_EXEC_DEPS[human_data]="eigen pinocchio libevdev launch launch_ros"

declare -A PKG_RESOURCE_DIRS
PKG_RESOURCE_DIRS[teleop_adapter]="launch config urdf"
PKG_RESOURCE_DIRS[robot_controller]="launch config urdf meshes specs"
PKG_RESOURCE_DIRS[nexus_manage]="launch config"
PKG_RESOURCE_DIRS[human_data]="launch config urdf"

###############################################################################
# 参数解析
###############################################################################

while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-build) SKIP_BUILD=true; shift ;;
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --arch)       ARCH="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--skip-build] [--output-dir DIR] [--arch ARCH]"
            echo "  --arch ARCH   Target architecture (default: uname -m, e.g. x86_64, aarch64)"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

###############################################################################
# 辅助函数
###############################################################################

log() { echo "[make_release] $*"; }
err() { echo "[make_release] ERROR: $*" >&2; exit 1; }

# 每次调用都先清理输出目录，防止 colcon 扫描到遗留的同名包
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"
touch "$OUTPUT_DIR/COLCON_IGNORE"
log "Output directory cleaned: ${OUTPUT_DIR}"

generate_cmake() {
    local pkg="$1"
    local output_dir="$2"
    local cmake_file="$output_dir/CMakeLists.txt"

    local -a deps extra_deps libs resource_dirs
    IFS=' ' read -ra deps <<< "${PKG_EXEC_DEPS[$pkg]:-}"
    IFS=' ' read -ra libs <<< "${PKG_LIBRARIES[$pkg]:-}"
    IFS=' ' read -ra extra_deps <<< "${PKG_EXTRA_EXEC_DEPS[$pkg]:-}"
    IFS=' ' read -ra resource_dirs <<< "${PKG_RESOURCE_DIRS[$pkg]:-}"

    {
        echo "cmake_minimum_required(VERSION 3.8)"
        echo "project(${pkg})"
        echo ""
        echo "find_package(ament_cmake REQUIRED)"
        for dep in "${deps[@]}"; do
            echo "find_package(${dep} REQUIRED)"
        done
        echo ""
        echo "# Detect target architecture"
        echo "if(CMAKE_SYSTEM_PROCESSOR MATCHES \"aarch64\")"
        echo "  set(NEXUS_ARCH \"aarch64\")"
        echo "else()"
        echo "  set(NEXUS_ARCH \"x86_64\")"
        echo "endif()"
        echo ""
        echo "# Install prebuilt libraries for the target architecture"
        echo "if(EXISTS \"\${CMAKE_CURRENT_SOURCE_DIR}/lib/\${NEXUS_ARCH}\")"
        echo "  install(DIRECTORY \${CMAKE_CURRENT_SOURCE_DIR}/lib/\${NEXUS_ARCH}/"
        echo "    DESTINATION lib"
        echo "    USE_SOURCE_PERMISSIONS"
        echo "    PATTERN \"${pkg}\" EXCLUDE"
        echo "  )"
        echo "endif()"
        echo ""
        echo "# Install executables for the target architecture"
        echo "if(EXISTS \"\${CMAKE_CURRENT_SOURCE_DIR}/lib/\${NEXUS_ARCH}/${pkg}\")"
        echo "  install(DIRECTORY \${CMAKE_CURRENT_SOURCE_DIR}/lib/\${NEXUS_ARCH}/${pkg}/"
        echo "    DESTINATION lib/${pkg}"
        echo "    USE_SOURCE_PERMISSIONS"
        echo "  )"
        echo "endif()"
        echo ""
        echo "# Install headers"
        echo 'if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/include")'
        echo '  install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/'
        echo '    DESTINATION include'
        echo '  )'
        echo 'endif()'
        echo ""

        for resdir in "${resource_dirs[@]}"; do
            echo "if(EXISTS \"\${CMAKE_CURRENT_SOURCE_DIR}/${resdir}\")"
            echo "  install(DIRECTORY ${resdir}/"
            echo "    DESTINATION share/\${PROJECT_NAME}/${resdir}"
            echo "  )"
            echo "endif()"
            echo ""
        done

        if [ "$pkg" = "robot_controller" ]; then
            echo "if(EXISTS \"\${CMAKE_CURRENT_SOURCE_DIR}/share/${pkg}/oscbf\")"
            echo "  install(DIRECTORY share/${pkg}/oscbf/"
            echo "    DESTINATION share/\${PROJECT_NAME}/oscbf"
            echo "  )"
            echo "endif()"
            echo ""
        fi

        echo "# Export for downstream packages"
        echo "ament_export_include_directories(include)"
        if [ ${#libs[@]} -gt 0 ]; then
            echo "ament_export_libraries(${libs[*]})"
        fi
        echo "ament_export_dependencies(${deps[*]})"
        echo ""
        echo "ament_package()"
    } > "$cmake_file"
}

generate_package_xml() {
    local pkg="$1"
    local output_dir="$2"
    local xml_file="$output_dir/package.xml"
    local src_xml="${WORKSPACE_ROOT}/src/${pkg}/package.xml"

    local -a deps extra_deps
    IFS=' ' read -ra deps <<< "${PKG_EXEC_DEPS[$pkg]:-}"
    IFS=' ' read -ra extra_deps <<< "${PKG_EXTRA_EXEC_DEPS[$pkg]:-}"

    local version="1.0.0"
    local description="${pkg} - prebuilt release"

    if [ -f "$src_xml" ]; then
        local v
        v=$(grep -oP '(?<=<version>)[^<]+' "$src_xml" 2>/dev/null || true)
        [ -n "$v" ] && version="$v"
        local d
        d=$(grep -oP '(?<=<description>)[^<]+' "$src_xml" 2>/dev/null || true)
        [ -n "$d" ] && description="$d"
    fi

    {
        echo '<?xml version="1.0"?>'
        echo '<package format="3">'
        echo "  <name>${pkg}</name>"
        echo "  <version>${version}</version>"
        echo "  <description>${description} (prebuilt)</description>"
        echo '  <maintainer email="infra@pegasus-ai.cn">Infra Embedded</maintainer>'
        echo '  <license>Proprietary</license>'
        echo ''
        echo '  <buildtool_depend>ament_cmake</buildtool_depend>'
        echo ''
        for dep in "${deps[@]}"; do
            echo "  <exec_depend>${dep}</exec_depend>"
        done
        for dep in "${extra_deps[@]}"; do
            echo "  <exec_depend>${dep}</exec_depend>"
        done
        echo ''
        echo '  <export>'
        echo '    <build_type>ament_cmake</build_type>'
        echo '  </export>'
        echo '</package>'
    } > "$xml_file"
}

###############################################################################
# 步骤 1: 构建
###############################################################################

if [ "$SKIP_BUILD" = false ]; then
    log "Step 1: Building workspace in Release mode..."
    cd "$WORKSPACE_ROOT"

    ALL_PACKAGES=("${RELEASE_PACKAGES[@]}" "${MSG_PACKAGES[@]}")

    colcon build \
        --packages-up-to "${ALL_PACKAGES[@]}" \
        --cmake-args -DCMAKE_BUILD_TYPE=Release

    log "Build completed."
else
    log "Step 1: Skipping build (--skip-build)."
fi

###############################################################################
# 步骤 2: 收集产物
###############################################################################

log "Step 2: Collecting release artifacts..."

# ---- 消息包: 复制源码包到输出目录（与功能包同级，客户解压后直接编译）----
for msg_pkg in "${MSG_PACKAGES[@]}"; do
    log "  Collecting message package source: $msg_pkg"
    MSG_SRC_DIR="${WORKSPACE_ROOT}/src/${msg_pkg}"

    if [ ! -d "$MSG_SRC_DIR" ]; then
        err "Message package source not found: $MSG_SRC_DIR"
    fi

    cp -rL "$MSG_SRC_DIR" "${OUTPUT_DIR}/${msg_pkg}"
    log "  Message package $msg_pkg collected."
done

# ---- 功能包 ----
for pkg in "${RELEASE_PACKAGES[@]}"; do
    log "  Collecting package: $pkg"

    PKG_SRC_DIR="${WORKSPACE_ROOT}/src/${pkg}"
    PKG_INSTALL_DIR="${INSTALL_DIR}/${pkg}"
    PKG_OUTPUT_DIR="${OUTPUT_DIR}/${pkg}"

    [ -d "$PKG_SRC_DIR" ] || err "Source directory not found: $PKG_SRC_DIR"
    [ -d "$PKG_INSTALL_DIR" ] || err "Install directory not found: $PKG_INSTALL_DIR. Did you build?"

    mkdir -p "$PKG_OUTPUT_DIR"

    # 2a: 收集预编译库（放入 lib/${ARCH}/ 子目录）
    # -L: find 跟随符号链接，cp -L 解引用为实体文件（确保发布包不含主机路径链接）
    ARCH_LIB_DIR="$PKG_OUTPUT_DIR/lib/${ARCH}"
    mkdir -p "$ARCH_LIB_DIR"
    if [ -d "$PKG_INSTALL_DIR/lib" ]; then
        find -L "$PKG_INSTALL_DIR/lib" -maxdepth 1 \( -name "*.so" -o -name "*.so.*" -o -name "*.a" \) \
            -exec cp -L {} "$ARCH_LIB_DIR/" \; 2>/dev/null || true
    fi

    # 收集可执行文件（放入 lib/${ARCH}/${pkg}/ 子目录）
    EXEC_DIR="${PKG_INSTALL_DIR}/lib/${pkg}"
    if [ -d "$EXEC_DIR" ]; then
        mkdir -p "$ARCH_LIB_DIR/${pkg}"
        find -L "$EXEC_DIR" -maxdepth 1 -type f -executable \
            -exec cp -L {} "$ARCH_LIB_DIR/${pkg}/" \; 2>/dev/null || true
    fi

    # 2a-extra: 收集可执行文件的外部运行时依赖（不在 workspace/ROS 安装目录中的 .so）
    # 避免部署机缺少 pinocchio/osqp 等第三方库导致节点无法启动
    for exe_file in "$ARCH_LIB_DIR/${pkg}"/*; do
        [ -f "$exe_file" ] && [ -x "$exe_file" ] || continue
        while IFS= read -r dep_path; do
            [ -f "$dep_path" ] || continue
            # 跳过已在 workspace install/ 或 /opt/ros/ 中的库（部署机可通过 apt/colcon 获取）
            case "$dep_path" in
                "${INSTALL_DIR}"/*|/opt/ros/*|/lib/*|/usr/lib/*) continue ;;
            esac
            dep_name="$(basename "$dep_path")"
            if [ ! -f "$ARCH_LIB_DIR/$dep_name" ]; then
                cp -L "$dep_path" "$ARCH_LIB_DIR/"
                log "    Collected external runtime dep: $dep_name"
            fi
        done < <(ldd "$exe_file" 2>/dev/null | grep '=>' | awk '{print $3}' | grep -v 'not found')
    done

    # 2a-fixrpath: 将可执行文件的 RUNPATH 修补为可移植的相对路径
    # 避免硬编码编译机绝对路径导致部署机找不到库
    if command -v patchelf >/dev/null 2>&1; then
        for exe_file in "$ARCH_LIB_DIR/${pkg}"/*; do
            [ -f "$exe_file" ] && [ -x "$exe_file" ] || continue
            # 可执行文件在 lib/<ARCH>/<pkg>/，.so 在同级的 lib/<ARCH>/ 下，故 $ORIGIN/.. 即可（勿用 $ORIGIN/../lib，会指向不存在的 lib/lib/）
            patchelf --set-rpath '$ORIGIN/..' "$exe_file" 2>/dev/null || true
        done
    else
        log "  WARNING: patchelf not found, skipping RPATH fix. Install with: sudo apt install patchelf"
    fi

    # 2b: 收集头文件（全量自动模式）
    #     - 排除 detail/ 子目录（PIMPL 内部实现，不分发给客户）
    if [ -d "$PKG_SRC_DIR/include" ]; then
        mkdir -p "$PKG_OUTPUT_DIR/include"
        rsync -aL --exclude='detail/' "$PKG_SRC_DIR/include/" "$PKG_OUTPUT_DIR/include/"
    fi

    # 2c: 收集资源文件
    IFS=' ' read -ra RESOURCE_DIRS_ARR <<< "${PKG_RESOURCE_DIRS[$pkg]:-}"
    for resdir in "${RESOURCE_DIRS_ARR[@]}"; do
        if [ -d "$PKG_SRC_DIR/$resdir" ]; then
            cp -rL "$PKG_SRC_DIR/$resdir" "$PKG_OUTPUT_DIR/$resdir"
        fi
    done

    # robot_controller 特殊资源
    if [ "$pkg" = "robot_controller" ]; then
        OSCBF_SRC="$PKG_SRC_DIR/src/oscbf"
        if [ -d "$OSCBF_SRC" ]; then
            mkdir -p "$PKG_OUTPUT_DIR/share/${pkg}/oscbf"
            find "$OSCBF_SRC" -name "*.urdf" -exec cp {} "$PKG_OUTPUT_DIR/share/${pkg}/oscbf/" \; 2>/dev/null || true
        fi
    fi

    # 2d: 生成客户 CMakeLists.txt
    # teleop_adapter：无 Y1 SDK 时本机不会生成 liby1_arm_adapter.a，导出的 ament 库列表不能含 y1，否则客户侧 colcon 会找库失败
    if [ "$pkg" = "teleop_adapter" ] && [ ! -f "$PKG_INSTALL_DIR/lib/liby1_arm_adapter.a" ]; then
        PKG_LIBRARIES[teleop_adapter]="serial_port device_adapter teleop_arm_adapter ar5_arm_adapter teleop_arm_driver_node"
        log "  teleop_adapter: 未安装 liby1_arm_adapter.a，发布包 ament_export_libraries 将不含 y1_arm_adapter"
    fi
    generate_cmake "$pkg" "$PKG_OUTPUT_DIR"

    # 2e: 生成客户 package.xml
    generate_package_xml "$pkg" "$PKG_OUTPUT_DIR"

    log "  Package $pkg collected."
done

# ---- tool 目录: 直接复制到输出根目录（非 ROS 包，不放 src/）----
TOOL_SRC_DIR="${WORKSPACE_ROOT}/tool"
if [ -d "$TOOL_SRC_DIR" ]; then
    log "  Collecting tool directory..."
    cp -rL "$TOOL_SRC_DIR" "${OUTPUT_DIR}/tool"
    log "  tool/ collected."
fi

###############################################################################
# 步骤 3: 打包
###############################################################################

log "Step 3: Creating release archive..."

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
ARCHIVE_NAME="nexus-sdk-release-${TIMESTAMP}.tar.gz"

cd "$OUTPUT_DIR"
tar czf "${WORKSPACE_ROOT}/${ARCHIVE_NAME}" ./*

log ""
log "=== Release Complete ==="
log "Architecture: ${ARCH}"
log "Archive:   ${WORKSPACE_ROOT}/${ARCHIVE_NAME}"
log "Directory: ${OUTPUT_DIR}"
log ""
log "Contents:"
echo "  tool/"
for msg_pkg in "${MSG_PACKAGES[@]}"; do
    echo "  ${msg_pkg}/"
done
for pkg in "${RELEASE_PACKAGES[@]}"; do
    if [ -d "${OUTPUT_DIR}/${pkg}" ]; then
        echo "  ${pkg}/"
    fi
done
log ""
log "Usage:"
log "  1. Create a workspace:  mkdir -p ~/ws/src"
log "  2. Extract archive:     tar xzf ${ARCHIVE_NAME} -C ~/ws/src"
log "  3. Build:                cd ~/ws && colcon build --packages-up-to ${RELEASE_PACKAGES[*]}"
log "  4. Source and run:       source ~/ws/install/setup.bash"
