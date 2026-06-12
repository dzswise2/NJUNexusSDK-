#!/usr/bin/env bash
#
# build_docker_release.sh — 构建 nexus-sdk 环境镜像并导出为可分发的 tar.gz 文件
#
# 用法:
#   ./scripts/build_docker_release.sh [--output-dir DIR] [--arch ARCH] [--tag TAG] [--no-export]
#
# 默认构建当前平台的原生镜像。通过 --arch 可指定目标架构，当目标架构与
# 本机架构不同时，自动启用 Docker Buildx + QEMU 交叉编译。
#
# 镜像内预装所有构建和运行时依赖（ROS2 Humble、Cyclone DDS、Pinocchio、OSQP 等），
# 不包含源码。使用时将宿主机工作空间挂载到容器 /ws 即可编译和运行。
#
# 导出文件名格式: nexus-sdk-env-${ARCH}.docker.tar.gz
# 导出路径: ${OUTPUT_DIR}/${ARCH}/nexus-sdk-env-${ARCH}.docker.tar.gz
#   （可通过 --output-dir 指定 OUTPUT_DIR，默认为 WORKSPACE_ROOT）
#
set -euo pipefail

###############################################################################
# 默认配置
###############################################################################

WORKSPACE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE_TAG="nexus-sdk-env:latest"
OUTPUT_DIR="${WORKSPACE_ROOT}"
ARCH="$(uname -m)"            # x86_64 or aarch64（目标架构）
NATIVE_ARCH="$(uname -m)"    # 当前主机架构
DO_EXPORT=true
# 可选：克隆 github.com 时使用镜像前缀，例如 https://ghfast.top/ 或 https://mirror.ghproxy.com/
#（完整 URL 为 ${GITHUB_MIRROR_PREFIX}https://github.com/...）
: "${NEXUS_GITHUB_MIRROR_PREFIX:=}"

###############################################################################
# 参数解析
###############################################################################

while [[ $# -gt 0 ]]; do
    case $1 in
        --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
        --arch)       ARCH="$2"; shift 2 ;;
        --tag)        IMAGE_TAG="$2"; shift 2 ;;
        --github-mirror-prefix) NEXUS_GITHUB_MIRROR_PREFIX="$2"; shift 2 ;;
        --no-export)  DO_EXPORT=false; shift ;;
        -h|--help)
            echo "Usage: $0 [--output-dir DIR] [--arch ARCH] [--tag TAG] [--github-mirror-prefix URL] [--no-export]"
            echo "  --arch ARCH       目标架构（默认 uname -m，如 x86_64、aarch64）"
            echo "                    若与本机架构不同，自动使用 Buildx + QEMU 交叉编译"
            echo "  --output-dir DIR  导出根目录（默认工作空间根目录）"
            echo "                    镜像实际写入 DIR/${ARCH}/nexus-sdk-env-${ARCH}.docker.tar.gz"
            echo "  --github-mirror-prefix URL  拉取 osqp 等时使用的 GitHub 反代前缀（亦可设环境变量 NEXUS_GITHUB_MIRROR_PREFIX）"
            echo "                    示例: https://ghfast.top/  或  https://mirror.ghproxy.com/"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

###############################################################################
# 辅助函数
###############################################################################

log() { echo "[build_docker_release] $*"; }
err() { echo "[build_docker_release] ERROR: $*" >&2; exit 1; }

###############################################################################
# 前置检查 & 自动安装 Docker
###############################################################################

if ! command -v docker >/dev/null 2>&1; then
    log "未检测到 docker，正在自动安装..."

    sudo apt-get update
    sudo apt-get install -y ca-certificates curl gnupg

    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
        | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    sudo chmod a+r /etc/apt/keyrings/docker.gpg

    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
      https://download.docker.com/linux/ubuntu \
      $(. /etc/os-release && echo "$VERSION_CODENAME") stable" \
      | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

    sudo apt-get update
    sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin

    sudo systemctl start docker || true

    if ! getent group docker >/dev/null 2>&1; then
        sudo groupadd docker
    fi
    sudo usermod -aG docker "$USER"

    log "Docker 安装完成 ✓"
fi

command -v docker >/dev/null 2>&1 || err "docker 安装失败，请手动安装后重试"

###############################################################################
# Docker 权限检查 — 无权限时自动提升
###############################################################################

DOCKER="docker"
if ! docker info >/dev/null 2>&1; then
    if sudo docker info >/dev/null 2>&1; then
        log "当前用户无 docker 权限，自动使用 sudo 运行 docker 命令"
        DOCKER="sudo docker"

        if ! getent group docker >/dev/null 2>&1; then
            sudo groupadd docker
        fi
        sudo usermod -aG docker "$USER"
        log "已将用户 $USER 加入 docker 组，下次登录后可免 sudo 使用 docker"
    else
        err "无法连接 Docker daemon，请检查 Docker 服务是否启动: sudo systemctl start docker"
    fi
fi

###############################################################################
# Docker 镜像加速 — 国内网络无法访问 Docker Hub 时自动配置镜像源
###############################################################################

DAEMON_JSON="/etc/docker/daemon.json"
MIRRORS=(
    "https://docker.m.daocloud.io"
    "https://registry.cn-hangzhou.aliyuncs.com"
    "https://mirror.ccs.tencentyun.com"
)

ensure_docker_mirror() {
    if [ -f "$DAEMON_JSON" ] && grep -q "registry-mirrors" "$DAEMON_JSON" 2>/dev/null; then
        return 0
    fi

    log "检测 Docker Hub 连通性..."
    if timeout 10 $DOCKER pull --quiet hello-world >/dev/null 2>&1; then
        $DOCKER rmi hello-world >/dev/null 2>&1 || true
        return 0
    fi

    log "Docker Hub 不可达，正在配置国内镜像加速器..."

    local mirror_json
    mirror_json=$(printf '"%s"' "${MIRRORS[0]}")
    for m in "${MIRRORS[@]:1}"; do
        mirror_json+=", \"$m\""
    done

    if [ -f "$DAEMON_JSON" ]; then
        local tmp
        tmp=$(mktemp)
        python3 -c "
import json, sys
cfg = json.load(open('$DAEMON_JSON'))
cfg['registry-mirrors'] = [${mirror_json}]
json.dump(cfg, open('$tmp', 'w'), indent=2)
" 2>/dev/null || echo "{\"registry-mirrors\": [${mirror_json}]}" > "$tmp"
        sudo cp "$tmp" "$DAEMON_JSON"
        rm -f "$tmp"
    else
        echo "{\"registry-mirrors\": [${mirror_json}]}" | sudo tee "$DAEMON_JSON" > /dev/null
    fi

    sudo systemctl daemon-reload
    sudo systemctl restart docker
    log "镜像加速器配置完成 ✓"
}

ensure_docker_mirror

###############################################################################
# 架构映射 & 交叉编译支持
###############################################################################

# 将 uname -m 格式转换为 Docker platform 格式
arch_to_platform() {
    case "$1" in
        x86_64)  echo "linux/amd64" ;;
        aarch64) echo "linux/arm64" ;;
        *)       err "不支持的架构: $1 (支持 x86_64 / aarch64)" ;;
    esac
}

NATIVE_PLATFORM=$(arch_to_platform "$NATIVE_ARCH")
TARGET_PLATFORM=$(arch_to_platform "$ARCH")
if [ "$NATIVE_PLATFORM" != "$TARGET_PLATFORM" ]; then
    CROSS_BUILD=true
    log "交叉编译模式: 本机 ${NATIVE_PLATFORM} → 目标 ${TARGET_PLATFORM}"
else
    CROSS_BUILD=false
fi

setup_qemu() {
    if [ "$CROSS_BUILD" = false ]; then
        return 0
    fi
    log "安装 QEMU 多架构支持..."
    $DOCKER run --rm --privileged multiarch/qemu-user-static --reset -p yes
    log "QEMU 安装完成 ✓"
}

setup_buildx() {
    if [ "$CROSS_BUILD" = false ]; then
        return 0
    fi

    # 确保 buildx 插件可用
    if ! $DOCKER buildx version >/dev/null 2>&1; then
        log "buildx 插件未安装，正在安装..."
        local bx_dir="${HOME}/.docker/cli-plugins"
        mkdir -p "$bx_dir"
        local bx_bin="${bx_dir}/docker-buildx"
        local bx_ver="v0.19.3"
        local bx_url
        case "$NATIVE_ARCH" in
            x86_64)  bx_url="https://github.com/docker/buildx/releases/download/${bx_ver}/buildx-${bx_ver}.linux-amd64" ;;
            aarch64) bx_url="https://github.com/docker/buildx/releases/download/${bx_ver}/buildx-${bx_ver}.linux-arm64" ;;
        esac
        log "下载 buildx ${bx_ver} → ${bx_bin}"
        curl -fsSL "$bx_url" -o "$bx_bin"
        chmod +x "$bx_bin"
        if ! $DOCKER buildx version >/dev/null 2>&1; then
            err "buildx 安装失败，请手动从 https://github.com/docker/buildx/releases 下载 docker-buildx 到 ${bx_dir}/"
        fi
        log "buildx 插件安装完成 ✓"
    fi

    # docker driver 只允许一个 builder 实例，使用默认 default builder
    if $DOCKER buildx inspect default >/dev/null 2>&1; then
        $DOCKER buildx use default
        log "buildx builder (default, docker driver) 就绪 ✓"
        return 0
    fi
    log "创建 buildx builder (docker driver)..."
    $DOCKER buildx create --name default --driver docker --use
    log "buildx builder 就绪 ✓"
}

setup_qemu
setup_buildx

###############################################################################
# 步骤 1: 构建 Docker 镜像
###############################################################################

log "Step 1: Building Docker image '${IMAGE_TAG}' (platform: ${TARGET_PLATFORM})..."
if [[ -n "$NEXUS_GITHUB_MIRROR_PREFIX" ]]; then
  log "GITHUB 镜像前缀: ${NEXUS_GITHUB_MIRROR_PREFIX}"
else
  log "GITHUB 镜像前缀: (无，使用直连 github.com；拉取失败时可设 NEXUS_GITHUB_MIRROR_PREFIX 或 --github-mirror-prefix)"
fi

# 将 Dockerfile 写入临时文件，避免 heredoc 重复
DOCKERFILE_TMP=$(mktemp)
trap "rm -f $DOCKERFILE_TMP" EXIT
cat > "$DOCKERFILE_TMP" <<'DOCKERFILE'
FROM ros:humble-ros-base-jammy

ARG GITHUB_MIRROR_PREFIX=
ENV DEBIAN_FRONTEND=noninteractive

# Intel RealSense SDK 2.0 仓库（Intel 轮换过 GPG 密钥，需同时导入新旧两把）
# 启用 universe 仓库（libgstreamer-plugins-bad1.0-dev 等在此仓库中）
RUN sed -i 's/^Components: main$/Components: main universe/' /etc/apt/sources.list.d/ubuntu.sources 2>/dev/null || true \
  && apt-get update && apt-get install -y --no-install-recommends software-properties-common \
  && apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
  && apt-key adv --keyserver keyserver.ubuntu.com --recv-key FB0B24895113F120 \
  && add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    vim \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    libeigen3-dev \
    libyaml-cpp-dev \
    libevdev-dev \
    libgoogle-glog-dev \
    libassimp-dev \
    libnlopt-dev \
    libnlopt-cxx-dev \
    ros-humble-pinocchio \
    ros-humble-rmw-cyclonedds-cpp \
    patchelf \
    rsync \
    librealsense2-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-nice \
    libsoup2.4-dev \
    nlohmann-json3-dev \
    libopencv-dev \
  && rm -rf /var/lib/apt/lists/*

RUN set -eux; \
  git config --global http.version HTTP/1.1; \
  git config --global http.postBuffer 524288000; \
  git config --global http.lowSpeedLimit 1000; \
  git config --global http.lowSpeedTime 300; \
  MP="${GITHUB_MIRROR_PREFIX}"; \
  for attempt in 1 2 3 4 5; do \
    git clone --branch release-0.6.3 --recursive --depth 1 \
      "${MP}https://github.com/osqp/osqp.git" /tmp/osqp && break; \
    if [ "$attempt" = 5 ]; then echo "osqp: git clone failed after 5 attempts"; exit 1; fi; \
    echo "osqp: clone failed, retrying ($attempt/5) in 25s..."; \
    sleep 25; \
  done; \
  mkdir /tmp/osqp/build && cd /tmp/osqp/build \
  && cmake -DCMAKE_INSTALL_PREFIX=/opt/osqp .. \
  && make -j"$(nproc)" && make install \
  && rm -rf /tmp/osqp

RUN set -eux; \
  MP="${GITHUB_MIRROR_PREFIX}"; \
  for attempt in 1 2 3 4 5; do \
    git clone --depth 1 \
      "${MP}https://github.com/robotology/osqp-eigen.git" /tmp/osqp-eigen && break; \
    if [ "$attempt" = 5 ]; then echo "osqp-eigen: git clone failed after 5 attempts"; exit 1; fi; \
    echo "osqp-eigen: clone failed, retrying ($attempt/5) in 25s..."; \
    sleep 25; \
  done; \
  mkdir /tmp/osqp-eigen/build && cd /tmp/osqp-eigen/build \
  && cmake -DCMAKE_INSTALL_PREFIX=/opt/osqp-eigen \
           -DCMAKE_PREFIX_PATH=/opt/osqp .. \
  && make -j"$(nproc)" && make install \
  && rm -rf /tmp/osqp-eigen

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV OSQP_INSTALL_DIR=/opt/osqp
ENV OSQP_EIGEN_INSTALL_DIR=/opt/osqp-eigen

RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/humble/setup.bash\n[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash\nexec "$@"\n' \
      > /entrypoint.sh && chmod +x /entrypoint.sh

WORKDIR /ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
DOCKERFILE

if [ "$CROSS_BUILD" = true ]; then
    $DOCKER buildx build --platform "$TARGET_PLATFORM" \
        --build-arg "GITHUB_MIRROR_PREFIX=${NEXUS_GITHUB_MIRROR_PREFIX}" \
        -t "$IMAGE_TAG" -f "$DOCKERFILE_TMP" "$WORKSPACE_ROOT" --load
else
    $DOCKER build \
        --build-arg "GITHUB_MIRROR_PREFIX=${NEXUS_GITHUB_MIRROR_PREFIX}" \
        -t "$IMAGE_TAG" -f "$DOCKERFILE_TMP" "$WORKSPACE_ROOT"
fi

rm -f "$DOCKERFILE_TMP"

log "Docker image built successfully: ${IMAGE_TAG}"

###############################################################################
# 步骤 2: 导出镜像
###############################################################################

if [ "$DO_EXPORT" = true ]; then
    log "Step 2: Exporting Docker image (arch=${ARCH})..."

    # 按架构分目录存放，文件名固定（无时间戳），方便 jd_release 分支覆盖更新
    ARCH_OUTPUT_DIR="${OUTPUT_DIR}/${ARCH}"
    mkdir -p "$ARCH_OUTPUT_DIR"
    ARCHIVE_NAME="nexus-sdk-env-${ARCH}.docker.tar.gz"
    ARCHIVE_PATH="${ARCH_OUTPUT_DIR}/${ARCHIVE_NAME}"

    $DOCKER save "$IMAGE_TAG" | gzip > "$ARCHIVE_PATH"

    IMAGE_SIZE=$($DOCKER image inspect "$IMAGE_TAG" --format='{{.Size}}' | awk '{printf "%.0f MB", $1/1024/1024}')
    ARCHIVE_SIZE=$(du -h "$ARCHIVE_PATH" | cut -f1)

    log ""
    log "=== Docker Build Complete ==="
    log "Image:        ${IMAGE_TAG} (${IMAGE_SIZE})"
    log "Platform:     ${TARGET_PLATFORM}"
    log "Architecture: ${ARCH}"
    if [ "$CROSS_BUILD" = true ]; then
        log "Built via:    cross-compilation (QEMU + Buildx)"
    fi
    log "Archive:      ${ARCHIVE_PATH} (${ARCHIVE_SIZE})"
    log ""
    log "客户端使用:"
    if [ "$CROSS_BUILD" = true ]; then
        log "  注意: 在 x86 主机上运行此镜像需 QEMU binfmt 支持（本脚本已自动安装）"
        log "  在 x86 上运行: docker run --platform ${TARGET_PLATFORM} --rm -it ..."
    fi
    log "  1. 加载镜像:  docker load < ${ARCHIVE_NAME}"
    log "  2. 编译并运行:"
    log "     docker run --rm -it --network host --privileged \\"
    log "       -v /dev:/dev \\"
    log "       -v /path/to/nexus-sdk:/ws \\"
    log "       ${IMAGE_TAG}"
    log ""
    log "  容器内操作:"
    log "     colcon build --packages-up-to teleop_adapter robot_controller nexus_manage human_data vision_data_hub"
    log "     source install/setup.bash"
    log "     ros2 launch nexus_manage nexus_arm_to_y1_real_system.launch.py"
else
    log "Step 2: Skipped export (--no-export)."
    log ""
    log "=== Docker Build Complete ==="
    log "Image:        ${IMAGE_TAG}"
    log "Platform:     ${TARGET_PLATFORM}"
    log "Architecture: ${ARCH}"
    if [ "$CROSS_BUILD" = true ]; then
        log "Built via:    cross-compilation (QEMU + Buildx)"
    fi
fi
