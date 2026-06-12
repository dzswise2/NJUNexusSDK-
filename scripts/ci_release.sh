#!/usr/bin/env bash
#
# ci_release.sh — 一键双架构客户发布脚本
#
# 用法:
#   ./scripts/ci_release.sh [--with-docker] [--no-push] [--release-repo DIR]
#
# x86_64  在 Docker 容器内编译（原生速度）
# aarch64 通过交叉编译工具链 + aarch64 sysroot 编译（无 QEMU，稳定可靠）
#
# 前提:
#   - /home/qj00431/infra/jd/nexus-sdk 已拉取最新源码
#   - Docker 已安装
#
set -euo pipefail

###############################################################################
# 配置
###############################################################################

WORKSPACE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RELEASE_REPO_DEFAULT="${HOME}/infra/jd/nexus-sdk-jd_release"
RELEASE_REPO="$RELEASE_REPO_DEFAULT"
WITH_DOCKER=false
DO_PUSH=true
ARCHS=("x86_64" "aarch64")
DOCKER_IMAGE_PREFIX="nexus-sdk-env"
SYSROOT_DIR="/tmp/nexus-aarch64-sysroot"

# colcon 构建包列表（与 make_release.sh 保持一致）
RELEASE_PACKAGES=(teleop_adapter robot_controller nexus_manage human_data)
MSG_PACKAGES=(infra_msg)
ALL_PACKAGES=("${MSG_PACKAGES[@]}" "${RELEASE_PACKAGES[@]}")

###############################################################################
# 参数解析
###############################################################################

while [[ $# -gt 0 ]]; do
    case $1 in
        --with-docker) WITH_DOCKER=true; shift ;;
        --no-push)     DO_PUSH=false; shift ;;
        --release-repo) RELEASE_REPO="$2"; shift 2 ;;
        -h|--help)
            echo "Usage: $0 [--with-docker] [--no-push] [--release-repo DIR]"
            echo ""
            echo "  --with-docker      同时导出 Docker 环境镜像到 release/docker/"
            echo "  --no-push          仅本地 commit，不推送到 remote"
            echo "  --release-repo DIR 客户发布仓库路径"
            echo "                     默认: ${RELEASE_REPO_DEFAULT}"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

###############################################################################
# 辅助函数
###############################################################################

log() { echo "[ci_release] $(date '+%H:%M:%S') $*"; }
err() { echo "[ci_release] ERROR: $*" >&2; exit 1; }

arch_to_platform() {
    case "$1" in
        x86_64)  echo "linux/amd64" ;;
        aarch64) echo "linux/arm64" ;;
        *)       err "Unsupported architecture: $1" ;;
    esac
}

###############################################################################
# 前置检查
###############################################################################

cd "$WORKSPACE_ROOT"

command -v docker >/dev/null 2>&1 || err "docker not found, please install Docker first"

if ! docker info >/dev/null 2>&1; then
    err "docker daemon is not running or current user has no permission"
fi

[ -f scripts/make_release.sh ] || err "make_release.sh not found in scripts/"
[ -f scripts/build_docker_release.sh ] || err "build_docker_release.sh not found in scripts/"

###############################################################################
# Step 1: 准备 Docker 编译镜像
###############################################################################

log "============================================================"
log "Step 1: Preparing Docker build images"
log "============================================================"

for ARCH in "${ARCHS[@]}"; do
    IMAGE="${DOCKER_IMAGE_PREFIX}:${ARCH}"
    if docker image inspect "$IMAGE" >/dev/null 2>&1; then
        log "  Image ${IMAGE} already exists, skip build."
    else
        if [ "$ARCH" = "x86_64" ] && docker image inspect "${DOCKER_IMAGE_PREFIX}:latest" >/dev/null 2>&1; then
            log "  Found legacy tag ${DOCKER_IMAGE_PREFIX}:latest, retagging to ${IMAGE}..."
            docker tag "${DOCKER_IMAGE_PREFIX}:latest" "$IMAGE"
        elif [ "$ARCH" = "aarch64" ] && docker image inspect "${DOCKER_IMAGE_PREFIX}:arm" >/dev/null 2>&1; then
            log "  Found legacy tag ${DOCKER_IMAGE_PREFIX}:arm, retagging to ${IMAGE}..."
            docker tag "${DOCKER_IMAGE_PREFIX}:arm" "$IMAGE"
        else
            log "  Building ${IMAGE} (this may take a while)..."
            "${WORKSPACE_ROOT}/scripts/build_docker_release.sh" \
                --arch "$ARCH" \
                --tag "$IMAGE" \
                --no-export
        fi
    fi
done

###############################################################################
# Step 2: 准备 aarch64 交叉编译环境
###############################################################################

CROSS_IMAGE="${DOCKER_IMAGE_PREFIX}:x86_64-cross"
TOOLCHAIN_FILE="/tmp/nexus-aarch64-toolchain.cmake"

log "============================================================"
log "Step 2: Preparing aarch64 cross-compilation environment"
log "============================================================"

# 2a: 构建交叉编译 Docker 镜像
if docker image inspect "$CROSS_IMAGE" >/dev/null 2>&1; then
    log "  Image ${CROSS_IMAGE} already exists, skip build."
else
    log "  Building cross-compilation image ${CROSS_IMAGE}..."
    docker build -t "$CROSS_IMAGE" - <<'DOCKERFILE' 2>&1 | tail -3
FROM nexus-sdk-env:x86_64
RUN apt-get update && apt-get install -y --no-install-recommends \
    crossbuild-essential-arm64 \
    && rm -rf /var/lib/apt/lists/*
DOCKERFILE
    log "  Cross-compilation image ready."
fi

# 2b: 从 aarch64 Docker 镜像提取 sysroot（缓存到 SYSROOT_DIR）
if [ -d "${SYSROOT_DIR}/opt/ros/humble" ]; then
    log "  Sysroot already exists at ${SYSROOT_DIR}, skip extraction."
else
    log "  Extracting aarch64 sysroot from ${DOCKER_IMAGE_PREFIX}:aarch64..."
    rm -rf "$SYSROOT_DIR"
    mkdir -p "$SYSROOT_DIR"
    CID=$(docker create "${DOCKER_IMAGE_PREFIX}:aarch64")
    docker export "$CID" | tar -C "$SYSROOT_DIR" -xf -
    docker rm "$CID" >/dev/null
    log "  Sysroot extracted ($(du -sh "$SYSROOT_DIR" | cut -f1))."
fi

# 2c: 生成 CMake 工具链文件
cat > "$TOOLCHAIN_FILE" <<'CMAKE'
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)
set(CMAKE_SYSROOT /sysroot)
set(CMAKE_FIND_ROOT_PATH /sysroot)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
# PACKAGE 用 BOTH：系统依赖优先从 sysroot 找，workspace 内部包（如 infra_msg）从 install/ 找
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE BOTH)

# ROS2 / 第三方库前缀（相对于 CMAKE_FIND_ROOT_PATH 即 /sysroot）
set(CMAKE_PREFIX_PATH /opt/ros/humble /opt/osqp /opt/osqp-eigen)

# Python 路径（CMAKE_SYSROOT=/sysroot 会自动添加前缀，此处不要带 /sysroot）
set(PYTHON_EXECUTABLE /usr/bin/python3)
set(PYTHON_LIBRARY /usr/lib/aarch64-linux-gnu/libpython3.10.so)
set(PYTHON_INCLUDE_DIR /usr/include/python3.10)
CMAKE
log "  Toolchain file created at ${TOOLCHAIN_FILE}"

###############################################################################
# Step 3: 清理 + x86_64 Docker 内编译
###############################################################################

log "Cleaning build/, install/, log/ before Docker builds..."
for d in build install log; do
    if [ -d "${WORKSPACE_ROOT}/${d}" ]; then
        rm -rf "${WORKSPACE_ROOT}/${d}" 2>/dev/null || {
            docker run --rm -v "${WORKSPACE_ROOT}:/ws" alpine:latest rm -rf "/ws/${d}" 2>/dev/null || {
                err "Cannot remove ${d}/. Please run: sudo rm -rf ${WORKSPACE_ROOT}/${d}"
            }
        }
    fi
done
log "  Cleaned."

log "============================================================"
log "Step 3: Building x86_64 in Docker (native)"
log "============================================================"

docker run \
    --platform linux/amd64 \
    --rm \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -v "${WORKSPACE_ROOT}:/ws" \
    "${DOCKER_IMAGE_PREFIX}:x86_64" \
    bash -c "cd /ws && ./scripts/make_release.sh --arch x86_64"

log "Saving x86_64 release artifacts..."
RELEASE_X86_DIR="$(mktemp -d "/tmp/release_x86_64_XXXXXX")"
cp -r "${WORKSPACE_ROOT}/release" "$RELEASE_X86_DIR"
log "  Saved to ${RELEASE_X86_DIR}"

###############################################################################
# Step 4: aarch64 交叉编译
###############################################################################

log "Cleaning build/, install/, log/ for aarch64 cross-compilation..."
for d in build install log; do
    if [ -d "${WORKSPACE_ROOT}/${d}" ]; then
        rm -rf "${WORKSPACE_ROOT}/${d}" 2>/dev/null || {
            docker run --rm -v "${WORKSPACE_ROOT}:/ws" alpine:latest rm -rf "/ws/${d}" 2>/dev/null || true
        }
    fi
done

log "============================================================"
log "Step 4: Building aarch64 via cross-compilation"
log "============================================================"

# 获取所有包名作为 colcon 参数
PACKAGES_STR="${ALL_PACKAGES[*]}"

docker run \
    --platform linux/amd64 \
    --rm \
    --user "$(id -u):$(id -g)" \
    -e HOME=/tmp \
    -v "${WORKSPACE_ROOT}:/ws" \
    -v "${SYSROOT_DIR}:/sysroot:ro" \
    -v "${SYSROOT_DIR}/opt/ros/humble:/opt/ros/humble:ro" \
    -v "${SYSROOT_DIR}/opt/osqp:/opt/osqp:ro" \
    -v "${SYSROOT_DIR}/opt/osqp-eigen:/opt/osqp-eigen:ro" \
    -v "${SYSROOT_DIR}/usr/lib/aarch64-linux-gnu:/usr/lib/aarch64-linux-gnu:ro" \
    -v "${SYSROOT_DIR}/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu:ro" \
    -v "${TOOLCHAIN_FILE}:/toolchain.cmake:ro" \
    "$CROSS_IMAGE" \
    bash -c "
        set -euo pipefail
        cd /ws
        colcon build \
            --packages-up-to ${PACKAGES_STR} \
            --cmake-args \
                -DCMAKE_TOOLCHAIN_FILE=/toolchain.cmake \
                -DCMAKE_BUILD_TYPE=Release \
                -DPYTHON_EXECUTABLE=/usr/bin/python3 \
                -DPython3_EXECUTABLE=/usr/bin/python3
        ./scripts/make_release.sh --arch aarch64 --skip-build
    "

###############################################################################
# Step 5: 合并双架构 lib/
###############################################################################

log "============================================================"
log "Step 5: Merging both architectures"
log "============================================================"

RELEASE_DIR="${WORKSPACE_ROOT}/release"

for pkg_dir in "${RELEASE_X86_DIR}/release/"*/; do
    [ -d "$pkg_dir" ] || continue
    pkg_name="$(basename "$pkg_dir")"

    [ "$pkg_name" = "tool" ] && continue
    [ "$pkg_name" = "docker" ] && continue

    SRC_LIB="${pkg_dir}lib/x86_64"
    DST_LIB="${RELEASE_DIR}/${pkg_name}/lib"

    if [ -d "$SRC_LIB" ]; then
        mkdir -p "${DST_LIB}/x86_64"
        cp -r "${SRC_LIB}/"* "${DST_LIB}/x86_64/"
        log "  Merged lib/x86_64/ for ${pkg_name}"
    fi
done

rm -rf "$RELEASE_X86_DIR"
log "  Merge complete."

###############################################################################
# Step 6: (可选) 导出 Docker 环境镜像
###############################################################################

if [ "$WITH_DOCKER" = true ]; then
    log "============================================================"
    log "Step 6: Exporting Docker environment images"
    log "============================================================"

    for ARCH in "${ARCHS[@]}"; do
        DOCKER_OUTPUT="${RELEASE_DIR}/docker/${ARCH}"
        mkdir -p "$DOCKER_OUTPUT"
        ARCHIVE_NAME="nexus-sdk-env-${ARCH}.docker.tar.gz"
        log "  Exporting ${DOCKER_IMAGE_PREFIX}:${ARCH} -> ${ARCHIVE_NAME}..."
        docker save "${DOCKER_IMAGE_PREFIX}:${ARCH}" | gzip > "${DOCKER_OUTPUT}/${ARCHIVE_NAME}"
        log "    $(du -h "${DOCKER_OUTPUT}/${ARCHIVE_NAME}" | cut -f1)"
    done
fi

###############################################################################
# Step 7: 同步到客户发布仓库
###############################################################################

log "============================================================"
log "Step 7: Syncing to release repo: ${RELEASE_REPO}"
log "============================================================"

if [ -d "${RELEASE_REPO}/.git" ]; then
    log "  Pulling latest from release repo..."
    git -C "$RELEASE_REPO" pull --rebase origin main 2>/dev/null || {
        log "  WARNING: git pull failed, continuing with local state"
    }
else
    log "  Cloning release repo..."
    RELEASE_REMOTE="https://github.com/YYDS-JH/nexus-sdk-jd_release.git"
    git clone "$RELEASE_REMOTE" "$RELEASE_REPO" || err "Failed to clone ${RELEASE_REMOTE}"
fi

if [ -d "${RELEASE_DIR}/tool" ]; then
    log "  Syncing tool/..."
    rsync -aL --delete "${RELEASE_DIR}/tool/" "${RELEASE_REPO}/tool/"
fi

if [ -d "${RELEASE_DIR}/docker" ]; then
    for docker_arch_dir in "${RELEASE_DIR}/docker/"*/; do
        [ -d "$docker_arch_dir" ] || continue
        arch_name="$(basename "$docker_arch_dir")"
        mkdir -p "${RELEASE_REPO}/docker/${arch_name}"
        log "  Syncing docker/${arch_name}/..."
        rsync -aL --delete "${RELEASE_DIR}/docker/${arch_name}/" "${RELEASE_REPO}/docker/${arch_name}/"
    done
fi

PKG_SRC_DIR="${RELEASE_REPO}/src"
mkdir -p "$PKG_SRC_DIR"

for pkg_dir in "${RELEASE_DIR}/"*/; do
    [ -d "$pkg_dir" ] || continue
    pkg_name="$(basename "$pkg_dir")"

    [ "$pkg_name" = "tool" ] && continue
    [ "$pkg_name" = "docker" ] && continue

    DST_PKG="${PKG_SRC_DIR}/${pkg_name}"
    mkdir -p "$DST_PKG"

    log "  Syncing ${pkg_name} -> src/${pkg_name}/..."

    rsync -aL --delete \
        --exclude='lib/' \
        "${pkg_dir}/" "${DST_PKG}/"

    for lib_arch_dir in "${pkg_dir}/lib/"*/; do
        [ -d "$lib_arch_dir" ] || continue
        arch_name="$(basename "$lib_arch_dir")"
        mkdir -p "${DST_PKG}/lib/${arch_name}"
        rsync -aL --delete \
            "${lib_arch_dir}/" "${DST_PKG}/lib/${arch_name}/"
        log "    lib/${arch_name}/ synced"
    done
done

###############################################################################
# Step 8: Commit & Push
###############################################################################

log "============================================================"
log "Step 8: Committing and pushing"
log "============================================================"

cd "$RELEASE_REPO"

git add -A

if git diff --cached --quiet; then
    log "  No changes to commit. Release repo is already up-to-date."
else
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    COMMIT_MSG="release: dual-arch (x86_64 + aarch64) ${TIMESTAMP}"
    git commit -m "$COMMIT_MSG"
    log "  Committed: ${COMMIT_MSG}"

    if [ "$DO_PUSH" = true ]; then
        log "  Pushing to origin main..."
        git push origin main
        log "  Push completed."
    else
        log "  Skipped push (--no-push)."
    fi
fi

cd "$WORKSPACE_ROOT"

###############################################################################
# 完成
###############################################################################

log "============================================================"
log "=== CI Release Complete ==="
log "============================================================"
log ""
log "Release repo: ${RELEASE_REPO}"
log "Branch:       main"
log "Architectures: x86_64 + aarch64"
log ""
log "Contents:"
echo "  tool/"
for pkg_dir in "${RELEASE_DIR}/"*/; do
    [ -d "$pkg_dir" ] || continue
    pkg_name="$(basename "$pkg_dir")"
    [ "$pkg_name" = "tool" ] && continue
    [ "$pkg_name" = "docker" ] && continue
    echo "  src/${pkg_name}/ (lib/x86_64/ + lib/aarch64/)"
done
if [ "$WITH_DOCKER" = true ]; then
    echo "  docker/x86_64/"
    echo "  docker/aarch64/"
fi
log ""
log "Client usage:"
log "  git clone https://github.com/YYDS-JH/nexus-sdk-jd_release.git ~/ws"
log "  cd ~/ws && colcon build"
