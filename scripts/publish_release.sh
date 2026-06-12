#!/usr/bin/env bash
#
# publish_release.sh — 构建发布包并同步到 jd_release 分支
#
# 用法:
#   ./scripts/publish_release.sh [--skip-build] [--with-docker] [--arch ARCH] [--remote REMOTE] [--no-push]
#
# 工作流:
#   1. 调用 make_release.sh 生成 release/ 目录（含架构子目录 lib/${ARCH}/）
#   2. （可选）调用 build_docker_release.sh 导出镜像到 release/docker/${ARCH}/
#   3. 通过 git worktree 检出 jd_release 分支到临时目录（无需切换当前分支）
#   4. 同步产物到 worktree：
#      - 头文件 / launch / config 等架构无关内容：全量同步（覆盖）
#      - lib/${ARCH}/：仅同步当前架构，保留其他架构的已有文件
#      - docker/${ARCH}/：仅同步当前架构镜像，保留其他架构的已有文件
#   5. git commit + push
#   6. 清理 worktree
#
set -euo pipefail

###############################################################################
# 默认配置
###############################################################################

WORKSPACE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
RELEASE_DIR="${WORKSPACE_ROOT}/release"
SKIP_BUILD=false
WITH_DOCKER=false
ARCH="$(uname -m)"
REMOTE="origin"
RELEASE_BRANCH="jd_release"
DO_PUSH=true

###############################################################################
# 参数解析
###############################################################################

while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-build)  SKIP_BUILD=true; shift ;;
        --with-docker) WITH_DOCKER=true; shift ;;
        --arch)        ARCH="$2"; shift 2 ;;
        --remote)      REMOTE="$2"; shift 2 ;;
        --no-push)     DO_PUSH=false; shift ;;
        -h|--help)
            echo "Usage: $0 [--skip-build] [--with-docker] [--arch ARCH] [--remote REMOTE] [--no-push]"
            echo ""
            echo "  --skip-build      跳过 colcon build，使用已有 install/ 产物"
            echo "  --with-docker     同时构建并发布 Docker 镜像到 docker/\${ARCH}/"
            echo "  --arch ARCH       目标架构（默认 uname -m，如 x86_64、aarch64）"
            echo "  --remote REMOTE   git remote 名称（默认 origin）"
            echo "  --no-push         仅本地 commit，不推送到 remote"
            exit 0 ;;
        *) echo "Unknown option: $1"; exit 1 ;;
    esac
done

###############################################################################
# 辅助函数
###############################################################################

log() { echo "[publish_release] $*"; }
err() { echo "[publish_release] ERROR: $*" >&2; exit 1; }

###############################################################################
# 前置检查
###############################################################################

cd "$WORKSPACE_ROOT"

command -v git   >/dev/null 2>&1 || err "git not found"
command -v rsync >/dev/null 2>&1 || err "rsync not found"

# 确认当前目录是 git 仓库
git rev-parse --git-dir >/dev/null 2>&1 || err "Not a git repository: ${WORKSPACE_ROOT}"

# 确认 jd_release 分支存在（本地或远端）
if ! git show-ref --verify --quiet "refs/heads/${RELEASE_BRANCH}" && \
   ! git show-ref --verify --quiet "refs/remotes/${REMOTE}/${RELEASE_BRANCH}"; then
    err "Branch '${RELEASE_BRANCH}' not found locally or in remote '${REMOTE}'. "
fi

###############################################################################
# 步骤 1: 生成 release/ 目录
###############################################################################

log "Step 1: Generating release artifacts (arch=${ARCH})..."

MAKE_RELEASE_ARGS="--arch ${ARCH}"
if [ "$SKIP_BUILD" = true ]; then
    MAKE_RELEASE_ARGS+=" --skip-build"
fi

"${WORKSPACE_ROOT}/scripts/make_release.sh" ${MAKE_RELEASE_ARGS}

[ -d "$RELEASE_DIR" ] || err "release/ directory not found after make_release.sh"

log "Release artifacts ready at: ${RELEASE_DIR}"

###############################################################################
# 步骤 1.5（可选）: 构建并导出 Docker 镜像到 release/docker/${ARCH}/
###############################################################################

DOCKER_RELEASE_DIR="${RELEASE_DIR}/docker"

if [ "$WITH_DOCKER" = true ]; then
    log "Step 1.5: Building Docker image (arch=${ARCH})..."
    "${WORKSPACE_ROOT}/scripts/build_docker_release.sh" \
        --arch "${ARCH}" \
        --output-dir "${DOCKER_RELEASE_DIR}"
    log "Docker image exported to: ${DOCKER_RELEASE_DIR}/${ARCH}/"
else
    log "Step 1.5: Skipped Docker build (use --with-docker to include)."
fi

###############################################################################
# 步骤 2: 检出 jd_release 分支到临时 worktree
###############################################################################

log "Step 2: Setting up git worktree for branch '${RELEASE_BRANCH}'..."

WORKTREE_DIR="$(mktemp -d "/tmp/jd_release_wt_XXXXXX")"

cleanup_worktree() {
    log "Cleaning up worktree..."
    git worktree remove --force "$WORKTREE_DIR" 2>/dev/null || true
    rm -rf "$WORKTREE_DIR"
}
trap cleanup_worktree EXIT

# 如果本地没有该分支，先从 remote 拉取
if ! git show-ref --verify --quiet "refs/heads/${RELEASE_BRANCH}"; then
    log "  Local branch not found, fetching from ${REMOTE}/${RELEASE_BRANCH}..."
    git fetch "$REMOTE" "${RELEASE_BRANCH}:${RELEASE_BRANCH}"
fi

git worktree add "$WORKTREE_DIR" "$RELEASE_BRANCH"
log "  Worktree created at: ${WORKTREE_DIR}"

# 拉取 remote 最新状态，确保 fast-forward push
cd "$WORKTREE_DIR"
if git ls-remote --exit-code "$REMOTE" "${RELEASE_BRANCH}" >/dev/null 2>&1; then
    log "  Pulling latest from ${REMOTE}/${RELEASE_BRANCH}..."
    git pull --rebase "$REMOTE" "$RELEASE_BRANCH" 2>/dev/null || true
fi
cd "$WORKSPACE_ROOT"

###############################################################################
# 步骤 3: 同步产物到 worktree
###############################################################################

log "Step 3: Syncing release artifacts to worktree (arch=${ARCH})..."

# 功能包统一放到 src/ 子目录下
DST_SRC_DIR="${WORKTREE_DIR}/src"
mkdir -p "$DST_SRC_DIR"

# tool/ 目录同步到 worktree 根目录（非 ROS 包）
if [ -d "${RELEASE_DIR}/tool" ]; then
    log "  Syncing tool/ -> tool/..."
    rsync -aL --delete "${RELEASE_DIR}/tool/" "${WORKTREE_DIR}/tool/"
fi

# docker/${ARCH}/ 增量同步（保留其他架构的已有镜像）
if [ -d "${DOCKER_RELEASE_DIR}/${ARCH}" ]; then
    log "  Syncing docker/${ARCH}/ -> docker/${ARCH}/..."
    mkdir -p "${WORKTREE_DIR}/docker/${ARCH}"
    rsync -aL --delete \
        "${DOCKER_RELEASE_DIR}/${ARCH}/" \
        "${WORKTREE_DIR}/docker/${ARCH}/"
    log "    docker/${ARCH}/ synced"
fi

# 列出 release/ 下的所有功能包（排除 tool/ 和 docker/）
PACKAGES=()
for d in "$RELEASE_DIR"/*/; do
    pkg_name="$(basename "$d")"
    [ -d "$d" ] && [ "$pkg_name" != "tool" ] && [ "$pkg_name" != "docker" ] && PACKAGES+=("$pkg_name")
done

for pkg in "${PACKAGES[@]}"; do
    SRC_PKG_DIR="${RELEASE_DIR}/${pkg}"
    DST_PKG_DIR="${DST_SRC_DIR}/${pkg}"

    log "  Syncing ${pkg} -> src/${pkg}..."

    mkdir -p "$DST_PKG_DIR"

    # 3a: 同步架构无关内容（全量覆盖，但排除 lib/ 目录）
    #     include/ launch/ config/ urdf/ meshes/ specs/ share/ CMakeLists.txt package.xml
    rsync -aL --delete \
        --exclude='lib/' \
        "${SRC_PKG_DIR}/" "${DST_PKG_DIR}/"

    # 3b: 同步当前架构的 lib/${ARCH}/ （增量，不删除其他架构目录）
    if [ -d "${SRC_PKG_DIR}/lib/${ARCH}" ]; then
        mkdir -p "${DST_PKG_DIR}/lib/${ARCH}"
        rsync -aL --delete \
            "${SRC_PKG_DIR}/lib/${ARCH}/" \
            "${DST_PKG_DIR}/lib/${ARCH}/"
        log "    lib/${ARCH}/ synced"
    else
        log "    No lib/${ARCH}/ found, skipping library sync for ${pkg}"
    fi
done

###############################################################################
# 步骤 4: Commit
###############################################################################

log "Step 4: Committing to branch '${RELEASE_BRANCH}'..."

cd "$WORKTREE_DIR"

git add -A

if git diff --cached --quiet; then
    log "  No changes to commit. Branch '${RELEASE_BRANCH}' is already up-to-date."
else
    TIMESTAMP=$(date +%Y%m%d_%H%M%S)
    COMMIT_MSG="release(${ARCH}): ${TIMESTAMP}"
    git commit -m "$COMMIT_MSG"
    log "  Committed: ${COMMIT_MSG}"

    ###############################################################################
    # 步骤 5: Push
    ###############################################################################

    if [ "$DO_PUSH" = true ]; then
        log "Step 5: Pushing to ${REMOTE}/${RELEASE_BRANCH}..."
        git push "$REMOTE" "${RELEASE_BRANCH}"
        log "  Pushed successfully."
    else
        log "Step 5: Skipped push (--no-push)."
    fi
fi

cd "$WORKSPACE_ROOT"

###############################################################################
# 完成
###############################################################################

log ""
log "=== Publish Complete ==="
log "Branch:       ${RELEASE_BRANCH}"
log "Architecture: ${ARCH}"
log "Remote:       ${REMOTE}"
log ""
log "客户克隆并使用发布包:"
log "  git clone --branch ${RELEASE_BRANCH} <repo_url> ~/ws"
log "  cd ~/ws && colcon build --packages-up-to teleop_adapter robot_controller nexus_manage human_data"
log "  source ~/ws/install/setup.bash"
