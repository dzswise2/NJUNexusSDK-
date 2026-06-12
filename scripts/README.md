# scripts/ 使用说明

本目录包含客户发布包的制造、发布与质量保障脚本。

## make_release.sh — 客户发布包制造脚本

从源码工作空间构建闭源客户发布包。发布包只包含预编译二进制（`.so`/`.a`）、
可执行文件、公开头文件和资源文件，不包含任何 `.cpp` 源码或 PIMPL 内部头文件（`detail/`）。

### 用法

```bash
# 完整流程（构建 + 收集 + 打包，自动检测当前架构）
./scripts/make_release.sh

# 跳过构建（使用已有 install/ 下的产物）
./scripts/make_release.sh --skip-build

# 显式指定架构（如在 x86_64 机器上收集，但标记为 aarch64 需交叉编译时使用）
./scripts/make_release.sh --arch aarch64

# 指定输出目录
./scripts/make_release.sh --output-dir /tmp/my_release
```

### 前提条件

- 当前目录为 `nexus-sdk` 工作空间根目录
- 已安装 ROS 2、`colcon`、`rsync` 以及所有构建依赖
- 厂商 SDK 库文件已正确放置（Rokae SDK、Y1 SDK 等）

### 输出产物

执行完成后在工作空间根目录生成 `nexus-sdk-release-<时间戳>.tar.gz`，内部结构：

```
nexus-sdk-release-YYYYMMDD_HHMMSS.tar.gz
├── infra_msg/            # 消息定义源码包（与功能包一起编译）
├── teleop_adapter/       # 功能包
│   ├── CMakeLists.txt    #   自动生成的客户 CMake（含架构检测）
│   ├── package.xml       #   自动生成的包描述
│   ├── lib/              #   预编译库 + 可执行文件（按架构分目录）
│   │   ├── x86_64/       #     x86_64 平台的库与可执行文件
│   │   └── aarch64/      #     aarch64 平台的库与可执行文件
│   ├── include/          #   公开头文件（不含 detail/）
│   ├── launch/           #   Launch 文件
│   ├── config/           #   配置文件
│   └── urdf/             #   URDF 模型
├── robot_controller/
├── nexus_manage/
└── human_data/
```

### 客户使用方式

```bash
# 1. 创建工作空间
mkdir -p ~/ws/src

# 2. 解压发布包（infra_msg 和功能包平铺在 src/ 下）
tar xzf nexus-sdk-release-*.tar.gz -C ~/ws/src

# 3. 编译（infra_msg 会作为依赖自动先编译）
cd ~/ws && colcon build --packages-up-to teleop_adapter robot_controller nexus_manage human_data

# 4. Source 并运行
source ~/ws/install/setup.bash
ros2 launch teleop_adapter slave_ar5_single.launch.py
```

### 配置修改

如需新增发布包或调整库/依赖列表，编辑脚本顶部的配置区域：

| 变量 | 说明 |
|------|------|
| `RELEASE_PACKAGES` | 需要制造的功能包列表 |
| `MSG_PACKAGES` | 需要分发的消息包列表 |
| `PKG_LIBRARIES[pkg]` | 各包包含的库目标名称 |
| `PKG_EXECUTABLES[pkg]` | 各包包含的可执行文件名称 |
| `PKG_EXEC_DEPS[pkg]` | 各包的核心运行时依赖 |
| `PKG_EXTRA_EXEC_DEPS[pkg]` | 各包的额外依赖（Eigen、pinocchio 等） |
| `PKG_RESOURCE_DIRS[pkg]` | 各包需要分发的资源目录 |

---

## publish_release.sh — 一键发布到 jd_release 分支

将 `make_release.sh` 生成的发布工程自动同步到 `jd_release` 分支并推送，无需手动复制文件或切换分支。
内部使用 `git worktree` 实现，不影响当前工作分支。

### 用法

```bash
# 完整流程（构建 + 收集 + 发布到 jd_release 分支）
./scripts/publish_release.sh

# 同时构建并发布 Docker 镜像（导出到 jd_release 分支的 docker/${ARCH}/）
./scripts/publish_release.sh --with-docker

# 跳过构建，只同步已有 install/ 产物
./scripts/publish_release.sh --skip-build

# 仅本地 commit，不推送（用于验证）
./scripts/publish_release.sh --skip-build --no-push

# aarch64 机器上运行（追加 aarch64 库，不影响已有 x86_64 库）
./scripts/publish_release.sh --arch aarch64
```

### 前提条件

- 当前目录为 `nexus-sdk` 工作空间根目录
- `jd_release` 分支已存在（本地或远端）
- 已安装 `git`、`rsync`

### 工作流程

```
publish_release.sh
  │
  ├─ 1. 调用 make_release.sh --arch ${ARCH}
  │      → 生成 release/ 目录（含 lib/${ARCH}/）
  │
  ├─ 2. git worktree add /tmp/jd_release_wt_XXXX jd_release
  │      → 在临时目录检出 jd_release，不切换当前分支
  │
  ├─ 3. rsync 同步
  │      ├─ 架构无关内容（include/ launch/ config/ urdf/ CMakeLists.txt package.xml）
  │      │   → 全量覆盖
  │      └─ lib/${ARCH}/
  │          → 增量同步，其他架构目录（如 lib/aarch64/）保持不变
  │
  ├─ 4. git commit -m "release(x86_64): 20260421_143000"
  │
  ├─ 5. git push origin jd_release
  │
  └─ 6. git worktree remove（自动清理）
```

### 多架构发布工作流

`jd_release` 分支的 `lib/` 目录由两个平台分别填充，互不覆盖：

```
jd_release 分支
├── docker/
│   ├── x86_64/
│   │   └── nexus-sdk-env-x86_64.docker.tar.gz   ← --with-docker 时填充
│   └── aarch64/
│       └── nexus-sdk-env-aarch64.docker.tar.gz  ← --with-docker 时填充
├── tool/
└── src/
    ├── teleop_adapter/
    │   └── lib/
    │       ├── x86_64/   ← 在 x86_64 机器运行一次后填充
    │       └── aarch64/  ← 在 aarch64 机器运行一次后填充
    └── ...
```

典型的两平台发布流程：

```bash
# x86_64 机器（开发机）
./scripts/publish_release.sh

# aarch64 机器（部署机，克隆同一仓库后）
./scripts/publish_release.sh
```

两次执行后，`jd_release` 分支同时包含两套平台的可执行文件和库，客户在任意平台 `colcon build` 时 CMake 会自动选择对应架构的 `lib/${CMAKE_SYSTEM_PROCESSOR}/`。

### 客户使用方式

```bash
# 克隆发布分支（仓库中 src/ 目录即为 colcon 工作空间）
git clone --branch jd_release <repo_url> ~/ws

# 编译
cd ~/ws && colcon build --packages-up-to teleop_adapter robot_controller nexus_manage human_data

# Source 并运行
source ~/ws/install/setup.bash
ros2 launch teleop_adapter slave_ar5_single.launch.py
```

---

## build_docker_release.sh — Docker 环境镜像构建与导出

构建预装所有依赖的 Docker 环境镜像（`nexus-sdk-env`），导出为可分发的 `.tar.gz` 文件。
镜像内不含源码，使用时将宿主机工作空间挂载到容器 `/ws` 即可编译和运行。

### 镜像内预装内容

| 类别 | 内容 |
|------|------|
| 基础系统 | Ubuntu 22.04 + ROS 2 Humble (`ros-base`) |
| DDS 中间件 | Cyclone DDS（`rmw_cyclonedds_cpp`，已设为默认） |
| 构建工具 | `build-essential`, `cmake`, `colcon`, `git` |
| 系统库 | Eigen3, yaml-cpp, libevdev, glog, assimp, nlopt |
| ROS 依赖 | `ros-humble-pinocchio` |
| QP 求解器 | OSQP v0.6.3 + OsqpEigen（源码编译，安装于 `/opt/`） |

### 构建镜像

```bash
# 完整流程（构建 + 导出 .tar.gz，自动检测当前架构）
./scripts/build_docker_release.sh

# 仅构建，不导出文件
./scripts/build_docker_release.sh --no-export

# 交叉编译 — 在 x86_64 机器上构建 aarch64 镜像
./scripts/build_docker_release.sh --arch aarch64

# 自定义镜像名和输出目录
./scripts/build_docker_release.sh --tag nexus-sdk-env:v2 --output-dir /tmp
```

### 交叉编译支持

在 x86_64 开发机上构建 aarch64（ARM64）镜像时，脚本自动启用 Docker Buildx + QEMU 用户态模拟：

1. **首次交叉编译**：脚本自动拉取并安装 QEMU 多架构支持（`multiarch/qemu-user-static`），注册 binfmt 处理器，创建 buildx builder（`cross-builder`）。此步骤只需执行一次，重启后仍然有效。
2. **构建过程**：QEMU 模拟执行 aarch64 的 `apt-get`、`cmake`、`gcc` 等工具，但产出的二进制文件是纯 aarch64 机器码，与在 aarch64 实机上构建的产物完全一致。
3. **最终镜像**：不含 QEMU，部署到 aarch64 实机上是原生性能。

```bash
# x86_64 开发机 → 构建 aarch64 镜像
./scripts/build_docker_release.sh --arch aarch64

# 将导出的 .tar.gz 拷贝到 aarch64 实机
scp aarch64/nexus-sdk-env-aarch64.docker.tar.gz user@orin:/tmp/

# aarch64 实机 → 加载并运行（纯原生性能）
docker load < /tmp/nexus-sdk-env-aarch64.docker.tar.gz
docker run --rm -it --network host --privileged \
  -v /dev:/dev \
  -v /path/to/nexus-sdk:/ws \
  nexus-sdk-env:latest
```

如需在 x86_64 上运行 aarch64 镜像（用于交叉编译验证），指定 `--platform`：

```bash
docker run --platform linux/arm64 --rm -it --network host --privileged \
  -v /dev:/dev \
  -v /path/to/nexus-sdk:/ws \
  nexus-sdk-env:latest
# QEMU 自动翻译 aarch64 指令，容器内 gcc 编译出的仍是 aarch64 二进制
```

### 客户端加载镜像

```bash
docker load < nexus-sdk-env-YYYYMMDD_HHMMSS.docker.tar.gz
```

### 使用方式

#### 交互式开发（推荐）

```bash
# 启动容器，挂载宿主机源码目录
docker run --rm -it --network host --privileged \
  -v /dev:/dev \
  -v /path/to/nexus-sdk:/ws \
  nexus-sdk-env:latest

# --- 以下操作在容器内 ---

# 首次编译（或源码有变更后）
colcon build --packages-up-to teleop_adapter robot_controller nexus_manage human_data \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# source 工作空间
source install/setup.bash

# 启动系统
ros2 launch nexus_manage nexus_arm_to_y1_real_system.launch.py
```

#### 一行式运行（已编译过的项目）

```bash
docker run --rm -it --network host --privileged \
  -v /dev:/dev \
  -v /path/to/nexus-sdk:/ws \
  nexus-sdk-env:latest \
  bash -c "source install/setup.bash && ros2 launch nexus_manage nexus_arm_to_y1_real_system.launch.py"
```

#### 自定义 Cyclone DDS 配置

```bash
docker run --rm -it --network host --privileged \
  -v /dev:/dev \
  -v /path/to/nexus-sdk:/ws \
  -v /path/to/cyclonedds.xml:/ws/cyclonedds.xml:ro \
  -e CYCLONEDDS_URI=file:///ws/cyclonedds.xml \
  nexus-sdk-env:latest
```

### 注意事项

- **首次编译**：如果宿主机上已有 `build/` 目录是在宿主机直接编译的，路径缓存会冲突（宿主机路径 vs 容器内 `/ws`）。首次在容器内编译前需清理：`rm -rf build/ install/ log/`
- **docker run 参数说明**：
  - `--network host`：ROS 2 DDS 节点发现和机器人网络通信
  - `--privileged`：访问 `/dev/input/*` 输入设备（键盘/脚踏板 evdev）
  - `-v /dev:/dev`：挂载宿主机 `/dev`，使 udev 规则创建的设备符号链接（如 `/dev/nexus_arm`）在容器内可见。Docker 默认创建独立的 devtmpfs，不会继承宿主机的 udev 符号链接
  - `-v ...:/ws`：将宿主机源码目录挂载到容器工作空间
- **编译产物持久化**：由于 `-v` 挂载，`build/`、`install/`、`log/` 写入的是宿主机磁盘，容器退出后不会丢失，下次启动无需重新编译

---

## ci_release.sh — 一键双架构客户发布

在 Docker 容器中分别编译 x86_64 和 aarch64 产物，合并后推送到客户发布仓库 `nexus-sdk-jd_release`。**单机一次执行**，无需分别在两台机器上操作。

与 `publish_release.sh` 的区别：前者发布到同仓库的 `jd_release` 分支，本脚本发布到**独立的外部仓库**（`nexus-sdk-jd_release`），且支持双架构合并。

### 用法

```bash
# 完整双架构发布（构建 + 合并 + 推送到客户仓库）
./scripts/ci_release.sh

# 同时导出 Docker 环境镜像到客户仓库 docker/ 目录
./scripts/ci_release.sh --with-docker

# 仅本地 commit，不推送（用于验证）
./scripts/ci_release.sh --no-push

# 指定客户发布仓库路径
./scripts/ci_release.sh --release-repo /path/to/nexus-sdk-jd_release
```

### 前提条件

- 当前目录为 `nexus-sdk` 工作空间根目录，源码已拉取最新
- Docker 已安装且当前用户有权限
- Docker 镜像 `nexus-sdk-env:x86_64` 和 `nexus-sdk-env:aarch64` 就绪（不存在时自动调用 `build_docker_release.sh` 构建）
- 客户发布仓库路径（默认 `~/infra/jd/nexus-sdk-jd_release`）存在或可克隆

### 工作流程

```
ci_release.sh
  │
  ├─ 1. 确保 Docker 编译镜像就绪
  │      ├─ nexus-sdk-env:x86_64  ← 兼容旧 tag nexus-sdk-env:latest
  │      └─ nexus-sdk-env:aarch64 ← 兼容旧 tag nexus-sdk-env:arm
  │
  ├─ 2. Docker 内编译 x86_64
  │      └─ docker run --platform linux/amd64 ... make_release.sh --arch x86_64
  │      → release/ 存为临时备份
  │
  ├─ 3. 清理 build/ install/，Docker 内编译 aarch64
  │      └─ docker run --platform linux/arm64 ... make_release.sh --arch aarch64
  │      → release/ 含 aarch64 产物
  │
  ├─ 4. 合并双架构
  │      └─ 将 x86_64 的 lib/x86_64/ 合并回 release/ 各包目录
  │      → 每个包同时包含 lib/x86_64/ 和 lib/aarch64/
  │
  ├─ 5. (可选) docker save 导出环境镜像
  │      → release/docker/{x86_64,aarch64}/
  │
  ├─ 6. 同步到客户发布仓库
  │      ├─ clone（首次）或 git pull（后续）
  │      ├─ rsync: 架构无关文件全量覆盖，lib/ 按架构增量合并
  │      └─ tool/ → tool/，ROS 包 → src/<pkg>/
  │
  └─ 7. git commit + git push origin main
```

### 多架构产物结构

执行一次后，客户仓库内每个包同时包含两个平台的库：

```
nexus-sdk-jd_release
├── tool/
├── docker/                      ← --with-docker 时填充
│   ├── x86_64/
│   │   └── nexus-sdk-env-x86_64.docker.tar.gz
│   └── aarch64/
│       └── nexus-sdk-env-aarch64.docker.tar.gz
└── src/
    ├── infra_msg/
    ├── teleop_adapter/
    │   └── lib/
    │       ├── x86_64/          ← Docker linux/amd64 容器编译
    │       └── aarch64/         ← Docker linux/arm64 容器编译（QEMU）
    ├── robot_controller/
    ├── nexus_manage/
    └── human_data/
```

CMake 在客户侧编译时自动根据 `CMAKE_SYSTEM_PROCESSOR` 选择对应架构。

### 与 publish_release.sh 的对比

| | `ci_release.sh` | `publish_release.sh` |
|------|------|------|
| 目标仓库 | 独立仓库 `nexus-sdk-jd_release` | 同仓库 `jd_release` 分支 |
| 编译方式 | Docker 容器内 | 宿主机原生 |
| 架构支持 | 一次执行双架构 | 单架构，需两台机器各跑一次 |
| Docker 镜像 | 可选导出 | 可选导出 |

---

## check_public_headers.sh — 公开头文件卫生检查

检查公开头文件是否泄露过多实现细节，可用作 CI 门禁或开发自检。

### 用法

```bash
# 检查所有发布包
./scripts/check_public_headers.sh

# 只检查指定包
./scripts/check_public_headers.sh teleop_adapter robot_controller
```

### 检查规则

| 规则 | 说明 | 严重程度 |
|------|------|---------|
| 厂商 SDK 头文件 | 检测 `#include` 是否引用了 `rokae/`、`y1_sdk/`、`unitree/` 等厂商头文件 | WARNING |
| 过多私有成员 | 检测 `private:`/`protected:` 段中是否有超过 8 个成员变量且未使用 PIMPL | WARNING |

### 输出示例

```
=== Public Header Check ===

Package: teleop_adapter

Package: robot_controller

Package: nexus_manage

Package: human_data

=== Summary ===
Warnings: 0
All public headers look clean.
```

脚本退出码：`0` 表示全部通过，`1` 表示存在警告。

### CI 集成

在 CI 流水线中添加：

```yaml
- name: Check public headers
  run: |
    ./scripts/check_public_headers.sh
    # 退出码非0时 CI 失败
```

---

## 辅助脚本（可忽略）

| 文件 | 说明 |
|------|------|
| `apply_arm_controller_pimpl.py` | PIMPL 重构辅助工具（一次性使用） |
| `refactor_arm_controller_pimpl.py` | PIMPL 重构辅助工具（一次性使用） |
