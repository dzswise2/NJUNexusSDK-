#!/usr/bin/env bash
# =============================================================================
# kill_ros_processes.sh — 一键清理所有 ROS 2 相关后台进程
#
# 用法:
#   ./scripts/kill_ros_processes.sh              # 杀死进程
#   ./scripts/kill_ros_processes.sh --dry-run    # 仅预览，不实际杀进程
# =============================================================================
set -e

DRY_RUN=false
if [[ "${1:-}" == "--dry-run" ]]; then
    DRY_RUN=true
    echo ">>> DRY RUN 模式，不会实际杀死进程 <<<"
    echo
fi

ME=$(whoami)

# 按进程模式 + 所属用户精确杀死
# 自动判断：root 进程用 sudo kill，普通用户进程直接 kill
_cleanup() {
    local pattern="$1"
    local desc="$2"
    local pids
    pids=$(pgrep -f "$pattern" 2>/dev/null || true)
    if [[ -z "$pids" ]]; then
        return
    fi

    local killed_any=false
    for pid in $pids; do
        local owner
        owner=$(ps -o user= -p "$pid" 2>/dev/null | tr -d ' ')
        if [[ -z "$owner" ]]; then
            continue
        fi

        if ! $killed_any; then
            echo "[$desc]"
            killed_any=true
        fi

        local cmd
        cmd=$(ps -o cmd= -p "$pid" 2>/dev/null | head -c 120)

        if $DRY_RUN; then
            if [[ "$owner" == "root" ]]; then
                echo "  [sudo] $pid $owner $cmd"
            else
                echo "         $pid $owner $cmd"
            fi
        else
            if [[ "$owner" == "root" ]]; then
                sudo kill -9 "$pid" 2>/dev/null && echo "  [sudo] 已杀死 $pid $owner" || echo "  [sudo] 失败 $pid $owner"
            elif [[ "$owner" == "$ME" ]]; then
                kill -9 "$pid" 2>/dev/null && echo "  已杀死 $pid $owner" || echo "  失败 $pid $owner"
            else
                echo "  跳过 $pid (owner=$owner, not $ME or root)"
            fi
        fi
    done
}

echo "========================================"
echo "  ROS 2 进程清理"
echo "========================================"

# —— nexus-sdk / 旧工作空间 ——
_cleanup "ros2 launch.*launch\.py"           "ROS 2 launch 进程"
_cleanup "teleop_manager_node"              "teleop_manager_node"
_cleanup "arm_control_node"                 "arm_control_node"
_cleanup "human_data_solver_node"           "human_data_solver_node"
_cleanup "simulator.*mujoco"                "mujoco_sim"

# —— 残留测试进程 ——
_cleanup "dds_multi_node_test"              "残留: DDS 测试"
_cleanup "action_examples"                  "残留: action_examples"
_cleanup "ros2_learning"                    "残留: ros2_learning"

# —— ROS 2 daemon ——
if ! $DRY_RUN; then
    echo "[ROS 2 daemon]"
    ros2 daemon stop 2>/dev/null && echo "  daemon 已停止" || echo "  daemon 未运行"
fi

# —— 清理临时 XML ——
if ! $DRY_RUN; then
    echo "[临时文件]"
    rm -f /tmp/cyclonedds_nexus_*.xml 2>/dev/null && echo "  已清理 CycloneDDS XML" || true
    rm -f /tmp/launch_params_* 2>/dev/null && true
fi

echo
echo "========================================"
if $DRY_RUN; then
    echo "  DRY RUN 完成（未实际杀进程）"
else
    echo "  清理完成"
fi
echo "========================================"
