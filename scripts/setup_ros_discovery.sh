#!/usr/bin/env bash
# shellcheck disable=SC2148
# -*- mode: shell-script -*-
# =============================================================================
# setup_ros_discovery.sh — 让当前终端能发现 launch 启动的 ROS2 节点
#
# 用法（必须 source，不能直接执行）:
#   source scripts/setup_ros_discovery.sh same-subnet [interface] [domain_id]
#   source scripts/setup_ros_discovery.sh cross-subnet master|slave [domain_id]
#   source scripts/setup_ros_discovery.sh reuse [domain_id]
#
# 示例:
#   source scripts/setup_ros_discovery.sh same-subnet enp2s0 18
#   source scripts/setup_ros_discovery.sh cross-subnet master
#   source scripts/setup_ros_discovery.sh cross-subnet slave 18
#   source scripts/setup_ros_discovery.sh reuse          # 复用 launch 文件生成的 XML
#
# ═══════════════════════════════════════════════════════════════════════════════
# 部署参数（与 launch 文件中的值保持一致，部署时修改此处）
# ═══════════════════════════════════════════════════════════════════════════════

# — 同子网（nexus_arm_v15_right_to_ar5_08l_suction_cup_real_system.launch.py）—
SAME_SUBNET_DEFAULT_DOMAIN_ID=18
SAME_SUBNET_DEFAULT_INTERFACE="enp2s0"
# 硬编码在 launch 模板中的 Peer 列表，空格分隔
SAME_SUBNET_PEERS=("10.18.16.30")

# — 跨子网（nexus_arm_v15_right_to_ar5_08l_suction_cup_real_system_cross_subnet.launch.py）—
CROSS_SUBNET_DEFAULT_DOMAIN_ID=18
CROSS_SUBNET_PORT_BASE=7000
CROSS_SUBNET_MAX_PARTICIPANT_INDEX=50

# Master 侧
CROSS_SUBNET_MASTER_IFACE="wlp3s0"
CROSS_SUBNET_MASTER_IP="10.18.20.42"
CROSS_SUBNET_MASTER_NODE_COUNT=4

# Slave 侧
CROSS_SUBNET_SLAVE_IFACE="enp2s0"
CROSS_SUBNET_SLAVE_IP="10.18.16.30"
CROSS_SUBNET_SLAVE_NODE_COUNT=3

# ═══════════════════════════════════════════════════════════════════════════════

_usage() {
    echo "Usage:"
    echo "  source setup_ros_discovery.sh same-subnet [interface] [domain_id]"
    echo "  source setup_ros_discovery.sh cross-subnet master|slave [domain_id]"
    echo "  source setup_ros_discovery.sh reuse [domain_id]"
}

_ros_ws_root() {
    # 从脚本所在位置推导 workspace 根目录
    local script_dir
    script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    dirname "$script_dir"
}

_gen_same_subnet_xml() {
    local iface="$1"
    local peers_xml=""
    for peer in "${SAME_SUBNET_PEERS[@]}"; do
        peers_xml+="        <Peer Address=\"${peer}\"/>"$'\n'
    done

    cat <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config
              https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="${iface}" multicast="false"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>50</MaxAutoParticipantIndex>
      <Peers>
${peers_xml}      </Peers>
    </Discovery>
    <Internal>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
      <OutputFile>stderr</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF
}

_make_cross_peers_xml() {
    local local_count="$1"
    local remote_count="$2"
    local peer_ip="$3"
    local local_ip="${4:-127.0.0.1}"
    local lines=""
    for ((i = 0; i < local_count; i++)); do
        lines+="        <Peer Address=\"${local_ip}:$((CROSS_SUBNET_PORT_BASE + i))\"/>"$'\n'
    done
    for ((i = 0; i < remote_count; i++)); do
        lines+="        <Peer Address=\"${peer_ip}:$((CROSS_SUBNET_PORT_BASE + i))\"/>"$'\n'
    done
    echo -n "$lines"
}

_gen_cross_subnet_xml() {
    local iface="$1" ext_ip="$2" local_count="$3" peer_ip="$4" remote_count="$5"
    local local_ip="${6:-127.0.0.1}"

    local ext_addr_xml=""
    if [[ -n "$ext_ip" ]]; then
        ext_addr_xml="      <ExternalNetworkAddress>${ext_ip}</ExternalNetworkAddress>"
    fi

    local peers_xml
    peers_xml="$(_make_cross_peers_xml "$local_count" "$remote_count" "$peer_ip" "$local_ip")"

    cat <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config"
            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config
              https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="${iface}" multicast="false"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
${ext_addr_xml}
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>${CROSS_SUBNET_MAX_PARTICIPANT_INDEX}</MaxAutoParticipantIndex>
      <Peers>
${peers_xml}      </Peers>
      <Ports>
        <Base>${CROSS_SUBNET_PORT_BASE}</Base>
        <DomainGain>0</DomainGain>
        <ParticipantGain>1</ParticipantGain>
        <UnicastMetaOffset>0</UnicastMetaOffset>
        <UnicastDataOffset>0</UnicastDataOffset>
      </Ports>
    </Discovery>
    <Internal>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
    <Tracing>
      <Verbosity>warning</Verbosity>
      <OutputFile>stderr</OutputFile>
    </Tracing>
  </Domain>
</CycloneDDS>
EOF
}

# ═══════════════════════════════════════════════════════════════════════════════
# 入口
# ═══════════════════════════════════════════════════════════════════════════════

# 查找 launch 文件生成的最新 CycloneDDS XML（排除本脚本自己生成的）
_find_launch_xml() {
    local pattern="/tmp/cyclonedds_nexus_cross_subnet_*.xml"
    local latest=""
    for f in $pattern; do
        # 跳过本脚本生成的 CLI XML
        [[ "$f" == *cli* ]] && continue
        [[ -f "$f" ]] || continue
        if [[ -z "$latest" ]] || [[ "$f" -nt "$latest" ]]; then
            latest="$f"
        fi
    done
    echo "$latest"
}

main() {
    if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
        echo "错误: 此脚本必须用 'source' 加载，不能直接执行。" >&2
        echo "  source ${0}" >&2
        return 1
    fi

    local mode="${1:-}"
    local arg2="${2:-}"
    local domain_id_arg="${3:-}"

    local domain_id iface peers_xml xml_path

    case "$mode" in
    same-subnet)
        iface="${arg2:-$SAME_SUBNET_DEFAULT_INTERFACE}"
        domain_id="${domain_id_arg:-$SAME_SUBNET_DEFAULT_DOMAIN_ID}"
        xml_path="/tmp/cyclonedds_nexus_cli_same_subnet.xml"
        _gen_same_subnet_xml "$iface" > "$xml_path"
        echo "[setup_ros_discovery] same-subnet mode: iface=${iface}, domain=${domain_id}" >&2
        ;;

    cross-subnet)
        local role="$arg2"
        if [[ "$role" != "master" && "$role" != "slave" ]]; then
            echo "错误: cross-subnet 需要第二个参数为 master 或 slave" >&2
            _usage >&2
            return 1
        fi
        domain_id="${domain_id_arg:-$CROSS_SUBNET_DEFAULT_DOMAIN_ID}"

        local local_count remote_count peer_ip
        if [[ "$role" == "master" ]]; then
            iface="$CROSS_SUBNET_MASTER_IFACE"
            local ext_ip="$CROSS_SUBNET_MASTER_IP"
            local_count="$CROSS_SUBNET_MASTER_NODE_COUNT"
            peer_ip="$CROSS_SUBNET_SLAVE_IP"
            remote_count="$CROSS_SUBNET_SLAVE_NODE_COUNT"
        else
            iface="$CROSS_SUBNET_SLAVE_IFACE"
            local ext_ip="$CROSS_SUBNET_SLAVE_IP"
            local_count="$CROSS_SUBNET_SLAVE_NODE_COUNT"
            peer_ip="$CROSS_SUBNET_MASTER_IP"
            remote_count="$CROSS_SUBNET_MASTER_NODE_COUNT"
        fi
        xml_path="/tmp/cyclonedds_nexus_cli_cross_subnet.xml"
        _gen_cross_subnet_xml "$iface" "$ext_ip" "$local_count" "$peer_ip" "$remote_count" "$ext_ip" > "$xml_path"
        echo "[setup_ros_discovery] cross-subnet mode: role=${role}, iface=${iface}, domain=${domain_id}" >&2
        ;;

    reuse)
        domain_id="${arg2:-$CROSS_SUBNET_DEFAULT_DOMAIN_ID}"
        xml_path="$(_find_launch_xml)"
        if [[ -z "$xml_path" ]]; then
            echo "错误: 未找到 launch 文件生成的 CycloneDDS XML" >&2
            echo "  请先启动 launch 文件，再执行此命令" >&2
            return 1
        fi
        echo "[setup_ros_discovery] reuse mode: 复用 launch 文件生成的 XML" >&2
        echo "[setup_ros_discovery]   XML 路径: ${xml_path}" >&2
        ;;

    *)
        echo "错误: 未知模式 '${mode}'" >&2
        _usage >&2
        return 1
        ;;
    esac

    # 设置环境变量
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export ROS_DOMAIN_ID="$domain_id"
    export CYCLONEDDS_URI="file://${xml_path}"

    # 清除 ROS2 daemon 缓存
    ros2 daemon stop 2>/dev/null || true

    echo "[setup_ros_discovery] 环境已就绪" >&2
    echo "[setup_ros_discovery]   RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}" >&2
    echo "[setup_ros_discovery]   ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" >&2
    echo "[setup_ros_discovery]   CYCLONEDDS_URI=${CYCLONEDDS_URI}" >&2
    echo "[setup_ros_discovery]   XML 文件已写入: ${xml_path}" >&2
}

main "$@"
