"""从臂注册表工具 — 供 cross_subnet launch 读取 slave_registry.yaml。"""

from __future__ import annotations

import socket
from pathlib import Path
from typing import Any

import yaml


def _local_ipv4_addresses() -> set[str]:
    addrs: set[str] = set()
    try:
        hostname = socket.gethostname()
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            addrs.add(info[4][0])
    except OSError:
        pass
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(('8.8.8.8', 80))
            addrs.add(sock.getsockname()[0])
    except OSError:
        pass
    addrs.discard('127.0.0.1')
    return addrs


def load_registry(registry_path: str | Path) -> list[dict[str, Any]]:
    path = Path(registry_path)
    if not path.is_file():
        return []
    with path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    slaves = data.get('slaves', [])
    return slaves if isinstance(slaves, list) else []


def find_slave_by_ip(slaves: list[dict[str, Any]]) -> dict[str, Any] | None:
    if not slaves:
        return None
    local_ips = _local_ipv4_addresses()
    for slave in slaves:
        candidates: list[str] = []
        for key in ('local_ips', 'ips'):
            value = slave.get(key)
            if isinstance(value, list):
                candidates.extend(str(item) for item in value)
        for key in ('ip', 'external_ip'):
            if slave.get(key):
                candidates.append(str(slave[key]))
        if local_ips.intersection(candidates):
            return slave
    return None


def generate_peers_xml(
    slaves: list[dict[str, Any]],
    *,
    local_count: int,
    remote_count: int,
    port_base: int,
) -> str:
    lines: list[str] = []
    for index in range(local_count):
        lines.append(f'        <Peer Address="127.0.0.1:{port_base + index}"/>')
    for slave in slaves:
        peer_ip = slave.get('ip') or slave.get('external_ip')
        if not peer_ip:
            continue
        for index in range(remote_count):
            lines.append(f'        <Peer Address="{peer_ip}:{port_base + index}"/>')
    return '\n'.join(lines)
