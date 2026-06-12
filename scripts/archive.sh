#!/bin/bash
#
# nexus-sdk 项目打包脚本
# =======================
#
# 用法:
#   ./scripts/archive.sh
#
# 打包后会生成 nexus-sdk_YYYYMMDD_HHMMSS.tar.gz 在上一级目录。
#
# 排除内容:
#   .git/        版本库
#   build/       colcon 编译产物
#   install/     colcon 安装产物
#   log/ logs/   编译和运行日志
#   __pycache__/ *.pyc  Python 字节码缓存
#
# 解压方法:
#   # 保留所有文件属性（含 owner/group，需 root 权限）
#   tar -xpzf nexus-sdk_YYYYMMDD_HHMMSS.tar.gz
#
#   # 保留权限和时间戳，但去掉 owner/group 信息（推荐普通用户使用）
#   tar -xpzf nexus-sdk_YYYYMMDD_HHMMSS.tar.gz --no-same-owner
#
#   -p  保留文件原来的权限位和时间戳
#   -x  解压
#   -z  gzip 解压
#   -f  指定文件
#
# 解压后生成 nexus-sdk/ 目录，所有内容都在该目录下。

set -e

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
PROJECT_NAME="$(basename "$PROJECT_DIR")"
PARENT_DIR="$(dirname "$PROJECT_DIR")"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_FILE="${PARENT_DIR}/${PROJECT_NAME}_${TIMESTAMP}.tar.gz"

echo "Project: $PROJECT_DIR"
echo "Output:  $OUTPUT_FILE"
echo

cd "$PARENT_DIR"

tar -czf "$OUTPUT_FILE" \
    --exclude='.git' \
    --exclude='build' \
    --exclude='install' \
    --exclude='log' \
    --exclude='logs' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    "$PROJECT_NAME"

echo "Done: $OUTPUT_FILE ($(du -h "$OUTPUT_FILE" | cut -f1))"
echo
echo "To extract (keep file attributes):"
echo "  tar -xpzf $(basename "$OUTPUT_FILE")"
echo
echo "To extract (no root, keep attributes for files you own):"
echo "  tar -xpzf $(basename "$OUTPUT_FILE") --no-same-owner"
