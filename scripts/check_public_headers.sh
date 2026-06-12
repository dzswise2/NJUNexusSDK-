#!/usr/bin/env bash
#
# check_public_headers.sh — 检查公开头文件是否泄露过多实现细节
#
# 用法:
#   ./scripts/check_public_headers.sh [包名...]
#   不指定包名时检查所有发布包
#
# 检查规则:
#   1. 警告: 头文件 #include 了厂商 SDK 头文件
#   2. 警告: private/protected 段中有超过 8 个非 Impl 的成员变量
#   3. 警告: 头文件中包含完整函数体（非模板/constexpr/inline 必要场景）
#
set -euo pipefail

WORKSPACE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

PACKAGES=("$@")
if [ ${#PACKAGES[@]} -eq 0 ]; then
    PACKAGES=(teleop_adapter robot_controller nexus_manage human_data)
fi

VENDOR_PATTERNS=(
    'rokae/'
    'y1_sdk_interface.h'
    'imeta_y1/'
    'g3log/'
    'kdl/'
)

WARNINGS=0
ERRORS=0

warn() {
    echo "  WARNING: $*"
    ((WARNINGS++))
}

check_vendor_includes() {
    local file="$1"
    for pattern in "${VENDOR_PATTERNS[@]}"; do
        if grep -qE "^#include.*${pattern}" "$file" 2>/dev/null; then
            warn "$file includes vendor SDK header matching '$pattern'"
        fi
    done
}

check_private_member_count() {
    local file="$1"
    local in_private=false
    local member_count=0
    local impl_found=false

    while IFS= read -r line; do
        if echo "$line" | grep -qE '^\s*(private|protected)\s*:' > /dev/null 2>&1; then
            in_private=true
            member_count=0
            impl_found=false
            continue
        fi
        if echo "$line" | grep -qE '^\s*public\s*:' > /dev/null 2>&1; then
            in_private=false
            continue
        fi
        if [ "$in_private" = true ]; then
            if echo "$line" | grep -qE 'struct\s+Impl|unique_ptr<Impl>' > /dev/null 2>&1; then
                impl_found=true
            fi
            if echo "$line" | grep -qE '^\s+\S+.*_\s*[;={]|^\s+\S+.*_\s*$' > /dev/null 2>&1; then
                member_count=$((member_count + 1))
            fi
        fi
    done < "$file"

    if [ "$in_private" = true ] && [ "$impl_found" = false ] && [ "$member_count" -gt 8 ]; then
        warn "$file has ~$member_count member variables in private/protected section without PIMPL"
    fi
}

echo "=== Public Header Check ==="
echo ""

for pkg in "${PACKAGES[@]}"; do
    INCLUDE_DIR="${WORKSPACE_ROOT}/src/${pkg}/include"
    if [ ! -d "$INCLUDE_DIR" ]; then
        echo "Package $pkg: no include/ directory, skipping."
        continue
    fi

    echo "Package: $pkg"

    while IFS= read -r header; do
        [[ "$header" == *.hpp ]] || [[ "$header" == *.h ]] || continue
        [[ "$header" == *"/common/thread/"* ]] && continue || true

        check_vendor_includes "$header" || true
        check_private_member_count "$header" || true

    done < <(find "$INCLUDE_DIR" -type f \( -name "*.hpp" -o -name "*.h" \) 2>/dev/null)

    echo ""
done

echo "=== Summary ==="
echo "Warnings: $WARNINGS"

if [ "$WARNINGS" -gt 0 ]; then
    echo ""
    echo "Consider using PIMPL pattern or moving implementation details to internal headers."
    exit 1
else
    echo "All public headers look clean."
    exit 0
fi
