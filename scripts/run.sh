#!/usr/bin/env bash
# ============================================================
# run.sh — 启动无人机上位机控制框架
#
# 用法：
#   ./scripts/run.sh                        # 默认连接真机
#   ./scripts/run.sh --dry-run              # 使用 FakeVehicle
#   ./scripts/run.sh --mission takeoff_and_hover
#   ./scripts/run.sh --dry-run --enable-stream --mission takeoff_and_hover
# ============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

# ── 激活虚拟环境 ──────────────────────────────
VENV_DIR="${PROJECT_DIR}/drone_env"
if [ ! -d "$VENV_DIR" ]; then
    # 向上查找（兼容 venv 在 home 目录的情况）
    VENV_DIR="$HOME/drone_env"
fi

if [ -f "$VENV_DIR/bin/activate" ]; then
    echo "激活虚拟环境: $VENV_DIR"
    source "$VENV_DIR/bin/activate"
else
    echo "警告: 未找到虚拟环境 ($VENV_DIR)，使用系统 Python"
fi

# ── 进入项目目录 ──────────────────────────────
cd "$PROJECT_DIR"

# ── 启动 ──────────────────────────────────────
echo "启动无人机控制框架…"
python -m src.main \
    --config config/vehicle.yaml \
    "$@"
