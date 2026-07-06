#!/usr/bin/env bash
set -euo pipefail

ROOT="$HOME/semantic_nav_ws"
TOOLS_DIR="$ROOT/tools"
OUTPUTS_DIR="$ROOT/outputs"

CONTROLLER="$TOOLS_DIR/drive_blue_goto_trace"
PLOTTER="$TOOLS_DIR/plot_trajectory.py"
LATEST_CSV="$TOOLS_DIR/trajectory_latest.csv"
LATEST_PNG="$TOOLS_DIR/trajectory_latest.png"

RUN_NAME="exp011_waypoint_route_v1"
RUN_DIR="$OUTPUTS_DIR/$RUN_NAME"
MANIFEST="$RUN_DIR/manifest.csv"

if [[ ! -x "$CONTROLLER" ]]; then
  echo "找不到控制器：$CONTROLLER"
  echo "请先确认 drive_blue_goto_trace 已成功编译。"
  exit 1
fi

if [[ ! -f "$PLOTTER" ]]; then
  echo "找不到绘图脚本：$PLOTTER"
  exit 1
fi

mkdir -p "$OUTPUTS_DIR"

if [[ -d "$RUN_DIR" ]]; then
  BACKUP_DIR="${RUN_DIR}_backup_$(date +%Y%m%d_%H%M%S)"
  echo "发现旧实验目录，自动备份为："
  echo "$BACKUP_DIR"
  mv "$RUN_DIR" "$BACKUP_DIR"
fi

mkdir -p "$RUN_DIR"

echo "waypoint_name,forward_m,left_m" > "$MANIFEST"

run_waypoint() {
  local waypoint_name="$1"
  local forward_m="$2"
  local left_m="$3"
  local waypoint_dir="$RUN_DIR/$waypoint_name"

  mkdir -p "$waypoint_dir"

  echo "$waypoint_name,$forward_m,$left_m" >> "$MANIFEST"

  echo
  echo "=================================================="
  echo "执行航点：$waypoint_name"
  echo "相对目标：前方 ${forward_m} m，左侧 ${left_m} m"
  echo "=================================================="

  "$CONTROLLER" rel "$forward_m" "$left_m" 2>&1 \
    | tee "$waypoint_dir/terminal_output.txt"

  cp "$LATEST_CSV" "$waypoint_dir/trajectory.csv"

  python3 "$PLOTTER" \
    > "$waypoint_dir/plot_output.txt"

  cp "$LATEST_PNG" "$waypoint_dir/trajectory.png"

  echo
  echo "[$waypoint_name] 已保存："
  echo "  $waypoint_dir/trajectory.csv"
  echo "  $waypoint_dir/trajectory.png"

  sleep 1
}

echo "开始执行 EXP-011 多航点路线导航。"
echo "本路线使用已验证的闭环单点导航控制器逐段执行。"

run_waypoint "wp01_forward_035" 0.35 0.00
run_waypoint "wp02_left_035" 0.00 0.35
run_waypoint "wp03_forward_035" 0.35 0.00
run_waypoint "wp04_right_035" 0.00 -0.35

echo
echo "=================================================="
echo "EXP-011 多航点路线执行完成。"
echo "结果目录：$RUN_DIR"
echo "=================================================="

find "$RUN_DIR" -maxdepth 2 -type f | sort
