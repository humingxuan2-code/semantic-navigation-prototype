#!/usr/bin/env bash
set -euo pipefail

ROOT="$HOME/semantic_nav_ws"
TOOLS_DIR="$ROOT/tools"
OUTPUTS_DIR="$ROOT/outputs"

CONTROLLER="$TOOLS_DIR/drive_blue_goto_trace"
PLOTTER="$TOOLS_DIR/plot_trajectory.py"
LATEST_CSV="$TOOLS_DIR/trajectory_latest.csv"
LATEST_PNG="$TOOLS_DIR/trajectory_latest.png"

RUN_DIR="$OUTPUTS_DIR/benchmark_rel_v1"

if [[ ! -x "$CONTROLLER" ]]; then
  echo "找不到可执行控制器：$CONTROLLER"
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
  echo "发现已有 benchmark_rel_v1，自动备份为："
  echo "$BACKUP_DIR"
  mv "$RUN_DIR" "$BACKUP_DIR"
fi

mkdir -p "$RUN_DIR"

MANIFEST="$RUN_DIR/manifest.csv"

echo "case_name,forward_m,left_m" > "$MANIFEST"

run_case() {
  local case_name="$1"
  local forward_m="$2"
  local left_m="$3"
  local case_dir="$RUN_DIR/$case_name"

  mkdir -p "$case_dir"

  echo "$case_name,$forward_m,$left_m" >> "$MANIFEST"

  echo
  echo "=================================================="
  echo "开始测试：$case_name"
  echo "相对目标：前方 ${forward_m} m，左侧 ${left_m} m"
  echo "=================================================="

  "$CONTROLLER" rel "$forward_m" "$left_m" 2>&1 \
    | tee "$case_dir/terminal_output.txt"

  cp "$LATEST_CSV" "$case_dir/trajectory.csv"

  python3 "$PLOTTER" \
    > "$case_dir/plot_output.txt"

  cp "$LATEST_PNG" "$case_dir/trajectory.png"

  echo
  echo "[$case_name] 已保存："
  echo "  $case_dir/terminal_output.txt"
  echo "  $case_dir/trajectory.csv"
  echo "  $case_dir/trajectory.png"

  sleep 1
}

run_case "case01_straight_030" 0.30 0.00
run_case "case02_left_030_020" 0.30 0.20
run_case "case03_right_030_020" 0.30 -0.20
run_case "case04_straight_050" 0.50 0.00
run_case "case05_left_040_030" 0.40 0.30
run_case "case06_right_040_030" 0.40 -0.30

echo
echo "=================================================="
echo "全部 6 组导航实验完成。"
echo "结果目录：$RUN_DIR"
echo "=================================================="

find "$RUN_DIR" -maxdepth 2 -type f | sort
