import csv
import math
from collections import OrderedDict
from pathlib import Path

import matplotlib.pyplot as plt


ROOT_DIR = Path.home() / "semantic_nav_ws"
RUN_DIR = ROOT_DIR / "outputs" / "exp012_global_rectangle_v1"
TRAJECTORY_PATH = RUN_DIR / "global_route_trajectory.csv"
SUMMARY_PATH = RUN_DIR / "global_route_summary.csv"
OUTPUT_DIR = RUN_DIR / "summary"


def read_csv(path):
    with path.open("r", encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise RuntimeError(f"文件没有数据：{path}")

    return rows


def path_length(rows):
    total = 0.0

    for index in range(1, len(rows)):
        x1 = float(rows[index - 1]["x"])
        y1 = float(rows[index - 1]["y"])
        x2 = float(rows[index]["x"])
        y2 = float(rows[index]["y"])

        total += math.hypot(x2 - x1, y2 - y1)

    return total


def main():
    if not TRAJECTORY_PATH.exists():
        raise FileNotFoundError(f"找不到轨迹文件：{TRAJECTORY_PATH}")

    if not SUMMARY_PATH.exists():
        raise FileNotFoundError(f"找不到汇总文件：{SUMMARY_PATH}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    trajectory_rows = read_csv(TRAJECTORY_PATH)
    summary_rows = read_csv(SUMMARY_PATH)

    waypoint_paths = OrderedDict()

    for row in trajectory_rows:
        waypoint_name = row["waypoint_name"]

        if waypoint_name not in waypoint_paths:
            waypoint_paths[waypoint_name] = []

        waypoint_paths[waypoint_name].append(row)

    route_plot_path = OUTPUT_DIR / "global_route_overview.png"
    error_plot_path = OUTPUT_DIR / "global_route_error_comparison.png"
    metrics_path = OUTPUT_DIR / "global_route_metrics.txt"

    plt.figure(figsize=(8, 7))

    for waypoint_name, rows in waypoint_paths.items():
        x_values = [float(row["x"]) for row in rows]
        y_values = [float(row["y"]) for row in rows]

        target_x = float(rows[0]["target_x"])
        target_y = float(rows[0]["target_y"])

        end_x = float(rows[-1]["x"])
        end_y = float(rows[-1]["y"])

        plt.plot(
            x_values,
            y_values,
            linewidth=2,
            label=waypoint_name,
        )

        plt.scatter(
            [target_x],
            [target_y],
            marker="*",
            s=160,
        )

        plt.scatter(
            [end_x],
            [end_y],
            marker="X",
            s=70,
        )

        plt.annotate(
            waypoint_name,
            (target_x, target_y),
            textcoords="offset points",
            xytext=(6, 6),
        )

    first_row = trajectory_rows[0]

    plt.scatter(
        [float(first_row["x"])],
        [float(first_row["y"])],
        marker="o",
        s=100,
        label="Route start",
    )

    plt.xlabel("World X (m)")
    plt.ylabel("World Y (m)")
    plt.title("EXP-012: Global Coordinate Waypoint Route")
    plt.grid(True)
    plt.axis("equal")
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(route_plot_path, dpi=180)
    plt.close()

    labels = [row["waypoint_name"] for row in summary_rows]
    errors_mm = [float(row["final_error_m"]) * 1000.0 for row in summary_rows]

    plt.figure(figsize=(9, 5))
    positions = list(range(len(labels)))

    plt.bar(positions, errors_mm)
    plt.xticks(positions, labels, rotation=20, ha="right")
    plt.xlabel("Global waypoint")
    plt.ylabel("Final goal error (mm)")
    plt.title("EXP-012: Global Route Waypoint Error")
    plt.grid(True, axis="y")
    plt.tight_layout()
    plt.savefig(error_plot_path, dpi=180)
    plt.close()

    total_path_length = sum(
        path_length(rows)
        for rows in waypoint_paths.values()
    )

    total_duration = sum(
        float(row["duration_sec"])
        for row in summary_rows
    )

    average_error_mm = sum(errors_mm) / len(errors_mm)
    maximum_error_mm = max(errors_mm)

    with metrics_path.open("w", encoding="utf-8") as file:
        file.write("EXP-012 Global Route Metrics\n")
        file.write("============================\n")
        file.write(f"Waypoint count: {len(summary_rows)}\n")
        file.write(f"Success count: {sum(row['status'] == 'success' for row in summary_rows)}\n")
        file.write(f"Average final error: {average_error_mm:.3f} mm\n")
        file.write(f"Maximum final error: {maximum_error_mm:.3f} mm\n")
        file.write(f"Total route path length: {total_path_length:.3f} m\n")
        file.write(f"Total execution time: {total_duration:.2f} s\n")

    print("EXP-012 可视化与指标汇总完成。")
    print()
    print(f"平均最终误差：{average_error_mm:.3f} mm")
    print(f"最大最终误差：{maximum_error_mm:.3f} mm")
    print(f"总路线长度：{total_path_length:.3f} m")
    print(f"总执行时间：{total_duration:.2f} s")
    print()
    print("生成文件：")
    print(route_plot_path)
    print(error_plot_path)
    print(metrics_path)


if __name__ == "__main__":
    main()
