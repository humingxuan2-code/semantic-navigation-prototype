import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


ROOT_DIR = Path.home() / "semantic_nav_ws"
RUN_DIR = ROOT_DIR / "outputs" / "exp011_waypoint_route_v1"
MANIFEST_PATH = RUN_DIR / "manifest.csv"
SUMMARY_DIR = RUN_DIR / "summary"


def get_float(row, key):
    return float(row[key])


def read_rows(csv_path):
    with csv_path.open("r", encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise RuntimeError(f"轨迹文件没有数据：{csv_path}")

    return rows


def calculate_path_length(rows):
    length = 0.0

    for index in range(1, len(rows)):
        x1 = get_float(rows[index - 1], "x")
        y1 = get_float(rows[index - 1], "y")
        x2 = get_float(rows[index], "x")
        y2 = get_float(rows[index], "y")

        length += math.hypot(x2 - x1, y2 - y1)

    return length


def summarize_waypoint(waypoint_name, forward_m, left_m):
    csv_path = RUN_DIR / waypoint_name / "trajectory.csv"
    rows = read_rows(csv_path)

    first_row = rows[0]
    last_row = rows[-1]

    start_x = get_float(first_row, "x")
    start_y = get_float(first_row, "y")

    target_x = get_float(first_row, "target_x")
    target_y = get_float(first_row, "target_y")

    end_x = get_float(last_row, "x")
    end_y = get_float(last_row, "y")

    initial_distance = math.hypot(
        target_x - start_x,
        target_y - start_y,
    )

    final_error = math.hypot(
        target_x - end_x,
        target_y - end_y,
    )

    return {
        "waypoint_name": waypoint_name,
        "forward_m": forward_m,
        "left_m": left_m,
        "start_x": start_x,
        "start_y": start_y,
        "target_x": target_x,
        "target_y": target_y,
        "end_x": end_x,
        "end_y": end_y,
        "initial_distance_m": initial_distance,
        "final_error_m": final_error,
        "path_length_m": calculate_path_length(rows),
        "duration_sec": get_float(last_row, "time_sec"),
        "samples": len(rows),
        "rows": rows,
    }


def save_summary_csv(results):
    output_path = SUMMARY_DIR / "waypoint_summary.csv"

    fieldnames = [
        "waypoint_name",
        "forward_m",
        "left_m",
        "start_x",
        "start_y",
        "target_x",
        "target_y",
        "end_x",
        "end_y",
        "initial_distance_m",
        "final_error_m",
        "path_length_m",
        "duration_sec",
        "samples",
    ]

    with output_path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()

        for result in results:
            writer.writerow({
                field: result[field]
                for field in fieldnames
            })

    return output_path


def plot_error_comparison(results):
    output_path = SUMMARY_DIR / "waypoint_error_comparison.png"

    labels = [
        result["waypoint_name"]
        for result in results
    ]

    errors_mm = [
        result["final_error_m"] * 1000.0
        for result in results
    ]

    positions = list(range(len(labels)))

    plt.figure(figsize=(9, 5))
    plt.bar(positions, errors_mm)
    plt.xticks(positions, labels, rotation=20, ha="right")
    plt.xlabel("Waypoint")
    plt.ylabel("Final goal error (mm)")
    plt.title("EXP-011: Waypoint Navigation Final Error")
    plt.grid(True, axis="y")
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close()

    return output_path


def plot_route_overview(results):
    output_path = SUMMARY_DIR / "waypoint_route_overview.png"

    plt.figure(figsize=(8, 7))

    for result in results:
        x_values = [
            get_float(row, "x")
            for row in result["rows"]
        ]

        y_values = [
            get_float(row, "y")
            for row in result["rows"]
        ]

        plt.plot(
            x_values,
            y_values,
            linewidth=2,
            label=result["waypoint_name"],
        )

        plt.scatter(
            [result["target_x"]],
            [result["target_y"]],
            marker="*",
            s=160,
        )

        plt.scatter(
            [result["end_x"]],
            [result["end_y"]],
            marker="X",
            s=70,
        )

        plt.annotate(
            result["waypoint_name"],
            (result["target_x"], result["target_y"]),
            textcoords="offset points",
            xytext=(6, 6),
        )

    first_result = results[0]

    plt.scatter(
        [first_result["start_x"]],
        [first_result["start_y"]],
        marker="o",
        s=100,
        label="Route start",
    )

    plt.xlabel("World X (m)")
    plt.ylabel("World Y (m)")
    plt.title("EXP-011: Multi-Waypoint Route Overview")
    plt.grid(True)
    plt.axis("equal")
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close()

    return output_path


def main():
    if not MANIFEST_PATH.exists():
        raise FileNotFoundError(f"找不到 manifest：{MANIFEST_PATH}")

    SUMMARY_DIR.mkdir(parents=True, exist_ok=True)

    results = []

    with MANIFEST_PATH.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            results.append(
                summarize_waypoint(
                    row["waypoint_name"],
                    float(row["forward_m"]),
                    float(row["left_m"]),
                )
            )

    summary_csv = save_summary_csv(results)
    error_plot = plot_error_comparison(results)
    route_plot = plot_route_overview(results)

    print("EXP-011 多航点路线汇总完成。")
    print()

    for result in results:
        print(
            f"{result['waypoint_name']}: "
            f"误差={result['final_error_m']:.4f} m, "
            f"路径长度={result['path_length_m']:.4f} m, "
            f"耗时={result['duration_sec']:.2f} s, "
            f"采样点={result['samples']}"
        )

    average_error = sum(
        result["final_error_m"]
        for result in results
    ) / len(results)

    total_path_length = sum(
        result["path_length_m"]
        for result in results
    )

    total_duration = sum(
        result["duration_sec"]
        for result in results
    )

    print()
    print(f"平均航点误差：{average_error:.4f} m")
    print(f"总路径长度：{total_path_length:.4f} m")
    print(f"总执行时间：{total_duration:.2f} s")
    print()
    print("生成文件：")
    print(summary_csv)
    print(error_plot)
    print(route_plot)


if __name__ == "__main__":
    main()
