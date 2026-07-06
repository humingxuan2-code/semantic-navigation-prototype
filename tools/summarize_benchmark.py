import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt


ROOT_DIR = Path.home() / "semantic_nav_ws"
BENCHMARK_DIR = ROOT_DIR / "outputs" / "benchmark_rel_v1"
MANIFEST_PATH = BENCHMARK_DIR / "manifest.csv"
SUMMARY_DIR = BENCHMARK_DIR / "summary"


def get_float(row, key):
    return float(row[key])


def read_csv_rows(csv_path):
    with csv_path.open("r", encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))

    if not rows:
        raise RuntimeError(f"轨迹文件没有数据：{csv_path}")

    return rows


def calculate_path_length(rows):
    total_length = 0.0

    for index in range(1, len(rows)):
        previous_x = get_float(rows[index - 1], "x")
        previous_y = get_float(rows[index - 1], "y")

        current_x = get_float(rows[index], "x")
        current_y = get_float(rows[index], "y")

        total_length += math.hypot(
            current_x - previous_x,
            current_y - previous_y,
        )

    return total_length


def summarize_case(case_name, forward_m, left_m):
    trajectory_path = BENCHMARK_DIR / case_name / "trajectory.csv"
    rows = read_csv_rows(trajectory_path)

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

    path_length = calculate_path_length(rows)

    duration_sec = get_float(last_row, "time_sec")

    turn_samples = sum(
        1 for row in rows
        if row["mode"] == "转向"
    )

    forward_samples = sum(
        1 for row in rows
        if row["mode"] == "前进"
    )

    path_efficiency = 0.0

    if path_length > 0.0:
        path_efficiency = initial_distance / path_length

    return {
        "case_name": case_name,
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
        "path_length_m": path_length,
        "path_efficiency": path_efficiency,
        "duration_sec": duration_sec,
        "samples": len(rows),
        "turn_samples": turn_samples,
        "forward_samples": forward_samples,
        "rows": rows,
    }


def save_summary_csv(results):
    output_path = SUMMARY_DIR / "benchmark_summary.csv"

    fieldnames = [
        "case_name",
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
        "path_efficiency",
        "duration_sec",
        "samples",
        "turn_samples",
        "forward_samples",
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


def plot_final_errors(results):
    output_path = SUMMARY_DIR / "final_error_comparison.png"

    labels = [
        result["case_name"]
        for result in results
    ]

    errors_mm = [
        result["final_error_m"] * 1000.0
        for result in results
    ]

    positions = list(range(len(labels)))

    plt.figure(figsize=(10, 5))
    plt.bar(positions, errors_mm)
    plt.xticks(positions, labels, rotation=25, ha="right")
    plt.xlabel("Navigation case")
    plt.ylabel("Final goal error (mm)")
    plt.title("Closed-Loop Navigation: Final Goal Error")
    plt.grid(True, axis="y")
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close()

    return output_path


def plot_trajectory_overlay(results):
    output_path = SUMMARY_DIR / "trajectory_overlay.png"

    plt.figure(figsize=(8, 7))

    for result in results:
        rows = result["rows"]

        x_values = [
            get_float(row, "x")
            for row in rows
        ]

        y_values = [
            get_float(row, "y")
            for row in rows
        ]

        plt.plot(
            x_values,
            y_values,
            linewidth=1.8,
            label=result["case_name"],
        )

        plt.scatter(
            [result["start_x"]],
            [result["start_y"]],
            marker="o",
            s=45,
        )

        plt.scatter(
            [result["target_x"]],
            [result["target_y"]],
            marker="*",
            s=120,
        )

        plt.scatter(
            [result["end_x"]],
            [result["end_y"]],
            marker="X",
            s=55,
        )

    plt.xlabel("World X (m)")
    plt.ylabel("World Y (m)")
    plt.title("Closed-Loop Navigation: Six Trajectory Runs")
    plt.grid(True)
    plt.axis("equal")
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(output_path, dpi=180)
    plt.close()

    return output_path


def main():
    if not MANIFEST_PATH.exists():
        raise FileNotFoundError(
            f"找不到 manifest 文件：{MANIFEST_PATH}"
        )

    SUMMARY_DIR.mkdir(parents=True, exist_ok=True)

    results = []

    with MANIFEST_PATH.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            result = summarize_case(
                row["case_name"],
                float(row["forward_m"]),
                float(row["left_m"]),
            )

            results.append(result)

    if not results:
        raise RuntimeError("manifest 中没有实验记录。")

    summary_csv = save_summary_csv(results)
    error_plot = plot_final_errors(results)
    overlay_plot = plot_trajectory_overlay(results)

    print("基准实验汇总完成。")
    print()
    print("各组结果：")

    for result in results:
        print(
            f"{result['case_name']}: "
            f"误差={result['final_error_m']:.4f} m, "
            f"路径长度={result['path_length_m']:.4f} m, "
            f"耗时={result['duration_sec']:.2f} s, "
            f"采样点={result['samples']}"
        )

    average_error = sum(
        result["final_error_m"]
        for result in results
    ) / len(results)

    max_error = max(
        result["final_error_m"]
        for result in results
    )

    print()
    print(f"平均最终误差：{average_error:.4f} m")
    print(f"最大最终误差：{max_error:.4f} m")
    print()
    print("生成文件：")
    print(summary_csv)
    print(error_plot)
    print(overlay_plot)


if __name__ == "__main__":
    main()
