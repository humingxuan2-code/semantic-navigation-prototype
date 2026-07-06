import csv
from pathlib import Path

import matplotlib.pyplot as plt


CSV_PATH = Path.home() / "semantic_nav_ws/tools/trajectory_latest.csv"
OUTPUT_PATH = Path.home() / "semantic_nav_ws/tools/trajectory_latest.png"


def main():
    if not CSV_PATH.exists():
        raise FileNotFoundError(f"找不到轨迹文件：{CSV_PATH}")

    time_sec = []
    x_values = []
    y_values = []
    target_x_values = []
    target_y_values = []
    distance_values = []
    mode_values = []

    with CSV_PATH.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)

        for row in reader:
            time_sec.append(float(row["time_sec"]))
            x_values.append(float(row["x"]))
            y_values.append(float(row["y"]))
            target_x_values.append(float(row["target_x"]))
            target_y_values.append(float(row["target_y"]))
            distance_values.append(float(row["distance_to_goal"]))
            mode_values.append(row["mode"])

    if not x_values:
        raise RuntimeError("CSV 中没有轨迹数据。")

    start_x = x_values[0]
    start_y = y_values[0]

    target_x = target_x_values[0]
    target_y = target_y_values[0]

    end_x = x_values[-1]
    end_y = y_values[-1]

    final_error = distance_values[-1]

    plt.figure(figsize=(8, 7))

    plt.plot(
        x_values,
        y_values,
        linewidth=2,
        label="Actual trajectory",
    )

    plt.scatter(
        [start_x],
        [start_y],
        marker="o",
        s=100,
        label="Start",
    )

    plt.scatter(
        [target_x],
        [target_y],
        marker="*",
        s=220,
        label="Target",
    )

    plt.scatter(
        [end_x],
        [end_y],
        marker="X",
        s=120,
        label="End",
    )

    plt.annotate(
        f"Start\n({start_x:.3f}, {start_y:.3f})",
        (start_x, start_y),
        textcoords="offset points",
        xytext=(8, 8),
    )

    plt.annotate(
        f"Target\n({target_x:.3f}, {target_y:.3f})",
        (target_x, target_y),
        textcoords="offset points",
        xytext=(8, 8),
    )

    plt.annotate(
        f"End\n({end_x:.3f}, {end_y:.3f})",
        (end_x, end_y),
        textcoords="offset points",
        xytext=(8, -30),
    )

    plt.xlabel("World X (m)")
    plt.ylabel("World Y (m)")
    plt.title("Vehicle Blue: Closed-Loop Point-to-Point Navigation")
    plt.grid(True)
    plt.axis("equal")
    plt.legend()

    plt.text(
        0.02,
        0.02,
        f"Final goal error: {final_error:.3f} m\n"
        f"Samples: {len(x_values)}",
        transform=plt.gca().transAxes,
        verticalalignment="bottom",
    )

    plt.tight_layout()
    plt.savefig(OUTPUT_PATH, dpi=180)
    plt.close()

    print("轨迹图已保存：")
    print(OUTPUT_PATH)
    print()
    print(f"起点：({start_x:.3f}, {start_y:.3f})")
    print(f"目标点：({target_x:.3f}, {target_y:.3f})")
    print(f"终点：({end_x:.3f}, {end_y:.3f})")
    print(f"最终目标误差：{final_error:.3f} m")
    print(f"轨迹采样点数量：{len(x_values)}")


if __name__ == "__main__":
    main()
