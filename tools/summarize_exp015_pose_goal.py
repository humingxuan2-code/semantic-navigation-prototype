from pathlib import Path
import csv
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


ROOT = Path.home() / "semantic_nav_ws"
OUT_DIR = ROOT / "outputs" / "exp015_pose_goal_v1"
SUMMARY_DIR = OUT_DIR / "summary"
SUMMARY_DIR.mkdir(parents=True, exist_ok=True)

SUMMARY_CSV = OUT_DIR / "pose_route_summary.csv"
TRAJ_CSV = OUT_DIR / "pose_route_trajectory.csv"

ROUTE_FIG = SUMMARY_DIR / "pose_goal_route_overview.png"
ERROR_FIG = SUMMARY_DIR / "pose_goal_error_summary.png"
METRICS_TXT = SUMMARY_DIR / "pose_goal_metrics.txt"

OBSTACLE_CENTER = (3.5, 0.0)
OBSTACLE_SIZE = (1.4, 1.6)
GOAL_TOLERANCE_MM = 25.0
YAW_TOLERANCE_DEG = 3.0


def read_rows(path):
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def f(row, key):
    return float(row[key])


summary_rows = read_rows(SUMMARY_CSV)
traj_rows = read_rows(TRAJ_CSV)

if not summary_rows:
    raise RuntimeError("pose_route_summary.csv is empty.")

if not traj_rows:
    raise RuntimeError("pose_route_trajectory.csv is empty.")

traj_x = [f(r, "x") for r in traj_rows]
traj_y = [f(r, "y") for r in traj_rows]

target_x = [f(r, "target_x") for r in summary_rows]
target_y = [f(r, "target_y") for r in summary_rows]
target_yaw = [f(r, "target_yaw_deg") for r in summary_rows]

end_x = [f(r, "end_x") for r in summary_rows]
end_y = [f(r, "end_y") for r in summary_rows]
end_yaw = [f(r, "end_yaw_deg") for r in summary_rows]

xy_errors_mm = [f(r, "final_error_m") * 1000.0 for r in summary_rows]
yaw_errors_deg = [abs(f(r, "final_yaw_error_deg")) for r in summary_rows]
durations = [f(r, "duration_sec") for r in summary_rows]
statuses = [r["status"] for r in summary_rows]

success_count = sum(s == "success" for s in statuses)
total_count = len(statuses)

# Figure 1: route overview
fig, ax = plt.subplots(figsize=(11, 7))

ox, oy = OBSTACLE_CENTER
sx, sy = OBSTACLE_SIZE
ax.add_patch(
    Rectangle(
        (ox - sx / 2.0, oy - sy / 2.0),
        sx,
        sy,
        alpha=0.35,
        label="Static obstacle",
    )
)

ax.plot(traj_x, traj_y, linewidth=2, label="Executed trajectory")
ax.scatter(target_x, target_y, marker="x", s=90, label="Pose targets")
ax.scatter(end_x, end_y, marker="o", s=50, label="Waypoint final poses")
ax.scatter([traj_x[0]], [traj_y[0]], marker="o", s=130, label="Start")
ax.scatter([target_x[-1]], [target_y[-1]], marker="*", s=220, label="Final target")

for row in summary_rows:
    name = row["waypoint_name"]
    x = f(row, "target_x")
    y = f(row, "target_y")
    yaw_deg = f(row, "target_yaw_deg")
    yaw_rad = math.radians(yaw_deg)

    ax.text(x + 0.05, y + 0.05, name.replace("pose_", ""), fontsize=8)
    ax.arrow(
        x,
        y,
        0.25 * math.cos(yaw_rad),
        0.25 * math.sin(yaw_rad),
        head_width=0.05,
        length_includes_head=True,
    )

ax.set_title("EXP-015: Pose Goal Navigation with Final Yaw Alignment")
ax.set_xlabel("Odom X (m)")
ax.set_ylabel("Odom Y (m)")
ax.axis("equal")
ax.grid(True)
ax.legend(loc="best")
fig.tight_layout()
fig.savefig(ROUTE_FIG, dpi=160)
plt.close(fig)

# Figure 2: error summary
labels = [r["waypoint_name"].replace("pose_", "") for r in summary_rows]

fig, ax1 = plt.subplots(figsize=(13, 6))

x_idx = list(range(len(labels)))
ax1.bar(x_idx, xy_errors_mm, label="Final position error (mm)")
ax1.axhline(GOAL_TOLERANCE_MM, linestyle="--", linewidth=1.5, label="Position tolerance 25 mm")
ax1.set_ylabel("Final position error (mm)")
ax1.set_xlabel("Pose waypoint")
ax1.set_xticks(x_idx)
ax1.set_xticklabels(labels, rotation=35, ha="right")
ax1.grid(True, axis="y")

ax2 = ax1.twinx()
ax2.plot(x_idx, yaw_errors_deg, marker="o", linewidth=2, label="Final yaw error (deg)")
ax2.axhline(YAW_TOLERANCE_DEG, linestyle=":", linewidth=1.5, label="Yaw tolerance 3 deg")
ax2.set_ylabel("Final yaw error (deg)")

lines1, labels1 = ax1.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax1.legend(lines1 + lines2, labels1 + labels2, loc="upper right")

ax1.set_title("EXP-015: Position and Final Yaw Error Summary")
fig.tight_layout()
fig.savefig(ERROR_FIG, dpi=160)
plt.close(fig)

lines = [
    "EXP-015 Pose Goal Navigation Summary",
    "",
    f"success rate: {success_count} / {total_count}",
    f"max final position error: {max(xy_errors_mm):.2f} mm",
    f"average final position error: {sum(xy_errors_mm) / len(xy_errors_mm):.2f} mm",
    f"max absolute final yaw error: {max(yaw_errors_deg):.2f} deg",
    f"average absolute final yaw error: {sum(yaw_errors_deg) / len(yaw_errors_deg):.2f} deg",
    f"total execution time: {sum(durations):.2f} s",
    "",
    "final pose goal:",
    f"- target: x={target_x[-1]:.3f}, y={target_y[-1]:.3f}, yaw={target_yaw[-1]:.2f} deg",
    f"- actual: x={end_x[-1]:.3f}, y={end_y[-1]:.3f}, yaw={end_yaw[-1]:.2f} deg",
    f"- final position error: {xy_errors_mm[-1]:.2f} mm",
    f"- final yaw error: {yaw_errors_deg[-1]:.2f} deg",
    "",
    f"route overview figure: {ROUTE_FIG}",
    f"error summary figure: {ERROR_FIG}",
]

METRICS_TXT.write_text("\n".join(lines) + "\n", encoding="utf-8")

print("\n".join(lines))
