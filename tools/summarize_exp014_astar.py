from pathlib import Path
import csv
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


ROOT = Path.home() / "semantic_nav_ws"

OUT_DIR = ROOT / "outputs" / "exp014_astar_comparison_v1"
SUMMARY_DIR = OUT_DIR / "summary"
SUMMARY_DIR.mkdir(parents=True, exist_ok=True)

V1_PLAN = ROOT / "outputs" / "exp014_astar_planning_v1" / "astar_simplified_waypoints.csv"
V2_PLAN = ROOT / "outputs" / "exp014_astar_planning_v2" / "astar_simplified_waypoints.csv"

V1_EXEC_TRAJ = ROOT / "outputs" / "exp014_astar_execution_v1" / "global_route_trajectory.csv"
V2_EXEC_TRAJ = ROOT / "outputs" / "exp014_astar_execution_v2" / "global_route_trajectory.csv"

V1_EXEC_SUMMARY = ROOT / "outputs" / "exp014_astar_execution_v1" / "global_route_summary.csv"
V2_EXEC_SUMMARY = ROOT / "outputs" / "exp014_astar_execution_v2" / "global_route_summary.csv"

ROUTE_FIG = SUMMARY_DIR / "exp014_v1_v2_route_comparison.png"
ERROR_FIG = SUMMARY_DIR / "exp014_v1_v2_error_comparison.png"
METRICS_TXT = SUMMARY_DIR / "exp014_comparison_metrics.txt"

OBSTACLE_CENTER = (3.5, 0.0)
OBSTACLE_SIZE = (1.4, 1.6)
V1_MARGIN = 0.75
V2_MARGIN = 1.45
GOAL_TOLERANCE_MM = 25.0


def read_xy(path):
    points = []

    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            if "x" not in row or "y" not in row:
                continue

            try:
                points.append((float(row["x"]), float(row["y"])))
            except ValueError:
                pass

    return points


def read_summary(path):
    rows = []

    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)

        for row in reader:
            rows.append(row)

    return rows


def final_errors_mm(rows):
    values = []

    for row in rows:
        values.append(float(row["final_error_m"]) * 1000.0)

    return values


def success_count(rows):
    return sum(row.get("status", "") == "success" for row in rows)


def path_length(points):
    total = 0.0

    for a, b in zip(points[:-1], points[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])

    return total


def add_obstacle(ax, margin, label):
    ox, oy = OBSTACLE_CENTER
    sx, sy = OBSTACLE_SIZE

    actual_x = ox - sx / 2.0
    actual_y = oy - sy / 2.0

    inflated_x = actual_x - margin
    inflated_y = actual_y - margin
    inflated_w = sx + 2.0 * margin
    inflated_h = sy + 2.0 * margin

    ax.add_patch(
        Rectangle(
            (inflated_x, inflated_y),
            inflated_w,
            inflated_h,
            alpha=0.12,
            label=label,
        )
    )

    ax.add_patch(
        Rectangle(
            (actual_x, actual_y),
            sx,
            sy,
            alpha=0.35,
            label="Actual obstacle",
        )
    )


v1_plan = read_xy(V1_PLAN)
v2_plan = read_xy(V2_PLAN)
v1_traj = read_xy(V1_EXEC_TRAJ)
v2_traj = read_xy(V2_EXEC_TRAJ)

v1_rows = read_summary(V1_EXEC_SUMMARY)
v2_rows = read_summary(V2_EXEC_SUMMARY)

v1_errors = final_errors_mm(v1_rows)
v2_errors = final_errors_mm(v2_rows)

# Route comparison figure
fig, ax = plt.subplots(figsize=(11, 7))

add_obstacle(ax, V2_MARGIN, "V2 inflated obstacle margin")

if v1_plan:
    ax.plot(
        [p[0] for p in v1_plan],
        [p[1] for p in v1_plan],
        marker="o",
        linestyle="--",
        linewidth=2,
        label="V1 planned simplified waypoints",
    )

if v2_plan:
    ax.plot(
        [p[0] for p in v2_plan],
        [p[1] for p in v2_plan],
        marker="o",
        linestyle="-",
        linewidth=2,
        label="V2 planned waypoint route",
    )

if v1_traj:
    ax.plot(
        [p[0] for p in v1_traj],
        [p[1] for p in v1_traj],
        linewidth=1.5,
        alpha=0.7,
        label="V1 executed trajectory",
    )

if v2_traj:
    ax.plot(
        [p[0] for p in v2_traj],
        [p[1] for p in v2_traj],
        linewidth=2,
        alpha=0.9,
        label="V2 executed trajectory",
    )

ax.scatter([0.0], [0.0], s=120, marker="o", label="Start")
ax.scatter([5.8], [2.0], s=220, marker="*", label="V2 goal")

ax.set_title("EXP-014: A* Route Comparison Before and After Safety Inflation")
ax.set_xlabel("Odom X (m)")
ax.set_ylabel("Odom Y (m)")
ax.axis("equal")
ax.grid(True)
ax.legend(loc="best")
fig.tight_layout()
fig.savefig(ROUTE_FIG, dpi=160)
plt.close(fig)

# Error comparison figure
labels = []
errors = []

for row in v1_rows:
    labels.append("v1_" + row["waypoint_name"])
    errors.append(float(row["final_error_m"]) * 1000.0)

for row in v2_rows:
    labels.append("v2_" + row["waypoint_name"])
    errors.append(float(row["final_error_m"]) * 1000.0)

fig, ax = plt.subplots(figsize=(12, 6))
ax.bar(labels, errors)
ax.axhline(
    GOAL_TOLERANCE_MM,
    linestyle="--",
    linewidth=1.5,
    label="Goal tolerance 25 mm",
)
ax.set_title("EXP-014: Final Waypoint Error Comparison")
ax.set_xlabel("Execution waypoint")
ax.set_ylabel("Final goal error (mm)")
ax.tick_params(axis="x", rotation=35)
ax.grid(True, axis="y")
ax.legend()
fig.tight_layout()
fig.savefig(ERROR_FIG, dpi=160)
plt.close(fig)

lines = [
    "EXP-014 A* static-obstacle detour comparison",
    "",
    "V1 result:",
    f"- controller waypoints: {len(v1_rows)}",
    f"- success rate: {success_count(v1_rows)} / {len(v1_rows)}",
    f"- max final error: {max(v1_errors):.2f} mm",
    f"- average final error: {sum(v1_errors) / len(v1_errors):.2f} mm",
    f"- executed trajectory length: {path_length(v1_traj):.3f} m",
    "- qualitative observation: endpoint tracking succeeded, but the robot body / wheels passed too close to the obstacle.",
    "- interpretation: point-robot A* plus over-simplified waypoints was insufficient for safe execution.",
    "",
    "V2 result:",
    f"- controller waypoints: {len(v2_rows)}",
    f"- success rate: {success_count(v2_rows)} / {len(v2_rows)}",
    f"- max final error: {max(v2_errors):.2f} mm",
    f"- average final error: {sum(v2_errors) / len(v2_errors):.2f} mm",
    f"- executed trajectory length: {path_length(v2_traj):.3f} m",
    "- qualitative observation: the robot safely detoured around the static obstacle without visible contact.",
    "- interpretation: larger obstacle inflation and denser intermediate waypoints improved execution safety.",
    "",
    f"route comparison figure: {ROUTE_FIG}",
    f"error comparison figure: {ERROR_FIG}",
]

METRICS_TXT.write_text("\n".join(lines) + "\n", encoding="utf-8")

print("\n".join(lines))
