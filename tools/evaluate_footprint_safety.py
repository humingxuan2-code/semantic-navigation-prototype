from pathlib import Path
import csv
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon


ROOT = Path.home() / "semantic_nav_ws"

OUT_DIR = ROOT / "outputs" / "exp016_footprint_safety_v1"
SUMMARY_DIR = OUT_DIR / "summary"
SUMMARY_DIR.mkdir(parents=True, exist_ok=True)

OBSTACLE_CENTER = (3.5, 0.0)
OBSTACLE_SIZE = (1.4, 1.6)

# Conservative footprint of the simulated differential-drive vehicle.
# This covers chassis + wheel sweep area more realistically than a point robot.
ROBOT_LENGTH = 2.20
ROBOT_WIDTH = 1.80

TRAJECTORIES = [
    (
        "exp014_v1_point_astar",
        ROOT / "outputs" / "exp014_astar_execution_v1" / "global_route_trajectory.csv",
    ),
    (
        "exp014_v2_inflated_astar",
        ROOT / "outputs" / "exp014_astar_execution_v2" / "global_route_trajectory.csv",
    ),
    (
        "exp015_pose_goal",
        ROOT / "outputs" / "exp015_pose_goal_v1" / "pose_route_trajectory.csv",
    ),
]

METRICS_CSV = OUT_DIR / "footprint_safety_summary.csv"
METRICS_TXT = SUMMARY_DIR / "footprint_safety_metrics.txt"
FIG_PATH = SUMMARY_DIR / "footprint_safety_comparison.png"


def read_csv_rows(path):
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def get_float(row, candidates, default=None):
    for key in candidates:
        if key in row and row[key] not in ("", None):
            return float(row[key])
    if default is not None:
        return default
    raise KeyError(f"Missing any of columns: {candidates}")


def infer_trajectory(path):
    rows = read_csv_rows(path)
    points = []

    for row in rows:
        x = get_float(row, ["x", "current_x", "end_x"])
        y = get_float(row, ["y", "current_y", "end_y"])
        yaw_deg = get_float(row, ["yaw_deg", "current_yaw_deg", "yaw"], default=0.0)

        # If a controller stored yaw in radians under "yaw", convert only if it looks radian-scale.
        if "yaw" in row and "yaw_deg" not in row and abs(yaw_deg) <= 2.0 * math.pi + 0.1:
            yaw_deg = math.degrees(yaw_deg)

        points.append((x, y, math.radians(yaw_deg)))

    return points


def obstacle_corners():
    ox, oy = OBSTACLE_CENTER
    sx, sy = OBSTACLE_SIZE

    x0 = ox - sx / 2.0
    x1 = ox + sx / 2.0
    y0 = oy - sy / 2.0
    y1 = oy + sy / 2.0

    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def robot_corners(x, y, yaw):
    hl = ROBOT_LENGTH / 2.0
    hw = ROBOT_WIDTH / 2.0

    local = [
        (hl, hw),
        (hl, -hw),
        (-hl, -hw),
        (-hl, hw),
    ]

    c = math.cos(yaw)
    s = math.sin(yaw)

    return [
        (x + lx * c - ly * s, y + lx * s + ly * c)
        for lx, ly in local
    ]


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def polygon_axes(poly):
    axes = []

    for p1, p2 in zip(poly, poly[1:] + poly[:1]):
        edge = (p2[0] - p1[0], p2[1] - p1[1])
        normal = (-edge[1], edge[0])
        length = math.hypot(normal[0], normal[1])

        if length > 1e-9:
            axes.append((normal[0] / length, normal[1] / length))

    return axes


def project(poly, axis):
    vals = [dot(p, axis) for p in poly]
    return min(vals), max(vals)


def polygons_intersect(poly_a, poly_b):
    for axis in polygon_axes(poly_a) + polygon_axes(poly_b):
        amin, amax = project(poly_a, axis)
        bmin, bmax = project(poly_b, axis)

        if amax < bmin or bmax < amin:
            return False

    return True


def point_segment_distance(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b

    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay

    denom = abx * abx + aby * aby

    if denom <= 1e-12:
        return math.hypot(px - ax, py - ay)

    t = max(0.0, min(1.0, (apx * abx + apy * aby) / denom))
    cx = ax + t * abx
    cy = ay + t * aby

    return math.hypot(px - cx, py - cy)


def polygon_distance(poly_a, poly_b):
    if polygons_intersect(poly_a, poly_b):
        return 0.0

    distances = []

    for p in poly_a:
        for a, b in zip(poly_b, poly_b[1:] + poly_b[:1]):
            distances.append(point_segment_distance(p, a, b))

    for p in poly_b:
        for a, b in zip(poly_a, poly_a[1:] + poly_a[:1]):
            distances.append(point_segment_distance(p, a, b))

    return min(distances)


def path_length(points):
    total = 0.0

    for a, b in zip(points[:-1], points[1:]):
        total += math.hypot(b[0] - a[0], b[1] - a[1])

    return total


obs_poly = obstacle_corners()
results = []

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

for name, traj_path in TRAJECTORIES:
    points = infer_trajectory(traj_path)

    min_clearance = float("inf")
    collision_count = 0
    worst_poly = None
    worst_pose = None

    for x, y, yaw in points:
        footprint = robot_corners(x, y, yaw)
        d = polygon_distance(footprint, obs_poly)

        if polygons_intersect(footprint, obs_poly):
            collision_count += 1

        if d < min_clearance:
            min_clearance = d
            worst_poly = footprint
            worst_pose = (x, y, yaw)

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]

    ax.plot(xs, ys, linewidth=2, label=name)

    if worst_poly:
        ax.add_patch(
            Polygon(
                worst_poly,
                closed=True,
                alpha=0.18,
                label=f"{name} closest footprint",
            )
        )

    status = "safe" if collision_count == 0 else "collision_risk"

    results.append({
        "experiment": name,
        "trajectory_points": len(points),
        "path_length_m": path_length(points),
        "min_clearance_m": min_clearance,
        "min_clearance_mm": min_clearance * 1000.0,
        "collision_or_overlap_samples": collision_count,
        "status": status,
        "trajectory_file": str(traj_path),
    })

ax.set_title("EXP-016: Footprint-Based Trajectory Safety Evaluation")
ax.set_xlabel("Odom X (m)")
ax.set_ylabel("Odom Y (m)")
ax.axis("equal")
ax.grid(True)
ax.legend(loc="best", fontsize=8)
fig.tight_layout()
fig.savefig(FIG_PATH, dpi=160)
plt.close(fig)

with METRICS_CSV.open("w", encoding="utf-8", newline="") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=[
            "experiment",
            "trajectory_points",
            "path_length_m",
            "min_clearance_m",
            "min_clearance_mm",
            "collision_or_overlap_samples",
            "status",
            "trajectory_file",
        ],
    )
    writer.writeheader()
    writer.writerows(results)

lines = [
    "EXP-016 Footprint-Based Trajectory Safety Evaluation",
    "",
    f"robot footprint length: {ROBOT_LENGTH:.2f} m",
    f"robot footprint width: {ROBOT_WIDTH:.2f} m",
    f"obstacle center in odom frame: {OBSTACLE_CENTER}",
    f"obstacle size: {OBSTACLE_SIZE}",
    "",
]

for r in results:
    lines.extend([
        r["experiment"],
        f"- trajectory points: {r['trajectory_points']}",
        f"- path length: {r['path_length_m']:.3f} m",
        f"- minimum footprint-obstacle clearance: {r['min_clearance_mm']:.2f} mm",
        f"- collision / overlap samples: {r['collision_or_overlap_samples']}",
        f"- status: {r['status']}",
        "",
    ])

lines.extend([
    f"summary csv: {METRICS_CSV}",
    f"comparison figure: {FIG_PATH}",
])

METRICS_TXT.write_text("\n".join(lines) + "\n", encoding="utf-8")

print("\n".join(lines))
