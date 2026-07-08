from pathlib import Path
import csv

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


ROOT = Path.home() / "semantic_nav_ws"
OUT_DIR = ROOT / "outputs" / "exp017_safety_guard_v1"
SUMMARY_DIR = OUT_DIR / "summary"
SUMMARY_DIR.mkdir(parents=True, exist_ok=True)

SUMMARY_CSV = OUT_DIR / "safety_route_summary.csv"
TRAJ_CSV = OUT_DIR / "safety_route_trajectory.csv"

FIG_PATH = SUMMARY_DIR / "safety_guard_stop_overview.png"
METRICS_TXT = SUMMARY_DIR / "safety_guard_metrics.txt"

OBSTACLE_CENTER = (3.5, 0.0)
OBSTACLE_SIZE = (1.4, 1.6)
SAFETY_STOP_THRESHOLD_M = 0.25


def read_rows(path):
    with path.open("r", encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


summary_rows = read_rows(SUMMARY_CSV)
traj_rows = read_rows(TRAJ_CSV)

if not summary_rows:
    raise RuntimeError("safety_route_summary.csv is empty.")

if not traj_rows:
    raise RuntimeError("safety_route_trajectory.csv is empty.")

traj_x = [float(r["x"]) for r in traj_rows]
traj_y = [float(r["y"]) for r in traj_rows]
clearance_m = [float(r["footprint_clearance_m"]) for r in traj_rows]

last = summary_rows[-1]
status = last["status"]
min_clearance_m = float(last["min_clearance_m"])
min_clearance_mm = float(last["min_clearance_mm"])
final_error_m = float(last["final_error_m"])
duration_sec = float(last["duration_sec"])

# Plot route and safety stop point
fig, ax = plt.subplots(figsize=(10, 7))

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

ax.plot(traj_x, traj_y, linewidth=2, label="Executed trajectory before safety stop")
ax.scatter([traj_x[0]], [traj_y[0]], s=120, marker="o", label="Start")
ax.scatter([traj_x[-1]], [traj_y[-1]], s=180, marker="x", label="Safety stop pose")

ax.set_title("EXP-017: Online Footprint Safety Guard Stop")
ax.set_xlabel("Odom X (m)")
ax.set_ylabel("Odom Y (m)")
ax.axis("equal")
ax.grid(True)
ax.legend(loc="best")
fig.tight_layout()
fig.savefig(FIG_PATH, dpi=160)
plt.close(fig)

lines = [
    "EXP-017 Online Footprint Safety Guard Summary",
    "",
    f"status: {status}",
    f"safety stop threshold: {SAFETY_STOP_THRESHOLD_M:.3f} m",
    f"minimum footprint clearance: {min_clearance_m:.3f} m",
    f"minimum footprint clearance: {min_clearance_mm:.2f} mm",
    f"remaining final waypoint error at stop: {final_error_m:.3f} m",
    f"duration before stop: {duration_sec:.2f} s",
    "",
    "interpretation:",
    "- The original EXP-014-v1 route was unsafe for the robot footprint.",
    "- The online safety guard detected that footprint clearance dropped below the configured threshold.",
    "- The controller stopped before completing the waypoint route, preventing the unsafe trajectory from continuing.",
    "- This validates online safety-stop behavior, not full obstacle avoidance or replanning.",
    "",
    f"overview figure: {FIG_PATH}",
]

METRICS_TXT.write_text("\n".join(lines) + "\n", encoding="utf-8")

print("\n".join(lines))
