from pathlib import Path
import csv
import heapq
import math

import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


START = (0.0, 0.0)
GOAL = (5.5, 2.0)

RESOLUTION = 0.10
X_MIN, X_MAX = -0.5, 6.0
Y_MIN, Y_MAX = -1.5, 4.8

# Obstacle position converted into vehicle_blue/odom frame.
# In the world file, obstacle center is (3.5, 2.0), while blue car starts at world y=2.0.
OBSTACLE_CENTER = (3.5, 0.0)
OBSTACLE_SIZE = (1.4, 1.6)

# Safety margin around the obstacle for planning.
SAFETY_MARGIN = 0.75


ROOT = Path.home() / "semantic_nav_ws"
ROUTE_DIR = ROOT / "routes"
OUT_DIR = ROOT / "outputs" / "exp014_astar_planning_v1"
SUMMARY_DIR = OUT_DIR / "summary"

ROUTE_DIR.mkdir(parents=True, exist_ok=True)
SUMMARY_DIR.mkdir(parents=True, exist_ok=True)

ROUTE_CSV = ROUTE_DIR / "exp014_astar_obstacle_detour.csv"
FULL_PATH_CSV = OUT_DIR / "astar_full_path.csv"
WAYPOINT_CSV = OUT_DIR / "astar_simplified_waypoints.csv"
PLOT_PATH = SUMMARY_DIR / "astar_obstacle_route.png"
METRICS_PATH = SUMMARY_DIR / "astar_planning_metrics.txt"


def to_grid(point):
    x, y = point
    ix = round((x - X_MIN) / RESOLUTION)
    iy = round((y - Y_MIN) / RESOLUTION)
    return ix, iy


def to_world(cell):
    ix, iy = cell
    x = X_MIN + ix * RESOLUTION
    y = Y_MIN + iy * RESOLUTION
    return round(x, 6), round(y, 6)


def in_bounds(cell):
    x, y = to_world(cell)
    return X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX


def inside_inflated_obstacle(point):
    x, y = point
    ox, oy = OBSTACLE_CENTER
    sx, sy = OBSTACLE_SIZE

    x0 = ox - sx / 2.0 - SAFETY_MARGIN
    x1 = ox + sx / 2.0 + SAFETY_MARGIN
    y0 = oy - sy / 2.0 - SAFETY_MARGIN
    y1 = oy + sy / 2.0 + SAFETY_MARGIN

    return x0 <= x <= x1 and y0 <= y <= y1


def is_blocked(cell):
    return inside_inflated_obstacle(to_world(cell))


def heuristic(a, b):
    ax, ay = to_world(a)
    bx, by = to_world(b)
    return math.hypot(ax - bx, ay - by)


def neighbors(cell):
    ix, iy = cell
    steps = [
        (-1, 0), (1, 0), (0, -1), (0, 1),
        (-1, -1), (-1, 1), (1, -1), (1, 1),
    ]

    for dx, dy in steps:
        nxt = (ix + dx, iy + dy)
        if not in_bounds(nxt):
            continue
        if is_blocked(nxt):
            continue
        cost = math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0
        yield nxt, cost * RESOLUTION


def astar(start, goal):
    start_cell = to_grid(start)
    goal_cell = to_grid(goal)

    if is_blocked(start_cell):
        raise RuntimeError("Start point is inside inflated obstacle.")
    if is_blocked(goal_cell):
        raise RuntimeError("Goal point is inside inflated obstacle.")

    frontier = []
    heapq.heappush(frontier, (0.0, start_cell))

    came_from = {start_cell: None}
    cost_so_far = {start_cell: 0.0}

    while frontier:
        _, current = heapq.heappop(frontier)

        if current == goal_cell:
            break

        for nxt, step_cost in neighbors(current):
            new_cost = cost_so_far[current] + step_cost

            if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                cost_so_far[nxt] = new_cost
                priority = new_cost + heuristic(nxt, goal_cell)
                heapq.heappush(frontier, (priority, nxt))
                came_from[nxt] = current

    if goal_cell not in came_from:
        raise RuntimeError("A* failed to find a path.")

    path = []
    cur = goal_cell

    while cur is not None:
        path.append(to_world(cur))
        cur = came_from[cur]

    path.reverse()
    return path


def line_is_free(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    length = math.hypot(x2 - x1, y2 - y1)

    if length == 0:
        return True

    steps = max(2, int(length / (RESOLUTION * 0.5)))

    for i in range(steps + 1):
        t = i / steps
        x = x1 + (x2 - x1) * t
        y = y1 + (y2 - y1) * t

        if not (X_MIN <= x <= X_MAX and Y_MIN <= y <= Y_MAX):
            return False
        if inside_inflated_obstacle((x, y)):
            return False

    return True


def simplify_path(path):
    if len(path) <= 2:
        return path

    simplified = [path[0]]
    i = 0

    while i < len(path) - 1:
        j = len(path) - 1

        while j > i + 1:
            if line_is_free(path[i], path[j]):
                break
            j -= 1

        simplified.append(path[j])
        i = j

    return simplified


def path_length(path):
    total = 0.0

    for p1, p2 in zip(path[:-1], path[1:]):
        total += math.hypot(p2[0] - p1[0], p2[1] - p1[1])

    return total


full_path = astar(START, GOAL)
waypoints = simplify_path(full_path)

# Do not include the start point in the controller route CSV.
controller_waypoints = waypoints[1:]

with ROUTE_CSV.open("w", encoding="utf-8", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["waypoint_name", "target_x", "target_y"])

    for idx, (x, y) in enumerate(controller_waypoints, start=1):
        name = f"astar_wp{idx:02d}"
        if idx == len(controller_waypoints):
            name = "astar_goal"
        writer.writerow([name, f"{x:.3f}", f"{y:.3f}"])

with FULL_PATH_CSV.open("w", encoding="utf-8", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])

    for x, y in full_path:
        writer.writerow([f"{x:.3f}", f"{y:.3f}"])

with WAYPOINT_CSV.open("w", encoding="utf-8", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["x", "y"])

    for x, y in waypoints:
        writer.writerow([f"{x:.3f}", f"{y:.3f}"])

actual_x = OBSTACLE_CENTER[0] - OBSTACLE_SIZE[0] / 2.0
actual_y = OBSTACLE_CENTER[1] - OBSTACLE_SIZE[1] / 2.0
inflated_x = actual_x - SAFETY_MARGIN
inflated_y = actual_y - SAFETY_MARGIN
inflated_w = OBSTACLE_SIZE[0] + 2.0 * SAFETY_MARGIN
inflated_h = OBSTACLE_SIZE[1] + 2.0 * SAFETY_MARGIN

plt.figure(figsize=(10, 7))

plt.gca().add_patch(
    Rectangle(
        (inflated_x, inflated_y),
        inflated_w,
        inflated_h,
        alpha=0.15,
        label="Inflated planning obstacle",
    )
)

plt.gca().add_patch(
    Rectangle(
        (actual_x, actual_y),
        OBSTACLE_SIZE[0],
        OBSTACLE_SIZE[1],
        alpha=0.35,
        label="Actual static obstacle",
    )
)

xs = [p[0] for p in full_path]
ys = [p[1] for p in full_path]
plt.plot(xs, ys, linewidth=1.5, label="A* grid path")

wxs = [p[0] for p in waypoints]
wys = [p[1] for p in waypoints]
plt.plot(wxs, wys, marker="o", linewidth=2.5, label="Simplified controller waypoints")

plt.scatter([START[0]], [START[1]], marker="o", s=120, label="Start")
plt.scatter([GOAL[0]], [GOAL[1]], marker="*", s=220, label="Goal")

for idx, (x, y) in enumerate(waypoints):
    label = "start" if idx == 0 else f"wp{idx:02d}"
    if idx == len(waypoints) - 1:
        label = "goal"
    plt.text(x + 0.06, y + 0.06, label)

plt.title("EXP-014: A* Planned Odom-Frame Detour Route")
plt.xlabel("Odom X (m)")
plt.ylabel("Odom Y (m)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.savefig(PLOT_PATH, dpi=160)
plt.close()

metrics = [
    "EXP-014 A* known-map obstacle detour planning",
    f"start: {START}",
    f"goal: {GOAL}",
    f"grid resolution: {RESOLUTION:.2f} m",
    f"safety margin: {SAFETY_MARGIN:.2f} m",
    f"full A* path points: {len(full_path)}",
    f"simplified waypoint count including start: {len(waypoints)}",
    f"controller waypoint count: {len(controller_waypoints)}",
    f"full path length: {path_length(full_path):.3f} m",
    f"simplified path length: {path_length(waypoints):.3f} m",
    f"route csv: {ROUTE_CSV}",
    f"plot: {PLOT_PATH}",
]

METRICS_PATH.write_text("\n".join(metrics) + "\n", encoding="utf-8")

print("\n".join(metrics))
