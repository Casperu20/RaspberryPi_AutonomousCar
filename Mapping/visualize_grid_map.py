#!/usr/bin/env python3
"""
Dynamic visualizer for robot_grid_map.json.

It automatically:
- reads map width/height from JSON
- detects known/visited/obstacle/free cells
- crops around the explored area
- keeps real grid coordinates on axes
- works even if MAP_WIDTH / MAP_HEIGHT changes in the mapper

Usage:
    python3 visualize_grid_map.py

Live update:
    python3 visualize_grid_map.py --watch

Full map:
    python3 visualize_grid_map.py --full
"""

import argparse
import json
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.patches import Patch


STATE_VALUES = {
    "unknown": 0,
    "free": 1,
    "visited": 2,
    "obstacle": 3,
    "robot": 4,
}

HEADING_SYMBOLS = {
    "NORTH": "^",
    "EAST": ">",
    "SOUTH": "v",
    "WEST": "<",
}


def load_map(path):
    with open(path, "r") as f:
        return json.load(f)


def get_map_dimensions(payload):
    """
    Read dimensions from the JSON if available.
    If not available, infer them from the grid itself.
    """
    grid = payload.get("grid", [])

    height_from_grid = len(grid)
    width_from_grid = len(grid[0]) if height_from_grid > 0 else 0

    width = payload.get("width", width_from_grid)
    height = payload.get("height", height_from_grid)

    return width, height


def find_known_points(payload):
    """
    Return all cells that should be included in the cropped view:
    - free cells
    - visited cells
    - obstacle cells
    - robot position
    """
    width, height = get_map_dimensions(payload)
    grid = payload.get("grid", [])
    robot = payload.get("robot", {})

    known_points = []

    for y in range(min(height, len(grid))):
        row = grid[y]

        for x in range(min(width, len(row))):
            cell = row[x]
            state = cell.get("state", "unknown")
            visited = cell.get("visited", False)

            if state != "unknown" or visited:
                known_points.append((x, y))

    rx = robot.get("x")
    ry = robot.get("y")

    if isinstance(rx, int) and isinstance(ry, int):
        if 0 <= rx < width and 0 <= ry < height:
            known_points.append((rx, ry))

    return known_points


def find_crop_bounds(payload, margin=3):
    """
    Dynamically crop around known cells.
    """
    width, height = get_map_dimensions(payload)
    known_points = find_known_points(payload)

    if not known_points:
        return 0, max(0, width - 1), 0, max(0, height - 1)

    min_x = max(0, min(x for x, y in known_points) - margin)
    max_x = min(width - 1, max(x for x, y in known_points) + margin)

    min_y = max(0, min(y for x, y in known_points) - margin)
    max_y = min(height - 1, max(y for x, y in known_points) + margin)

    return min_x, max_x, min_y, max_y


def build_array(payload, crop=True, margin=3):
    width, height = get_map_dimensions(payload)
    grid = payload.get("grid", [])
    robot = payload.get("robot", {})

    if crop:
        min_x, max_x, min_y, max_y = find_crop_bounds(payload, margin)
    else:
        min_x, max_x, min_y, max_y = 0, width - 1, 0, height - 1

    cropped_width = max(1, max_x - min_x + 1)
    cropped_height = max(1, max_y - min_y + 1)

    arr = np.zeros((cropped_height, cropped_width), dtype=int)

    for global_y in range(min_y, max_y + 1):
        for global_x in range(min_x, max_x + 1):
            local_x = global_x - min_x
            local_y = global_y - min_y

            if global_y >= len(grid):
                arr[local_y, local_x] = STATE_VALUES["unknown"]
                continue

            row = grid[global_y]

            if global_x >= len(row):
                arr[local_y, local_x] = STATE_VALUES["unknown"]
                continue

            cell = row[global_x]
            state = cell.get("state", "unknown")
            visited = cell.get("visited", False)

            if state == "obstacle":
                arr[local_y, local_x] = STATE_VALUES["obstacle"]
            elif visited:
                arr[local_y, local_x] = STATE_VALUES["visited"]
            elif state == "free":
                arr[local_y, local_x] = STATE_VALUES["free"]
            else:
                arr[local_y, local_x] = STATE_VALUES["unknown"]

    rx = robot.get("x")
    ry = robot.get("y")

    if isinstance(rx, int) and isinstance(ry, int):
        if min_x <= rx <= max_x and min_y <= ry <= max_y:
            arr[ry - min_y, rx - min_x] = STATE_VALUES["robot"]

    crop_info = {
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
        "width": cropped_width,
        "height": cropped_height,
        "full_width": width,
        "full_height": height,
    }

    return arr, crop_info


def render_map(input_file, output_file, crop=True, margin=3):
    payload = load_map(input_file)
    arr, crop_info = build_array(payload, crop=crop, margin=margin)

    robot = payload.get("robot", {})
    robot_x = robot.get("x", "?")
    robot_y = robot.get("y", "?")
    robot_heading = robot.get("heading", "?")
    step = payload.get("step", None)

    cmap = ListedColormap([
        "#bdbdbd",  # unknown
        "#ffffff",  # free
        "#4da3ff",  # visited
        "#000000",  # obstacle
        "#ff3333",  # robot
    ])

    norm = BoundaryNorm([-0.5, 0.5, 1.5, 2.5, 3.5, 4.5], cmap.N)

    height, width = arr.shape

    # Dynamic figure size based on cropped output.
    fig_width = max(6, min(16, width * 0.45))
    fig_height = max(6, min(16, height * 0.45))

    fig, ax = plt.subplots(figsize=(fig_width, fig_height))
    ax.imshow(arr, cmap=cmap, norm=norm, origin="upper")

    title = (
        f"Robot Grid Map | Robot @ ({robot_x}, {robot_y}) "
        f"heading {robot_heading}"
    )

    if step is not None:
        title += f" | Step {step}"

    if crop:
        title += (
            f"\nCropped: x={crop_info['min_x']}..{crop_info['max_x']}, "
            f"y={crop_info['min_y']}..{crop_info['max_y']} "
            f"from full {crop_info['full_width']}x{crop_info['full_height']}"
        )
    else:
        title += f"\nFull map: {crop_info['full_width']}x{crop_info['full_height']}"

    ax.set_title(title)

    # Axis labels use real grid coordinates, not cropped local coordinates.
    x_labels = list(range(crop_info["min_x"], crop_info["max_x"] + 1))
    y_labels = list(range(crop_info["min_y"], crop_info["max_y"] + 1))

    ax.set_xticks(range(width))
    ax.set_yticks(range(height))
    ax.set_xticklabels(x_labels, fontsize=8, rotation=90)
    ax.set_yticklabels(y_labels, fontsize=8)

    ax.set_xticks(np.arange(-0.5, width, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, height, 1), minor=True)
    ax.grid(which="minor", linewidth=0.5)
    ax.tick_params(which="minor", bottom=False, left=False)

    # Draw heading arrow on robot cell.
    rx = robot.get("x")
    ry = robot.get("y")
    heading_symbol = HEADING_SYMBOLS.get(robot_heading, "?")

    if isinstance(rx, int) and isinstance(ry, int):
        local_rx = rx - crop_info["min_x"]
        local_ry = ry - crop_info["min_y"]

        if 0 <= local_rx < width and 0 <= local_ry < height:
            ax.text(
                local_rx,
                local_ry,
                heading_symbol,
                ha="center",
                va="center",
                fontsize=18,
                fontweight="bold",
                color="white",
            )

    legend_items = [
        Patch(facecolor="#bdbdbd", edgecolor="black", label="Unknown"),
        Patch(facecolor="#ffffff", edgecolor="black", label="Free"),
        Patch(facecolor="#4da3ff", edgecolor="black", label="Visited"),
        Patch(facecolor="#000000", edgecolor="black", label="Obstacle"),
        Patch(facecolor="#ff3333", edgecolor="black", label="Robot"),
    ]

    ax.legend(handles=legend_items, loc="upper right", bbox_to_anchor=(1.35, 1.0))

    plt.tight_layout()
    plt.savefig(output_file, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="robot_grid_map.json")
    parser.add_argument("--output", default="robot_grid_map.png")
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--interval", type=float, default=1.0)
    parser.add_argument("--margin", type=int, default=4)
    parser.add_argument(
        "--full",
        action="store_true",
        help="Show the full map instead of cropped explored area",
    )

    args = parser.parse_args()

    input_path = Path(args.input)
    crop = not args.full

    if args.watch:
        print(f"Watching {args.input}. Press Ctrl+C to stop.")

        while True:
            try:
                if input_path.exists():
                    render_map(
                        args.input,
                        args.output,
                        crop=crop,
                        margin=args.margin,
                    )
                    print(f"Updated {args.output}")
                else:
                    print(f"Waiting for {args.input}...")
            except json.JSONDecodeError:
                print("Map file is being written. Retrying...")
            except Exception as e:
                print(f"Visualization error: {e}")

            time.sleep(args.interval)

    else:
        render_map(
            args.input,
            args.output,
            crop=crop,
            margin=args.margin,
        )
        print(f"Saved {args.output}")


if __name__ == "__main__":
    main()