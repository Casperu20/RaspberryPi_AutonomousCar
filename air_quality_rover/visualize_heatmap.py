#!/usr/bin/env python3
"""
Visualize temperature or humidity heatmap from robot_grid_map.json.

Usage:
    python3 visualize_heatmap.py --field temperature
    python3 visualize_heatmap.py --field humidity

Live update:
    python3 visualize_heatmap.py --field temperature --watch
"""

import argparse
import json
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_map(path):
    with open(path, "r") as f:
        return json.load(f)


def get_map_dimensions(payload):
    grid = payload.get("grid", [])

    height_from_grid = len(grid)
    width_from_grid = len(grid[0]) if height_from_grid > 0 else 0

    width = payload.get("width", width_from_grid)
    height = payload.get("height", height_from_grid)

    return width, height


def find_known_bounds(payload, field, margin=3):
    width, height = get_map_dimensions(payload)
    grid = payload.get("grid", [])
    robot = payload.get("robot", {})

    points = []

    for y in range(min(height, len(grid))):
        row = grid[y]

        for x in range(min(width, len(row))):
            cell = row[x]
            value = cell.get(field)

            if value is not None:
                points.append((x, y))

    rx = robot.get("x")
    ry = robot.get("y")

    if isinstance(rx, int) and isinstance(ry, int):
        points.append((rx, ry))

    if not points:
        return 0, width - 1, 0, height - 1

    min_x = max(0, min(x for x, y in points) - margin)
    max_x = min(width - 1, max(x for x, y in points) + margin)
    min_y = max(0, min(y for x, y in points) - margin)
    max_y = min(height - 1, max(y for x, y in points) + margin)

    return min_x, max_x, min_y, max_y


def build_heatmap_array(payload, field, margin=3, full=False):
    width, height = get_map_dimensions(payload)
    grid = payload.get("grid", [])

    if full:
        min_x, max_x, min_y, max_y = 0, width - 1, 0, height - 1
    else:
        min_x, max_x, min_y, max_y = find_known_bounds(payload, field, margin)

    cropped_width = max_x - min_x + 1
    cropped_height = max_y - min_y + 1

    arr = np.full((cropped_height, cropped_width), np.nan)

    for global_y in range(min_y, max_y + 1):
        for global_x in range(min_x, max_x + 1):
            local_x = global_x - min_x
            local_y = global_y - min_y

            if global_y >= len(grid):
                continue

            row = grid[global_y]

            if global_x >= len(row):
                continue

            value = row[global_x].get(field)

            if value is not None:
                arr[local_y, local_x] = float(value)

    crop_info = {
        "min_x": min_x,
        "max_x": max_x,
        "min_y": min_y,
        "max_y": max_y,
    }

    return arr, crop_info


def render_heatmap(input_file, output_file, field, margin=3, full=False):
    payload = load_map(input_file)
    arr, crop_info = build_heatmap_array(
        payload,
        field=field,
        margin=margin,
        full=full,
    )

    robot = payload.get("robot", {})
    robot_x = robot.get("x")
    robot_y = robot.get("y")
    robot_heading = robot.get("heading", "?")

    height, width = arr.shape

    fig_width = max(6, min(16, width * 0.5))
    fig_height = max(6, min(16, height * 0.5))

    fig, ax = plt.subplots(figsize=(fig_width, fig_height))

    cmap = plt.get_cmap("inferno").copy()
    cmap.set_bad(color="#bdbdbd")

    image = ax.imshow(arr, cmap=cmap, origin="upper")

    cbar = plt.colorbar(image, ax=ax)

    if field == "temperature":
        cbar.set_label("Temperature (C)")
        title_field = "Temperature"
    elif field == "humidity":
        cbar.set_label("Humidity (%)")
        title_field = "Humidity"
    else:
        cbar.set_label(field)
        title_field = field

    ax.set_title(
        f"{title_field} Heatmap | "
        f"Robot @ ({robot_x}, {robot_y}) heading {robot_heading}\n"
        f"View: x={crop_info['min_x']}..{crop_info['max_x']}, "
        f"y={crop_info['min_y']}..{crop_info['max_y']}"
    )

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

    if isinstance(robot_x, int) and isinstance(robot_y, int):
        local_rx = robot_x - crop_info["min_x"]
        local_ry = robot_y - crop_info["min_y"]

        if 0 <= local_rx < width and 0 <= local_ry < height:
            ax.scatter(local_rx, local_ry, s=160, marker="o")
            ax.text(
                local_rx,
                local_ry,
                "R",
                ha="center",
                va="center",
                fontsize=10,
                fontweight="bold",
                color="white",
            )

    plt.tight_layout()
    plt.savefig(output_file, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", default="robot_grid_map.json")
    parser.add_argument("--field", default="temperature", choices=["temperature", "humidity", "air_quality"])
    parser.add_argument("--output", default=None)
    parser.add_argument("--watch", action="store_true")
    parser.add_argument("--interval", type=float, default=2.0)
    parser.add_argument("--margin", type=int, default=4)
    parser.add_argument("--full", action="store_true")

    args = parser.parse_args()

    output_file = args.output

    if output_file is None:
        output_file = f"{args.field}_heatmap.png"

    input_path = Path(args.input)

    if args.watch:
        print(f"Watching {args.input}. Press Ctrl+C to stop.")

        while True:
            try:
                if input_path.exists():
                    render_heatmap(
                        args.input,
                        output_file,
                        field=args.field,
                        margin=args.margin,
                        full=args.full,
                    )
                    print(f"Updated {output_file}")
                else:
                    print(f"Waiting for {args.input}...")
            except json.JSONDecodeError:
                print("Map file is being written. Retrying...")
            except Exception as e:
                print(f"Heatmap error: {e}")

            time.sleep(args.interval)

    else:
        render_heatmap(
            args.input,
            output_file,
            field=args.field,
            margin=args.margin,
            full=args.full,
        )
        print(f"Saved {output_file}")


if __name__ == "__main__":
    main()