import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import argparse
import itertools

def plot_problem_and_results(problem_file: str, result_files: list[str]):
    # --- Load YAML problem file ---
    with open(problem_file, "r") as f:
        problem = yaml.safe_load(f)

    # --- Extract environment ---
    env = problem["environment"]
    min_x, min_y = env["min"]
    max_x, max_y = env["max"]

    obstacles = env.get("obstacles", [])

    # --- Plot ---
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.set_xlim(min_x, max_x)
    ax.set_ylim(min_y, max_y)

    # Draw obstacles as rectangles
    for obs in obstacles:
        if obs["type"] == "box":
            cx, cy = obs["center"]
            sx, sy = obs["size"]
            rect = patches.Rectangle(
                (cx - sx / 2, cy - sy / 2), sx, sy,
                linewidth=1, edgecolor="black", facecolor="gray"
            )
            ax.add_patch(rect)

    # --- Plot results with different colors ---
    colors = itertools.cycle(plt.cm.tab10.colors)  # cycle through 10 distinct colors
    for idx, (result_file, color) in enumerate(zip(result_files, colors)):
        with open(result_file, "r") as f:
            result = yaml.safe_load(f)

        states = result.get("states", [])
        states_xy = [(s[0], s[1]) for s in states]

        if states_xy:
            xs, ys = zip(*states_xy)
            ax.plot(xs, ys, marker="o", markersize=4, linewidth=1.2,
                    c=color, label=f"Robot {idx+1}")

        # --- Draw start & goal for this robot ---
        if idx < len(problem["robots"]):  # safeguard
            robot = problem["robots"][idx]
            start = robot["start"][:2]
            goal = robot["goal"][:2]

            ax.scatter(*start, c=color, s=70, marker="o", edgecolor="black", zorder=5)
            ax.scatter(*goal, c=color, s=70, marker="x", linewidths=2, zorder=5)

    ax.set_aspect("equal", adjustable="box")
    ax.legend()
    plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("env", help="input file containing map")
    parser.add_argument("--results", nargs="+", help="one or more result files containing solutions")

    args = parser.parse_args()

    plot_problem_and_results(args.env, args.results)

if __name__ == "__main__":
    main()
