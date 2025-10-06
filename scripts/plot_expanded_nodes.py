import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import argparse
from matplotlib.lines import Line2D

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

    robot_colors = ["#4477AA", "#CCBB44", "#66CCEE", "#d62728",
                    "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
                    "#bcbd22", "#17becf"]
    
    legend_handles = []
    for idx, result_file in enumerate(result_files):
        with open(result_file, "r") as f:
            result = yaml.safe_load(f)

        states = result.get("states", [])
        states_xy = [(s[0], s[1]) for s in states]

        color = robot_colors[idx % len(robot_colors)]

        if states_xy:
            xs, ys = zip(*states_xy)
            ax.plot(xs, ys, marker="o", markersize=4, linestyle="None", c=color)

        # --- Draw start & goal for this robot ---
        if idx < len(problem["robots"]):  # safeguard
            robot = problem["robots"][idx]
            start = robot["start"][:2]
            goal = robot["goal"][:2]

            # Start: circle with black edge
            ax.scatter(*start, c=color, s=260, marker="*",
                       edgecolor="black", linewidths=1.5, zorder=5)

            # Goal: X marker
            ax.scatter(*goal, c=color, s=260, marker="X",
                       edgecolor="black", linewidths=1.5, zorder=5)
            ax.plot(xs, ys, marker="o", markersize=4, linestyle="None", c=color)

            legend_handles.append(Line2D([0], [0], marker="o", color="w",
                                        markerfacecolor=color, markersize=10,
                                        label=f"Robot {idx+1}"))

    ax.set_aspect("equal", adjustable="box")
    # ax.legend(handles=legend_handles, fontsize=14, loc='upper right')
    # ax.tick_params(axis="both", which="major", labelsize=14)
    # invisible ticks
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    plt.savefig("../results/est_search.pdf", format="pdf", bbox_inches="tight")
    plt.show()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("env", help="input file containing map")
    parser.add_argument("--results", nargs="+", help="one or more result files containing solutions")

    args = parser.parse_args()

    plot_problem_and_results(args.env, args.results)

if __name__ == "__main__":
    main()
