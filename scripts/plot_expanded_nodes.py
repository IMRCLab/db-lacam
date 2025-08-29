import yaml
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import argparse

def plot_problem_and_result(problem_file: str, result_file: str):
    # --- Load YAML files ---
    with open(problem_file, "r") as f:
        problem = yaml.safe_load(f)

    with open(result_file, "r") as f:
        result = yaml.safe_load(f)

    # --- Extract environment ---
    env = problem["environment"]
    min_x, min_y = env["min"]
    max_x, max_y = env["max"]

    obstacles = env.get("obstacles", [])

    # --- Extract start and goal ---
    robot = problem["robots"][0]
    start = robot["start"][:2]  # only x, y
    goal = robot["goal"][:2]    # only x, y

    # --- Extract states from result ---
    states = result.get("states", [])
    states_xy = [(s[0], s[1]) for s in states]

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

    # Draw trajectory states (black circles)
    if states_xy:
        xs, ys = zip(*states_xy)
        ax.scatter(xs, ys, c="black", s=15, label="States")

    # Draw start & goal (red circles)
    ax.scatter(*start, c="red", s=60, marker="o", label="Start")
    ax.scatter(*goal, c="red", s=60, marker="x", label="Goal")

    ax.set_aspect("equal", adjustable="box")
    ax.legend()
    plt.show()

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("env", help="input file containing map")
  parser.add_argument("--result", help="output file containing solution")

  args = parser.parse_args()

  plot_problem_and_result(args.env, args.result)

if __name__ == "__main__":
  main()
