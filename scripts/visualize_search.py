import matplotlib.pyplot as plt
import yaml
import subprocess 
import pathlib
import argparse
from matplotlib.patches import Rectangle

def plot_expanded_trajs2(filename_env, filename_trajs, filename_result = None, filename_video = None):
    with open(filename_env) as env_file:
      env = yaml.safe_load(env_file)
    fig = plt.figure()
    ax = fig.add_subplot(111, aspect="equal")
    ax.set_xlim(env["environment"]["min"][0], env["environment"]["max"][0])
    ax.set_ylim(env["environment"]["min"][1], env["environment"]["max"][1])

    # Draw obstacles
    for obs in env["environment"].get("obstacles", []):
      if obs["type"] == "box":
          center = obs["center"]
          size = obs["size"]
          bottom_left = [center[0] - size[0]/2, center[1] - size[1]/2]
          rect = Rectangle(bottom_left, size[0], size[1], color="black", alpha=0.5)
          ax.add_patch(rect)
    plt.tick_params(
    top=False, bottom=False, left=False, right=False, labelleft=False, labelbottom=False)
    plt.axis("off")
    fig.tight_layout()
    # read trajs
    with open(filename_trajs) as motions_file:
        motions = yaml.safe_load(motions_file)
    trajs = motions["trajs"]
    starts = []
    N = len(trajs)
    fps, duration = 24, 100
    folder = f"../results/plot/" # maybe better way with not saving images
    pathlib.Path(folder).mkdir(parents=True,exist_ok=True)
    for i in range(0,N,1):
        states = trajs[i]["states"]
        X = [s[0] for s in states]
        Y = [s[1] for s in states]
        starts.append([states[0][0], states[0][1]])
        ax.plot(X, Y, color=".5", alpha=0.2)
        print("saving ", i)
        file = f"{folder}/fig_{i}.png"
        plt.savefig(file)
    if filename_result is not None:
        with open(filename_result, "r") as f:
            data_sol = yaml.safe_load(f)
        result = data_sol["result"][1]
        x_sol = [X[0] for X in result["states"]]
        y_sol = [X[1] for X in result["states"]]
        ax.plot(x_sol, y_sol, color="orange")
    ax.scatter([s[0] for s in starts], [s[1] for s in starts], s=2, color="k")
    plt.savefig(f"../results/plot/fig_{i+1}.png")
    if filename_video is not None:
        subprocess.call(["ffmpeg","-y","-r",str(fps),"-i", "../results/plot/fig_%d.png","-vcodec","mpeg4", "-qscale","5", "-r", str(fps), "../results/plot/" + filename_video])

def plot_expanded_trajs(
    filename_env,
    filenames_trajs,
    filename_result=None,
    filename_video=None
):
    with open(filename_env) as env_file:
        env = yaml.safe_load(env_file)

    fig = plt.figure()
    ax = fig.add_subplot(111, aspect="equal")

    ax.set_xlim(
        env["environment"]["min"][0],
        env["environment"]["max"][0]
    )
    ax.set_ylim(
        env["environment"]["min"][1],
        env["environment"]["max"][1]
    )

    # Draw obstacles
    for obs in env["environment"].get("obstacles", []):
        if obs["type"] == "box":
            center = obs["center"]
            size = obs["size"]

            bottom_left = [
                center[0] - size[0] / 2,
                center[1] - size[1] / 2
            ]

            rect = Rectangle(
                bottom_left,
                size[0],
                size[1],
                color="black",
                alpha=0.5
            )
            ax.add_patch(rect)

    # Show coordinate system
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    # Coordinate ticks
    ax.tick_params(
        top=False,
        bottom=True,
        left=True,
        right=False,
        labelleft=True,
        labelbottom=True
    )

    # Grid
    ax.grid(True, linestyle="--", alpha=0.4)

    # Keep equal scaling for x and y
    ax.set_aspect("equal", adjustable="box")

    # One distinct color per trajectory file
    colors = plt.cm.tab10.colors
    starts = []

    # Plot trajectories from all files on the same figure
    for file_idx, filename_trajs in enumerate(filenames_trajs):

        with open(filename_trajs) as motions_file:
            motions = yaml.safe_load(motions_file)

        trajs = motions["trajs"]
        color = colors[file_idx % len(colors)]

        for traj_idx, traj in enumerate(trajs):
            states = traj["states"]

            X = [s[0] for s in states]
            Y = [s[1] for s in states]

            starts.append([states[0][0], states[0][1]])

            ax.plot(
                X,
                Y,
                color=color,
                alpha=0.7,
                label=filename_trajs if traj_idx == 0 else None
            )

    # Start. Goal states
    robots = env["robots"]

    for robot_idx, robot in enumerate(robots):
        start = robot["start"]
        goal = robot["goal"]

        # Start position: circle
        ax.scatter(
            start[0],
            start[1],
            color=colors[robot_idx],
            marker="o",
            s=60,
            zorder=6
        )

        # Goal position: cross
        ax.scatter(
            goal[0],
            goal[1],
            color=colors[robot_idx],
            marker="x",
            s=70,
            linewidths=2,
            zorder=6
        )
    # Plot the final solution, if provided
    if filename_result is not None:
        with open(filename_result, "r") as f:
            data_sol = yaml.safe_load(f)

        result = data_sol["result"][1]

        x_sol = [s[0] for s in result["states"]]
        y_sol = [s[1] for s in result["states"]]

        ax.plot(
            x_sol,
            y_sol,
            color="orange",
            linewidth=2,
            label="Final solution"
        )

    # Draw all starting positions
    if starts:
        ax.scatter(
            [s[0] for s in starts],
            [s[1] for s in starts],
            s=5,
            color="black",
            zorder=3
        )

    # Show a legend identifying the trajectory files
    ax.legend(loc="best", fontsize=8)

    fig.tight_layout()

    if filename_video is None:
        plt.show()
    else:
        plt.savefig(filename_video, dpi=200, bbox_inches="tight")

    plt.close(fig)

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "env",
        help="Input file containing the map"
    )

    parser.add_argument(
        "--trajs",
        nargs="+",
        required=True,
        help="One or more files containing expanded trajectories"
    )

    parser.add_argument(
        "--result",
        help="File with the final solution"
    )

    parser.add_argument(
        "--video",
        help="Output image filename"
    )

    args = parser.parse_args()

    plot_expanded_trajs(
        args.env,
        args.trajs,
        args.result,
        args.video
    )


if __name__ == "__main__":
    main()