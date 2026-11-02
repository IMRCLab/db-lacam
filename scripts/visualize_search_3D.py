import argparse
import yaml
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from itertools import product


def plot_box(ax, center, size):
    """Draw a 3D box obstacle."""
    cx, cy, cz = center
    sx, sy, sz = size

    vertices = [
        [cx - sx/2, cy - sy/2, cz - sz/2],
        [cx + sx/2, cy - sy/2, cz - sz/2],
        [cx + sx/2, cy + sy/2, cz - sz/2],
        [cx - sx/2, cy + sy/2, cz - sz/2],
        [cx - sx/2, cy - sy/2, cz + sz/2],
        [cx + sx/2, cy - sy/2, cz + sz/2],
        [cx + sx/2, cy + sy/2, cz + sz/2],
        [cx - sx/2, cy + sy/2, cz + sz/2],
    ]

    faces = [
        [vertices[i] for i in [0, 1, 2, 3]],
        [vertices[i] for i in [4, 5, 6, 7]],
        [vertices[i] for i in [0, 1, 5, 4]],
        [vertices[i] for i in [2, 3, 7, 6]],
        [vertices[i] for i in [1, 2, 6, 5]],
        [vertices[i] for i in [0, 3, 7, 4]],
    ]

    box = Poly3DCollection(
        faces,
        facecolors="black",
        edgecolors="black",
        alpha=0.25
    )

    ax.add_collection3d(box)


def set_axes_equal(ax):
    """Set equal scaling for all three axes."""
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])

    x_middle = sum(x_limits) / 2
    y_middle = sum(y_limits) / 2
    z_middle = sum(z_limits) / 2

    radius = 0.5 * max(x_range, y_range, z_range)

    ax.set_xlim3d([x_middle - radius, x_middle + radius])
    ax.set_ylim3d([y_middle - radius, y_middle + radius])
    ax.set_zlim3d([z_middle - radius, z_middle + radius])


def plot_expanded_trajs(
    filename_env,
    filenames_trajs,
    filename_result=None,
    filename_video=None
):
    with open(filename_env) as env_file:
        env = yaml.safe_load(env_file)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # Environment boundaries
    env_min = env["environment"]["min"]
    env_max = env["environment"]["max"]

    ax.set_xlim(env_min[0], env_max[0])
    ax.set_ylim(env_min[1], env_max[1])
    ax.set_zlim(env_min[2], env_max[2])

    # Draw obstacles
    for obs in env["environment"].get("obstacles", []):
        if obs["shape"]["type"] == "box":
            plot_box(
                ax,
                obs["center"],
                obs["shape"]["size"]
            )

    # Coordinate system
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")

    ax.set_title("Expanded 3D Trajectories")

    # Trajectory colors
    colors = plt.cm.tab10.colors
    starts = []

    # Plot trajectories
    for file_idx, filename_trajs in enumerate(filenames_trajs):

        with open(filename_trajs) as motions_file:
            motions = yaml.safe_load(motions_file)

        trajs = motions["trajs"]
        color = colors[file_idx % len(colors)]

        for traj_idx, traj in enumerate(trajs):
            states = traj["states"]

            X = [s[0] for s in states]
            Y = [s[1] for s in states]
            Z = [s[2] for s in states]

            starts.append([X[0], Y[0], Z[0]])

            ax.plot(
                X, Y, Z,
                color=color,
                alpha=0.7,
                label=filename_trajs if traj_idx == 0 else None
            )

    # Start and goal positions
    robots = env["robots"]

    for robot_idx, robot in enumerate(robots):
        start = robot["start"]
        goal = robot["goal"]

        color = colors[robot_idx % len(colors)]

        ax.scatter(
            start[0], start[1], start[2],
            color=color,
            marker="o",
            s=60,
            zorder=6
        )

        ax.scatter(
            goal[0], goal[1], goal[2],
            color=color,
            marker="x",
            s=70,
            linewidths=2,
            zorder=6
        )

    # Final solution
    if filename_result is not None:
        with open(filename_result, "r") as f:
            data_sol = yaml.safe_load(f)

        result = data_sol["result"][1]

        states = result["states"]

        x_sol = [s[0] for s in states]
        y_sol = [s[1] for s in states]
        z_sol = [s[2] for s in states]

        ax.plot(
            x_sol,
            y_sol,
            z_sol,
            color="orange",
            linewidth=2.5,
            label="Final solution"
        )

    # Starting positions of expanded trajectories
    if starts:
        ax.scatter(
            [s[0] for s in starts],
            [s[1] for s in starts],
            [s[2] for s in starts],
            s=5,
            color="black",
            zorder=3
        )

    # Equal axis scaling
    set_axes_equal(ax)

    ax.legend(loc="upper left", fontsize=8)

    fig.tight_layout()

    if filename_video is None:
        plt.show()
    else:
        plt.savefig(filename_video, dpi=200, bbox_inches="tight")

    plt.close(fig)


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("env", help="Input file containing the map")

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