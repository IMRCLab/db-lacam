import yaml
import numpy as np
import matplotlib.pyplot as plt


def plot_all_motions(yaml_file):
    with open(yaml_file, "r") as f:
        data = yaml.safe_load(f)

    # YAML contains a list of motion primitives
    if isinstance(data, dict):
        data = [data]

    plt.figure(figsize=(10, 8))

    for i, traj in enumerate(data):
        states = np.asarray(traj["states"], dtype=float)

        # State format: [x, y, vx, vy]
        x = states[:, 0]
        y = states[:, 1]

        # Plot the entire XY motion
        plt.plot(x, y, linewidth=0.8, alpha=0.5)

        # Mark start and goal
        plt.scatter(x[0], y[0], s=8, marker="o")
        plt.scatter(x[-1], y[-1], s=8, marker="x")

    plt.xlabel("x")
    plt.ylabel("y")
    plt.title(f"All Motion Primitives ({len(data)} trajectories)")
    plt.axis("equal")
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


# Example
plot_all_motions("/home/akmarak-laptop/IMRC/mrmp-benchmark/planners/dbLaCAM/db-lacam/motion_primitives/integrator2_2d_v0/500/spread/my_motions.bin.im.bin.sp.bin.yaml")