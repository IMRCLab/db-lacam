import yaml
import numpy as np
import random

def gen_env(min_bounds, max_bounds, N, robot_types, filename):
    """Generate environment with N heterogeneous robots and no obstacles."""
    r = {
        "environment": {
            "min": min_bounds.tolist(),
            "max": max_bounds.tolist(),
            "obstacles": []
        },
        "robots": []
    }

    # Randomly assign how many robots of each type (sum = N)
    counts = np.random.multinomial(N, [1/len(robot_types)] * len(robot_types))
    print(f"Distribution: {dict(zip(robot_types, counts))}")

    for robot_type, count in zip(robot_types, counts):
        for _ in range(count):
            for _ in range(100):  # try up to 100 times to place robot
                if robot_type == "unicycle1_3d_v0":
                    start = np.random.uniform([min_bounds[0]+0.5, min_bounds[1]+0.5, 0, -np.pi], [max_bounds[0]-0.5, max_bounds[1]-0.5, 0, np.pi])
                    goal = np.random.uniform([min_bounds[0]+0.5, min_bounds[1]+0.5, 0, -np.pi], [max_bounds[0]-0.5, max_bounds[1]-0.5, 0, np.pi])
                    if np.linalg.norm(start[:3] - goal[:3]) < 2:
                        continue
                if robot_type == "integrator2_3d_v0":
                    start = np.random.uniform([min_bounds[0]+0.5, min_bounds[1]+0.5, min_bounds[2]+0.5, 0, 0, 0], [max_bounds[0]-0.5, max_bounds[1]-0.5, max_bounds[2]-0.5, 0, 0, 0])
                    goal = np.random.uniform([min_bounds[0]+0.5, min_bounds[1]+0.5, min_bounds[2]+0.5, 0, 0, 0], [max_bounds[0]-0.5, max_bounds[1]-0.5, max_bounds[2]-0.5, 0, 0, 0])

                r["robots"].append({
                    "type": robot_type,
                    "start": start.tolist(),
                    "goal": goal.tolist()
                })
                break
            else:
                print(f"Could not place {robot_type} after 100 tries")

    with open(filename, "w") as f:
        yaml.dump(r, f, sort_keys=False)


def main():
    min_bounds = np.array([0, 0, 0])
    max_bounds = np.array([5, 5, 1])
    K = 1   # number of instances
    N = 10  # number of robots

    # Provide your list of heterogeneous robot dynamics here:
    robot_types = ["unicycle1_3d_v0", "integrator2_3d_v0"]

    for k in range(K):
        filename = f"../example/test_n{N}_{k}_hetero.yaml"
        gen_env(min_bounds, max_bounds, N, robot_types, filename)


if __name__ == '__main__':
    main()
