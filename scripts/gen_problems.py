import yaml
import numpy as np
# does not run the planner to check if the problem is valid/feasible
def gen_env(min, max, N, filename):
    """Generate environment with N robots and no obstacles."""
    r = dict()
    r["environment"] = {
        "min": min.tolist(),
        "max": max.tolist(),
        "obstacles": []  # no obstacles
    }
    r["robots"] = []

    while len(r["robots"]) < N:
        type = "unicycle1_v0"

        for _ in range(100):  # try up to 100 times to place robot
            start = np.random.uniform(
                [min[0] + 0.5, min[1] + 0.5, -np.pi],
                [max[0] - 0.5, max[1] - 0.5, np.pi]
            )
            goal = np.random.uniform(
                [min[0] + 0.5, min[1] + 0.5, -np.pi],
                [max[0] - 0.5, max[1] - 0.5, np.pi]
            )

            # require that goal is not too close to start
            if np.linalg.norm(start[:2] - goal[:2]) < 2:
                continue

            r["robots"].append({
                "type": type,
                "start": start.tolist(),
                "goal": goal.tolist()
            })
            break
        else:
            print(f"Could not place robot {len(r['robots'])} after 100 tries")

    with open(filename, "w") as f:
        yaml.dump(r, f)


def main():
    min = np.array([0,0])
    max = np.array([20,20])
    K = 1   # number of instances
    N = 10  # number of robots

    for k in range(K):
        filename = f"../example/test_n{N}_{k}_unicycle.yaml"
        gen_env(min, max, N, filename)


if __name__ == '__main__':
    main()
