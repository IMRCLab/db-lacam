import yaml
import numpy as np
import tempfile
import subprocess
import copy

def check_problem(cfg):
    with tempfile.TemporaryDirectory() as tmpdirname:
        print('created temporary directory', tmpdirname)

        # inflate the obstacles so that robots aren't too close
        cfg_copy = copy.deepcopy(cfg)
        for o in cfg_copy["environment"]["obstacles"]:
            o["size"][0] += 0.1
            o["size"][1] += 0.1

        with open(tmpdirname + "/env.yaml", "w") as f:
            yaml.dump(cfg_copy, f)

        result = {
            "result":
            [
                {
                    "states": [r["start"], r["goal"]],
                    "actions": [[0,0]]
                }
                for r in cfg["robots"]
            ]
        }
        print(result)
        with open(tmpdirname + "/result.yaml", "w") as f:
            yaml.dump(result, f)

        out = subprocess.run(["./run_dblacam",
                    "-i", tmpdirname + "/env.yaml",
                    "-o", "../results/result.yaml",
                    "--stats", "../result/test_stats.yaml",
                    "--cfg", "../example/algorithms.yaml",
                    "-t" , str(1e6)])
        return out.returncode == 0

import numpy as np
import yaml

def is_in_obstacle(state_xy, obstacles):
    """ Check if a given (x, y) state is inside any of the axis-aligned box obstacles. """
    for o in obstacles:
        center = np.array(o["center"])
        size = np.array(o["size"])
        p1 = center - size / 2
        p2 = center + size / 2
        if p1[0] <= state_xy[0] <= p2[0] and p1[1] <= state_xy[1] <= p2[1]:
            return True
    return False

def gen_env(min, max, obs_density, N, filename):
    r = dict()
    r["environment"] = dict()
    r["environment"]["min"] = min.tolist()
    r["environment"]["max"] = max.tolist()

    area = np.prod(max - min)
    print("Total area:", area)

    filled_area = 0
    obs = []
    while filled_area < obs_density * area:
        size = np.random.normal([0.5, 0.75], 0.5)  # Increased mean size
        if np.any(size < 0.2):
            continue
        center = np.random.uniform(min + size / 2, max - size / 2)
        p1 = center - size / 2
        p2 = center + size / 2

        # Check for collision with existing obstacles
        collision = False
        for o in obs:
            p1o = np.array(o["center"]) - np.array(o["size"]) / 2
            p2o = np.array(o["center"]) + np.array(o["size"]) / 2
            if p1[0] < p2o[0] and p1o[0] < p2[0] and p1[1] < p2o[1] and p1o[1] < p2[1]:
                collision = True
                break
        if collision:
            continue

        filled_area += np.prod(size)
        obs.append({
            "type": "box",
            "center": center.tolist(),
            "size": size.tolist()
        })

    r["environment"]["obstacles"] = obs
    r["robots"] = []

    while len(r["robots"]) < N:
        type = str(np.random.choice(["unicycle1_v0"]))  # can add more types if needed

        for _ in range(100):  # attempt multiple times before giving up
            start = np.random.uniform([min[0] + 0.5, min[1] + 0.5, -np.pi], [max[0] - 0.5, max[1] - 0.5, np.pi])
            goal = np.random.uniform([min[0] + 0.5, min[1] + 0.5, -np.pi], [max[0] - 0.5, max[1] - 0.5, np.pi])

            if np.linalg.norm(start[:2] - goal[:2]) < 2:
                continue
            if is_in_obstacle(start[:2], obs) or is_in_obstacle(goal[:2], obs):
                continue

            r["robots"].append({
                "type": type,
                "start": start.tolist(),
                "goal": goal.tolist()
            })

            if not check_problem(r):  # assuming check_problem validates path feasibility
                r["robots"].pop()
            else:
                break
        else:
            print(f"Could not place robot {len(r['robots'])} after 100 tries")

    with open(filename, "w") as f:
        yaml.dump(r, f)


def main():
    min = np.array([0,0])
    max = np.array([20,20])
    obs_density = 1 # percent
    K = 1 # num instances

    for N in [20]: # 4, 8, 12
        for k in range(K):
            filename = "../example/test_p{}_n{}_{}_unicycle.yaml".format(obs_density, N, k)
            gen_env(min, max, obs_density / 100.0, N, filename)

if __name__ == '__main__':
    main()