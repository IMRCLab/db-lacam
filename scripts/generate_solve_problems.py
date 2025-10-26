import yaml
import numpy as np
import os
import copy
import subprocess


def solve_problem(env_file, output_file, stats_file, conf_file):
    """Run db-LaCAM on the given environment file and save the result."""
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    out = subprocess.run([
        "./run_dblacam",
        "-i", env_file,
        "-o", output_file,
        "--stats", stats_file,
        "--cfg", conf_file,
        "-t", "1e4"
    ])
    return out.returncode == 0


def gen_random_env(min, max, obs_density, N, filename):
    """Generate a random environment with N robots and random obstacles."""
    r = {
        "environment": {"min": min.tolist(), "max": max.tolist(), "obstacles": []},
        "robots": []
    }

    # Random obstacles
    area = np.prod(max - min)
    filled_area = 0
    obs = []
    while filled_area < obs_density * area:
        size = np.random.normal([0.75, 0.75], 0.5)
        if np.any(size < 0.1):
            continue
        filled_area += np.prod(size)
        center = np.random.uniform(min + size / 2, max - size / 2)
        p1, p2 = center - size / 2, center + size / 2
        if any(
            p1[0] < o["center"][0] + o["size"][0]/2 and o["center"][0] - o["size"][0]/2 < p2[0] and
            p1[1] < o["center"][1] + o["size"][1]/2 and o["center"][1] - o["size"][1]/2 < p2[1]
            for o in obs
        ):
            continue
        obs.append({"type": "box", "center": center.tolist(), "size": size.tolist()})
    r["environment"]["obstacles"] = obs

    # Random robots
    while len(r["robots"]) < N:
        start = np.random.uniform([min[0]+0.5, min[1]+0.5, -np.pi], [max[0]-0.5, max[1]-0.5, np.pi])
        goal = np.random.uniform([min[0]+0.5, min[1]+0.5, -np.pi], [max[0]-0.5, max[1]-0.5, np.pi])
        if np.linalg.norm(start[:2] - goal[:2]) < 2:
            continue
        r["robots"].append({"type": "unicycle1_sphere_v0", "start": start.tolist(), "goal": goal.tolist()})

    with open(filename, "w") as f:
        yaml.dump(r, f)


def livelock_templates():
    """Return six base livelock configurations."""
    return [
        # Template 1
        {
            "environment": {
                "obstacles": [
                    {"center": [5.0, 6.75], "size": [6.0, 0.4], "type": "box"},
                    {"center": [5.0, 4.25], "size": [6.0, 0.4], "type": "box"},
                    {"center": [1.0, 5.0], "size": [0.4, 1.2], "type": "box"},
                    {"center": [9.0, 5.0], "size": [0.4, 1.2], "type": "box"}
                ]
            },
            "robots": [
                {"start": [2.0, 5.0, 0.0], "goal": [8.0, 5.0, 3.14]},
                {"start": [8.0, 5.0, 3.14], "goal": [2.0, 5.0, 0.0]}
            ]
        },
        # Template 2
        {
            "robots": [
                {"start": [3.0, 5.0, 0.0], "goal": [6.0, 5.0, 3.14]},
                {"start": [6.0, 5.0, 3.14], "goal": [3.0, 5.0, 0.0]},
                {"start": [5.0, 3.0, 1.57], "goal": [5.0, 6.0, -1.57]}
            ]
        },
        # Template 3
        {
            "robots": [
                {"start": [3.8, 3.8, 0.0], "goal": [6.2, 6.2, 3.14]},
                {"start": [6.2, 6.2, 3.14], "goal": [3.8, 3.8, 0.0]},
                {"start": [6.2, 3.8, 1.57], "goal": [3.8, 6.2, -1.57]}
            ]
        },
        # Template 4
        {
            # "environment": {"obstacles": [{"center": [4, 3], "size": [4, 2], "type": "box"}]},
            "robots": [
                {"start": [4.0, 5.0, 1.57], "goal": [1.0, 2.0, 3.14]},
                {"start": [2.0, 1.0, 3.14], "goal": [4.0, 5.0, 0.0]}
            ]
        },
        # Template 5
        {
            "environment": {"obstacles": [{"center": [5, 5], "size": [0.6, 3], "type": "box"}]},
            "robots": [
                {"start": [3, 5, 0], "goal": [7, 5, 3.14]},
                {"start": [7, 5, 3.14], "goal": [3, 5, 0]},
                {"start": [5, 3, 1.57], "goal": [5, 7, -1.57]}
            ]
        },
        # Template 6
        {
            "environment": {
                "obstacles": [
                    {"center": [5.0, 5.0], "size": [0.6, 3.0], "type": "box"},
                    {"center": [5.0, 5.0], "size": [3.0, 0.6], "type": "box"}
                ]
            },
            "robots": [
                {"start": [3.0, 5.0, 0.0], "goal": [7.0, 5.0, 3.14]},
                {"start": [7.0, 5.0, 3.14], "goal": [3.0, 5.0, 0.0]},
                {"start": [5.0, 3.0, 1.57], "goal": [5.0, 7.0, -1.57]}
            ]
        }
    ]


def gen_livelock_variant(template, min, max, N):
    """Generate a perturbed livelock scenario from a base template."""
    env = {"environment": {"min": min.tolist(), "max": max.tolist()}, "robots": []}

    # Perturb obstacles
    obs_list = []
    if "environment" in template and "obstacles" in template["environment"]:
        for o in template["environment"]["obstacles"]:
            center = np.array(o["center"]) + np.random.uniform(-0.5, 0.5, 2)
            size = np.array(o["size"]) + np.random.uniform(-0.2, 0.2, 2)
            obs_list.append({"center": center.tolist(), "size": np.maximum(size, 0.2).tolist(), "type": "box"})
    env["environment"]["obstacles"] = obs_list

    # Perturb livelock robots
    for r in template["robots"]:
        start = np.array(r["start"]) + np.random.uniform([-0.3, -0.3, -0.3], [0.3, 0.3, 0.3])
        goal = np.array(r["goal"]) + np.random.uniform([-0.3, -0.3, -0.3], [0.3, 0.3, 0.3])
        env["robots"].append({"type": "unicycle1_sphere_v0", "start": start.tolist(), "goal": goal.tolist()})

    # Add random robots (non-livelock)
    while len(env["robots"]) < N:
        start = np.random.uniform([min[0]+0.5, min[1]+0.5, -np.pi], [max[0]-0.5, max[1]-0.5, np.pi])
        goal = np.random.uniform([min[0]+0.5, min[1]+0.5, -np.pi], [max[0]-0.5, max[1]-0.5, np.pi])
        if np.linalg.norm(start[:2] - goal[:2]) < 2:
            continue
        env["robots"].append({"type": "unicycle1_sphere_v0", "start": start.tolist(), "goal": goal.tolist()})

    return env


def main():
    min = np.array([0, 0])
    max = np.array([10, 10])
    obs_density = 0
    K = 200
    N = 8
    include_livelock = True

    folder = "../example/livelock2/"
    os.makedirs(folder, exist_ok=True)

    templates = livelock_templates()

    for k in range(K):
        base_name = f"livelock_n{N}_{k}_unicycle_sphere"
        env_file = os.path.join(folder, f"{base_name}.yaml")   

        if include_livelock:
            t = np.random.choice(templates)
            env = gen_livelock_variant(t, min, max, N)
            with open(env_file, "w") as f:
                yaml.dump(env, f)
        else:
            gen_random_env(min, max, obs_density / 100.0, N, env_file)

        print(f"Generated {'livelock' if include_livelock else 'random'} env: {env_file}")


if __name__ == "__main__":
    main()
