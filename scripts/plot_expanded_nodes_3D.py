import numpy as np
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
import argparse
import yaml
import os
from pathlib import Path
import itertools

# color cycle per agent
COLOR_CYCLE = [
    0xFF0000,  # red
    0x00FF00,  # green
    0x0000FF,  # blue
    0xFF00FF,  # magenta
    0x00FFFF,  # cyan
    0xFFFF00,  # yellow
    0xFFA500,  # orange
    0x800080,  # purple
]

def visualize(env_file, result_files):
    vis = meshcat.Visualizer()

    vis["/Cameras/default"].set_transform(
        tf.translation_matrix([0, 0, 0]).dot(
            tf.euler_matrix(0, np.radians(-30), np.radians(90)))
    )
    
    # name of the saved file
    file_name = "states_" + os.path.basename(env_file).split('.')[0] + ".html"

    # load environment (to get robot start/goal)
    with open(env_file) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    robots = data["robots"]
    # -- Plot obstacles
    obstacles = data["environment"]["obstacles"]
    for k, obs in enumerate(obstacles):
      center = obs["center"]
      size = obs["size"]
      obs_type = obs["type"]
      if (obs_type == 'octomap'):
         octomap_stl = obs["octomap_stl"]
         vis[f"Obstacle{k}"].set_object(g.StlMeshGeometry.from_file(octomap_stl), g.MeshLambertMaterial(opacity=0.8, color=0xFFFFFF)) 
      elif (obs_type == 'box'):
        vis[f"Obstacle{k}"].set_object(g.Mesh(g.Box(size)))
        vis[f"Obstacle{k}"].set_transform(tf.translation_matrix(center))
      else:
         print("Unknown Obstacle type!")
    # --- Plot start & goal spheres (one per robot) ---
    for r, robot in enumerate(robots):
        color = COLOR_CYCLE[r % len(COLOR_CYCLE)]
        start = robot["start"][:3]
        goal = robot["goal"][:3]

        vis[f"agent{r}_start"].set_object(
            g.Mesh(g.Sphere(0.04), g.MeshLambertMaterial(color=color)))
        vis[f"agent{r}_start"].set_transform(tf.translation_matrix(start))

        vis[f"agent{r}_goal"].set_object(g.Box([0.05, 0.05, 0.05]), g.MeshLambertMaterial(opacity=0.4, color=color))
        vis[f"agent{r}_goal"].set_transform(tf.translation_matrix(goal))

    # --- Loop through result files ---
    colors = itertools.cycle(COLOR_CYCLE)  # infinite color cycle

    for file_idx, (result_file, color) in enumerate(zip(result_files, colors)):
        with open(result_file, "r") as f:
            result = yaml.safe_load(f)

        states = result.get("states", [])
        if not states:
            continue  # skip empty files

        for k, s in enumerate(states):
            vis[f"file{file_idx}_state{k}"].set_object(
                g.Mesh(g.Sphere(0.01), g.MeshLambertMaterial(color=color))
            )
            vis[f"file{file_idx}_state{k}"].set_transform(
                tf.translation_matrix(s[:3])
            )


    # --- Save html ---
    result_folder = Path(result_files[0]).resolve().parent
    res = vis.static_html()
    with open(result_folder / file_name, "w") as f:
        f.write(res)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("env", help="input file containing map")
    parser.add_argument('--results', nargs='+', help="one or more result files containing solutions")
    args = parser.parse_args()

    visualize(args.env, args.results)

if __name__ == "__main__":
    main()
