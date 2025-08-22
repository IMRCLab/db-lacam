import numpy as np

# visualization related
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation
import argparse
import yaml
import time
import os
from pathlib import Path
import itertools

def rgb_to_hex(rgb):
    """Converts RGB color tuple to hex color code."""
    return '#{:02x}{:02x}{:02x}'.format(int(rgb[0]), int(rgb[1]), int(rgb[2]))

# a simple color cycle (extend if you expect many trajs)
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

def visualize(env_file, trajs_files, result_file=None, show_cost=False):
    vis = meshcat.Visualizer()
    anim = Animation()

    res = vis.static_html()

    vis["/Cameras/default"].set_transform(
        tf.translation_matrix([0, 0, 0]).dot(
            tf.euler_matrix(0, np.radians(-30), np.radians(90))))
    
    vis["/Cameras/default/rotated/<object>"].set_transform(
        tf.translation_matrix([1, 0, 0]))

    # name of the saved file
    file_name = "octomap_exp_" + os.path.basename(env_file).split('.')[0] + ".html"

    # get environment info
    with open(env_file) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
    
    obstacles = data["environment"]["obstacles"]
    for k, obs in enumerate(obstacles):
        center = obs["center"]
        size = obs["size"]
        obs_type = obs["type"]
        if (obs_type == 'octomap'):
            octomap_stl = obs["octomap_stl"]
            vis[f"Obstacle{k}"].set_object(
                g.StlMeshGeometry.from_file(octomap_stl),
                g.MeshLambertMaterial(opacity=0.4, color=0xFFFFFF)) 
        elif (obs_type == 'box'):
            vis[f"Obstacle{k}"].set_object(g.Mesh(g.Box(size)))
            vis[f"Obstacle{k}"].set_transform(tf.translation_matrix(center))
        else:
            print("Unknown Obstacle type!")

    # plot each traj file with a different color
    traj_idx = 0
    for traj_file_idx, trajs_file in enumerate(trajs_files):
        with open(trajs_file) as f:
            expanded_trajs = yaml.load(f, Loader=yaml.FullLoader)

        color = COLOR_CYCLE[traj_file_idx % len(COLOR_CYCLE)]
        step = 1
        for i in range(0, len(expanded_trajs["trajs"]), step):
            traj = []
            for s in expanded_trajs["trajs"][i]["states"]:
                traj.append(s)
            start = [traj[0][0], traj[0][1], traj[0][2]]
            position = [[sublist[j] for sublist in traj] for j in range(3)]
            position = np.array(position)
            vis[f"expanded_trajs_{traj_file_idx}_{traj_idx}"].set_object(
                g.Line(g.PointsGeometry(position),
                       g.LineBasicMaterial(color=color)))
            vis[f"sphere_{traj_file_idx}_{traj_idx}"].set_object(
                g.Mesh(g.Sphere(0.01), g.MeshLambertMaterial(color=color)))
            vis[f"sphere_{traj_file_idx}_{traj_idx}"].set_transform(
                tf.translation_matrix(start))

            traj_idx += 1
    # start, goal states
    robots = data["robots"]
    for r in range(len(robots)):
        start = robots[r]["start"][:3]
        goal = robots[r]["goal"][:3]
        vis[f"sphere_start_{r}"].set_object(
            g.Mesh(g.Sphere(0.03), g.MeshLambertMaterial(color=0x00FF3300)))  # red
        vis[f"sphere_start_{r}"].set_transform(tf.translation_matrix(start))
        vis[f"sphere_goal_{r}"].set_object(
            g.Mesh(g.Sphere(0.03), g.MeshLambertMaterial(color=0x00FF3300)))  # red
        vis[f"sphere_goal_{r}"].set_transform(tf.translation_matrix(goal))

    # get results
    if result_file is not None:
        with open(result_file) as res_file:
            result = yaml.load(res_file, Loader=yaml.FullLoader)
        states = []
        name_robot = 0
        max_k = 0
        for i in range(len(result["result"])):
            state = []
            for s in result["result"][i]["states"]:
                state.append(s)
            max_k = max(max_k, len(state))
            states.append(state)
            position = [[sublist[j] for sublist in state] for j in range(3)]
            position = np.array(position)
            vis["Quadrotor" + str(name_robot)].set_object(
                g.StlMeshGeometry.from_file('../meshes/cf2_assembly.stl'),
                g.MeshLambertMaterial(color=0x0000FF)) # blue
            vis["trajectory" + str(name_robot)].set_object(
                g.Line(g.PointsGeometry(position),
                       g.LineBasicMaterial(color=0x00FF00))) # green
            name_robot += 1
        for k in range(max_k):
            for l in range(len(states)): # for each robot
                with anim.at_frame(vis, 10*k) as frame:
                    if k >= len(states[l]):
                        robot_state = states[l][-1]
                    else:
                        robot_state = states[l][k]
                    frame["Quadrotor" + str(l)].set_transform(
                        tf.translation_matrix(robot_state[0:3]).dot(
                            tf.quaternion_matrix(np.array([1,0,0,0]))))
            time.sleep(0.1)

    result_folder = Path(trajs_files[0]).resolve().parent
    vis.set_animation(anim)
    res = vis.static_html()
    with open(result_folder / file_name, "w") as f:
        f.write(res)
            
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('env', help="input file containing map")
    parser.add_argument('--trajs', nargs='+', help="one or more files with expanded trajectories during the search")
    parser.add_argument('--result', help="output file containing solution")
    parser.add_argument('--show_cost', action='store_true', help="indicate if cost vis is needed")

    args = parser.parse_args()

    visualize(args.env, args.trajs, args.result, args.show_cost)

if __name__ == "__main__":
    main()
