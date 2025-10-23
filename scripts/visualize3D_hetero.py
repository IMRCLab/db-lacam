import numpy as np
import meshcat
import meshcat.geometry as g
import meshcat.transformations as tf
from meshcat.animation import Animation
from pathlib import Path
import argparse
import yaml
import time

def visualize(env_file, result_file):
    vis = meshcat.Visualizer()
    anim = Animation()

    vis["/Cameras/default"].set_transform(
        tf.translation_matrix([0, 0, 0]).dot(
            tf.euler_matrix(0, np.radians(-30), np.radians(90))))
    vis["/Cameras/default/rotated/<object>"].set_transform(
        tf.translation_matrix([1, 0, 0]))

    with open(env_file) as f:
        data = yaml.load(f, Loader=yaml.FullLoader)

    obstacles = data["environment"]["obstacles"]
    for k, obs in enumerate(obstacles):
      center = obs["center"]
      size = obs["size"]
      obs_type = obs["type"]
      if (obs_type == 'octomap'):
         octomap_stl = obs["octomap_stl"]
         vis[f"Obstacle{k}"].set_object(g.StlMeshGeometry.from_file(octomap_stl), g.MeshLambertMaterial(opacity=1.0, color=0xFFFFFF)) 
      elif (obs_type == 'box'):
        vis[f"Obstacle{k}"].set_object(g.Mesh(g.Box(size)))
        vis[f"Obstacle{k}"].set_transform(tf.translation_matrix(center))
      else:
         print("Unknown Obstacle type!")
         
    with open(result_file) as res_file:
        result = yaml.load(res_file, Loader=yaml.FullLoader)

    states = []
    name_robot = 0
    max_k = 0
    polulu_size = [0.05, 0.05, 0.60]

    # define consistent colors
    color_drone = 0x0000FF   # blue
    color_polulu = 0xCCCC00  # dark yellow

    color_traj_drone = 0x1E90FF  # lighter blue for trajectory
    color_traj_polulu = 0xFFD700  # yellow/gold for trajectory

    for i in range(len(result["result"])):
        state = []
        for s in result["result"][i]["states"]:
            state.append(s)

        max_k = max(max_k, len(state))
        states.append(state)

        position = np.array([[sublist[j] for sublist in state] for j in range(3)])
        robot_type = data["robots"][i]["type"]

        if robot_type == "integrator2_3d_v0":  # drone
            vis[f"Quadrotor{name_robot}"].set_object(
                g.StlMeshGeometry.from_file('../meshes/cf2_assembly.stl'),
                g.MeshLambertMaterial(color=color_drone)
            )
             # **Set initial position**
            first_state = state[0]
            vis[f"Quadrotor{i}"].set_transform(
                tf.translation_matrix(first_state[:3]).dot(tf.quaternion_matrix([1,0,0,0]))
            )

            vis[f"trajectory{i}"].set_object(g.Line(
                g.PointsGeometry(position),
                g.LineBasicMaterial(color=color_traj_drone)
            ))
            vis[f"trajectory{name_robot}"].set_object(g.Line(
                g.PointsGeometry(position),
                g.LineBasicMaterial(color=color_traj_drone, linewidth=4)
            ))
        elif robot_type == "unicycle1_3d_v0":  # polulu
            vis[f"Polulu{name_robot}"].set_object(
                g.Box(polulu_size),
                g.MeshLambertMaterial(color=color_polulu)
            )
            # **Set initial position**
            first_state = state[0]
            vis[f"Polulu{i}"].set_transform(
                tf.translation_matrix(first_state[:3] + np.array([0,0,0.30])).dot(tf.quaternion_matrix([1,0,0,0]))
            )
            vis[f"trajectory{name_robot}"].set_object(g.Line(
                g.PointsGeometry(position),
                g.LineBasicMaterial(color=color_traj_polulu, linewidth=4)
            ))

        name_robot += 1

    # animate
    for k in range(max_k):
        for l in range(len(states)):
            with anim.at_frame(vis, 10 * k) as frame:
                robot_state = states[l][min(k, len(states[l]) - 1)]
                robot_type = data["robots"][l]["type"]

                if robot_type == "integrator2_3d_v0":
                    frame[f"Quadrotor{l}"].set_transform(
                        tf.translation_matrix(robot_state[:3]).dot(
                            tf.quaternion_matrix(np.array([1, 0, 0, 0]))
                        )
                    )
                elif robot_type == "unicycle1_3d_v0":
                    frame[f"Polulu{l}"].set_transform(
                        tf.translation_matrix(robot_state[:3] + np.array([0, 0, 0.30])).dot(
                            tf.quaternion_matrix(np.array([1, 0, 0, 0]))
                        )
                    )
        time.sleep(0.1)

    vis.set_animation(anim)
    html_file = Path(result_file).with_suffix(".html")
    with open(html_file, "w") as f:
        f.write(vis.static_html())


            
def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("env", help="input file containing map")
  parser.add_argument("--result", help="output file containing solution")
  args = parser.parse_args()

  visualize(args.env, args.result)

if __name__ == "__main__":
  main()