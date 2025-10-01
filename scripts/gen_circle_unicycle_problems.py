import math
import numpy as np
import yaml

def to_py(x):
    """Convert NumPy types to pure Python types recursively."""
    if isinstance(x, (np.float32, np.float64)):
        return float(x)
    if isinstance(x, (np.int32, np.int64)):
        return int(x)
    if isinstance(x, (list, tuple, np.ndarray)):
        return [to_py(v) for v in x]
    return x

def generate_circle_swap_yaml(num_robots, env_size=20.0, margin=1.0, robot_type="unicycle1_v0"):
    center = (env_size / 2.0, env_size / 2.0)
    max_radius = env_size / 2.0 - margin
    R = max_radius
    
    angles = np.linspace(0, 2*math.pi, num_robots, endpoint=False)
    
    start_positions, goal_positions = [], []
    for ang in angles:
        sx = center[0] + R * math.cos(ang)
        sy = center[1] + R * math.sin(ang)
        stheta = ang
        gx = center[0] + R * math.cos(ang + math.pi)
        gy = center[1] + R * math.sin(ang + math.pi)
        gtheta = (stheta + math.pi) % (2*math.pi)
        start_positions.append([sx, sy, stheta])
        goal_positions.append([gx, gy, gtheta])
    
    data = {
        "environment": {
            "min": [0.0, 0.0],
            "max": [env_size, env_size],
            "obstacles": []
        },
        "robots": []
    }
    
    for i in range(num_robots):
        data["robots"].append({
            "type": robot_type,
            "start": to_py([round(v, 3) for v in start_positions[i]]),
            "goal": to_py([round(v, 3) for v in goal_positions[i]])
        })
    
    return yaml.dump(data, sort_keys=False)

if __name__ == "__main__":
    num_robots = 20
    yaml_str = generate_circle_swap_yaml(num_robots)
    # print(yaml_str)
    with open("../example/unicycle_circle_swap_" + str(num_robots) + ".yaml", "w") as f:
        f.write(yaml_str)
