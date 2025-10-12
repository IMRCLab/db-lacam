import yaml
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import os


def clustering_analysis(a, i, itr):
    folder = "/home/akmarak-laptop/IMRC/db-lacam/results/ICAPS26/clustering/"
    t_values = {key: [] for key in a}
    cost_values = {key: [] for key in a}
    colors = ['#4477AA', '#CCBB44']
    labels = ['GOC', 'SC-GOC']
    i_map = {
          'gen_p10_n8_0_unicycle_sphere': "1",
          'gen_p10_n8_1_unicycle_sphere': "2",
          'gen_p10_n8_2_unicycle_sphere': "3",
          'gen_p10_n8_3_unicycle_sphere': "4",
          'gen_p10_n8_4_unicycle_sphere': "5",
          'gen_p10_n8_5_unicycle_sphere': "6",
          'gen_p10_n8_6_unicycle_sphere': "7",
          'gen_p10_n8_7_unicycle_sphere': "8",
          'gen_p10_n8_8_unicycle_sphere': "9",
          'gen_p10_n8_9_unicycle_sphere': "10",
      }
    x = np.arange(len(i))  # one x per instance

    for a_instance in a:
        for i_instance in i:
            t = 0
            cost = 0
            valid = False
            for it in range(itr):
                yaml_file = folder + a_instance + "/" + i_instance + "/db-lacam/00" + str(it) + "/stats.yaml"
                try:
                    with open(yaml_file, 'r') as file:
                        data = yaml.safe_load(file)
                        if 't' in data["stats"][0]:
                            t += data["stats"][0]['t']
                            cost += data["stats"][0]['cost']
                            valid = True
                except FileNotFoundError:
                    print(f"Error: {yaml_file} not found")
                    t_values[a_instance].append(None)
                    cost_values[a_instance].append(None)
            if valid:
                t_values[a_instance].append(t / itr)
                cost_values[a_instance].append(cost / itr)

    fig, ax = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
    
    # Width for grouped bars
    bar_width = 0.40
    font_size = 24
    parameters = {'Runtime [s]': t_values, 'Cost [s]': cost_values}
    for idx, (ylabel, values) in enumerate(parameters.items()):
        for j, algo in enumerate(a):
            ax[idx].bar(
                x + j * bar_width, 
                values[algo], 
                width=bar_width, 
                color=colors[j], 
                edgecolor=(0, 0, 0, 0.5),
                alpha=0.8, 
                label=labels[j] if idx == 0 else None  # legend only once
            )
        ax[idx].set_ylabel(ylabel, fontsize=font_size)
        ax[idx].grid(axis="x", linestyle="dashed",alpha=0.6)
        ax[idx].grid(axis="y", linestyle="dashed",alpha=0.6)


    ax[0].legend(fontsize = font_size-7)
    ax[-1].set_xticks(x + bar_width / 2)
    ax[-1].set_xticklabels([i_map[key] for key in i], fontsize=font_size)
    ax[0].tick_params(axis='both', labelsize=font_size-2)
    ax[1].tick_params(axis='both', labelsize=font_size-2)

    plt.tight_layout()
    plt.savefig("../results/ICAPS26/clustering/clustering_analysis.pdf", format="pdf", bbox_inches="tight") 


def heuristic_estimation_analysis(a, i, itr):
    folder = "/home/akmarak-laptop/IMRC/db-lacam/results/ICAPS26/heuristic-estimation/"
    t_values = {key: [] for key in a}
    cost_values = {key: [] for key in a}
    colors = ['#4477AA', '#CCBB44']
    labels = ['db-A*', 'EST']
    i_map = {
          'circle2_unicycle': "2",
          'circle4_unicycle': "4",
          'circle6_unicycle': "6",
          'circle8_unicycle': "8",
          'circle10_unicycle': "10",
    }
    t_values = {algo: {inst: [] for inst in i} for algo in a}
    cost_values = {algo: {inst: [] for inst in i} for algo in a}
    x = np.arange(len(i)) 

    for a_instance in a:
        for i_instance in i:
            for it in range(itr):
                yaml_file = folder + a_instance + "/" + i_instance + "/db-lacam/00" + str(it) + "/stats.yaml"
                try:
                    with open(yaml_file, 'r') as file:
                        data = yaml.safe_load(file)
                        if 't' in data["stats"][0]:
                            t = data["stats"][0]['t']
                            cost = data["stats"][0]['cost']
                            t_values[a_instance][i_instance].append(t)
                            cost_values[a_instance][i_instance].append(cost)
                except FileNotFoundError:
                    print(f"Error: {yaml_file} not found")

    fig, ax = plt.subplots(1, 1, figsize=(7, 4))
    font_size = 18

    for j, algo in enumerate(a):
        means = []
        stds = []
        for inst in i:
            vals = np.array(t_values[algo][inst])
            if len(vals) > 0:
                means.append(vals.mean())
                stds.append(vals.std())
            else:
                means.append(np.nan)  # missing data
                stds.append(0)

        ax.plot(x, means, color=colors[j], marker="o", label=labels[j])
        ax.fill_between(x, np.array(means)-np.array(stds), np.array(means)+np.array(stds),
                        color=colors[j], alpha=0.2)

    ax.set_ylabel("Runtime [s]", fontsize=font_size)
    ax.set_xticks(x)
    ax.set_xticklabels([i_map[key] for key in i], fontsize=font_size)
    ax.tick_params(axis="both", labelsize=font_size-2)
    ax.grid(axis="x", linestyle="dashed",alpha=0.6)
    ax.legend(fontsize=font_size-2)

    plt.tight_layout()
    # plt.show()
    plt.savefig("../results/ICAPS26/heuristic-estimation/heuristic_estimation_analysis.pdf", format="pdf", bbox_inches="tight") 

def read_time_stats(file_path):
    # Read and parse the YAML file
    print(file_path)
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    try:
        return {
            "reverse_search": data['data']['reverse_search'],
            "time_rollout": data['data']['time_rollout'],
            "time_clustering": data['data']['time_clustering'],
            "time_collision_with_env": data['data']['time_collision_with_env'],
            "time_collision_with_planned": data['data']['time_collision_with_planned'],
            "time_collision_with_unplanned": data['data']['time_collision_with_unplanned'],
        }
    except KeyError as e:
        print(f"Error: Missing expected key {e} in the YAML data.")
        return None
    
# compares time for main components of the planner, focus on h-estimation (dbA* vs. EST)
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

def time_analysis_plot(data_iterations):
    instance_names = [
        "2-dbA*", "2-EST",
        "4-dbA*", "4-EST",
        "8-dbA*", "8-EST",
        "10-dbA*", "10-EST",
    ]

    categories_bottom = {
        'Motion Rollout': 'time_rollout',
        'Motion Cluster': 'time_clustering',
        'Collision-Env.': 'time_collision_with_env',
        'Collision-Rob.': 'time_collision_with_planned',
        'Collision-Pot.': 'time_collision_with_unplanned',
    }

    colors_bars = ['#66CCEE', '#CCBB44', '#664477', '#AA3377', '#BBBBBB']
    h_est_color = '#4477AA'
    width = 0.35
    font_size = 22

    # Group positions (4, 8, 10)
    groups = [4, 6, 8, 10]
    x_positions = list(range(len(groups)))
    offset = width / 2

    fig, (ax_top, ax_bottom) = plt.subplots(
        2, 1, figsize=(9, 7), sharex=True,
        gridspec_kw={'height_ratios': [1, 2]}
    )

    # ----------------------------
    # TOP subplot: all components + h-Estimation + legend
    # ----------------------------
    for group_idx, group in enumerate(groups):
        for j, algo in enumerate(["dbA*", "EST"]):
            data_idx = 2 * group_idx + j
            shift = -offset if algo == "dbA*" else offset
            x = x_positions[group_idx] + shift

            # h-Estimation (reverse_search)
            val_h = data_iterations[data_idx]['reverse_search']
            hatch = '//' if algo == 'dbA*' else None
            ax_top.bar(x, val_h, width=width, color=h_est_color, edgecolor=(0, 0, 0, 0.5), alpha=0.8, hatch=hatch, linewidth=1, label='h-Estimation' if group_idx == 0 and j == 0 else None)

            # Stacked runtime breakdown
            bottom_val = val_h
            for k, (category, key) in enumerate(categories_bottom.items()):
                val = data_iterations[data_idx][key]
                color = colors_bars[k % len(colors_bars)]
                label = category if group_idx == 0 and j == 0 else None
                ax_top.bar(x, val, width=width, bottom=bottom_val, color=color, alpha=0.8, hatch=hatch, label=label)
                bottom_val += val

    ax_top.set_ylabel("Runtime [ms]", fontsize=font_size)
    ax_top.grid(axis='y', linestyle='dashed', alpha=0.6)
    ax_top.grid(axis='x', linestyle='dashed', alpha=0.6)
    ax_top.tick_params(axis='both', labelsize=font_size - 2)

    # BOTTOM subplot: runtime breakdown only (no h-Estimation, no legend)
    # ----------------------------
    for group_idx, group in enumerate(groups):
        for j, algo in enumerate(["dbA*", "EST"]):
            data_idx = 2 * group_idx + j
            shift = -offset if algo == "dbA*" else offset
            x = x_positions[group_idx] + shift

            bottom_val = 0
            hatch = '//' if algo == 'dbA*' else None
            for k, (category, key) in enumerate(categories_bottom.items()):
                val = data_iterations[data_idx][key]
                color = colors_bars[k % len(colors_bars)]
                ax_bottom.bar(x, val, width=width, bottom=bottom_val, color=color, edgecolor=(0, 0, 0, 0.5), alpha=0.8, hatch=hatch)
                bottom_val += val

    legend_elements_main = [
    Patch(facecolor=h_est_color, label='h-Estimation')
    ]
    for idx, (category, _) in enumerate(categories_bottom.items()):
        legend_elements_main.append(Patch(facecolor=colors_bars[idx % len(colors_bars)], label=category))

    # ----- Secondary legend: planner type (hatching explanation) -----
    legend_elements_hatch = [
        Patch(facecolor='white', edgecolor='black', label='EST'),
        Patch(facecolor='white', edgecolor='black', hatch='//', label='dbA*')
    ]

    # Combine both in the bottom subplot
    main_legend = ax_bottom.legend(handles=legend_elements_main,
                                   fontsize=font_size - 5,
                                   ncol=2,
                                   loc='upper left')
    ax_bottom.add_artist(main_legend)
    ax_bottom.legend(handles=legend_elements_hatch,
                 fontsize=font_size - 5,
                 ncol=2,
                 bbox_to_anchor=(0.43, 0.72),
                 frameon=False)
    
    ax_bottom.set_ylabel("Runtime [ms]", fontsize=font_size)
    ax_bottom.set_xticks(x_positions)
    ax_bottom.set_xticklabels([str(g) for g in groups], fontsize=font_size)
    ax_bottom.grid(axis='y', linestyle='dashed', alpha=0.6)
    ax_bottom.grid(axis='x', linestyle='dashed', alpha=0.6)
    ax_bottom.tick_params(axis='both', labelsize=font_size - 2)
    ax_bottom.set_xlabel("Number of Robots", fontsize=font_size)

    plt.tight_layout()
    plt.savefig("../results/ICAPS26/results_time_analysis.pdf", format="pdf", bbox_inches="tight")
    plt.show()

def main():
# 1. clustering methods h-based vs. state-based
#   a = ["h-based", "state-based"]
#   i = [
#     'gen_p10_n8_0_unicycle_sphere',
#     'gen_p10_n8_1_unicycle_sphere',
#     'gen_p10_n8_2_unicycle_sphere',
#     'gen_p10_n8_3_unicycle_sphere',
#     'gen_p10_n8_4_unicycle_sphere',
#     'gen_p10_n8_5_unicycle_sphere',
#     'gen_p10_n8_6_unicycle_sphere',
#     'gen_p10_n8_7_unicycle_sphere',
#     'gen_p10_n8_8_unicycle_sphere',
#     'gen_p10_n8_9_unicycle_sphere',
#   ]
#   clustering_analysis(a, i, 5)
# 2. heuristic estimation using db-A* and EST on demand
    # a = ["reverse-search", "est"]
    # i = [
    #     'circle2_unicycle',
    #     'circle4_unicycle',
    #     'circle6_unicycle',
    #     'circle8_unicycle',
    #     'circle10_unicycle',
    # ]
    # heuristic_estimation_analysis(a, i, 5)
# 3. time analysis on planner's main components
    path = "/home/akmarak-laptop/IMRC/db-lacam/results/ICAPS26/time/"
    instances = ["circle4_unicycle", "circle6_unicycle", "circle8_unicycle", "circle10_unicycle"]
    folder = [
        "dbastar",
        "est"
    ]
    file_name = "time_search.yaml"
    file_paths = []
    # Generate paths by combining the base path, instance, and algorithm
    for instance in instances:
        for f in folder:
            file_paths.append(path + f + "/" + instance + "/db-lacam/000/" + file_name)
    # Read data from each YAML file
    data_iterations = []
    for file_path in file_paths:
        if os.path.exists(file_path):
            data = read_time_stats(file_path)
            if data:
                data_iterations.append(data)
            else:
                print(file_path)
        else: 
            print(file_path)
    time_analysis_plot(data_iterations)

if __name__ == "__main__":
  main()