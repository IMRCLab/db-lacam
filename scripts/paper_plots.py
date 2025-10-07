import os
import yaml
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches

def read_time_stats(file_path):
    # Read and parse the YAML file
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    # time_nearestMotion 
    # time_hfun 
    try:
        return {
            "time_collision_heuristic": data['data']['time_collision_heuristic'],
            "time_collisions": data['data']['time_collisions'],
            "time_nearestNode": data['data']['time_nearestNode'],
            "time_rebuild_focal_set": data['data']['time_rebuild_focal_set'],
            # "time_search": data['data']['time_search'],
        }
    except KeyError as e:
        print(f"Error: Missing expected key {e} in the YAML data.")
        return None

# global normalization - max cost per instance
def plot_results(instances, num_trials, normalize_cost=False):
   
    results_path = "../results_test"
    planners = {
        "db-cbs": {"marker": "1", "color": "#88CCEE"},   
        "db-ecbs": {"marker": "2", "color": "#009988"},  
        "db-pibt": {"marker": "|", "color": "#E7B503"},  
        "db-lacam": {"marker": "+", "color": "#993404"}     
    }
    name_map = {
        "db-cbs": "db-CBS",
        "db-ecbs": "db-ECBS",
        "db-pibt": "db-PIBT",
        "db-lacam": "db-LaCAM"
        }
    instance_map = {
    "alcove_unicycle":"alcove-u",
    "atgoal_unicycle":"atgoal-u",
    "circle2_unicycle":"circle-u-n2",
    "circle4_unicycle":"circle-u-n4",
    "circle6_unicycle":"circle-u-n6",
    "circle8_unicycle":"circle-u-n8",
    "circle10_unicycle":"circle-u-n10",
    "forest4":"forest-n4-3D",
    "corridor4":"corridor-n4-3D",
    "circle6":"circle-n6-3D",
    "circle7_swap":"circle-n7-3D",
    "passage6":"passage-n6-3D",
    'gen_p10_n8_0_unicycle_sphere': "random-n8-u\u209B",
    'gen_p10_n8_1_unicycle_sphere': "random-n8-u\u209B",
    'gen_p10_n8_2_unicycle_sphere': "random-n8-u\u209B",
    'gen_p10_n8_3_unicycle_sphere': "random-n8-u\u209B",
    'gen_p10_n8_4_unicycle_sphere': "random-n8-u\u209B",
    'gen_p10_n8_5_unicycle_sphere': "random-n8-u\u209B",
    'gen_p10_n8_6_unicycle_sphere': "random-n8-u\u209B",
    'gen_p10_n8_7_unicycle_sphere': "random-n8-u\u209B",
    'gen_p10_n8_8_unicycle_sphere': "random-n8-u\u209B",
    'gen_p10_n8_9_unicycle_sphere': "random-n8-u\u209B",
    'gen_p10_n8_0_unicycle': "random-n8-u",
    'gen_p10_n8_1_unicycle': "random-n8-u",
    'gen_p10_n8_2_unicycle': "random-n8-u",
    'gen_p10_n8_3_unicycle': "random-n8-u",
    'gen_p10_n8_4_unicycle': "random-n8-u",
    'gen_p10_n8_5_unicycle': "random-n8-u",
    'gen_p10_n8_6_unicycle': "random-n8-u",
    'gen_p10_n8_7_unicycle': "random-n8-u",
    'gen_p10_n8_8_unicycle': "random-n8-u",
    'gen_p10_n8_9_unicycle': "random-n8-u"}

    data = {p: {inst: {"time": [], "cost": [], "fail": 0} for inst in instances} for p in planners}

    # Collect data
    for inst in instances:
        for planner in planners:
            for trial in range(num_trials):
                trial_dir = os.path.join(results_path, inst, planner, f"{trial:03d}")
                stats_file = os.path.join(trial_dir, "stats.yaml")

                if not os.path.exists(stats_file):
                    data[planner][inst]["fail"] += 1
                    continue

                with open(stats_file, "r") as f:
                    stats = yaml.safe_load(f)

                if not stats or "stats" not in stats or not stats["stats"]:
                    data[planner][inst]["fail"] += 1
                    continue

                first = stats["stats"][0]
                if "t" in first and "cost" in first:
                    if first["cost"] > 400:
                        continue
                    data[planner][inst]["time"].append(first["t"])
                    data[planner][inst]["cost"].append(first["cost"])

    # Normalize cost per instance if requested
    if normalize_cost:
        for inst in instances:
            # Use db-lacam cost as max for this instance
            ref_costs = data["db-lacam"][inst]["cost"]
            max_cost = max(ref_costs) if ref_costs else 1
            # divide each planner's cost for this instance by db-lacam's max
            for p in planners:
                data[p][inst]["cost"] = [c / max_cost for c in data[p][inst]["cost"]]

    # Map instances
    plot_instances = [instance_map.get(inst, inst) for inst in instances]

    # Plot
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 4),
                             gridspec_kw={'height_ratios':[0.25, 1, 1], 'hspace': 0})
    ax_fail, ax_time, ax_cost = axes
    x = np.arange(len(instances))

    # Vertical dashed lines
    for ax in axes:
        for i in range(len(instances)):
            ax.axvline(i - 0.5, color="gray", linestyle="--", linewidth=0.7, alpha=0.6)

    num_planners = len(planners)
    planner_indices = {p: i for i, p in enumerate(planners)}

    for planner, style in planners.items():
        planner_idx = planner_indices[planner]
        for idx, inst in enumerate(instances):
            fail_count = data[planner][inst]["fail"]
            times = data[planner][inst]["time"]
            costs = data[planner][inst]["cost"]

            # Failures
            if fail_count > 0:
                x_offsets = np.linspace(-0.2, 0.2, fail_count)
                x_positions = idx + x_offsets
                y_position = planner_idx*0.5 + 1 
                ax_fail.scatter(x_positions, [y_position]*fail_count,
                                marker=style["marker"], color=style["color"], s=80)
            # Runtime
            if times:
                ax_time.scatter([idx]*len(times), times, marker=style["marker"],
                                color=style["color"], s=80)
            # Cost
            if costs:
                ax_cost.scatter([idx]*len(costs), costs, marker=style["marker"],
                                color=style["color"], s=80)

    # Labels
    ax_fail.set_ylabel("Failure")
    ax_fail.set_yticks([])
    ax_fail.set_ylim(0, num_planners * 0.5 + 1)
    ax_time.set_ylabel("Runtime [s]")
    ax_cost.set_ylabel("Normalized Cost [s]" if normalize_cost else "Cost [s]")
    ax_cost.set_xticks(x)
    ax_cost.set_xticklabels(plot_instances, rotation=45, ha='right')

    # Legend
    legend_handles = []
    for planner, style in planners.items():
        handle = mlines.Line2D(
            [], [], 
            color=style["color"], 
            marker=style.get("marker", "o"),  # fallback if marker not defined
            linestyle='None', 
            markersize=8, 
            label=name_map.get(planner, planner)  # use mapped name
        )
        legend_handles.append(handle)

    fig.legend(
        handles=legend_handles, 
        ncol=len(planners),
        loc='upper center', 
        bbox_to_anchor=(0.5, 0.98),  # adjust vertical position
        frameon=False,                # no rectangle
        fontsize=9
    )

    # Adjust top margin
    plt.subplots_adjust(top=0.90)
    plt.savefig("../results/results_plot_markers_normalized.pdf", format="pdf", bbox_inches="tight") if normalize_cost else plt.savefig("../results/results_plot_markers.pdf", format="pdf", bbox_inches="tight")
    plt.show()

# used for scalability plot
def plot_results_runtime(instances, num_trials, font_size=18):
    results_path = "../results"
    planners = {
        "db-cbs": {"color": "#88CCEE"},   
        "db-ecbs": {"color": "#009988"},  
        "db-pibt": {"color": "#E7B503"},  
        "db-lacam": {"color": "#993404"}     
    }

    instance_map = {
        "test_n10_0_unicycle":"10",
        "test_n20_0_unicycle":"20",
        "test_n30_0_unicycle":"30",
        "test_n40_0_unicycle":"40",
        "test_n50_0_unicycle":"50"
    }

    # storage
    data = {p: {inst: [] for inst in instances} for p in planners}

    # load stats
    for inst in instances:
        for planner in planners:
            for trial in range(num_trials):
                trial_dir = os.path.join(results_path, inst, planner, f"{trial:03d}")
                stats_file = os.path.join(trial_dir, "stats.yaml")

                if not os.path.exists(stats_file):
                    continue

                with open(stats_file, "r") as f:
                    stats = yaml.safe_load(f)

                if not stats or "stats" not in stats or not stats["stats"]:
                    continue

                first = stats["stats"][0]
                if "t" in first:
                    data[planner][inst].append(first["t"])

    # prepare x labels
    plot_instances = [instance_map.get(inst, inst) for inst in instances]
    x = np.arange(len(instances))

    # figure
    fig, ax_time = plt.subplots(figsize=(7, 4))

    # vertical separators between instances
    for i in range(len(instances) - 1):
        ax_time.axvline(x=i + 0.5, color="gray", linestyle="--", linewidth=0.7, alpha=0.6)

    # plot each planner with mean ± std as shaded area
    for planner, style in planners.items():
        color = style["color"]
        times_mean, times_std = [], []
        for inst in instances:
            tvals = data[planner][inst]
            if tvals:
                times_mean.append(np.mean(tvals))
                times_std.append(np.std(tvals))
            else:
                times_mean.append(np.nan)
                times_std.append(0)

        times_mean = np.array(times_mean)
        times_std = np.array(times_std)

        # plot mean line
        ax_time.plot(x, times_mean, marker="o", linewidth=2, color=color, label=planner)
        # plot shadow for ± std
        ax_time.fill_between(x, times_mean - times_std, times_mean + times_std,
                             color=color, alpha=0.25)

    # labels
    ax_time.set_ylabel("Runtime [s]", fontsize=font_size)
    ax_time.set_xlabel("Number of Robots", fontsize=font_size)
    ax_time.set_xticks(x)
    ax_time.set_xticklabels(plot_instances, fontsize=font_size)
    ax_time.tick_params(axis='both', labelsize=font_size-2)

    # legend
    # ax_time.legend(handles=legend_patches, fontsize=font_size-2, loc="best", frameon=True)
    name_map = {
    "db-cbs": "db-CBS",
    "db-ecbs": "db-ECBS",
    "db-pibt": "db-PIBT",
    "db-lacam": "db-LaCAM"
    }

    # create legend patches using mapped names
    legend_patches = [
        mpatches.Patch(color=style["color"], label=name_map.get(planner, planner))
        for planner, style in planners.items()
    ]

    # place legend on top without rectangle
    ax_time.legend(
    handles=legend_patches,
    ncol=len(planners),           # keep all planners in one row
    loc='upper center',
    bbox_to_anchor=(0.5, 1.19),  # vertical position
    fontsize=font_size-2,
    frameon=False,
    handlelength=0.5,             # length of the color box
    handleheight=0.75,                # height of the color box
    handletextpad=0.2             # space between color box and label
    )

    plt.tight_layout()
    plt.savefig("../results/results_runtime.pdf", format="pdf", bbox_inches="tight")
    plt.show()

# compares time for main components of the planner, focus on h-estimation (dbA* vs. EST)
def time_analysis_plot(data_iterations):
    instance_names = [  # small, big delta order for each problem
    "2-dbA*", 
    "2-EST",
    #
    "4-dbA*",
    "4-EST", 
    #
    "8-dbA*",
    "8-EST"
    ]
    # Data for plotting
    categories = {
        'h-Estimation': 'reverse_search',
        'Motion Rollout': 'time_rollout',
        'Motion Clustering': 'time_clustering',
        'Collision (env)': 'time_collision_with_env',
        'Collision (robots)': 'time_collision_with_planned',
        'Collision (potential)': 'time_collision_with_unplanned',
    }
   
    colors = ['#4477AA', '#66CCEE', '#CCBB44', '#664477' '#AA3377', '#BBBBB'] # blue, cyan, yellow, red, purple, grey
    labels = instance_names 
    # Initialize plot
    fig, ax = plt.subplots()
    width = 0.35 # Bar width
    x_positions = range(len(data_iterations))  # Positions for each bar
    # Create stacked bars for each iteration
    for category_index, (category, key) in enumerate(categories.items()):
        bottoms = [sum(data[categories[c]] for c in list(categories.keys())[:category_index]) for data in data_iterations]
        values = [data[key] for data in data_iterations]
        ax.bar(x_positions, 
               values, 
               width, 
               label=category,
               alpha=0.9, 
               color=colors[category_index], 
               bottom=bottoms,
               edgecolor='black')
    
    ax.grid(which='both', axis='x', linestyle='dashed')
    ax.grid(which='major', axis='y', linestyle='dashed')
    ax.set_ylabel("Runtime [ms]")
    ax.set_xticks(x_positions)
    ax.set_xticklabels(labels, rotation=45, ha='right')
    ax.legend(loc='upper left')
    plt.tight_layout()
    plt.grid(True)
    plt.show()

if __name__ == "__main__":

# 1. results plot
#   instances = [
#   "alcove_unicycle",
#   "atgoal_unicycle",
#   "circle2_unicycle",
#   "circle4_unicycle",
#   "circle6_unicycle",
#   "circle8_unicycle",
#   "circle10_unicycle",
#   "test_n10_0_unicycle",
#   "test_n20_0_unicycle",
#   "test_n30_0_unicycle",
#   "test_n40_0_unicycle",
#   "test_n50_0_unicycle",
#   ]
#   for kind in ["unicycle","unicycle_sphere"]: 
#     for n in [8]:
#       for k in range(10):
#         instances.append("gen_p10_n{}_{}_{}".format(n,k, kind))
#   instances.append("forest4")
#   instances.append("corridor4")
#   instances.append("circle6")
#   instances.append("circle7_swap")
#   instances.append("passage6")
                   
#   num_trials = 5  # max number of trials per instance
#   plot_results(instances, num_trials, True)
#   plot_results_runtime(instances, num_trials)

# 2. time analysis on planner's main components
    path = "/home/akmarak-laptop/IMRC/db-CBS/ICAPS26/time/"
    instances = ["circle2", "circle4", "circle8", "circle10"]
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


