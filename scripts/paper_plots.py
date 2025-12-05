import os
import yaml
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches

# global normalization - max cost per instance
def plot_results(instances, num_trials, normalize_cost=False):
   
    results_path = "../results_with_maze"
    planners = {
        "db-cbs": {"marker": "1", "color": "#88CCEE"},   
        "db-ecbs": {"marker": "2", "color": "#009988"},  
        # "db-pibt": {"marker": "3", "color": "#E7B503"},  
        "db-lacam": {"marker": "+", "color": "#993404"}     
    }
    name_map = {
        "db-cbs": "db-CBS",
        "db-ecbs": "db-ECBS",
        # "db-pibt": "db-PIBT",
        "db-lacam": "db-LaCAM"
        }
    instance_map = {
    # a
    "alcove_unicycle":"alcove-u",
    # b
    "atgoal_unicycle":"atgoal-u",
    # c
    "circle2_unicycle":"circle-u-n2",
    "circle4_unicycle":"circle-u-n4",
    "circle6_unicycle":"circle-u-n6",
    "circle8_unicycle":"circle-u-n8",
    "circle10_unicycle":"circle-u-n10",
    # d 
    'gen_p10_n8_0_unicycle': "random-n8-u",
    'gen_p10_n8_1_unicycle': "random-n8-u",
    'gen_p10_n8_2_unicycle': "random-n8-u",
    'gen_p10_n8_3_unicycle': "random-n8-u",
    'gen_p10_n8_4_unicycle': "random-n8-u",
    'gen_p10_n8_5_unicycle': "random-n8-u",
    'gen_p10_n8_6_unicycle': "random-n8-u",
    'gen_p10_n8_7_unicycle': "random-n8-u",
    'gen_p10_n8_8_unicycle': "random-n8-u",
    'gen_p10_n8_9_unicycle': "random-n8-u",
    # e
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
    # f
    "maze_unicycle":"maze-n10",
    # g
    "passage6":"passage-n6-3D",
    # h
    "door4":"door-n4-3D",
    # i
    "forest4":"forest-n4-3D",
    "forest10":"forest-n10-3D",
    # j
    "swap8_hetero":"swap-n8-hetero",
    # k
    "random10_0_hetero":"random-n10-hetero",
    "random10_1_hetero":"random-n10-hetero",
    "random10_2_hetero":"random-n10-hetero",
    "random10_3_hetero":"random-n10-hetero"
    }

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
            ref_costs = data["db-lacam"][inst]["cost"]
            max_cost = max(ref_costs) if ref_costs else 1
            for p in planners:
                data[p][inst]["cost"] = [c / max_cost for c in data[p][inst]["cost"]]

    group_names = ["a", "b", "c", "d", "e", "f","g", "h", "i", "j", "k"]
    group_sizes = [1,    1,   5,   10,  10,  1,  1,   1,   2,   1,   4] 
    assert sum(group_sizes) == len(instances), "Group sizes must sum to total instances"

   # === Plot setup ===
    fig, axes = plt.subplots(
        3, 1, sharex=True, figsize=(9, 3.5),
        gridspec_kw={'height_ratios': [0.25, 1, 1], 'hspace': 0}
    )
    ax_fail, ax_time, ax_cost = axes
    x = np.arange(len(instances))

    num_planners = len(planners)
    planner_indices = {p: i for i, p in enumerate(planners)}

    # === Plot all data ===
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
                y_position = planner_idx * 0.5 + 1
                ax_fail.scatter(x_positions, [y_position] * fail_count,
                                marker=style["marker"], color=style["color"], s=80)
            # Runtime
            if times:
                ax_time.scatter([idx] * len(times), times,
                                marker=style["marker"], color=style["color"], s=80)
            # Cost
            if costs:
                ax_cost.scatter([idx] * len(costs), costs,
                                marker=style["marker"], color=style["color"], s=80)

    # === Add grouped background shading and labels ===
    start = 0
    for gname, gsize in zip(group_names, group_sizes):
        end = start + gsize
        mid = (start + end - 1) / 2

        for ax in axes:
            # light shading for alternating groups
            if group_names.index(gname) % 2 == 0:
                ax.axvspan(start - 0.5, end - 0.5, color='gray', alpha=0.08, zorder=0)
            # vertical group boundary
            ax.axvline(end - 0.5, color='gray', linestyle='--', alpha=0.5, linewidth=0.8)

        # group label under x-axis
        ax_cost.text(
            mid, -0.02, gname, ha='center', va='top',
            transform=ax_cost.get_xaxis_transform(),
            fontsize=10)
        start = end

    # === Add horizontal dashed grid lines for all subplots ===
    for ax in axes:
        ax.yaxis.grid(True, linestyle='--', alpha=0.5)
        ax.set_axisbelow(True)

    # === Axis labels and scales ===
    ax_fail.set_ylabel("Fail")
    ax_fail.set_yticks([])
    ax_fail.set_ylim(0, num_planners * 0.5 + 1)

    ax_time.set_ylabel("Runtime [s]")
    ax_cost.set_ylabel("Normalized Cost" if normalize_cost else "Cost [s]")

    fig.align_ylabels([ax_fail, ax_time, ax_cost])  

    # Hide per-instance labels to save space
    ax_cost.set_xticks([])

    # remove the free space before the first label
    for ax in axes:
        ax.set_xlim(-0.5, len(instances) - 0.5)
        ymin, ymax = ax.get_ylim()
        ymargin = 0.04 * (ymax - ymin)
        ax.set_ylim(ymin - ymargin, ymax + ymargin)
    # === Legend ===
    import matplotlib.lines as mlines
    legend_handles = []
    for planner, style in planners.items():
        handle = mlines.Line2D(
            [], [], color=style["color"],
            marker=style.get("marker", "o"),
            linestyle='None', markersize=9,
            label=name_map.get(planner, planner)
        )
        legend_handles.append(handle)

    fig.legend(
        handles=legend_handles,
        # ncol=2,  # stack vertically (or change to >1 if you prefer multiple columns)
        loc='upper left',
        bbox_to_anchor=(0.12, 0.83),  # position inside figure
        frameon=True,
        facecolor="white",  # background color for contrast
        edgecolor='black',  # optional: border for better visibility
        fontsize=9.5
    )

    # === Layout tweaks ===
    plt.subplots_adjust(top=0.90, bottom=0.18)
    plt.savefig("../results/ICAPS26/results_maze_plot_markers_normalized.pdf", format="pdf", bbox_inches="tight") if normalize_cost else plt.savefig("../results/results_plot_markers.pdf", format="pdf", bbox_inches="tight")
    plt.show()

# used for scalability plot
def plot_results_runtime(instances, num_trials, font_size=18):
    results_path = "../results/ICAPS26/n50"
    planners = {
        "db-cbs": {"color": "#88CCEE"},   
        "db-ecbs": {"color": "#009988"},  
        # "db-pibt": {"color": "#E7B503"},  
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
        # ax_time.axvline(x=i + 0.5, color="gray", linestyle="--", linewidth=0.7, alpha=0.6)
        ax_time.yaxis.grid(True, linestyle='--', alpha=0.6)
        ax_time.xaxis.grid(True, linestyle='--', alpha=0.6)


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
    plt.savefig("../results/ICAPS26/results_runtime_new.pdf", format="pdf", bbox_inches="tight")
    plt.show()



if __name__ == "__main__":
  instances = [
  "alcove_unicycle",
  "atgoal_unicycle",
  "circle2_unicycle",
  "circle4_unicycle",
  "circle6_unicycle",
  "circle8_unicycle",
  "circle10_unicycle",
  # scalability test
#   "test_n10_0_unicycle",
#   "test_n20_0_unicycle",
#   "test_n30_0_unicycle",
#   "test_n40_0_unicycle",
#   "test_n50_0_unicycle",
  ]
  for kind in ["unicycle","unicycle_sphere"]: 
    for n in [8]:
      for k in range(10):
        instances.append("gen_p10_n{}_{}_{}".format(n,k, kind))
  instances.append("maze_unicycle")
  instances.append("passage6")
  instances.append("door4")
  instances.append("forest4")
  instances.append("forest10")
  instances.append("swap8_hetero")
  instances.append("random10_0_hetero")
  instances.append("random10_1_hetero")
  instances.append("random10_2_hetero")
  instances.append("random10_3_hetero")
  
                   
  num_trials = 10  # max number of trials per instance
  plot_results(instances, num_trials, True)
#   plot_results_runtime(instances, num_trials)


