import os
import yaml
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches

# global normalization - max cost per instance
def plot_results(instances, num_trials, normalize_cost=False):
   
    results_path = "../results_test"
    planners = {
        "db-cbs": {"marker": "1", "color": "#88CCEE"},   
        "db-ecbs": {"marker": "2", "color": "#009988"},  
        "db-pibt": {"marker": "|", "color": "#E7B503"},  
        "db-lacam": {"marker": "+", "color": "#993404"}     
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
    ax_fail.set_ylabel("Failures")
    ax_fail.set_yticks([])
    ax_fail.set_ylim(0, num_planners * 0.5 + 1)
    ax_time.set_ylabel("Runtime [s]")
    ax_cost.set_ylabel("Normalized Cost [s]" if normalize_cost else "Cost [s]")
    ax_cost.set_xticks(x)
    ax_cost.set_xticklabels(plot_instances, rotation=45, ha='right')

    # Legend
    legend_handles = []
    for planner, style in planners.items():
        handle = mlines.Line2D([], [], color=style["color"], marker=style["marker"],
                               linestyle='None', markersize=8, label=planner)
        legend_handles.append(handle)

    fig.legend(handles=legend_handles, ncol=num_planners,
               loc='upper center', bbox_to_anchor=(0.5, 0.99))

    # Adjust top margin
    plt.subplots_adjust(top=0.90)
    plt.savefig("../results/results_plot_markers_normalized.pdf", format="pdf", bbox_inches="tight") if normalize_cost else plt.savefig("../results/results_plot_markers.pdf", format="pdf", bbox_inches="tight")
    plt.show()

def plot_results_bar_chart(instances, num_trials):
    results_path = "../results_test"
    planners = {
        "db-cbs": {"color": "#88CCEE"},   
        "db-ecbs": {"color": "#009988"},  
        "db-pibt": {"color": "#E7B503"},  
        "db-lacam": {"color": "#993404"}     
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

    # Map instances for plotting
    plot_instances = [instance_map.get(inst, inst) for inst in instances]
    x = np.arange(len(instances))

    fig, (ax_time, ax_cost) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    width = 0.7 / len(planners)  # small bars grouped inside each instance

    for i, (planner, style) in enumerate(planners.items()):
        color = style["color"]
        offset = (i - len(planners)/2) * width + width/2

        times_mean = []
        costs_mean = []
        fails_frac = []

        for inst in instances:
            tvals = data[planner][inst]["time"]
            cvals = data[planner][inst]["cost"]
            fails = data[planner][inst]["fail"]

            # average runtime / cost
            times_mean.append(np.mean(tvals) if tvals else 0)
            costs_mean.append(np.mean(cvals) if cvals else 0)
            # failure fraction
            fails_frac.append(fails / num_trials)

        # Plot stacked runtime bars
        ax_time.bar(x + offset, times_mean, width, color=color, alpha=0.8)
        ax_time.bar(x + offset, fails_frac, width, bottom=times_mean,
                    color="lightgray", hatch="//", alpha=0.6, label=None if i>0 else "failures")

        # Plot stacked cost bars
        ax_cost.bar(x + offset, costs_mean, width, color=color, alpha=0.8)
        ax_cost.bar(x + offset, fails_frac, width, bottom=costs_mean,
                    color="lightgray", hatch="//", alpha=0.6)

    # Labels
    ax_time.set_ylabel("Runtime [s]")
    ax_cost.set_ylabel("Cost [s]")
    ax_cost.set_xticks(x)
    ax_cost.set_xticklabels(plot_instances, rotation=45, ha='right')

    # Legend
    legend_patches = [mpatches.Patch(color=style["color"], label=planner) for planner, style in planners.items()]
    legend_patches.append(mpatches.Patch(facecolor="lightgray", hatch="//", label="failures", alpha=0.6))
    fig.legend(handles=legend_patches, ncol=len(planners)+1, loc='upper center', bbox_to_anchor=(0.5, 0.98))

    plt.tight_layout(rect=[0,0,1,0.95])
    plt.savefig("../results/results_plot_bar_chart.pdf", format="pdf", bbox_inches="tight")
    plt.show()


def plot_results_stacked_bar_chart(instances, num_trials):
    results_path = "../results_test"
    planners = {
        "db-cbs": {"color": "#88CCEE"},   
        "db-ecbs": {"color": "#009988"},  
        "db-pibt": {"color": "#E7B503"},  
        "db-lacam": {"color": "#993404"}     
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

    data = {p: {inst: {"time": [], "cost": []} for inst in instances} for p in planners}

    # collect results
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
                if "t" in first and "cost" in first:
                    if first["cost"] > 400:  # filter outliers
                        continue
                    data[planner][inst]["time"].append(first["t"])
                    data[planner][inst]["cost"].append(first["cost"])

    # map instance names
    plot_instances = [instance_map.get(inst, inst) for inst in instances]
    x = np.arange(len(instances))
    width = 0.6

    fig, (ax_time, ax_cost) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # stacked bars for runtime and cost
    for ax, metric in zip((ax_time, ax_cost), ("time", "cost")):
        bottoms = np.zeros(len(instances))
        for planner, style in planners.items():
            values = []
            for inst in instances:
                vals = data[planner][inst][metric]
                values.append(np.mean(vals) if vals else 0)
            ax.bar(x, values, width, bottom=bottoms,
                   color=style["color"], alpha=0.9, label=planner)
            bottoms += np.array(values)

        ax.set_ylabel("Runtime [s]" if metric == "time" else "Cost")

    ax_cost.set_xticks(x)
    ax_cost.set_xticklabels(plot_instances, rotation=45, ha='right')

    # legend
    legend_patches = [mpatches.Patch(color=style["color"], label=planner) for planner, style in planners.items()]
    fig.legend(handles=legend_patches, ncol=len(planners), loc='upper center', bbox_to_anchor=(0.5, 0.98))

    plt.tight_layout(rect=[0, 0, 1, 0.95])
    plt.savefig("../results/results_plot_stacked_bar_chart.pdf", format="pdf", bbox_inches="tight")
    plt.show()


def plot_results_bar_chart2(instances, num_trials):
    results_path = "../results_test"
    planners = {
        "db-cbs": {"color": "#88CCEE"},   
        "db-ecbs": {"color": "#009988"},  
        "db-pibt": {"color": "#E7B503"},  
        "db-lacam": {"color": "#993404"}     
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

    # collect results
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
                    if first["cost"] > 400:  # filter outliers
                        continue
                    data[planner][inst]["time"].append(first["t"])
                    data[planner][inst]["cost"].append(first["cost"])

    # map instance names
    plot_instances = [instance_map.get(inst, inst) for inst in instances]
    x = np.arange(len(instances))

    fig, (ax_fail, ax_time, ax_cost) = plt.subplots(
        3, 1, figsize=(12, 11), sharex=True,
        gridspec_kw={'height_ratios': [0.6, 1, 1]}
    )

    # === Failures subplot ===
    num_planners = len(planners)
    bar_width = 0.7 / num_planners  # grouped bars
    offsets = np.linspace(-0.35, 0.35, num_planners)

    for i, (planner, style) in enumerate(planners.items()):
        fail_rates = []
        for inst in instances:
            fails = data[planner][inst]["fail"]
            fail_rates.append(fails / num_trials)
        ax_fail.bar(x + offsets[i], fail_rates, bar_width,
                    color=style["color"], alpha=0.9, label=planner)

    ax_fail.set_ylabel("Failure Rate")
    ax_fail.set_ylim(0, 1.05)
    ax_fail.axhline(0.0, color="gray", linestyle="--", linewidth=0.8)

    # === Runtime and cost stacked bars ===
    width = 0.6
    for ax, metric in zip((ax_time, ax_cost), ("time", "cost")):
        bottoms = np.zeros(len(instances))
        for planner, style in planners.items():
            values = []
            for inst in instances:
                vals = data[planner][inst][metric]
                values.append(np.mean(vals) if vals else 0)
            ax.bar(x, values, width, bottom=bottoms,
                   color=style["color"], alpha=0.9, label=planner)
            bottoms += np.array(values)

        ax.set_ylabel("Runtime [s]" if metric == "time" else "Cost")

    # x-axis labels only at bottom
    ax_cost.set_xticks(x)
    ax_cost.set_xticklabels(plot_instances, rotation=45, ha='right')

    # === Legend ===
    legend_patches = [mpatches.Patch(color=style["color"], label=planner) for planner, style in planners.items()]
    fig.legend(handles=legend_patches, ncol=len(planners),
               loc='upper center', bbox_to_anchor=(0.5, 0.99))

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    # save as pdf
    plt.savefig("../results/results_plot_bar_chart2.pdf", format="pdf", bbox_inches="tight")
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
  instances.append("forest4")
  instances.append("corridor4")
  instances.append("circle6")
  instances.append("circle7_swap")
  instances.append("passage6")

                   
  num_trials = 5  # max number of trials per instance
  plot_results(instances, num_trials, True)
#   plot_results_bar_chart(instances, num_trials)
#   plot_results_bar_chart2(instances, num_trials)
#   plot_results_stacked_bar_chart(instances, num_trials)


