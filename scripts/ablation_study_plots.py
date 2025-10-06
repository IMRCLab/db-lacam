import yaml
import numpy as np
import matplotlib.pyplot as plt

def clustering_analysis(a, i, itr):
    folder = "/home/akmarak-laptop/IMRC/db-lacam/results/ICAPS26/clustering/"
    t_values = {key: [] for key in a}
    cost_values = {key: [] for key in a}
    colors = ['#4477AA', '#CCBB44']
    labels = ['GOC', 'SC-GOC']
    i_map = {
          'gen_p10_n8_0_unicycle_sphere': "rand-n8-u\u209B-1",
          'gen_p10_n8_1_unicycle_sphere': "rand-n8-u\u209B-2",
          'gen_p10_n8_2_unicycle_sphere': "rand-n8-u\u209B-3",
          'gen_p10_n8_3_unicycle_sphere': "rand-n8-u\u209B-4",
          'gen_p10_n8_4_unicycle_sphere': "rand-n8-u\u209B-5",
          'gen_p10_n8_5_unicycle_sphere': "rand-n8-u\u209B-6",
          'gen_p10_n8_6_unicycle_sphere': "rand-n8-u\u209B-7",
          'gen_p10_n8_7_unicycle_sphere': "rand-n8-u\u209B-8",
          'gen_p10_n8_8_unicycle_sphere': "rand-n8-u\u209B-9",
          'gen_p10_n8_9_unicycle_sphere': "rand-n8-u\u209B-10",
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
                alpha=0.8, 
                label=labels[j] if idx == 0 else None  # legend only once
            )
        ax[idx].set_ylabel(ylabel, fontsize=font_size)
        ax[idx].grid(axis="x", linestyle="dashed")

    ax[0].legend(fontsize = font_size-7)
    ax[-1].set_xticks(x + bar_width / 2)
    ax[-1].set_xticklabels([i_map[key] for key in i], rotation=45, ha="right", fontsize=font_size)
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
    ax.grid(axis="x", linestyle="dashed")
    ax.legend(fontsize=font_size-2)

    plt.tight_layout()
    # plt.show()
    plt.savefig("../results/ICAPS26/heuristic-estimation/heuristic_estimation_analysis.pdf", format="pdf", bbox_inches="tight") 

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
    a = ["reverse-search", "est"]
    i = [
        'circle2_unicycle',
        'circle4_unicycle',
        'circle6_unicycle',
        'circle8_unicycle',
        'circle10_unicycle',
    ]
    heuristic_estimation_analysis(a, i, 5)

if __name__ == "__main__":
  main()