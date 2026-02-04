from pathlib import Path
import yaml
import numpy as np
import subprocess

def compute_results(instances, algs, results_path, trials, T, regret=False):
  all_result = dict()

  if isinstance(trials, int):
    trials = [trials]*len(instances)

  for instance, itrials in zip(instances, trials):
    result = dict()
    for alg in algs:
      if not regret:
        result_folder = results_path / instance / alg
        stat_files = [str(p) for p in result_folder.glob("**/stats.yaml")]
      else:
        stat_files = [str(p) for p in results_path.glob(instance + "/"+alg+"/**/stats.yaml")]
      initial_time_regrets = []
      final_regrets = []

      # load data
      initial_times = []
      initial_time_regrets = []
      initial_costs = []
      initial_regrets = []
      final_costs = []
      final_regrets = []

      for stat_file in stat_files:
        final_cost_base = None
        initial_time_base = None
        if regret:
          stat_file_base = stat_file.replace(alg, "db-pibt")
          if Path(stat_file_base).exists():
            with open(stat_file_base) as sf:
              stats = yaml.safe_load(sf)
            if stats is not None and "stats" in stats and stats["stats"] is not None:
              for k, d in enumerate(stats["stats"]):
                # skip results that were after our time horizon
                if d["t"] > T:
                  break
                if k == 0:
                  initial_time_base = d["t"]
                final_cost_base = d["cost"]

        with open(stat_file) as sf:
          stats = yaml.safe_load(sf)
        if stats is not None and "stats" in stats and stats["stats"] is not None:
          last_cost = None
          for k, d in enumerate(stats["stats"]):
            # skip results that were after our time horizon
            if d["t"] > T:
              break
            if k == 0:
              initial_times.append(d["t"])
              initial_costs.append(d["cost"])
              if initial_time_base is not None:
                initial_time_regrets.append((d["t"] - initial_time_base)/d["t"] * 100)
                initial_regrets.append((d["cost"] - final_cost_base)/d["cost"] * 100)

            last_cost = d["cost"]
          if last_cost is not None:
            final_costs.append(last_cost)
          if last_cost is not None and final_cost_base is not None:
            final_regrets.append((last_cost - final_cost_base)/last_cost * 100)
      # compute for the currently considered algorithm      
      result[alg] = {
        'success': len(initial_times)/itrials,
        't^st_mean': np.mean(initial_times) if len(initial_times) > 0 else None,
        'tr^st_mean': np.mean(initial_time_regrets) if len(initial_time_regrets) > 0 else None,
        'J^st_mean': np.mean(initial_costs) if len(initial_costs) > 0 else None,
        'Jr^st_mean': np.mean(initial_regrets) if len(initial_regrets) > 0 else None,
        'J^f_mean': np.mean(final_costs) if len(final_costs) > 0 else None,
        'Jr^f_mean': np.mean(final_regrets) if len(final_regrets) > 0 else None,
      }

      if alg == "s2m2" and len(initial_times) == 0 and "unicycle_sphere" not in instance:
        for key in result[alg].keys():
          result[alg][key] = '*'

    all_result[instance] = result
  return all_result

# support std, mean
# supports std, energy cost also
def compute_results_with_std(instances, algs, results_path, trials, T):
  all_result = dict()

  if isinstance(trials, int):
    trials = [trials]*len(instances)

  for instance, itrials in zip(instances, trials):
    result = dict()
    for alg in algs:
      result_folder = results_path / instance / alg
      stat_files = [str(p) for p in result_folder.glob("**/stats.yaml")]
      initial_time_regrets = []

      # load data
      initial_times = []
      initial_time_regrets = []
      initial_costs = []
      final_costs = []

      for stat_file in stat_files:
        final_cost_base = None
        initial_time_base = None
      
        with open(stat_file) as sf:
          stats = yaml.safe_load(sf)
        if stats is not None and "stats" in stats and stats["stats"] is not None:
          last_cost = None
          for k, d in enumerate(stats["stats"]):
            # skip results that were after our time horizon
            if d["t"] > T:
              break
            if k == 0:
              initial_times.append(d["t"])
              initial_costs.append(d["cost"])
              if initial_time_base is not None:
                initial_time_regrets.append((d["t"] - initial_time_base)/d["t"] * 100)

            last_cost = d["cost"]
          
          if last_cost is not None:
            final_costs.append(last_cost)

      success_rate = len(initial_times)/itrials # success rate is rounded up. Numbers < 0.5 are zeroed for the table
      result[alg] = {
        'success': len(initial_times)/itrials,
        't^st_mean': np.mean(initial_times) if success_rate > 0.05 else None,
        't^st_std': np.std(initial_times) if success_rate > 0.05 else None,

        'J^st_mean': np.mean(initial_costs) if success_rate > 0.05 else None,
        'J^st_std': np.std(initial_costs) if success_rate > 0.05 else None,

        'J^f_mean': np.mean(final_costs) if success_rate > 0.05 else None,
        'J^f_std': np.std(final_costs) if success_rate > 0.05 else None,
      }
      
    all_result[instance] = result
  return all_result

def gen_pdf(output_path):
  # run pdflatex
  subprocess.run(['pdflatex', output_path.with_suffix(".tex")], check=True, cwd=output_path.parent)
  # delete temp files
  output_path.with_suffix(".aux").unlink()
  output_path.with_suffix(".log").unlink()
  

# add std into the table
def generate_latex_row_cells(result, alg, algs, keys, digits=1, show_std=True, is_anytime=False):
    def format_val_std(val, std, is_best):
      if val == "*":
          return r"\ensuremath{\star}"
      elif val is None:
          return r"\textemdash"

      # main value
      if is_best:
          out = r"\ensuremath{\mathbf{" + f"{val:.{digits}f}" + r"}}"
      else:
          out = r"\ensuremath{" + f"{val:.{digits}f}" + r"}"

      # std
      if show_std and std is not None:
          try:
              std_f = float(std)
              out += r" {\scriptsize\textcolor{gray}{\ensuremath{\pm " + f"{std_f:.{digits}f}" + r"}}}"
          except:
              pass

      return out

    row = ""
    for key in keys:
        row += " & "

        def is_best_min(val, key):
            return val is not None and val != "*" and all(
                round(val, digits) <= round(result[a].get(key), digits)
                for a in algs if result[a].get(key) not in [None, "*"]
            )

        if key in ['J^st_mean', 'J^f_mean']:
            val = result[alg].get(key)
            std = result[alg].get(key.replace('_mean', '_std'))
            top = format_val_std(val, std, is_best_min(val, key))

            # secondary metric (E / Er)
            if key.startswith('Jr'):
                key2 = 'Er^st_mean' if is_anytime else 'Er^f_mean'
            else:
                key2 = 'E^st_mean' if is_anytime else 'E^f_mean'

            val2 = result[alg].get(key2)
            std2 = result[alg].get(key2.replace('_mean', '_std'))

            if val2 is not None:
                bottom = format_val_std(val2, std2, is_best=False)
                row += r"\begin{tabular}[t]{@{}l@{}}" + top + r" \\" + bottom + r"\end{tabular}"
            else:
                row += r"\begin{tabular}[t]{@{}l@{}}" + top + r"\end{tabular}"

        else:
            val = result[alg].get(key)
            if key == 'success':
                is_best = val is not None and val != "*" and all(
                    round(val, digits) >= round(result[a].get(key), digits)
                    for a in algs if result[a].get(key) not in [None, "*"]
                )
                row += format_val_std(val, None, is_best)
            else:
                std = result[alg].get(key.replace('_mean', '_std'))
                row += format_val_std(val, std, is_best_min(val, key))

    return row


def print_and_highlight_best(out, key, result, alg, algs, digits=1):
  out += " & "
  is_best = False
  if result[alg][key] != None and result[alg][key] != "*":
    # we only look at one digit
    is_best = np.array([round(result[alg][key],1) <= round(result[other][key],1) for other in algs if result[other][key] != None and result[other][key] != "*"]).all()
  if is_best:
    out += r"\bfseries "
  if result[alg][key] == "*":
    out += r"$\star$"
  elif result[alg][key] is not None:
    out += ("{:."+str(digits)+"f}").format(result[alg][key])
  else:
    out += r"\textemdash"
  return out

def print_and_highlight_best_max(out, key, result, alg, algs, digits=1):
  out += " & "
  is_best = False
  if result[alg][key] != None and result[alg][key] != "*":
    # we only look at one digit
    is_best = np.array([round(result[alg][key],1) >= round(result[other][key],1) for other in algs if result[other][key] != None and result[other][key] != "*"]).all()
  if is_best:
    out += r"\bfseries "
  if result[alg][key] == "*":
    out += r"$\star$"
  elif result[alg][key] is not None:
    out += ("{:."+str(digits)+"f}").format(result[alg][key])
  else:
    out += r"\textemdash"
  return out

def get_alg_name(alg_key):
  # all algorithms we consider so far
  mapping = {
    "db-cbs": "db-CBS",
    "db-ecbs": "db-ECBS",
    "db-pibt": "db-PIBT",
    "db-lacam": "db-LaCAM",
  }

  if alg_key in mapping:
    return mapping[alg_key]
  return alg_key.upper().replace("-", "_")

def write_table(rows, algs, results_path, fname, trials, T, regret=False):

  result = compute_results(rows, algs, results_path, trials, T, regret)
  output_path = Path(results_path) / Path(fname)
  with open(output_path.with_suffix(".tex"), "w") as f:

    f.write(r"\documentclass{standalone}")
    f.write("\n")
    f.write(r"\begin{document}")
    f.write("\n")
    f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")

    alg_names = {key: get_alg_name(key) for key in algs}

    out = r"\begin{tabular}{c || c"
    for alg in algs:
      if (alg == "db-ecbs" or alg == "db-lacam") and not regret:
        out += r" || r|r|r|r"
      else:
        out += r" || r|r|r"
    out += "}\n"
    f.write(out)
    out = r"\# & Instance"
    for k, alg in enumerate(algs):
      if k == len(algs) - 1:
        if (alg == "db-ecbs" or alg == "db-lacam") and not regret:
          out += r" & \multicolumn{4}{c}{"
        else:
          out += r" & \multicolumn{3}{c}{"
      else:
        if (alg == "db-ecbs" or alg == "db-lacam") and not regret:
          out += r" & \multicolumn{4}{c||}{"
        else:
          out += r" & \multicolumn{3}{c||}{"
      out += alg_names[alg]
      out += r"}"
    out += r"\\"
    f.write(out)
    out = r"& "
    if not regret:
      for alg in algs:
        if (alg == "db-ecbs" or alg == "db-lacam"):
          out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st}} [s]$ & $J^{f} [s]$"
        else:
          out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st},f} [s]$"
    else:
      for alg in algs:
        out += r" & $p$ & $t_r^{\mathrm{st}} [\%]$ & $J_r^{f} [\%]$"
    out += r"\\"
    f.write(out)
    f.write(r"\hline")

    for r_number, row in enumerate(rows):

      out = ""
      out += r"\hline"
      out += "\n"
      out += "{} & ".format(r_number+1)
      out += "{} ".format(row.replace("_", "\_"))

      for alg in algs:

        if not regret:
          out = print_and_highlight_best_max(out, 'success', result[row], alg, algs)
          out = print_and_highlight_best(out, 't^st_mean', result[row], alg, algs)
          out = print_and_highlight_best(out, 'J^st_mean', result[row], alg, algs)
          if (alg == "db-ecbs" or alg == "db-lacam"):
            out = print_and_highlight_best(out, 'J^f_mean', result[row], alg, algs)
        else:
          out = print_and_highlight_best_max(out, 'success', result[row], alg, algs)
          out = print_and_highlight_best(out, 'tr^st_mean', result[row], alg, algs)
          out = print_and_highlight_best(out, 'Jr^f_mean', result[row], alg, algs)

      out += r"\\"
      f.write(out)

    f.write("\n")
    f.write(r"\end{tabular}")
    f.write("\n")
    f.write(r"\end{document}")

  # run pdflatex
  gen_pdf(output_path)

def main():
  results_path = Path("../results")
  trial = 5
  T = 2 * 60
  rows = [
     "circle2_integrator",
    "circle4_integrator",
    "circle6_integrator",
    "circle8_integrator",
    "circle10_integrator",
    "circle2_unicycle",
    "circle4_unicycle",
    "circle6_unicycle",
    "circle8_unicycle",
    "circle10_unicycle",
    # difficult cases
    "alcove_unicycle",
    "atgoal_unicycle",
    "swap2_unicycle",
    "swap3_unicycle",
    "swap4_unicycle",
  ]
  algs = [
    "db-cbs",
    "db-pibt",
    "db-lacam",
  ]

  write_table(rows, algs, results_path, "table.pdf", trial, T)

if __name__ == '__main__':
  main()