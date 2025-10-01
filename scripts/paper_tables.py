from pathlib import Path
import benchmark_table
from benchmark_table import write_table

def write_table_2d(trials, timelimit):
  instances = [
    "alcove_unicycle",
    "atgoal_unicycle",
    "circle2_unicycle",
    "circle4_unicycle",
    "circle8_unicycle",
    "circle10_unicycle",

    "<<HLINE>>",

    "gen_p10_n2_*_unicycle_sphere",
    "gen_p10_n4_*_unicycle_sphere",
    "gen_p10_n8_*_unicycle_sphere",

    "<<HLINE>>",

    "gen_p10_n2_*_unicycle",
    "gen_p10_n4_*_unicycle",
    "gen_p10_n8_*_unicycle",
  ]

  algs = [
    "db-cbs",
    "db-ecbs",
    "db-pibt",
    "db-lacam",
  ]

  instance_names = {
    'alcove_unicycle': "alcove",
    'atgoal_unicycle': "at goal",
    'circle2_unicycle': "circle (N=2)",
    'circle4_unicycle': "circle (N=4)",
    'circle8_unicycle': "circle (N=8)",
    'circle10_unicycle': "circle (N=8)",
    'gen_p10_n2_*_unicycle_sphere': "rand sphere(N=2)",
    'gen_p10_n4_*_unicycle_sphere': "rand sphere(N=4)",
    'gen_p10_n8_*_unicycle_sphere': "rand sphere(N=8)",
    'gen_p10_n2_*_hetero': "rand (N=2)",
    'gen_p10_n4_*_hetero': "rand (N=4)",
    'gen_p10_n8_*_hetero': "rand (N=8)",
    
  }

  alg_names = {
    "db-cbs": "db-CBS",
    "db-ecbs": "db-ECBS",
    "db-pibt": "db-PIBT",
    "db-lacam": "db-LaCAM",
  }

  result = benchmark_table.compute_results(instances, algs, Path("../results/"), trials, timelimit, True)
  output_path = Path("../results/paper_table_2d.pdf")
  with open(output_path.with_suffix(".tex"), "w") as f:

    f.write(r"\documentclass{standalone}")
    f.write("\n")
    f.write(r"\begin{document}")
    f.write("\n")
    f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")

    out = r"\begin{tabular}{c || c"
    for alg in algs:
      if (alg == "db-ecbs" or alg == "db-lacam"):
        out += r" || r|r|r|r"
      else:
        out += r" || r|r|r"
    out += "}\n"
    f.write(out)
    out = r"\# & Instance"
    for k, alg in enumerate(algs):
      if k == len(algs) - 1:
        if (alg == "db-ecbs" or alg == "db-lacam"):
          out += r" & \multicolumn{4}{c}{"
        else:
          out += r" & \multicolumn{3}{c}{"
      else:
        if (alg == "db-ecbs" or alg == "db-lacam"):
          out += r" & \multicolumn{4}{c||}{"
        else:
          out += r" & \multicolumn{3}{c||}{"
      out += alg_names[alg]
      out += r"}"
    out += r"\\"
    f.write(out)
    out = r"& "
    for alg in algs:
      if (alg == "db-ecbs" or alg == "db-lacam"):
          out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st}} [s]$ & $J^{f} [s]$"
      else:
        out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st},f} [s]$"
    out += r"\\"
    f.write(out)
    f.write(r"\hline")

    r_number = 0
    for instance in instances:

      if instance == "<<HLINE>>":
        f.write(r"\hline")
        f.write("\n")
        continue

      out = ""
      out += r"\hline"
      out += "\n"
      out += "{} & ".format(r_number+1)
      if instance in instance_names:
        out += instance_names[instance]
      else:
        out += "{} ".format(instance.replace("_", "\_"))

      for alg in algs:

        out = benchmark_table.print_and_highlight_best_max(out, 'success', result[instance], alg, algs)
        out = benchmark_table.print_and_highlight_best(out, 't^st_mean', result[instance], alg, algs)
        out = benchmark_table.print_and_highlight_best(out, 'J^st_mean', result[instance], alg, algs)
        # out = benchmark_table.print_and_highlight_best(out, 'Jr^st_median', result[instance], alg, algs, digits=0) // without notion of regret
        if (alg == "db-ecbs" or alg == "db-lacam"):
          out = benchmark_table.print_and_highlight_best(out, 'J^f_mean', result[instance], alg, algs)
      out += r"\\"
      f.write(out)
      r_number += 1

    f.write("\n")
    f.write(r"\end{tabular}")
    f.write("\n")
    f.write(r"\end{document}")

  benchmark_table.gen_pdf(output_path)

def write_table_3d(trials, timelimit):
  instances = [
    "forest4",
    "corridor4",
    "circle6",
    "circle7_swap",
    "passage6",
    # "passage10",
  ]

  algs = [
    "db-cbs",
    "db-ecbs",
    "db-pibt",
    "db-lacam",
  ]

  instance_names = {
    'forest4': "forest (N=4)",
    'corridor4': "corridor (N=4)",
    'circle6': "circle (N=6)",
    'circle7_swap': "circle_swap (N=7)",
    'passage6': "passage (N=6)",
    # 'passage10': "passage (N=10)",
  }

  alg_names = {
    "db-cbs": "db-CBS",
    "db-ecbs": "db-ECBS",
    "db-pibt": "db-PIBT",
    "db-lacam": "db-LaCAM",
  }

  result = benchmark_table.compute_results(instances, algs, Path("../results/"), trials, timelimit, True)
  output_path = Path("../results/paper_table_3d.pdf")
  with open(output_path.with_suffix(".tex"), "w") as f:

    f.write(r"\documentclass{standalone}")
    f.write("\n")
    f.write(r"\begin{document}")
    f.write("\n")
    f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")

    out = r"\begin{tabular}{c || c"
    for alg in algs:
      if (alg == "db-ecbs" or alg == "db-lacam"):
        out += r" || r|r|r|r"
      else:
        out += r" || r|r|r"
    out += "}\n"
    f.write(out)
    out = r"\# & Instance"
    for k, alg in enumerate(algs):
      if k == len(algs) - 1:
        if (alg == "db-ecbs" or alg == "db-lacam"):
          out += r" & \multicolumn{4}{c}{"
        else:
          out += r" & \multicolumn{3}{c}{"
      else:
        if (alg == "db-ecbs" or alg == "db-lacam"):
          out += r" & \multicolumn{4}{c||}{"
        else:
          out += r" & \multicolumn{3}{c||}{"
      out += alg_names[alg]
      out += r"}"
    out += r"\\"
    f.write(out)
    out = r"& "
    for alg in algs:
      if (alg == "db-ecbs" or alg == "db-lacam"):
        out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st}} [s]$ & $J^{f} [s]$"
      else:
        out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st},f} [s]$"
    out += r"\\"
    f.write(out)
    f.write(r"\hline")

    r_number = 0
    for instance in instances:

      if instance == "<<HLINE>>":
        f.write(r"\hline")
        f.write("\n")
        continue

      out = ""
      out += r"\hline"
      out += "\n"
      out += "{} & ".format(r_number+1)
      if instance in instance_names:
        out += instance_names[instance]
      else:
        out += "{} ".format(instance.replace("_", "\_"))

      for alg in algs:

        out = benchmark_table.print_and_highlight_best_max(out, 'success', result[instance], alg, algs)
        out = benchmark_table.print_and_highlight_best(out, 't^st_mean', result[instance], alg, algs)
        out = benchmark_table.print_and_highlight_best(out, 'J^st_mean', result[instance], alg, algs)
        # out = benchmark_table.print_and_highlight_best(out, 'Jr^st_median', result[instance], alg, algs, digits=0) // without notion of regret
        if (alg == "db-ecbs" or alg == "db-lacam"):
          out = benchmark_table.print_and_highlight_best(out, 'J^f_mean', result[instance], alg, algs)
      out += r"\\"
      f.write(out)
      r_number += 1

    f.write("\n")
    f.write(r"\end{tabular}")
    f.write("\n")
    f.write(r"\end{document}")

  benchmark_table.gen_pdf(output_path)

def write_table_scalability(trials, timelimit):
  instances = [
    "test_n10_0_unicycle",
    "test_n20_0_unicycle",
    "test_n30_0_unicycle",
    "test_n40_0_unicycle",
    "test_n50_0_unicycle",
  ]

  algs = [
    "db-cbs",
    "db-ecbs",
    "db-pibt",
    "db-lacam",
  ]

  instance_names = {
    'test_n10_0_unicycle': "rand (N=10)",
    'test_n20_0_unicycle': "rand (N=20)",
    'test_n30_0_unicycle': "rand (N=30)",
    'test_n40_0_unicycle': "rand (N=40)",
    'test_n50_0_unicycle': "rand (N=50)",
  }

  alg_names = {
    "db-cbs": "db-CBS",
    "db-ecbs": "db-ECBS",
    "db-pibt": "db-PIBT",
    "db-lacam": "db-LaCAM",
  }

  result = benchmark_table.compute_results(instances, algs, Path("../results/"), trials, timelimit, True)
  output_path = Path("../results/paper_table_scalability.pdf")
  with open(output_path.with_suffix(".tex"), "w") as f:

    f.write(r"\documentclass{standalone}")
    f.write("\n")
    f.write(r"\begin{document}")
    f.write("\n")
    f.write(r"% GENERATED - DO NOT EDIT - " + output_path.name + "\n")

    out = r"\begin{tabular}{c || c"
    for alg in algs:
      if (alg == "db-lacam"):
        out += r" || r|r|r|r"
      else:
        out += r" || r|r|r"
    out += "}\n"
    f.write(out)
    out = r"\# & Instance"
    for k, alg in enumerate(algs):
      if k == len(algs) - 1:
        if (alg == "db-lacam"):
          out += r" & \multicolumn{4}{c}{"
        else:
          out += r" & \multicolumn{3}{c}{" 
      else:
        if (alg == "db-lacam"):
          out += r" & \multicolumn{4}{c||}{"
        else:
          out += r" & \multicolumn{3}{c||}{"
      out += alg_names[alg]
      out += r"}"
    out += r"\\"
    f.write(out)
    out = r"& "
    for alg in algs:
      if (alg == "db-lacam"):
        out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st}} [s]$ & $J^{f} [s]$"
      else:
        out += r" & $p$ & $t^{\mathrm{st}} [s]$ & $J^{\mathrm{st},f} [s]$"
    out += r"\\"
    f.write(out)
    f.write(r"\hline")

    r_number = 0
    for instance in instances:

      if instance == "<<HLINE>>":
        f.write(r"\hline")
        f.write("\n")
        continue

      out = ""
      out += r"\hline"
      out += "\n"
      out += "{} & ".format(r_number+1)
      if instance in instance_names:
        out += instance_names[instance]
      else:
        out += "{} ".format(instance.replace("_", "\_"))

      for alg in algs:

        out = benchmark_table.print_and_highlight_best_max(out, 'success', result[instance], alg, algs)
        out = benchmark_table.print_and_highlight_best(out, 't^st_mean', result[instance], alg, algs)
        out = benchmark_table.print_and_highlight_best(out, 'J^st_mean', result[instance], alg, algs)
        if (alg == "db-lacam"):
          out = benchmark_table.print_and_highlight_best(out, 'J^f_mean', result[instance], alg, algs)
      out += r"\\"
      f.write(out)
      r_number += 1

    f.write("\n")
    f.write(r"\end{tabular}")
    f.write("\n")
    f.write(r"\end{document}")

  benchmark_table.gen_pdf(output_path)
def main():
  trials = 1
  timelimit = 2*60
  write_table_2d(trials, timelimit) 
  # write_table_3d(trials, timelimit) 
  # write_table_scalability(trials, timelimit) 

if __name__ == '__main__':
    main()