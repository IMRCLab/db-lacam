from dataclasses import dataclass
import subprocess
from pathlib import Path
import yaml
import shutil
import psutil
import tqdm
import multiprocessing as mp
from main_dbpibt import run_dbpibt
from main_dbcbs import run_dbcbs
from main_dbecbs import run_dbecbs
from main_dblacam import run_dblacam
from benchmark_table import write_table
from benchmark_stats import run_benchmark_stats
import paper_tables

@dataclass
class ExecutionTask:
  """Class for keeping track of an item in inventory."""
  # env: Path
  # cfg: Path
  # result_folder: Path
  instance: str
  alg: str
  trial: int
  timelimit: float
  
def run_visualize(script, filename_env, filename_result):
  subprocess.run(["python3",
        script,
        filename_env,
        "--result", filename_result,
        "--video", filename_result.with_suffix(".mp4")])  
  

def execute_task(task: ExecutionTask):
  scripts_path = Path("../scripts")
  results_path = Path("../results")
  env_path = Path().resolve() / "../example"
  env = (env_path / task.instance).with_suffix(".yaml") 
  assert(env.is_file())

  cfg = env_path / "algorithms.yaml" # using single alg.yaml
  assert(cfg.is_file())

  with open(cfg) as f:
    cfg = yaml.safe_load(f)

  result_folder = results_path / task.instance / task.alg / "{:03d}".format(task.trial)
  if result_folder.exists():
      print("Warning! {} exists already. Deleting...".format(result_folder))
      shutil.rmtree(result_folder)
  result_folder.mkdir(parents=True, exist_ok=False)

  # find cfg
  mycfg = cfg[task.alg]
  mycfg = mycfg['default']
  # wildcard matching
  import fnmatch
  for k, v in cfg[task.alg].items():
    if fnmatch.fnmatch(Path(task.instance).name, k):
      mycfg = {**mycfg, **v} # merge two dictionaries

  if Path(task.instance).name in cfg[task.alg]:
    mycfg_instance = cfg[task.alg][Path(task.instance).name]
    mycfg = {**mycfg, **mycfg_instance} # merge two dictionaries

  for k, v in cfg[task.alg].items():
      if "*" in k and fnmatch.fnmatch(Path(task.instance), k):
          mycfg |= v  

  if Path(task.instance) in cfg[task.alg]:
      mycfg |= cfg[task.alg][Path(task.instance)]

  print("Using configurations ", mycfg)

  if task.alg == "db-pibt":
    run_dbpibt(str(env), str(result_folder), task.timelimit, mycfg)
    visualize_files = [p.name for p in result_folder.glob('result_*')]
  elif task.alg == "db-cbs":
    run_dbcbs(str(env), str(result_folder), task.timelimit, mycfg)
    visualize_files = [p.name for p in result_folder.glob('result_*')]
  elif task.alg == "db-ecbs":
    run_dbecbs(str(env), str(result_folder), task.timelimit, mycfg)
    visualize_files = [p.name for p in result_folder.glob('result_*')]
  elif task.alg == "db-lacam":
    run_dblacam(str(env), str(result_folder), task.timelimit, mycfg)
    visualize_files = [p.name for p in result_folder.glob('result_*')]
    
  # vis_script = scripts_path / "visualize.py"
  # for file in visualize_files:
  #   run_visualize(vis_script, env, result_folder / file)
      
def main():
  parallel = True
  instances = [
    # "circle2_integrator",
    # "circle4_integrator",
    # "circle6_integrator",
    # "circle8_integrator",
    # "circle10_integrator",
    # "circle2_unicycle",
    # "circle4_unicycle",
    # "circle6_unicycle",
    # "circle8_unicycle",
    # "circle10_unicycle",
    # "alcove_unicycle",
    # "atgoal_unicycle",
    # 3D case
    # "forest4",
    # "corridor4",
    # "circle6",
    # "circle7_swap",
    # "passage6",
    # "passage10",
    # scalability test
    # "test_n10_0_unicycle",
    # "test_n20_0_unicycle",
    # "test_n30_0_unicycle",
    # "test_n40_0_unicycle",
    # "test_n50_0_unicycle",
  ]
  for kind in ["unicycle_sphere"]: #"unicycle",
    for n in [8]:
      for k in range(10):
        instances.append("gen_p10_n{}_{}_{}".format(n,k, kind))

  instances_n = ["test_n10_0_unicycle", "test_n20_0_unicycle", "test_n30_0_unicycle", "test_n40_0_unicycle", "test_n50_0_unicycle"] 

  algs = [
    # "db-cbs",
    # "db-ecbs",
    # "db-pibt",
    "db-lacam",
  ]
  trials = 1 * 5
  timelimit_2d = 1 * 60 
  timelimit_n = 5 * 60 # scalability 
  timelimit_max = 0 # plot needs higher timelimit
  tasks = []
  for instance in instances:
    if instance in instances_n:
        timelimit = timelimit_n
    else:
        timelimit = timelimit_2d
    timelimit_max = max(timelimit_max, timelimit)
    for alg in algs:
      for trial in range(trials):
        tasks.append(ExecutionTask(instance, alg, trial, timelimit))

  if parallel and len(tasks) > 1:
    use_cpus = psutil.cpu_count(logical=False)-1
    print("Using {} CPUs".format(use_cpus))
    with mp.Pool(use_cpus) as p:
      for _ in tqdm.tqdm(p.imap_unordered(execute_task, tasks)):
        pass
  else:
    for task in tasks:
      execute_task(task)

  run_benchmark_stats(instances, algs, trials, timelimit_max)
  write_table(instances, algs, Path("../results"), "table.pdf", trials, timelimit_max)
  # paper_tables.write_table_2d(trials, timelimit)
  # paper_tables.write_table_3d(trials, timelimit)
  # paper_tables.write_table_scalability(trials, timelimit)


if __name__ == '__main__':
  main()