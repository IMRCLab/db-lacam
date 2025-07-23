import subprocess
import time
import tempfile
from pathlib import Path
import sys
import os
import yaml
sys.path.append(os.getcwd())

def run_dblacam(filename_env, folder, timelimit, cfg):
    with tempfile.TemporaryDirectory() as tmpdirname:
        p = Path(tmpdirname)
        filename_cfg = p / "cfg.yaml"
        with open(filename_cfg, 'w') as f:
            yaml.dump(cfg, f, Dumper=yaml.CSafeDumper)
        filename_stats = "{}/stats.yaml".format(folder)
        duration_dblacam = 0
        with open(filename_stats, 'w') as stats:
            stats.write("stats:\n")
            
            filename_result_dblacam = Path(folder) / "result_dblacam.yaml"
            filename_stats = Path(folder) / "stats.yaml"
            start = time.time()
            cmd = ["./run_dblacam", 
                "-i", filename_env,
                "-o", filename_result_dblacam,
                "--cfg", str(filename_cfg),
                "-t", str(1e6)]
            print(subprocess.list2cmdline(cmd))
            try:
                with open("{}/log.txt".format(folder), 'w') as logfile:
                    result = subprocess.run(cmd, timeout=timelimit, stdout=logfile, stderr=logfile)
                stop = time.time()
                duration_dblacam += stop - start
                if result.returncode != 0:
                    print("db-lacam failed ", result.returncode)
                else:
                    cost = 0
                    with open(filename_result_dblacam) as f:
                        result = yaml.safe_load(f)
                        for r in result["result"]:
                            cost += len(r["actions"]) * 0.1 # time step = 0.1
        
                    now = time.time()
                    t = now - start
                    print("success!", cost, t)
                    stats.write("  - t: {}\n".format(t))
                    stats.write("    cost: {}\n".format(cost))
                    stats.write("    duration_dblacam: {}\n".format(duration_dblacam))
                    stats.flush()

            except Exception as e:
                print(f"An unexpected error occurred: {e}")


