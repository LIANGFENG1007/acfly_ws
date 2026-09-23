#!/usr/bin/env python3
"""串行跑一组仿真试验并逐轮出报告(追加 sim_runs/LOG.md)。

用法：campaign.py "说明" 名称:布局[:二进制[:参数yaml]] ...
  二进制/参数留空表示用当前安装的规划器 / profiles/sim_arena_planner.yaml。
  例：campaign.py "续航参数A" r11_L1:L1_sparse4::tools/sim/profiles/var_a.yaml
Python 在启动时整体读入本文件，运行中修改它不影响正在执行的批次(bash 脚本会)。
"""
import subprocess
import sys
from pathlib import Path

ROOT = Path("/home/yk/acfly_ws")
DEFAULT_PARAMS = "tools/sim/profiles/sim_arena_planner.yaml"


def main():
    note = sys.argv[1]
    for item in sys.argv[2:]:
        parts = (item.split(":") + ["", "", ""])[:4]
        name, layout, binary, params = parts
        params = params or DEFAULT_PARAMS
        command = ["/usr/bin/python3", "tools/sim/run_trial.py", "--seed", "1",
                   "--layout", f"tools/sim/layouts/{layout}.json", "--duration", "420",
                   "--domain", "87", "--headless", "--stack-launch",
                   "--planner-params", params, "--output", f"sim_runs/{name}"]
        label = note
        if binary:
            command += ["--planner-binary", binary]
            label += f" [bin {Path(binary).name}]"
        if params != DEFAULT_PARAMS:
            label += f" [params {Path(params).stem}]"
        with open(ROOT / f"sim_runs/.{name}.out", "w") as log:
            code = subprocess.run(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT).returncode
        print(f"{name} exit={code}", flush=True)
        subprocess.run(["/usr/bin/python3", "tools/sim/trial_report.py", f"sim_runs/{name}", label], cwd=ROOT)
        sys.stdout.flush()


if __name__ == "__main__":
    main()
