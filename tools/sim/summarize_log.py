#!/usr/bin/env python3
"""按"规划器版本 × 布局"聚合 sim_runs/LOG.md：全程用时均值/次数、倾角高频、前进反复、失败数。

版本取自每行备注里的 [bin planner_vN]；基线 = r01 快照二进制(备注含 exploration_planner_node 或早期 r01/r07/r09/r15)。
用法：summarize_log.py [--versions v10 v15 v16]
"""
import argparse
import collections
import re
from pathlib import Path

LOG = Path(__file__).resolve().parents[2] / "sim_runs/LOG.md"
BASE_EARLY = {"r01_L1_base", "r07_L3base", "r09_L2base", "r15_L4base"}


def version_of(name, note):
    m = re.search(r"planner_(v\d+)", note)
    if m:
        return m.group(1)
    if name in BASE_EARLY or "exploration_planner_node" in note:
        return "基线"
    return None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--versions", nargs="*")
    args = parser.parse_args()
    data = collections.defaultdict(list)
    for line in LOG.read_text().splitlines():
        if not line.startswith("| r"):
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        name, layout, status, _, total = cells[0], cells[1], cells[2], cells[3], cells[4]
        version = version_of(name, cells[-1])
        if version is None:
            continue

        def num(i):
            try:
                return float(cells[i])
            except ValueError:
                return None
        data[(version, layout)].append({"ok": status == "completed", "total": num(4), "tilt": num(11), "rev": num(8)})
    versions = sorted({v for v, _ in data}, key=lambda v: (v != "基线", int(v[1:]) if v[1:].isdigit() else 0))
    if args.versions:
        versions = [v for v in versions if v in args.versions or v == "基线"]
    layouts = sorted({l for _, l in data})
    print(f"{'布局':14s}" + "".join(f"{v:>18s}" for v in versions))
    for layout in layouts:
        row = f"{layout:14s}"
        for v in versions:
            runs = data.get((v, layout), [])
            ok = [r for r in runs if r["ok"] and r["total"]]
            if not runs:
                row += f"{'-':>18s}"
                continue
            mean = sum(r["total"] for r in ok) / len(ok) if ok else float("nan")
            tilt = sum(r["tilt"] for r in ok if r["tilt"] is not None) / max(1, len(ok))
            fail = len(runs) - len(ok)
            row += f"{mean:8.1f}s×{len(ok)}{'!' + str(fail) if fail else '':2s} {tilt:.2f}°"
        print(row)


if __name__ == "__main__":
    main()
