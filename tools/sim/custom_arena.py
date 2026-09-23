#!/usr/bin/env python3
"""按指定布局生成 edc_arena.sdf（替代 randomize_arena.py 的随机摆放）。

布局文件 JSON，障碍坐标用 camera_init/SLAM 系（起飞点=原点，与 params.hpp 同口径）：
    {
      "name": "gap_between_pillars",
      "description": "...",
      "obstacles": [{"x": 3.0, "y": 0.0, "r": 0.2}, ...],
      "gates": {"gate_wall_A": "center", "gate_wall_B": "left"}
    }
r 省略时取 0.2。gates 省略时两道门都用 center。

坐标换算：PX4 出生点 world(-4,0) 对应 delivery_field 本地 (0,-4)，本地→world 为 (lx,ly)→(ly,-lx)。
SLAM 与 world 同轴、原点在出生点，因此 本地 lx = -slam_y，ly = slam_x - 4。

用法：custom_arena.py LAYOUT.json [--sim-root ~/uav_sim_env]
"""

import argparse
import importlib.util
import json
import math
import sys
from pathlib import Path

SPAWN_WORLD_X = -4.0


def load_randomizer(sim_root):
    spec = importlib.util.spec_from_file_location("randomize_arena", sim_root / "randomize_arena.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def slam_to_local(x, y):
    return -y, x + SPAWN_WORLD_X


def make_link(index, lx, ly, radius, height):
    z = height / 2.0
    geometry = f"<cylinder><radius>{radius:.3f}</radius><length>{height:.3f}</length></cylinder>"
    return (
        f'      <link name="rand_obs_{index}">\n'
        f'        <pose>{lx:.3f} {ly:.3f} 0 0 0 0</pose>\n'
        f'        <collision name="c"><pose>0 0 {z:.3f} 0 0 0</pose><geometry>{geometry}</geometry></collision>\n'
        f'        <visual name="v"><pose>0 0 {z:.3f} 0 0 0</pose>\n'
        f'          <geometry>{geometry}</geometry>\n'
        f'          <material><ambient>0.6 0.25 0.05 1</ambient><diffuse>0.85 0.4 0.1 1</diffuse></material>\n'
        f'        </visual>\n'
        f'      </link>\n'
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("layout", type=Path)
    parser.add_argument("--sim-root", type=Path, default=Path.home() / "uav_sim_env")
    args = parser.parse_args()

    layout = json.loads(args.layout.read_text(encoding="utf-8"))
    ra = load_randomizer(args.sim_root)
    sdf = Path(ra.BASE).read_text(encoding="utf-8")
    if ra.MARKER not in sdf:
        raise SystemExit("模板里找不到 delivery_field 的 </model> 标记")

    gates = layout.get("gates", {})
    for name in ra.GATE_NAMES:
        side = gates.get(name, "center")
        if side not in ("left", "center", "right"):
            raise SystemExit(f"{name} 开口只能是 left/center/right，收到 {side}")
        ra.GATE_NAMES, ra.GATE_OPENINGS = (name,), (side,)
        sdf, reports = ra.randomize_gates(sdf)
        for report in reports:
            print(f"  {name}: {report['label']}开口，净宽 {report['width']:.3f}m")

    block = "\n      <!-- ====== 指定障碍圆柱 (custom_arena.py) ====== -->\n"
    for index, obstacle in enumerate(layout.get("obstacles", []), 1):
        x, y = float(obstacle["x"]), float(obstacle["y"])
        radius = float(obstacle.get("r", ra.RADIUS))
        if not all(math.isfinite(v) for v in (x, y, radius)) or radius <= 0:
            raise SystemExit(f"障碍 #{index} 坐标或半径无效")
        lx, ly = slam_to_local(x, y)
        if not (ra.WALL_X_MIN < lx < ra.WALL_X_MAX and ra.WALL_Y_MIN < ly < ra.PARTITION_Y):
            raise SystemExit(f"障碍 #{index} SLAM({x},{y}) 超出场地")
        block += make_link(index, lx, ly, radius, ra.HEIGHT)
        print(f"  rand_obs_{index}: SLAM ({x:+.2f},{y:+.2f}) r={radius:.2f}")
    sdf = sdf.replace(ra.MARKER, block + ra.MARKER, 1)
    Path(ra.OUT).write_text(sdf, encoding="utf-8")
    print(f"已写出布局 {layout.get('name', args.layout.stem)} -> {ra.OUT}")


if __name__ == "__main__":
    sys.exit(main())
