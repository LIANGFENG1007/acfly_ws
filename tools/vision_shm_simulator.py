#!/usr/bin/env python3
"""Interactive SHM vision sender for local exploration-planner testing.

The window uses the same fixed SLAM field as the planner defaults:
    x in [0.0, 7.5], y in [-5.0, 5.0], aircraft start = (0.0, 0.0).
Click the field to add/select a simulated target.  The script writes the
existing 512-byte little-endian /dev/shm/uav_cv_out layout and never starts a
ROS node or sends a flight command.
"""

import argparse
import math
import mmap
import os
import struct
import time
import tkinter as tk
from tkinter import messagebox, ttk


SHM_SIZE = 512
MAX_TARGETS = 6
FIELD_MIN_X, FIELD_MAX_X = 0.0, 7.5
FIELD_MIN_Y, FIELD_MAX_Y = -5.0, 5.0
AIRCRAFT_X, AIRCRAFT_Y = 0.0, 0.0
TARGET_STRIDE = 40
TARGET_OFFSET = 40


class VisionShmSimulator:
    def __init__(self, path: str, period_ms: int, jitter_m: float):
        self.path = path
        self.period_ms = period_ms
        self.jitter_m = max(0.0, jitter_m)
        self.targets = []  # [id, x, y]
        self.selected = None
        self.seq = 0
        self.sending = False
        self.last_stamp = 0.0
        self.fd = None
        self.mm = None

        self.root = tk.Tk()
        self.root.title("视觉 SHM 模拟发送器 · /dev/shm/uav_cv_out")
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self._build_ui()
        self._open_shm()
        self._draw()
        self.root.after(self.period_ms, self._tick)

    def _build_ui(self):
        outer = ttk.Frame(self.root, padding=8)
        outer.grid(row=0, column=0, sticky="nsew")
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)

        self.canvas = tk.Canvas(outer, width=820, height=620, bg="#222831",
                                highlightthickness=1, highlightbackground="#607080")
        self.canvas.grid(row=0, column=0, rowspan=8, padx=(0, 10))
        self.canvas.bind("<Button-1>", self._click_field)

        ttk.Label(outer, text="目标 ID（1～6）").grid(row=0, column=1, sticky="w")
        self.id_var = tk.IntVar(value=1)
        self.id_spin = tk.Spinbox(outer, from_=1, to=6, width=6, textvariable=self.id_var)
        self.id_spin.grid(row=1, column=1, sticky="w", pady=(2, 10))

        self.send_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(outer, text="连续发送", variable=self.send_var,
                        command=self._toggle_send).grid(row=2, column=1, sticky="w")
        ttk.Button(outer, text="发送一帧", command=lambda: self._publish(force=True)).grid(
            row=3, column=1, sticky="ew", pady=3)
        ttk.Button(outer, text="发送空帧", command=self._publish_empty).grid(
            row=4, column=1, sticky="ew", pady=3)
        ttk.Button(outer, text="清空目标", command=self._clear_targets).grid(
            row=5, column=1, sticky="ew", pady=3)

        ttk.Label(outer, text="点击：添加目标\n再次点击目标：选中它\n右键：删除目标").grid(
            row=6, column=1, sticky="nw", pady=(15, 4))
        self.status_var = tk.StringVar(value="准备中")
        ttk.Label(outer, textvariable=self.status_var, wraplength=210).grid(
            row=7, column=1, sticky="sw")

        outer.columnconfigure(1, weight=1)
        outer.rowconfigure(7, weight=1)

    def _open_shm(self):
        try:
            self.fd = os.open(self.path, os.O_CREAT | os.O_RDWR, 0o660)
            os.ftruncate(self.fd, SHM_SIZE)
            self.mm = mmap.mmap(self.fd, SHM_SIZE, access=mmap.ACCESS_WRITE)
            old_seq = struct.unpack_from("<Q", self.mm, 0)[0]
            self.seq = old_seq & ~1
            self.status_var.set(f"已打开 {self.path}\n场地点击发送模式")
        except OSError as exc:
            messagebox.showerror("SHM 打开失败", f"无法打开 {self.path}:\n{exc}")
            self.close()
            raise

    def _xy_from_canvas(self, px, py):
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        x = FIELD_MIN_X + (px / width) * (FIELD_MAX_X - FIELD_MIN_X)
        y = FIELD_MAX_Y - (py / height) * (FIELD_MAX_Y - FIELD_MIN_Y)
        return x, y

    def _canvas_from_xy(self, x, y):
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        px = (x - FIELD_MIN_X) / (FIELD_MAX_X - FIELD_MIN_X) * width
        py = (FIELD_MAX_Y - y) / (FIELD_MAX_Y - FIELD_MIN_Y) * height
        return px, py

    def _click_field(self, event):
        x, y = self._xy_from_canvas(event.x, event.y)
        if event.num == 3:
            self._delete_nearest(x, y)
            return
        nearest = None
        nearest_d = float("inf")
        for index, (_, tx, ty) in enumerate(self.targets):
            d = math.hypot(tx - x, ty - y)
            if d < nearest_d:
                nearest, nearest_d = index, d
        if nearest is not None and nearest_d <= 0.25:
            self.selected = nearest
            self.targets[nearest][0] = self._read_id()
        elif len(self.targets) < MAX_TARGETS:
            self.targets.append([self._read_id(), x, y])
            self.selected = len(self.targets) - 1
        else:
            self.status_var.set("已达到每帧最多 6 个目标；右键删除后再添加")
        self._draw()
        if self.sending:
            self._publish(force=True)

    def _read_id(self):
        try:
            value = int(self.id_var.get())
        except (TypeError, ValueError):
            value = 1
        return max(1, min(6, value))

    def _delete_nearest(self, x, y):
        if not self.targets:
            return
        index = min(range(len(self.targets)),
                    key=lambda i: math.hypot(self.targets[i][1] - x, self.targets[i][2] - y))
        if math.hypot(self.targets[index][1] - x, self.targets[index][2] - y) <= 0.35:
            self.targets.pop(index)
            self.selected = min(index, len(self.targets) - 1) if self.targets else None
            self._draw()
            if self.sending:
                self._publish(force=True)

    def _toggle_send(self):
        self.sending = self.send_var.get()
        self.status_var.set("连续发送中" if self.sending else "已停止连续发送；可手动发送一帧")
        if self.sending:
            self._publish(force=True)

    def _clear_targets(self):
        self.targets.clear()
        self.selected = None
        self._draw()
        if self.sending:
            self._publish(force=True)

    def _publish_empty(self):
        self.targets.clear()
        self.selected = None
        self._draw()
        self._publish(force=True)

    def _publish(self, force=False):
        if self.mm is None or (not force and not self.sending):
            return
        now = time.monotonic()
        if now <= self.last_stamp:
            now = self.last_stamp + 1e-6
        frame = bytearray(SHM_SIZE)
        struct.pack_into("<d", frame, 8, now)
        struct.pack_into("<i", frame, 32, len(self.targets))
        for index, (target_id, x, y) in enumerate(self.targets):
            if self.jitter_m:
                # Deterministic tiny motion-like perturbation, useful for seeing
                # the receiver's weighted averaging without changing the target.
                phase = self.seq / 2.0 + index * 1.7
                x += self.jitter_m * math.sin(phase)
                y += self.jitter_m * math.cos(phase * 0.83)
            offset = TARGET_OFFSET + TARGET_STRIDE * index
            struct.pack_into("<i", frame, offset, int(target_id))
            struct.pack_into("<d", frame, offset + 16, x)
            struct.pack_into("<d", frame, offset + 24, y)

        odd = self.seq + 1
        even = self.seq + 2
        # Keep the producer protocol explicit: odd means payload is changing,
        # even means a complete coherent frame is available to the reader.
        struct.pack_into("<Q", self.mm, 0, odd)
        self.mm.flush()
        self.mm[8:SHM_SIZE] = frame[8:SHM_SIZE]
        self.mm.flush()
        struct.pack_into("<Q", self.mm, 0, even)
        self.mm.flush()
        self.seq = even
        self.last_stamp = now
        self.status_var.set(f"已发送 seq={self.seq}，目标={len(self.targets)}，"
                            f"ID={[t[0] for t in self.targets]}")

    def _tick(self):
        if self.sending:
            self._publish()
        self.root.after(self.period_ms, self._tick)

    def _draw(self):
        self.canvas.delete("all")
        width = max(1, self.canvas.winfo_width())
        height = max(1, self.canvas.winfo_height())
        # Outer field and a light coordinate grid.
        self.canvas.create_rectangle(1, 1, width - 1, height - 1, outline="#AAB7C4", width=2)
        for i in range(1, 8):
            px, _ = self._canvas_from_xy(i, FIELD_MIN_Y)
            self.canvas.create_line(px, 0, px, height, fill="#39434D")
        for y in range(-4, 5):
            _, py = self._canvas_from_xy(FIELD_MIN_X, y)
            self.canvas.create_line(0, py, width, py, fill="#39434D")
        self.canvas.create_text(8, 8, anchor="nw", fill="#D9E2EC",
                                text=f"camera_init  x=[{FIELD_MIN_X},{FIELD_MAX_X}]  y=[{FIELD_MIN_Y},{FIELD_MAX_Y}]")
        ax, ay = self._canvas_from_xy(AIRCRAFT_X, AIRCRAFT_Y)
        self.canvas.create_oval(ax - 8, ay - 8, ax + 8, ay + 8, fill="#FFD166", outline="white", width=2)
        self.canvas.create_text(ax + 12, ay, anchor="w", fill="#FFD166", text="飞机 (0, 0)")
        for index, (target_id, x, y) in enumerate(self.targets):
            px, py = self._canvas_from_xy(x, y)
            selected = index == self.selected
            color = "#FF4D6D" if selected else "#41D3BD"
            radius = 10 if selected else 8
            self.canvas.create_oval(px - radius, py - radius, px + radius, py + radius,
                                    fill=color, outline="white", width=2)
            self.canvas.create_text(px + 13, py - 12, anchor="sw", fill=color,
                                    text=f"#{index + 1} ID={target_id} ({x:.2f},{y:.2f})")

    def close(self):
        # Publish one empty frame so a running reader stops treating the last
        # target list as a live stream; then leave the file for its mmap reader.
        if self.mm is not None:
            try:
                self.targets.clear()
                self._publish(force=True)
            except (OSError, ValueError):
                pass
            self.mm.close()
            self.mm = None
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def run(self):
        self.root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--path", default="/dev/shm/uav_cv_out", help="SHM path (default: /dev/shm/uav_cv_out)")
    parser.add_argument("--period-ms", type=int, default=50, help="Continuous send period (default: 50 ms)")
    parser.add_argument("--jitter-m", type=float, default=0.0,
                        help="Optional deterministic coordinate jitter per frame (default: 0)")
    args = parser.parse_args()
    if args.period_ms < 10:
        parser.error("--period-ms must be at least 10")
    if args.jitter_m < 0:
        parser.error("--jitter-m must be nonnegative")
    VisionShmSimulator(args.path, args.period_ms, args.jitter_m).run()


if __name__ == "__main__":
    main()
