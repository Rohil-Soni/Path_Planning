#!/usr/bin/env python3
"""
GUI for Lifelong Planning A* (LPA*) Visualization.
Shows initial planning and incremental replanning after obstacle updates.
"""

import os
import random
import tkinter as tk
from datetime import datetime
from tkinter import messagebox, ttk

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.colors import ListedColormap
from matplotlib.figure import Figure
from matplotlib.lines import Line2D

try:
    from Algo_LPA import LPAStarPathfinder
except ImportError:
    from .Algo_LPA import LPAStarPathfinder


class LPAGUI:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("LPA* Algorithm Visualizer")
        self.root.geometry("1250x820")

        self.rows = tk.IntVar(value=20)
        self.cols = tk.IntVar(value=20)

        self.start_row = tk.IntVar(value=0)
        self.start_col = tk.IntVar(value=0)
        self.goal_row = tk.IntVar(value=19)
        self.goal_col = tk.IntVar(value=19)

        self.min_obstacles = tk.IntVar(value=40)
        self.dynamic_changes = tk.IntVar(value=10)
        self.change_mode = tk.StringVar(value="toggle")

        self.auto_timestamp = tk.BooleanVar(value=True)
        self.custom_filename = tk.StringVar(value="lpa_visualization")

        self.setting_mode = tk.StringVar(value="none")
        self.grid_canvas: tk.Canvas | None = None

        self.create_widgets()

    def create_widgets(self) -> None:
        main = ttk.Frame(self.root, padding="10")
        main.grid(row=0, column=0, sticky="nsew")

        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        left = ttk.Frame(main, padding="6")
        left.grid(row=0, column=0, sticky="nsew")

        right = ttk.Frame(main, padding="6")
        right.grid(row=0, column=1, sticky="nsew")

        row_idx = 0
        ttk.Label(left, text="LPA* Settings", font=("Arial", 14, "bold")).grid(
            row=row_idx, column=0, columnspan=2, pady=(0, 14)
        )
        row_idx += 1

        ttk.Label(left, text="Grid", font=("Arial", 10, "bold")).grid(
            row=row_idx, column=0, columnspan=2, sticky=tk.W, pady=4
        )
        row_idx += 1

        ttk.Label(left, text="Rows (5-60):").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(left, from_=5, to=60, textvariable=self.rows, width=15).grid(
            row=row_idx, column=1, sticky="we", pady=2
        )
        row_idx += 1

        ttk.Label(left, text="Cols (5-60):").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(left, from_=5, to=60, textvariable=self.cols, width=15).grid(
            row=row_idx, column=1, sticky="we", pady=2
        )
        row_idx += 1

        ttk.Button(left, text="Update Grid Preview", command=self.update_grid_preview).grid(
            row=row_idx, column=0, columnspan=2, pady=(6, 10)
        )
        row_idx += 1

        ttk.Separator(left, orient="horizontal").grid(
            row=row_idx, column=0, columnspan=2, sticky="we", pady=8
        )
        row_idx += 1

        ttk.Label(left, text="Start / Goal", font=("Arial", 10, "bold")).grid(
            row=row_idx, column=0, columnspan=2, sticky=tk.W, pady=4
        )
        row_idx += 1

        ttk.Label(left, text="Start Row:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(left, from_=0, to=59, textvariable=self.start_row, width=15).grid(
            row=row_idx, column=1, sticky="we", pady=2
        )
        row_idx += 1

        ttk.Label(left, text="Start Col:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(left, from_=0, to=59, textvariable=self.start_col, width=15).grid(
            row=row_idx, column=1, sticky="we", pady=2
        )
        row_idx += 1

        ttk.Button(left, text="Click Grid To Set Start", command=lambda: self.set_click_mode("start")).grid(
            row=row_idx, column=0, columnspan=2, pady=3
        )
        row_idx += 1

        ttk.Label(left, text="Goal Row:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(left, from_=0, to=59, textvariable=self.goal_row, width=15).grid(
            row=row_idx, column=1, sticky="we", pady=2
        )
        row_idx += 1

        ttk.Label(left, text="Goal Col:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(left, from_=0, to=59, textvariable=self.goal_col, width=15).grid(
            row=row_idx, column=1, sticky="we", pady=2
        )
        row_idx += 1

        ttk.Button(left, text="Click Grid To Set Goal", command=lambda: self.set_click_mode("goal")).grid(
            row=row_idx, column=0, columnspan=2, pady=3
        )
        row_idx += 1

        ttk.Separator(left, orient="horizontal").grid(
            row=row_idx, column=0, columnspan=2, sticky="we", pady=8
        )
        row_idx += 1

        ttk.Label(left, text="Dynamic Scenario", font=("Arial", 10, "bold")).grid(
            row=row_idx, column=0, columnspan=2, sticky=tk.W, pady=4
        )
        row_idx += 1

        ttk.Label(left, text="Min Obstacles:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(left, from_=0, to=2000, textvariable=self.min_obstacles, width=15).grid(
            row=row_idx, column=1, sticky="we", pady=2
        )
        row_idx += 1

        ttk.Label(left, text="Dynamic Changes:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        ttk.Spinbox(left, from_=1, to=200, textvariable=self.dynamic_changes, width=15).grid(
            row=row_idx, column=1, sticky="we", pady=2
        )
        row_idx += 1

        ttk.Label(left, text="Change Mode:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        ttk.Combobox(
            left,
            values=["toggle", "add", "remove"],
            textvariable=self.change_mode,
            state="readonly",
            width=13,
        ).grid(row=row_idx, column=1, sticky="we", pady=2)
        row_idx += 1

        ttk.Separator(left, orient="horizontal").grid(
            row=row_idx, column=0, columnspan=2, sticky="we", pady=8
        )
        row_idx += 1

        ttk.Label(left, text="Output", font=("Arial", 10, "bold")).grid(
            row=row_idx, column=0, columnspan=2, sticky=tk.W, pady=4
        )
        row_idx += 1

        ttk.Checkbutton(left, text="Auto-add timestamp", variable=self.auto_timestamp).grid(
            row=row_idx, column=0, columnspan=2, sticky=tk.W, pady=2
        )
        row_idx += 1

        ttk.Label(left, text="Filename:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        ttk.Entry(left, textvariable=self.custom_filename, width=20).grid(
            row=row_idx, column=1, sticky="we", pady=2
        )
        row_idx += 1

        ttk.Separator(left, orient="horizontal").grid(
            row=row_idx, column=0, columnspan=2, sticky="we", pady=10
        )
        row_idx += 1

        ttk.Button(left, text="Run LPA* Initial + Replan", command=self.run_simulation).grid(
            row=row_idx, column=0, columnspan=2, pady=8, ipadx=12, ipady=8
        )
        row_idx += 1

        self.status_label = ttk.Label(left, text="Ready", foreground="green")
        self.status_label.grid(row=row_idx, column=0, columnspan=2, pady=4)

        ttk.Label(right, text="Grid Preview", font=("Arial", 12, "bold")).pack(pady=(0, 8))
        ttk.Label(
            right,
            text="Red = Start, Gold = Goal. Use click modes to move them.",
            foreground="blue",
            font=("Arial", 9),
        ).pack(pady=3)

        self.canvas_frame = ttk.Frame(right)
        self.canvas_frame.pack(fill=tk.BOTH, expand=True)
        self.update_grid_preview()

    def update_grid_preview(self) -> None:
        rows = self.rows.get()
        cols = self.cols.get()

        if self.start_row.get() >= rows:
            self.start_row.set(rows - 1)
        if self.start_col.get() >= cols:
            self.start_col.set(cols - 1)
        if self.goal_row.get() >= rows:
            self.goal_row.set(rows - 1)
        if self.goal_col.get() >= cols:
            self.goal_col.set(cols - 1)

        for widget in self.canvas_frame.winfo_children():
            widget.destroy()

        canvas_size = min(520, 460)
        cell = max(4, canvas_size // max(rows, cols))

        self.grid_canvas = tk.Canvas(
            self.canvas_frame,
            width=cols * cell,
            height=rows * cell,
            bg="white",
            highlightthickness=1,
            highlightbackground="black",
        )
        self.grid_canvas.pack()

        for i in range(rows + 1):
            self.grid_canvas.create_line(0, i * cell, cols * cell, i * cell, fill="gray")
        for j in range(cols + 1):
            self.grid_canvas.create_line(j * cell, 0, j * cell, rows * cell, fill="gray")

        self.draw_positions(cell)
        self.grid_canvas.bind("<Button-1>", lambda e: self.on_grid_click(e, cell))

    def draw_positions(self, cell_size: int) -> None:
        if self.grid_canvas is None:
            return

        start = (self.start_row.get(), self.start_col.get())
        goal = (self.goal_row.get(), self.goal_col.get())

        self.grid_canvas.delete("marker")

        sx, sy = start[1] * cell_size, start[0] * cell_size
        self.grid_canvas.create_rectangle(
            sx, sy, sx + cell_size, sy + cell_size, fill="red", tags="marker"
        )
        self.grid_canvas.create_text(
            sx + cell_size // 2,
            sy + cell_size // 2,
            text="S",
            fill="white",
            font=("Arial", 11, "bold"),
            tags="marker",
        )

        gx, gy = goal[1] * cell_size, goal[0] * cell_size
        self.grid_canvas.create_rectangle(
            gx, gy, gx + cell_size, gy + cell_size, fill="gold", tags="marker"
        )
        self.grid_canvas.create_text(
            gx + cell_size // 2,
            gy + cell_size // 2,
            text="G",
            fill="black",
            font=("Arial", 11, "bold"),
            tags="marker",
        )

    def set_click_mode(self, mode: str) -> None:
        self.setting_mode.set(mode)
        if mode == "start":
            self.status_label.config(text="Click to set START", foreground="blue")
        elif mode == "goal":
            self.status_label.config(text="Click to set GOAL", foreground="blue")

    def on_grid_click(self, event: tk.Event, cell_size: int) -> None:
        c = event.x // cell_size
        r = event.y // cell_size
        if r < 0 or c < 0 or r >= self.rows.get() or c >= self.cols.get():
            return

        mode = self.setting_mode.get()
        if mode == "start":
            self.start_row.set(r)
            self.start_col.set(c)
            self.setting_mode.set("none")
            self.status_label.config(text=f"Start set to ({r}, {c})", foreground="green")
        elif mode == "goal":
            self.goal_row.set(r)
            self.goal_col.set(c)
            self.setting_mode.set("none")
            self.status_label.config(text=f"Goal set to ({r}, {c})", foreground="green")

        self.draw_positions(cell_size)

    def generate_random_obstacles(
        self,
        rows: int,
        cols: int,
        start: tuple,
        goal: tuple,
        min_obstacles: int,
    ) -> tuple:
        max_attempts = 120
        max_obstacles = int(rows * cols * 0.38)
        min_obstacles = min(min_obstacles, max_obstacles)

        for _ in range(max_attempts):
            n_obs = random.randint(min_obstacles, max_obstacles)
            obs = set()
            while len(obs) < n_obs:
                r = random.randint(0, rows - 1)
                c = random.randint(0, cols - 1)
                if (r, c) != start and (r, c) != goal:
                    obs.add((r, c))

            planner = LPAStarPathfinder((rows, cols), list(obs))
            result = planner.plan(start, goal)
            if result["success"]:
                return list(obs), planner, result

        fallback = []
        planner = LPAStarPathfinder((rows, cols), fallback)
        return fallback, planner, planner.plan(start, goal)

    def choose_dynamic_changes(
        self,
        rows: int,
        cols: int,
        obstacles: list,
        initial_path: list,
        start: tuple,
        goal: tuple,
    ) -> list:
        count = self.dynamic_changes.get()
        mode = self.change_mode.get().strip().lower()

        obstacle_set = set(obstacles)
        free_cells = [
            (r, c)
            for r in range(rows)
            for c in range(cols)
            if (r, c) not in obstacle_set and (r, c) != start and (r, c) != goal
        ]
        blocked_cells = [
            (r, c)
            for r in range(rows)
            for c in range(cols)
            if (r, c) in obstacle_set and (r, c) != start and (r, c) != goal
        ]

        # Bias updates near the previous path to force meaningful replanning.
        path_candidates = [p for p in initial_path[1:-1] if p != start and p != goal]
        random.shuffle(path_candidates)

        changes = []

        if mode == "add":
            candidates = path_candidates + free_cells
            seen = set()
            for cell in candidates:
                if cell in seen or cell in obstacle_set:
                    continue
                changes.append((cell[0], cell[1], True))
                seen.add(cell)
                if len(changes) >= count:
                    break

        elif mode == "remove":
            random.shuffle(blocked_cells)
            for cell in blocked_cells[:count]:
                changes.append((cell[0], cell[1], False))

        else:
            random.shuffle(free_cells)
            random.shuffle(blocked_cells)
            add_count = count // 2
            rem_count = count - add_count
            for cell in (path_candidates + free_cells)[:add_count]:
                if cell not in obstacle_set:
                    changes.append((cell[0], cell[1], True))
            for cell in blocked_cells[:rem_count]:
                changes.append((cell[0], cell[1], False))

        unique = {}
        for r, c, b in changes:
            unique[(r, c)] = b

        final = [(r, c, b) for (r, c), b in unique.items()]
        return final[:count]

    def run_simulation(self) -> None:
        rows = self.rows.get()
        cols = self.cols.get()
        start = (self.start_row.get(), self.start_col.get())
        goal = (self.goal_row.get(), self.goal_col.get())

        if start == goal:
            messagebox.showerror("Error", "Start and goal must be different.")
            return

        self.status_label.config(text="Building initial map...", foreground="orange")
        self.root.update()

        try:
            obstacles, planner, initial_result = self.generate_random_obstacles(
                rows, cols, start, goal, self.min_obstacles.get()
            )
            if not initial_result["success"]:
                messagebox.showerror("Error", "Failed to build a valid initial path.")
                self.status_label.config(text="No valid initial path.", foreground="red")
                return

            changes = self.choose_dynamic_changes(
                rows,
                cols,
                obstacles,
                initial_result["path"],
                start,
                goal,
            )

            self.status_label.config(text="Running incremental replanning...", foreground="blue")
            self.root.update()

            replanned_result = planner.apply_obstacle_changes(changes)

            # Rebuild current obstacle list from planner after updates.
            updated_obstacles = []
            for r in range(rows):
                for c in range(cols):
                    if planner.obstacle_grid[r][c]:
                        updated_obstacles.append((r, c))

            self.create_animation(
                rows,
                cols,
                start,
                goal,
                obstacles,
                updated_obstacles,
                changes,
                initial_result,
                replanned_result,
            )

            self.status_label.config(
                text="Simulation complete. Inspect visualization window.", foreground="green"
            )

        except Exception as exc:
            messagebox.showerror("Error", f"Simulation failed: {exc}")
            self.status_label.config(text="Simulation failed.", foreground="red")

    def create_animation(
        self,
        rows: int,
        cols: int,
        start: tuple,
        goal: tuple,
        initial_obstacles: list,
        updated_obstacles: list,
        changes: list,
        initial_result: dict,
        replanned_result: dict,
    ) -> None:
        window = tk.Toplevel(self.root)
        window.title("LPA* Side-by-Side: Before vs Replanned")
        window.geometry("1280x820")

        fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(12.8, 6.8))

        # 0 empty, 1 obstacle, 2 previous explored, 3 previous path,
        # 4 start, 5 goal, 6 new exploration during replan, 7 replanned path.
        colors = ["white", "#2f2f2f", "#9fd2f5", "#7ad67a", "#d93636", "#ffd24d", "#b553d6", "#1f9d4c"]
        cmap = ListedColormap(colors)

        left_grid = np.zeros((rows, cols), dtype=np.int32)
        right_grid = np.zeros((rows, cols), dtype=np.int32)

        for obs in initial_obstacles:
            left_grid[obs[0]][obs[1]] = 1

        for obs in updated_obstacles:
            right_grid[obs[0]][obs[1]] = 1

        left_grid[start[0]][start[1]] = 4
        left_grid[goal[0]][goal[1]] = 5
        right_grid[start[0]][start[1]] = 4
        right_grid[goal[0]][goal[1]] = 5

        im_left = ax_left.imshow(left_grid, cmap=cmap, interpolation="nearest", origin="upper", vmin=0, vmax=7)
        im_right = ax_right.imshow(
            right_grid, cmap=cmap, interpolation="nearest", origin="upper", vmin=0, vmax=7
        )

        for axis in (ax_left, ax_right):
            axis.set_xticks(np.arange(-0.5, cols, 1), minor=True)
            axis.set_yticks(np.arange(-0.5, rows, 1), minor=True)
            axis.grid(which="minor", color="gray", linestyle="-", linewidth=0.45)
            axis.tick_params(which="minor", size=0)
            axis.set_xticks(np.arange(0, cols, 1))
            axis.set_yticks(np.arange(0, rows, 1))
            axis.set_xlabel("Column")
            axis.set_ylabel("Row")
            axis.text(
                start[1], start[0], "S", ha="center", va="center", color="white", fontweight="bold"
            )
            axis.text(goal[1], goal[0], "G", ha="center", va="center", color="black", fontweight="bold")

        left_title = ax_left.set_title("Before Changes", fontweight="bold")
        right_title = ax_right.set_title("After Changes + Replan", fontweight="bold")

        added_change_cells = [(r, c) for r, c, blocked in changes if blocked]
        removed_change_cells = [(r, c) for r, c, blocked in changes if not blocked]

        legend_items = [
            patches.Patch(facecolor="#d93636", edgecolor="black", label="Start"),
            patches.Patch(facecolor="#ffd24d", edgecolor="black", label="Goal"),
            patches.Patch(facecolor="#2f2f2f", edgecolor="black", label="Obstacle"),
            patches.Patch(facecolor="#9fd2f5", edgecolor="black", label="Previous Explored"),
            patches.Patch(facecolor="#7ad67a", edgecolor="black", label="Previous Path"),
            patches.Patch(facecolor="#b553d6", edgecolor="black", label="Newly Explored (Replan)"),
            patches.Patch(facecolor="#1f9d4c", edgecolor="black", label="Replanned Path"),
            Line2D(
                [0],
                [0],
                marker="X",
                color="none",
                markeredgecolor="#ff8c1a",
                markersize=9,
                linestyle="None",
                label="Changed To Obstacle",
            ),
            Line2D(
                [0],
                [0],
                marker="o",
                color="none",
                markeredgecolor="#1589ff",
                markerfacecolor="none",
                markersize=9,
                linestyle="None",
                label="Changed To Free Cell",
            ),
        ]
        fig.legend(handles=legend_items, loc="lower center", bbox_to_anchor=(0.5, 0.01), ncol=3, fontsize=9)

        plt.tight_layout(rect=(0, 0.08, 1, 1))

        canvas = FigureCanvasTkAgg(fig, master=window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        control = ttk.Frame(window)
        control.pack(fill=tk.X, padx=8, pady=4)

        status = tk.StringVar(value="Preparing animation...")
        ttk.Label(control, textvariable=status, font=("Arial", 10)).pack(side=tk.LEFT, padx=4)

        save_btn = ttk.Button(control, text="Save Final Figure", state="disabled")
        save_btn.pack(side=tk.RIGHT, padx=4)

        calc_text = tk.Text(window, height=8, wrap="word")
        calc_text.pack(fill=tk.X, padx=8, pady=5)

        calc_text.insert(
            tk.END,
            "LPA* equations used:\n"
            "rhs(s_start)=0, rhs(s)=min(g(pred)+c(pred,s))\n"
            "key(s)=(min(g,rhs)+h(s,goal), min(g,rhs))\n\n"
        )

        for entry in replanned_result.get("calculation_snapshot", []):
            node = entry["node"]
            calc_text.insert(
                tk.END,
                f"Node {node}: g={entry['g']:.3f}, rhs={entry['rhs']:.3f}, "
                f"h={entry['h']:.3f}, key=({entry['key'][0]:.3f}, {entry['key'][1]:.3f}), "
                f"consistent={entry['consistent']}\n",
            )

        calc_text.config(state="disabled")

        initial_expanded = initial_result.get("expanded_nodes", [])
        initial_path = initial_result.get("path", [])
        replan_expanded = replanned_result.get("expanded_nodes", [])
        replan_path = replanned_result.get("path", [])

        changed_cells = [(r, c) for r, c, _ in changes]
        initial_expanded_set = set(initial_expanded)

        added_from_expanded = sum(1 for cell in added_change_cells if cell in initial_expanded_set)
        removed_from_expanded = sum(1 for cell in removed_change_cells if cell in initial_expanded_set)

        influenced_zone = set(changed_cells)
        for r, c in changed_cells:
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        influenced_zone.add((nr, nc))

        state = {"phase": 0, "idx": 0, "running": True}
        replan_old_count = {"value": 0}
        replan_new_count = {"value": 0}
        marker_artists_left = []
        marker_artists_right = []

        def draw_change_markers() -> None:
            for artist in marker_artists_left:
                artist.remove()
            marker_artists_left.clear()

            for artist in marker_artists_right:
                artist.remove()
            marker_artists_right.clear()

            if added_change_cells:
                xs = [c for _, c in added_change_cells]
                ys = [r for r, _ in added_change_cells]
                marker_artists_left.append(
                    ax_left.scatter(xs, ys, marker="X", s=65, c="#ff8c1a", linewidths=1.2, zorder=5)
                )
                marker_artists_right.append(
                    ax_right.scatter(xs, ys, marker="X", s=65, c="#ff8c1a", linewidths=1.2, zorder=5)
                )

            if removed_change_cells:
                xs = [c for _, c in removed_change_cells]
                ys = [r for r, _ in removed_change_cells]
                marker_artists_left.append(
                    ax_left.scatter(
                        xs,
                        ys,
                        marker="o",
                        s=70,
                        facecolors="none",
                        edgecolors="#1589ff",
                        linewidths=1.6,
                        zorder=5,
                    )
                )
                marker_artists_right.append(
                    ax_right.scatter(
                        xs,
                        ys,
                        marker="o",
                        s=70,
                        facecolors="none",
                        edgecolors="#1589ff",
                        linewidths=1.6,
                        zorder=5,
                    )
                )

        def step() -> None:
            if not state["running"]:
                return

            phase = state["phase"]
            idx = state["idx"]

            if phase == 0:
                if idx < len(initial_expanded):
                    node = initial_expanded[idx]
                    if node != start and node != goal and left_grid[node[0]][node[1]] == 0:
                        left_grid[node[0]][node[1]] = 2
                    im_left.set_array(left_grid)
                    left_title.set_text(f"Before Changes: Expanded {idx + 1}/{len(initial_expanded)}")
                    status.set(
                        f"Initial expansion {idx + 1}/{len(initial_expanded)} | cost={initial_result['total_cost']:.2f}"
                    )
                    canvas.draw()
                    state["idx"] += 1
                    window.after(30, step)
                    return
                state["phase"] = 1
                state["idx"] = 0
                window.after(350, step)
                return

            if phase == 1:
                if idx < len(initial_path):
                    node = initial_path[idx]
                    if node != start and node != goal:
                        left_grid[node[0]][node[1]] = 3
                    im_left.set_array(left_grid)
                    left_title.set_text(f"Before Changes: Path {idx + 1}/{len(initial_path)}")
                    status.set("Drawing initial optimal path")
                    canvas.draw()
                    state["idx"] += 1
                    window.after(70, step)
                    return
                state["phase"] = 2
                state["idx"] = 0
                window.after(450, step)
                return

            if phase == 2:
                # In right graph, keep previous knowledge then apply changed environment.
                for cell in initial_expanded:
                    if cell != start and cell != goal and right_grid[cell[0]][cell[1]] == 0:
                        right_grid[cell[0]][cell[1]] = 2
                for cell in initial_path:
                    if cell != start and cell != goal and right_grid[cell[0]][cell[1]] != 1:
                        right_grid[cell[0]][cell[1]] = 3

                im_right.set_array(right_grid)
                draw_change_markers()
                right_title.set_text("After Changes: Compare with Left Graph")
                status.set(
                    f"Right graph updated | changes={len(changed_cells)} add={len(added_change_cells)} "
                    f"remove={len(removed_change_cells)}"
                )
                calc_text.config(state="normal")
                calc_text.insert(
                    tk.END,
                    "\nChange breakdown:\n"
                    f"- Added obstacles: {len(added_change_cells)} "
                    f"(from previously explored: {added_from_expanded}, "
                    f"from unexplored: {len(added_change_cells) - added_from_expanded})\n"
                    f"- Removed obstacles: {len(removed_change_cells)} "
                    f"(from previously explored: {removed_from_expanded}, "
                    f"from unexplored: {len(removed_change_cells) - removed_from_expanded})\n",
                )
                calc_text.config(state="disabled")
                canvas.draw()
                state["phase"] = 3
                state["idx"] = 0
                window.after(600, step)
                return

            if phase == 3:
                if idx < len(replan_expanded):
                    node = replan_expanded[idx]
                    reason = "frontier refinement"
                    if node != start and node != goal:
                        if node in initial_expanded_set:
                            replan_old_count["value"] += 1
                            reason = "reused because it was already part of previous search tree"
                        else:
                            if right_grid[node[0]][node[1]] != 1:
                                right_grid[node[0]][node[1]] = 6
                            replan_new_count["value"] += 1
                            if node in influenced_zone:
                                reason = "newly explored because nearby edge costs changed"
                            else:
                                reason = "newly explored because old frontier was insufficient"

                    im_right.set_array(right_grid)
                    right_title.set_text(f"Replanning: Expanded {idx + 1}/{len(replan_expanded)}")
                    status.set(
                        f"Replan {idx + 1}/{len(replan_expanded)} | reused={replan_old_count['value']} "
                        f"new={replan_new_count['value']} | {reason}"
                    )
                    canvas.draw()
                    state["idx"] += 1
                    window.after(35, step)
                    return
                state["phase"] = 4
                state["idx"] = 0
                window.after(350, step)
                return

            if phase == 4:
                if idx < len(replan_path):
                    node = replan_path[idx]
                    if node != start and node != goal:
                        right_grid[node[0]][node[1]] = 7
                    im_right.set_array(right_grid)
                    right_title.set_text(f"Replanned Path {idx + 1}/{len(replan_path)}")
                    status.set("Drawing replanned path")
                    canvas.draw()
                    state["idx"] += 1
                    window.after(80, step)
                    return

                state["running"] = False
                if replanned_result["success"]:
                    final_status = (
                        f"Complete: initial expanded {initial_result['nodes_expanded']} vs "
                        f"replan expanded {replanned_result['nodes_expanded']} | "
                        f"reused={replan_old_count['value']} new={replan_new_count['value']}"
                    )
                else:
                    final_status = "Complete: replanning found no path after updates"
                right_title.set_text("After Changes: Replan Complete")
                status.set(final_status)
                canvas.draw()

                save_btn.config(
                    state="normal",
                    command=lambda: self.save_visualization(fig, initial_result, replanned_result, window),
                )

        window.after(400, step)

        def on_close() -> None:
            state["running"] = False
            plt.close(fig)
            window.destroy()

        window.protocol("WM_DELETE_WINDOW", on_close)

    def save_visualization(
        self,
        fig: Figure,
        initial_result: dict,
        replanned_result: dict,
        parent: tk.Toplevel,
    ) -> None:
        try:
            base = self.custom_filename.get().strip() or "lpa_visualization"
            if self.auto_timestamp.get():
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"{base}_{stamp}.png"
            else:
                filename = f"{base}.png"

            path = os.path.join(os.path.dirname(__file__), filename)
            fig.savefig(path, dpi=150, bbox_inches="tight")

            messagebox.showinfo(
                "Saved",
                f"Saved: {filename}\n\n"
                f"Initial cost: {initial_result['total_cost']:.3f}\n"
                f"Initial expanded: {initial_result['nodes_expanded']}\n"
                f"Replan cost: {replanned_result['total_cost']:.3f}\n"
                f"Replan expanded: {replanned_result['nodes_expanded']}",
                parent=parent,
            )
        except Exception as exc:
            messagebox.showerror("Error", f"Could not save figure: {exc}", parent=parent)


def main() -> None:
    root = tk.Tk()
    app = LPAGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()