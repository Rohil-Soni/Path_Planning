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
from typing import Any

import matplotlib

matplotlib.use("TkAgg")
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.colors import ListedColormap
from matplotlib.figure import Figure

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

        ttk.Button(left, text="Run LPA* Live Replan", command=self.run_simulation).grid(
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

    def choose_single_block_change(
        self,
        path: list,
        planner: LPAStarPathfinder,
        start: tuple,
        goal: tuple,
    ) -> tuple | None:
        if len(path) < 3:
            return None

        # Primary behavior: obstacle is the immediate next step from current start,
        # so replanning begins from the node directly before the obstacle.
        candidate_indices = [1]
        remaining = list(range(2, len(path) - 1))
        random.shuffle(remaining)
        candidate_indices.extend(remaining)

        for idx in candidate_indices:
            block_cell = path[idx]
            predecessor = path[idx - 1]
            if block_cell == start or block_cell == goal:
                continue
            if planner.obstacle_grid[block_cell[0]][block_cell[1]]:
                continue
            return block_cell, predecessor

        return None

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

            obstacle_set = set(obstacles)
            self.status_label.config(text="Starting live replanning...", foreground="blue")
            self.root.update()

            self.create_live_simulation(
                rows=rows,
                cols=cols,
                start=start,
                goal=goal,
                planner=planner,
                obstacle_set=obstacle_set,
                initial_result=initial_result,
            )

            self.status_label.config(text="Live simulation window opened.", foreground="green")

        except Exception as exc:
            messagebox.showerror("Error", f"Simulation failed: {exc}")
            self.status_label.config(text="Simulation failed.", foreground="red")

    def create_live_simulation(
        self,
        rows: int,
        cols: int,
        start: tuple,
        goal: tuple,
        planner: LPAStarPathfinder,
        obstacle_set: set,
        initial_result: dict,
    ) -> None:
        window = tk.Toplevel(self.root)
        window.title("LPA* Live Replanning")
        window.geometry("1080x800")

        fig, ax = plt.subplots(1, 1, figsize=(10.2, 7.0))

        # 0 empty, 1 obstacle, 2 expanded, 3 path, 4 start, 5 goal, 6 animated frontier
        colors = ["white", "#2f2f2f", "#9fd2f5", "#1f9d4c", "#d93636", "#ffd24d", "#b553d6"]
        cmap = ListedColormap(colors)

        grid = np.zeros((rows, cols), dtype=np.int32)
        im = ax.imshow(grid, cmap=cmap, interpolation="nearest", origin="upper", vmin=0, vmax=6)

        ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
        ax.grid(which="minor", color="gray", linestyle="-", linewidth=0.45)
        ax.tick_params(which="minor", size=0)
        ax.set_xticks(np.arange(0, cols, 1))
        ax.set_yticks(np.arange(0, rows, 1))
        ax.set_xlabel("Column")
        ax.set_ylabel("Row")

        ax.set_title("Live Replanning", fontweight="bold")

        legend_items = [
            patches.Patch(facecolor="#d93636", edgecolor="black", label="Current Start"),
            patches.Patch(facecolor="#ffd24d", edgecolor="black", label="Goal"),
            patches.Patch(facecolor="#2f2f2f", edgecolor="black", label="Obstacle"),
            patches.Patch(facecolor="#9fd2f5", edgecolor="black", label="Expanded"),
            patches.Patch(facecolor="#1f9d4c", edgecolor="black", label="Current Path"),
        ]
        fig.legend(handles=legend_items, loc="lower center", bbox_to_anchor=(0.5, 0.01), ncol=3, fontsize=9)
        plt.tight_layout(rect=(0, 0.06, 1, 1))

        canvas = FigureCanvasTkAgg(fig, master=window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        control = ttk.Frame(window)
        control.pack(fill=tk.X, padx=8, pady=4)

        status = tk.StringVar(value="Live replanning started")
        ttk.Label(control, textvariable=status, font=("Arial", 10)).pack(side=tk.LEFT, padx=4)

        save_btn = ttk.Button(control, text="Save Final Figure", state="disabled")
        save_btn.pack(side=tk.RIGHT, padx=4)

        state = {
            "running": True,
            "steps": 0,
            "current_start": start,
            "current_result": initial_result,
            "display_result": initial_result,
            "last_change": None,
            "pending_start": None,
            "pending_result": None,
            "phase": "idle",
            "phase_idx": 0,
            "phase_expanded": [],
            "phase_path": [],
        }

        start_text: dict[str, Any | None] = {"artist": None}
        goal_text: dict[str, Any | None] = {"artist": None}
        change_marker: dict[str, Any | None] = {"artist": None}

        def render_frame() -> None:
            grid[:, :] = 0

            for r, c in obstacle_set:
                grid[r][c] = 1

            for node in state["display_result"].get("expanded_nodes", []):
                if node != state["current_start"] and node != goal and grid[node[0]][node[1]] == 0:
                    grid[node[0]][node[1]] = 2

            for node in state["display_result"].get("path", []):
                if node != state["current_start"] and node != goal and grid[node[0]][node[1]] != 1:
                    grid[node[0]][node[1]] = 3

            # Animated search frontier in current cycle.
            if state["phase"] in ("expand", "path") and state["pending_result"] is not None:
                for node in state["phase_expanded"][: state["phase_idx"]]:
                    if node != state["current_start"] and node != goal and grid[node[0]][node[1]] == 0:
                        grid[node[0]][node[1]] = 6

                if state["phase"] == "path":
                    for node in state["phase_path"][: state["phase_idx"]]:
                        if node != state["current_start"] and node != goal and grid[node[0]][node[1]] != 1:
                            grid[node[0]][node[1]] = 3

            grid[state["current_start"][0]][state["current_start"][1]] = 4
            grid[goal[0]][goal[1]] = 5

            im.set_array(grid)

            if start_text["artist"] is not None:
                start_text["artist"].remove()
            if goal_text["artist"] is not None:
                goal_text["artist"].remove()
            if change_marker["artist"] is not None:
                change_marker["artist"].remove()
                change_marker["artist"] = None

            start_text["artist"] = ax.text(
                state["current_start"][1],
                state["current_start"][0],
                "S",
                ha="center",
                va="center",
                color="white",
                fontweight="bold",
            )
            goal_text["artist"] = ax.text(
                goal[1],
                goal[0],
                "G",
                ha="center",
                va="center",
                color="black",
                fontweight="bold",
            )

            if state["last_change"] is not None:
                cr, cc = state["last_change"]
                change_marker["artist"] = ax.scatter(
                    [cc], [cr], marker="X", s=80, c="#ff8c1a", linewidths=1.4, zorder=5
                )

            ax.set_title(
                f"Live Replanning | step={state['steps']} | "
                f"expanded={state['display_result'].get('nodes_expanded', 0)} | "
                f"cost={state['display_result'].get('total_cost', float('inf')):.2f}",
                fontweight="bold",
            )
            canvas.draw()

        def stop_with_status(text: str, ok: bool) -> None:
            state["running"] = False
            status.set(text)
            self.status_label.config(text=text, foreground="green" if ok else "red")
            save_btn.config(
                state="normal",
                command=lambda: self.save_visualization(fig, initial_result, state["current_result"], window),
            )

        def tick() -> None:
            if not state["running"]:
                return

            if not state["current_result"].get("success", False):
                render_frame()
                stop_with_status("No path available. Live simulation stopped.", ok=False)
                return

            selection = self.choose_single_block_change(
                state["current_result"].get("path", []),
                planner,
                state["current_start"],
                goal,
            )

            if selection is None:
                render_frame()
                stop_with_status("No valid next obstacle on path. Live simulation complete.", ok=True)
                return

            block_cell, replan_start = selection

            # Replan only after a real obstacle addition.
            if block_cell in obstacle_set:
                render_frame()
                status.set("Skipped replan: selected cell already blocked")
                window.after(500, tick)
                return

            planner.obstacle_grid[block_cell[0]][block_cell[1]] = True
            obstacle_set.add(block_cell)
            planner.update_graph([block_cell])

            replanned_result = planner.plan(replan_start, goal)

            # Prepare visible step-by-step animation for this replan cycle.
            state["pending_start"] = replan_start
            state["pending_result"] = replanned_result
            state["last_change"] = block_cell
            state["phase_expanded"] = replanned_result.get("expanded_nodes", [])
            state["phase_path"] = replanned_result.get("path", [])
            state["phase"] = "expand"
            state["phase_idx"] = 0

            status.set(
                f"step {state['steps'] + 1}: obstacle at {block_cell}, animating replan from {replan_start}"
            )
            self.status_label.config(
                text=f"Live: obstacle {block_cell} added, replanning from {replan_start}",
                foreground="blue",
            )
            render_frame()
            window.after(40, animate_cycle)

        def animate_cycle() -> None:
            if not state["running"]:
                return

            phase = state["phase"]
            if phase == "expand":
                if state["phase_idx"] < len(state["phase_expanded"]):
                    state["phase_idx"] += 1
                    render_frame()
                    window.after(22, animate_cycle)
                    return

                state["phase"] = "path"
                state["phase_idx"] = 0
                render_frame()
                window.after(80, animate_cycle)
                return

            if phase == "path":
                if state["phase_idx"] < len(state["phase_path"]):
                    state["phase_idx"] += 1
                    render_frame()
                    window.after(45, animate_cycle)
                    return

                # Commit completed cycle.
                state["current_start"] = state["pending_start"]
                state["current_result"] = state["pending_result"]
                state["display_result"] = state["pending_result"]
                state["steps"] += 1
                state["phase"] = "idle"
                state["phase_idx"] = 0
                render_frame()

                status.set(
                    f"step {state['steps']}: replanned from predecessor {state['current_start']}"
                )
                self.status_label.config(
                    text=f"Live step {state['steps']} complete",
                    foreground="blue",
                )

                if not state["current_result"].get("success", False):
                    stop_with_status("Path blocked after latest obstacle. Live simulation stopped.", ok=False)
                    return

                window.after(420, tick)
                return

        render_frame()
        window.after(700, tick)

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