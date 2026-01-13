#!/usr/bin/env python3
"""
GUI for Dijkstra's Algorithm Visualization
Interactive interface for setting parameters and running simulations
"""
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.colors import ListedColormap
from matplotlib.animation import FuncAnimation
import numpy as np
import random
import time
from datetime import datetime
import os

from Algo_Dij import DijkstraPathfinder


class DijkstraGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Dijkstra's Algorithm Visualizer")
        self.root.geometry("1200x800")
        
        # Default values
        self.rows = tk.IntVar(value=10)
        self.cols = tk.IntVar(value=10)
        self.start_row = tk.IntVar(value=0)
        self.start_col = tk.IntVar(value=0)
        self.goal_row = tk.IntVar(value=9)
        self.goal_col = tk.IntVar(value=9)
        self.min_obstacles = tk.IntVar(value=20)
        self.auto_timestamp = tk.BooleanVar(value=True)
        self.custom_filename = tk.StringVar(value="dijkstra_animation")
        
        # Grid for clicking start/goal
        self.grid_canvas = None
        self.setting_mode = tk.StringVar(value="none")  # "start", "goal", "none"
        
        self.create_widgets()
        
    def create_widgets(self):
        """Create all GUI widgets"""
        
        # Main container with two columns
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)
        
        # Left panel - Controls
        left_panel = ttk.Frame(main_frame, padding="5")
        left_panel.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Right panel - Grid preview
        right_panel = ttk.Frame(main_frame, padding="5")
        right_panel.grid(row=0, column=1, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # === LEFT PANEL CONTENTS ===
        
        # Title
        title_label = ttk.Label(left_panel, text="Dijkstra's Algorithm Settings", 
                               font=('Arial', 14, 'bold'))
        title_label.grid(row=0, column=0, columnspan=2, pady=(0, 15))
        
        row_idx = 1
        
        # Grid Dimensions Section
        ttk.Label(left_panel, text="Grid Dimensions", 
                 font=('Arial', 10, 'bold')).grid(row=row_idx, column=0, columnspan=2, sticky=tk.W, pady=(5, 5))
        row_idx += 1
        
        ttk.Label(left_panel, text="Rows (5-50):").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        rows_spinbox = ttk.Spinbox(left_panel, from_=5, to=50, textvariable=self.rows, width=15)
        rows_spinbox.grid(row=row_idx, column=1, sticky=(tk.W, tk.E), pady=2)
        row_idx += 1
        
        ttk.Label(left_panel, text="Columns (5-50):").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        cols_spinbox = ttk.Spinbox(left_panel, from_=5, to=50, textvariable=self.cols, width=15)
        cols_spinbox.grid(row=row_idx, column=1, sticky=(tk.W, tk.E), pady=2)
        row_idx += 1
        
        # Update grid preview button
        update_btn = ttk.Button(left_panel, text="Update Grid Preview", 
                               command=self.update_grid_preview)
        update_btn.grid(row=row_idx, column=0, columnspan=2, pady=10)
        row_idx += 1
        
        # Start Position Section
        ttk.Separator(left_panel, orient='horizontal').grid(row=row_idx, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        row_idx += 1
        
        ttk.Label(left_panel, text="Start Position", 
                 font=('Arial', 10, 'bold')).grid(row=row_idx, column=0, columnspan=2, sticky=tk.W, pady=(5, 5))
        row_idx += 1
        
        ttk.Label(left_panel, text="Start Row:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        start_row_spinbox = ttk.Spinbox(left_panel, from_=0, to=49, textvariable=self.start_row, width=15)
        start_row_spinbox.grid(row=row_idx, column=1, sticky=(tk.W, tk.E), pady=2)
        row_idx += 1
        
        ttk.Label(left_panel, text="Start Column:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        start_col_spinbox = ttk.Spinbox(left_panel, from_=0, to=49, textvariable=self.start_col, width=15)
        start_col_spinbox.grid(row=row_idx, column=1, sticky=(tk.W, tk.E), pady=2)
        row_idx += 1
        
        set_start_btn = ttk.Button(left_panel, text="Click Grid to Set Start", 
                                   command=lambda: self.set_click_mode("start"))
        set_start_btn.grid(row=row_idx, column=0, columnspan=2, pady=5)
        row_idx += 1
        
        # Goal Position Section
        ttk.Separator(left_panel, orient='horizontal').grid(row=row_idx, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        row_idx += 1
        
        ttk.Label(left_panel, text="Goal Position", 
                 font=('Arial', 10, 'bold')).grid(row=row_idx, column=0, columnspan=2, sticky=tk.W, pady=(5, 5))
        row_idx += 1
        
        ttk.Label(left_panel, text="Goal Row:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        goal_row_spinbox = ttk.Spinbox(left_panel, from_=0, to=49, textvariable=self.goal_row, width=15)
        goal_row_spinbox.grid(row=row_idx, column=1, sticky=(tk.W, tk.E), pady=2)
        row_idx += 1
        
        ttk.Label(left_panel, text="Goal Column:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        goal_col_spinbox = ttk.Spinbox(left_panel, from_=0, to=49, textvariable=self.goal_col, width=15)
        goal_col_spinbox.grid(row=row_idx, column=1, sticky=(tk.W, tk.E), pady=2)
        row_idx += 1
        
        set_goal_btn = ttk.Button(left_panel, text="Click Grid to Set Goal", 
                                 command=lambda: self.set_click_mode("goal"))
        set_goal_btn.grid(row=row_idx, column=0, columnspan=2, pady=5)
        row_idx += 1
        
        # Obstacles Section
        ttk.Separator(left_panel, orient='horizontal').grid(row=row_idx, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        row_idx += 1
        
        ttk.Label(left_panel, text="Obstacles", 
                 font=('Arial', 10, 'bold')).grid(row=row_idx, column=0, columnspan=2, sticky=tk.W, pady=(5, 5))
        row_idx += 1
        
        ttk.Label(left_panel, text="Min Obstacles:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        obs_spinbox = ttk.Spinbox(left_panel, from_=0, to=200, textvariable=self.min_obstacles, width=15)
        obs_spinbox.grid(row=row_idx, column=1, sticky=(tk.W, tk.E), pady=2)
        row_idx += 1
        
        # Output Settings Section
        ttk.Separator(left_panel, orient='horizontal').grid(row=row_idx, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        row_idx += 1
        
        ttk.Label(left_panel, text="Output Settings", 
                 font=('Arial', 10, 'bold')).grid(row=row_idx, column=0, columnspan=2, sticky=tk.W, pady=(5, 5))
        row_idx += 1
        
        timestamp_check = ttk.Checkbutton(left_panel, text="Auto-add timestamp to filename", 
                                         variable=self.auto_timestamp)
        timestamp_check.grid(row=row_idx, column=0, columnspan=2, sticky=tk.W, pady=5)
        row_idx += 1
        
        ttk.Label(left_panel, text="Filename:").grid(row=row_idx, column=0, sticky=tk.W, pady=2)
        filename_entry = ttk.Entry(left_panel, textvariable=self.custom_filename, width=20)
        filename_entry.grid(row=row_idx, column=1, sticky=(tk.W, tk.E), pady=2)
        row_idx += 1
        
        # Run Button
        ttk.Separator(left_panel, orient='horizontal').grid(row=row_idx, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=15)
        row_idx += 1
        
        run_btn = ttk.Button(left_panel, text="🚀 Run Simulation", 
                            command=self.run_simulation, 
                            style='Accent.TButton')
        run_btn.grid(row=row_idx, column=0, columnspan=2, pady=10, ipadx=20, ipady=10)
        row_idx += 1
        
        # Status label
        self.status_label = ttk.Label(left_panel, text="Ready", 
                                     foreground="green", font=('Arial', 9))
        self.status_label.grid(row=row_idx, column=0, columnspan=2, pady=5)
        
        # === RIGHT PANEL CONTENTS ===
        
        ttk.Label(right_panel, text="Grid Preview", 
                 font=('Arial', 12, 'bold')).pack(pady=(0, 10))
        
        info_label = ttk.Label(right_panel, 
                              text="Red = Start | Yellow = Goal\nClick 'Set Start/Goal' buttons, then click on grid",
                              font=('Arial', 9), foreground="blue")
        info_label.pack(pady=5)
        
        # Canvas for grid preview
        self.canvas_frame = ttk.Frame(right_panel)
        self.canvas_frame.pack(fill=tk.BOTH, expand=True)
        
        self.update_grid_preview()
        
    def update_grid_preview(self):
        """Update the grid preview canvas"""
        rows = self.rows.get()
        cols = self.cols.get()
        
        # Validate and update start/goal positions
        if self.start_row.get() >= rows:
            self.start_row.set(rows - 1)
        if self.start_col.get() >= cols:
            self.start_col.set(cols - 1)
        if self.goal_row.get() >= rows:
            self.goal_row.set(rows - 1)
        if self.goal_col.get() >= cols:
            self.goal_col.set(cols - 1)
        
        # Clear previous canvas
        for widget in self.canvas_frame.winfo_children():
            widget.destroy()
        
        # Create new canvas
        canvas_size = min(500, 400)
        cell_size = canvas_size // max(rows, cols)
        
        self.grid_canvas = tk.Canvas(self.canvas_frame, 
                                     width=cols * cell_size, 
                                     height=rows * cell_size,
                                     bg='white', highlightthickness=1, 
                                     highlightbackground='black')
        self.grid_canvas.pack()
        
        # Draw grid
        for i in range(rows + 1):
            self.grid_canvas.create_line(0, i * cell_size, cols * cell_size, i * cell_size, fill='gray')
        for j in range(cols + 1):
            self.grid_canvas.create_line(j * cell_size, 0, j * cell_size, rows * cell_size, fill='gray')
        
        # Draw start and goal
        self.draw_positions(cell_size)
        
        # Bind click event
        self.grid_canvas.bind('<Button-1>', lambda e: self.on_grid_click(e, cell_size))
        
    def draw_positions(self, cell_size):
        """Draw start and goal positions on canvas"""
        rows = self.rows.get()
        cols = self.cols.get()
        start = (self.start_row.get(), self.start_col.get())
        goal = (self.goal_row.get(), self.goal_col.get())
        
        # Clear previous markers
        self.grid_canvas.delete('marker')
        
        # Draw start (red)
        x1 = start[1] * cell_size
        y1 = start[0] * cell_size
        self.grid_canvas.create_rectangle(x1, y1, x1 + cell_size, y1 + cell_size, 
                                         fill='red', tags='marker')
        self.grid_canvas.create_text(x1 + cell_size//2, y1 + cell_size//2, 
                                     text='S', font=('Arial', 12, 'bold'), 
                                     fill='white', tags='marker')
        
        # Draw goal (yellow)
        x2 = goal[1] * cell_size
        y2 = goal[0] * cell_size
        self.grid_canvas.create_rectangle(x2, y2, x2 + cell_size, y2 + cell_size, 
                                         fill='gold', tags='marker')
        self.grid_canvas.create_text(x2 + cell_size//2, y2 + cell_size//2, 
                                     text='G', font=('Arial', 12, 'bold'), 
                                     fill='black', tags='marker')
    
    def set_click_mode(self, mode):
        """Set the clicking mode for start/goal"""
        self.setting_mode.set(mode)
        if mode == "start":
            self.status_label.config(text="Click on grid to set START position", foreground="blue")
        elif mode == "goal":
            self.status_label.config(text="Click on grid to set GOAL position", foreground="blue")
    
    def on_grid_click(self, event, cell_size):
        """Handle grid click to set start/goal"""
        col = event.x // cell_size
        row = event.y // cell_size
        
        mode = self.setting_mode.get()
        if mode == "start":
            self.start_row.set(row)
            self.start_col.set(col)
            self.status_label.config(text=f"Start set to ({row}, {col})", foreground="green")
            self.setting_mode.set("none")
        elif mode == "goal":
            self.goal_row.set(row)
            self.goal_col.set(col)
            self.status_label.config(text=f"Goal set to ({row}, {col})", foreground="green")
            self.setting_mode.set("none")
        
        self.draw_positions(cell_size)
    
    def generate_random_obstacles(self, rows, cols, start, goal, min_obstacles):
        """Generate random obstacles ensuring path exists"""
        max_attempts = 100
        
        for attempt in range(max_attempts):
            num_obstacles = random.randint(min_obstacles, int(rows * cols * 0.35))
            obstacles = []
            
            while len(obstacles) < num_obstacles:
                r = random.randint(0, rows - 1)
                c = random.randint(0, cols - 1)
                
                if (r, c) != start and (r, c) != goal and (r, c) not in obstacles:
                    obstacles.append((r, c))
            
            pathfinder = DijkstraPathfinder((rows, cols), obstacles)
            result = pathfinder.dijkstra(start, goal)
            
            if result['success']:
                return obstacles, result
        
        # Fallback
        obstacles = []
        while len(obstacles) < min_obstacles:
            r = random.randint(0, rows - 1)
            c = random.randint(0, cols - 1)
            if (r, c) != start and (r, c) != goal and (r, c) not in obstacles:
                obstacles.append((r, c))
        
        pathfinder = DijkstraPathfinder((rows, cols), obstacles)
        result = pathfinder.dijkstra(start, goal)
        return obstacles, result
    
    def run_simulation(self):
        """Run the pathfinding simulation"""
        # Validate inputs
        rows = self.rows.get()
        cols = self.cols.get()
        start = (self.start_row.get(), self.start_col.get())
        goal = (self.goal_row.get(), self.goal_col.get())
        min_obs = self.min_obstacles.get()
        
        if start == goal:
            messagebox.showerror("Error", "Start and Goal positions must be different!")
            return
        
        self.status_label.config(text="Generating obstacles...", foreground="orange")
        self.root.update()
        
        try:
            # Generate obstacles
            obstacles, result = self.generate_random_obstacles(rows, cols, start, goal, min_obs)
            
            if not result['success']:
                messagebox.showerror("Error", "No path found!")
                self.status_label.config(text="Failed: No path found", foreground="red")
                return
            
            self.status_label.config(text="Opening visualization...", foreground="blue")
            self.root.update()
            
            # Create animation
            self.create_animation(rows, cols, start, goal, obstacles, result)
            
        except Exception as e:
            messagebox.showerror("Error", f"Simulation failed: {str(e)}")
            self.status_label.config(text="Failed!", foreground="red")
    
    def create_animation(self, rows, cols, start, goal, obstacles, result):
        """Create and display live animation in GUI window"""
        explored_nodes = result['explored_nodes']
        path_nodes = result['path']
        
        # Create new window for visualization
        viz_window = tk.Toplevel(self.root)
        viz_window.title("Dijkstra's Algorithm - Live Simulation")
        viz_window.geometry("900x700")
        
        # Create figure for matplotlib
        fig, ax = plt.subplots(figsize=(8, 6))
        
        colors = ['white', 'black', '#87CEEB', '#00FF00', '#FF0000', '#FFD700']
        cmap = ListedColormap(colors)
        
        grid = np.zeros((rows, cols))
        
        for obs in obstacles:
            grid[obs[0]][obs[1]] = 1
        
        grid[start[0]][start[1]] = 4
        grid[goal[0]][goal[1]] = 5
        
        im = ax.imshow(grid, cmap=cmap, interpolation='nearest', origin='upper')
        
        ax.set_xticks(np.arange(-0.5, cols, 1), minor=True)
        ax.set_yticks(np.arange(-0.5, rows, 1), minor=True)
        ax.grid(which='minor', color='gray', linestyle='-', linewidth=0.5)
        ax.tick_params(which='minor', size=0)
        
        ax.set_xticks(np.arange(0, cols, 1))
        ax.set_yticks(np.arange(0, rows, 1))
        
        ax.set_xlabel('Column', fontsize=10)
        ax.set_ylabel('Row', fontsize=10)
        
        legend_elements = [
            patches.Patch(facecolor='#FF0000', edgecolor='black', label='Start'),
            patches.Patch(facecolor='#FFD700', edgecolor='black', label='Goal'),
            patches.Patch(facecolor='black', edgecolor='black', label='Obstacles'),
            patches.Patch(facecolor='#87CEEB', edgecolor='black', label='Explored'),
            patches.Patch(facecolor='#00FF00', edgecolor='black', label='Path')
        ]
        ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1.02, 1), 
                  fontsize=9, frameon=True, shadow=True)
        
        ax.text(start[1], start[0], 'S', ha='center', va='center', 
                fontsize=10, fontweight='bold', color='white')
        ax.text(goal[1], goal[0], 'G', ha='center', va='center', 
                fontsize=10, fontweight='bold', color='black')
        
        title_text = ax.text(0.5, 1.08, 'Initializing...', transform=ax.transAxes,
                            ha='center', fontsize=11, fontweight='bold')
        
        plt.tight_layout()
        
        # Embed matplotlib in tkinter
        canvas = FigureCanvasTkAgg(fig, master=viz_window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Control frame
        control_frame = ttk.Frame(viz_window)
        control_frame.pack(fill=tk.X, padx=10, pady=5)
        
        status_var = tk.StringVar(value="Starting simulation...")
        status_label = ttk.Label(control_frame, textvariable=status_var, font=('Arial', 10))
        status_label.pack(side=tk.LEFT, padx=5)
        
        save_btn = ttk.Button(control_frame, text="Save Animation", state='disabled')
        save_btn.pack(side=tk.RIGHT, padx=5)
        
        # Animation state
        current_frame = [0]
        animation_phase = [0]  # 0: exploring, 1: showing path
        is_running = [True]
        
        def animate_step():
            if not is_running[0]:
                return
            
            phase = animation_phase[0]
            
            if phase == 0:  # Exploration phase
                if current_frame[0] < len(explored_nodes):
                    node = explored_nodes[current_frame[0]]
                    if node != start and node != goal:
                        grid[node[0]][node[1]] = 2
                    
                    im.set_array(grid)
                    title_text.set_text(
                        f"Exploring Nodes - Step {current_frame[0] + 1}/{len(explored_nodes)}"
                    )
                    status_var.set(
                        f"Exploring... {current_frame[0] + 1}/{len(explored_nodes)} nodes | "
                        f"Obstacles: {len(obstacles)}"
                    )
                    canvas.draw()
                    current_frame[0] += 1
                    viz_window.after(50, animate_step)  # 50ms delay
                else:
                    # Move to path drawing phase
                    animation_phase[0] = 1
                    current_frame[0] = 0
                    viz_window.after(500, animate_step)  # Pause before showing path
            
            elif phase == 1:  # Path drawing phase
                if current_frame[0] < len(path_nodes):
                    node = path_nodes[current_frame[0]]
                    if node != start and node != goal:
                        grid[node[0]][node[1]] = 3
                    
                    im.set_array(grid)
                    title_text.set_text(
                        f"Drawing Final Path - Step {current_frame[0] + 1}/{len(path_nodes)}"
                    )
                    status_var.set(
                        f"Drawing path... {current_frame[0] + 1}/{len(path_nodes)} nodes | "
                        f"Total cost: {len(path_nodes) - 1}"
                    )
                    canvas.draw()
                    current_frame[0] += 1
                    viz_window.after(100, animate_step)  # Slower for path
                else:
                    # Animation complete
                    is_running[0] = False
                    title_text.set_text("Simulation Complete!")
                    status_var.set(
                        f"✓ COMPLETE | Explored: {result['nodes_explored']} | "
                        f"Path: {result['path_length']} | Time: {result['execution_time_ms']:.2f}ms"
                    )
                    canvas.draw()
                    save_btn.config(state='normal', command=lambda: self.save_animation_files(
                        fig, result, obstacles, viz_window
                    ))
                    self.root.after(0, lambda: self.status_label.config(
                        text="✓ Simulation complete! Check visualization window.", 
                        foreground="green"
                    ))
        
        # Start animation
        viz_window.after(500, animate_step)
        
        # Handle window close
        def on_close():
            is_running[0] = False
            plt.close(fig)
            viz_window.destroy()
        
        viz_window.protocol("WM_DELETE_WINDOW", on_close)
    
    def save_animation_files(self, fig, result, obstacles, viz_window):
        """Save the animation to GIF and PNG files"""
        try:
            # Generate filename
            base_filename = self.custom_filename.get()
            if self.auto_timestamp.get():
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                final_file = f"{base_filename}_{timestamp}_final.png"
            else:
                final_file = f"{base_filename}_final.png"
            
            # Save in the Graphs_Algo directory
            final_path = os.path.join(os.path.dirname(__file__), final_file)
            
            fig.savefig(final_path, dpi=150, bbox_inches='tight')
            
            messagebox.showinfo(
                "Saved", 
                f"Visualization saved!\n\n"
                f"PNG: {final_file}\n\n"
                f"Path Length: {result['path_length']}\n"
                f"Nodes Explored: {result['nodes_explored']}\n"
                f"Time: {result['execution_time_ms']:.2f}ms",
                parent=viz_window
            )
        except Exception as e:
            messagebox.showerror("Error", f"Failed to save: {str(e)}", parent=viz_window)


def main():
    root = tk.Tk()
    app = DijkstraGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
